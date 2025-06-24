#!/usr/bin/env python3

#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2025 Hans Baier <foss@hans-baier.de>
# SPDX-License-Identifier: BSD-2-Clause
# Since this board has no uart, you want to build with JTAG UART:
# python litex_boards/targets/hyvision_pcie_opt01_revf.py --build --uart-name=jtag_uart
#
# JTAG Connectors: j11 or j13
# Pinout:
#
# | Pins |  1  |  2  |  3  |  4  |  5  |  6  |
# |------|-----|-----|-----|-----|-----|-----|
# | Pins | TMS | TDI | TDO | TCK | GND | VCC |

from migen import *

from litex.gen import *

from litex_boards.platforms import hyvision_pcie_opt01_revf

from litex.build.generic_platform import *

from litex.soc.interconnect.csr     import *
from litex.soc.interconnect         import stream
from litex.soc.interconnect         import wishbone

from litex.soc.integration.soc      import SoCRegion
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder  import *

from litex.soc.cores.clock import *

from litepcie.phy.s7pciephy import S7PCIEPHY
from litepcie.software import generate_litepcie_software_headers

from gateware.uart              import UARTShared
from gateware.soc               import LiteXWRNICSoC
from gateware.time              import TimeGenerator
from gateware.qpll              import SharedQPLL
from gateware.wb_clock_crossing import WishboneClockCrossing
from gateware.wrf_stream2wb     import Stream2Wishbone
from gateware.wrf_wb2stream     import Wishbone2Stream
from gateware.ad5683r.core      import AD5683RDAC
from gateware.ad9516.core       import AD9516PLL, AD9516_MAIN_CONFIG, AD9516_EXT_CONFIG
from gateware.measurement       import MultiClkMeasurement
from gateware.delay.core        import MacroDelay, CoarseDelay, FineDelay
from gateware.pps               import PPSGenerator
from gateware.clk10m            import Clk10MGenerator
from gateware.nic.phy           import LiteEthPHYWRGMII

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.rst              = Signal()
        self.cd_sys           = ClockDomain()
        self.cd_clk200        = ClockDomain()
        self.cd_refclk_pcie   = ClockDomain()
        self.cd_refclk_eth    = ClockDomain()
        self.cd_clk_125m_gtp  = ClockDomain()
        self.cd_clk_62m5_dmtd = ClockDomain()
        self.cd_clk10m_in     = ClockDomain()
        self.cd_clk62m5_in    = ClockDomain()

        # # #

        # Sys PLL (Free-Running from clk200).
        # -----------------------------------
        clk200 = platform.request("clk200")

        self.pll = pll = S7PLL(speedgrade=-1)
        self.comb += pll.reset.eq(self.rst)
        pll.register_clkin(clk200, 200e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq, margin=0)
        self.comb += self.cd_clk200.clk.eq(pll.clkin)

        # RefClk MMCM (125MHz).
        # ---------------------
        self.refclk_mmcm = S7MMCM(speedgrade=-3)
        self.comb += self.refclk_mmcm.reset.eq(self.rst)
        self.refclk_mmcm.register_clkin(ClockSignal("clk200"), 200e6)
        self.refclk_mmcm.create_clkout(self.cd_clk_125m_gtp,  125e6, margin=0)
        self.refclk_mmcm.expose_dps("clk200", with_csr=False)
        self.refclk_mmcm.params.update(p_CLKOUT0_USE_FINE_PS="TRUE")
        self.comb += self.cd_refclk_eth.clk.eq(self.cd_clk_125m_gtp.clk)

        # DMTD MMCM (62.5MHz).
        # --------------------
        self.dmtd_mmcm = S7MMCM(speedgrade=-3)
        self.comb += self.dmtd_mmcm.reset.eq(self.rst)
        self.dmtd_mmcm.register_clkin(ClockSignal("clk200"), 200e6)
        self.dmtd_mmcm.create_clkout(self.cd_clk_62m5_dmtd, 62.5e6, margin=0)
        self.dmtd_mmcm.expose_dps("clk200", with_csr=False)
        self.dmtd_mmcm.params.update(p_CLKOUT0_USE_FINE_PS="TRUE")

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(LiteXWRNICSoC):
    def __init__(self, sys_clk_freq=125e6,
        # PCIe Parameters.
        # ----------------
        with_pcie = True,

        # White Rabbit Parameters.
        # ------------------------
        with_white_rabbit          = True,
        white_rabbit_sfp_connector = 0,
        white_rabbit_cpu_firmware  = "firmware/spec_a7_wrc.bram",
    ):
        # Platform ---------------------------------------------------------------------------------

        platform = hyvision_pcie_opt01_revf.Platform()
        platform.add_extension([
            # White Rabbit
            ("pps_out",    0, Pins("U20"), IOStandard("LVCMOS33")), # DVP.LVAL
            ("wr_clk_out", 0, Pins("T18"), IOStandard("LVCMOS33")), # DVP.FVAL

            # Serial
            ("serial", 0,
                Subsignal("tx", Pins("T19")), # DVP.DTX
                Subsignal("rx", Pins("U19")), # DVP.DRX
                IOStandard("LVCMOS33")
            ),
        ])

        # Clocking ---------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq)

        # SoCMini ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform,
            clk_freq      = sys_clk_freq,
            ident         = "LiteX-WR-NIC on HVS HyVision PCIe OPT01 revf",
            ident_version = True
        )

        # JTAGBone ---------------------------------------------------------------------------------

        self.add_jtagbone()
        platform.add_period_constraint(self.jtagbone_phy.cd_jtag.clk, 1e9/20e6)
        platform.add_false_path_constraints(self.jtagbone_phy.cd_jtag.clk, self.crg.cd_sys.clk)

        # PCIe PHY ---------------------------------------------------------------------------------
        if with_pcie:
            self.pcie_phy = S7PCIEPHY(platform, platform.request("pcie_x4"),
                data_width  = 128,
                bar0_size   = 0x20000,
                with_ptm    = True,
                refclk_freq = 100e6,
            )
            self.pcie_phy.update_config({
                "Base_Class_Menu"          : "Network_controller",
                "Sub_Class_Interface_Menu" : "Ethernet_controller",
                "Class_Code_Base"          : "02",
                "Class_Code_Sub"           : "00",
            })
            self.comb += ClockSignal("refclk_pcie").eq(self.pcie_phy.pcie_refclk)
            #self.pcie_phy.use_external_qpll(qpll_channel=self.qpll.get_channel("pcie"))
            platform.add_period_constraint(self.crg.cd_sys.clk, 1e9/sys_clk_freq)

            # PCIe <-> Sys-Clk false paths.
            false_paths = [
                ("{{*s7pciephy_clkout0}}", "sys_clk"),
                ("{{*s7pciephy_clkout1}}", "sys_clk"),
                ("{{*s7pciephy_clkout3}}", "sys_clk"),
                ("{{*s7pciephy_clkout0}}", "{{*s7pciephy_clkout1}}")
            ]
            for clk0, clk1 in false_paths:
                platform.toolchain.pre_placement_commands.append(f"set_false_path -from [get_clocks {clk0}] -to [get_clocks {clk1}]")
                platform.toolchain.pre_placement_commands.append(f"set_false_path -from [get_clocks {clk1}] -to [get_clocks {clk0}]")

        # White Rabbit -----------------------------------------------------------------------------

        if with_white_rabbit:
            # Clks.
            # -----
            self.cd_wr = ClockDomain("wr")

            # Pads.
            # -----
            sfp_pads            = platform.request("sfp",            white_rabbit_sfp_connector)
            sfp_det_pads        = platform.request("sfp_rs1",        white_rabbit_sfp_connector)
            sfp_lost_pads       = platform.request("sfp_los",        white_rabbit_sfp_connector)
            sfp_tx_disable_pads = platform.request("sfp_tx_disable", white_rabbit_sfp_connector)
            sfp_tx_fault_pads   = platform.request("sfp_tx_fault",   white_rabbit_sfp_connector)
            serial_pads         = platform.request("serial")

            # Signals.
            # --------
            self.pps             = pps             = Signal()
            self.led_pps         = led_pps         = Signal()
            self.led_link        = led_link        = Signal()
            self.led_act         = led_act         = Signal()
            self.dac_refclk_load = dac_refclk_load = Signal()
            self.dac_refclk_data = dac_refclk_data = Signal(16)
            self.dac_dmtd_load   = dac_dmtd_load   = Signal()
            self.dac_dmtd_data   = dac_dmtd_data   = Signal(16)
            self.pps_out_pulse   = pps_out_pulse   = Signal()
            self.tm_seconds      = tm_seconds      = Signal(40)

            # White Rabbit Fabric Interface.
            # ------------------------------
            wrf_src = wishbone.Interface(data_width=16, address_width=2, adressing="byte")
            wrf_snk = wishbone.Interface(data_width=16, address_width=2, adressing="byte")

            self.wrf_stream2wb = wrf_stream2wb = Stream2Wishbone(  cd_to="wr")
            self.wrf_wb2stream = wrf_wb2stream = Wishbone2Stream(cd_from="wr")

            # White Rabbit Slave Interface.
            # -----------------------------
            self.wb_slave_sys = wb_slave_sys = wishbone.Interface(data_width=32, address_width=32, adressing="byte")
            self.wb_slave_wr  = wb_slave_wr  = wishbone.Interface(data_width=32, address_width=32, adressing="byte")
            self.bus.add_slave(name="wr_wb_slave", slave=wb_slave_sys, region=SoCRegion(
                 origin = 0x2000_0000,
                 size   = 0x0100_0000,
             ))
            self.submodules += WishboneClockCrossing(platform,
                wb_from = wb_slave_sys,
                cd_from = "sys",
                wb_to   = wb_slave_wr,
                cd_to   = "wr",
            )

            # White Rabbit RefClk / DMTD MMCM Phase Shift.
            # --------------------------------------------

            # RefClk MMCM Phase Shift.
            self.refclk_mmcm_ps_gen = Instance("ps_gen",
                p_WIDTH       = 16,
                p_DIV         = 16,
                p_MULT        = 7,

                i_pswidth     = dac_refclk_data,
                i_pswidth_set = dac_refclk_load,
                i_pswidth_clk = ClockSignal("wr"),

                i_psclk       = ClockSignal("clk200"),
                i_psdone      = self.crg.refclk_mmcm.psdone,
                o_psen        = self.crg.refclk_mmcm.psen,
                o_psincdec    = self.crg.refclk_mmcm.psincdec,
            )

            # DMTD MMCM Phase Shift.
            self.dac_mmcm_ps_gen = Instance("ps_gen",
                p_WIDTH       = 16,
                p_DIV         = 16,
                p_MULT        = 7,

                i_pswidth     = dac_dmtd_data,
                i_pswidth_set = dac_dmtd_load,
                i_pswidth_clk = ClockSignal("wr"),

                i_psclk       = ClockSignal("clk200"),
                i_psdone      = self.crg.dmtd_mmcm.psdone,
                o_psen        = self.crg.dmtd_mmcm.psen,
                o_psincdec    = self.crg.dmtd_mmcm.psincdec,
            )

            # White Rabbit Core Instance.
            # ---------------------------
            self.specials += Instance("xwrc_board_hyvision_wrapper",
                # Parameters.
                p_g_dpram_initf       = os.path.abspath(white_rabbit_cpu_firmware),
                p_g_with_external_clock_input = "FALSE",
                p_g_dpram_size        = 131072//4,
                p_txpolarity          = 0, # Inverted on Acorn and on baseboard.
                p_rxpolarity          = 0, # Inverted on Acorn.

                # Clocks/resets.
                i_areset_n_i          = ~ResetSignal("sys"),
                i_clk_62m5_dmtd_i     = ClockSignal("clk_62m5_dmtd"),
                i_clk_125m_gtp_i      = ClockSignal("clk_125m_gtp"),
                i_clk_10m_ext_i       = 0,
                o_clk_62m5_sys_o      = ClockSignal("wr"),
                o_rst_62m5_sys_o      = ResetSignal("wr"),

                # DAC RefClk Interface.
                o_dac_refclk_load     = dac_refclk_load,
                o_dac_refclk_data     = dac_refclk_data,

                # DAC DMTD Interface.
                o_dac_dmtd_load       = dac_dmtd_load,
                o_dac_dmtd_data       = dac_dmtd_data,

                # SFP Interface.
                o_sfp_txp_o           = sfp_pads.txp,
                o_sfp_txn_o           = sfp_pads.txn,
                i_sfp_rxp_i           = sfp_pads.rxp,
                i_sfp_rxn_i           = sfp_pads.rxn,
                i_sfp_det_i           = sfp_det_pads,
                io_sfp_sda            = sfp_pads.sda,
                io_sfp_scl            = sfp_pads.scl,
                i_sfp_tx_fault_i      = sfp_tx_fault_pads,
                i_sfp_tx_los_i        = sfp_lost_pads,
                o_sfp_tx_disable_o    = sfp_tx_disable_pads,

                # One-Wire Interface.
                i_onewire_i           = 0,
                o_onewire_oen_o       = Open(),

                # UART Interface.
                i_uart_rxd_i          = serial_pads.rx,
                o_uart_txd_o          = serial_pads.tx,

                # SPI Flash Interface.
                o_spi_sclk_o          = Open(),
                o_spi_ncs_o           = Open(),
                o_spi_mosi_o          = Open(),
                i_spi_miso_i          = 0,

                # PPS / Leds.
                o_pps_valid_o         = Open(),
                i_pps_ext_i           = 0,
                o_pps_csync_o         = pps_out_pulse,
                o_pps_p_o             = pps,
                o_pps_led_o           = led_pps,
                o_led_link_o          = led_link,
                o_led_act_o           = led_act,

                # QPLL Interface (for GTPE2_Common Sharing).
                o_gt0_ext_qpll_reset  = Open(),
                i_gt0_ext_qpll_clk    = Constant(0, 1),
                i_gt0_ext_qpll_refclk = Constant(0, 1),
                i_gt0_ext_qpll_lock   = Constant(0, 1),

                # Wishbone Slave Interface (MMAP).
                i_wb_slave_cyc        = wb_slave_wr.cyc,
                i_wb_slave_stb        = wb_slave_wr.stb,
                i_wb_slave_we         = wb_slave_wr.we,
                i_wb_slave_adr        = Cat(Signal(2), (wb_slave_wr.adr & 0x00ff_ffff)),
                i_wb_slave_sel        = wb_slave_wr.sel,
                i_wb_slave_dat_i      = wb_slave_wr.dat_w,
                o_wb_slave_dat_o      = wb_slave_wr.dat_r,
                o_wb_slave_ack        = wb_slave_wr.ack,
                o_wb_slave_err        = wb_slave_wr.err,
                o_wb_slave_rty        = Open(),
                o_wb_slave_stall      = Open(),

                # Wishbone Fabric Source Interface.
                o_wrf_src_adr         = wrf_wb2stream.bus.adr,
                o_wrf_src_dat         = wrf_wb2stream.bus.dat_w,
                o_wrf_src_cyc         = wrf_wb2stream.bus.cyc,
                o_wrf_src_stb         = wrf_wb2stream.bus.stb,
                o_wrf_src_we          = wrf_wb2stream.bus.we,
                o_wrf_src_sel         = wrf_wb2stream.bus.sel,

                i_wrf_src_ack         = wrf_wb2stream.bus.ack,
                i_wrf_src_stall       = 0, # Not Used.
                i_wrf_src_err         = wrf_wb2stream.bus.err,
                i_wrf_src_rty         = 0, # Not Used.

                # Wishbone Fabric Sink Interface.
                i_wrf_snk_adr         = wrf_stream2wb.bus.adr,
                i_wrf_snk_dat         = wrf_stream2wb.bus.dat_w,
                i_wrf_snk_cyc         = wrf_stream2wb.bus.cyc,
                i_wrf_snk_stb         = wrf_stream2wb.bus.stb,
                i_wrf_snk_we          = wrf_stream2wb.bus.we,
                i_wrf_snk_sel         = wrf_stream2wb.bus.sel,

                o_wrf_snk_ack         = wrf_stream2wb.bus.ack,
                o_wrf_snk_stall       = Open(), # Not Used.
                o_wrf_snk_err         = wrf_stream2wb.bus.err,
                o_wrf_snk_rty         = Open(), # Not Used.

                # Time.
                o_tm_link_up_o        = Open(),
                o_tm_time_valid_o     = Open(),
                o_tm_tai_o            = tm_seconds,
                o_tm_cycles_o         = Open(),
            )
            platform.add_platform_command("set_property SEVERITY {{Warning}} [get_drc_checks REQP-123]") # FIXME: Add 10MHz Ext Clk.
            platform.add_platform_command("set_property SEVERITY {{Warning}} [get_drc_checks REQP-52]")
            self.add_sources()
            platform.add_platform_command("create_clock -name wr_txoutclk -period 16.000 [get_pins -hierarchical *gtpe2_i/TXOUTCLK]")
            platform.add_platform_command("create_clock -name wr_rxoutclk -period 16.000 [get_pins -hierarchical *gtpe2_i/RXOUTCLK]")

            self.comb += [
                # Leds Output.
                platform.request("user_led", 0).eq(~led_link),
                platform.request("user_led", 1).eq(~led_act),
                platform.request("user_led", 2).eq(~led_pps),

                # PPS/Clk Output.
                platform.request("pps_out").eq(pps),
                platform.request("wr_clk_out").eq(ClockSignal("wr")),

                # SFP.
                platform.request("sfp_rs0", white_rabbit_sfp_connector).eq(1)
            ]

            # White Rabbit Ethernet PHY (over White Rabbit Fabric) ---------------------------------

            self.ethphy0 = LiteEthPHYWRGMII(wrf_stream2wb, wrf_wb2stream)

        # PCIe NIC ---------------------------------------------------------------------------------

        if with_pcie and with_white_rabbit:
            self.add_pcie_nic(pcie_phy=self.pcie_phy, eth_phys=[self.ethphy0], with_timing_constraints=False)
            self.add_pcie_ptm()

        # Etherbone --------------------------------------------------------------------------------

        if (not with_pcie) and with_white_rabbit:
            self.add_etherbone(phy=self.ethphy0, data_width=8, with_timing_constraints=False)

        # Time Generator ---------------------------------------------------------------------------

        if with_pcie and with_white_rabbit:
            # Time Generator.
            self.time_generator = TimeGenerator(
                clk_domain = "wr",
                clk_freq   = 62.5e6,
            )

            # Connect White Rabbit Time Interface to TimeGenerator's Sync Interface.
            self.comb += [
                self.time_generator.time_sync.eq(pps_out_pulse),
                self.time_generator.time_seconds.eq(tm_seconds),
            ]

            # Connect TimeGenerator's Time to PCIe PTM.
            self.comb += [
                self.ptm_requester.time_clk.eq(ClockSignal("wr")),
                self.ptm_requester.time_rst.eq(ResetSignal("wr")),
                self.ptm_requester.time.eq(self.time_generator.time)
            ]

        # Timing Constraints -----------------------------------------------------------------------

        asynchronous_clk_domains = [
            self.crg.cd_sys.clk,
            self.crg.cd_clk_62m5_dmtd.clk,
            self.crg.cd_clk_125m_gtp.clk,
            self.crg.cd_clk10m_in.clk,
            self.crg.cd_clk62m5_in.clk,
            "wr_txoutclk",
            "wr_rxoutclk",
        ]
        if with_white_rabbit:
            asynchronous_clk_domains += []

        platform.add_false_path_constraints(*asynchronous_clk_domains)

        # Clk Measurement (Debug) ------------------------------------------------------------------

        self.clk_measurement = MultiClkMeasurement(clks={
            "clk0" : ClockSignal("sys"),
            "clk1" : ClockSignal("clk_62m5_dmtd"),
            "clk2" : ClockSignal("clk_125m_gtp"),
            "clk3" : ClockSignal("clk10m_in"),
            "clk4" : ClockSignal("clk62m5_in"),
        })

# Build --------------------------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="LiteX-WR-NIC on Acorn Baseboard Mini.")

    # Build/Load/Flash Arguments.
    # ---------------------------
    parser.add_argument("--build", action="store_true", help="Build bitstream.")
    parser.add_argument("--load",  action="store_true", help="Load bitstream.")
    parser.add_argument("--flash", action="store_true", help="Flash bitstream.")

    # Probes.
    # -------
    parser.add_argument("--with-wishbone-fabric-interface-probe", action="store_true")
    parser.add_argument("--with-wishbone-slave-probe",            action="store_true")
    parser.add_argument("--with-dac-vcxo-probe",                  action="store_true")
    parser.add_argument("--with-time-pps-probe",                  action="store_true")

    args = parser.parse_args()

    # Build Firmware.
    # ---------------
    if args.build:
        print("Building firmware...")
        r = os.system("cd firmware && ./build.py --target acorn")
        if r != 0:
            raise RuntimeError("Firmware build failed.")

    # Build SoC/Gateware (with integrated Firmware).
    # ----------------------------------------------
    soc = BaseSoC()
    if args.with_wishbone_fabric_interface_probe:
        soc.add_wishbone_fabric_interface_probe()
    if args.with_wishbone_slave_probe:
        soc.add_wishbone_slave_probe()
    if args.with_dac_vcxo_probe:
        soc.add_dac_vcxo_probe()
    if args.with_time_pps_probe:
        soc.add_time_pps_probe()
    builder = Builder(soc, csr_csv="test/csr.csv")
    builder.build(run=args.build)

    # Generate PCIe C Headers.
    # ------------------------
    generate_litepcie_software_headers(soc, "software/kernel")

    # Load FPGA.
    # ----------
    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

    # Flash FPGA.
    # -----------
    if args.flash:
        prog = soc.platform.create_programmer()
        prog.flash(0, builder.get_bitstream_filename(mode="flash"))

if __name__ == "__main__":
    main()
