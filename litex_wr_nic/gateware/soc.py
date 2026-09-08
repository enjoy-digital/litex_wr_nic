#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
# Copyright (c) 2022 Tongchen126 <https://github.com/tongchen126>
# SPDX-License-Identifier: BSD-2-Clause

import os
import sys

from migen import *
from migen.genlib.cdc import MultiReg

from litex.build    import tools
from litex.build.io import SDRTristate, SDROutput

from litex.gen import *

from litex.soc.interconnect import wishbone

from litex.soc.integration.soc      import SoCRegion
from litex.soc.integration.soc_core import *

from litescope import LiteScopeAnalyzer

from litepcie.frontend.ptm  import PCIePTMSniffer
from litepcie.frontend.ptm  import PTMCapabilities, PTMRequester

from litex_wr_nic.gateware.nic import sram
sys.modules["liteeth.mac.sram"] = sram #  Replace Liteeth SRAM with our custom implementation.
from litex_wr_nic.gateware.nic.dma import LitePCIe2WishboneDMA

from litex_wr_nic.gateware.wr_common         import (
    wr_core_init,
    wr_core_files,
    patch_wr_subsystem_mux_class,
    patch_wr_pps_gen_iob,
    patch_wr_clock_monitor_presc_cdc,
    patch_wr_external_cpu_memory,
)
from litex_wr_nic.gateware.wr_cpu            import WRCPUMemoryBridge, wr_cpu_word_bus
from litex_wr_nic.gateware.wrf_stream2wb     import Stream2Wishbone
from litex_wr_nic.gateware.wrf_wb2stream     import Wishbone2Stream
from litex_wr_nic.gateware.wb_clock_crossing import WishboneClockCrossing

# LiteX WR NIC SoC ---------------------------------------------------------------------------------

class LiteXWRNICSoC(SoCMini):
    SoCMini.csr_map = {
        # Common (0-7).
        "identifier_mem"          : 1,
        "ctrl"                    : 2,
        "leds"                    : 3,
        "clk_measurement"         : 4,

        # PCIe (8-15).
        "pcie_endpoint"           : 8,
        "pcie_msi"                : 9,
        "pcie_phy"                : 10,
        "pcie_dma0"               : 11,
        "ptm_requester"           : 12,

        # Ethernet (16-20).
        "ethmac0"                 : 16,
        "ethmac1"                 : 17,

        # White Rabbit (20-31).
        "refclk_pll"              : 20,
        "refclk_dac"              : 21,
        "dmtd_dac"                : 22,
        "rf_out_pll"              : 23,
        "sync_in_pll"             : 24,
        "time_generator"          : 25,
        "pps_in_macro_delay"      : 26,
        "pps_out_macro_delay"     : 27,
        "pps_out_coarse_delay"    : 28,
        "clk10m_macro_delay"      : 29,
        "clk10m_out_coarse_delay" : 30,
        "fine_delay"              : 31,
        "wr_cpu_bridge"           : 5,
        "wr_cpu_boot"             : 6,
        "hyperram"                : 7,
    }
    SoCMini.mem_map = {
        "csr"      : 0x0000_0000,
        "ethmac0"  : 0x0002_0000,
        "ethmac1"  : 0x0004_0000,
    }

    # Add White Rabbit Core ------------------------------------------------------------------------
    def add_wr_core(self,
        # CPU.
        cpu_firmware,
        cpu_memory_region = None,
        cpu_memory_ready  = 1,
        cpu_boot_loader   = None,

        # Board name.
        board_name  = "NA  ",

        # SFP.
        sfp_pads         = None,
        sfp_i2c_pads     = None,
        sfp_tx_polarity  = 0,
        sfp_rx_polarity  = 0,
        sfp_disable_pads = None,
        sfp_fault_pads   = None,
        sfp_los_pads     = None,
        sfp_det_pads     = None,

        # Clocking.
        qpll         = None,
        with_ext_clk = True,

        # Serial.
        serial_pads = None,

        # Flash.
        flash_pads = None,

        # Temp 1Wire.
        temp_1wire_pads = None,

        # Wishbone Slave.
        wb_slave_origin = 0x2000_0000,
        wb_slave_size   = 0x0100_0000,

        dac_bits      = 16
    ):

        # Clks.
        # -----
        # Keep PPS/timecode in the PHY reference domain. The WR CPU, fabric,
        # Wishbone and DAC commands run on the independent WR system clock.
        self.cd_wr     = ClockDomain("wr")
        self.cd_wr_sys = ClockDomain("wr_sys")

        # Signals.
        # --------
        self.led_pps         = Signal()
        self.led_link        = Signal()
        self.led_act         = Signal()
        self.dac_refclk_load = Signal()
        self.dac_refclk_data = Signal(dac_bits)
        self.dac_dmtd_load   = Signal()
        self.dac_dmtd_data   = Signal(dac_bits)
        self.pps_in          = Signal()
        self.pps_out_valid   = Signal()
        self.pps_out         = Signal()
        self.pps_out_pulse   = Signal()
        self.tm_link_up      = Signal()
        self.tm_time_valid   = Signal()
        self.tm_seconds      = Signal(40)
        self.tm_cycles       = Signal(28)

        # Optional WR CPU Memory Master.
        # ------------------------------
        external_cpu_memory = cpu_memory_region is not None
        memory_ready_wr     = Signal(reset=not external_cpu_memory)
        if external_cpu_memory:
            wr_cpu_bus_sys = wishbone.Interface(data_width=32, address_width=32, addressing="byte")
            self.wr_cpu_bridge = wr_cpu_bridge = WRCPUMemoryBridge(monitor_bus=wr_cpu_bus_sys)
            self.submodules.wr_cpu_memory_cdc = WishboneClockCrossing(self.platform,
                wb_from        = wr_cpu_bridge.bus,
                cd_from        = "wr_sys",
                wb_to          = wr_cpu_bus_sys,
                cd_to          = "sys",
                timeout_cycles = 1024,
            )
            self.bus.add_master(name="wr_cpu",
                master = wr_cpu_word_bus(self, wr_cpu_bus_sys),
                region = cpu_memory_region,
            )
            self.specials += MultiReg(cpu_memory_ready, memory_ready_wr, "wr_sys")

        # White Rabbit Fabric Interface.
        # ------------------------------
        self.wrf_stream2wb = wrf_stream2wb = Stream2Wishbone(  cd_to="wr_sys")
        self.wrf_wb2stream = wrf_wb2stream = Wishbone2Stream(cd_from="wr_sys")

        # White Rabbit Slave Interface.
        # -----------------------------
        wb_slave_mask = (wb_slave_size - 1)
        self.wb_slave_sys = wb_slave_sys = wishbone.Interface(data_width=32, address_width=32, adressing="byte")
        self.wb_slave_wr  = wb_slave_wr  = wishbone.Interface(data_width=32, address_width=32, adressing="byte")
        self.bus.add_slave(name="wr_wb_slave", slave=wb_slave_sys, region=SoCRegion(
             origin = wb_slave_origin,
             size   = wb_slave_size,
         ))
        self.submodules += WishboneClockCrossing(self.platform,
            wb_from = wb_slave_sys,
            cd_from = "sys",
            wb_to   = wb_slave_wr,
            cd_to   = "wr_sys",
        )

        # Temp 1-Wire Logic.
        # ------------------
        if temp_1wire_pads is not None:
            temp_1wire_oe_n = Signal()
            temp_1wire_i    = Signal()
            self.specials += SDRTristate(
                io = temp_1wire_pads,
                o  = Constant(0b0, 1),
                oe = ~temp_1wire_oe_n,
                i  = temp_1wire_i,
                clk = ClockSignal("wr_sys"),
            )

        # Flash Logic.
        # ------------
        if flash_pads is not None:
            flash_clk     = Signal()
            wr_flash_clk  = Signal()
            wr_flash_cs   = Signal(reset=1)
            wr_flash_mosi = Signal()
            if cpu_boot_loader is None:
                self.comb += [
                    flash_clk.eq(wr_flash_clk),
                    flash_pads.cs_n.eq(wr_flash_cs),
                    flash_pads.mosi.eq(wr_flash_mosi),
                ]
            else:
                self.comb += [
                    cpu_boot_loader.miso.eq(flash_pads.miso),
                    flash_clk.eq(Mux(cpu_boot_loader.owner,
                        cpu_boot_loader.clk, wr_flash_clk)),
                    flash_pads.cs_n.eq(Mux(cpu_boot_loader.owner,
                        cpu_boot_loader.cs_n, wr_flash_cs)),
                    flash_pads.mosi.eq(Mux(cpu_boot_loader.owner,
                        cpu_boot_loader.mosi, wr_flash_mosi)),
                ]
            self.specials += Instance("STARTUPE2",
                i_CLK       = 0,
                i_GSR       = 0,
                i_GTS       = 0,
                i_KEYCLEARB = 0,
                i_PACK      = 0,
                i_USRCCLKO  = flash_clk,
                i_USRCCLKTS = 0,
                i_USRDONEO  = 1,
                i_USRDONETS = 1,
            )
            # Keep optional WP#/HOLD# lines deasserted in 1-bit SPI mode.
            if hasattr(flash_pads, "wp"):
                self.comb += flash_pads.wp.eq(1)
            if hasattr(flash_pads, "hold"):
                self.comb += flash_pads.hold.eq(1)

        # White Rabbit Core Instance.
        # ---------------------------
        self.specials += Instance("xwrc_board_litex_wr_nic_wrapper",
            # Parameters.
            p_g_dpram_initf               = os.path.abspath(cpu_firmware),
            p_g_dpram_size                = 131072//4,
            # Vivado binds quoted "FALSE" to true across the Verilog/VHDL
            # boundary; use a numeric boolean for deterministic elaboration.
            p_g_external_cpu_memory       = int(external_cpu_memory),
            p_txpolarity                  = sfp_tx_polarity,
            p_rxpolarity                  = sfp_rx_polarity,
            p_g_with_external_clock_input = str(with_ext_clk).upper(),
            p_g_fpga_family               = {True: "artix7", False: "kintex7"}[self.platform.device.startswith("xc7a")],
            p_g_board_name                = board_name,
            p_g_dac_bits                  = dac_bits,

            # Clocks/resets.
            i_areset_n_i          = ~ResetSignal("sys"),
            i_clk_62m5_dmtd_i     = ClockSignal("clk_62m5_dmtd"),
            i_clk_125m_gtp_i      = ClockSignal("clk_125m_gtp"),
            i_clk_10m_ext_i       = ClockSignal("clk10m_in"),
            o_clk_62m5_sys_o      = ClockSignal("wr_sys"),
            o_rst_62m5_sys_o      = ResetSignal("wr_sys"),
            o_clk_62m5_ref_o      = ClockSignal("wr"),
            o_rst_62m5_ref_o      = ResetSignal("wr"),

            # DAC RefClk Interface.
            o_dac_refclk_load     = self.dac_refclk_load,
            o_dac_refclk_data     = self.dac_refclk_data,

            # DAC DMTD Interface.
            o_dac_dmtd_load       = self.dac_dmtd_load,
            o_dac_dmtd_data       = self.dac_dmtd_data,

            # SFP Interface.
            o_sfp_txp_o           = sfp_pads.txp,
            o_sfp_txn_o           = sfp_pads.txn,
            i_sfp_rxp_i           = sfp_pads.rxp,
            i_sfp_rxn_i           = sfp_pads.rxn,
            i_sfp_det_i           = 0      if sfp_det_pads is None else sfp_det_pads,
            io_sfp_sda            = sfp_i2c_pads.sda,
            io_sfp_scl            = sfp_i2c_pads.scl,
            i_sfp_tx_fault_i      = 0      if   sfp_fault_pads is None else   sfp_fault_pads,
            i_sfp_tx_los_i        = 0      if     sfp_los_pads is None else     sfp_los_pads,
            o_sfp_tx_disable_o    = Open() if sfp_disable_pads is None else sfp_disable_pads,

            # One-Wire Interface.
            i_onewire_i           = 0      if temp_1wire_pads is None else temp_1wire_i,
            o_onewire_oen_o       = Open() if temp_1wire_pads is None else temp_1wire_oe_n,

            # UART Interface.
            i_uart_rxd_i          = 0      if serial_pads is None else serial_pads.rx,
            o_uart_txd_o          = Open() if serial_pads is None else serial_pads.tx,

            # SPI Flash Interface.
            o_spi_sclk_o          = Open() if flash_pads is None else wr_flash_clk,
            o_spi_ncs_o           = Open() if flash_pads is None else wr_flash_cs,
            o_spi_mosi_o          = Open() if flash_pads is None else wr_flash_mosi,
            i_spi_miso_i          = 0      if flash_pads is None else flash_pads.miso,

            # Optional WR CPU Memory Master.
            o_cpu_mem_cyc_o       = Open() if not external_cpu_memory else wr_cpu_bridge.cyc,
            o_cpu_mem_stb_o       = Open() if not external_cpu_memory else wr_cpu_bridge.stb,
            o_cpu_mem_we_o        = Open() if not external_cpu_memory else wr_cpu_bridge.we,
            o_cpu_mem_adr_o       = Open() if not external_cpu_memory else wr_cpu_bridge.adr,
            o_cpu_mem_sel_o       = Open() if not external_cpu_memory else wr_cpu_bridge.sel,
            o_cpu_mem_dat_o       = Open() if not external_cpu_memory else wr_cpu_bridge.dat_w,
            i_cpu_mem_dat_i       = 0      if not external_cpu_memory else wr_cpu_bridge.dat_r,
            i_cpu_mem_ack_i       = 0      if not external_cpu_memory else wr_cpu_bridge.ack,
            i_cpu_mem_err_i       = 0      if not external_cpu_memory else wr_cpu_bridge.err,
            i_cpu_mem_rty_i       = 0      if not external_cpu_memory else wr_cpu_bridge.rty,
            i_cpu_mem_stall_i     = 0      if not external_cpu_memory else wr_cpu_bridge.stall,
            i_cpu_mem_ready_i     = 1      if not external_cpu_memory else memory_ready_wr,

            # PPS / Leds.
            i_pps_ext_i           = self.pps_in,
            o_pps_valid_o         = self.pps_out_valid,
            o_pps_csync_o         = self.pps_out_pulse,
            o_pps_p_o             = self.pps_out,
            o_pps_led_o           = self.led_pps,
            o_led_link_o          = self.led_link,
            o_led_act_o           = self.led_act,

            # QPLL Interface (for GTPE2_Common Sharing).
            o_gt0_ext_qpll_reset  = Open() if qpll is None else qpll.get_channel("eth").reset,
            i_gt0_ext_qpll_clk    = 0      if qpll is None else qpll.get_channel("eth").clk,
            i_gt0_ext_qpll_refclk = 0      if qpll is None else qpll.get_channel("eth").refclk,
            i_gt0_ext_qpll_lock   = 0      if qpll is None else qpll.get_channel("eth").lock,

            # Wishbone Slave Interface (MMAP).
            i_wb_slave_cyc        = wb_slave_wr.cyc,
            i_wb_slave_stb        = wb_slave_wr.stb,
            i_wb_slave_we         = wb_slave_wr.we,
            i_wb_slave_adr        = Cat(Signal(2), (wb_slave_wr.adr & wb_slave_mask)),
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
            o_tm_link_up_o        = self.tm_link_up,
            o_tm_time_valid_o     = self.tm_time_valid,
            o_tm_tai_o            = self.tm_seconds,
            o_tm_cycles_o         = self.tm_cycles,
        )

    # Add PCIe NIC ---------------------------------------------------------------------------------

    def add_pcie_nic(self, pcie_phy=None, eth_phys=[], eth_ntxslots=4, eth_nrxslots=4, with_timing_constraints=True):
        # PCIe MSIs.
        # ----------
        pcie_msis = {}
        for n in range(len(eth_phys)):
            pcie_msis.update({
                f"ethmac{n}_rx" : Signal(),
                f"ethmac{n}_tx" : Signal(),
            })

        # PCIe Core.
        # ----------
        self.add_pcie(name="pcie", phy=pcie_phy,
            ndmas                = len(eth_phys),
            max_pending_requests = 4,
            data_width           = 64,
            with_dma_buffering   = False,
            with_dma_loopback    = False,
            with_dma_table       = False,
            with_ptm             = True,
            with_msi             = True,
            msis                 = pcie_msis,
        )

        # Ethernet PHYs/MACs.
        # -------------------
        for n, eth_phy in enumerate(eth_phys):
            pcie_dma = self.get_module(f"pcie_dma{n}")

            # Ethernet MAC.
            # -------------
            ethmac_name = f"ethmac{n}"
            eth_phy_cd  = "eth"
            if len(eth_phys) > 1:
                eth_phy_cd = f"ethphy{n}_eth"
            self.add_ethernet(
                name       = ethmac_name,
                phy        = eth_phy,
                phy_cd     = eth_phy_cd,
                data_width = 64,
                ntxslots   = eth_ntxslots,
                nrxslots   = eth_nrxslots,
                with_timing_constraints = with_timing_constraints,
            )
            ethmac = self.get_module(ethmac_name)
            del self.bus.slaves[f"{ethmac_name}_tx"] # Remove from SoC bus since directly connected to PCIe.
            del self.bus.slaves[f"{ethmac_name}_rx"] # Remove from SoC bus since directly connected to PCIe.

            # RX Datapath: Ethernet (RX) -> PCIe -> Host.
            # -------------------------------------------
            align_bits = log2_int(512)
            pcie_wb2pcie_dma = LitePCIe2WishboneDMA(
                endpoint   = self.pcie_endpoint,
                dma        = pcie_dma.writer,
                data_width = 64,
                mode       = "wb2pcie",
            )
            self.add_module(name=f"pcie_wb2pcie_dma{n}", module=pcie_wb2pcie_dma)
            self.comb += [
                pcie_wb2pcie_dma.desc.valid.eq(ethmac.interface.sram.writer.start),
                ethmac.interface.sram.writer.ready.eq(pcie_wb2pcie_dma.desc.ready),
                pcie_wb2pcie_dma.desc.bus_addr.eq(ethmac.interface.sram.writer.stat_fifo.source.slot * ethmac.slot_size.constant),
                pcie_wb2pcie_dma.desc.host_addr.eq(ethmac.interface.sram.writer.pcie_host_addr),
                pcie_wb2pcie_dma.desc.length[align_bits:].eq(ethmac.interface.sram.writer.stat_fifo.source.length[align_bits:] + 1),
            ]
            self.submodules += wishbone.InterconnectPointToPoint(
                master = pcie_wb2pcie_dma.bus,
                slave  = ethmac.bus_rx,
            )

            # TX Datapath: Host -> PCIe -> Ethernet (TX).
            # -------------------------------------------
            align_bits = log2_int(512)
            pcie_pcie2wb_dma = LitePCIe2WishboneDMA(
                endpoint   = self.pcie_endpoint,
                dma        = pcie_dma.reader,
                data_width = 64,
                mode       = "pcie2wb",
            )
            self.add_module(name=f"pcie_pcie2wb_dma{n}", module=pcie_pcie2wb_dma)
            self.comb += [
                pcie_pcie2wb_dma.desc.valid.eq(ethmac.interface.sram.reader.start),
                ethmac.interface.sram.reader.ready.eq(pcie_pcie2wb_dma.desc.ready),
                pcie_pcie2wb_dma.desc.bus_addr.eq(ethmac.interface.sram.reader.cmd_fifo.source.slot * ethmac.slot_size.constant),
                pcie_pcie2wb_dma.desc.host_addr.eq(ethmac.interface.sram.reader.pcie_host_addr),
                pcie_pcie2wb_dma.desc.length[align_bits:].eq(ethmac.interface.sram.reader.cmd_fifo.source.length[align_bits:] + 1),
            ]
            self.submodules += wishbone.InterconnectPointToPoint(
                master = pcie_pcie2wb_dma.bus,
                slave  = ethmac.bus_tx,
            )

            # Connect MSIs.
            # -------------
            self.comb += [
                self.msis[f"{ethmac_name}_rx"].eq(ethmac.interface.sram.writer.pcie_irq),
                self.msis[f"{ethmac_name}_tx"].eq(ethmac.interface.sram.reader.pcie_irq),
            ]

    # Add PCIe PTM ---------------------------------------------------------------------------------

    def add_pcie_ptm(self):
        # PCIe PTM Sniffer.
        # -----------------

        # Since Xilinx PHY does not allow redirecting PTM TLP Messages to the AXI inferface, we have
        # to sniff the GTPE2 -> PCIE2 RX Data to re-generate PTM TLP Messages.

        # Sniffer Signals.
        # ----------------
        sniffer_rst_n   = Signal()
        sniffer_clk     = Signal()
        sniffer_rx_data = Signal(16)
        sniffer_rx_ctl  = Signal(2)

        # Sniffer Tap.
        # ------------
        rx_data = Signal(16)
        rx_ctl  = Signal(2)
        self.sync.pclk += rx_data.eq(rx_data + 1)
        self.sync.pclk += rx_ctl.eq(rx_ctl + 1)
        self.specials += Instance("sniffer_tap",
            i_rst_n_in    = 1,
            i_clk_in     = ClockSignal("pclk"),
            i_rx_data_in = rx_data, # /!\ Fake, will be re-connected post-synthesis /!\.
            i_rx_ctl_in  = rx_ctl,  # /!\ Fake, will be re-connected post-synthesis /!\.
            o_rst_n_out   = sniffer_rst_n,
            o_clk_out     = sniffer_clk,
            o_rx_data_out = sniffer_rx_data,
            o_rx_ctl_out  = sniffer_rx_ctl,
        )

        # Sniffer.
        # --------
        self.pcie_ptm_sniffer = PCIePTMSniffer(
            rx_rst_n = sniffer_rst_n,
            rx_clk   = sniffer_clk,
            rx_data  = sniffer_rx_data,
            rx_ctrl  = sniffer_rx_ctl,
        )
        self.pcie_ptm_sniffer.add_sources(self.platform)


        # PTM
        # ---

        # PTM Capabilities.
        self.ptm_capabilities = PTMCapabilities(
            pcie_endpoint     = self.pcie_endpoint,
            requester_capable = True,
        )

        # PTM Requester.
        self.ptm_requester = PTMRequester(
            pcie_endpoint    = self.pcie_endpoint,
            pcie_ptm_sniffer = self.pcie_ptm_sniffer,
            sys_clk_freq     = self.sys_clk_freq,
        )

        # Sniffer Post-Synthesis connections.
        # -----------------------------------
        pcie_ptm_sniffer_connections = []
        for n in range(2):
            pcie_ptm_sniffer_connections.append((
                f"pcie_s7/inst/inst/gt_top_i/gt_rx_data_k_wire_filter[{n}]", # Src.
                f"sniffer_tap/rx_ctl_in[{n}]",                               # Dst.
            ))
        for n in range(16):
            pcie_ptm_sniffer_connections.append((
                f"pcie_s7/inst/inst/gt_top_i/gt_rx_data_wire_filter[{n}]", # Src.
                f"sniffer_tap/rx_data_in[{n}]",                            # Dst.
            ))
        for _from, _to in pcie_ptm_sniffer_connections:
            self.platform.toolchain.pre_optimize_commands.append(f"set pin_driver [get_nets -of [get_pins {_to}]]")
            self.platform.toolchain.pre_optimize_commands.append(f"disconnect_net -net $pin_driver -objects {_to}")
            self.platform.toolchain.pre_optimize_commands.append(f"connect_net -hier -net {_from} -objects {_to}")

    # Add Sources ----------------------------------------------------------------------------------

    def add_sources(self):
        if not os.path.exists("wr-cores"):
            wr_core_init()
        patch_wr_subsystem_mux_class()
        patch_wr_pps_gen_iob()
        patch_wr_clock_monitor_presc_cdc()
        patch_wr_external_cpu_memory()
        for file in wr_core_files:
            self.platform.add_source(file)

    # Add Probes -----------------------------------------------------------------------------------

    def add_wishbone_fabric_interface_probe(self):
       analyzer_signals = [
           self.wrf_stream2wb.bus,
           self.wrf_stream2wb.sink,
           self.wrf_stream2wb.fsm,

           self.wrf_wb2stream.bus,
           self.wrf_wb2stream.source,
           self.wrf_wb2stream.fsm,
       ]
       self.analyzer = LiteScopeAnalyzer(analyzer_signals,
           depth        = 256,
           clock_domain = "wr",
           samplerate   = int(62.5e6),
           register     = True,
           csr_csv      = "test/analyzer.csv"
       )

    def add_wishbone_slave_probe(self):
        analyzer_signals = [
            self.wb_slave_sys,
            self.wb_slave_wr,
        ]
        self.analyzer = LiteScopeAnalyzer(analyzer_signals,
           depth        = 256,
           clock_domain = "sys",
           samplerate   = 125e6,
           register     = True,
           csr_csv      = "test/analyzer.csv"
       )

    def add_dac_vcxo_probe(self):
        analyzer_signals = [
            self.refclk_pll._done.status,
            self.refclk_dac._current.status,
            self.dmtd_dac._current.status,
        ]
        self.analyzer = LiteScopeAnalyzer(analyzer_signals,
            depth        = 512,
            clock_domain = "wr",
            samplerate   = int(62.5e6),
            register     = True,
            csr_csv      = "test/analyzer.csv"
        )

    def add_time_pps_probe(self):
        analyzer_signals = [
            self.platform.lookup_request("pps_in"),
            self.pps_in,
            self.pps_out,
            self.pps_out_pulse,
            self.tm_link_up,
            self.tm_time_valid,
            self.tm_seconds,
            self.tm_cycles,
        ]
        if hasattr(self, "time_generator"):
            analyzer_signals += [
                self.time_generator.time_sync,
                self.time_generator.time_seconds,
                self.time_generator.time,
            ]
        self.analyzer = LiteScopeAnalyzer(analyzer_signals,
            depth        = 512,
            clock_domain = "wr",
            samplerate   = int(62.5e6),
            register     = True,
            csr_csv      = "test/analyzer.csv"
        )
