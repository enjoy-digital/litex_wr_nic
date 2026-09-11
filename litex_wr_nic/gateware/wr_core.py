#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
# Copyright (c) 2022 Tongchen126 <https://github.com/tongchen126>
# SPDX-License-Identifier: BSD-2-Clause

import os

from migen import *
from migen.genlib.cdc import MultiReg

from litex.gen import *

from litex.build.io import SDRTristate

from litex.soc.interconnect import wishbone

from litex_wr_nic.gateware.wr_common         import (
    wr_core_init,
    wr_core_files,
    patch_wr_subsystem_mux_class,
    patch_wr_pps_gen_iob,
    patch_wr_clock_monitor_presc_cdc,
    patch_wr_external_cpu_memory,
    patch_wr_diags_control_word,
)
from litex_wr_nic.gateware.wr_cpu            import (
    WRCPUMemoryBridge,
    WRCPUMemoryMonitor,
    WRLiteXCPU,
    resolve_wr_cpu_variant,
    wr_cpu_word_bus,
)
from litex_wr_nic.gateware.wrf_stream2wb     import Stream2Wishbone
from litex_wr_nic.gateware.wrf_wb2stream     import Wishbone2Stream
from litex_wr_nic.gateware.wb_clock_crossing import WishboneClockCrossing

# White Rabbit Core -------------------------------------------------------------------------------

class WhiteRabbitCore(LiteXModule):
    """White Rabbit without a NIC or a SoC address-map dependency.

    ``bus`` is a word-addressed host slave and ``cpu_bus`` an optional
    word-addressed memory master, both in ``sys``. ``sink``/``source`` carry
    application Ethernet bytes in ``sys``. Control/fabric/DAC signals use
    ``wr_sys``; PPS and timecode use ``wr`` (the PHY reference clock).
    The caller supplies board pads and clocks and registers the bus regions.
    HDL sources are registered automatically during finalization.
    """

    def __init__(self, platform,
        # CPU.
        cpu_firmware,
        cpu_type          = "urv",
        cpu_variant       = None,
        with_cpu_memory   = False,
        cpu_memory_ready  = 1,
        cpu_boot_loader   = None,

        # Board name.
        board_name = "NA  ",

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
        wb_slave_size = 0x0100_0000,

        dac_bits      = 16
    ):

        self.platform = platform
        self.cpu_bus  = None

        # # #

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
        cpu_variant         = resolve_wr_cpu_variant(cpu_type, cpu_variant)
        external_cpu        = cpu_type != "urv"
        external_cpu_memory = with_cpu_memory
        if external_cpu and not external_cpu_memory:
            raise ValueError(f"WR CPU type {cpu_type} requires external CPU memory.")

        memory_ready_wr   = Signal(reset=not external_cpu_memory)
        wr_cpu_bridge     = None
        wr_cpu_peripheral = None
        wr_cpu_irq        = Signal()
        wr_cpu_reset      = Signal()
        if external_cpu_memory:
            if external_cpu:
                self.wr_cpu = WRLiteXCPU(self.platform,
                    cpu_type       = cpu_type,
                    variant        = cpu_variant,
                    irq            = wr_cpu_irq,
                    software_reset = wr_cpu_reset,
                    memory_ready   = memory_ready_wr,
                )
                wr_cpu_bus_wr     = self.wr_cpu.memory_bus
                wr_cpu_bus_sys    = wishbone.Interface.like(wr_cpu_bus_wr)
                wr_cpu_peripheral = self.wr_cpu.peripheral_bridge
                self.wr_cpu_bridge = wr_cpu_bridge = WRCPUMemoryMonitor(monitor_bus=wr_cpu_bus_sys)
            else:
                wr_cpu_bus_sys = wishbone.Interface(
                    data_width    = 32,
                    address_width = 32,
                    addressing    = "byte",
                )
                self.wr_cpu_bridge = wr_cpu_bridge = WRCPUMemoryBridge(monitor_bus=wr_cpu_bus_sys)
                wr_cpu_bus_wr = wr_cpu_bridge.bus
            self.submodules.wr_cpu_memory_cdc = WishboneClockCrossing(self.platform,
                wb_from        = wr_cpu_bus_wr,
                cd_from        = "wr_sys",
                wb_to          = wr_cpu_bus_sys,
                cd_to          = "sys",
                timeout_cycles = 1024,
            )
            self.cpu_bus = wr_cpu_word_bus(self, wr_cpu_bus_sys)
            self.specials += MultiReg(cpu_memory_ready, memory_ready_wr, "wr_sys")

        # White Rabbit Fabric Interface.
        # ------------------------------
        self.wrf_stream2wb = wrf_stream2wb = Stream2Wishbone(  cd_to="wr_sys")
        self.wrf_wb2stream = wrf_wb2stream = Wishbone2Stream(cd_from="wr_sys")

        # White Rabbit Slave Interface.
        # -----------------------------
        # The host bus uses word addresses; the region size is in bytes.
        wb_slave_mask = (wb_slave_size // 4) - 1
        self.wb_slave_sys = wb_slave_sys = wishbone.Interface(
            data_width=32, address_width=32, addressing="word")
        self.wb_slave_wr  = wb_slave_wr  = wishbone.Interface(
            data_width=32, address_width=32, addressing="word")
        self.bus    = wb_slave_sys
        self.sink   = wrf_stream2wb.sink
        self.source = wrf_wb2stream.source
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
                io  = temp_1wire_pads,
                o   = Constant(0b0, 1),
                oe  = ~temp_1wire_oe_n,
                i   = temp_1wire_i,
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
            p_g_external_cpu              = int(external_cpu),
            p_txpolarity                  = sfp_tx_polarity,
            p_rxpolarity                  = sfp_rx_polarity,
            p_g_with_external_clock_input = int(with_ext_clk),
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
            i_uart_rxd_i          = 1      if serial_pads is None else serial_pads.rx,
            o_uart_txd_o          = Open() if serial_pads is None else serial_pads.tx,

            # SPI Flash Interface.
            o_spi_sclk_o          = Open() if flash_pads is None else wr_flash_clk,
            o_spi_ncs_o           = Open() if flash_pads is None else wr_flash_cs,
            o_spi_mosi_o          = Open() if flash_pads is None else wr_flash_mosi,
            i_spi_miso_i          = 0      if flash_pads is None else flash_pads.miso,

            # Optional WR CPU Memory Master.
            o_cpu_mem_cyc_o       = Open() if not external_cpu_memory or external_cpu else wr_cpu_bridge.cyc,
            o_cpu_mem_stb_o       = Open() if not external_cpu_memory or external_cpu else wr_cpu_bridge.stb,
            o_cpu_mem_we_o        = Open() if not external_cpu_memory or external_cpu else wr_cpu_bridge.we,
            o_cpu_mem_adr_o       = Open() if not external_cpu_memory or external_cpu else wr_cpu_bridge.adr,
            o_cpu_mem_sel_o       = Open() if not external_cpu_memory or external_cpu else wr_cpu_bridge.sel,
            o_cpu_mem_dat_o       = Open() if not external_cpu_memory or external_cpu else wr_cpu_bridge.dat_w,
            i_cpu_mem_dat_i       = 0      if not external_cpu_memory or external_cpu else wr_cpu_bridge.dat_r,
            i_cpu_mem_ack_i       = 0      if not external_cpu_memory or external_cpu else wr_cpu_bridge.ack,
            i_cpu_mem_err_i       = 0      if not external_cpu_memory or external_cpu else wr_cpu_bridge.err,
            i_cpu_mem_rty_i       = 0      if not external_cpu_memory or external_cpu else wr_cpu_bridge.rty,
            i_cpu_mem_stall_i     = 0      if not external_cpu_memory or external_cpu else wr_cpu_bridge.stall,
            i_cpu_mem_ready_i     = 1      if not external_cpu_memory else memory_ready_wr,

            # Optional LiteX WR CPU peripheral master, interrupt and reset.
            i_cpu_ext_cyc_i       = 0 if not external_cpu else wr_cpu_peripheral.cyc,
            i_cpu_ext_stb_i       = 0 if not external_cpu else wr_cpu_peripheral.stb,
            i_cpu_ext_we_i        = 0 if not external_cpu else wr_cpu_peripheral.we,
            i_cpu_ext_adr_i       = 0 if not external_cpu else Cat(
                Constant(0, 2), wr_cpu_peripheral.adr),
            i_cpu_ext_sel_i       = 0 if not external_cpu else wr_cpu_peripheral.sel,
            i_cpu_ext_dat_i       = 0 if not external_cpu else wr_cpu_peripheral.dat_w,
            o_cpu_ext_dat_o       = Open() if not external_cpu else wr_cpu_peripheral.dat_r,
            o_cpu_ext_ack_o       = Open() if not external_cpu else wr_cpu_peripheral.ack,
            o_cpu_ext_err_o       = Open() if not external_cpu else wr_cpu_peripheral.err,
            o_cpu_ext_rty_o       = Open() if not external_cpu else wr_cpu_peripheral.rty,
            o_cpu_ext_stall_o     = Open() if not external_cpu else wr_cpu_peripheral.stall,
            o_cpu_ext_irq_o       = Open() if not external_cpu else wr_cpu_irq,
            o_cpu_ext_reset_o     = Open() if not external_cpu else wr_cpu_reset,

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

    def do_finalize(self):
        self.add_sources(self.platform)

    @staticmethod
    def add_sources(platform):
        # Keep explicit calls from older integrations harmless when the core
        # subsequently registers its sources during finalization.
        if getattr(platform, "_wr_core_sources_added", False):
            return
        wr_core_init()
        patch_wr_subsystem_mux_class()
        patch_wr_pps_gen_iob()
        patch_wr_clock_monitor_presc_cdc()
        patch_wr_external_cpu_memory()
        patch_wr_diags_control_word()
        for filename in wr_core_files:
            platform.add_source(filename)
        platform._wr_core_sources_added = True

# SoC Integration ---------------------------------------------------------------------------------

def add_white_rabbit(soc, cpu_firmware, cpu_memory_region=None,
    wb_slave_origin=0x2000_0000, wb_slave_size=0x0100_0000, wb_slave_region=None, **kwargs):
    """Attach the standalone core, preserving the original target attributes.

    New integrations can instantiate WhiteRabbitCore directly to choose their
    own hierarchy and CSR names. This adapter retains the existing names used
    by the NIC targets and their host software.

    wb_slave_region supplies the host mapping and overrides the legacy
    wb_slave_origin/wb_slave_size arguments when provided.
    """
    from litex.soc.integration.soc import SoCRegion

    if hasattr(soc, "wr_core"):
        raise ValueError("White Rabbit is already attached to this SoC.")
    if wb_slave_region is None:
        wb_slave_region = SoCRegion(origin=wb_slave_origin, size=wb_slave_size)
    soc.wr_core = core = WhiteRabbitCore(soc.platform,
        cpu_firmware    = cpu_firmware,
        with_cpu_memory = cpu_memory_region is not None,
        wb_slave_size = wb_slave_region.size_pow2,
        **kwargs,
    )
    soc.bus.add_slave(name="wr_wb_slave", slave=core.bus, region=wb_slave_region)
    if core.cpu_bus is not None:
        soc.bus.add_master(name="wr_cpu", master=core.cpu_bus, region=cpu_memory_region)

    # Aliases must not register the same Migen module twice. Exclude the nested
    # CSR traversal so legacy register names keep a single owner.
    soc.autocsr_exclude = set(getattr(soc, "autocsr_exclude", ())) | {"wr_core"}
    for name in (
        "cd_wr", "cd_wr_sys", "wr_cpu", "wr_cpu_bridge", "wr_cpu_memory_cdc",
        "wrf_stream2wb", "wrf_wb2stream", "wb_slave_sys", "wb_slave_wr",
        "led_pps", "led_link", "led_act", "dac_refclk_load", "dac_refclk_data",
        "dac_dmtd_load", "dac_dmtd_data", "pps_in", "pps_out_valid", "pps_out",
        "pps_out_pulse", "tm_link_up", "tm_time_valid", "tm_seconds", "tm_cycles",
    ):
        if hasattr(core, name):
            object.__setattr__(soc, name, getattr(core, name))
            if hasattr(getattr(core, name), "get_csrs"):
                core.autocsr_exclude = set(getattr(core, "autocsr_exclude", ())) | {name}
    return core
