#!/usr/bin/env python3

#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import argparse
import os

from migen.genlib.cdc import MultiReg, PulseSynchronizer

from litex.gen import *
from litex.gen.genlib.misc import WaitTimer

from spec_a7_platform import *

from litex.build.generic_platform import *
from litex.build.io               import DifferentialInput, DifferentialOutput
from litex.build.openfpgaloader   import OpenFPGALoader

from litex.soc.interconnect.csr     import *
from litex.soc.interconnect         import stream
from litex.soc.interconnect         import wishbone

from litex.soc.integration.soc_core import *
from litex.soc.integration.builder  import *

from litex.soc.cores.clock          import S7PLL, S7MMCM
from litex.soc.cores.led            import LedChaser
from litex.soc.cores.spi.spi_master import SPIMaster
from litex.soc.cores.hyperbus       import HyperRAM
from litex.soc.integration.soc      import SoCRegion

from litepcie.phy.s7pciephy import S7PCIEPHY
from litepcie.software      import generate_litepcie_software_headers

from litescope import LiteScopeAnalyzer

from litex_wr_nic.gateware.uart              import UARTShared
from litex_wr_nic.gateware.soc               import LiteXWRNICSoC
from litex_wr_nic.gateware.time              import TimeGenerator
from litex_wr_nic.gateware.qpll              import SharedQPLL
from litex_wr_nic.gateware.ad5683r.core      import AD5683RDAC
from litex_wr_nic.gateware.wr_clock          import WRDACBackend
from litex_wr_nic.gateware.ad9516.core       import AD9516PLL, AD9516_MAIN_CONFIG, AD9516_EXT_CONFIG
from litex_wr_nic.gateware.measurement       import MultiClkMeasurement
from litex_wr_nic.gateware.delay.core        import MacroDelay, CoarseDelay, FineDelay
from litex_wr_nic.gateware.pps               import PPSGenerator
from litex_wr_nic.gateware.clk10m            import Clk10MGenerator
from litex_wr_nic.gateware.nic.phy           import LiteEthPHYWRGMII
from litex_wr_nic.gateware.wr_memory         import add_wr_cpu_memory, resolve_wr_boot
from litex_wr_nic.gateware.wr_cpu            import (
    WR_CPU_MEMORY_ORIGIN,
    WR_CPU_MEMORY_SIZE,
    WR_CPU_TYPES,
    validate_wr_cpu_config,
    wr_cpu_firmware_filename,
)
from litex_wr_nic.wr_boot import WR_BOOT_FLASH_OFFSET, WR_SDB_FLASH_OFFSET, validate_flash_layout

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq, with_white_rabbit=True):
        self.rst              = Signal()
        self.cd_sys           = ClockDomain()
        self.cd_refclk_pcie   = ClockDomain()
        self.cd_refclk_eth    = ClockDomain()
        self.cd_clk_125m_gtp  = ClockDomain()
        self.cd_clk_62m5_dmtd = ClockDomain()
        self.cd_clk10m_in     = ClockDomain()
        self.cd_clk62m5_in    = ClockDomain()

        # # #

        # Sys PLL (Free-Running from clk125).
        # ----------------------------------
        clk125m_oe = platform.request("clk125m_oe")
        clk125m    = platform.request("clk125m")
        self.comb += clk125m_oe.eq(1)

        self.pll = pll = S7PLL(speedgrade=-2)
        self.comb += pll.reset.eq(self.rst)
        pll.register_clkin(clk125m, 125e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq, margin=0)

        # RefClk Input (125MHz from 25MHz VCXO x 5 (AD9516)).
        # ---------------------------------------------------
        refclk125m_pads = platform.request("refclk125m")
        refclk125m_se   = Signal()
        self.specials += Instance("IBUFDS_GTE2",
            i_CEB = 0,
            i_I   = refclk125m_pads.p,
            i_IB  = refclk125m_pads.n,
            o_O   = refclk125m_se,
        )
        self.comb += self.cd_clk_125m_gtp.clk.eq(refclk125m_se)
        self.comb += self.cd_refclk_eth.clk.eq(refclk125m_se)

        # DMTD PLL (62.5MHz from VCXO).
        # -----------------------------
        clk62m5_dmtd_pads = platform.request("clk62m5_dmtd")
        clk62m5_dmtd = Signal()
        self.specials += Instance("BUFG",
            i_I = clk62m5_dmtd_pads,
            o_O = clk62m5_dmtd,
        )
        self.comb += self.cd_clk_62m5_dmtd.clk.eq(clk62m5_dmtd)

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
        white_rabbit_cpu_firmware  = None,
        white_rabbit_cpu_binary    = None,
        wr_cpu_type               = "urv",
        wr_cpu_variant            = None,
        wr_cpu_memory             = "private",
        wr_cpu_boot               = "auto",

        # Sync-In Parameters.
        # -------------------
        pps_in_macro_delay_default  = 62500000, # 16ns taps (Up to 2**32-1 taps).

        # Sync-Out Parameters.
        # --------------------
        bypass_pps_out_macro_coarse_delays = True,
        # PPS Out (Adjusted over JTAGBone with test/test_delay.py).
        pps_out_macro_delay_default  = 62499998, # 16ns taps (Up to 2**32-1 taps).
        pps_out_coarse_delay_default =        1, #  2ns taps (64 taps).
        pps_out_fine_delay_default   =      100, # 11ps taps (512 taps).

        # Clk10M Out (Adjusted over JTAGBone with test/test_delay.py).
        clk10m_out_macro_delay_default  = 6250000, # 16ns taps (Up to 2**32-1 taps).
        clk10m_out_coarse_delay_default =       1, #  2ns taps (64 taps).
        clk10m_out_fine_delay_default   =     200, # 11ps taps (512 taps).

        # Sync-In Parameters.
        # -------------------
        with_sync_in_pll = True,

        # RF-Out Parameters.
        # ------------------
        with_rf_out = True,
    ):
        # Platform ---------------------------------------------------------------------------------

        platform      = Platform(variant="xc7a50t")
        platform.name = "spec_a7_wr_nic"

        # Clocking ---------------------------------------------------------------------------------

        # General / WR.
        self.crg = _CRG(platform,
            sys_clk_freq      = sys_clk_freq,
            with_white_rabbit = with_white_rabbit,
        )

        # Shared QPLL.
        self.qpll = SharedQPLL(platform,
            with_pcie           = True, # Always True even when PCIe is disabled for correct WR Clocking.
            with_eth            = with_white_rabbit,
            eth_refclk_freq     = 125e6,
            eth_refclk_from_pll = False, # Use SPEC-A7 dedicated MGTREFCLK1, not GTGREFCLK.
        )
        self.qpll.enable_pll_refclk()

        # SoCMini ----------------------------------------------------------------------------------

        SoCMini.__init__(self, platform,
            clk_freq      = sys_clk_freq,
            ident         = "LiteX-WR-NIC on SPEC-A7.",
            ident_version = True,
        )

        # WR CPU Memory ---------------------------------------------------------------------------

        if wr_cpu_memory not in ("private", "integrated", "hyperram"):
            raise ValueError(f"Unsupported WR CPU memory mode: {wr_cpu_memory}")
        wr_cpu_variant = validate_wr_cpu_config(
            wr_cpu_type, wr_cpu_variant, wr_cpu_memory)
        if not with_white_rabbit and wr_cpu_memory != "private":
            raise ValueError("External WR CPU memory requires White Rabbit support.")
        if white_rabbit_cpu_firmware is None:
            white_rabbit_cpu_firmware = os.path.join("litex_wr_nic", "firmware",
                wr_cpu_firmware_filename(wr_cpu_type, "bram"))
        if white_rabbit_cpu_binary is None:
            white_rabbit_cpu_binary = os.path.join("litex_wr_nic", "firmware",
                wr_cpu_firmware_filename(wr_cpu_type, "bin"))

        wr_cpu_boot   = resolve_wr_boot(wr_cpu_memory, wr_cpu_boot)
        wr_cpu_region = None
        if wr_cpu_memory == "hyperram":
            wr_cpu_region = SoCRegion(origin=WR_CPU_MEMORY_ORIGIN, size=WR_CPU_MEMORY_SIZE, mode="rwx")
            wr_cpu_bus    = wishbone.Interface(data_width=32, address_width=32, addressing="word")
            self.bus.add_slave(name="wr_cpu_mem", slave=wr_cpu_bus, region=wr_cpu_region)
            self.wr_cpu_cache = FullMemoryWE()(wishbone.Cache(
                cachesize = (8*KILOBYTE)//4,
                master    = wr_cpu_bus,
                slave     = wishbone.Interface(data_width=32, address_width=32, addressing="word"),
            ))
            self.hyperram = HyperRAM(
                pads         = platform.request("hyperram"),
                latency      = 7,
                latency_mode = "variable",
                sys_clk_freq = sys_clk_freq,
                # 4:1 generates the 31.25 MHz HyperRAM clock from the 125 MHz
                # system clock and avoids introducing a timing-critical 250 MHz
                # FPGA domain on the Artix-7.
                clk_ratio    = "4:1",
            )
            self.comb += self.wr_cpu_cache.slave.connect(self.hyperram.bus)
            self.add_config("WR_CPU_CACHE_SIZE", 8*KILOBYTE)
        wr_memory = add_wr_cpu_memory(self,
            cpu_type     = wr_cpu_type,
            memory       = "region" if wr_cpu_memory == "hyperram" else wr_cpu_memory,
            boot         = wr_cpu_boot,
            firmware     = white_rabbit_cpu_binary,
            region       = wr_cpu_region,
            sys_clk_freq = sys_clk_freq,
        )
        wr_cpu_region = wr_memory["cpu_memory_region"]
        wr_cpu_ready  = wr_memory["cpu_memory_ready"]
        wr_cpu_loader = wr_memory["cpu_boot_loader"]

        # UART -------------------------------------------------------------------------------------

        self.uart = UARTShared(pads=platform.request("serial"), sys_clk_freq=sys_clk_freq)

        # JTAGBone ---------------------------------------------------------------------------------

        self.add_jtagbone()
        platform.add_period_constraint(self.jtagbone_phy.cd_jtag.clk, 1e9/20e6)
        platform.add_false_path_constraints(self.jtagbone_phy.cd_jtag.clk, self.crg.cd_sys.clk)

        # PCIe PHY ---------------------------------------------------------------------------------

        if with_pcie:
            self.pcie_phy = S7PCIEPHY(platform, platform.request("pcie_x1"),
                data_width                = 64,
                bar0_size                 = 0x20000,
                with_ptm                  = True,
                refclk_freq               = 100e6,
                pclk_mux_direct_from_mmcm = True,
            )
            self.pcie_phy.update_config({
                "Base_Class_Menu"          : "Network_controller",
                "Sub_Class_Interface_Menu" : "Ethernet_controller",
                "Class_Code_Base"          : "02",
                "Class_Code_Sub"           : "00",
            })
            self.comb += ClockSignal("refclk_pcie").eq(self.pcie_phy.pcie_refclk)
            self.pcie_phy.use_external_qpll(qpll_channel=self.qpll.get_channel("pcie"))
            platform.add_period_constraint(self.crg.cd_sys.clk, 1e9/sys_clk_freq)
            platform.toolchain.pre_placement_commands.append("reset_property LOC [get_cells -hierarchical -filter {{NAME=~pcie_s7/*gtp_channel.gtpe2_channel_i}}]")
            platform.toolchain.pre_placement_commands.append("set_property LOC GTPE2_CHANNEL_X0Y0 [get_cells -hierarchical -filter {{NAME=~pcie_s7/*gtp_channel.gtpe2_channel_i}}]")

            # PCIe <-> Sys-Clk false paths.
            platform.toolchain.pre_placement_commands.append(
                "set_false_path -quiet -from [get_clocks -quiet {{*s7pciephy_clkout*}}] -to [get_clocks -quiet sys_clk]")
            platform.toolchain.pre_placement_commands.append(
                "set_false_path -quiet -from [get_clocks -quiet sys_clk] -to [get_clocks -quiet {{*s7pciephy_clkout*}}]")
            platform.toolchain.pre_placement_commands.append(
                "set_false_path -quiet -from [get_clocks -quiet {{*s7pciephy_clkout0}}] -to [get_clocks -quiet {{*s7pciephy_clkout1}}]")
            platform.toolchain.pre_placement_commands.append(
                "set_false_path -quiet -from [get_clocks -quiet {{*s7pciephy_clkout1}}] -to [get_clocks -quiet {{*s7pciephy_clkout0}}]")

        # White Rabbit -----------------------------------------------------------------------------

        if with_white_rabbit:
            # White Rabbit Core.
            # ------------------
            self.add_wr_core(
                # CPU.
                cpu_firmware      = white_rabbit_cpu_firmware,
                cpu_type          = wr_cpu_type,
                cpu_variant       = wr_cpu_variant,
                cpu_memory_region = wr_cpu_region,
                cpu_memory_ready  = wr_cpu_ready,
                cpu_boot_loader   = wr_cpu_loader,

                # Board name.
                board_name       = "SPA7",

                # SFP.
                sfp_pads         = platform.request("sfp",     white_rabbit_sfp_connector),
                sfp_i2c_pads     = platform.request("sfp_i2c", white_rabbit_sfp_connector),
                sfp_tx_polarity  = 0, # Not Inverted.
                sfp_rx_polarity  = 0, # Not Inverted.
                sfp_disable_pads = platform.request("sfp_disable", white_rabbit_sfp_connector),
                sfp_fault_pads   = platform.request("sfp_fault",   white_rabbit_sfp_connector),
                sfp_los_pads     = platform.request("sfp_los",     white_rabbit_sfp_connector),
                sfp_det_pads     = platform.request("sfp_det",     white_rabbit_sfp_connector),

                # QPLL.
                qpll             = self.qpll,
                with_ext_clk     = True,

                # Serial.
                serial_pads      = self.uart.shared_pads,

                # Flash.
                flash_pads       = platform.request("flash"),

                # Temp 1Wire.
                temp_1wire_pads  = platform.request("temp_1wire"),
            )

            # Pads.
            # -----
            dac_refclk_pads  = platform.request("dac_refclk")
            dac_dmtd_pads    = platform.request("dac_dmtd")
            clk10m_in_pads   = platform.request("clk10m_in")
            clk62m5_in_pads  = platform.request("clk62m5_in")
            pps_in_pads      = platform.request("pps_in")

            # White Rabbit RefClk AD9516 PLL Driver.
            # --------------------------------------
            self.refclk_pll = AD9516PLL(
                platform   = platform,
                pads       = platform.request("pll"),
                config     = AD9516_MAIN_CONFIG,
                name       = "main",
                clk_domain = "sys",
            )

            # White Rabbit RefClk / DMTD DAC Drivers.
            # ---------------------------------------

            # RefClk DAC.
            self.refclk_tuning = WRDACBackend(cd="wr_sys")
            self.comb += self.wr_core.refclk_tuning.connect(self.refclk_tuning.command)
            self.refclk_dac = AD5683RDAC(platform,
                pads  = dac_refclk_pads,
                load  = self.refclk_tuning.load,
                value = self.refclk_tuning.value,
                gain  = 2, # 2 for 0-3V range to be able to accelerate enough RefClk, not working with 1.
                clk_domain = "wr_sys",
            )

            # DMTD DAC.
            self.dmtd_tuning = WRDACBackend(cd="wr_sys")
            self.comb += self.wr_core.dmtd_tuning.connect(self.dmtd_tuning.command)
            self.dmtd_dac = AD5683RDAC(platform,
                pads  = dac_dmtd_pads,
                load  = self.dmtd_tuning.load,
                value = self.dmtd_tuning.value,
                gain  = 2, # Preserve the effective gain used before g_enable_x2_gain was connected.
                clk_domain = "wr_sys",
            )

            # White Rabbit Clk-In.
            # --------------------

            # White Rabbit Sync-In AD9516 PLL Driver.
            if with_sync_in_pll:
                self.sync_in_pll = AD9516PLL(
                    platform   = platform,
                    pads       = platform.request("sync_in_pll"),
                    config     = AD9516_EXT_CONFIG,
                    name       = "sync_in",
                    clk_domain = "sys",
                )

            # Clk10m In Logic.
            clk10m_in = Signal()
            self.specials += DifferentialInput(
                i_p = clk10m_in_pads.p,
                i_n = clk10m_in_pads.n,
                o   = clk10m_in,
            )
            self.specials += Instance("BUFG",
                i_I = clk10m_in,
                o_O = self.crg.cd_clk10m_in.clk,
            )

            # Clk62m5 In Logic.
            clk62m5_in = Signal()
            self.specials += DifferentialInput(
                i_p = clk62m5_in_pads.p,
                i_n = clk62m5_in_pads.n,
                o   = clk62m5_in,
            )
            self.specials += Instance("BUFG",
                i_I = clk62m5_in,
                o_O = self.crg.cd_clk62m5_in.clk,
            )

            # White Rabbit PPS-In and Macro Delay.
            # ------------------------------------
            pps_in             = Signal()
            pps_in_d           = Signal()
            pps_in_pulse       = Signal()
            pps_in_macro_delay = Signal()

            self.comb += platform.request("pps_in_term_en").eq(1) # CHECKME: Make it configurable?

            self.specials += MultiReg(pps_in_pads, pps_in, odomain="wr")
            self.sync.wr += pps_in_d.eq(pps_in)
            self.comb += pps_in_pulse.eq(pps_in & ~pps_in_d)

            self.pps_in_macro_delay = MacroDelay(
                pulse_i = pps_in_pulse,
                pulse_o = pps_in_macro_delay,
                clk_domain    = "wr",
                default_delay = pps_in_macro_delay_default,
            )
            self.comb += self.pps_in.eq(pps_in_macro_delay)

            # Timings Constraints.
            # --------------------
            platform.add_platform_command("create_clock -name wr_txoutclk -period 16.000 [get_pins -hierarchical *gtpe2_i/TXOUTCLK]")
            platform.add_platform_command("create_clock -name wr_rxoutclk -period 16.000 [get_pins -hierarchical *gtpe2_i/RXOUTCLK]")
            platform.toolchain.pre_placement_commands.append("set_false_path -from [get_clocks clk_sys] -to [get_clocks clk10m_in_p]")
            platform.toolchain.pre_placement_commands.append("set_false_path -from [get_clocks clk10m_in_p] -to [get_clocks clk_sys]")
            platform.toolchain.pre_placement_commands.append("set_false_path -quiet -to [get_pins -hierarchical -filter {{NAME =~ *U_Sampler/*clk_i_d0_reg/D}}]")

            # White Rabbit Ethernet PHY (over White Rabbit Fabric) ---------------------------------

            self.ethphy0 = LiteEthPHYWRGMII(
                wrf_stream2wb = self.wrf_stream2wb,
                wrf_wb2stream = self.wrf_wb2stream,
            )

            # White Rabbit Sync-Out ----------------------------------------------------------------

            # PPS Free-Running.
            # -----------------
            self.pps_freerun = ClockDomainsRenamer("wr")(WaitTimer(int(62.5e6 - 1)))
            self.comb += self.pps_freerun.wait.eq(~self.pps_freerun.done)

            # PPS Out Valid.
            # --------------
            # PPS is considered inactive if no PPS pulse from WR for 2s.
            pps_out_valid        = Signal()
            pps_out_active_timer = WaitTimer(2.0*62.5e6)
            pps_out_active_timer = ClockDomainsRenamer("wr")(pps_out_active_timer)
            self.submodules += pps_out_active_timer
            self.comb += [
                pps_out_active_timer.wait.eq(~self.pps_out_pulse),
                pps_out_valid.eq(~pps_out_active_timer.done),
            ]

            # PPS WR / Free-Running Selection.
            # --------------------------------
            pps_out_pulse_sel = Signal()
            self.comb += [
                # Use PPS from WR when active.
                If(pps_out_valid,
                    pps_out_pulse_sel.eq(self.pps_out_pulse)
                # Else Switch back to Free-Running PPS.
                ).Else(
                    pps_out_pulse_sel.eq(self.pps_freerun.done)
                )
            ]

            # Sync-Out PLL.
            # -------------
            self.cd_wr_syncout   = ClockDomain()
            self.cd_wr_syncout8x = ClockDomain()
            self.syncout_pll = syncout_pll = S7MMCM(speedgrade=-2)
            self.comb += syncout_pll.reset.eq(ResetSignal("wr"))
            syncout_pll.register_clkin(ClockSignal("wr"), 62.5e6)
            syncout_pll.create_clkout(self.cd_wr_syncout,   62.5e6, margin=0, phase=0)
            syncout_pll.create_clkout(self.cd_wr_syncout8x, 500e6,  margin=0, phase=0)

            # Clk10M SMA Out.
            # ---------------

            # Clk10M Macro Delay.
            clk10m_out_macro_delay  = Signal()
            self.clk10m_macro_delay = MacroDelay(
                pulse_i = pps_out_pulse_sel,
                pulse_o = clk10m_out_macro_delay,
                clk_domain    = "wr",
                default_delay = clk10m_out_macro_delay_default,
            )
            clk10m_out_macro_delay_sync = PulseSynchronizer("wr", "wr_syncout")
            self.submodules += clk10m_out_macro_delay_sync
            self.comb += clk10m_out_macro_delay_sync.i.eq(clk10m_out_macro_delay)

            # Clk10M Generator.
            clk10m_out_gen  = Signal(8)
            self.clk10m_gen = Clk10MGenerator(
                pulse_i  = clk10m_out_macro_delay_sync.o,
                clk10m_o = clk10m_out_gen,
                clk_domain = "wr_syncout",
            )

            # Clk10M Coarse Delay.
            clk10m_out_coarse_delay      = Signal()
            self.clk10m_out_coarse_delay = CoarseDelay(
                rst = ~syncout_pll.locked,
                i   = clk10m_out_gen,
                o   = clk10m_out_coarse_delay,
                clk_domain = "wr_syncout",
                clk_cycles = 8, # 64-taps.
                default_delay = clk10m_out_coarse_delay_default,
            )

            # Clk10M Out.
            clk10m_out_pads = platform.request("clk10m_out")
            self.specials += DifferentialOutput(
                i   = clk10m_out_coarse_delay,
                o_p = clk10m_out_pads.p,
                o_n = clk10m_out_pads.n,
            )

            # PPS SMA Out.
            # ------------

            pps_out_led_timer = ClockDomainsRenamer("wr")(WaitTimer(int(0.1*62.5e6)))
            self.submodules += pps_out_led_timer
            pps_out_led = Signal()
            self.sync.wr += [
                If(pps_out_pulse_sel,
                    pps_out_led.eq(1)
                ).Elif(pps_out_led_timer.done,
                    pps_out_led.eq(0)
                )
            ]
            self.comb += pps_out_led_timer.wait.eq(pps_out_led)

            # PPS from WR Core + configurable Macro/Coarse delays.
            if not bypass_pps_out_macro_coarse_delays:

                # PPS Macro Delay.
                pps_out_macro_delay      = Signal()
                self.pps_out_macro_delay = MacroDelay(
                    pulse_i = pps_out_pulse_sel,
                    pulse_o = pps_out_macro_delay,
                    clk_domain    = "wr",
                    default_delay = pps_out_macro_delay_default,
                )

                # PPS Generator.
                pps_out_gen      = Signal()
                self.pps_out_gen = PPSGenerator(
                    i = pps_out_macro_delay,
                    o = pps_out_gen,
                    clk_domain = "wr",
                    clk_freq   = int(62.5e6),
                    duty_cycle = 20/100, # 20% High / 80% Low PPS.
                )

                # PPS Coarse Delay.
                pps_out_coarse_delay = Signal(attr={("IOB", "TRUE")})
                self.pps_out_coarse_delay = CoarseDelay(
                    rst = ~syncout_pll.locked,
                    i   = pps_out_gen,
                    o   = pps_out_coarse_delay,
                    clk_domain = "wr",
                    clk_cycles = 8, # 64-taps.
                    default_delay = pps_out_coarse_delay_default,
                )
                pps_out_sma = pps_out_coarse_delay

            # PPS from WR Core.
            else:
                pps_out_sma = self.pps_out

            # PPS Out.
            pps_out_pads = platform.request("pps_out")
            self.specials += DifferentialOutput(
                i   = pps_out_sma,
                o_p = pps_out_pads.p,
                o_n = pps_out_pads.n,
            )

            # Fine Delay (Clk10M & PPS Out).
            # ------------------------------
            self.fine_delay = FineDelay(
                pads           = platform.request("fine_delay"),
                sys_clk_freq   = sys_clk_freq,
                default_delays = [
                    clk10m_out_fine_delay_default,
                    pps_out_fine_delay_default,
                ]
            )

            # FrontPanel Leds.
            # ----------------
            self.comb += [
                platform.request("clk10m_out_led").eq(1),
                platform.request("pps_out_led").eq(pps_out_led),
                platform.request("act_out_led").eq(self.led_link & ~self.led_act)
            ]

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
                self.time_generator.time_sync.eq(self.pps_out_pulse),
                self.time_generator.time_seconds.eq(self.tm_seconds),
            ]

            # Connect TimeGenerator's Time to PCIe PTM.
            self.comb += [
                self.ptm_requester.time_clk.eq(ClockSignal("wr")),
                self.ptm_requester.time_rst.eq(ResetSignal("wr")),
                self.ptm_requester.time.eq(self.time_generator.time)
            ]

        # RF Out (LMX2572) -------------------------------------------------------------------------
        # CHECKME: Connect SYNC if useful.

        if with_rf_out:
            rf_out_pll_pads = platform.request("rf_out_pll")
            rf_out_pll_pads.miso = Signal()
            self.rf_out_pll = SPIMaster(
                pads         = rf_out_pll_pads,
                data_width   = 24,
                sys_clk_freq = sys_clk_freq,
                spi_clk_freq = 5e6,
                mode         = "aligned",
            )

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
            asynchronous_clk_domains += [self.fine_delay.cd_fine_delay.clk]
            # Host/WR register and fabric traffic crosses asynchronous FIFOs.
            platform.add_false_path_constraints(self.crg.cd_sys.clk, self.cd_wr_sys.clk)

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
    parser = argparse.ArgumentParser(description="LiteX-WR-NIC on SPEC-A7.")

    # Build/Load/Flash Arguments.
    # ---------------------------
    parser.add_argument("--build", action="store_true", help="Build bitstream.")
    parser.add_argument("--load",  action="store_true", help="Load bitstream.")
    parser.add_argument("--flash", action="store_true", help="Flash bitstream.")
    parser.add_argument("--wr-cpu-memory", default="private",
        choices=["private", "integrated", "hyperram"],
        help="WR CPU memory implementation (default: private).")
    parser.add_argument("--wr-cpu-boot", default="auto", choices=["auto", "embedded", "spi", "host"],
        help="WR firmware source (auto: embedded for BRAM, SPI for HyperRAM).")
    parser.add_argument("--wr-cpu-type", default="urv",
        choices=WR_CPU_TYPES,
        help="WR CPU implementation (default: embedded uRV).")
    parser.add_argument("--wr-cpu-variant", default=None,
        help="LiteX WR CPU variant (VexRiscv defaults to lite).")
    parser.add_argument("--output-dir", default=None,
        help="Build directory (useful for resource comparisons).")
    parser.add_argument("--skip-firmware-build", action="store_true",
        help="Reuse existing WR firmware when building gateware.")
    parser.add_argument("--skip-software-headers", action="store_true",
        help=argparse.SUPPRESS)

    # Probes.
    # -------
    parser.add_argument("--with-wishbone-fabric-interface-probe", action="store_true")
    parser.add_argument("--with-wishbone-slave-probe",            action="store_true")
    parser.add_argument("--with-dac-vcxo-probe",                  action="store_true")
    parser.add_argument("--with-time-pps-probe",                  action="store_true")

    args = parser.parse_args()

    # Build Firmware.
    # ---------------
    if args.build and not args.skip_firmware_build:
        print("Building firmware...")
        r = os.system("cd litex_wr_nic/firmware && ./build.py --wr-cpu-type {}".format(
            args.wr_cpu_type))
        if r != 0:
            raise RuntimeError("Firmware build failed.")

    # Build SoC/Gateware (with integrated Firmware).
    # ----------------------------------------------
    soc = BaseSoC(
        wr_cpu_type    = args.wr_cpu_type,
        wr_cpu_variant = args.wr_cpu_variant,
        wr_cpu_memory  = args.wr_cpu_memory,
        wr_cpu_boot    = args.wr_cpu_boot,
    )
    if args.with_wishbone_fabric_interface_probe:
        soc.add_wishbone_fabric_interface_probe()
    if args.with_wishbone_slave_probe:
        soc.add_wishbone_slave_probe()
    if args.with_dac_vcxo_probe:
        soc.add_dac_vcxo_probe()
    if args.with_time_pps_probe:
        soc.add_time_pps_probe()
    builder = Builder(soc, csr_csv="test/csr.csv", **(
        {} if args.output_dir is None else {"output_dir": args.output_dir}))
    builder.build(
        run=args.build,
        vivado_place_directive="Explore",
        vivado_route_directive="Explore",
        vivado_post_route_phys_opt_directive="Explore",
    )

    # Generate PCIe C Headers.
    # ------------------------
    if not args.skip_software_headers:
        generate_litepcie_software_headers(soc, "litex_wr_nic/software/kernel")

    # Generate Bitstream.
    # -------------------
    if args.load or args.flash:
        os.system("python3 litex_wr_nic/gateware/xilinx-bitstream.py {bit_file} {bin_file}".format(
            bit_file = builder.get_bitstream_filename(mode="sram"),
            bin_file = builder.get_bitstream_filename(mode="flash"),
        ))

    # Load FPGA.
    # ----------
    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="flash"))

    # Flash FPGA.
    # -----------
    if args.flash:
        bitstream = builder.get_bitstream_filename(mode="flash")
        sdb_image = "litex_wr_nic/firmware/sdb-wrpc.bin"
        boot_image = (
            os.path.join("litex_wr_nic", "firmware", wr_cpu_firmware_filename(args.wr_cpu_type, "boot"))
            if resolve_wr_boot(args.wr_cpu_memory, args.wr_cpu_boot) == "spi" else None
        )
        validate_flash_layout(bitstream, sdb_image, boot_image)
        prog = soc.platform.create_programmer()
        prog.flash(0x0000_0000, bitstream)
        prog.flash(WR_SDB_FLASH_OFFSET, sdb_image)
        if resolve_wr_boot(args.wr_cpu_memory, args.wr_cpu_boot) == "spi":
            prog.flash(WR_BOOT_FLASH_OFFSET, boot_image)

if __name__ == "__main__":
    main()
