#!/usr/bin/env python3

#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2024 Gwenhael Goavec-Merou <gwenhael@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

# Build/use
# Build/Load bitstream:
# ./xilinx_zc7006.py --with-etherbone --uart-name=crossover --csr-csv=csr.csv --build --load
#
# Test Ethernet:
# ping 192.168.1.50
#
# Test Console:
# litex_server --udp
# litex_term crossover
#
#
# Build/Load bitstream:
# ./xilinx_zc706.py --with-jtagbone --uart-name=crossover --csr-csv=csr.csv --build --load
#
# litex_server --jtag --jtag-config openocd_xc7z_smt2-nc.cfg
#
# In a second terminal:
# litex_cli --regs # to dump all registers
# Or
# litex_term crossover # to have access to LiteX bios
#
# --------------------------------------------------------------------------------------------------

from migen import *

from litex.gen import *

from litex_boards.platforms import xilinx_zc706

from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.cores.led import LedChaser

from litedram.modules import MT8JTF12864
from litedram.phy import s7ddrphy

from liteeth.phy.k7_1000basex import K7_1000BASEX

from litepcie.phy.s7pciephy import S7PCIEPHY
from litepcie.software import generate_litepcie_software

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.rst       = Signal()
        self.cd_sys    = ClockDomain()
        self.cd_sys4x  = ClockDomain()
        self.cd_idelay = ClockDomain()
        self.cd_eth    = ClockDomain()

        # # #

        # Clk/Rst.
        clk200 = platform.request("clk200")

        # PLL.
        self.pll = pll = S7PLL(speedgrade=-1)
        self.comb += pll.reset.eq(self.rst)
        pll.register_clkin(clk200, 200e6)
        pll.create_clkout(self.cd_sys,    sys_clk_freq)
        pll.create_clkout(self.cd_sys4x,  4*sys_clk_freq)
        pll.create_clkout(self.cd_idelay, 200e6)
        pll.create_clkout(self.cd_eth,    200e6)
        platform.add_false_path_constraints(self.cd_sys.clk, pll.clkin) # Ignore sys_clk to pll.clkin path created by SoC's rst.

        # IDelayCtrl.
        self.idelayctrl = S7IDELAYCTRL(self.cd_idelay)

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    def __init__(self, sys_clk_freq=125e6,
        eth_ip          = "192.168.1.50",
        remote_ip       = None,
        eth_dynamic_ip  = False,
        with_led_chaser = True,
        **kwargs):
        platform = xilinx_zc706.Platform()

        # When nor jtagbone, nor etherbone are set forces jtagbone.
        kwargs["uart_name"]     = "crossover"
        if not (kwargs["with_jtagbone"] or with_etherbone):
            kwargs["with_jtagbone"] = True

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq)

        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, sys_clk_freq, ident="LiteX SoC on ZC706", **kwargs)

        # Ethernet / Etherbone ---------------------------------------------------------------------
        self.ethphy = K7_1000BASEX(
            refclk_or_clk_pads = self.crg.cd_eth.clk,
            data_pads          = self.platform.request("sfp", 0),
            sys_clk_freq       = self.clk_freq,
            with_csr           = False
        )
        self.comb += self.platform.request("sfp_tx_disable_n", 0).eq(1)
        platform.add_platform_command("set_property SEVERITY {{Warning}} [get_drc_checks REQP-52]")

        # PCIe -------------------------------------------------------------------------------------
        self.pcie_phy = S7PCIEPHY(platform, platform.request("pcie_x4"),
            data_width = 128,
            bar0_size  = 0x20000)

        # PCIe + Ethernet --------------------------------------------------------------------------
        self.add_ethernet_pcie(phy=self.ethphy, pcie_phy=self.pcie_phy)

        # Leds -------------------------------------------------------------------------------------
        if with_led_chaser:
            self.leds = LedChaser(
                pads         = platform.request_all("user_led"),
                sys_clk_freq = sys_clk_freq
            )

    def add_ethernet_pcie(self, name="ethmac", phy=None, pcie_phy=None, phy_cd="eth", dynamic_ip=False,
                          software_debug=False,
                          nrxslots=32,
                          ntxslots=32,
                          with_timing_constraints=True,
                          max_pending_requests=8,
                          with_msi=True):
        # Imports
        from litex.soc.integration.soc import SoCBusHandler, SoCRegion, SoCIORegion
        from litex.soc.interconnect import wishbone

        from liteeth.mac import LiteEthMAC
        from liteeth.phy.model import LiteEthPHYModel
        data_width=128
        self.submodules.pcie_mem_bus_rx = SoCBusHandler(
            data_width=data_width
        )
        self.submodules.pcie_mem_bus_tx = SoCBusHandler(
            data_width=data_width
        )
        # MAC.
        self.check_if_exists(name)
        ethmac = LiteEthMAC(
            phy=phy,
            dw=data_width,
            interface="wishbone",
            endianness=self.cpu.endianness,
            nrxslots=nrxslots,
            ntxslots=ntxslots,
            timestamp=None,
            with_preamble_crc=not software_debug)
        self.add_constant("ETHMAC_RX_WAIT_OFFSET", ethmac.interface.wait_ack_offset)
        self.add_constant("ETHMAC_TX_READY_OFFSET", ethmac.interface.tx_ready_offset)
        # Use PHY's eth_tx/eth_rx clock domains.
        ethmac = ClockDomainsRenamer({
            "eth_tx": phy_cd + "_tx",
            "eth_rx": phy_cd + "_rx"})(ethmac)
        setattr(self.submodules, name, ethmac)
        # Compute Regions size and add it to the SoC.
        ethmac_region_rx = SoCRegion(origin=0, size=ethmac.rx_slots.constant * ethmac.slot_size.constant, cached=False)
        ethmac_region_tx = SoCRegion(origin=0, size=ethmac.tx_slots.constant * ethmac.slot_size.constant, cached=False)
        self.pcie_mem_bus_rx.add_region(name="io",region=SoCIORegion(0x00000000,0x100000000))
        self.pcie_mem_bus_tx.add_region(name="io",region=SoCIORegion(0x00000000,0x100000000))
        self.pcie_mem_bus_rx.add_slave(name='ethmac_rx', slave=ethmac.rx_bus, region=ethmac_region_rx)
        self.pcie_mem_bus_tx.add_slave(name='ethmac_tx', slave=ethmac.tx_bus, region=ethmac_region_tx)

        # Timing constraints
        if with_timing_constraints:
            eth_rx_clk = getattr(phy, "crg", phy).cd_eth_rx.clk
            eth_tx_clk = getattr(phy, "crg", phy).cd_eth_tx.clk
            if not isinstance(phy, LiteEthPHYModel) and not getattr(phy, "model", False):
                self.platform.add_period_constraint(eth_rx_clk, 1e9 / phy.rx_clk_freq)
                self.platform.add_period_constraint(eth_tx_clk, 1e9 / phy.tx_clk_freq)
                self.platform.add_false_path_constraints(self.crg.cd_sys.clk, eth_rx_clk, eth_tx_clk)

        # PCIe

        from litedram.common import LiteDRAMNativePort
        from litedram.core import LiteDRAMCore
        from litedram.frontend.wishbone import LiteDRAMWishbone2Native
        from gateware.litepcie.wishbone_dma import LitePCIe2WishboneDMANative, LiteWishbone2PCIeDMANative, PCIeInterruptTest
        from litepcie.core import LitePCIeEndpoint, LitePCIeMSI
        from litepcie.frontend.dma import LitePCIeDMA
        from litepcie.frontend.wishbone import LitePCIeWishboneMaster

        name = "pcie"
        self.check_if_exists(f"{name}_endpoint")
        endpoint = LitePCIeEndpoint(pcie_phy, max_pending_requests=max_pending_requests, endianness=pcie_phy.endianness)
        setattr(self.submodules, f"{name}_endpoint", endpoint)

        # MMAP.
        self.check_if_exists(f"{name}_mmap")
        mmap = LitePCIeWishboneMaster(self.pcie_endpoint, base_address=self.mem_map["csr"])
        self.add_wb_master(mmap.wishbone)
        setattr(self.submodules, f"{name}_mmap", mmap)

        pcie_host_wb2pcie_dma = LiteWishbone2PCIeDMANative(endpoint, data_width)
        self.submodules.pcie_host_wb2pcie_dma = pcie_host_wb2pcie_dma
        self.pcie_mem_bus_rx.add_master("pcie_master_wb2pcie", pcie_host_wb2pcie_dma.bus_wr)
        pcie_host_pcie2wb_dma = LitePCIe2WishboneDMANative(endpoint, data_width)
        self.submodules.pcie_host_pcie2wb_dma = pcie_host_pcie2wb_dma
        self.pcie_mem_bus_tx.add_master("pcie_master_pcie2wb", pcie_host_pcie2wb_dma.bus_rd)

        align_bits = log2_int(512)
        self.comb += [
            pcie_host_wb2pcie_dma.bus_addr.eq(ethmac_region_rx.origin + ethmac.interface.sram.writer.stat_fifo.source.slot * ethmac.slot_size.constant),
            pcie_host_wb2pcie_dma.host_addr.eq(ethmac.interface.sram.writer.pcie_host_addr),
            pcie_host_wb2pcie_dma.length.eq(Cat(Signal(align_bits,reset=0), (ethmac.interface.sram.writer.stat_fifo.source.length[align_bits:] + 1))),
            pcie_host_wb2pcie_dma.start.eq(ethmac.interface.sram.writer.start_transfer),
            ethmac.interface.sram.writer.transfer_ready.eq(pcie_host_wb2pcie_dma.ready),
        ]
        self.comb += [
            pcie_host_pcie2wb_dma.bus_addr.eq(ethmac_region_tx.origin + ethmac.interface.sram.reader.cmd_fifo.source.slot * ethmac.slot_size.constant),
            pcie_host_pcie2wb_dma.host_addr.eq(ethmac.interface.sram.reader.pcie_host_addr),
            pcie_host_pcie2wb_dma.length.eq(Cat(Signal(align_bits, reset=0), (ethmac.interface.sram.reader.cmd_fifo.source.length[align_bits:] + 1))),
            pcie_host_pcie2wb_dma.start.eq(ethmac.interface.sram.reader.start_transfer),
            ethmac.interface.sram.reader.transfer_ready.eq(pcie_host_pcie2wb_dma.ready),
        ]

        self.submodules.bus_interconnect_tx = wishbone.InterconnectPointToPoint(
            master=next(iter(self.pcie_mem_bus_tx.masters.values())),
            slave=next(iter(self.pcie_mem_bus_tx.slaves.values())))

        self.submodules.bus_interconnect_rx = wishbone.InterconnectPointToPoint(
            master=next(iter(self.pcie_mem_bus_rx.masters.values())),
            slave=next(iter(self.pcie_mem_bus_rx.slaves.values())))

        from litepcie.core import LitePCIeMSI
        if with_msi:
            self.check_if_exists(f"{name}_msi")
            msi = LitePCIeMSI()
            setattr(self.submodules, f"{name}_msi", msi)
            self.comb += msi.source.connect(pcie_phy.msi)
            self.msis = {}

            self.msis["ETHRX"] = ethmac.rx_pcie_irq
            self.msis["ETHTX"] = ethmac.tx_pcie_irq

            for i, (k, v) in enumerate(sorted(self.msis.items())):
                self.comb += msi.irqs[i].eq(v)
                self.add_constant(k + "_INTERRUPT", i)

        # Timing constraints.
        self.platform.add_false_path_constraints(self.crg.cd_sys.clk, pcie_phy.cd_pcie.clk)

# Build --------------------------------------------------------------------------------------------
def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=xilinx_zc706.Platform, description="LiteX SoC on ZC706.")
    parser.add_target_argument("--sys-clk-freq",   default=125e6, type=float, help="System clock frequency.")
    parser.add_target_argument("--programmer",     default="vivado",          help="Programmer select from Vivado/openFPGALoader.")
    parser.add_target_argument("--eth-ip",         default="192.168.1.50",    help="Ethernet/Etherbone IP address.")
    parser.add_target_argument("--remote-ip",      default="192.168.1.100",   help="Remote IP address of TFTP server.")
    parser.add_target_argument("--eth-dynamic-ip", action="store_true",       help="Enable dynamic Ethernet IP addresses setting.")
    parser.add_target_argument("--with-pcie",      action="store_true",       help="Enable PCIe support.")
    parser.add_target_argument("--driver",         action="store_true",       help="Generate PCIe driver.")
    args = parser.parse_args()

    #args.driver        = True
    args.cpu_name      = "None"

    soc = BaseSoC(
        sys_clk_freq   = args.sys_clk_freq,
        eth_ip         = args.eth_ip,
        remote_ip      = args.remote_ip,
        eth_dynamic_ip = args.eth_dynamic_ip,
        **parser.soc_argdict
    )
    builder = Builder(soc, **parser.builder_argdict)
    if args.build:
        builder.build(**parser.toolchain_argdict)

    if args.driver:
        generate_litepcie_software(soc, os.path.join(builder.output_dir, "driver"))

    if args.load:
        prog = soc.platform.create_programmer(args.programmer)
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"), device=1)

if __name__ == "__main__":
    main()
