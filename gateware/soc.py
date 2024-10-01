#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import os
import sys

from migen import *

from litex.build import tools

from litex.gen import *

from litex.soc.interconnect import wishbone

from litex.soc.integration.soc_core import *

from gateware.nic import sram
sys.modules["liteeth.mac.sram"] = sram #  Replace Liteeth SRAM with our custom implementation.
from gateware.nic.dma import LitePCIe2WishboneDMA

from gateware.wr_common import wr_core_init, wr_core_files

# LiteX WR NIC SoC ---------------------------------------------------------------------------------

class LiteXWRNICSoC(SoCMini):
    SoCMini.csr_map = {
        "ethmac"           : 1,
        "ethphy"           : 2,
        "identifier_mem"   : 3,
        "leds"             : 4,

        # PCIe.
        "pcie_endpoint"    : 5,
        "pcie_pcie2wb_dma" : 6,
        "pcie_wb2pcie_dma" : 7,
        "pcie_msi"         : 8,
        "pcie_phy"         : 9,
    }

    def add_pcie_nic(self, pcie_phy=None, eth_phy=None, ntxslots=4, nrxslots=4, with_timing_constraints=True):
        # Ethernet MAC.
        # -------------
        self.add_ethernet(
            name                    = "ethmac",
            phy                     = eth_phy,
            phy_cd                  = "eth",
            data_width              = 64,
            nrxslots                = ntxslots,
            ntxslots                = nrxslots,
            with_timing_constraints = with_timing_constraints,
        )
        ethmac = self.ethmac
        del self.bus.slaves["ethmac_tx"] # Remove from SoC bus since directly connected to PCIe.
        del self.bus.slaves["ethmac_rx"] # Remove from SoC bus since directly connected to PCIe.

        # PCIe Core.
        # ----------
        self.add_pcie(name="pcie", phy=pcie_phy,
            ndmas                = 1,
            max_pending_requests = 4,
            data_width           = 64,
            with_dma_buffering   = False,
            with_dma_loopback    = False,
            with_dma_table       = False,
            with_ptm             = False,
            with_msi             = True,
            msis                 = {
                "ETHMAC_RX" : ethmac.interface.sram.rx_pcie_irq,
                "ETHMAC_TX" : ethmac.interface.sram.tx_pcie_irq,
            },
        )

        # RX Datapath: Ethernet (RX) -> PCIe -> Host.
        # -------------------------------------------
        align_bits = log2_int(512)
        self.pcie_wb2pcie_dma = pcie_wb2pcie_dma = LitePCIe2WishboneDMA(
            endpoint   = self.pcie_endpoint,
            dma        = self.pcie_dma0.writer,
            data_width = 64,
            mode       = "wb2pcie",
        )
        self.comb += [
            pcie_wb2pcie_dma.desc.valid.eq(ethmac.interface.sram.writer.start),
            ethmac.interface.sram.writer.ready.eq(pcie_wb2pcie_dma.desc.ready),
            pcie_wb2pcie_dma.desc.bus_addr.eq(ethmac.interface.sram.writer.stat_fifo.source.slot * ethmac.slot_size.constant),
            pcie_wb2pcie_dma.desc.host_addr.eq(ethmac.interface.sram.writer.pcie_host_addr),
            pcie_wb2pcie_dma.desc.length[align_bits:].eq(ethmac.interface.sram.writer.stat_fifo.source.length[align_bits:] + 1),
        ]
        self.bus_interconnect_rx = wishbone.InterconnectPointToPoint(
            master = pcie_wb2pcie_dma.bus,
            slave  = ethmac.bus_rx,
        )

        # TX Datapath: Host -> PCIe -> Ethernet (TX).
        # -------------------------------------------
        align_bits = log2_int(512)
        self.pcie_pcie2wb_dma = pcie_pcie2wb_dma = LitePCIe2WishboneDMA(
            endpoint   = self.pcie_endpoint,
            dma        = self.pcie_dma0.reader,
            data_width = 64,
            mode       = "pcie2wb",
        )
        self.comb += [
            pcie_pcie2wb_dma.desc.valid.eq(ethmac.interface.sram.reader.start),
            ethmac.interface.sram.reader.ready.eq(pcie_pcie2wb_dma.desc.ready),
            pcie_pcie2wb_dma.desc.bus_addr.eq(ethmac.interface.sram.reader.cmd_fifo.source.slot * ethmac.slot_size.constant),
            pcie_pcie2wb_dma.desc.host_addr.eq(ethmac.interface.sram.reader.pcie_host_addr),
            pcie_pcie2wb_dma.desc.length[align_bits:].eq(ethmac.interface.sram.reader.cmd_fifo.source.length[align_bits:] + 1),
        ]
        self.bus_interconnect_tx = wishbone.InterconnectPointToPoint(
            master = pcie_pcie2wb_dma.bus,
            slave  = ethmac.bus_tx,
        )

    def add_sources(self):
        if not os.path.exists("wr-cores"):
            wr_core_init()
        for file in wr_core_files:
            self.platform.add_source(file)

    def add_ext_ram(self, platform):
        # CHECKME: Check if the best approach, we could also completely replace uRV and provide
        #          a similar instance?
        # CHECKME: When working, try to also play with drive to see if uRV core is handling it
        #          correctly.
        # CHECKME: When working with valid, try to replace with a Wishbone interface to allow
        #          connecting it to a SPIFlash core or HyperRAM.

        # External ROM.
        # -------------
        rom_init = get_mem_data("firmware/wrpc-sw/wrc.bin",
            data_width = 32,
            endianness = "little"
        )
        rom      = Memory(32, depth=131072//4, init=rom_init)
        rom_port = rom.get_port()
        self.specials += rom, rom_port

        ext_ram_adr   = Signal(32) # /!\ Fake, will be re-connected post-synthesis /!\.
        ext_ram_dat_r = Signal(32) # /!\ Fake, will be re-connected post-synthesis /!\.
        self.specials += Instance("ext_ram_tap",
            i_ext_ram_i_adr   = ext_ram_adr,
            o_ext_ram_i_dat_r = ext_ram_dat_r,
            o_ext_ram_o_adr   = Cat(Signal(2), rom_port.adr),
            i_ext_ram_o_dat_r = rom_port.dat_r,
        )
        platform.add_source("gateware/ext_ram_tap.v")

        # Connect CPU Adr -> Ext ROM Adr.
        # ---------------------------
        ext_ram_connections_adr = []
        for n in range(32):
            ext_ram_connections_adr.append((
                f"xwrc_board_artix7_wrapper/u_xwrc_board_artix7/cmp_board_common/cmp_xwr_core/WRPC/U_CPU/im_addr[{n}]", # Src.
                f"ext_ram_tap/ext_ram_i_adr[{n}]",                                                                      # Dst.
            ))
        for _from, _to in ext_ram_connections_adr:
            # Find Src Driver.
            #platform.toolchain.pre_optimize_commands.append(f"set_property DONT_TOUCH false [get_nets {_from}]")
            platform.toolchain.pre_optimize_commands.append(f"set pin_driver_from [get_pins -of_objects [get_nets {_from}] -filter {{{{DIRECTION == OUT}}}}]")
            platform.toolchain.pre_optimize_commands.append(f"disconnect_net -objects $pin_driver_from")

            # Find Dst Driver and disconnect it.
            platform.toolchain.pre_optimize_commands.append(f"set_property DONT_TOUCH false [get_nets {_to}]")
            platform.toolchain.pre_optimize_commands.append(f"set pin_driver_to [get_pins -of_objects [get_nets {_to}] -filter {{{{DIRECTION == IN}}}}]")
            platform.toolchain.pre_optimize_commands.append(f"disconnect_net -objects $pin_driver_to")

            # Connect Src to Dst.
            platform.toolchain.pre_optimize_commands.append(f"connect_net -hier -net $pin_driver_from -objects $pin_driver_to")


        # Connect Ext ROM Dat -> CPU Dat.
        # -------------------------------
        ext_ram_connections_dat = []
        for n in range(32):
            ext_ram_connections_dat.append((
                f"ext_ram_tap/ext_ram_i_dat_r[{n}]",                                                                    # Src.
                f"xwrc_board_artix7_wrapper/u_xwrc_board_artix7/cmp_board_common/cmp_xwr_core/WRPC/U_CPU/im_data[{n}]", # Dst.
            ))
        for _from, _to in ext_ram_connections_dat:
            # Find Src Driver and disconnect it.
            platform.toolchain.pre_optimize_commands.append(f"set_property DONT_TOUCH false [get_nets {_from}]")
            platform.toolchain.pre_optimize_commands.append(f"set pin_driver_from [get_pins -of_objects [get_nets {_from}] -filter {{{{DIRECTION == OUT}}}}]")
            platform.toolchain.pre_optimize_commands.append(f"disconnect_net -objects $pin_driver_from")

            # Find Dst Driver and disconnect it.
            #platform.toolchain.pre_optimize_commands.append(f"set_property DONT_TOUCH false [get_nets {_to}]")
            platform.toolchain.pre_optimize_commands.append(f"set pin_driver_to [get_pins -of_objects [get_nets {_to}] -filter {{{{DIRECTION == IN}}}}]")
            platform.toolchain.pre_optimize_commands.append(f"disconnect_net -objects $pin_driver_to")

            # Connect Src to Dst.
            platform.toolchain.pre_optimize_commands.append(f"connect_net -hier -net $pin_driver_from -objects $pin_driver_to")
