#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import sys

from migen import *

from litex.build import tools

from litex.gen import *

from litex.soc.interconnect import wishbone

from litex.soc.integration.soc_core import *

from gateware.nic import sram
sys.modules["liteeth.mac.sram"] = sram #  Replace Liteeth SRAM with our custom implementation.
from gateware.nic.dma import LitePCIe2WishboneDMA

# PCIe NIC SoC -------------------------------------------------------------------------------------

class PCIeNICSoC(SoCMini):
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

    def add_pcie_nic(self, pcie_phy=None, eth_phy=None, ntxslots=4, nrxslots=4):
        # Ethernet MAC.
        # -------------
        self.add_ethernet(
            name       = "ethmac",
            phy        = eth_phy,
            phy_cd     = "eth",
            data_width = 64,
            nrxslots   = ntxslots,
            ntxslots   = nrxslots,
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
                "ETHRX" : ethmac.interface.sram.rx_pcie_irq,
                "ETHTX" : ethmac.interface.sram.tx_pcie_irq,
            },
        )

        align_bits = log2_int(512)

        # RX Datapath: Ethernet (RX) -> PCIe -> Host.
        # -------------------------------------------
        self.pcie_wb2pcie_dma = pcie_wb2pcie_dma = LitePCIe2WishboneDMA(
            endpoint   = self.pcie_endpoint,
            dma        = self.pcie_dma0.writer,
            data_width = 64,
            mode       = "wb2pcie",
        )
        self.comb += [
            pcie_wb2pcie_dma.desc.bus_addr.eq(ethmac.interface.sram.writer.stat_fifo.source.slot * ethmac.slot_size.constant),
            pcie_wb2pcie_dma.desc.host_addr.eq(ethmac.interface.sram.writer.pcie_host_addr),
            pcie_wb2pcie_dma.desc.length[align_bits:].eq(ethmac.interface.sram.writer.stat_fifo.source.length[align_bits:] + 1),
            pcie_wb2pcie_dma.desc.valid.eq(ethmac.interface.sram.writer.start),
            ethmac.interface.sram.writer.ready.eq(pcie_wb2pcie_dma.desc.ready),
        ]
        self.bus_interconnect_rx = wishbone.InterconnectPointToPoint(
            master = pcie_wb2pcie_dma.bus,
            slave  = ethmac.bus_rx,
        )

        # TX Datapath: Host -> PCIe -> Ethernet (TX).
        # -------------------------------------------
        self.pcie_pcie2wb_dma = pcie_pcie2wb_dma = LitePCIe2WishboneDMA(
            endpoint   = self.pcie_endpoint,
            dma        = self.pcie_dma0.reader,
            data_width = 64,
            mode       = "pcie2wb",
        )
        self.comb += [
            pcie_pcie2wb_dma.desc.bus_addr.eq(ethmac.interface.sram.reader.cmd_fifo.source.slot * ethmac.slot_size.constant),
            pcie_pcie2wb_dma.desc.host_addr.eq(ethmac.interface.sram.reader.pcie_host_addr),
            pcie_pcie2wb_dma.desc.length[align_bits:].eq(ethmac.interface.sram.reader.cmd_fifo.source.length[align_bits:] + 1),
            pcie_pcie2wb_dma.desc.valid.eq(ethmac.interface.sram.reader.start),
            ethmac.interface.sram.reader.ready.eq(pcie_pcie2wb_dma.desc.ready),
        ]
        self.bus_interconnect_tx = wishbone.InterconnectPointToPoint(
            master = pcie_pcie2wb_dma.bus,
            slave  = ethmac.bus_tx,
        )
