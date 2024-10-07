#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *

from litex.soc.interconnect import wishbone

from litex.soc.cores.dma import *

from litepcie.frontend.dma import *

# DMA Layouts --------------------------------------------------------------------------------------

def dma_descriptor_layout():
    return [
        ("host_addr", 32),
        ("bus_addr" , 32),
        ("length"   , 32),
    ]

# LitePCIe2WishboneDMA -----------------------------------------------------------------------------

class LitePCIe2WishboneDMA(LiteXModule):
    def __init__(self, endpoint, dma, data_width=64, mode="pcie2wb"):
        assert mode in ["pcie2wb", "wb2pcie"]
        assert dma.data_width == data_width
        self.bus  =  bus = wishbone.Interface(data_width=data_width)
        self.desc = desc = stream.Endpoint(dma_descriptor_layout())
        self.irq  = Signal()

        # # #

        # FIFOs.
        # ------
        self.desc_fifo = desc_fifo = stream.SyncFIFO(dma_descriptor_layout(), 8)
        self.dma_fifo  = dma_fifo  = stream.SyncFIFO(descriptor_layout(),     8)

        # PCIe -> Wishbone.
        # -----------------
        if mode == "pcie2wb":
            self.wb_dma = wb_dma = WishboneDMAWriter(self.bus, endianness="big")
            self.wb_dma.add_ctrl()
            self.comb += dma.source.connect(wb_dma.sink)

        # Wishbone -> PCIe.
        # -----------------
        if mode == "wb2pcie":
            self.wb_dma = wb_dma = WishboneDMAReader(self.bus, endianness="big")
            self.wb_dma.add_ctrl()
            self.comb += wb_dma.source.connect(dma.sink)

        # Datapath.
        # ---------
        self.comb += [
            desc.connect(desc_fifo.sink, omit={"ready"}),

            dma_fifo.sink.address.eq(desc_fifo.source.host_addr),
            dma_fifo.sink.length.eq(desc_fifo.source.length),
            dma_fifo.source.connect(dma.desc_sink),

            wb_dma.base.eq(desc_fifo.source.bus_addr),
            wb_dma.length.eq(desc_fifo.source.length),
        ]

        # FSM.
        # ----
        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            wb_dma.enable.eq(0),
            If(desc_fifo.source.valid & dma_fifo.sink.ready,
                NextState("RUN"),
                dma_fifo.sink.valid.eq(1),
            )
        )
        fsm.act("RUN",
            wb_dma.enable.eq(1),
            If(wb_dma.done,
                desc_fifo.source.ready.eq(1),
                self.irq.eq(1),
                desc.ready.eq(1),
                NextState("IDLE"),
            )
        )
