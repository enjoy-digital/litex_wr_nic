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

        self.irq_disable = CSRStorage(1, description="Disable DMA IRQ")
        self.irq         = Signal()

        # # #

        self.fifo = fifo = stream.SyncFIFO(dma_descriptor_layout(), 16)

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
            desc.connect(fifo.sink, omit={"ready"}),

            wb_dma.base.eq(fifo.source.bus_addr),
            wb_dma.length.eq(fifo.source.length),
            dma.desc_sink.address.eq(fifo.source.host_addr),
            dma.desc_sink.length.eq(fifo.source.length),
        ]

        # FSM.
        # ----
        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            wb_dma.enable.eq(0),
            If(fifo.source.valid,
                NextState("RUN"),
                dma.desc_sink.valid.eq(1),
            )
        )
        fsm.act("RUN",
            wb_dma.enable.eq(1),
            If(wb_dma.done,
                fifo.source.ready.eq(1),
                self.irq.eq(~self.irq_disable.storage),
                desc.ready.eq(1),
                NextState("IDLE"),
            )
        )
