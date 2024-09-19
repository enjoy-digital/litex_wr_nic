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
    layout = [
        ("host_addr", 32),
        ("bus_addr" , 32),
        ("length"   , 32),
    ]
    return EndpointDescription(layout)

# LitePCIe2WishboneDMA -----------------------------------------------------------------------------

class LitePCIe2WishboneDMA(LiteXModule):
    def __init__(self, endpoint, dma, data_width=32, mode="pcie2wb"):
        assert mode in ["pcie2wb", "wb2pcie"]
        self.bus  =  bus = wishbone.Interface(data_width=data_width)
        self.desc = desc = stream.Endpoint(dma_descriptor_layout())

        # # #

        dma_desc = stream.Endpoint(descriptor_layout())
        dma_fifo = stream.SyncFIFO(descriptor_layout(), 1)
        fifo = stream.SyncFIFO(dma_descriptor_layout(), 16)
        self.submodules += dma_fifo, fifo

        self.irq_disable = CSRStorage(1, description="Disable PCIe2Wishbone IRQ")
        self.start       = Signal()
        self.ready       = Signal()
        self.irq         = Signal()

        if mode == "pcie2wb":
            self.wb_dma = wb_dma = WishboneDMAWriter(self.bus, endianness="big")
            self.conv   = conv   = stream.Converter(nbits_from=endpoint.phy.data_width, nbits_to=data_width)
        if mode == "wb2pcie":
            self.wb_dma = wb_dma = WishboneDMAReader(self.bus, endianness="big")
            self.conv   = conv   = stream.Converter(nbits_from=data_width, nbits_to=endpoint.phy.data_width)
        wb_dma.add_ctrl()
        self.irq = Signal()

        if mode == "pcie2wb":
            self.comb += [
                dma.source.connect(conv.sink),
                conv.source.connect(wb_dma.sink),
            ]
        if mode == "wb2pcie":
            self.comb += [
                wb_dma.source.connect(conv.sink),
                conv.source.connect(dma.sink),
            ]

        self.comb += [
            dma_desc.connect(dma_fifo.sink),
            dma_fifo.source.connect(dma.desc_sink),
            desc.connect(fifo.sink),

            desc.valid.eq(self.start),

            wb_dma.base.eq(fifo.source.bus_addr),
            wb_dma.length.eq(fifo.source.length),
            dma_desc.address.eq(fifo.source.host_addr),
            dma_desc.length.eq(fifo.source.length),
        ]

        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            wb_dma.enable.eq(0),
            If(fifo.source.valid & dma_desc.ready,
                NextState("RUN"),
                dma_desc.valid.eq(1),
            )
        )
        fsm.act("RUN",
            wb_dma.enable.eq(1),
            If(wb_dma.done,
                fifo.source.ready.eq(1),
                self.irq.eq(~self.irq_disable.storage),
                self.ready.eq(1),
                NextState("IDLE"),
            )
        )
