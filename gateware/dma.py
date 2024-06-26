#
# This file is part of LitePCIe.
#
# Copyright (c) 2015-2020 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *

from litex.soc.interconnect import wishbone

from litepcie.common import *

from litex.soc.cores.dma import *

from litepcie.frontend.dma import *

# DMA Layouts --------------------------------------------------------------------------------------

def dma_descriptor_layout():
    layout = [("host_addr", 32), ("bus_addr",32), ("length",  32)]
    return EndpointDescription(layout)

# LitePCIe2WishboneDMA -----------------------------------------------------------------------------

class LitePCIe2WishboneDMA(LiteXModule):
    def __init__(self, endpoint, dma, data_width=32, mode="pcie2wb"):
        assert mode in ["pcie2wb", "wb2pcie"]

        dma_desc = stream.Endpoint(descriptor_layout())
        self.dma_fifo = dma_fifo = stream.SyncFIFO(descriptor_layout(), 1)
        desc = stream.Endpoint(dma_descriptor_layout())
        self.fifo = fifo = stream.SyncFIFO(dma_descriptor_layout(), 16)

        self.host_addr   = host_addr   = Signal(32)
        self.length      = length      = Signal(32)
        self.bus_addr    = bus_addr    = Signal(32)
        self.irq_disable = irq_disable = CSRStorage(1, description="Disable PCIe2Wishbone IRQ", reset=0)
        self.start       = start       = Signal(1)
        self.ready       = Signal(reset=0)

        self.bus = wishbone.Interface(data_width=data_width)
        if mode == "pcie2wb":
            self.wb_dma = wb_dma = WishboneDMAWriter(self.bus, endianness="big")
            self.conv   = conv   = stream.Converter(nbits_from=endpoint.phy.data_width, nbits_to=data_width)
        if mode == "wb2pcie":
            self.wb_dma = wb_dma = WishboneDMAReader(self.bus, endianness="big")
            self.conv   = conv   = stream.Converter(nbits_from=data_width, nbits_to=endpoint.phy.data_width)
        wb_dma.add_ctrl()
        self.irq = Signal()

        dma_enable = Signal()

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
            wb_dma.enable.eq(dma_enable),
            dma_desc.connect(dma_fifo.sink),
            dma_fifo.source.connect(dma.desc_sink),
            desc.connect(fifo.sink),

            desc.host_addr.eq(host_addr),
            desc.length.eq(length),
            desc.bus_addr.eq(bus_addr),
            desc.valid.eq(start),

            wb_dma.base.eq(fifo.source.bus_addr),
            wb_dma.length.eq(fifo.source.length),
            dma_desc.address.eq(fifo.source.host_addr),
            dma_desc.length.eq(fifo.source.length),
        ]

        self.fsm = fsm = ResetInserter()(FSM(reset_state="IDLE"))
        fsm.act("IDLE",
            If(fifo.source.valid & dma_desc.ready,
                NextState("RUN"),
                dma_desc.valid.eq(1),
                NextValue(dma_enable, 1),
            )
        )
        fsm.act("RUN",
            If(wb_dma.done,
                fifo.source.ready.eq(1),
                NextState("IDLE"),
                NextValue(dma_enable, 0),
                self.irq.eq(~irq_disable.storage),
                self.ready.eq(1),
            )
        )
