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

# LitePCIe2WishboneDMANative -----------------------------------------------------------------------

class LitePCIe2WishboneDMANative(LiteXModule):
    def __init__(self, endpoint, dma_rd, data_width=32):

        dma_rd_desc = stream.Endpoint(descriptor_layout())
        self.dma_fifo = dma_fifo = stream.SyncFIFO(descriptor_layout(), 1)
        desc_rd = stream.Endpoint(dma_descriptor_layout())
        self.fifo_rd = fifo_rd = stream.SyncFIFO(dma_descriptor_layout(), 16)

        self.host_addr   = host_addr   = Signal(32)
        self.length      = length      = Signal(32)
        self.bus_addr    = bus_addr    = Signal(32)
        self.irq_disable = irq_disable = CSRStorage(1, description="Disable PCIe2Wishbone IRQ", reset=0)
        self.start       = start       = Signal(1)
        self.ready       = Signal(reset=0)

        self.bus_rd = wishbone.Interface(data_width=data_width)
        self.wb_dma = wb_dma = WishboneDMAWriter(self.bus_rd, endianness="big")
        wb_dma.add_ctrl()
        self.irq = Signal()
        self.conv_rd = conv_rd = stream.Converter(nbits_from=endpoint.phy.data_width, nbits_to=data_width)
        dma_enable = Signal()

        self.comb += [
            wb_dma.enable.eq(dma_enable),

            dma_rd.source.connect(conv_rd.sink),
            conv_rd.source.connect(wb_dma.sink),
            dma_rd_desc.connect(dma_fifo.sink),
            dma_fifo.source.connect(dma_rd.desc_sink),
            desc_rd.connect(fifo_rd.sink),

            desc_rd.host_addr.eq(host_addr),
            desc_rd.length.eq(length),
            desc_rd.bus_addr.eq(bus_addr),
            desc_rd.valid.eq(self.start),

            wb_dma.base.eq(fifo_rd.source.bus_addr),
            wb_dma.length.eq(fifo_rd.source.length),
            dma_rd_desc.address.eq(fifo_rd.source.host_addr),
            dma_rd_desc.length.eq(fifo_rd.source.length),
        ]

        self.fsm = fsm = ResetInserter()(FSM(reset_state="IDLE"))
        fsm.act("IDLE",
            If(fifo_rd.source.valid & dma_rd_desc.ready,
                NextState("RUN"),
                dma_rd_desc.valid.eq(1),
                NextValue(dma_enable, 1),
            )
        )
        fsm.act("RUN",
            If(wb_dma.done,
                fifo_rd.source.ready.eq(1),
                NextState("IDLE"),
                NextValue(dma_enable, 0),
                self.irq.eq(~irq_disable.storage),
                self.ready.eq(1),
            )
        )
