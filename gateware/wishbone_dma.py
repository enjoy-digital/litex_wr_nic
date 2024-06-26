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

# WishboneDMAReaderCtrl ----------------------------------------------------------------------------

class WishboneDMAReaderCtrl(WishboneDMAReader):
    """Read data from Wishbone MMAP memory.

    For every address written to the sink, one word will be produced on the source.

    Parameters
    ----------
    bus : bus
        Wishbone bus of the SoC to read from.

    Attributes
    ----------
    sink : Record("address")
        Sink for MMAP addresses to be read.

    source : Record("data")
        Source for MMAP word results from reading.
    """
    def __init__(self, bus, endianness="big"):
        assert isinstance(bus, wishbone.Interface)
        WishboneDMAReader.__init__(self,bus,endianness=endianness,with_csr=False)

    def add_ctrl(self, default_base=0, default_length=0, default_enable=0, default_loop=0):
        self.base   = Signal(64, reset=default_base)
        self.length = Signal(32, reset=default_length)
        self.enable = Signal(reset=default_enable)
        self.done   = Signal()
        self.loop   = Signal(reset=default_loop)
        self.offset = Signal(32)

        # # #

        shift   = log2_int(self.bus.data_width//8)
        base    = Signal(self.bus.adr_width)
        offset  = Signal(self.bus.adr_width)
        length  = Signal(self.bus.adr_width)
        self.comb += base.eq(self.base[shift:])
        self.comb += length.eq(self.length[shift:])

        self.comb += self.offset.eq(offset)

        fsm = FSM(reset_state="IDLE")
        fsm = ResetInserter()(fsm)
        self.submodules += fsm
        self.comb += fsm.reset.eq(~self.enable)
        fsm.act("IDLE",
            NextValue(offset, 0),
            NextState("RUN"),
        )
        fsm.act("RUN",
            self.sink.valid.eq(1),
            self.sink.last.eq(offset == (length - 1)),
            self.sink.address.eq(base + offset),
            If(self.sink.ready,
                NextValue(offset, offset + 1),
                If(self.sink.last,
                    If(self.loop,
                        NextValue(offset, 0)
                    ).Else(
                        NextState("DONE")
                    )
                )
            )
        )
        fsm.act("DONE", self.done.eq(1))

# WishboneDMAWriterCtrl ----------------------------------------------------------------------------

class WishboneDMAWriterCtrl(WishboneDMAWriter):
    """Write data to Wishbone MMAP memory.

    Parameters
    ----------
    bus : bus
        Wishbone bus of the SoC to read from.

    Attributes
    ----------
    sink : Record("address", "data")
        Sink for MMAP addresses/datas to be written.
    """
    def __init__(self, bus, endianness="big"):
        assert isinstance(bus, wishbone.Interface)
        WishboneDMAWriter.__init__(self,bus,endianness,with_csr=False)

    def add_ctrl(self, default_base=0, default_length=0, default_enable=0, default_loop=0):
        self._sink = self.sink
        self.sink  = stream.Endpoint([("data", self.bus.data_width)])

        self.base   = Signal(64, reset=default_base)
        self.length = Signal(32, reset=default_length)
        self.enable = Signal(reset=default_enable)
        self.done   = Signal()
        self.loop   = Signal(reset=default_loop)
        self.offset = Signal(32)

        # # #

        shift   = log2_int(self.bus.data_width//8)
        base    = Signal(self.bus.adr_width)
        offset  = Signal(self.bus.adr_width)
        length  = Signal(self.bus.adr_width)
        self.comb += base.eq(self.base[shift:])
        self.comb += length.eq(self.length[shift:])

        self.comb += self.offset.eq(offset)

        fsm = FSM(reset_state="IDLE")
        fsm = ResetInserter()(fsm)
        self.submodules += fsm
        self.comb += fsm.reset.eq(~self.enable)
        fsm.act("IDLE",
            self.sink.ready.eq(1),
            NextValue(offset, 0),
            NextState("RUN"),
        )
        fsm.act("RUN",
            self._sink.valid.eq(self.sink.valid),
            self._sink.last.eq(offset == (length - 1)),
            self._sink.address.eq(base + offset),
            self._sink.data.eq(self.sink.data),
            self.sink.ready.eq(self._sink.ready),
            If(self.sink.valid & self.sink.ready,
                NextValue(offset, offset + 1),
                If(self._sink.last,
                    If(self.loop,
                        NextValue(offset, 0)
                    ).Else(
                        NextState("DONE")
                    )
                )
            )
        )
        fsm.act("DONE", self.done.eq(1))


# LiteWishbone2PCIeDMANative -----------------------------------------------------------------------

class LiteWishbone2PCIeDMANative(LiteXModule):
    def __init__(self, endpoint, dma_wr, data_width=32):

        dma_wr_desc = stream.Endpoint(descriptor_layout())
        self.dma_fifo = dma_fifo = stream.SyncFIFO(descriptor_layout(), 16)

        desc_wr = stream.Endpoint(dma_descriptor_layout())
        self.fifo_wr = fifo_wr = stream.SyncFIFO(dma_descriptor_layout(), 16)

        self.host_addr = host_addr = Signal(32, reset=0)
        self.length = length = Signal(32, reset=0)
        self.bus_addr = bus_addr = Signal(32, reset=0)
        self.irq_disable = irq_disable = CSRStorage(1, description="Disable PCIe2Wishbone IRQ", reset=0)
        self.start = start = Signal(1, reset=0)
        self.ready = Signal(reset=0)

        self.bus_wr = wishbone.Interface(data_width=data_width)
        self.wb_dma = wb_dma = WishboneDMAReaderCtrl(self.bus_wr)
        wb_dma.add_ctrl()
        self.conv_wr = conv_wr = stream.Converter(nbits_from=data_width, nbits_to=endpoint.phy.data_width)
        self.irq = Signal(reset=0)
        dma_enable = Signal(reset=0)

        self.comb += [
            wb_dma.enable.eq(dma_enable),

            wb_dma.source.connect(conv_wr.sink),
            conv_wr.source.connect(dma_wr.sink),
            dma_wr_desc.connect(dma_fifo.sink),
            dma_fifo.source.connect(dma_wr.desc_sink),
            desc_wr.connect(fifo_wr.sink),

            desc_wr.host_addr.eq(host_addr),
            desc_wr.length.eq(length),
            desc_wr.bus_addr.eq(bus_addr),
            desc_wr.valid.eq(start),

            wb_dma.base.eq(fifo_wr.source.bus_addr),
            wb_dma.length.eq(fifo_wr.source.length),
            dma_wr_desc.address.eq(fifo_wr.source.host_addr),
            dma_wr_desc.length.eq(fifo_wr.source.length),
        ]

        ctrl_fsm = FSM(reset_state="IDLE")
        ctrl_fsm = ResetInserter()(ctrl_fsm)
        self.ctrl_fsm = ctrl_fsm
        ctrl_fsm.act("IDLE",
            If(fifo_wr.source.valid & dma_wr_desc.ready,
                NextState("RUN"),
                dma_wr_desc.valid.eq(1),
                NextValue(dma_enable, 1),
            )
        )
        ctrl_fsm.act("RUN",
            If(wb_dma.done,
                fifo_wr.source.ready.eq(1),
                NextState("IDLE"),
                NextValue(dma_enable, 0),
                self.irq.eq(~irq_disable.storage),
                self.ready.eq(1)
            )
        )


# LitePCIe2WishboneDMANative -----------------------------------------------------------------------

class LitePCIe2WishboneDMANative(LiteXModule):
    def __init__(self, endpoint, dma_rd, data_width=32):

        dma_rd_desc = stream.Endpoint(descriptor_layout())
        self.dma_fifo = dma_fifo = stream.SyncFIFO(descriptor_layout(), 1)
        desc_rd = stream.Endpoint(dma_descriptor_layout())
        self.fifo_rd = fifo_rd = stream.SyncFIFO(dma_descriptor_layout(), 16)

        self.host_addr = host_addr = Signal(32, reset=0)
        self.length = length = Signal(32, reset=0)
        self.bus_addr = bus_addr = Signal(32, reset=0)
        self.irq_disable = irq_disable = CSRStorage(1, description="Disable PCIe2Wishbone IRQ", reset=0)
        self.start = start = Signal(1, reset=0)
        self.ready = Signal(reset=0)

        self.bus_rd = wishbone.Interface(data_width=data_width)
        self.wb_dma = wb_dma = WishboneDMAWriterCtrl(self.bus_rd)
        wb_dma.add_ctrl()
        self.irq = Signal(reset=0)
        self.conv_rd = conv_rd = stream.Converter(nbits_from=endpoint.phy.data_width, nbits_to=data_width)
        dma_enable = Signal(reset=0)

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

        ctrl_fsm = FSM(reset_state="IDLE")
        ctrl_fsm = ResetInserter()(ctrl_fsm)
        self.ctrl_fsm = ctrl_fsm
        ctrl_fsm.act("IDLE",
            If(fifo_rd.source.valid & dma_rd_desc.ready,
                NextState("RUN"),
                dma_rd_desc.valid.eq(1),
                NextValue(dma_enable, 1),
            )
        )
        ctrl_fsm.act("RUN",
            If(wb_dma.done,
                fifo_rd.source.ready.eq(1),
                NextState("IDLE"),
                NextValue(dma_enable, 0),
                self.irq.eq(~irq_disable.storage),
                self.ready.eq(1),
            )
        )
