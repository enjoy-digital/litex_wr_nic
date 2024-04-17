#
# This file is part of LitePCIe.
#
# Copyright (c) 2015-2020 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.soc.interconnect import wishbone

from litepcie.common import *

from litex.soc.cores.dma import *

from litepcie.frontend.dma import *

# WishboneDMAReaderCtrl --------------------------------------------------------------------------------

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

# WishboneDMAWriterCtrl --------------------------------------------------------------------------------

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

# dma_descriptor_layout --------------------------------------------------------------------------------
def dma_descriptor_layout():
    layout = [("host_addr", 32), ("bus_addr",32), ("length",  32)]
    return EndpointDescription(layout)

# LiteWishbone2PCIeDMA --------------------------------------------------------------------------------

class LiteWishbone2PCIeDMA(Module,AutoCSR):
    def __init__(self, endpoint,data_width = 32):

        port_wr = endpoint.crossbar.get_master_port(write_only=True)
        self.submodules.dma_wr = dma_wr = LitePCIeDMAWriter(
            endpoint=endpoint,
            port=port_wr,
            with_table=False)

        dma_wr_desc = stream.Endpoint(descriptor_layout())
        self.submodules.dma_fifo = dma_fifo = stream.SyncFIFO(descriptor_layout(), 1)

        desc_wr = stream.Endpoint(dma_descriptor_layout())
        self.submodules.fifo_wr = fifo_wr = stream.SyncFIFO(dma_descriptor_layout(), 16)

        self.host_addr = host_addr = CSRStorage(32,description="Host ADDR",reset=0)
        self.length = length = CSRStorage(32,description="Length",reset=0)
        self.bus_addr = bus_addr = CSRStorage(32,description="SoC Bus ADDR",reset=0)
        self.wr_enable = wr_enable = CSRStorage(1,description="Write Table Enable",reset=0)
        self.irq_disable = irq_disable = CSRStorage(1, description="Disable Wishbone2PCIe IRQ", reset=0)
        self.irq = Signal(reset=0)

        self.bus_wr = wishbone.Interface(data_width=data_width)
        self.submodules.wb_dma = wb_dma = WishboneDMAReaderCtrl(self.bus_wr)
        wb_dma.add_ctrl()
        self.submodules.conv_wr = conv_wr = stream.Converter(nbits_from=data_width, nbits_to=endpoint.phy.data_width)

        dma_enable = Signal(reset=0)

        self.test1 = CSRStatus(32,reset=0)
        self.test2 = CSRStatus(32,reset=0)
        self.test3 = CSRStatus(32,reset=0)
        
        self.comb += [
            wb_dma.enable.eq(dma_enable),

            wb_dma.source.connect(conv_wr.sink),
            conv_wr.source.connect(dma_wr.sink),
            dma_wr_desc.connect(dma_fifo.sink),
            dma_fifo.source.connect(dma_wr.desc_sink),
            desc_wr.connect(fifo_wr.sink),

            desc_wr.host_addr.eq(host_addr.storage),
            desc_wr.length.eq(length.storage),
            desc_wr.bus_addr.eq(bus_addr.storage),
            desc_wr.valid.eq(wr_enable.storage & wr_enable.re),

            wb_dma.base.eq(fifo_wr.source.bus_addr),
            wb_dma.length.eq(fifo_wr.source.length),
            dma_wr_desc.address.eq(fifo_wr.source.host_addr),
            dma_wr_desc.length.eq(fifo_wr.source.length),
        ]

        ctrl_fsm = FSM(reset_state="IDLE")
        ctrl_fsm = ResetInserter()(ctrl_fsm)
        self.submodules.ctrl_fsm = ctrl_fsm
        ctrl_fsm.act("IDLE",
                     If(fifo_wr.source.valid & dma_wr_desc.ready,
                        NextState("RUN"),
                        NextValue(self.test1.status, self.test1.status + 1),
                        dma_wr_desc.valid.eq(1),
                        NextValue(dma_enable,1),
                    )
        )
        ctrl_fsm.act("RUN",
                    NextValue(self.test2.status, self.test2.status + 1),
                    If(wb_dma.done,
                       fifo_wr.source.ready.eq(1),
                       NextValue(self.test3.status, self.test3.status + 1),
                       NextState("IDLE"),
                       NextValue(dma_enable, 0),
                       self.irq.eq(~irq_disable.storage)
                    )
        )


# LitePCIe2WishboneDMA --------------------------------------------------------------------------------

class LitePCIe2WishboneDMA(Module, AutoCSR):
    def __init__(self, endpoint, data_width = 32):
        port_rd = endpoint.crossbar.get_master_port()
        self.submodules.dma_rd = dma_rd = LitePCIeDMAReader(
            endpoint=endpoint,
            port=port_rd,
            with_table=False)

        dma_rd_desc = stream.Endpoint(descriptor_layout())
        self.submodules.dma_fifo = dma_fifo = stream.SyncFIFO(descriptor_layout(), 1)
        desc_rd = stream.Endpoint(dma_descriptor_layout())
        self.submodules.fifo_rd = fifo_rd = stream.SyncFIFO(dma_descriptor_layout(), 16)

        self.host_addr = host_addr = CSRStorage(32, description="Host ADDR", reset=0)
        self.length = length = CSRStorage(32, description="Length", reset=0)
        self.bus_addr = bus_addr = CSRStorage(32, description="SoC Bus ADDR", reset=0)
        self.rd_enable = rd_enable = CSRStorage(1, description="Read Enable", reset=0)
        self.irq_disable = irq_disable = CSRStorage(1, description="Disable PCIe2Wishbone IRQ", reset=0)

        self.bus_rd = wishbone.Interface(data_width=data_width)
        self.submodules.wb_dma = wb_dma = WishboneDMAWriterCtrl(self.bus_rd)
        wb_dma.add_ctrl()
        self.irq = Signal(reset=0)

        self.submodules.conv_rd = conv_rd = stream.Converter(nbits_from=endpoint.phy.data_width, nbits_to=data_width)
        dma_enable = Signal(reset=0)
        
        self.test1 = CSRStatus(32,reset=0)
        self.test2 = CSRStatus(32,reset=0)
        self.test3 = CSRStatus(32,reset=0)

        self.comb += [
            wb_dma.enable.eq(dma_enable),

            dma_rd.source.connect(conv_rd.sink),
            conv_rd.source.connect(wb_dma.sink),
            dma_rd_desc.connect(dma_fifo.sink),
            dma_fifo.source.connect(dma_rd.desc_sink),
            desc_rd.connect(fifo_rd.sink),

            desc_rd.host_addr.eq(host_addr.storage),
            desc_rd.length.eq(length.storage),
            desc_rd.bus_addr.eq(bus_addr.storage),
            desc_rd.valid.eq(rd_enable.storage & rd_enable.re),

            wb_dma.base.eq(fifo_rd.source.bus_addr),
            wb_dma.length.eq(fifo_rd.source.length),
            dma_rd_desc.address.eq(fifo_rd.source.host_addr),
            dma_rd_desc.length.eq(fifo_rd.source.length),
        ]
        
        ctrl_fsm = FSM(reset_state="IDLE")
        ctrl_fsm = ResetInserter()(ctrl_fsm)
        self.submodules.ctrl_fsm = ctrl_fsm
        ctrl_fsm.act("IDLE",
                     If(fifo_rd.source.valid & dma_rd_desc.ready,
                        NextState("RUN"),
                        NextValue(self.test1.status, self.test1.status + 1),
                        dma_rd_desc.valid.eq(1),
                        NextValue(dma_enable,1),
                    )
        )
        ctrl_fsm.act("RUN",
                    NextValue(self.test2.status, self.test2.status + 1),
                    If(wb_dma.done,
                       fifo_rd.source.ready.eq(1),
                       NextValue(self.test3.status, self.test3.status + 1),
                       NextState("IDLE"),
                       NextValue(dma_enable, 0),
                       self.irq.eq(~irq_disable.storage)
                    )
        )

class PCIeInterruptTest(Module,AutoCSR):
    def __init__(self):
        self.irq1_csr = CSRStorage(1,reset=0)
        self.irq2_csr = CSRStorage(1,reset=0)
        self.irq3_csr = CSRStorage(1,reset=0)
        self.irq1 = Signal()
        self.irq2 = Signal()
        self.irq3 = Signal()
        self.comb += [
            self.irq1.eq(self.irq1_csr.storage & self.irq1_csr.re),
            self.irq2.eq(self.irq2_csr.storage & self.irq2_csr.re),
            self.irq3.eq(self.irq3_csr.storage & self.irq3_csr.re)
        ]


class LiteWishbone2PCIeDMANative(Module, AutoCSR):
    def __init__(self, endpoint, data_width=32):
        port_wr = endpoint.crossbar.get_master_port(write_only=True)
        self.submodules.dma_wr = dma_wr = LitePCIeDMAWriter(
            endpoint=endpoint,
            port=port_wr,
            with_table=False)

        dma_wr_desc = stream.Endpoint(descriptor_layout())
        self.submodules.dma_fifo = dma_fifo = stream.SyncFIFO(descriptor_layout(), 16)

        desc_wr = stream.Endpoint(dma_descriptor_layout())
        self.submodules.fifo_wr = fifo_wr = stream.SyncFIFO(dma_descriptor_layout(), 16)

        self.host_addr = host_addr = Signal(32, reset=0)
        self.length = length = Signal(32, reset=0)
        self.bus_addr = bus_addr = Signal(32, reset=0)
        self.irq_disable = irq_disable = CSRStorage(1, description="Disable PCIe2Wishbone IRQ", reset=0)
        self.start = start = Signal(1, reset=0)
        self.ready = Signal(reset=0)

        self.bus_wr = wishbone.Interface(data_width=data_width)
        self.submodules.wb_dma = wb_dma = WishboneDMAReaderCtrl(self.bus_wr)
        wb_dma.add_ctrl()
        self.submodules.conv_wr = conv_wr = stream.Converter(nbits_from=data_width, nbits_to=endpoint.phy.data_width)
        self.irq = Signal(reset=0)
        dma_enable = Signal(reset=0)

        self.test1 = CSRStatus(32, reset=0)
        self.test2 = CSRStatus(32, reset=0)
        self.test3 = CSRStatus(32, reset=0)

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
        self.submodules.ctrl_fsm = ctrl_fsm
        ctrl_fsm.act("IDLE",
                     If(fifo_wr.source.valid & dma_wr_desc.ready,
                        NextState("RUN"),
                        NextValue(self.test1.status, self.test1.status + 1),
                        dma_wr_desc.valid.eq(1),
                        NextValue(dma_enable, 1),
                        )
                     )
        ctrl_fsm.act("RUN",
                     NextValue(self.test2.status, self.test2.status + 1),
                     If(wb_dma.done,
                        fifo_wr.source.ready.eq(1),
                        NextValue(self.test3.status, self.test3.status + 1),
                        NextState("IDLE"),
                        NextValue(dma_enable, 0),
                        self.irq.eq(~irq_disable.storage),
                        self.ready.eq(1)
                        )
                     )


# LitePCIe2WishboneDMA --------------------------------------------------------------------------------

class LitePCIe2WishboneDMANative(Module, AutoCSR):
    def __init__(self, endpoint, data_width=32):
        port_rd = endpoint.crossbar.get_master_port()
        self.submodules.dma_rd = dma_rd = LitePCIeDMAReader(
            endpoint=endpoint,
            port=port_rd,
            with_table=False)

        dma_rd_desc = stream.Endpoint(descriptor_layout())
        self.submodules.dma_fifo = dma_fifo = stream.SyncFIFO(descriptor_layout(), 1)
        desc_rd = stream.Endpoint(dma_descriptor_layout())
        self.submodules.fifo_rd = fifo_rd = stream.SyncFIFO(dma_descriptor_layout(), 16)

        self.host_addr = host_addr = Signal(32, reset=0)
        self.length = length = Signal(32, reset=0)
        self.bus_addr = bus_addr = Signal(32, reset=0)
        self.irq_disable = irq_disable = CSRStorage(1, description="Disable PCIe2Wishbone IRQ", reset=0)
        self.start = start = Signal(1, reset=0)
        self.ready = Signal(reset=0)

        self.bus_rd = wishbone.Interface(data_width=data_width)
        self.submodules.wb_dma = wb_dma = WishboneDMAWriterCtrl(self.bus_rd)
        wb_dma.add_ctrl()
        self.irq = Signal(reset=0)
        self.submodules.conv_rd = conv_rd = stream.Converter(nbits_from=endpoint.phy.data_width, nbits_to=data_width)
        dma_enable = Signal(reset=0)

        self.test1 = CSRStatus(32, reset=0)
        self.test2 = CSRStatus(32, reset=0)
        self.test3 = CSRStatus(32, reset=0)

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
        self.submodules.ctrl_fsm = ctrl_fsm
        ctrl_fsm.act("IDLE",
                     If(fifo_rd.source.valid & dma_rd_desc.ready,
                        NextState("RUN"),
                        NextValue(self.test1.status, self.test1.status + 1),
                        dma_rd_desc.valid.eq(1),
                        NextValue(dma_enable, 1),
                        )
                     )
        ctrl_fsm.act("RUN",
                     NextValue(self.test2.status, self.test2.status + 1),
                     If(wb_dma.done,
                        fifo_rd.source.ready.eq(1),
                        NextValue(self.test3.status, self.test3.status + 1),
                        NextState("IDLE"),
                        NextValue(dma_enable, 0),
                        self.irq.eq(~irq_disable.storage),
                        self.ready.eq(1),
                    )
        )