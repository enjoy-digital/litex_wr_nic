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

# LitePCIe2WishboneDMA -----------------------------------------------------------------------------

class LitePCIe2WishboneDMA(LiteXModule):
    def __init__(self, endpoint, dma, data_width=64, mode="pcie2wb"):
        assert mode in ["pcie2wb", "wb2pcie"]
        assert dma.data_width == data_width
        self.bus  =  bus = wishbone.Interface(data_width=data_width)
        self.desc = desc = stream.Endpoint([("host_addr", 32), ("bus_addr", 32), ("length", 32)])
        self.irq  = Signal()

        # # #

        # Signals.
        # --------
        host_addr = Signal(32)
        bus_addr  = Signal(32)
        length    = Signal(32)
        dma_done  = Signal()

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

        # FSM.
        # ----
        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            wb_dma.enable.eq(0),
            # Wait for a Descriptor.
            If(desc.valid,
                NextValue(host_addr, desc.host_addr),
                NextValue(bus_addr,  desc.bus_addr),
                NextValue(length,    desc.length),
                NextValue(dma_done,  0),
                NextState("RUN")
            )
        )
        fsm.act("RUN",
            # Initiate Wishbone DMA.
            wb_dma.enable.eq(1),
            wb_dma.base.eq(bus_addr),
            wb_dma.length.eq(length),

            # Initiate PCIe DMA.
            dma.desc_sink.valid.eq(~dma_done),
            dma.desc_sink.address.eq(host_addr),
            dma.desc_sink.length.eq(length),
            If(dma.desc_sink.ready,
                NextValue(dma_done, 1)
            ),

            # Wait Wishbone DMA and PCIe DMA to be done.
            If(wb_dma.done & dma_done,
                self.irq.eq(1),
                desc.ready.eq(1),
                NextState("IDLE"),
            )
        )
