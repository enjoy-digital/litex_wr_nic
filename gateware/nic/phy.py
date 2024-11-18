#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream

from liteeth.common import eth_phy_description

# LiteEth GMII WR PHY ------------------------------------------------------------------------------

class LiteEthPHYWRGMII(LiteXModule):
    dw = 8
    with_preamble_crc = False
    with_padding      = False
    def __init__(self, wrf_stream2wb, wrf_wb2stream):
        self.sink    = sink   = stream.Endpoint(eth_phy_description(8))
        self.source  = source = stream.Endpoint(eth_phy_description(8))

        # # #

        self.cd_eth_rx = ClockDomain()
        self.cd_eth_tx = ClockDomain()
        self.comb += [
            self.cd_eth_rx.clk.eq(ClockSignal("sys")),
            self.cd_eth_rx.rst.eq(ResetSignal("sys")),
            self.cd_eth_tx.clk.eq(ClockSignal("sys")),
            self.cd_eth_tx.clk.eq(ClockSignal("sys")),
        ]

        self.comb += [
            sink.connect(wrf_stream2wb.sink,     omit={"last_be", "error"}),
            wrf_wb2stream.source.connect(source, omit={"last_be", "error"}),
        ]
