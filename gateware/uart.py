#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *

from litex.soc.cores.uart import UARTPHY, UART

from litex.soc.interconnect.csr import *

# UART Pads ----------------------------------------------------------------------------------------

class UARTPads:
    def __init__(self):
        self.tx = Signal()
        self.rx = Signal()

# UART ---------------------------------------------------------------------------------------------

class UARTShared(LiteXModule):
    def __init__(self, pads, sys_clk_freq):
        self.uart_control = CSRStorage(fields=[
            CSRField("sel", size=1, offset=0, values=[
                ("``0b0``", "WR UART connected to FT4232 UART."),
                ("``0b1``", "WR UART connected to Crossover UART.")
            ], reset=0)
        ])

        # # #

        self.xover_pads = xover_pads = UARTPads()
        self.wr_pads    = wr_pads    = UARTPads()

        self.uart_xover_phy = UARTPHY(xover_pads, clk_freq=sys_clk_freq, baudrate=115200)
        self.uart_xover     = UART(self.uart_xover_phy, rx_fifo_depth=16, rx_fifo_rx_we=True)

        self.comb += [
            # Crossover.
            If(self.uart_control.fields.sel == 0b0,
                pads.tx.eq(wr_pads.tx),
                wr_pads.rx.eq(pads.rx),
            # FT4232.
            ).Else(
                xover_pads.rx.eq(wr_pads.tx),
                wr_pads.rx.eq(xover_pads.tx),
            )
        ]
