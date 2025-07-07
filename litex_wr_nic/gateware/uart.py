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

# UART Constants -----------------------------------------------------------------------------------

WR_UART_PHYSICAL_MODE  = 0b0
WR_UART_CROSSOVER_MODE = 0b1

WR_UART_MANUAL_MODE = 0b0
WR_UART_AUTO_MODE   = 0b1

# UART Pads ----------------------------------------------------------------------------------------

class UARTPads:
    def __init__(self):
        self.tx = Signal()
        self.rx = Signal()

# UART ---------------------------------------------------------------------------------------------

class UARTShared(LiteXModule):
    def __init__(self, pads, sys_clk_freq, default_sel=WR_UART_PHYSICAL_MODE, default_mode=WR_UART_AUTO_MODE):
        # Control registers for UART mode and selection.
        self.control = CSRStorage(fields=[
            CSRField("sel", size=1, values=[
                ("``0b0``", "WR UART connected to Physical UART."),
                ("``0b1``", "WR UART connected to Crossover UART."),
            ], reset=default_sel),
            CSRField("auto_mode", size=1, values=[
                ("``0b0``", "WR UART Manual Mode."),
                ("``0b1``", "WR UART Auto Mode."),
            ], reset=default_mode),
        ])

        # Pads for crossover UART and shared UART.
        self.crossover_pads = crossover_pads = UARTPads()
        self.shared_pads    = shared_pads    = UARTPads()

        # UARTPHY and UART for crossover interface.
        self.xover_phy = UARTPHY(crossover_pads, clk_freq=sys_clk_freq, baudrate=115200)
        self.xover     = UART(self.xover_phy, rx_fifo_depth=128, rx_fifo_rx_we=True)

        # Signal for the active UART selection.
        active_uart = Signal(reset=default_sel)  # Tracks the currently active port (last RX activity).
        uart_select = Signal()                   # Determines the active UART based on mode (auto/manual).

        # RX Activity Detection: Update active UART on RX activity.
        self.sync += [
            # Physical UART activity.
            If(~pads.rx,
                active_uart.eq(WR_UART_PHYSICAL_MODE)
            ),
            # Crossover UART activity.
            If(~crossover_pads.tx,
                active_uart.eq(WR_UART_CROSSOVER_MODE)
            )
        ]

        # UART Selection Logic.
        self.comb += [
            If(self.control.fields.auto_mode,
                uart_select.eq(active_uart)  # Use last active port in auto mode.
            ).Else(
                uart_select.eq(self.control.fields.sel)  # Use manual selection.
            )
        ]

        # UART MUX Logic: Connect shared_pads based on the selected UART.
        self.comb += [
            # Physical UART selected.
            If(uart_select == WR_UART_PHYSICAL_MODE,
                pads.tx.eq(shared_pads.tx),
                shared_pads.rx.eq(pads.rx),
            ),
            # Crossover UART selected.
            If(uart_select == WR_UART_CROSSOVER_MODE,
                crossover_pads.rx.eq(shared_pads.tx),
                shared_pads.rx.eq(crossover_pads.tx),
            )
        ]
