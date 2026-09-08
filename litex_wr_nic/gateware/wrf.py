#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from migen.genlib.cdc import BusSynchronizer

from litex.gen import *

from litex.soc.interconnect import wishbone
from litex.soc.interconnect.csr import CSRStatus

from litex_wr_nic.gateware.wr_cdc import WRClockCrossing

# WR Fabric Interfaces -----------------------------------------------------------------------------

class WRFInterface(wishbone.Interface):
    def __init__(self):
        super().__init__(data_width=16, address_width=2, addressing="byte")
        # Unlike classic Wishbone, WR fabric separates request acceptance
        # (STB and not STALL) from response completion (ACK/ERR/RTY).
        for name in ("stall", "rty"):
            setattr(self, name, Signal(name=name))
            self.layout.append((name, 1, DIR_S_TO_M))


WRFClockCrossing = WRClockCrossing

class WRFCounters(LiteXModule):
    def __init__(self, cd):
        for name, description in {
            "packets"  : "Completed WR fabric packets.",
            "errors"   : "WR fabric packets with errors.",
            "stalls"   : "Cycles with a stalled WR fabric request.",
            "overflow" : "WR fabric requests changed or withdrawn while stalled.",
        }.items():
            counter = Signal(32, name=name)
            status  = CSRStatus(32, name=name, description=description + " Wraps at 32 bits.")
            cdc     = BusSynchronizer(32, cd, "sys")
            setattr(self, name, counter)
            setattr(self, "_" + name, status)
            setattr(self, name + "_cdc", cdc)
            self.comb += [
                cdc.i.eq(counter),
                status.status.eq(cdc.o),
            ]
