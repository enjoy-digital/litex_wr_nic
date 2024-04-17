#
# This file is part of LiteEth.
#
# Copyright (c) 2015-2020 Florent Kermarrec <florent@enjoy-digital.fr>
# Copyright (c) 2023 LumiGuide Fietsdetectie B.V. <goemansrowan@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

from liteeth.common import *
from liteeth.mac.common import *
from liteeth.mac.core import LiteEthMACCore
from liteeth.mac.wishbone import LiteEthMACWishboneInterface

# MAC ----------------------------------------------------------------------------------------------

class LiteEthMAC(Module, AutoCSR):
    def __init__(self, phy, dw,
        interface          = "crossbar",
        endianness         = "big",
        with_preamble_crc  = True,
        nrxslots           = 2,
        rxslots_read_only  = True,
        ntxslots           = 2,
        txslots_write_only = False,
        hw_mac             = None,
        timestamp          = None,
        full_memory_we     = False,
        with_sys_datapath  = False,
        tx_cdc_depth       = 32,
        tx_cdc_buffered    = False,
        rx_cdc_depth       = 32,
        rx_cdc_buffered    = False,
    ):
        assert dw%8 == 0
        assert interface  in ["crossbar", "wishbone", "hybrid", "pcie"]
        assert endianness in ["big", "little"]

        self.submodules.core = LiteEthMACCore(
            phy               = phy,
            dw                = dw,
            with_sys_datapath = with_sys_datapath,
            with_preamble_crc = with_preamble_crc,
            tx_cdc_depth      = tx_cdc_depth,
            tx_cdc_buffered   = tx_cdc_buffered,
            rx_cdc_depth      = rx_cdc_depth,
            rx_cdc_buffered   = rx_cdc_buffered,
        )
        self.csrs = []
        if interface == "crossbar":
            self.submodules.crossbar     = LiteEthMACCrossbar(dw)
            self.submodules.packetizer   = LiteEthMACPacketizer(dw)
            self.submodules.depacketizer = LiteEthMACDepacketizer(dw)
            self.comb += [
                self.crossbar.master.source.connect(self.packetizer.sink),
                self.packetizer.source.connect(self.core.sink),
                self.core.source.connect(self.depacketizer.sink),
                self.depacketizer.source.connect(self.crossbar.master.sink)
            ]
        else:
            # Wishbone MAC
            self.rx_slots  = CSRConstant(nrxslots)
            self.tx_slots  = CSRConstant(ntxslots)
            self.slot_size = CSRConstant(2**bits_for(eth_mtu))
            wishbone_interface = LiteEthMACWishboneInterface(
                dw         = dw,
                nrxslots   = nrxslots, rxslots_read_only  = rxslots_read_only,
                ntxslots   = ntxslots, txslots_write_only = txslots_write_only,
                endianness = endianness,
                timestamp  = timestamp,
            )
            # On some targets (Intel/Altera), the complex ports aren't inferred
            # as block ram, but are created with LUTs.  FullMemoryWe splits such
            # `Memory` instances up into 4 separate memory blocks, each
            # containing 8 bits which gets inferred correctly on intel/altera.
            # Yosys on ECP5 inferrs the original correctly, so FullMemoryWE
            # leads to additional block ram instances being used, which
            # increases memory usage by a lot.
            if full_memory_we:
                wishbone_interface = FullMemoryWE()(wishbone_interface)
            self.submodules.interface = wishbone_interface
            if interface == "pcie":
                self.rx_bus = self.interface.rx_bus
                self.tx_bus = self.interface.tx_bus
                self.rx_pcie_irq = self.interface.sram.rx_pcie_irq
                self.tx_pcie_irq = self.interface.sram.tx_pcie_irq
            else:
                self.ev, self.bus = self.interface.sram.ev, self.interface.bus
            self.csrs = self.interface.get_csrs() + self.core.get_csrs()

            if interface == "hybrid":
                # Hardware MAC
                self.submodules.crossbar     = LiteEthMACCrossbar(dw)
                self.submodules.mac_crossbar = LiteEthMACCoreCrossbar(self.core, self.crossbar, self.interface, dw, hw_mac)
            else:
                self.comb += self.interface.source.connect(self.core.sink)
                self.comb += self.core.source.connect(self.interface.sink)

    def get_csrs(self):
        return self.csrs

