import sys

from migen import *

from litex.build import tools

from litex.gen import *

from litex.soc.interconnect import wishbone

from litex.soc.cores.clock          import *
from litex.soc.integration.soc      import SoCBusHandler, SoCRegion, SoCIORegion
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder  import *
from litex.soc.integration.export   import get_csr_header, get_soc_header, get_mem_header

from gateware import sram
sys.modules["liteeth.mac.sram"] = sram #  Replace Liteeth SRAM with our custom implementation.

class EthernetPCIeSoC(SoCMini):
    SoCMini.csr_map = {
        "ethmac":                1,
        "ethphy":                2,
        "identifier_mem":        3,
        "leds":                  4,

        # PCIe.
        "pcie_endpoint":         5,
        "pcie_host_pcie2wb_dma": 6,
        "pcie_host_wb2pcie_dma": 7,
        "pcie_msi":              8,
        "pcie_phy":              9,
    }

    def add_ethernet_pcie(self, name="ethmac", phy=None, pcie_phy=None, phy_cd="eth",
        nrxslots                = 32,
        ntxslots                = 32,
        with_timing_constraints = True,
        max_pending_requests    = 8,
        with_msi                = True):

        data_width = 64
        self.pcie_mem_bus_tx = SoCBusHandler(data_width=data_width)
        
        # MAC.
        self.add_ethernet(
            name                    = name,
            phy                     = phy,
            phy_cd                  = phy_cd,
            data_width              = data_width,
            nrxslots                = nrxslots,
            ntxslots                = ntxslots, 
        )
        self.add_constant("ETHMAC_RX_WAIT_OFFSET",  0) # CHECKME: See purpose in software.
        self.add_constant("ETHMAC_TX_READY_OFFSET", 1) # CHECKME: See purpose in software.
        del self.bus.slaves["ethmac_tx"]
        del self.bus.slaves["ethmac_rx"]

         # Compute Regions size and add it to the SoC.
        self.ethmac_region_tx = SoCRegion(origin=0, size=self.ethmac.tx_slots.constant * self.ethmac.slot_size.constant, cached=False)
        self.pcie_mem_bus_tx.add_region(name="io",region=SoCIORegion(0x00000000,0x100000000))
        self.pcie_mem_bus_tx.add_slave(name='ethmac_tx', slave=self.ethmac.bus_tx, region=self.ethmac_region_tx)

        # PCIe
        self.add_pcie(name="pcie", phy=pcie_phy,
            ndmas                = 1,
            max_pending_requests = max_pending_requests,
            data_width           = data_width,
            with_dma_buffering   = False,
            with_dma_loopback    = False,
            with_dma_table       = False,
            with_msi             = with_msi,
            msis                 = {
                "ETHRX" : self.ethmac.interface.sram.rx_pcie_irq,
                "ETHTX" : self.ethmac.interface.sram.tx_pcie_irq,
            },
            with_ptm             = False,
        )

        from gateware.dma import LitePCIe2WishboneDMA

        align_bits = log2_int(512)

        # RX: Wishbone -> PCIe.
        # ---------------------
        pcie_wb2pcie_dma = LitePCIe2WishboneDMA(self.pcie_endpoint, self.pcie_dma0.writer, data_width, mode="wb2pcie")
        self.pcie_wb2pcie_dma = pcie_wb2pcie_dma
        self.comb += [
            self.pcie_wb2pcie_dma.bus_addr.eq(self.ethmac.interface.sram.writer.stat_fifo.source.slot * self.ethmac.slot_size.constant),
            self.pcie_wb2pcie_dma.host_addr.eq(self.ethmac.interface.sram.writer.pcie_host_addr),
            self.pcie_wb2pcie_dma.length.eq(Cat(Signal(align_bits,reset=0), (self.ethmac.interface.sram.writer.stat_fifo.source.length[align_bits:] + 1))),
            self.pcie_wb2pcie_dma.start.eq(self.ethmac.interface.sram.writer.start_transfer),
            self.ethmac.interface.sram.writer.transfer_ready.eq(self.pcie_wb2pcie_dma.ready),
        ]
        self.bus_interconnect_rx = wishbone.InterconnectPointToPoint(
            master = pcie_wb2pcie_dma.bus,
            slave  = self.ethmac.bus_rx,
        )

        # TX: PCIe -> Wishbone.
        # ---------------------
        pcie_pcie2wb_dma = LitePCIe2WishboneDMA(self.pcie_endpoint, self.pcie_dma0.reader, data_width, mode="pcie2wb")
        self.pcie_pcie2wb_dma = pcie_pcie2wb_dma
        self.pcie_mem_bus_tx.add_master("pcie_pcie2wb_dma", pcie_pcie2wb_dma.bus)
        self.comb += [
            self.pcie_pcie2wb_dma.bus_addr.eq(self.ethmac.interface.sram.reader.cmd_fifo.source.slot * self.ethmac.slot_size.constant),
            self.pcie_pcie2wb_dma.host_addr.eq(self.ethmac.interface.sram.reader.pcie_host_addr),
            self.pcie_pcie2wb_dma.length.eq(Cat(Signal(align_bits, reset=0), (self.ethmac.interface.sram.reader.cmd_fifo.source.length[align_bits:] + 1))),
            self.pcie_pcie2wb_dma.start.eq(self.ethmac.interface.sram.reader.start_transfer),
            self.ethmac.interface.sram.reader.transfer_ready.eq(self.pcie_pcie2wb_dma.ready),
        ]
        self.bus_interconnect_tx = wishbone.InterconnectPointToPoint(
            master = next(iter(self.pcie_mem_bus_tx.masters.values())),
            slave  = next(iter( self.pcie_mem_bus_tx.slaves.values())),
        )

    def generate_software_header(self, dst):
        csr_header = get_csr_header(self.csr_regions, self.constants, with_access_functions=False)
        tools.write_to_file(os.path.join(dst, "csr.h"), csr_header)
        self_header = get_soc_header(self.constants, with_access_functions=False)
        tools.write_to_file(os.path.join(dst, "soc.h"), self_header)
        mem_header = get_mem_header(self.mem_regions)
        tools.write_to_file(os.path.join(dst, "mem.h"), mem_header)
