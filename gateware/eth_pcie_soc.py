 
from migen import *

from litex.build import tools

from litex.gen import *

from litex.soc.cores.clock import *
from litex.soc.integration.soc import SoCBusHandler, SoCRegion, SoCIORegion
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.integration.export import get_csr_header, get_soc_header, get_mem_header

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

    # Add Ethernet ---------------------------------------------------------------------------------
    def __add_ethernet(self, name="ethmac", phy=None, phy_cd="eth", dynamic_ip=False, software_debug=False,
        data_width              = 8,
        nrxslots                = 2, rxslots_read_only  = True,
        ntxslots                = 2, txslots_write_only = False,
        with_timestamp          = False,
        with_timing_constraints = True,
        local_ip                = None,
        remote_ip               = None,
        with_pcie_eth           = True):
        # Imports
        from gateware.liteeth.mac import LiteEthMAC
        from liteeth.phy.model import LiteEthPHYModel

        # MAC.
        assert data_width in [8, 32, 128]
        with_sys_datapath = (data_width == 32)
        self.check_if_exists(name)
        if with_timestamp:
            self.timer0.add_uptime()
        ethmac = LiteEthMAC(
            phy        = phy,
            dw         = data_width,
            interface  = "pcie",
            endianness = self.cpu.endianness,
            nrxslots   = nrxslots, rxslots_read_only  = rxslots_read_only,
            ntxslots   = ntxslots, txslots_write_only = txslots_write_only,
            timestamp  = None if not with_timestamp else self.timer0.uptime_cycles,
            with_preamble_crc = not software_debug,
            with_sys_datapath = with_sys_datapath)

        if with_pcie_eth:
            self.add_constant("ETHMAC_RX_WAIT_OFFSET", ethmac.interface.wait_ack_offset)
            self.add_constant("ETHMAC_TX_READY_OFFSET", ethmac.interface.tx_ready_offset)

        if not with_sys_datapath:
            # Use PHY's eth_tx/eth_rx clock domains.
            ethmac = ClockDomainsRenamer({
                "eth_tx": phy_cd + "_tx",
                "eth_rx": phy_cd + "_rx"})(ethmac)
        self.add_module(name=name, module=ethmac)
        # Compute Regions size and add it to the SoC.
        if with_pcie_eth:
            self.ethmac_region_rx = SoCRegion(origin=0, size=ethmac.rx_slots.constant * ethmac.slot_size.constant, cached=False)
            self.ethmac_region_tx = SoCRegion(origin=0, size=ethmac.tx_slots.constant * ethmac.slot_size.constant, cached=False)
            self.pcie_mem_bus_rx.add_region(name="io",region=SoCIORegion(0x00000000,0x100000000))
            self.pcie_mem_bus_tx.add_region(name="io",region=SoCIORegion(0x00000000,0x100000000))
            self.pcie_mem_bus_rx.add_slave(name='ethmac_rx', slave=ethmac.rx_bus, region=self.ethmac_region_rx)
            self.pcie_mem_bus_tx.add_slave(name='ethmac_tx', slave=ethmac.tx_bus, region=self.ethmac_region_tx)
        else:
            ethmac_region_size = (ethmac.rx_slots.constant + ethmac.tx_slots.constant)*ethmac.slot_size.constant
            ethmac_region = SoCRegion(origin=self.mem_map.get(name, None), size=ethmac_region_size, cached=False)
            self.bus.add_slave(name=name, slave=ethmac.bus, region=ethmac_region)

            # Add IRQs (if enabled).
            if self.irq.enabled:
                self.irq.add(name, use_loc_if_exists=True)

        # Dynamic IP (if enabled).
        if dynamic_ip:
            assert local_ip is None
            self.add_constant("ETH_DYNAMIC_IP")

        # Local/Remote IP Configuration (optional).
        if local_ip:
            add_ip_address_constants(self, "LOCALIP", local_ip)
        if remote_ip:
            add_ip_address_constants(self, "REMOTEIP", remote_ip)

        # Software Debug
        if software_debug:
            self.add_constant("ETH_UDP_TX_DEBUG")
            self.add_constant("ETH_UDP_RX_DEBUG")

        # Timing constraints
        if with_timing_constraints:
            eth_rx_clk = getattr(phy, "crg", phy).cd_eth_rx.clk
            eth_tx_clk = getattr(phy, "crg", phy).cd_eth_tx.clk
            if not isinstance(phy, LiteEthPHYModel) and not getattr(phy, "model", False):
                self.platform.add_period_constraint(eth_rx_clk, 1e9/phy.rx_clk_freq)
                if not eth_rx_clk is eth_tx_clk:
                    self.platform.add_period_constraint(eth_tx_clk, 1e9/phy.tx_clk_freq)
                    self.platform.add_false_path_constraints(self.crg.cd_sys.clk, eth_rx_clk, eth_tx_clk)
                else:
                    self.platform.add_false_path_constraints(self.crg.cd_sys.clk, eth_rx_clk)

    # Add PCIe -------------------------------------------------------------------------------------
    def __add_pcie(self, name="pcie", phy=None, ndmas=0, max_pending_requests=8, address_width=32, data_width=None,
        with_dma_buffering    = True, dma_buffering_depth=1024,
        with_dma_loopback     = True,
        with_dma_synchronizer = False,
        with_dma_monitor      = False,
        with_dma_status       = False,
        with_msi              = True, msi_type="msi", msi_width=32,
        with_ptm              = False,
        with_pcie_eth         = True):
        # Imports
        from litepcie.phy.uspciephy import USPCIEPHY
        from litepcie.phy.usppciephy import USPPCIEPHY
        from litepcie.core import LitePCIeEndpoint, LitePCIeMSI, LitePCIeMSIMultiVector, LitePCIeMSIX
        from litepcie.frontend.dma import LitePCIeDMA
        from litepcie.frontend.wishbone import LitePCIeWishboneMaster

        # Checks.
        assert self.csr.data_width == 32

        # Endpoint.
        self.check_if_exists(f"{name}_endpoint")
        endpoint = LitePCIeEndpoint(phy,
            max_pending_requests = max_pending_requests,
            endianness           = phy.endianness,
            address_width        = address_width,
            with_ptm             = with_ptm,
        )
        self.add_module(name=f"{name}_endpoint", module=endpoint)

        # MMAP.
        self.check_if_exists(f"{name}_mmap")
        mmap = LitePCIeWishboneMaster(self.pcie_endpoint, base_address=self.mem_map["csr"])
        self.add_module(name=f"{name}_mmap", module=mmap)
        self.bus.add_master(name=f"{name}_mmap", master=mmap.wishbone)

        # MSI.
        if with_msi:
            assert msi_type in ["msi", "msi-multi-vector", "msi-x"]
            self.check_if_exists(f"{name}_msi")
            if msi_type == "msi":
                msi = LitePCIeMSI(width=msi_width)
            if msi_type == "msi-multi-vector":
                msi = LitePCIeMSIMultiVector(width=msi_width)
            if msi_type == "msi-x":
                msi = LitePCIeMSIX(endpoint=self.pcie_endpoint, width=msi_width)
            self.add_module(name=f"{name}_msi", module=msi)
            if msi_type in ["msi", "msi-multi-vector"]:
                self.comb += msi.source.connect(phy.msi)
            self.msis = {}

            if with_pcie_eth:
                self.msis["ETHRX"] = self.ethmac.rx_pcie_irq
                self.msis["ETHTX"] = self.ethmac.tx_pcie_irq

        # DMAs.
        for i in range(ndmas):
            assert with_msi
            self.check_if_exists(f"{name}_dma{i}")
            dma = LitePCIeDMA(phy, endpoint,
                with_buffering    = with_dma_buffering, buffering_depth=dma_buffering_depth,
                with_loopback     = with_dma_loopback,
                with_synchronizer = with_dma_synchronizer,
                with_monitor      = with_dma_monitor,
                with_status       = with_dma_status,
                address_width     = address_width,
                data_width        = data_width,
                with_table        = not with_pcie_eth,
            )
            self.add_module(name=f"{name}_dma{i}", module=dma)
            if not with_pcie_eth:
                self.msis[f"{name.upper()}_DMA{i}_WRITER"] = dma.writer.irq
                self.msis[f"{name.upper()}_DMA{i}_READER"] = dma.reader.irq
        self.add_constant("DMA_CHANNELS",   ndmas)
        self.add_constant("DMA_ADDR_WIDTH", address_width)

        # Map/Connect IRQs.
        if with_msi:
            for i, (k, v) in enumerate(sorted(self.msis.items())):
                self.comb += msi.irqs[i].eq(v)
                self.add_constant(k + "_INTERRUPT", i)

        # Timing constraints.
        self.platform.add_false_path_constraints(self.crg.cd_sys.clk, phy.cd_pcie.clk)

    def add_ethernet_pcie(self, name="ethmac", phy=None, pcie_phy=None, phy_cd="eth", dynamic_ip=False,
                          software_debug=False,
                          nrxslots=32,
                          ntxslots=32,
                          with_timing_constraints=True,
                          max_pending_requests=8,
                          with_msi=True):
        # Imports
        from litex.soc.interconnect import wishbone

        data_width = 128

        self.submodules.pcie_mem_bus_rx = SoCBusHandler(
            data_width=data_width
        )
        self.submodules.pcie_mem_bus_tx = SoCBusHandler(
            data_width=data_width
        )
        
        # MAC.
        self.__add_ethernet(
            name                    = name,
            phy                     = phy,
            phy_cd                  = phy_cd,
            dynamic_ip              = dynamic_ip,
            software_debug          = software_debug,
            data_width              = data_width,
            nrxslots                = nrxslots,
            ntxslots                = ntxslots, 
            with_timing_constraints = with_timing_constraints,
            with_pcie_eth           = True
        )

        # PCIe
        self.__add_pcie(name="pcie", phy=pcie_phy,
            ndmas                = 1,
            max_pending_requests = max_pending_requests,
            data_width           = data_width,
            with_dma_buffering   = False,
            with_dma_loopback    = False,
            with_msi             = with_msi,
            with_ptm             = False,
            with_pcie_eth        = True
        )

        from gateware.litepcie.wishbone_dma import LitePCIe2WishboneDMANative, LiteWishbone2PCIeDMANative, PCIeInterruptTest
        pcie_host_wb2pcie_dma = LiteWishbone2PCIeDMANative(self.pcie_endpoint, self.pcie_dma0.writer, data_width)
        self.pcie_host_wb2pcie_dma = pcie_host_wb2pcie_dma
        self.pcie_mem_bus_rx.add_master("pcie_master_wb2pcie", pcie_host_wb2pcie_dma.bus_wr)
        pcie_host_pcie2wb_dma = LitePCIe2WishboneDMANative(self.pcie_endpoint, self.pcie_dma0.reader, data_width)
        self.pcie_host_pcie2wb_dma = pcie_host_pcie2wb_dma
        self.pcie_mem_bus_tx.add_master("pcie_master_pcie2wb", pcie_host_pcie2wb_dma.bus_rd)

        align_bits = log2_int(512)
        self.comb += [
            self.pcie_host_wb2pcie_dma.bus_addr.eq(self.ethmac_region_rx.origin + self.ethmac.interface.sram.writer.stat_fifo.source.slot * self.ethmac.slot_size.constant),
            self.pcie_host_wb2pcie_dma.host_addr.eq(self.ethmac.interface.sram.writer.pcie_host_addr),
            self.pcie_host_wb2pcie_dma.length.eq(Cat(Signal(align_bits,reset=0), (self.ethmac.interface.sram.writer.stat_fifo.source.length[align_bits:] + 1))),
            self.pcie_host_wb2pcie_dma.start.eq(self.ethmac.interface.sram.writer.start_transfer),
            self.ethmac.interface.sram.writer.transfer_ready.eq(self.pcie_host_wb2pcie_dma.ready),
        ]
        self.comb += [
            self.pcie_host_pcie2wb_dma.bus_addr.eq(self.ethmac_region_tx.origin + self.ethmac.interface.sram.reader.cmd_fifo.source.slot * self.ethmac.slot_size.constant),
            self.pcie_host_pcie2wb_dma.host_addr.eq(self.ethmac.interface.sram.reader.pcie_host_addr),
            self.pcie_host_pcie2wb_dma.length.eq(Cat(Signal(align_bits, reset=0), (self.ethmac.interface.sram.reader.cmd_fifo.source.length[align_bits:] + 1))),
            self.pcie_host_pcie2wb_dma.start.eq(self.ethmac.interface.sram.reader.start_transfer),
            self.ethmac.interface.sram.reader.transfer_ready.eq(self.pcie_host_pcie2wb_dma.ready),
        ]

        self.submodules.bus_interconnect_tx = wishbone.InterconnectPointToPoint(
            master=next(iter(self.pcie_mem_bus_tx.masters.values())),
            slave=next(iter(self.pcie_mem_bus_tx.slaves.values())))

        self.submodules.bus_interconnect_rx = wishbone.InterconnectPointToPoint(
            master=next(iter(self.pcie_mem_bus_rx.masters.values())),
            slave=next(iter(self.pcie_mem_bus_rx.slaves.values())))


    def generate_software_header(self, dst):
        csr_header = get_csr_header(self.csr_regions, self.constants, with_access_functions=False)
        tools.write_to_file(os.path.join(dst, "csr.h"), csr_header)
        self_header = get_soc_header(self.constants, with_access_functions=False)
        tools.write_to_file(os.path.join(dst, "soc.h"), self_header)
        mem_header = get_mem_header(self.mem_regions)
        tools.write_to_file(os.path.join(dst, "mem.h"), mem_header)
