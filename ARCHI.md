# gateware/litepcie/wishbone_dma.py:

## WishboneDMAReaderCtrl
The `WishboneDMAReaderCtrl` module is responsible for reading data from the Wishbone MMAP memory and sending it to a destination. It inherits from `WishboneDMAReader`.

### Parameters:
- `bus`: The Wishbone bus of the SoC to read from.
- `endianness`: Specifies the byte order for reading.

### Attributes:
- `sink`: Receives addresses to read.
- `source`: Outputs the read data.

### Functionality:
- The `add_ctrl` method initializes various control signals (base address, length, enable, etc.) and sets up a Finite State Machine (FSM) to manage the reading process. The FSM transitions between `IDLE`, `RUN`, and `DONE` states, handling address incrementing and looping if required.

## WishboneDMAWriterCtrl
The `WishboneDMAWriterCtrl` module writes data to the Wishbone MMAP memory. It extends `WishboneDMAWriter`.

### Parameters:
- `bus`: The Wishbone bus to write to.
- `endianness`: Specifies the byte order for writing.

### Attributes:
- `sink`: Receives addresses and data to write.

### Functionality:
- The `add_ctrl` method configures control signals and an FSM similar to `WishboneDMAReaderCtrl`. The FSM manages writing data to the specified addresses and handles looping if needed.

## dma_descriptor_layout
This function defines the layout for DMA descriptors, specifying the fields for host address, bus address, and length. This layout is used to describe data transfers.

## LiteWishbone2PCIeDMA
The `LiteWishbone2PCIeDMA` module handles DMA transfers from the Wishbone bus to the PCIe interface.

### Parameters:
- `endpoint`: The PCIe endpoint.
- `data_width`: The data width of the bus.

### Functionality:
- Integrates various submodules (`dma_wr`, `dma_fifo`, `fifo_wr`, `wb_dma`, `conv_wr`) to manage the flow of data from the Wishbone bus to PCIe.
- Uses control registers (CSR) for configuration (host address, length, bus address, etc.).
- An FSM controls the state of the DMA transfer, ensuring proper data flow and signaling completion via interrupts.

## LitePCIe2WishboneDMA
The `LitePCIe2WishboneDMA` module handles DMA transfers from the PCIe interface to the Wishbone bus.

### Parameters:
- `endpoint`: The PCIe endpoint.
- `data_width`: The data width of the bus.

### Functionality:
- Integrates submodules (`dma_rd`, `dma_fifo`, `fifo_rd`, `wb_dma`, `conv_rd`) to manage data flow from PCIe to the Wishbone bus.
- Uses CSR for configuration and control.
- An FSM manages the transfer states, ensuring data is correctly written to the Wishbone memory and signaling completion via interrupts.

## PCIeInterruptTest
This module is a simple test for PCIe interrupts, providing CSR to trigger three different IRQ signals.

## LiteWishbone2PCIeDMANative
The `LiteWishbone2PCIeDMANative` module is similar to `LiteWishbone2PCIeDMA` but focuses on native DMA transfer without additional descriptor tables.

### Functionality:
- Manages DMA transfers directly using control signals and FSM.
- Simplifies the process by eliminating intermediate descriptor tables, directly linking Wishbone memory read operations to PCIe writes.

## LitePCIe2WishboneDMANative
The `LitePCIe2WishboneDMANative` module handles native DMA transfers from PCIe to Wishbone.

### Functionality:
- Directly manages DMA transfers from PCIe reads to Wishbone writes.
- Uses control signals and an FSM to streamline data transfer without intermediate descriptors.

# gateware/liteeth/mac/sram.py:

## LiteEthMACSRAMWriter
The `LiteEthMACSRAMWriter` module is responsible for writing Ethernet frames to SRAM. It handles incoming packets and stores them in memory, including CRC error checking and optional timestamping.

### Parameters:
- `dw`: Data width.
- `depth`: Depth of the SRAM.
- `nslots`: Number of slots for buffering.
- `endianness`: Byte order (big or little).
- `timestamp`: Optional timestamp signal for packets.
- `with_eth_pcie`: Enable PCIe integration.

### Attributes:
- `sink`: Endpoint for incoming Ethernet packets.
- `crc_error`: Signal indicating CRC errors.
- Various CSR registers for status, error count, and control.

### Functionality:
- Sets up a finite state machine (FSM) to manage packet writing, error handling, and discarding invalid packets.
- Supports optional PCIe integration with control and status signals.
- Manages memory writes with endianness handling and multi-slot buffering.

## LiteEthMACSRAMReader
The `LiteEthMACSRAMReader` module reads Ethernet frames from SRAM and outputs them through a source endpoint. It supports optional timestamping and PCIe integration.

### Parameters:
- `dw`: Data width.
- `depth`: Depth of the SRAM.
- `nslots`: Number of slots for buffering.
- `endianness`: Byte order (big or little).
- `timestamp`: Optional timestamp signal for packets.
- `with_eth_pcie`: Enable PCIe integration.

### Attributes:
- `source`: Endpoint for outgoing Ethernet packets.
- Various CSR registers for control and status.

### Functionality:
- Uses an FSM to manage reading from memory, handling packet lengths, and triggering events.
- Supports optional PCIe integration with control and status signals.
- Manages memory reads with endianness handling and multi-slot buffering.

## LiteEthMACSRAM
The `LiteEthMACSRAM` module combines the writer and reader modules to create a complete SRAM-based Ethernet MAC. It supports both packet transmission and reception, with optional PCIe integration.

### Parameters:
- `dw`: Data width.
- `depth`: Depth of the SRAM.
- `nrxslots`: Number of RX slots.
- `ntxslots`: Number of TX slots.
- `endianness`: Byte order (big or little).
- `timestamp`: Optional timestamp signal for packets.
- `with_eth_pcie`: Enable PCIe integration.

### Attributes:
- `sink`: Endpoint for incoming Ethernet packets (connected to writer).
- `source`: Endpoint for outgoing Ethernet packets (connected to reader).

### Functionality:
- Integrates the `LiteEthMACSRAMWriter` and `LiteEthMACSRAMReader` modules.
- Provides optional PCIe integration with IRQ handling.
- Supports shared IRQ for both writer and reader when PCIe is not used.

# gateware/liteeth/mac/wishbone.py:

## LiteEthMACWishboneInterface
The `LiteEthMACWishboneInterface` module facilitates the integration of Ethernet MAC functionality with a Wishbone bus interface. It handles the storage and transfer of Ethernet frames using SRAM and connects to the Wishbone bus for memory-mapped access.

### Parameters:
- `dw`: Data width.
- `nrxslots`: Number of RX slots.
- `ntxslots`: Number of TX slots.
- `endianness`: Byte order (big or little).
- `timestamp`: Optional timestamp signal for packets.
- `rxslots_read_only`: If RX slots should be read-only.
- `txslots_write_only`: If TX slots should be write-only.
- `with_pcie_eth`: Enable PCIe Ethernet integration.

### Attributes:
- `sink`: Endpoint for incoming Ethernet packets.
- `source`: Endpoint for outgoing Ethernet packets.
- `rx_bus`: Wishbone interface for RX when PCIe Ethernet is enabled.
- `tx_bus`: Wishbone interface for TX when PCIe Ethernet is enabled.
- `bus`: Wishbone interface for non-PCIe Ethernet configurations.

### Functionality:
- The module sets up storage for Ethernet frames in SRAM with configurable depth based on the Ethernet MTU and data width.
- It instantiates the `LiteEthMACSRAM` module to manage the actual SRAM storage for both RX and TX operations.
- Connections are made between the sink/source endpoints and the `LiteEthMACSRAM` module for packet handling.
- Depending on the `with_pcie_eth` parameter, it configures Wishbone interfaces for RX and TX slots, creating separate SRAM interfaces for each slot.
- Exposes these SRAM interfaces on a single Wishbone bus using the `wishbone.SRAM` and `wishbone.Decoder` modules for both RX and TX operations.
- For PCIe Ethernet configurations, separate Wishbone buses are used for RX and TX, with corresponding decoders.
