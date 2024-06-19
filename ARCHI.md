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
