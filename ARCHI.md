# Project Overview: LitePCIe and LiteEth Integration

## Goals
The primary goal of this project is to integrate Ethernet MAC functionality with PCIe capabilities within an FPGA-based System-on-Chip (SoC) design. This integration aims to provide efficient data transfer and communication between Ethernet and PCIe subsystems using the LiteX framework, leveraging Wishbone interfaces and DMA engines for optimal performance.

## Architecture
The project architecture revolves around two main subsystems: Ethernet MAC and PCIe, interconnected through Wishbone buses. The integration ensures seamless communication and high-speed data transfer between these subsystems.

### Main Components
1. **Ethernet MAC (LiteEth)**
   - Handles Ethernet frame transmission and reception.
   - Uses SRAM for packet buffering.
   - Connects to the Wishbone bus for memory-mapped access.

2. **PCIe Subsystem (LitePCIe)**
   - Provides high-speed data transfer capabilities via PCIe.
   - Includes endpoint and DMA engines for efficient communication.
   - Connects to the Wishbone bus for memory-mapped access.

3. **Wishbone Bus**
   - Acts as the primary interconnect for memory-mapped peripherals.
   - Connects Ethernet MAC and PCIe subsystems to the CPU and other SoC components.

4. **DMA Engines**
   - Facilitate direct memory access between PCIe and Wishbone interfaces.
   - Ensure high-performance data transfer.

5. **MSI (Message Signaled Interrupts)**
   - Manage interrupt signals for efficient event handling, particularly for Ethernet RX/TX operations.

### Component Descriptions

#### WishboneDMAReaderCtrl
- Reads data from Wishbone MMAP memory and sends it to a destination.
- Manages control signals and data flow using a Finite State Machine (FSM).

#### WishboneDMAWriterCtrl
- Writes data to Wishbone MMAP memory.
- Uses an FSM to manage writing operations, control signals, and looping.

#### dma_descriptor_layout
- Defines the layout for DMA descriptors used in data transfers.

#### LiteWishbone2PCIeDMA
- Handles DMA transfers from the Wishbone bus to the PCIe interface.
- Manages data flow, configuration, and control using various submodules and CSRs.

#### LitePCIe2WishboneDMA
- Handles DMA transfers from the PCIe interface to the Wishbone bus.
- Uses submodules and CSRs for configuration and control, ensuring proper data flow.

#### PCIeInterruptTest
- Tests PCIe interrupts by providing CSR to trigger IRQ signals.

#### LiteWishbone2PCIeDMANative
- Manages native DMA transfers from Wishbone to PCIe without additional descriptor tables.
- Directly links Wishbone memory read operations to PCIe writes.

#### LitePCIe2WishboneDMANative
- Handles native DMA transfers from PCIe to Wishbone.
- Uses control signals and FSM for streamlined data transfer.

#### LiteEthMACSRAMWriter
- Writes Ethernet frames to SRAM.
- Manages packet writing, error handling, and optional timestamping.

#### LiteEthMACSRAMReader
- Reads Ethernet frames from SRAM and outputs them through a source endpoint.
- Supports optional timestamping and PCIe integration.

#### LiteEthMACSRAM
- Combines writer and reader modules for complete SRAM-based Ethernet MAC functionality.
- Supports both packet transmission and reception with optional PCIe integration.

#### LiteEthMACWishboneInterface
- Integrates Ethernet MAC functionality with a Wishbone bus interface.
- Manages Ethernet frame storage and transfer using SRAM and connects to the Wishbone bus.

## Integration Methods

### EthernetPCIeSoC Class
This class integrates Ethernet MAC and PCIe functionalities into a cohesive SoC design.

#### CSR Map
Maps various components to Control and Status Registers (CSR) for software configuration and control.

#### `__add_ethernet` Method
Adds an Ethernet MAC module to the SoC, configuring its parameters and connecting it to the appropriate interfaces. It also configures memory regions and Wishbone interfaces for RX and TX slots.

#### `__add_pcie` Method
Adds a PCIe subsystem to the SoC, configuring the PCIe PHY, endpoint, DMA engines, and MSI. It manages memory mapping and interrupt handling for efficient data transfer.

#### `add_ethernet_pcie` Method
Integrates both Ethernet and PCIe subsystems in the SoC, ensuring proper configuration and connectivity. It sets up interconnections between Ethernet MAC and PCIe DMA engines for seamless data flow.

#### `generate_software_header` Method
Generates software headers for CSR, SoC, and memory regions, enabling software access to hardware configurations.

## Global Architecture and Interconnections
The `EthernetPCIeSoC` class integrates Ethernet MAC and PCIe subsystems into a unified SoC architecture with the following components:

1. **Ethernet MAC**: Manages Ethernet frame transmission and reception using SRAM for buffering and Wishbone interfaces for access.
2. **PCIe Subsystem**: Provides high-speed data transfer with endpoint and DMA engines connected to the Wishbone bus.
3. **Wishbone Bus**: Acts as the main interconnect for peripherals, ensuring communication between Ethernet MAC, PCIe, CPU, and other components.
4. **DMA Engines**: Enable direct memory access between PCIe and Wishbone, ensuring efficient data transfer.
5. **MSI**: Manages interrupts for handling Ethernet RX/TX operations efficiently.

The integration ensures high-performance data transfer and communication between Ethernet and PCIe subsystems, leveraging DMA engines and Wishbone interfaces for optimal performance and flexibility.
