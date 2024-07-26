# gateware/liteeth/mac/sram.py:

#### General Overview

1. **Enhanced Constructor**:
   - Added a `with_eth_pcie` parameter to enable or disable PCIe integration features.
   - Extended data width support to include `128-bit`.

2. **Extended Data Width Support**:
   - Ensured compatibility for 128-bit data width.

3. **Additional CSRs and Signals**:
   - Introduced new CSR registers and signals for managing PCIe-related operations, such as:
     - Enabling/disabling transfers.
     - Handling pending slots.
     - Managing PCIe host addresses.

4. **Conditional Signal Enable**:
   - Conditional handling for the `enable` signal based on whether PCIe integration (`with_eth_pcie`) is enabled or not.

5. **Extended Length Increment Signal**:
   - Increased the bit width of the `length_inc` signal to support higher increments, suitable for 128-bit data width.

6. **Additional Length Increment Cases**:
   - Added more cases in the length increment logic to handle increments up to `128-bit`.

7. **FSM State Transitions**:
   - Modified the Finite State Machine (FSM) to include additional states and logic specific to PCIe integration, such as managing start and wait for transfers.

8. **Event Manager for PCIe**:
   - Added an event manager and FSM for handling PCIe-specific operations, including:
     - Transfer start signals.
     - Transfer ready signals.
     - IRQ handling for PCIe transfers.

9. **Integration in `LiteEthMACSRAM`**:
   - Incorporated PCIe IRQ handling in the combined SRAM module to facilitate seamless integration of PCIe and Ethernet functionalities.

#### Summary

These changes enhance the `sram.py` module to support PCIe integration, providing the following benefits:
- Compatibility with 128-bit data width.
- New CSR registers and signals to manage PCIe-specific operations.
- Enhanced FSM logic to handle PCIe transfers.
- PCIe IRQ handling integrated within the `LiteEthMACSRAM` module.

# gateware/liteeth/mac/wishbone.py:

#### General Overview

1. **Import Path Change**:
   - Adjusted the import path for the `sram` module to match the new project structure.

2. **Constructor Modification**:
   - Added a `with_pcie_eth` parameter to enable or disable PCIe-specific configurations.

3. **PCIe-Specific Wishbone Interfaces**:
   - Introduced separate Wishbone interfaces for RX and TX when PCIe Ethernet integration is enabled.

4. **Bus Initialization**:
   - Conditional initialization of Wishbone buses based on the `with_pcie_eth` parameter.

5. **Offsets for PCIe**:
   - Defined offsets (`wait_ack_offset` and `tx_ready_offset`) for PCIe integration.

6. **Decoder Configuration**:
   - Modified decoder configuration to handle PCIe-specific bus structures.
   - Conditional setup of Wishbone decoders for RX and TX slots based on PCIe integration.

#### Summary

These changes enhance the `wishbone.py` module to support PCIe integration, providing the following benefits:
- Introduced new parameters and interfaces to manage PCIe-specific configurations.
- Conditional initialization and handling of Wishbone buses for RX and TX operations.
- Enhanced decoder configuration to support PCIe-based data transfers.
