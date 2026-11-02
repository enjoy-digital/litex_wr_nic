# AGENTS.md: LiteX-WR-NIC

LiteX based White Rabbit PCIe NIC: White Rabbit synchronization (slave, master, grandmaster) with
PPS/10MHz outputs, a 1G LiteEth NIC with Linux driver, and PCIe PTM to share the WR time with the host.

## Where things are

- Targets: `spec_a7_wr_nic.py` (SPEC-A7), `acorn_wr_nic.py` (LiteX Acorn Baseboard Mini), `hyvision_pcie_opt01_revf.py`, `tang_mega_138k_pro_wr.py`; LiteEth-only NICs `spec_a7_liteeth_nic.py`, `acorn_liteeth_nic.py`; `spec_a7_bringup.py`.
- `litex_wr_nic/`: WR core integration, gateware, firmware, software; `tools/`; `doc/` (`boards.md`, `spec_a7_grandmaster.md`, `tang_mega_138k_pro.md`, datasheets).
- `test/`: Python tests and HDL simulations.

## Verify a change

```sh
python3 -m pytest -v test                                           # tests (no CI workflow; 260 tests)
python3 spec_a7_wr_nic.py --help                                    # target options
python3 spec_a7_wr_nic.py --build                                   # gateware build (vendor toolchain)
```

- WR locking needs two boards and a WR peer: check that the peer is not also in slave mode, and
  that the VCXO tuning center matches the board's crystal.
- The WR CPU (uRV, VexRiscv or AE350) runs from local memory for performance; changes to its bus or
  caches change the WR timing loops.

## LiteX

This project is built with LiteX: read https://github.com/enjoy-digital/litex/wiki/LiteX-for-AI-Agents (framework map, conventions,
how to verify a change, contribute back the generic fixes to LiteX and its cores).

## Debug on hardware

Method: https://github.com/enjoy-digital/litex/wiki/Debug-A-SoC-On-Hardware (check the link and the SoC identity first, registers
before traces, narrow LiteScope captures, read the generated Verilog, turn findings into tests).
`litex_mcp` gives an agent the same loop as MCP tools through `litex_server` (CSRs by name, LiteScope
captures as data, console, builds with parsed timing, simulation; writes and `--load`/`--flash`
only with `--allow-write`): https://github.com/enjoy-digital/litex/wiki/LiteX-for-AI-Agents#drive-a-soc-through-mcp.

## Rules

- LiteX style: https://github.com/enjoy-digital/litex/blob/master/doc/coding_style.md.
- Never edit generated files (`build/`, `csr.h`, `csr.csv`, generated Verilog): change the Python.
- Never flash a board or write its SPI flash unless the user asked; `--load` (volatile) for experiments.
- Never commit flash dumps, stock firmware or vendor bitstreams.
- Pull requests: one topic each, with what was tested and on which hardware; say that an agent wrote
  it and who supervised.
