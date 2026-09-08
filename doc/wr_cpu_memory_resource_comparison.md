# WR CPU Memory Resource Comparison

Target: `spec_a7_wr_nic` / `xc7a50tcsg325-2`
Tool: `Vivado v.2024.1 (lin64) Build 5076996 Wed May 22 18:36:09 MDT 2024`
Git: `cee1cb39fa171af9c3981647d81ae95c62f688f3` (dirty)
Firmware SHA-256: `ca79808b467881257f3df500c400dae7302cde619fc0ca252dba14939b996727`

HyperRAM: `8 KiB cache, 4:1, 31.25 MHz`

| Metric | Private | Integrated | Δ integrated | HyperRAM | Δ HyperRAM |
|---|---:|---:|---:|---:|---:|
| Slice LUTs | 12346 | 12370 | +24 (+0.2%) | 12978 | +632 (+5.1%) |
| Logic LUTs | 11961 | 11911 | -50 (-0.4%) | 12499 | +538 (+4.5%) |
| LUT memory | 385 | 459 | +74 (+19.2%) | 479 | +94 (+24.4%) |
| Flip-flops | 14855 | 15141 | +286 (+1.9%) | 15933 | +1078 (+7.3%) |
| RAMB36 | 65 | 65 | +0 (+0.0%) | 34 | -31 (-47.7%) |
| RAMB18 | 16 | 16 | +0 (+0.0%) | 21 | +5 (+31.2%) |
| BRAM18 equivalents | 146 | 146 | +0 (+0.0%) | 89 | -57 (-39.0%) |
| DSPs | 8 | 8 | +0 (+0.0%) | 8 | +0 (+0.0%) |
| Bitstream bytes | 1792872 | 1863912 | +71040 (+4.0%) | 1593740 | -199132 (-11.1%) |
| WNS (ns) | 0.052 | -0.045 | -0.097 | 0.0 | -0.052 |
| WHS (ns) | 0.028 | 0.028 | +0.000 | 0.028 | +0.000 |
| Build time (s) | 419.92 | 425.327 | +5.407 | 467.467 | +47.547 |

## Result

Integrated RAM changes block-memory use by +0 BRAM18 equivalents; HyperRAM changes it by -57. A negative delta is the FPGA block-memory saving.

Timing status: private MET (+0.052 ns WNS), integrated VIOLATED (-0.045 ns WNS), hyperram MET (+0.000 ns WNS).

All resource figures are from placed designs; timing is from the final timing report.
