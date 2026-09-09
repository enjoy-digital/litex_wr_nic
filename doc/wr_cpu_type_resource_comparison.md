# WR CPU Type Resource Comparison

- Target: `spec_a7_wr_nic` / `xc7a50tcsg325-2`
- Tool: `Vivado v.2024.1 (lin64) Build 5076996 Wed May 22 18:36:09 MDT 2024`
- Git: `20b3641a6bdf46779ce5e05717d46bc0df071772` (dirty)
- CPU clock: `62.5 MHz`

## Integrated memory

| Metric | uRV | VexRiscv lite | VexRiscv Δ |
|---|---:|---:|---:|
| Slice LUTs | 12361 | 12292 | -69 (-0.6%) |
| Logic LUTs | 11902 | 11837 | -65 (-0.5%) |
| LUT memory | 459 | 455 | -4 (-0.9%) |
| Flip-flops | 15141 | 15167 | +26 (+0.2%) |
| RAMB36 | 65 | 65 | +0 (+0.0%) |
| RAMB18 | 16 | 18 | +2 (+12.5%) |
| BRAM18 equivalents | 146 | 148 | +2 (+1.4%) |
| DSPs | 8 | 4 | -4 (-50.0%) |
| Bitstream bytes | 1829716 | 1810396 | -19320 (-1.1%) |
| WR clock WNS (ns) | 4.54 | 4.413 | -0.127 |
| WNS (ns) | 0.004 | -0.011 | -0.015 |
| WHS (ns) | 0.028 | 0.028 | +0.000 |
| Build time (s) | 316.442 | 446.811 | +130.369 |

Resource result: VexRiscv changes Slice LUTs by -69 (-0.6%), DSPs by -4 (-50.0%), and BRAM18 equivalents by +2 (+1.4%).

Timing: uRV MET (+0.004 ns WNS), VexRiscv VIOLATED (-0.011 ns WNS).

WR-clock timing: uRV MET (+4.540 ns WNS), VexRiscv MET (+4.413 ns WNS).

## HyperRAM memory

| Metric | uRV | VexRiscv lite | VexRiscv Δ |
|---|---:|---:|---:|
| Slice LUTs | 12975 | 12754 | -221 (-1.7%) |
| Logic LUTs | 12496 | 12277 | -219 (-1.8%) |
| LUT memory | 479 | 477 | -2 (-0.4%) |
| Flip-flops | 15933 | 15970 | +37 (+0.2%) |
| RAMB36 | 34 | 34 | +0 (+0.0%) |
| RAMB18 | 21 | 23 | +2 (+9.5%) |
| BRAM18 equivalents | 89 | 91 | +2 (+2.2%) |
| DSPs | 8 | 4 | -4 (-50.0%) |
| Bitstream bytes | 1611720 | 1594152 | -17568 (-1.1%) |
| WR clock WNS (ns) | 5.282 | 5.102 | -0.180 |
| WNS (ns) | -0.144 | -0.527 | -0.383 |
| WHS (ns) | 0.028 | 0.028 | +0.000 |
| Build time (s) | 475.554 | 431.982 | -43.572 |

Resource result: VexRiscv changes Slice LUTs by -221 (-1.7%), DSPs by -4 (-50.0%), and BRAM18 equivalents by +2 (+2.2%).

Timing: uRV VIOLATED (-0.144 ns WNS), VexRiscv VIOLATED (-0.527 ns WNS).

WR-clock timing: uRV MET (+5.282 ns WNS), VexRiscv MET (+5.102 ns WNS).

All utilization figures are from placed designs; timing is from the final timing report.
