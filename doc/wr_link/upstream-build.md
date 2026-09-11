# Complete upstream WR rebuild

The original September 11 qualification used older project pins with backported
fixes. The upstream rebuild uses complete source trees from the official
development repositories, including their pinned submodules:

| Source | Revision fetched on September 11, 2026 |
| --- | --- |
| [WR-core](https://gitlab.com/ohwr/project/wr-cores) | `8cc5e53275d229fbbf50b96a6ae4cb9c87626d53` |
| [WRPC](https://gitlab.com/ohwr/project/wrpc-sw) | `13527cd68e1833214a89e4ee8c5b208188ff0e6a` |
| Firmware PPSI | `33d8c46c8353be56d35dc57dd3492769e276c00f` |
| WR-core general-cores | `d99c8460ae8b939ada406fe23198c41d236bf49b` |
| WR-core uRV | `de05caf973e17d1fa46f4fdb95762230bd1ab8c4` |

WR-core also pins a WRPC submodule at `7e5ab46665b65b94a37dad54f0dd713edc65f552`.
That submodule is initialized exactly as upstream specifies; the firmware used
in these images is compiled separately from the newer WRPC revision listed
above by `litex_wr_nic/firmware/build.py`. No prebuilt upstream firmware image
is substituted for that compilation.

The LiteX integration still provides its board/PHY adapters and CPU-memory
interfaces. The upstream CPU wrapper now has a BRAM-macro generic, which is
preserved when adding those interfaces. The unused local `wr_core.vhd` copy
was removed in favor of the upstream file. The SPI MOSI fix is already present
upstream. The diagnostic address, host diagnostic control-word, and SFP storage
error fixes remain necessary and are applied to the current sources.

Use separate checkouts for the two boards and the commands in the
[USB qualification guide](../wr_link_qualification.md). If an existing
`wr-cores/` checkout has an older revision, move it aside before rebuilding.
The build rejects mismatched WR-core and submodule revisions without discarding
local changes. Initializing the pinned tree uses the current HTTPS GitLab URLs
and recursively initializes its submodules.

## SPEC implementation settings

The first complete upstream SPEC build missed setup timing by 0.276 ns on the
250 MHz PCIe PTM sniffer path from packet alignment through filtering to the
FIFO write enable. WR clock domains met timing. Re-optimizing that routed
design did not resolve the violation.

Using `ExtraTimingOpt` placement followed by `AggressiveExplore` physical
optimization before routing met timing in the placement experiment, with
+0.083 ns setup and +0.053 ns hold slack. The SPEC target now requests those
settings through LiteX's Vivado backend. The RTL and clock requirements are
unchanged by this implementation adjustment.

A fresh build with those settings still missed timing by 0.037 ns. Further
physical optimization reduced the violation to 0.006 ns, but repeated passes
and timing cleanup did not close it. The remaining path passed through the
packet classifier to the FIFO data input: the filter assigned data inside its
valid conditions even though the FIFO already gates writes with valid/ready.

[LitePCIe PR 186](https://github.com/enjoy-digital/litepcie/pull/186) drives that
data input directly and retains the existing write-enable logic. This removes
the redundant gating without adding latency. Packet tests cover request and
response messages, unrelated packets, gaps and backpressure; a separate
12,000-cycle comparison against the original code matched all valid output
fields and their cycle timing.

The subsequent Acorn and SPEC builds use LitePCIe
`f9a2d43e4e9c29ae837641fb1c86cc0ffdec5da4` from that PR. Use that revision in
the LitePCIe checkout imported by Python when reproducing these builds. On
this bench, `PYTHONPATH` selected an isolated checkout for both build commands.

## UART formatter overflow

The first timing-passing upstream images booted the expected WRPC revision,
but the console repeatedly reported `PRINTF OVF`. The project's configuration
allocated only 16 bytes for `pp_printf`'s static output buffer. The formatter
uses unbounded `pp_vsprintf`; even the firmware version line writes beyond that
buffer. The earlier pinned WRPC revision had the same unbounded write but did
not emit the overflow warning. This was a pre-existing project configuration
bug exposed by the newer firmware, and the affected hardware run was rejected.

The project now uses a 256-byte buffer, matching upstream's standard SPEC
configuration. A regression compiles the actual upstream formatter with
AddressSanitizer: the 16-byte negative control reports a global buffer overflow,
while the project configuration formats version, role and 64-bit numeric output
successfully. The build review checks that `print_buf` occupies 256 bytes in
each compiled ELF. The qualifier rejects overflow warnings in boot, command
responses and monitor output. Both FPGA images are rebuilt to embed the
corrected firmware.

The corrected Acorn build meets final setup/hold timing at +0.174/+0.036 ns;
SPEC meets it at +0.045/+0.057 ns. Both role directions passed 30 uninterrupted
minutes of WR phase tracking, three PCS interruption recoveries and three
paired SRAM reload recoveries. All twelve recoveries completed within 53 seconds.
Raw UART replay and the snapshot audit passed for both directions.

The [complete upstream hardware results](results-upstream/README.md) contain
the plots, image/firmware hashes, exact source provenance and replayable raw
evidence. The older pinned-source results remain separately labeled as
historical evidence, including their now-known console-buffer defect.
