# Prepared upstream patches

These patches apply to the official GitLab repositories and contain independent
fixes discovered during Acorn/SPEC-A7 WR qualification. GitLab HTTPS/API credentials
were unavailable and existing SSH authentication was rejected. No GitLab merge
request has been submitted from this environment.

| Patch | Repository / tested upstream base | Local commit |
| --- | --- | --- |
| [Diagnostic CTRL selector](wr-cores-diag-control.patch) | [wr-cores](https://gitlab.com/ohwr/project/wr-cores), `8f9b06565ef8d23712494314def63f6fc3813f8c` | `628cfa0792c74bab83fed378b54ac0dbaa288436` |
| [CPU diagnostic address](wrpc-diag-address.patch) | [wrpc-sw](https://gitlab.com/ohwr/project/wrpc-sw), `13527cd68e1833214a89e4ee8c5b208188ff0e6a` | `225231378741cd3ded7132778b2cde90633e1698` |
| [SFP storage error handling](wrpc-sfp-storage-errors.patch) | Same WRPC base; may be applied independently | `d49516e520d95a0defcc9df44253e086ca1aa6f4` |

Apply with `git am <patch>` in the corresponding repository and open a focused
merge request using the commit description. The WR diagnostic-selector GHDL test
and SFP actual-C regression live in this integration's `test/` directory. The
hardware uses backports to its pinned WR-core/WRPC revisions; the complete latest
upstream revisions were not substituted into the qualified FPGA images.

The SPI MOSI clear-strobe fix is already upstream as WR-core `ff0d6950`; this work
backports it and does not propose a duplicate upstream change. The host tool fix
has been submitted separately as [LiteX PR 2590](https://github.com/enjoy-digital/litex/pull/2590).
