# Upstream fixes and contribution PRs

Three independent fixes discovered during Acorn/SPEC-A7 qualification are public
as PRs in **unofficial GitHub contribution mirrors**. Their `master` branches are
unchanged snapshots of the maintained GitLab upstreams. Each PR contains one
commit on the corresponding upstream base, so reviewers can fetch or cherry-pick
it without importing LiteX-specific changes.

| Fix | Public contribution PR | Patch / commit |
| --- | --- | --- |
| Host snapshot CTRL decoder | [wr-cores #1](https://github.com/enjoy-digital/wr-cores/pull/1) | [Patch](wr-cores-diag-control.patch), `628cfa0792c74bab83fed378b54ac0dbaa288436` |
| Firmware private diagnostic address | [wrpc-sw #1](https://github.com/enjoy-digital/wrpc-sw/pull/1) | [Patch](wrpc-diag-address.patch), `225231378741cd3ded7132778b2cde90633e1698` |
| Negative SFP storage errors treated as matches | [wrpc-sw #2](https://github.com/enjoy-digital/wrpc-sw/pull/2) | [Patch](wrpc-sfp-storage-errors.patch), `7983b2c9ec5fbca4e4f9d799b61b2c80457d2c56` |

These are not official GitLab merge requests, and the mirrors are not a new
upstream. No GitLab MR has been submitted or maintainer notification sent.

## Repository migration and the exact publication failure

OHWR development moved to GitLab.com in 2025; ohwr.org became a catalogue. See the
[CERN migration announcement](https://indico.cern.ch/event/1524513/contributions/6555958/attachments/3092423/5477416/WRC_status_and_plans.pdf).
The [old WRPC path](https://gitlab.com/ohwr/hdl-core-lib/wr-cores/wrpc-sw)
is archived and describes itself as a read-only project preserving old URLs.

The active repositories and `master` snapshots fetched for these focused patch
checks earlier on 2026-09-11 are:

| Official repository | Patch review base | Commit date |
| --- | --- | --- |
| [wr-cores](https://gitlab.com/ohwr/project/wr-cores) | `8f9b06565ef8d23712494314def63f6fc3813f8c` | 2026-09-07 |
| [wrpc-sw](https://gitlab.com/ohwr/project/wrpc-sw) | `13527cd68e1833214a89e4ee8c5b208188ff0e6a` | 2026-09-09 |

WR-core's current README explicitly names `master` as the development branch
since 2025. Both projects' `.ohwr.yaml` files point to these current locations.
All three bugs remain present at these revisions.

WR-core subsequently advanced to `8cc5e532` on the same day. The
[complete upstream build](../upstream-build.md) uses that newer revision and
WRPC `13527cd6`; its [hardware results](../results-upstream/README.md) are
recorded separately from this focused patch audit.

Anonymous HTTPS fetches from these repositories succeed. The failure is account
authentication for publication: `ssh -T git@gitlab.com` returns
`Permission denied (publickey)` before any repository path is evaluated. No
GitLab API token or GitLab credential-helper credential is available. Existing
GitHub authentication works, which allows the contribution-mirror workaround.
Changing the repository URL alone does not supply GitLab write authentication.

## Fresh upstream checks

[upstream-audit.json](upstream-audit.json) records failing unpatched cases and
passing fixed cases for the exact published commits:

- The host-write decoder permits VER at byte offset `0x00`, instead of CTRL at
  `0x04`. GHDL checks the actual extracted decoder assignments across 64 words
  and inactive WE/STB/CYC conditions.
- `BASE_WDIAGS_PRIV` is `DEV_BASE + 0x900`, while the generated CPU header and
  WR-core Cheby map both specify `0x800`.
- The actual `sfp_match()` C function treats `-EIO` as a successful calibration
  match. A host-compiled regression with mocked I2C/storage covers a match,
  no match, `-EIO`, `-ENODEV`, and clearing a previous matched state.

Reproduce from an integration checkout with Git, GHDL, Python 3 and a host C
compiler. Use fresh clone paths if the example directories already exist:

```sh
git clone https://github.com/enjoy-digital/wr-cores.git /tmp/wr-cores-review
git clone https://github.com/enjoy-digital/wrpc-sw.git /tmp/wrpc-review
python3 doc/wr_link/upstream/verify_upstream.py \
    --wr-cores /tmp/wr-cores-review --wrpc-sw /tmp/wrpc-review \
    --output /tmp/wr-upstream-audit.json
```

Apply an individual downloaded patch with `git am <patch>` in its corresponding
repository. Each PR description also supplies an exact `git fetch`/`cherry-pick`
command. The SFP commit was separated from the earlier two-commit local branch;
its patch content is unchanged, but its parent is now the unmodified WRPC base.

The original hardware qualification used backports to older project pins.
These focused checks audit the three published patches; they do not themselves
establish hardware qualification. See the separate complete upstream build and
hardware results linked above for the newly rebuilt images.

The SPI MOSI clear-strobe fix is already upstream as WR-core `ff0d6950`. The
original build backported it; the current WR-core pin already includes it.
No duplicate upstream change is proposed. The host tool fix
has been submitted separately as [LiteX PR 2590](https://github.com/enjoy-digital/litex/pull/2590).
