# SPEC-A7 Grandmaster reference

SPEC-A7 uses the external AD9516 (IC15) to multiply the 10 MHz reference to
62.5 MHz for the WR SoftPLL. The raw 10 MHz still feeds the PPS aligner.
The WR system clock, main VCXO and DMTD clock retain their existing paths.

Connect a common reference source's 10 MHz output to J3 and its PPS output
to J21, using the input levels and termination in the
[SPEC-A7 schematic](spec_a7_schematic.pdf). Connect the WR peer through the
selected SFP port (J12 on the USB bench).

Build normally with `python3 spec_a7_wr_nic.py --build`. For comparison with
the previous FPGA multiplier, build with `--wr-ext-clock-multiplier mmcm`.
This is a build-time choice; there is no clock mux that switches during operation.

In the WR UART console:

```text
ptp stop
ptp gm
ptp start
pll stat
stat
```

`ptp gm` waits up to 60 seconds for initial lock. With the reference absent,
the firmware reports `Lock timeout`; connect both inputs and retry the sequence.

With the AD9516 build, the host CSRs `sync_in_pll_reference_present`,
`sync_in_pll_done` and `sync_in_pll_locked` report raw input activity,
configuration completion and qualified PLL lock. Clock measurement channels
3 and 4 count the raw 10 MHz and multiplied 62.5 MHz inputs respectively.
The activity detector is independent of the AD9516 reset: WR firmware first
waits for 10 MHz, then resets/reconfigures the AD9516 and waits for lock.
Its reset request and PLL status cross between the WR and host clock domains.
The delayed PPS is extended from 16 ns to 256 ns so the 10 MHz aligner can
capture it at any clock phase; its leading edge and macro delay are preserved.

For hardware validation, check Grandmaster lock with both inputs present,
remove and restore the 10 MHz input, and confirm reacquisition without a host
reset. Also test booting without the reference and connecting it afterward.
Compare input/output PPS with an independent timing instrument, including
repeat power cycles, before setting the PPS delay compensation. A WR link
or locked servo alone does not establish absolute PPS accuracy or jitter.

The frequency configuration is unchanged: R=4, N=16×37+8=600 gives a
1.5 GHz VCO; the VCO /2 and output /12 produce 62.5 MHz. The LD pin uses
active-high digital lock detection (register 0x01A = 0), as documented in the
[AD9516-4 data sheet](https://www.analog.com/media/en/technical-documentation/data-sheets/AD9516-4.pdf).
