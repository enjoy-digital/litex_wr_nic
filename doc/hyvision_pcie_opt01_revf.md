# HyVision PCIe OPT01 rev.F

Board page for `hyvision_pcie_opt01_revf.py`: an XC7K70T (FBG676, speed
grade -1) on a PCIe card with two SFP cages, two DDR2 devices and 200 MHz /
50 MHz oscillators. The target builds a PCIe NIC with White Rabbit and PTM.
See [board targets and builds](boards.md) for the target matrix and the
qualification status.

## Build

```sh
python3 hyvision_pcie_opt01_revf.py --build --load
```

The defaults are a 125 MHz system clock, PCIe x1, WR on SFP0/CN1 and the
embedded uRV CPU. `--sfp 1` moves WR and the NIC to CN2. CPU and memory
selection follows the other targets; see [WR integration](wr_integration.md).
`--with-wr-pll-debug` adds the SoftPLL trace FIFO and the FPGA temperature
and supply CSRs. The firmware image is built and embedded automatically;
`--skip-firmware-build` reuses a separately built one.

## Console

The board has no dedicated console connector. The shared WR/LiteX UART is
brought out on **J7**, the DVP header on the top edge of the card. Every
signal on J7 is named on the silkscreen, including `DTX` and `DRX` next to
the second-to-last column on the right.

| Signal | FPGA | J7 pin | Direction |
| --- | --- | --- | --- |
| DTX | T19 | 37 | FPGA transmit, adapter RXD |
| DRX | U19 | 38 | FPGA receive, adapter TXD |

The UART runs at 115200 baud, 8N1, no flow control, at 3.3 V LVCMOS.

```text
adapter GND  ->  J7 pins 1, 2, 19, 20, 39 or 40 (see ground note)
             or  J11/J13 pin 5
adapter RXD  <-  J7 pin 37 (DTX)
adapter TXD  ->  J7 pin 38 (DRX)
adapter VCC  ->  not connected
```

Connect ground first, then the data lines. Use a 3.3 V adapter; the pins are
not 5 V tolerant. The FPGA receives on DRX and drives DTX, so adapter TXD
goes to pin 38 and adapter RXD to pin 37.

**Ground.** J11/J13 pin 5 is the documented ground and is the reference for
the JTAG header pinout below. On J7, pins 1, 2, 19, 20, 39 and 40 carry no
silkscreen label. Their positions — both ends of the header and the boundary
between the green and red pixel banks — are the usual place for grounds on a
DVP header, but the vendor pin table does not list them. Check continuity to
the SFP cage shield or the bracket before using them as ground.

**Pin numbering.** Pin 1 is the square pad at the left end of the bottom row.
Pins follow the standard dual-row convention: odd pins on the bottom row,
even pins on the top row, one column per pair. The vendor pin table names the
signals but does not number the connector, so the silkscreen labels are
authoritative if a unit is numbered differently.

Bring-up:

1. Power off. Check the candidate ground pins against the SFP cage shield.
2. Power the board, then the adapter. DTX should idle high at about 3.3 V.
3. Open the port at 115200 baud and look for the WRPC `wrc#` prompt.
4. If there is no output, swap TX and RX once before investigating further.
5. The design also instantiates JTAGBone. J11 or J13 gives console access
   without the DVP header when the UART is unavailable.

The [USB bench guide](wr_usb_bench.md) covers stable UART paths, role
selection and per-board master trim.

JTAG connectors J11/J13:

| Pins | 1 | 2 | 3 | 4 | 5 | 6 |
| --- | --- | --- | --- | --- | --- | --- |
| Signal | TMS | TDI | TDO | TCK | GND | VCC |

## J7 pin map

J7 is a 2x20, 2.54 mm header on the top edge of the card, component side,
above the DDR2 devices. It carries the whole DVP camera group plus the two
serial lines. Rows are printed as column pairs.

| Pin | Label | FPGA | Pin | Label | FPGA |
| --- | --- | --- | --- | --- | --- |
| 1 | - | - | 2 | - | - |
| 3 | B0_0 | J23 | 4 | B0_1 | H24 |
| 5 | B0_2 | L23 | 6 | B0_3 | K23 |
| 7 | B0_4 | G24 | 8 | B0_5 | F24 |
| 9 | B0_6 | F23 | 10 | B0_7 | E23 |
| 11 | G0_0 | M19 | 12 | G0_1 | C24 |
| 13 | G0_2 | C22 | 14 | G0_3 | N21 |
| 15 | G0_4 | M21 | 16 | G0_5 | L22 |
| 17 | G0_6 | K22 | 18 | G0_7 | J21 |
| 19 | - | - | 20 | - | - |
| 21 | R0_0 | H22 | 22 | R0_1 | G22 |
| 23 | R0_2 | F22 | 24 | R0_3 | E21 |
| 25 | R0_4 | D21 | 26 | R0_5 | N19 |
| 27 | R0_6 | P20 | 28 | R0_7 | P21 |
| 29 | CC_D0 | R22 | 30 | CC_D1 | T22 |
| 31 | CC_D2 | R21 | 32 | CC_D3 | T20 |
| 33 | LVAL | U20 | 34 | FVAL | T18 |
| 35 | DVAL | M20 | 36 | PCLK | D23 |
| **37** | **DTX** | **T19** | **38** | **DRX** | **U19** |
| 39 | - | - | 40 | - | - |

`-` is a pin without a silkscreen label. The unlabelled pins are the six
ground candidates described above. The signal-to-FPGA-pin assignments match
the vendor pin table shipped with the board, `hvs_75t.xlsx`.

## LEDs

Ethernet runs over the SFP cages; there is no RJ45 connector. Two independent
LED groups exist.

Fitted SMD LEDs, driven by `user_led`:

| LED | FPGA | Resource | Use in this target |
| --- | --- | --- | --- |
| D5 | D26 | `user_led 0` | WR link, driven inverted |
| D3 | D25 | `user_led 1` | activity, driven inverted |
| D6 | C26 | `user_led 2` | PPS, driven inverted |
| D4 | C21 | `user_led 3` | free |
| D8 | G25 | `user_led 4` | free |
| D7 | F25 | `user_led 5` | free |
| D10 | E26 | `user_led 6` | free |
| D9 | E25 | `user_led 7` | free |

D12 (J26), D14 (J25), D13 (H26) and D15 (G26) are unpopulated.

The faceplate carries one red/green through-hole pair per SFP port, active
low:

| LED | FPGA | Port |
| --- | --- | --- |
| red 1 | E15 | SFP0/CN1 |
| green 1 | E16 | SFP0/CN1 |
| red 2 | E17 | SFP1/CN2 |
| green 2 | E18 | SFP1/CN2 |

The vendor demonstration uses these as per-port status: red lit for a fibre
present, green lit for a locked link. This target does not claim
`user_led 8..11`, so the faceplate LEDs stay dark and the WR state is shown
on D5/D3 instead.

**LED polarity.** The platform file comments the SMD LEDs as active high, but
this target drives `user_led 0..2` inverted. One of the two is wrong. Until
that is measured, treat the link, activity and PPS indication as unverified
and do not read the faceplate LEDs as a qualification result.

## Timing outputs

Two J7 pins carry timing in this target:

| Signal | FPGA | J7 pin | Function |
| --- | --- | --- | --- |
| LVAL | U20 | 33 | PPS output |
| FVAL | T18 | 34 | WR reference clock output |

Both are 3.3 V LVCMOS. Use them with an oscilloscope or a frequency counter
when checking PPS alignment or the WR clock; a successful build or a lit LED
does not show either.

## Open items

- Confirm the J7 ground candidates (pins 1, 2, 19, 20, 39, 40) with a
  continuity check.
- Measure the SMD LED polarity and align the platform comment with the
  target drive, or fix whichever is wrong.
- Confirm the J7 numbering on a second board revision; the numbers here are
  derived from the silkscreen and the pin-1 pad.
- WR link acquisition, tuning range and master/slave roles on this board are
  still unqualified; see [board targets](boards.md).

## References

- [Board targets and builds](boards.md)
- [USB bench guide](wr_usb_bench.md)
- [WR integration](wr_integration.md)
- `litex_boards/platforms/hyvision_pcie_opt01_revf.py` in LiteX-Boards for
  the pin resources, and the vendor pin table `hvs_75t.xlsx` for the signal
  names.
