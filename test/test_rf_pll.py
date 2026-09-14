#!/usr/bin/env python3

#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024-2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import argparse

from litex import RemoteClient

from litex_wr_nic.rf_pll import LMX2572, LMX2572Config


# Main ---------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Configure the SPEC-A7 LMX2572 over LiteX Server.")
    actions = parser.add_mutually_exclusive_group(required=True)
    actions.add_argument("--config", help="TICS Pro .txt register export or .tcs project.")
    actions.add_argument("--write-reg", nargs=2, type=lambda value: int(value, 0),
        metavar=("ADDRESS", "VALUE"), help="Write one register (does not recalibrate automatically).")
    actions.add_argument("--sync", action="store_true", help="Pulse manual SYNC; requires a matching bitstream.")
    parser.add_argument("--dry-run", action="store_true", help="Validate a configuration without accessing hardware.")
    parser.add_argument("--host", default="localhost", help="LiteX Server host (default: localhost).")
    parser.add_argument("--port", type=int, default=1234, help="LiteX Server port (default: 1234).")
    parser.add_argument("--csr-csv", default="csr.csv", help="CSR map matching the loaded bitstream.")
    parser.add_argument("--name", default="rf_out_pll", help="SPI CSR prefix (default: rf_out_pll).")
    parser.add_argument("--timeout", type=float, default=2.0, help="Bus/SPI timeout in seconds (default: 2).")
    parser.add_argument("--ref-clk-freq", type=float, default=25e6,
        help="Physical reference frequency in Hz (stock SPEC-A7: 25e6). Does not change the board clock.")
    args = parser.parse_args()
    if args.dry_run and not args.config:
        parser.error("--dry-run requires --config")

    bus = None
    try:
        config = None
        if args.config:
            config = LMX2572Config.from_file(args.config)
            frequencies = config.frequencies(args.ref_clk_freq)
            for name, value in frequencies.items():
                print(f"{name}: {value/1e6:.9f} MHz (configured)" if value is not None else f"{name}: disabled")
            if args.dry_run:
                print("Configuration checks passed; no hardware accessed.")
                return
        bus = RemoteClient(host=args.host, port=args.port, csr_csv=args.csr_csv,
            timeout=args.timeout, raise_on_timeout=True)
        pll = LMX2572(bus, name=args.name, timeout=args.timeout)
        # New images describe the board clock explicitly; older SPEC images use 25 MHz too.
        reference = getattr(bus.constants, "rf_out_pll_ref_clk_freq", None)
        if reference is not None and reference != args.ref_clk_freq:
            raise ValueError(f"CSR map specifies a {reference:g} Hz RF PLL reference")
        bus.open()
        if config is not None:
            pll.load(config, args.ref_clk_freq)
            print("Programming complete. Check LD9 and measure J8/J9; SPI completion does not establish lock.")
        elif args.write_reg:
            pll.write_reg(*args.write_reg)
            print(f"Wrote R{args.write_reg[0]} = 0x{args.write_reg[1]:04x}.")
        else:
            pll.toggle_sync()
            print("Manual SYNC pulse sent; this does not establish alignment to WR PPS.")
    except (OSError, ValueError, RuntimeError, TimeoutError, AttributeError) as error:
        parser.exit(1, f"RF PLL: {error}\n")
    finally:
        if bus is not None:
            bus.close()


if __name__ == "__main__":
    main()
