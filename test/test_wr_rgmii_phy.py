#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Exercise the actual Xilinx DDR/PLL models, including peer-clock restart."""

from pathlib import Path
import shutil
import subprocess

import pytest

from migen import ClockDomain, Record, Signal

from litex.gen import LiteXModule

from litex.build.xilinx import XilinxPlatform

from litex_wr_nic.gateware.rgmii import WRRGMIIPhy


def generate_phy(output, role, delay):
    platform = XilinxPlatform("xc7a50t-csg324-2", [], toolchain="vivado")
    dut        = LiteXModule()
    dut.cd_sys = ClockDomain("sys")
    pads = Record([
        ("rx_clk",  1),
        ("rx_data", 4),
        ("rx_dv",   1),
        ("tx_clk",  1),
        ("tx_data", 4),
        ("tx_en",   1),
    ], name="pads")
    tx_prefix, rx_prefix = ("tx", "rx") if role == "mac" else ("rx", "tx")
    mapping = {
        "rx_clock_in":  getattr(pads, rx_prefix + "_clk"),
        "rx_data_in":   getattr(pads, rx_prefix + "_data"),
        "rx_ctl_in":    getattr(pads, "rx_dv" if rx_prefix == "rx" else "tx_en"),
        "tx_clock_out": getattr(pads, tx_prefix + "_clk"),
        "tx_data_out":  getattr(pads, tx_prefix + "_data"),
        "tx_ctl_out":   getattr(pads, "rx_dv" if tx_prefix == "rx" else "tx_en"),
    }
    for name, signal in mapping.items():
        signal.name_override = name
    dut.phy = WRRGMIIPhy(platform, pads,
        role            = role,
        tx_delay        = delay,
        rx_delay        = delay,
        rx_phase_adjust = -0.75e-9 if role == "phy" else 0,
    )
    ports = {}
    for name, size, destination in [
        ("tx_byte",  8, dut.phy.sink.data),
        ("tx_valid", 1, dut.phy.sink.valid),
        ("tx_error", 1, dut.phy.sink.error),
    ]:
        ports[name] = Signal(size, name_override=name)
        dut.comb += destination.eq(ports[name])
    for name, size, source in [
        ("rx_byte",         8, dut.phy.source.data),
        ("rx_valid",        1, dut.phy.source.valid),
        ("rx_last",         1, dut.phy.source.last),
        ("rx_error",        1, dut.phy.source.error),
        ("ready",           1, ~dut.phy.reset),
        ("tx_domain_clock", 1, dut.phy.cd_eth_tx.clk),
        ("rx_domain_clock", 1, dut.phy.cd_eth_rx.clk),
    ]:
        ports[name] = Signal(size, name_override=name)
        dut.comb += ports[name].eq(source)
    ios = set(mapping.values()) | set(ports.values()) | {dut.cd_sys.clk, dut.cd_sys.rst}
    platform.get_verilog(dut, ios=ios, name="phy_dut").write(str(output / "phy_dut.v"))


@pytest.mark.skipif(any(shutil.which(tool) is None for tool in ("vivado", "xvlog", "xelab", "xsim")),
    reason="Xilinx Vivado simulation tools are required for the DDR/PLL models")
@pytest.mark.parametrize("role,delay", [("mac", 2e-9), ("phy", 2e-9), ("phy", 0)])
def test_rgmii_ddr_and_clock_recovery(tmp_path, role, delay):
    generate_phy(tmp_path, role, delay)
    tb = (Path(__file__).parent / "hdl/wr_rgmii_tb.sv").read_text()
    tb = tb.replace("PEER_DELAY = 0.0", f"PEER_DELAY = {2 - delay * 1e9:.1f}")
    (tmp_path / "phy_tb.sv").write_text(tb)
    vivado = Path(shutil.which("vivado")).resolve().parents[1]
    commands = [
        ["xvlog", "--sv", "phy_dut.v", "phy_tb.sv", str(vivado / "data/verilog/src/glbl.v")],
        ["xelab", "phy_tb", "glbl", "-L", "unisims_ver", "-s", "phy_tb_sim", "--timescale", "1ns/1ps"],
        ["xsim", "phy_tb_sim", "-runall"],
    ]
    for number, command in enumerate(commands):
        with (tmp_path / f"{number}.log").open("w") as log:
            subprocess.run(command,
                cwd=tmp_path, check=True, timeout=180,
                stdout=log, stderr=subprocess.STDOUT,
            )
    assert "RGMII PHY PASS" in (tmp_path / "2.log").read_text()
