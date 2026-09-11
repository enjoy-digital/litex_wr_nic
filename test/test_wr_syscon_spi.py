"""Exercise the pinned WR-core SPI process with real set/clear strobes."""
from pathlib import Path
import re
import shutil
import subprocess

import pytest

from litex_wr_nic.gateware import wr_common

ROOT = Path(__file__).resolve().parents[1]


def test_spi_mosi_gpio_clear(tmp_path, monkeypatch):
    source = ROOT / wr_common.WR_SYSCON_VHD
    if not source.exists() or shutil.which("ghdl") is None:
        pytest.skip("Pinned WR-core sources and GHDL are required")
    path = tmp_path / "wrc_syscon.vhd"
    # Start from the pinned upstream version, regardless of prior local builds.
    original = subprocess.check_output(["git", "-C", str(ROOT / "wr-cores"),
        "show", wr_common.WR_CORES_SHA1 + ":modules/wrc_core/wrc_syscon.vhd"], text=True)
    path.write_text(original)
    monkeypatch.setattr(wr_common, "WR_SYSCON_VHD", str(path))

    def simulate(text):
        logic = re.search(r"  p_drive_spi: process.*?end process;", text, re.S).group()
        fields = sorted(set(re.findall(r"sysc_regs_o\.(\w+)", logic)))
        bench = tmp_path / "spi_tb.vhd"
        bench.write_text("""
library ieee;
use ieee.std_logic_1164.all;
entity spi_tb is end;
architecture test of spi_tb is
  signal clk_sys_i : std_logic := '0';
  signal rst_n_i : std_logic := '0';
  signal spi_mosi_o, spi_ncs_o, spi_sclk_o : std_logic;
  type registers is record
""" + "".join("    " + field + " : std_logic;\n" for field in fields) + """
  end record;
  signal sysc_regs_o : registers := (others => '0');
begin
  clk_sys_i <= not clk_sys_i after 5 ns;
""" + logic + """
  stimulus: process
  begin
    wait for 10 ns;
    assert spi_mosi_o = '0' and spi_ncs_o = '1' and spi_sclk_o = '0'
      report "SPI reset state" severity failure;
    rst_n_i <= '1';
    -- Shift alternating bits, including isolated GPCR strobes. No GPSR write
    -- may be necessary to clear MOSI, and idle cycles must retain its state.
    for i in 1 to 16 loop
      sysc_regs_o.gpsr_wr <= '1';
      sysc_regs_o.gpsr_spi_mosi <= '1';
      wait for 10 ns;
      assert spi_mosi_o = '1' report "MOSI set failed" severity failure;
      sysc_regs_o.gpsr_wr <= '0';
      sysc_regs_o.gpsr_spi_mosi <= '0';
      wait for 10 ns;
      assert spi_mosi_o = '1' report "MOSI hold failed" severity failure;
      sysc_regs_o.gpcr_wr <= '1';
      sysc_regs_o.gpcr_spi_mosi <= '1';
      wait for 10 ns;
      assert spi_mosi_o = '0' report "MOSI clear failed" severity failure;
      sysc_regs_o.gpcr_wr <= '0';
      wait for 10 ns;
      assert spi_mosi_o = '0' report "MOSI hold failed" severity failure;
    end loop;
    report "SPI checks passed";
    std.env.stop;
    wait;
  end process;
end;
""")
        for action in ("-a", "-e"):
            result = subprocess.run(["ghdl", action, "--std=08",
                str(bench) if action == "-a" else "spi_tb"],
                cwd=tmp_path, capture_output=True, text=True, timeout=30)
            assert result.returncode == 0, result.stdout + result.stderr
        return subprocess.run(["ghdl", "-r", "--std=08", "spi_tb", "--assert-level=error"],
            cwd=tmp_path, capture_output=True, text=True, timeout=30)

    # Current upstream already contains the MOSI fix. Retain a negative
    # control so the test proves that an isolated GPCR strobe is exercised.
    broken = original.replace(
        "elsif sysc_regs_o.gpcr_wr = '1' and sysc_regs_o.gpcr_spi_mosi = '1' then",
        "elsif sysc_regs_o.gpsr_wr = '1' and sysc_regs_o.gpcr_spi_mosi = '1' then")
    assert broken != original
    baseline = simulate(broken)
    assert baseline.returncode != 0 and "MOSI clear failed" in baseline.stdout
    wr_common.patch_wr_syscon_spi_mosi()
    fixed = path.read_text()
    wr_common.patch_wr_syscon_spi_mosi()
    assert path.read_text() == fixed
    result = simulate(fixed)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "SPI checks passed" in result.stdout
