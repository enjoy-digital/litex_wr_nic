"""Simulate the board reset circuit with GHDL and the pinned wr-cores checkout."""

from pathlib import Path
import re
import shutil
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[1]
BOARD = ROOT / "litex_wr_nic/gateware/wr-cores/board/litex_wr_nic/xwrc_board_litex_wr_nic.vhd"
COMMON = ROOT / "wr-cores/ip_cores/general-cores/modules/common"


def test_wr_reset_survives_phy_pll_unlock(tmp_path):
    if shutil.which("ghdl") is None:
        pytest.skip("GHDL is required for the WR reset simulation")
    if not (COMMON / "gc_reset.vhd").is_file():
        pytest.skip("Initialize the pinned wr-cores checkout before running this simulation")

    # Exercise the actual board reset circuit without simulating the CPU or
    # vendor transceiver primitives. Model their independent clocks/lock inputs.
    board = BOARD.read_text()
    start = board.index("  cmp_arst_edge:")
    end = board.index("  rst_62m5_n <= rstlogic_rst_out(0);")
    reset_logic = board[start:end] + "  rst_62m5_n <= rstlogic_rst_out(0);\n"
    clock_exports = "\n".join(re.findall(
        r"^  (?:clk|rst)_62m5_(?:sys|ref)_o <= .*;$", board, re.MULTILINE))
    testbench = tmp_path / "wr_reset_tb.vhd"
    testbench.write_text("""
library ieee;
use ieee.std_logic_1164.all;
use work.gencores_pkg.all;

entity wr_reset_tb is end;

architecture test of wr_reset_tb is
  signal clk_pll_62m5, clk_ref_62m5, clk_62m5_dmtd_i : std_logic := '0';
  signal pll_locked, clk_ref_locked, areset_n_i : std_logic := '0';
  signal areset_edge_n_i : std_logic := '0';
  signal areset_edge_ppulse, rstlogic_arst_n, rst_62m5_n : std_logic;
  signal rstlogic_clk_in, rstlogic_rst_out : std_logic_vector(1 downto 0);
  signal clk_62m5_sys_o, rst_62m5_sys_o, clk_62m5_ref_o, rst_62m5_ref_o : std_logic;
begin
  clk_pll_62m5 <= not clk_pll_62m5 after 8 ns;
  clk_62m5_dmtd_i <= not clk_62m5_dmtd_i after 8.1 ns;
  clk_ref_62m5 <= not clk_ref_62m5 after 8 ns when clk_ref_locked = '1' else '0';
""" + reset_logic + clock_exports + """
  check_exports : postponed process(all)
  begin
    if now > 0 ns then
    assert clk_62m5_sys_o = clk_pll_62m5
      report "WR system clock must keep running when the PHY clock stops" severity failure;
    assert clk_62m5_ref_o = clk_ref_62m5
      report "PPS reference clock must come from the PHY" severity failure;
    assert rst_62m5_sys_o = not rstlogic_rst_out(0)
      report "WR system reset must use the synchronized board reset" severity failure;
    assert rst_62m5_ref_o = not rstlogic_rst_out(1)
      report "PPS reference reset must use its own synchronizer" severity failure;
    end if;
  end process;

  stimulus : process
    procedure expect_reset(value : std_logic; cycles : positive) is
    begin
      for i in 1 to cycles loop
        wait until falling_edge(clk_pll_62m5);
        assert rst_62m5_n = value
          report "Unexpected WR CPU reset state" severity failure;
      end loop;
    end procedure;
  begin
    -- Power-on reset must wait for the system PLL and external reset release.
    expect_reset('0', 32);
    clk_ref_locked <= '1';
    areset_n_i <= '1';
    expect_reset('0', 32);
    pll_locked <= '1';
    wait for 1 us;
    expect_reset('1', 32);

    -- Firmware resets the PHY during endpoint initialization. This drops the
    -- transceiver PLL lock and can stop its TX clock, but must not reboot WR.
    for attempt in 1 to 3 loop
      clk_ref_locked <= '0';
      expect_reset('1', 128);
      clk_ref_locked <= '1';
      expect_reset('1', 128);
    end loop;

    -- Loss of system PLL lock must still reset WR, even with the PHY stopped.
    clk_ref_locked <= '0';
    pll_locked <= '0';
    wait for 1 us;
    expect_reset('0', 32);
    pll_locked <= '1';
    wait for 1 us;
    expect_reset('1', 32);

    -- Explicit board reset remains effective.
    areset_n_i <= '0';
    wait for 1 us;
    expect_reset('0', 32);
    areset_n_i <= '1';
    wait for 1 us;
    expect_reset('1', 32);

    -- PCIe reset being held low must allow standalone operation. Its rising
    -- edge must still generate a reset pulse, followed by normal operation.
    areset_edge_n_i <= '1';
    wait until rst_62m5_n = '0' for 1 us;
    assert rst_62m5_n = '0' report "Missing PCIe reset pulse" severity failure;
    wait for 1 us;
    expect_reset('1', 32);
    areset_edge_n_i <= '0';
    expect_reset('1', 64);

    report "WR reset checks passed";
    std.env.stop;
    wait;
  end process;
end;
""")

    def run(*args):
        result = subprocess.run(
            ["ghdl", args[0], "--std=08", "-frelaxed-rules", *args[1:]],
            cwd=tmp_path, capture_output=True, text=True, timeout=30)
        assert result.returncode == 0, result.stdout + result.stderr
        return result.stdout

    run("-a", *[str(COMMON / name) for name in (
        "gencores_pkg.vhd", "gc_sync.vhd", "gc_edge_detect.vhd",
        "gc_sync_ffs.vhd", "gc_reset.vhd")], str(testbench))
    run("-e", "wr_reset_tb")
    output = run("-r", "wr_reset_tb", "--assert-level=error", "--stop-time=30us")
    assert "WR reset checks passed" in output
