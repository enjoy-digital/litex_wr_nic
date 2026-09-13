"""Exercise the upstream GTX PHY's ready output with stopped/restarting RX clocks.

Only the transceiver analog behavior and BUFG are stubbed. The PHY, reset
logic, bitslide and synchronizers are the real pinned VHDL sources.
"""
from pathlib import Path
import re
import shutil
import subprocess

import pytest

from litex_wr_nic.gateware import wr_common

ROOT = Path(__file__).resolve().parents[1]


def test_gtx_ready_release_and_clock_loss(tmp_path, monkeypatch):
    if not shutil.which('ghdl') or not (ROOT / 'wr-cores/.git').exists():
        pytest.skip('Requires GHDL and the pinned wr-cores checkout')
    directory = Path('wr-cores/platform/xilinx/7Series/GTXE2')
    (tmp_path / directory).mkdir(parents=True)
    for name in ('wr_gtx_phy_family7.vhd', 'whiterabbit_gtxe2_channel_wrapper_gt.vhd'):
        source = subprocess.check_output(['git', '-C', str(ROOT / 'wr-cores'), 'show',
            wr_common.WR_CORES_SHA1 + ':' + str((directory / name).relative_to('wr-cores'))], text=True)
        (tmp_path / directory / name).write_text(source)
        if name == "wr_gtx_phy_family7.vhd":
            original_phy = source
    monkeypatch.chdir(tmp_path)
    wr_common.patch_wr_gtx_clocking()
    patched = (directory / 'wr_gtx_phy_family7.vhd').read_text()
    wr_common.patch_wr_gtx_clocking()
    assert (directory / 'wr_gtx_phy_family7.vhd').read_text() == patched

    def run(action, *args):
        result = subprocess.run(['ghdl', action, '--std=08', '-frelaxed-rules', '--syn-binding',
            *args], text=True, capture_output=True, timeout=30)
        assert result.returncode == 0, result.stdout + result.stderr
        return result

    (tmp_path / 'bufg.vhd').write_text('''library ieee; use ieee.std_logic_1164.all;
package vcomponents is
  component BUFG is port(I: in std_logic; O: out std_logic); end component;
end package;
library ieee; use ieee.std_logic_1164.all;
entity BUFG is port(I: in std_logic; O: out std_logic); end entity;
architecture model of BUFG is begin O <= I; end architecture;
''')
    run('-a', '--work=unisim', 'bufg.vhd')
    (tmp_path / 'controls.vhd').write_text('''library ieee; use ieee.std_logic_1164.all;
package controls is
  signal rxclk: std_logic := '0';
  signal running: boolean := true;
  signal locked, done: std_logic := '0';
end package;
''')
    run('-a', 'controls.vhd')
    # Keep the exact lower-wrapper interface so component binding also checks
    # the new clock-source and polarity generics against the parent PHY.
    source = (directory / 'whiterabbit_gtxe2_channel_wrapper_gt.vhd').read_text()
    entity = re.search(r'entity whiterabbit_gtxe2_channel_wrapper_gt is.*?end whiterabbit_gtxe2_channel_wrapper_gt;', source, re.S | re.I).group()
    values = dict(CPLLLOCK_OUT='locked', RXRESETDONE_OUT='done', TXRESETDONE_OUT='done',
        RXCDRLOCK_OUT='locked', RXOUTCLK_OUT='rxclk', TXOUTCLK_OUT='GTREFCLK0_IN')
    assignments = []
    for name, kind in re.findall(r'(\w+)\s*:\s*out\s+(std_logic(?:_vector\s*\([^;]+?\))?)', entity, re.I):
        value = values.get(name, "(others => '0')" if 'vector' in kind else "'0'")
        assignments.append(f'{name} <= {value};')
    (tmp_path / 'channel.vhd').write_text('library ieee; use ieee.std_logic_1164.all;\n'
        + entity + '\nlibrary ieee; use ieee.std_logic_1164.all; use work.controls.all;\n'
        + 'architecture model of whiterabbit_gtxe2_channel_wrapper_gt is begin\n'
        + '\n'.join(assignments) + '\nend architecture;\n')
    common = ROOT / 'wr-cores/ip_cores/general-cores/modules/common'
    for path in (common / 'gencores_pkg.vhd', common / 'gc_sync.vhd', common / 'gc_edge_detect.vhd',
            common / 'gc_sync_ffs.vhd', ROOT / 'wr-cores/modules/wr_tbi_phy/disparity_gen_pkg.vhd',
            ROOT / 'wr-cores/platform/xilinx/common/gtp_bitslide.vhd', Path('channel.vhd'),
            directory / 'wr_gtx_phy_family7.vhd'):
        run('-a', str(path))
    (tmp_path / 'ready_tb.vhd').write_text('''library ieee; use ieee.std_logic_1164.all;
use work.controls.all;
entity ready_tb is end entity;
architecture test of ready_tb is
  signal refclk: std_logic := '0';
  signal reset: std_logic := '1';
  signal ready: std_logic;
begin
  refclk <= not refclk after 4 ns;
  process begin
    wait for 8 ns;
    if running then rxclk <= not rxclk; else rxclk <= '0'; end if;
  end process;
  dut: entity work.wr_gtx_phy_family7 generic map(g_simulation => 1)
    port map(clk_gtx_i => refclk, tx_out_clk_o => open, tx_locked_o => open,
      tx_data_i => x"0000", tx_k_i => "00", tx_disparity_o => open, tx_enc_err_o => open,
      rx_rbclk_o => open, rx_data_o => open, rx_k_o => open, rx_enc_err_o => open,
      rx_bitslide_o => open, rst_i => reset, loopen_i => "000", tx_prbs_sel_i => "000",
      pad_txn_o => open, pad_txp_o => open, rdy_o => ready);
  process begin
    wait for 100 ns; reset <= '0';
    wait for 1 us; locked <= '1';
    wait for 100 ns;
    wait until falling_edge(rxclk); done <= '1';
    wait for 1 ns; assert ready = '0' report "Ready rose asynchronously" severity failure;
    wait until rising_edge(rxclk); wait for 1 ns;
    assert ready = '0' report "Ready skipped synchronization" severity failure;
    wait until rising_edge(rxclk); wait for 1 ns;
    assert ready = '1' report "Ready failed to release" severity failure;
    wait until falling_edge(rxclk); running <= false;
    wait for 10 ns; locked <= '0';
    wait for 1 ns; assert ready = '0' report "Stopped RX clock held ready high" severity failure;
    locked <= '1'; wait for 100 ns;
    assert ready = '0' report "Ready released without RX clocks" severity failure;
    running <= true; wait for 100 ns;
    assert ready = '1' report "Ready failed after RX restart" severity failure;
    done <= '0'; wait for 1 ns;
    assert ready = '0' report "Reset completion loss did not clear ready" severity failure;
    std.env.stop;
  end process;
end architecture;
''')
    run('-a', 'ready_tb.vhd')
    run('-e', 'ready_tb')
    run('-r', 'ready_tb', '--assert-level=error', '--stop-time=5us')

    # The same assertions must catch the original combinational ready path.
    (directory / 'wr_gtx_phy_family7.vhd').write_text(original_phy)
    run('-a', str(directory / 'wr_gtx_phy_family7.vhd'))
    run('-a', 'ready_tb.vhd')
    result = subprocess.run(['ghdl', '-r', '--std=08', '-frelaxed-rules', '--syn-binding',
        'ready_tb', '--assert-level=error', '--stop-time=5us'], text=True, capture_output=True, timeout=30)
    assert result.returncode != 0
    assert 'Ready rose asynchronously' in result.stdout
