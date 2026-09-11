"""Exercise the real WR host write decoder against diagnostic v2 addresses."""
from pathlib import Path
import re
import shutil
import subprocess

import pytest
from litex_wr_nic.gateware import wr_common

ROOT = Path(__file__).resolve().parents[1]


def test_only_diagnostic_control_is_host_writable(tmp_path, monkeypatch):
    if not (ROOT/'wr-cores').exists() or not shutil.which('ghdl'):
        pytest.skip('Pinned WR-core checkout and GHDL are required')
    original = subprocess.check_output(['git','-C',str(ROOT/'wr-cores'),'show',
        wr_common.WR_CORES_SHA1+':modules/wrc_core/wrc_diags_dpram.vhd'],text=True)
    path=tmp_path/'wrc_diags_dpram.vhd';path.write_text(original)
    monkeypatch.setattr(wr_common,'WR_DIAGS_VHD',str(path))
    def simulate(source):
        logic='\n'.join(re.findall(r'^  (?:s_is_control_word|s_we_user) <= .*?;$',source,re.M))
        assert len(logic.splitlines())==2
        tb=tmp_path/'diags_tb.vhd'
        tb.write_text('''
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
entity diags_tb is end;
architecture test of diags_tb is
  constant g_size : integer := 256;
  function f_log2_size(value:integer) return integer is
  begin return 8; end;
  type slave_in is record
    adr: std_logic_vector(31 downto 0);
    we, stb, cyc: std_logic;
  end record;
  signal slave_user_i: slave_in := (adr=>(others=>'0'), we=>'0',stb=>'0',cyc=>'0');
  signal s_is_control_word, s_we_user: std_logic;
begin
''' + logic + '''
  stimulus: process
  begin
    slave_user_i.we<='1';slave_user_i.stb<='1';slave_user_i.cyc<='1';
    for word in 0 to 63 loop
      slave_user_i.adr<=std_logic_vector(to_unsigned(word*4,32));
      wait for 1 ns;
      if word=1 then
        assert s_we_user='1' report "Snapshot CTRL must be writable" severity failure;
      else
        assert s_we_user='0' report "Version and diagnostics must be read-only" severity failure;
      end if;
    end loop;
    slave_user_i.adr<=x"00000004";
    slave_user_i.we<='0';wait for 1 ns;
    assert s_we_user='0' severity failure;
    slave_user_i.we<='1';slave_user_i.stb<='0';wait for 1 ns;
    assert s_we_user='0' severity failure;
    slave_user_i.stb<='1';slave_user_i.cyc<='0';wait for 1 ns;
    assert s_we_user='0' severity failure;
    report "Diagnostic decoder checks passed";
    std.env.stop;wait;
  end process;
end;
''')
        for action,target in [('-a',str(tb)),('-e','diags_tb')]:
            result=subprocess.run(['ghdl',action,'--std=08',target],cwd=tmp_path,capture_output=True,text=True,timeout=20)
            assert result.returncode==0,result.stderr
        return subprocess.run(['ghdl','-r','--std=08','diags_tb','--assert-level=error'],cwd=tmp_path,capture_output=True,text=True,timeout=20)
    assert simulate(original).returncode != 0
    wr_common.patch_wr_diags_control_word()
    fixed=path.read_text();wr_common.patch_wr_diags_control_word();assert path.read_text()==fixed
    result=simulate(fixed)
    assert result.returncode==0,result.stdout+result.stderr
