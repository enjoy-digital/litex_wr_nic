"""Check DAC gain binding and the commands sent to the physical SPI pins."""

from pathlib import Path
import shutil
import subprocess
from types import SimpleNamespace

import pytest
from migen import Instance, Record, Signal

from litex_wr_nic.gateware.ad5683r.core import AD5683RDAC


ROOT = Path(__file__).resolve().parents[1]
SOURCES = ROOT / "litex_wr_nic/gateware/ad5683r"


@pytest.mark.parametrize("gain", [None, 1, 2], ids=["default-x2", "x1", "x2"])
def test_dac_gain_command(tmp_path, gain):
    if shutil.which("ghdl") is None:
        pytest.skip("GHDL is required for the DAC simulation")

    platform = SimpleNamespace(add_source=lambda path: None)
    pads = Record([(name, 1) for name in ("ldac_n", "sync_n", "sclk", "sdi")])
    kwargs = {} if gain is None else {"gain": gain}
    dut = AD5683RDAC(platform, pads, Signal(), Signal(16), clk_domain="wr_sys", **kwargs)
    instance, = [s for s in dut.get_fragment().specials if isinstance(s, Instance)]
    # Feed the actual Python instance's generic names/values to the VHDL
    # elaborator, so an ignored or misspelled mixed-language generic fails.
    generic_map = []
    for item in instance.items:
        if isinstance(item, Instance.Parameter):
            value = item.value.value
            if item.name in ("g_invert_sclk", "g_enable_x2_gain"):
                value = str(bool(value)).lower()
            generic_map.append(f"{item.name} => {value}")
    config = "400000" if gain == 1 else "408000"
    tb = tmp_path / "dac_tb.vhd"
    tb.write_text(f"""
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
entity dac_tb is end;
architecture test of dac_tb is
  signal clk, rst_n, load : std_logic := '0';
  signal sync_n, sclk, sdi : std_logic;
  signal value : std_logic_vector(15 downto 0) := x"a53c";
begin
  clk <= not clk after 8 ns;
  dut : entity work.serial_dac_arb
    generic map ({", ".join(generic_map)})
    port map (clk_i => clk, rst_n_i => rst_n, val_i => value, load_i => load,
      dac_ldac_n_o => open, dac_clr_n_o => open, dac_sync_n_o => sync_n,
      dac_sclk_o => sclk, dac_din_o => sdi);
  stimulus : process
  begin
    wait for 160 ns;
    wait until falling_edge(clk);
    rst_n <= '1';
    -- Queue a value while the reference/gain command is being transmitted.
    wait until falling_edge(sync_n);
    wait until falling_edge(clk);
    load <= '1';
    wait until falling_edge(clk);
    load <= '0';
    wait;
  end process;
  receiver : process
    variable command : std_logic_vector(23 downto 0);
    variable count : natural;
  begin
    for transaction in 0 to 1 loop
      wait until falling_edge(sync_n);
      command := (others => '0');
      count := 0;
      loop
        wait on sclk, sync_n;
        exit when sync_n = '1';
        if falling_edge(sclk) then
          command := command(22 downto 0) & sdi;
          count := count + 1;
        end if;
      end loop;
      assert count = 24 report "Wrong SPI command length" severity failure;
      if transaction = 0 then
        assert command = x"{config}"
          report "Wrong DAC reference/gain command" severity failure;
      else
        assert command = x"3a53c0"
          report "Lost or corrupted queued DAC value" severity failure;
      end if;
    end loop;
    std.env.stop;
    wait;
  end process;
  watchdog : process
  begin
    wait for 20 us;
    assert false report "DAC command timed out" severity failure;
  end process;
end;
""")
    for source in [SOURCES / "serial_dac.vhd", SOURCES / "serial_dac_arb.vhd", tb]:
        subprocess.run(["ghdl", "-a", "--std=08", str(source)], cwd=tmp_path, check=True)
    subprocess.run(["ghdl", "-e", "--std=08", "dac_tb"], cwd=tmp_path, check=True)
    subprocess.run(["ghdl", "-r", "--std=08", "dac_tb", "--assert-level=error"],
                   cwd=tmp_path, check=True)
