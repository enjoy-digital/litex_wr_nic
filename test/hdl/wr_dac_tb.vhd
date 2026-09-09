--
-- This file is part of LiteX-WR-NIC.
--
-- Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
-- SPDX-License-Identifier: BSD-2-Clause

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity wr_dac_tb is
  generic (x2_gain : boolean := true);
end;

architecture test of wr_dac_tb is
  signal clk, rst_n, load : std_logic := '0';
  signal value : std_logic_vector(15 downto 0) := (others => '0');
  signal sync_n, sclk, sdi : std_logic;
  signal legacy_sync_n, legacy_sclk, legacy_sdi : std_logic;
  signal words : natural := 0;
  type t_values is array (natural range <>) of std_logic_vector(15 downto 0);
  constant values : t_values := (x"0000", x"8000", x"FFFF", x"AAAA", x"5555");
begin
  clk <= not clk after 8 ns;

  dut : entity work.serial_dac_arb
    generic map (g_invert_sclk => false, g_enable_x2_gain => x2_gain)
    port map (
      clk_i => clk, rst_n_i => rst_n, val_i => value, load_i => load,
      dac_ldac_n_o => open, dac_clr_n_o => open,
      dac_sync_n_o => sync_n, dac_sclk_o => sclk, dac_din_o => sdi);

  -- The old Python generic was ignored by Vivado. Omitting it reproduces
  -- the VHDL default actually used by both SPEC-A7 DACs before this series.
  legacy : entity work.serial_dac_arb
    generic map (g_invert_sclk => false)
    port map (
      clk_i => clk, rst_n_i => rst_n, val_i => value, load_i => load,
      dac_ldac_n_o => open, dac_clr_n_o => open,
      dac_sync_n_o => legacy_sync_n, dac_sclk_o => legacy_sclk, dac_din_o => legacy_sdi);

  compare_legacy : postponed process(all)
  begin
    if x2_gain and rst_n = '1' then
      assert sync_n = legacy_sync_n and sclk = legacy_sclk and sdi = legacy_sdi
        report "Default DAC serial waveform changed" severity failure;
    end if;
  end process;

  decode : process
    variable word : std_logic_vector(23 downto 0);
  begin
    wait until falling_edge(sync_n);
    for bit_index in 23 downto 0 loop
      wait until falling_edge(sclk);
      word(bit_index) := sdi;
    end loop;
    wait until rising_edge(sync_n);
    if words = 0 then
      if x2_gain then
        assert word = x"408000" report "Expected legacy x2 initialization" severity failure;
      else
        assert word = x"400000" report "Expected explicit x1 initialization" severity failure;
      end if;
    else
      assert words <= values'length report "Unexpected DAC update" severity failure;
      assert word = x"3" & values(words - 1) & x"0"
        report "DAC tuning code changed" severity failure;
    end if;
    words <= words + 1;
  end process;

  stimulus : process
  begin
    wait for 100 ns;
    wait until falling_edge(clk);
    rst_n <= '1';
    wait until words = 1;
    for index in values'range loop
      wait until falling_edge(clk);
      value <= values(index);
      load <= '1';
      wait until falling_edge(clk);
      load <= '0';
      wait until words = index + 2;
    end loop;
    wait for 1 us;
    assert words = values'length + 1 report "Missing DAC update" severity failure;
    report "DAC serial checks passed";
    std.env.stop;
    wait;
  end process;
end;
