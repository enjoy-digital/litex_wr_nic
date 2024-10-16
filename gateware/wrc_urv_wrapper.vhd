--------------------------------------------------------------------------------
-- CERN BE-CO-HT
-- Mock Turtle
-- https://gitlab.cern.ch/coht/mockturtle
--------------------------------------------------------------------------------
--
-- unit name:   mt_urv_wrapper
--
-- description: A small wrapper for the URV encompassing the internal RAM and
-- access to the RAM through CPU CSR register block.
--
--------------------------------------------------------------------------------
-- Copyright (c) 2014-2019 CERN (home.cern)
--------------------------------------------------------------------------------
-- Copyright and related rights are licensed under the Solderpad Hardware
-- License, Version 2.0 (the "License"); you may not use this file except
-- in compliance with the License. You may obtain a copy of the License at
-- http://solderpad.org/licenses/SHL-2.0.
-- Unless required by applicable law or agreed to in writing, software,
-- hardware and materials distributed under this License is distributed on an
-- "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express
-- or implied. See the License for the specific language governing permissions
-- and limitations under the License.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.genram_pkg.all;
use work.wishbone_pkg.all;
use work.wrc_cpu_csr_wbgen2_pkg.all;
use work.urv_pkg.all;

entity wrc_urv_wrapper is
  generic(
    g_IRAM_SIZE : integer;
    g_IRAM_INIT : string;
    g_CPU_ID    : integer);
  port(
    clk_sys_i : in  std_logic;
    rst_n_i   : in  std_logic;
    irq_i     : in  std_logic;
    dwb_o     : out t_wishbone_master_out;
    dwb_i     : in  t_wishbone_master_in;
    host_slave_i : in t_wishbone_slave_in;
    host_slave_o : out t_wishbone_slave_out
    );
end wrc_urv_wrapper;

architecture arch of wrc_urv_wrapper is

  impure function f_x_to_zero (x : std_logic_vector) return std_logic_vector
  is
    variable tmp : std_logic_vector(x'length-1 downto 0);
    variable found_undef : boolean := false;
  begin
-- synthesis translate_off
    for i in 0 to x'length-1 loop
      if( x(i) = 'U' or x(i) = 'Z' or x(i) = 'X' ) then
        found_undef := true;
      end if;

      if x(i) = '1' or x(i) = 'H' then
        tmp(i) := '1';
      else
        tmp(i) := '0';
      end if;
    end loop;
    return tmp;

    if found_undef then
      report "Undefined data value read from memory" severity warning;
    end if;

-- synthesis translate_on
    return x;
  end function f_x_to_zero;

  function f_swap_endian_32(x : std_logic_vector) return std_logic_vector
  is
  begin
    return x(7 downto 0) & x(15 downto 8) & x(23 downto 16) & x(31 downto 24);
  end f_swap_endian_32;

  signal cpu_rst        : std_logic;
  signal cpu_rst_d      : std_logic;

  signal im_addr  : std_logic_vector(31 downto 0);
  signal im_data  : std_logic_vector(31 downto 0);
  signal im_valid : std_logic;

  signal dm_addr, dm_data_s, dm_data_l                  : std_logic_vector(31 downto 0);
  signal dm_data_select                                 : std_logic_vector(3 downto 0);
  signal dm_load, dm_store, dm_load_done, dm_store_done : std_logic;

  signal dm_cycle_in_progress, dm_is_wishbone : std_logic;

  signal dm_mem_rdata, dm_wb_rdata : std_logic_vector(31 downto 0);
  signal dm_wb_write, dm_select_wb : std_logic;
  signal dm_data_write             : std_logic;

  constant c_INSN_NOP : std_logic_vector(31 downto 0) := x"0000_0013";
  signal dbg_insn     : std_logic_vector(31 downto 0);

  signal dwb_out         : t_wishbone_master_out;
begin

  dwb_o <= dwb_out;

  U_cpu_core : entity work.urv_cpu
    generic map (
      g_with_hw_debug => 0,
      g_with_hw_mulh  => 1,
      g_with_hw_mul   => 1,
      g_with_hw_div   => 1,
      g_with_compressed_insns => 1
    )
    port map (
      clk_i            => clk_sys_i,
      rst_i            => cpu_rst,
      irq_i            => irq_i,
      im_addr_o        => im_addr,
      im_data_i        => im_data,
      im_valid_i       => im_valid,
      dm_addr_o        => dm_addr,
      dm_data_s_o      => dm_data_s,
      dm_data_l_i      => dm_data_l,
      dm_data_select_o => dm_data_select,
      dm_store_o       => dm_store,
      dm_load_o        => dm_load,
      dm_load_done_i   => dm_load_done,
      dm_store_done_i  => dm_store_done,
      dbg_force_i      => '0',
      dbg_enabled_o    => open,
      dbg_insn_i       => (others => '0'),
      dbg_insn_set_i   => '0',
      dbg_insn_ready_o => open,
      dbg_mbx_data_i   => (others => '0'),
      dbg_mbx_write_i  => '0',
      dbg_mbx_data_o   => open);

  -- 1st MByte of the mem is the IRAM
  dm_is_wishbone <= '1' when dm_addr(31 downto 20) /= x"000" else '0';

  U_iram : generic_dpram
    generic map (
      g_DATA_WIDTH               => 32,
      g_SIZE                     => g_IRAM_SIZE,
      g_WITH_BYTE_ENABLE         => TRUE,
      g_ADDR_CONFLICT_RESOLUTION => "dont_care",
      g_INIT_FILE                => g_IRAM_INIT,
      g_FAIL_IF_FILE_NOT_FOUND   => TRUE,
      g_DUAL_CLOCK               => FALSE)
    port map (
      rst_n_i => rst_n_i,
      clka_i  => clk_sys_i,
      bwea_i  => "1111",
      wea_i   => '0',
      aa_i    => im_addr(f_log2_size(g_IRAM_SIZE)+1 downto 2),
      da_i    => (others => '0'),
      qa_o    => im_data,
      clkb_i  => clk_sys_i,
      bweb_i  => dm_data_select,
      web_i   => dm_data_write,
      ab_i    => dm_addr(f_log2_size(g_IRAM_SIZE)+1 downto 2),
      db_i    => dm_data_s,
      qb_o    => dm_mem_rdata);

  -- Wishbone bus arbitration / internal RAM access
  p_wishbone_master : process(clk_sys_i)
  begin
    if rising_edge(clk_sys_i) then
      if rst_n_i = '0' or cpu_rst = '1' then
        dwb_out.cyc          <= '0';
        dwb_out.stb          <= '0';
        dwb_out.adr          <= (others => '0');
        dwb_out.sel          <= x"0";
        dwb_out.we           <= '0';
        dwb_out.dat          <= (others => '0');
        dm_cycle_in_progress <= '0';
        dm_load_done         <= '0';
        dm_store_done        <= '0';
        dm_select_wb         <= '0';
      else
        if dm_cycle_in_progress = '0' then
          if dm_is_wishbone = '0' then
            -- access to internal memory
            dm_select_wb  <= '0';
            if dm_store = '1' then
              dm_load_done  <= '0';
              dm_store_done <= '1';
            elsif dm_load = '1' then
              dm_load_done  <= '1';
              dm_store_done <= '0';
            else
              dm_store_done <= '0';
              dm_load_done  <= '0';
            end if;
          else
            if dm_load = '1' or dm_store = '1' then
              dwb_out.cyc          <= '1';
              dwb_out.stb          <= '1';
              dwb_out.we           <= dm_store;
              dm_wb_write          <= dm_store;
              dwb_out.adr          <= dm_addr;
              dwb_out.dat          <= dm_data_s;
              dwb_out.sel          <= dm_data_select;
              dm_load_done         <= '0';
              dm_store_done        <= '0';
              dm_cycle_in_progress <= '1';
            else
              dm_store_done        <= '0';
              dm_load_done         <= '0';
              dm_cycle_in_progress <= '0';
            end if;
          end if;
        else
          if dwb_i.stall = '0' then
            dwb_out.stb <= '0';
          end if;

          if dwb_i.ack = '1' then
            if dm_wb_write = '0' then
              dm_wb_rdata  <= f_x_to_zero(dwb_i.dat);
              dm_select_wb <= '1';
              dm_load_done <= '1';
            else
              dm_store_done <= '1';
              dm_select_wb  <= '0';
            end if;

            dm_cycle_in_progress <= '0';
            dwb_out.cyc          <= '0';
          end if;
        end if;
      end if;
    end if;
  end process p_wishbone_master;

  dm_data_write <= not dm_is_wishbone and dm_store;
  dm_data_l     <= dm_wb_rdata when dm_select_wb = '1' else dm_mem_rdata;

  p_im_valid : process(clk_sys_i)
  begin
    if rising_edge(clk_sys_i) then
      if cpu_rst = '1' then
        im_valid  <= '0';
        cpu_rst_d <= '1';
      else
        cpu_rst_d <= cpu_rst;
        im_valid  <= (not cpu_rst_d);
      end if;
    end if;
  end process p_im_valid;

  cpu_rst        <= not rst_n_i;

end architecture arch;
