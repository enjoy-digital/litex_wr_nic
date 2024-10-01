--
-- This file is part of LiteX-WR-NIC.
--
-- Copyright (c) 2024 Warsaw University of Technology
-- Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
-- SPDX-License-Identifier: BSD-2-Clause
--
-- Note: This wrapper translates between VHDL records/structures and Verilog signals, since records
-- cannot be directly instantiated in LiteX/Migen Verilog instances.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity xwb_clock_crossing_wrapper is
   generic(
      g_size : natural := 16);
   port(
      -- Slave control port
      slave_clk_i    : in  std_logic;
      slave_rst_n_i  : in  std_logic;
      slave_cyc_i    : in  std_logic;
      slave_stb_i    : in  std_logic;
      slave_adr_i    : in  std_logic_vector(31 downto 0); -- 32-bit address width
      slave_sel_i    : in  std_logic_vector(3 downto 0);  -- 4-bit byte select
      slave_we_i     : in  std_logic;
      slave_dat_i    : in  std_logic_vector(31 downto 0); -- 32-bit data width
      slave_ack_o    : out std_logic;
      slave_err_o    : out std_logic;
      slave_rty_o    : out std_logic;
      slave_stall_o  : out std_logic;
      slave_dat_o    : out std_logic_vector(31 downto 0); -- 32-bit data width
      -- Master reader port
      master_clk_i   : in  std_logic;
      master_rst_n_i : in  std_logic;
      master_ack_i   : in  std_logic;
      master_err_i   : in  std_logic;
      master_rty_i   : in  std_logic;
      master_stall_i : in  std_logic;
      master_dat_i   : in  std_logic_vector(31 downto 0); -- 32-bit data width
      master_cyc_o   : out std_logic;
      master_stb_o   : out std_logic;
      master_adr_o   : out std_logic_vector(31 downto 0); -- 32-bit address width
      master_sel_o   : out std_logic_vector(3 downto 0);  -- 4-bit byte select
      master_we_o    : out std_logic;
      master_dat_o   : out std_logic_vector(31 downto 0); -- 32-bit data width
      -- Flow control back-channel for acks
      slave_ready_o  : out std_logic;
      slave_stall_i  : in  std_logic
   );
end xwb_clock_crossing_wrapper;

architecture rtl of xwb_clock_crossing_wrapper is
begin

   -- Instance of the original Verilog module
   xwb_clock_crossing_inst : entity work.xwb_clock_crossing
   generic map(
      g_size => g_size
   )
   port map(
      slave_clk_i    => slave_clk_i,
      slave_rst_n_i  => slave_rst_n_i,
      slave_i.cyc    => slave_cyc_i,
      slave_i.stb    => slave_stb_i,
      slave_i.adr    => slave_adr_i,
      slave_i.sel    => slave_sel_i,
      slave_i.we     => slave_we_i,
      slave_i.dat    => slave_dat_i,
      slave_o.ack    => slave_ack_o,
      slave_o.err    => slave_err_o,
      slave_o.rty    => slave_rty_o,
      slave_o.stall  => slave_stall_o,
      slave_o.dat    => slave_dat_o,
      master_clk_i   => master_clk_i,
      master_rst_n_i => master_rst_n_i,
      master_i.ack   => master_ack_i,
      master_i.err   => master_err_i,
      master_i.rty   => master_rty_i,
      master_i.stall => master_stall_i,
      master_i.dat   => master_dat_i,
      master_o.cyc   => master_cyc_o,
      master_o.stb   => master_stb_o,
      master_o.adr   => master_adr_o,
      master_o.sel   => master_sel_o,
      master_o.we    => master_we_o,
      master_o.dat   => master_dat_o,
      slave_ready_o  => slave_ready_o,
      slave_stall_i  => slave_stall_i
   );

end rtl;
