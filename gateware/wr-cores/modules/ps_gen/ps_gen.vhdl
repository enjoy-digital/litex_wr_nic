-------------------------------------------------------------------------------
-- Title      : MMCM Dynamic Phase Shift DAC converter
-- Project    : WR PTP Core
-- URL        : http://www.ohwr.org/projects/wr-cores/wiki/Wrpc_core
-------------------------------------------------------------------------------
-- File       : ps_gen.vhd
-- Author(s)  : Frederik Pfautsch <frederik.pfautsch@missinglinkelectronics.com>
-- Company    : Missing Link Electronics
-- Created    : 2023-08-03
-- Last update: 2023-08-03
-- Standard   : VHDL'93
-------------------------------------------------------------------------------
-- Description: Initiates the dynamic phase shift of an MMCM regularly
--              based on a n-bit unsigned input value.
-------------------------------------------------------------------------------
-- Copyright (c) 2023 CERN
-------------------------------------------------------------------------------
-- GNU LESSER GENERAL PUBLIC LICENSE
--
-- This source file is free software; you can redistribute it
-- and/or modify it under the terms of the GNU Lesser General
-- Public License as published by the Free Software Foundation;
-- either version 2.1 of the License, or (at your option) any
-- later version.
--
-- This source is distributed in the hope that it will be
-- useful, but WITHOUT ANY WARRANTY; without even the implied
-- warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
-- PURPOSE.  See the GNU Lesser General Public License for more
-- details.
--
-- You should have received a copy of the GNU Lesser General
-- Public License along with this source; if not, download it
-- from http://www.gnu.org/licenses/lgpl-2.1.html
--
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

entity ps_gen is
  generic (
    WIDTH : integer := 16;
    DIV : integer := 1;
    MULT : integer := 1;
    ENABLE_MARK_DEBUG : integer range 0 to 1 := 0
  );
  port (
    pswidth : in std_logic_vector(WIDTH-1 downto 0);
    pswidth_set : in std_logic;
    pswidth_clk : in std_logic;

    psclk : in std_logic;
    psdone : in std_logic;
    psen : out std_logic;
    psincdec : out std_logic
  );
end ps_gen;

architecture behavioral of ps_gen is
  --------------------------------------------------------------------------------
  -- constants / types / signals / attributes / functions
  --------------------------------------------------------------------------------
  attribute X_INTERFACE_INFO : string;
  attribute X_INTERFACE_PARAMETER : string;
  attribute X_INTERFACE_MODE : STRING;

  -- clock
  attribute X_INTERFACE_INFO of psclk : signal is "xilinx.com:signal:clock:1.0 psclk CLK";
  attribute X_INTERFACE_INFO of pswidth_clk : signal is "xilinx.com:signal:clock:1.0 pswidth_clk CLK";

  constant PSWIDTH_CENTER : integer := 2**(WIDTH-1);

  signal cntr : integer := 0;
  signal running_cntr : unsigned(WIDTH-2 downto 0) := (others => '0');

  signal pswidth_reg : std_logic_vector(WIDTH-1 downto 0) := (others => '0');
  signal psincdec_reg : std_logic := '0';

  signal pswidth_cdc_t, pswidth_cdc : std_logic_vector(WIDTH-2 downto 0) := (others => '0');
  signal psincdec_cdc_t, psincdec_cdc : std_logic := '0';

  attribute ASYNC_REG : string;
  attribute ASYNC_REG of pswidth_cdc_t : signal is "true";
  attribute ASYNC_REG of pswidth_cdc : signal is "true";
  attribute ASYNC_REG of psincdec_cdc_t : signal is "true";
  attribute ASYNC_REG of psincdec_cdc : signal is "true";

  attribute SHREG_EXTRACT : string;
  attribute SHREG_EXTRACT of pswidth_cdc_t : signal is "no";
  attribute SHREG_EXTRACT of pswidth_cdc : signal is "no";
  attribute SHREG_EXTRACT of psincdec_cdc_t : signal is "no";
  attribute SHREG_EXTRACT of psincdec_cdc : signal is "no";

  attribute MARK_DEBUG : string;
  attribute MARK_DEBUG of pswidth : signal is integer'image(ENABLE_MARK_DEBUG);
  attribute MARK_DEBUG of pswidth_set : signal is integer'image(ENABLE_MARK_DEBUG);
  attribute MARK_DEBUG of psdone : signal is integer'image(ENABLE_MARK_DEBUG);
  attribute MARK_DEBUG of psen : signal is integer'image(ENABLE_MARK_DEBUG);
  attribute MARK_DEBUG of psincdec : signal is integer'image(ENABLE_MARK_DEBUG);
  attribute MARK_DEBUG of cntr : signal is integer'image(ENABLE_MARK_DEBUG);
  attribute MARK_DEBUG of running_cntr : signal is integer'image(ENABLE_MARK_DEBUG);
  attribute MARK_DEBUG of pswidth_reg : signal is integer'image(ENABLE_MARK_DEBUG);
begin
  PROC_pswidth: process
    variable pswidth_t : signed(WIDTH downto 0);
    variable pswidth_scaled_t : signed(2*(WIDTH+1)-1 downto 0);
  begin
    wait until rising_edge(pswidth_clk);

    if pswidth_set = '1' then
      -- First, center pswidth around 0, then convert pswidth to an
      -- unsigned value (keep in mind that abs(-32768) does not fit in
      -- 15 bit, hence subtract 1). Last, scale with appropriate factors.
      -- Make sure that DIV is a multiple of 2, otherwise complex logic
      -- will be genereated!
      pswidth_t := signed('0' & pswidth) - PSWIDTH_CENTER;
      pswidth_t := pswidth_t + signed'('0' & pswidth_t(WIDTH));
      psincdec_reg <= pswidth_t(WIDTH);

      pswidth_scaled_t := (pswidth_t * MULT) / DIV;
      pswidth_reg <= std_logic_vector(abs(pswidth_scaled_t(WIDTH-1 downto 0)));
    end if;
  end process;

  PROC_cdc: process
  begin
    wait until rising_edge(psclk);

    pswidth_cdc_t <= pswidth_reg(WIDTH-2 downto 0);
    pswidth_cdc <= pswidth_cdc_t;

    psincdec_cdc_t <= psincdec_reg;
    psincdec_cdc <= psincdec_cdc_t;
  end process;

  PROC_cntr: process
    variable cntr_t : integer;
  begin
    wait until rising_edge(psclk);

    cntr_t := cntr + 1;
    if cntr_t = 13 then
      cntr_t := 0;
    end if;

    cntr <= cntr_t;
    psen <= '0';

    if cntr_t = 0 or psdone = '1' then
      cntr <= 0;
      running_cntr <= running_cntr + unsigned(pswidth_cdc);
      psincdec <= psincdec_cdc;

      if running_cntr + unsigned(pswidth_cdc) < running_cntr then
        psen <= '1';
      end if;
    end if;
  end process;
end behavioral;
