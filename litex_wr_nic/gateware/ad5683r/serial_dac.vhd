-------------------------------------------------------------------------------
-- title      : serial dac interface
-- project    : white rabbit switch
-------------------------------------------------------------------------------
-- file       : serial_dac.vhd
-- author     : paas, slayer
-- company    : cern be-co-ht
-- created    : 2010-02-25
-- last update: 2011-05-10
-- platform   : fpga-generic
-- standard   : vhdl'87
-------------------------------------------------------------------------------
-- description: the dac unit provides an interface to a 16 bit serial digita to analogue converter (max5441, spi?/qspi?/microwire? compatible) 
--
-------------------------------------------------------------------------------
-- copyright (c) 2010 cern
-------------------------------------------------------------------------------
-- revisions  :1
-- date        version  author  description
-- 2009-01-24  1.0      paas    created
-- 2010-02-25  1.1      slayer  modified for rev 1.1 switch
-- 2012-10-15  2.0      pwb     modified for ad5663r of cute-wr
-------------------------------------------------------------------------------


library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity serial_dac is

  generic (
    g_num_data_bits  : integer := 16;
    g_num_extra_bits : integer := 8
    );

  port (
-- clock & reset
    clk_i   : in std_logic;
    rst_n_i : in std_logic;

-- channel 1 value and value load strobe
    value_i  : in std_logic_vector(g_num_extra_bits+g_num_data_bits-1 downto 0);
    load_i   : in std_logic;

-- sclk divider: 000 = clk_i/8 ... 111 = clk_i/1024
    sclk_divsel_i : in std_logic_vector(2 downto 0);

-- dac i/f
    plldac_sclk_o   : out std_logic;
    plldac_sdata_o  : out std_logic;
    plldac_sync_n_o : out std_logic;   

    xdone_o : out std_logic
    );
end serial_dac;


architecture syn of serial_dac is

  signal divider        : unsigned(11 downto 0);
  signal datash         : std_logic_vector(g_num_data_bits + g_num_extra_bits-1 downto 0);
  signal bitcounter     : std_logic_vector(g_num_data_bits + g_num_extra_bits+1 downto 0);
  signal endsendingdata : std_logic;
  signal sendingdata    : std_logic;
  signal idacclk        : std_logic;
  signal ivalidvalue    : std_logic;

  signal divider_muxed : std_logic;

begin

  -- modified by weibin
  
  select_divider : process (divider, sclk_divsel_i)
  begin  -- process
    case sclk_divsel_i is
      when "000"  => divider_muxed <= divider(2);  -- sclk = clk_i/8
      when "001"  => divider_muxed <= divider(3);  -- sclk = clk_i/16
      when "010"  => divider_muxed <= divider(4);  -- sclk = clk_i/32
      when "011"  => divider_muxed <= divider(5);  -- sclk = clk_i/64
      when "100"  => divider_muxed <= divider(6);  -- sclk = clk_i/128
      when "101"  => divider_muxed <= divider(7);  -- sclk = clk_i/256
      when "110"  => divider_muxed <= divider(8);  -- sclk = clk_i/512
      when "111"  => divider_muxed <= divider(9);  -- sclk = clk_i/1024
      when others => null;
    end case;
  end process;


  ivalidvalue <= load_i;

  process(clk_i, rst_n_i)
  begin
    if rising_edge(clk_i) then
      if rst_n_i = '0' then
        sendingdata <= '0';
      else
        if ivalidvalue = '1' and sendingdata = '0' then
          sendingdata <= '1';
        elsif endsendingdata = '1' then
          sendingdata <= '0';
        end if;
      end if;
    end if;
  end process;

  process(clk_i)
  begin
    if rising_edge(clk_i) then
      if ivalidvalue = '1' then
        divider <= (divider'high downto 1 => '0') & '1';
      elsif sendingdata = '1' then
        if(divider_muxed = '1') then
          divider <= (divider'high downto 1 => '0') & '1';
        else
          divider <= divider + 1;
        end if;
      elsif endsendingdata = '1' then
        divider <= (divider'high downto 1 => '0') & '1';
      end if;
    end if;
  end process;


  process(clk_i, rst_n_i)
  begin
    if rising_edge(clk_i) then
      if rst_n_i = '0' then
        idacclk <= '1';                 -- 0
      else
        if ivalidvalue = '1' then
          idacclk <= '1';               -- 0
        elsif divider_muxed = '1' then
          idacclk <= not(idacclk);
        elsif endsendingdata = '1' then
          idacclk <= '1';               -- 0
        end if;
      end if;
    end if;
  end process;

  process(clk_i, rst_n_i)
  begin
    if rising_edge(clk_i) then
      if rst_n_i = '0' then
        datash <= (others => '0');
      else
        if ivalidvalue = '1' and sendingdata = '0' then
          datash        <= value_i;
        elsif sendingdata = '1' and divider_muxed = '1' and idacclk = '0' then
          datash(0)                    <= datash(datash'left);
          datash(datash'left downto 1) <= datash(datash'left - 1 downto 0);
        end if;
      end if;
    end if;
  end process;

  process(clk_i)
  begin
    if rising_edge(clk_i) then
      if ivalidvalue = '1' and sendingdata = '0' then
        bitcounter(0)                        <= '1';
        bitcounter(bitcounter'left downto 1) <= (others => '0');
      elsif sendingdata = '1' and to_integer(divider) = 1 and idacclk = '1' then
        bitcounter(0)                        <= '0';
        bitcounter(bitcounter'left downto 1) <= bitcounter(bitcounter'left - 1 downto 0);
      end if;
    end if;
  end process;

  endsendingdata <= bitcounter(bitcounter'left);

  xdone_o <= not sendingdata;

  plldac_sdata_o <= datash(datash'left);
  
  plldac_sync_n_o <= not sendingdata;

  plldac_sclk_o <= idacclk;


end syn;
