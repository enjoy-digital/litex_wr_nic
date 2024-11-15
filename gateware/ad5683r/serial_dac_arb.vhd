library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity serial_dac_arb is
  generic(
    g_invert_sclk    : boolean;
    g_num_data_bits  : integer := 16;
    g_num_extra_bits : integer := 8;
    g_enable_x2_gain : boolean := true
    );
  port(
    clk_i   : in std_logic;
    rst_n_i : in std_logic;

    val_i  : in std_logic_vector(g_num_data_bits-1 downto 0);
    load_i : in std_logic;

    dac_ldac_n_o    : out std_logic;
    dac_clr_n_o     : out std_logic;
    dac_sync_n_o    : out std_logic;
    dac_sclk_o      : out std_logic;
    dac_din_o       : out std_logic);

end serial_dac_arb;

architecture behavioral of serial_dac_arb is

  component serial_dac
    generic (
      g_num_data_bits  : integer;
      g_num_extra_bits : integer);
    port (
      clk_i          : in  std_logic;
      rst_n_i        : in  std_logic;
      value_i        : in  std_logic_vector(g_num_extra_bits+g_num_data_bits-1 downto 0);
      load_i         : in  std_logic;
      sclk_divsel_i  : in  std_logic_vector(2 downto 0);
      plldac_sclk_o  : out std_logic;
      plldac_sdata_o : out std_logic;
      plldac_sync_n_o: out std_logic;
      xdone_o        : out std_logic);
  end component;

  signal d1_ready     : std_logic;
  signal dac_data     : std_logic_vector(g_num_data_bits-1 downto 0);
  signal dac_cmd      : std_logic_vector(g_num_extra_bits-1 downto 0);
  signal dac_value    : std_logic_vector(g_num_data_bits+g_num_extra_bits-1 downto 0);
  signal dac_load     : std_logic;
  signal dac_cs_sel   : std_logic_vector(1 downto 0);
  signal dac_done     : std_logic;
  signal dac_sclk_int : std_logic;

  signal init_cnt : unsigned(7 downto 0);

  type t_state is (INIT, ENABLE_INT_REF, WAIT_DONE, LOAD_DAC, WAIT_DATA);
  signal state : t_state;

begin  -- behavioral

  dac_ldac_n_o  <= '0';
  dac_clr_n_o   <= '0';

  U_DAC : serial_dac
    generic map (
      g_num_data_bits  => 16,
      g_num_extra_bits => g_num_extra_bits)
    port map (
      clk_i         => clk_i,
      rst_n_i       => rst_n_i,
      value_i       => dac_value,
      load_i        => dac_load,
      sclk_divsel_i => "000",
      plldac_sclk_o   => dac_sclk_int,
      plldac_sdata_o  => dac_din_o,
      plldac_sync_n_o => dac_sync_n_o,
      xdone_o         => dac_done);

  p_drive_sclk: process(dac_sclk_int)
    begin
      if(g_invert_sclk) then
        dac_sclk_o <= not dac_sclk_int;
      else
        dac_sclk_o <= dac_sclk_int;
      end if;
    end process;

  process(clk_i)
  begin
    if rising_edge(clk_i) then
      if rst_n_i = '0' then
        dac_data   <= (others => '0');
        d1_ready   <= '0';
        dac_load   <= '0';
        init_cnt   <= (others => '0');
        state      <= INIT;
      else

        if(load_i = '1') then
          if(load_i = '1') then
            d1_ready <= '1';
            dac_data <= val_i;
          end if;
        else
          case state is
            when INIT =>
              if init_cnt(6) = '1' then
                state <= ENABLE_INT_REF;
              else
                init_cnt <= init_cnt + 1;
              end if;

            when ENABLE_INT_REF =>
              if g_enable_x2_gain then
                dac_value <= "0100"&"000010"&"00000000000000";  -- Enable internal reference / X2 Gain.
              else
                dac_value <= "0100"&"000000"&"00000000000000";  -- Enable internal reference / X1 Gain.
              end if;
              dac_load  <= '1';
              state <= LOAD_DAC;

            when WAIT_DATA =>
              if(d1_ready = '1') then
                dac_value   <= "0011"&dac_data&"0000";
                dac_load   <= '1';
                d1_ready   <= '0';
                state      <= LOAD_DAC;
              end if;

            when LOAD_DAC =>
              dac_load <= '0';
              state    <= WAIT_DONE;

            when WAIT_DONE =>
              if(dac_done = '1') then
                state <= WAIT_DATA;
              end if;
            when others => null;
          end case;
        end if;
      end if;
    end if;
  end process;

end behavioral;
