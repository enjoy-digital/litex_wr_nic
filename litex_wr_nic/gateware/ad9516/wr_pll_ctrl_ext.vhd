library ieee;
use ieee.std_logic_1164.all;

use ieee.numeric_std.all;

library work;
use work.wr_pll_ctrl_ext_pkg.all;

entity wr_pll_ctrl_ext is
generic (
  g_project_name : string := "NORMAL";
  -- clk_spi = clk/(div+1)/2 --> 6.25MHz sclk for 62.5 input (up to 25MHz)
  g_spi_clk_freq : std_logic_vector(31 downto 0) := x"00000004"
);
port (
    clk_i        : in  std_logic;
    rst_n_i      : in  std_logic;
    --- pll status/control
    pll_lock_i   : in  std_logic:='0';
    pll_reset_n_o: out std_logic;
    pll_status_i : in  std_logic:='0';
    pll_refsel_o : out std_logic;
    pll_sync_n_o : out std_logic;
    -- spi bus - pll control
    pll_cs_n_o   : out std_logic;
    pll_sck_o    : out std_logic;
    pll_mosi_o   : out std_logic;
    pll_miso_i   : in  std_logic;
    -- spi controller status
    done_o       : out std_logic);
end wr_pll_ctrl_ext;

architecture Behavioral of wr_pll_ctrl_ext is

  component spi_top is
  port(
    wb_clk_i   : in  std_logic;
    wb_rst_i   : in  std_logic;
    wb_adr_i   : in  std_logic_vector(4 downto 0);
    wb_dat_i   : in  std_logic_vector(31 downto 0);
    wb_dat_o   : out std_logic_vector(31 downto 0);
    wb_sel_i   : in  std_logic_vector(3 downto 0);
    wb_we_i    : in  std_logic;
    wb_stb_i   : in  std_logic;
    wb_cyc_i   : in  std_logic;
    wb_ack_o   : out std_logic;
    wb_err_o   : out std_logic;
    int_o      : out std_logic;
    ss_pad_o   : out std_logic_vector(8-1 downto 0);
    sclk_pad_o : out std_logic;
    mosi_pad_o : out std_logic;
    miso_pad_i : in  std_logic
  );
  end component;

-- fsm master
  type t_master is (T_IDLE, T_spi_send, T_read_reg, T_reg_conf, T_set_ctrl, T_set_divclk, T_wait);
  signal fsm_master : t_master := T_IDLE;
  signal next_state : t_master := T_IDLE;

  -- Wishbone slave port
  signal wb_rst_i   : std_logic;
  signal wb_sel_i   : std_logic_vector(3 downto 0) := (others => '0');
  signal wb_cyc_i   : std_logic;
  signal wb_stb_i   : std_logic;
  signal wb_we_i    : std_logic;
  signal wb_adr_i   : std_logic_vector(4 downto 0):= (others => '0');
  signal wb_dat_i   : std_logic_vector(31 downto 0):= (others => '0');
  signal wb_dat_o   : std_logic_vector(31 downto 0):= (others => '0');
  signal wb_ack_o   : std_logic:= '0';
  signal wb_err_o   : std_logic;
  signal wb_int_o   : std_logic;

  --- SPI signals                                     
  signal ss_pad_o   : std_logic_vector(8-1 downto 0);
  signal sclk_pad_o : std_logic;
  signal mosi_pad_o : std_logic;
  signal miso_pad_i : std_logic;

  -- SPI State Machine signals
  signal spi_time_out    : std_logic;
  signal spi_adr_pos     : integer range 0 to (c_spi_addr_array'length+1); 
  signal spi_ack_cnt     : integer range 0 to (c_spi_addr_array'length+1);
  signal spi_dat_cnt     : integer range 0 to (c_spi_addr_array'length+1);
  signal spi_test_err    : std_logic;
  signal spi_active_en   : std_logic := '0'; 
  signal spi_enable      : std_logic := '0'; 
  signal spi_read_reg    : std_logic_vector(15 downto 0) := (others => '0');
  
  signal spi_data_array_wr  : t_data_array(0 to ((c_spi_data_array'length)-1));
  signal spi_data_array_rd  : t_data_array(0 to ((c_spi_data_array'length)-1));
  signal spi_addr_array     : t_addr_array(0 to ((c_spi_data_array'length)-1));

  signal spi_write_done  : std_logic := '0'; 
  signal spi_error       : std_logic;
  signal spi_done        : std_logic;

begin

  spi_data_array_wr <= c_spi_data_array;
  spi_data_array_rd(0 to ((c_spi_data_array'length)-2)) <= c_spi_data_array(0 to ((c_spi_data_array'length)-2));
  spi_data_array_rd((c_spi_data_array'length)-1) <= x"00";
  spi_addr_array    <= c_spi_addr_array;

  -- SPI signals
  pll_cs_n_o   <= ss_pad_o(0);
  process (clk_i)
  begin
    if rising_edge(clk_i) then
      pll_sck_o    <= sclk_pad_o;
      pll_mosi_o   <= mosi_pad_o;
      miso_pad_i   <= pll_miso_i;
    end if;
  end process;
  pll_refsel_o <= '0'; -- ref1 (signal low) , ref2 (signal high)
  pll_sync_n_o <= '1';
  pll_reset_n_o  <= rst_n_i;
  done_o       <= spi_done;

  wb_rst_i  <= (not rst_n_i);

  inst_spi_top : spi_top 
  port map(
    wb_clk_i   => clk_i,
    wb_rst_i   => wb_rst_i,
    wb_adr_i   => wb_adr_i,
    wb_dat_i   => wb_dat_i,
    wb_dat_o   => wb_dat_o,
    wb_sel_i   => wb_sel_i,
    wb_we_i    => wb_we_i,
    wb_stb_i   => wb_stb_i,
    wb_cyc_i   => wb_cyc_i,
    wb_ack_o   => wb_ack_o,
    wb_err_o   => wb_err_o,
    int_o      => wb_int_o,
    ss_pad_o   => ss_pad_o,
    sclk_pad_o => sclk_pad_o,
    mosi_pad_o => mosi_pad_o,
    miso_pad_i => miso_pad_i 
  );

  p_PLL_start: process(clk_i) is
    variable v_cnt_enable : integer := 0;
  begin
    if rising_edge(clk_i) then
      if (rst_n_i = '0')then
        v_cnt_enable := 0;
        spi_enable <= '0';
      else
        if (v_cnt_enable<40) then
          spi_enable <= '0';
          v_cnt_enable := v_cnt_enable + 1;
        elsif(v_cnt_enable<90) then
          spi_enable <= '1';
          v_cnt_enable := v_cnt_enable + 1;
        else
          spi_enable <= '0';
          if(spi_error = '1') then
            v_cnt_enable := 0;
          end if;
        end if;
      end if;
    end if;
  end process p_PLL_start;

  p_spi_master : process(clk_i) is
    constant c_time_out_max_cnt : integer := 200000000;
    variable time_out_cnt       : integer := 0;
  begin
    if rising_edge(clk_i) then
      if (rst_n_i = '0')then
        wb_sel_i       <= "0000";
        wb_cyc_i       <= '0';
        wb_stb_i       <= '0';
        wb_we_i        <= '0';  
        wb_adr_i       <= (others=>'0');
        wb_dat_i       <= (others=>'0');
        spi_time_out   <= '0';    -- Default value
        time_out_cnt   := 0;
        spi_done       <= '0';
        spi_adr_pos    <= 0;
        spi_ack_cnt    <= 0;
        spi_dat_cnt    <= 0;
        spi_test_err   <= '0'; 
        spi_write_done  <= '1';
        spi_active_en  <= '0';
        fsm_master     <= T_IDLE;
        next_state     <= T_IDLE;
        spi_read_reg <= (others=>'0');
      else
        case fsm_master is
          when T_IDLE =>
            spi_adr_pos   <= 0; 
            spi_ack_cnt   <= 0;
            spi_dat_cnt   <= 0;
            spi_test_err  <= '0'; -- Default value
            spi_time_out  <= '0'; -- Default value
            if (spi_active_en = '1') then
              spi_done      <= '0';
              next_state    <= T_set_divclk;
              fsm_master    <= T_wait; --(next state T_set_divclk)
              wb_sel_i      <= "1111";
              wb_cyc_i      <= '1';
              wb_stb_i      <= '1';
              wb_we_i       <= '1';
              wb_dat_i      <= x"00000001";
              wb_adr_i      <= '1' & x"8";     
            else
              wb_sel_i      <= "0000";
              wb_we_i       <= '0';
              wb_cyc_i      <= '0';
              wb_stb_i      <= '0';       
              fsm_master    <= T_IDLE;
              next_state    <= T_IDLE;
            end if;
          
          when T_wait =>
            if time_out_cnt > c_time_out_max_cnt then 
              wb_sel_i      <= "0000";
              wb_cyc_i      <= '0';
              wb_stb_i      <= '0';
              wb_we_i       <= '0';
              time_out_cnt  := 0;
              spi_test_err  <= '0';
              spi_time_out  <= '1';
              spi_write_done <= '1'; 
              spi_active_en <= '0';
              fsm_master    <= T_IDLE;
              next_state    <= T_IDLE;
            else
              wb_we_i       <= '1';
              wb_cyc_i      <= '1';
              if(wb_ack_o  = '1') then
                time_out_cnt := 0;
                wb_sel_i    <= "0000";
                wb_stb_i    <= '0';
                fsm_master  <= next_state;
                if next_state = T_spi_send then
                  wb_we_i     <= '0';
                  wb_cyc_i    <= '0';
                  spi_ack_cnt <= spi_ack_cnt + 1;
                end if;
              else
                 if (wb_int_o = '1' and next_state = T_wait) then 
                   time_out_cnt := 0;
                   if (spi_write_done = '1') then
                     fsm_master  <= T_reg_conf;
                   else
                     fsm_master  <= T_read_reg;
                   end if;
                 else  -- This state is just to end the loop in case something goes wrong (2 seconds later)
                   time_out_cnt := time_out_cnt + 1;
                   fsm_master     <= T_wait;
                 end if;
              end if;
            end if;

          when T_set_divclk =>  
            next_state  <= T_set_ctrl;
            fsm_master  <= T_wait;  --(next state T_set_ctrl)
            wb_sel_i    <= "1111";
            wb_cyc_i    <= '1';
            wb_stb_i    <= '1';
            wb_we_i     <= '1'; 
            wb_dat_i    <= g_spi_clk_freq;
            wb_adr_i    <= '1' & x"4";
               
          when T_set_ctrl => 
            next_state  <= T_reg_conf;
            fsm_master  <= T_wait;--(next state T_reg_conf)
            wb_sel_i    <= "1111";
            wb_cyc_i    <= '1';
            wb_stb_i    <= '1';
            wb_we_i     <= '1'; 
            wb_adr_i    <= '1' & x"0";
            wb_dat_i (31 downto 14) <= (others => '0');
            wb_dat_i (13 downto 0)  <= "1101000" & "0011000"; -- (6:0) how many bits are transmitted in one tranfer

          when T_reg_conf => 
            next_state  <= T_spi_send;
            wb_dat_i (31 downto 24)  <= (others => '0');
            wb_adr_i    <= '0' & x"0";
            wb_we_i     <= '1';
            if(spi_write_done = '1') then
              -- Write first data
              wb_dat_i (20 downto 8)  <= spi_addr_array(spi_adr_pos)(12 downto 0);
              wb_dat_i (7 downto 0)   <= spi_data_array_wr(spi_adr_pos);
              wb_dat_i (23 downto 21) <= "000"; -- 2-R/W ; (1:0)-Bytes to transfer
            else
              -- Read first data
              wb_dat_i (20 downto 8)  <= spi_addr_array(spi_adr_pos)(12 downto 0);
              wb_dat_i (7 downto 0)   <= spi_data_array_wr(spi_adr_pos); -- not necesary
              wb_dat_i (23 downto 21) <= "100"; -- 2-R/W ; (1:0)-Bytes to transfer
            end if;

            if(spi_ack_cnt <c_spi_addr_array'length) then
              fsm_master  <= T_wait;
              wb_sel_i    <= "1111";
              wb_stb_i    <= '1';
              wb_cyc_i    <= '1';
            else
              wb_sel_i    <= "0000";
              wb_cyc_i    <= '0';
              wb_stb_i    <= '0';
              if spi_write_done  = '0' then
                spi_done       <= '1';
                spi_active_en  <= '0';
                spi_write_done <= '1'; 
              else
                -- after write all registers, read them to verify their value
                spi_write_done  <= '0';
              end if;
              fsm_master  <= T_IDLE;
            end if; 
 
          when T_spi_send =>
            next_state   <= T_wait;
            fsm_master   <= T_wait;
            --check if the pipeline is not stalled
            if(spi_dat_cnt < c_spi_addr_array'length) then
              spi_dat_cnt <= spi_dat_cnt + 1;     
              wb_sel_i    <= "1111";
              wb_stb_i    <= '1';
              wb_cyc_i    <= '1';
              wb_we_i     <= '1';
              wb_adr_i    <= '1' & x"0";
              wb_dat_i (31 downto 14) <= (others => '0');
              -- 8-active transfer + Control register(same as T_set_ctrl)
              wb_dat_i (13 downto 0)  <= "1101010" & "0011000";
              if( spi_adr_pos < (c_spi_addr_array'length - 1)) then
                spi_adr_pos   <= spi_adr_pos + 1;
              else
                spi_adr_pos   <= 0;
              end if;
            end if;

          when T_read_reg => 
            next_state    <= T_read_reg;
            wb_sel_i      <= "0011";
            wb_we_i       <= '0';
            wb_adr_i      <= '0' & x"0";
            spi_read_reg  <= x"00" & spi_data_array_rd(spi_ack_cnt-1);
            -- Read data
            if wb_ack_o  = '1' then
              -- neither register 0x02/0x03 nor 0x001F/0x1A3 are tested because they are reserved
              if ((spi_read_reg(7 downto 0) = wb_dat_o (7 downto 0)) or (spi_ack_cnt = 3 or spi_ack_cnt = 4 or spi_ack_cnt = 21 or spi_ack_cnt = 63)) then
                spi_test_err       <= '0';
              else
                spi_test_err       <= '1';
              end if;
              wb_stb_i    <= '0';
              wb_cyc_i    <= '0';
              fsm_master  <= T_reg_conf;
            else
              fsm_master  <= T_read_reg;
              wb_stb_i    <= '1';
              wb_cyc_i    <= '1';
            end if;

          when others =>
            wb_sel_i    <= "0000";
            wb_cyc_i    <= '0';
            wb_stb_i    <= '0';
            wb_we_i     <= '0';
            fsm_master  <= T_IDLE;

        end case;

        if(spi_enable = '1' and spi_active_en = '0') then
          spi_active_en <= '1';
        end if;

      end if;
    end if;
  end process p_spi_master;


  p_error : process( clk_i ) is
  begin
    if rising_edge(clk_i) then
      if (rst_n_i = '0')then
        spi_error <= '0';
      else 
        if (spi_enable = '1' and spi_active_en = '0') then
          spi_error <= '0';
        elsif (wb_err_o = '1' or (spi_test_err = '1' or spi_time_out = '1')) then
          spi_error <= '1';
        end if;
      end if;
    end if;
  end process p_error;

end Behavioral;

