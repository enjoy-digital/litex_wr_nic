library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.gencores_pkg.all;
use work.wishbone_pkg.all;

entity fine_delay_ctrl is
generic (
    g_project_name        : string := "NORMAL"
);
port (
    rst_sys_n_i       : in  std_logic;
    clk_sys_i         : in  std_logic;
  
    fine_dly_req_i    : in  std_logic;
    fine_dly_sel_i    : in  std_logic;
    fine_dly_values_i : in  std_logic_vector(8 downto 0);
    fine_dly_busy_o   : out std_logic;

    delay_en_o        : out std_logic;
    delay_sload_o     : out std_logic;
    delay_sdin_o      : out std_logic;
    delay_sclk_o      : out std_logic
    );
end fine_delay_ctrl;

architecture struct of fine_delay_ctrl is
  
  type fdly_ctrl_state is (S_IDLE,S_CONFIG_EN,S_SPI_LOAD,S_SPI_TRANS,S_SPI_END);

  signal delay_en      : std_logic;      
  signal delay_sload   : std_logic;    
  signal delay_sdin    : std_logic;     
  signal delay_sclk    : std_logic;     
  signal fine_dly_busy : std_logic;
  signal channel_sel   : std_logic:='0';
  signal fine_dly      : std_logic_vector(8 downto 0):=(others=>'0');
  signal spi_data      : std_logic_vector(10 downto 0):=(others=>'0');

begin
  
  delay_en_o    <= delay_en;        
  delay_sload_o <= delay_sload;      
  delay_sdin_o  <= delay_sdin;       
  delay_sclk_o  <= delay_sclk;       
  fine_dly_busy_o <= fine_dly_busy; 

  P_FINE_DELAY_SPI: process(clk_sys_i)
    variable state  : fdly_ctrl_state := S_IDLE;
    variable spi_cnt: natural range 0 to 12;
  begin
    if rising_edge(clk_sys_i) then
      if (rst_sys_n_i='0') then
        spi_data      <= (others=>'0');
        channel_sel   <= '0';        
        delay_sload   <= '0';
        delay_sdin    <= '0';
        delay_sclk    <= '0';
        delay_en      <= '0';
        fine_dly_busy <= '0';
        spi_cnt       := 0;
        state         := S_IDLE;
      else 
        case(state) is
          when S_IDLE => 
            channel_sel <= '0';
            fine_dly    <= (others=>'0');
            spi_data      <= (others=>'0');
            delay_en    <= '0';
            delay_sload <= '0';
            delay_sdin  <= '0';
            delay_sclk  <= '0';
            fine_dly_busy <= '0';
            spi_cnt     := 0;
            if (fine_dly_req_i='1') then
              fine_dly       <= fine_dly_values_i;
              channel_sel    <= fine_dly_sel_i;
              fine_dly_busy  <= '1';
              state          := S_CONFIG_EN;
            end if;

          when S_CONFIG_EN =>
            spi_data <= fine_dly & '0' & channel_sel; 
            delay_en <= '1';
            state    := S_SPI_LOAD;

          when S_SPI_LOAD =>
            delay_sdin <= spi_data(0);
            delay_sclk <= '0';
            state      := S_SPI_TRANS;
            if(spi_cnt = 11) then
              delay_sload <= '1';
              state       := S_SPI_END;
            end if;

          when S_SPI_TRANS =>
            spi_data   <= '0' & spi_data(10 downto 1);
            delay_sclk <= '1';
            spi_cnt    := spi_cnt + 1;
            state      := S_SPI_LOAD;

          when S_SPI_END =>
            delay_sload <= '0';
            delay_en    <= '0';
            state       := S_IDLE;
            fine_dly_busy <= '0';
          when others =>
            state := S_IDLE;

        end case;
      end if;
    end if;
  end process P_FINE_DELAY_SPI;

end struct;

