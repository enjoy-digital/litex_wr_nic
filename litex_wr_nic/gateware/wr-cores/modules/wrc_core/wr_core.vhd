-------------------------------------------------------------------------------
-- SPDX-FileCopyrightText: 2011 CERN (home.cern)
--
-- SPDX-License-Identifier: CERN-OHL-W-2.0+
-------------------------------------------------------------------------------
-- Title      : WhiteRabbit PTP Core
-- Project    : WhiteRabbit
-------------------------------------------------------------------------------
-- File       : wr_core.vhd
-- Author     : Grzegorz Daniluk <grzegorz.daniluk@cern.ch>
-- Company    : CERN (BE-CO-HT)
-- Created    : 2011-02-02
-- Platform   : FPGA-generics
-- Standard   : VHDL
-------------------------------------------------------------------------------
-- Description:
-- WR PTP Core is a HDL module implementing a complete gigabit Ethernet
-- interface (MAC + PCS + PHY) with integrated PTP slave ordinary clock
-- compatible with White Rabbit protocol. It performs subnanosecond clock
-- synchronization via WR protocol and also acts as an Ethernet "gateway",
-- providing access to TX/RX interfaces of the built-in WR MAC.
--
-- Starting from version 2.0 all modules are interconnected with pipelined
-- wishbone interface (using wb crossbars). Separate pipelined wishbone bus is
-- used for passing packets between Endpoint, Mini-NIC and External
-- MAC interface.
-------------------------------------------------------------------------------
-- Memory map:
--      0x000: Minic
--      0x100: Endpoint
--      0x200: Softpll
--      0x300: PPS gen
--      0x400: Syscon
--      0x500: UART
--      0x600: OneWire
--      0x800: WRPC diagnostics registers (for user)
--      0x900: WRPC diagnostics registers (for firmware)
--      0xa00: freq monitor
--      0xb00: cpu csr
--      0xc00: secbar sdb
--      0x8000: Auxillary space (Etherbone config, etc)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.wrcore_pkg.all;
use work.gencores_pkg.all;
use work.genram_pkg.all;
use work.wishbone_pkg.all;
use work.endpoint_pkg.all;
use work.wr_fabric_pkg.all;
use work.softpll_pkg.all;
use work.wr_timecode_pkg.all;

entity wr_core is
  generic(
    --if set to 1, then blocks in PCS use smaller calibration counter to speed
    --up simulation
    g_simulation                : integer                        := 0;
    -- set to false to reduce the number of information printed during simulation
    g_verbose                   : boolean                        := true;
    g_with_external_clock_input : boolean                        := true;
    g_ram_address_space_size_kb : integer                        := 128;
   --
    g_board_name                : string                         := "NA  ";
    g_flash_secsz_kb            : integer                        := 256;        -- default for SVEC (M25P128)
    g_flash_sdbfs_baddr         : integer                        := 16#600000#; -- default for SVEC (M25P128)
    g_phys_uart                 : boolean                        := true;
    g_virtual_uart              : boolean                        := true;
    g_with_phys_uart_fifo       : boolean                        := false;
    g_phys_uart_tx_fifo_size    : integer                        := 1024;
    g_phys_uart_rx_fifo_size    : integer                        := 1024;
    g_aux_clks                  : integer                        := 0;
    g_rx_buffer_size            : integer                        := 1024;
    g_tx_runt_padding           : boolean                        := true;
    g_dpram_initf               : string                         := "default";
    g_dpram_size                : integer                        := 131072/4;  --in 32-bit words
    g_use_platform_specific_dpram        : boolean := FALSE;
    g_interface_mode            : t_wishbone_interface_mode      := PIPELINED;
    g_address_granularity       : t_wishbone_address_granularity := BYTE;
    g_aux_sdb                   : t_sdb_device                   := c_wrc_periph3_sdb;
    g_softpll_enable_debugger   : boolean                        := false;
    g_softpll_use_sampled_ref_clocks : boolean := false;
    g_softpll_reverse_dmtds : boolean := false;
    g_vuart_fifo_size           : integer                        := 1024;
    g_pcs_16bit                 : boolean                        := false;
    g_records_for_phy           : boolean                        := false;
    g_diag_id                   : integer                        := 0;
    g_diag_ver                  : integer                        := 0;
    g_diag_ro_size              : integer                        := 0;
    g_diag_rw_size              : integer                        := 0;
    g_dac_bits                  : integer                        := 16;
    g_softpll_aux_channel_config : t_softpll_channels_config_array := c_softpll_default_channels_config;
    g_with_clock_freq_monitor   : boolean                        := true;
    g_aux_timing_config         : t_wr_timecode_config           := c_WR_TIMECODE_NONE;
    g_direct_tag                : boolean                        := false;
    g_hwbld_date                : std_logic_vector(31 downto 0)  := (others => 'X')
    );
  port(
    ---------------------------------------------------------------------------
    -- Clocks/resets
    ---------------------------------------------------------------------------

    -- system reference clock (any frequency <= f(clk_ref_i))
    clk_sys_i : in std_logic;

    -- DDMTD offset clock (125.x MHz)
    clk_dmtd_i : in std_logic;
    clk_dmtd_over_i : in std_logic := '0';

    -- Timing reference (125 MHz)
    clk_ref_i : in std_logic;

    -- Aux clocks (i.e. the FMC clock), which can be disciplined by the WR Core
    clk_aux_i : in std_logic_vector(g_aux_clks-1 downto 0) := (others => '0');

    clk_ext_mul_i : in std_logic := '0';
    clk_ext_mul_locked_i  : in  std_logic := '1';
    clk_ext_stopped_i     : in  std_logic := '0';
    clk_ext_rst_o         : out std_logic;

    -- External 10 MHz reference (cesium, GPSDO, etc.), used in Grandmaster mode
    clk_ext_i : in std_logic := '0';

    -- LockSweep signals
    --lock_sweep_i         : in std_logic := '0';
    --lock_sweep_phase_i   : in std_logic_vector(15 downto 0) := (others => '0');

    -- External PPS input (cesium, GPSDO, etc.), used in Grandmaster mode
    pps_ext_i : in std_logic := '0';

    rst_n_i : in std_logic;

    -----------------------------------------
    -- Timing system
    -- Set helper pll and main pll DAC values
    -----------------------------------------
    dac_hpll_load_p1_o : out std_logic;
    dac_hpll_data_o    : out std_logic_vector(g_dac_bits-1 downto 0);

    dac_dpll_load_p1_o : out std_logic;
    dac_dpll_data_o    : out std_logic_vector(g_dac_bits-1 downto 0);

    direct_tag0_i       : in std_logic_vector(23 downto 0);
    direct_tag0_valid_i : in std_logic;

    -- PHY I/f
    phy_ref_clk_i : in std_logic;

    phy_tx_data_o      : out std_logic_vector(f_pcs_data_width(g_pcs_16bit)-1 downto 0);
    phy_tx_k_o         : out std_logic_vector(f_pcs_k_width(g_pcs_16bit)-1 downto 0);
    phy_tx_disparity_i : in  std_logic;
    phy_tx_enc_err_i   : in  std_logic;

    phy_rx_data_i     : in std_logic_vector(f_pcs_data_width(g_pcs_16bit)-1 downto 0);
    phy_rx_rbclk_i    : in std_logic;
    phy_rx_k_i        : in std_logic_vector(f_pcs_k_width(g_pcs_16bit)-1 downto 0);
    phy_rx_enc_err_i  : in std_logic;
    phy_rx_bitslide_i : in std_logic_vector(f_pcs_bts_width(g_pcs_16bit)-1 downto 0);

    phy_rst_o            : out std_logic;
    phy_rdy_i            : in  std_logic := '1';
    phy_loopen_o         : out std_logic;
    phy_loopen_vec_o     : out std_logic_vector(2 downto 0);
    phy_tx_prbs_sel_o    : out std_logic_vector(2 downto 0);
    phy_sfp_tx_fault_i   : in std_logic := '0';
    phy_sfp_los_i        : in std_logic := '0';
    phy_sfp_tx_disable_o : out std_logic;

    phy_rx_rbclk_sampled_i : in std_logic;

    -- clk_sys_i domain!
    phy_mdio_master_cyc_o : out std_logic;
    phy_mdio_master_stb_o : out std_logic;
    phy_mdio_master_we_o : out std_logic;
    phy_mdio_master_dat_o : out std_logic_vector(31 downto 0);
    phy_mdio_master_sel_o : out std_logic_vector(3 downto 0) := "0000";
    phy_mdio_master_adr_o : out std_logic_vector(31 downto 0);
    phy_mdio_master_ack_i : in std_logic := '0';
    phy_mdio_master_stall_i : in std_logic := '0';
    phy_mdio_master_dat_i : in std_logic_vector(31 downto 0) := x"00000000";


    -- PHY I/F record-based
    phy8_o  : out t_phy_8bits_from_wrc;
    phy8_i  : in  t_phy_8bits_to_wrc  := c_dummy_phy8_to_wrc;
    phy16_o : out t_phy_16bits_from_wrc;
    phy16_i : in  t_phy_16bits_to_wrc := c_dummy_phy16_to_wrc;

    -----------------------------------------
    --GPIO
    -----------------------------------------
    led_act_o  : out std_logic;
    led_link_o : out std_logic;
    scl_o      : out std_logic;
    scl_i      : in  std_logic := '1';
    sda_o      : out std_logic;
    sda_i      : in  std_logic := '1';
    sfp_scl_o  : out std_logic;
    sfp_scl_i  : in  std_logic := '1';
    sfp_sda_o  : out std_logic;
    sfp_sda_i  : in  std_logic := '1';
    sfp_det_i  : in  std_logic := '1';
    btn1_i     : in  std_logic := '1';
    btn2_i     : in  std_logic := '1';
    spi_sclk_o : out std_logic;
    spi_ncs_o  : out std_logic;
    spi_mosi_o : out std_logic;
    spi_miso_i : in  std_logic := '0';

    -----------------------------------------
    --UART
    -----------------------------------------
    uart_rxd_i : in  std_logic := '1';
    uart_txd_o : out std_logic;

    -----------------------------------------
    -- 1-wire
    -----------------------------------------
    owr_pwren_o : out std_logic_vector(1 downto 0);
    owr_en_o    : out std_logic_vector(1 downto 0);
    owr_i       : in  std_logic_vector(1 downto 0) := (others => '1');

    -----------------------------------------
    --External WB interface
    -----------------------------------------
    wb_adr_i   : in  std_logic_vector(c_wishbone_address_width-1 downto 0)   := (others => '0');
    wb_dat_i   : in  std_logic_vector(c_wishbone_data_width-1 downto 0)      := (others => '0');
    wb_dat_o   : out std_logic_vector(c_wishbone_data_width-1 downto 0);
    wb_sel_i   : in  std_logic_vector(c_wishbone_address_width/8-1 downto 0) := (others => '0');
    wb_we_i    : in  std_logic                                               := '0';
    wb_cyc_i   : in  std_logic                                               := '0';
    wb_stb_i   : in  std_logic                                               := '0';
    wb_ack_o   : out std_logic;
    wb_err_o   : out std_logic;
    wb_rty_o   : out std_logic;
    wb_stall_o : out std_logic;

    -----------------------------------------
    -- Auxillary WB master
    -----------------------------------------
    aux_adr_o   : out std_logic_vector(c_wishbone_address_width-1 downto 0);
    aux_dat_o   : out std_logic_vector(c_wishbone_data_width-1 downto 0);
    aux_dat_i   : in  std_logic_vector(c_wishbone_data_width-1 downto 0);
    aux_sel_o   : out std_logic_vector(c_wishbone_address_width/8-1 downto 0);
    aux_we_o    : out std_logic;
    aux_cyc_o   : out std_logic;
    aux_stb_o   : out std_logic;
    aux_ack_i   : in  std_logic := '1';
    aux_stall_i : in  std_logic := '0';

    -----------------------------------------
    -- External Fabric I/F
    -----------------------------------------
    ext_snk_adr_i   : in  std_logic_vector(1 downto 0)  := "00";
    ext_snk_dat_i   : in  std_logic_vector(15 downto 0) := x"0000";
    ext_snk_sel_i   : in  std_logic_vector(1 downto 0)  := "00";
    ext_snk_cyc_i   : in  std_logic                     := '0';
    ext_snk_we_i    : in  std_logic                     := '0';
    ext_snk_stb_i   : in  std_logic                     := '0';
    ext_snk_ack_o   : out std_logic;
    ext_snk_err_o   : out std_logic;
    ext_snk_stall_o : out std_logic;

    ext_src_adr_o   : out std_logic_vector(1 downto 0);
    ext_src_dat_o   : out std_logic_vector(15 downto 0);
    ext_src_sel_o   : out std_logic_vector(1 downto 0);
    ext_src_cyc_o   : out std_logic;
    ext_src_stb_o   : out std_logic;
    ext_src_we_o    : out std_logic;
    ext_src_ack_i   : in  std_logic := '1';
    ext_src_err_i   : in  std_logic := '0';
    ext_src_stall_i : in  std_logic := '0';

    ------------------------------------------
    -- External TX Timestamp I/F
    ------------------------------------------
    txtsu_port_id_o      : out std_logic_vector(4 downto 0);
    txtsu_frame_id_o     : out std_logic_vector(15 downto 0);
    txtsu_ts_value_o     : out std_logic_vector(31 downto 0);
    txtsu_ts_incorrect_o : out std_logic;
    txtsu_stb_o          : out std_logic;
    txtsu_ack_i          : in  std_logic := '1';

    -----------------------------------------
    -- Timestamp helper signals, used for Absolute Calibration
    -----------------------------------------
    abscal_txts_o        : out std_logic;
    abscal_rxts_o        : out std_logic;

    -----------------------------------------
    -- Pause Frame Control
    -----------------------------------------
    fc_tx_pause_req_i   : in  std_logic                     := '0';
    fc_tx_pause_delay_i : in  std_logic_vector(15 downto 0) := x"0000";
    fc_tx_pause_ready_o : out std_logic;

    -----------------------------------------
    -- Timecode/Servo Control
    -----------------------------------------

    tm_link_up_o         : out std_logic;
    -- DAC Control
    tm_dac_value_o       : out std_logic_vector(31 downto 0);
    tm_dac_wr_o          : out std_logic_vector(g_aux_clks-1 downto 0);
    -- Aux clock lock enable
    tm_clk_aux_lock_en_i : in  std_logic_vector(g_aux_clks-1 downto 0) := (others => '0');
    -- Aux clock locked flag
    tm_clk_aux_locked_o  : out std_logic_vector(g_aux_clks-1 downto 0);
    -- Timecode output
    tm_time_valid_o      : out std_logic;
    tm_tai_o             : out std_logic_vector(39 downto 0);
    tm_cycles_o          : out std_logic_vector(27 downto 0);
    -- 1PPS output
    pps_csync_o          : out std_logic;
    pps_valid_o          : out std_logic;
    pps_p_o              : out std_logic;
    pps_led_o            : out std_logic;

    aux_timing_serdes_locked_i  : in std_logic := '0';  --pll locked indicator from pll for platform specific serdes.  can be left unconnected if aux timing is not used

    --Aux Timing outputs
    utc_year_o           : out std_logic_vector(11 downto 0);
    utc_diy_o            : out std_logic_vector(8 downto 0);
    utc_month_o          : out std_logic_vector(3 downto 0);
    utc_day_o            : out std_logic_vector(4 downto 0);
    utc_hour_o           : out std_logic_vector(5 downto 0);
    utc_min_o            : out std_logic_vector(5 downto 0);
    utc_sec_o            : out std_logic_vector(5 downto 0);
    utc_sbs_o            : out std_logic_vector(16 downto 0);
    utc_valid_o          : out std_logic;
    ls_val_o             : out std_logic_vector(7 downto 0);
    ls_flag_o            : out std_logic_vector(1 downto 0);
    ls_valid_o           : out std_logic;
    irig_o               : out std_logic;
    irig_valid_o         : out std_logic;
    nmea_o               : out std_logic;
    nmea_valid_o         : out std_logic;
    serdes_dat_o         : out std_logic_vector(7 downto 0);

    rst_aux_n_o : out std_logic;

    link_ok_o : out std_logic;

    -------------------------------------
    -- DIAG to/from external modules
    -------------------------------------
    aux_diag_i : in  t_generic_word_array(g_diag_ro_size-1 downto 0) := (others=>(others=>'0'));
    aux_diag_o : out t_generic_word_array(g_diag_rw_size-1 downto 0)
    );
end wr_core;

architecture struct of wr_core is
  signal ext_snk_rty, txtsu_port_id_5, aux_timing_serdes_out : std_logic;
begin

  WRPC : entity work.xwr_core
    generic map(
      g_simulation                => g_simulation,
      g_verbose                   => g_verbose,
      g_ram_address_space_size_kb => g_ram_address_space_size_kb,
      g_board_name                => g_board_name,
      g_flash_secsz_kb            => g_flash_secsz_kb,
      g_flash_sdbfs_baddr         => g_flash_sdbfs_baddr,
      g_phys_uart                 => g_phys_uart,
      g_with_phys_uart_fifo       => g_with_phys_uart_fifo,
      g_phys_uart_tx_fifo_size    => g_phys_uart_tx_fifo_size,
      g_phys_uart_rx_fifo_size    => g_phys_uart_rx_fifo_size,
      g_virtual_uart              => g_virtual_uart,
      g_ep_rxbuf_size             => g_rx_buffer_size,
      g_tx_runt_padding           => g_tx_runt_padding,
      g_with_external_clock_input => g_with_external_clock_input,
      g_aux_clks                  => g_aux_clks,
      g_dpram_initf               => g_dpram_initf,
      g_dpram_size                => g_dpram_size,
      g_interface_mode            => g_interface_mode,
      g_address_granularity       => g_address_granularity,
      g_aux_sdb                   => g_aux_sdb,
      g_softpll_enable_debugger   => g_softpll_enable_debugger,
      g_softpll_use_sampled_ref_clocks => g_softpll_use_sampled_ref_clocks,
      g_softpll_reverse_dmtds => g_softpll_reverse_dmtds,
      g_vuart_fifo_size           => g_vuart_fifo_size,
      g_pcs_16bit                 => g_pcs_16bit,
      g_records_for_phy           => g_records_for_phy,
      g_diag_id                   => g_diag_id,
      g_diag_ver                  => g_diag_ver,
      g_diag_ro_size              => g_diag_ro_size,
      g_diag_rw_size              => g_diag_rw_size,
      g_dac_bits                  => g_dac_bits,
      g_use_platform_specific_dpram => g_use_platform_specific_dpram,
      g_softpll_aux_channel_config => g_softpll_aux_channel_config,
      g_with_clock_freq_monitor   => g_with_clock_freq_monitor,
      g_aux_timing_config         => g_aux_timing_config,
      g_direct_tag                => g_direct_tag,
      g_hwbld_date                => g_hwbld_date
      )
    port map(
      clk_sys_i     => clk_sys_i,
      clk_dmtd_i    => clk_dmtd_i,
      clk_dmtd_over_i => clk_dmtd_over_i,
      clk_ref_i     => clk_ref_i,
      clk_aux_i     => clk_aux_i,
      clk_ext_i     => clk_ext_i,
      clk_ext_mul_i => clk_ext_mul_i,
      clk_ext_mul_locked_i  => clk_ext_mul_locked_i,
      clk_ext_stopped_i => clk_ext_stopped_i,
      clk_ext_rst_o     => clk_ext_rst_o,
      pps_ext_i     => pps_ext_i,
      rst_n_i       => rst_n_i,

      direct_tag0_valid_i  => direct_tag0_valid_i,
      direct_tag0_i        => direct_tag0_i,

      dac_hpll_load_p1_o   => dac_hpll_load_p1_o,
      dac_hpll_data_o      => dac_hpll_data_o,
      dac_dpll_load_p1_o   => dac_dpll_load_p1_o,
      dac_dpll_data_o      => dac_dpll_data_o,

      phy_ref_clk_i        => phy_ref_clk_i,
      phy_tx_data_o        => phy_tx_data_o,
      phy_tx_k_o           => phy_tx_k_o,
      phy_tx_disparity_i   => phy_tx_disparity_i,
      phy_tx_enc_err_i     => phy_tx_enc_err_i,
      phy_rx_data_i        => phy_rx_data_i,
      phy_rx_rbclk_i       => phy_rx_rbclk_i,
      phy_rx_rbclk_sampled_i => phy_rx_rbclk_sampled_i,
      phy_rx_k_i           => phy_rx_k_i,
      phy_rx_enc_err_i     => phy_rx_enc_err_i,
      phy_rx_bitslide_i    => phy_rx_bitslide_i,
      phy_rst_o            => phy_rst_o,
      phy_rdy_i            => phy_rdy_i,
      phy_loopen_o         => phy_loopen_o,
      phy_loopen_vec_o     => phy_loopen_vec_o,
      phy_tx_prbs_sel_o    => phy_tx_prbs_sel_o,
      phy_sfp_tx_fault_i   => phy_sfp_tx_fault_i,
      phy_sfp_los_i        => phy_sfp_los_i,
      phy_sfp_tx_disable_o => phy_sfp_tx_disable_o,

      phy_mdio_master_o.cyc    => phy_mdio_master_cyc_o,
      phy_mdio_master_o.stb    => phy_mdio_master_stb_o,
      phy_mdio_master_o.we     => phy_mdio_master_we_o,
      phy_mdio_master_o.sel    => phy_mdio_master_sel_o,
      phy_mdio_master_o.adr    => phy_mdio_master_adr_o,
      phy_mdio_master_o.dat    => phy_mdio_master_dat_o,
      phy_mdio_master_i.dat    => phy_mdio_master_dat_i,
      phy_mdio_master_i.stall  => phy_mdio_master_stall_i,
      phy_mdio_master_i.ack    => phy_mdio_master_ack_i,
      phy_mdio_master_i.err    => '0',
      phy_mdio_master_i.rty    => '0',

      phy8_o     => phy8_o,
      phy8_i     => phy8_i,
      phy16_o    => phy16_o,
      phy16_i    => phy16_i,

      led_act_o  => led_act_o,
      led_link_o => led_link_o,
      scl_o      => scl_o,
      scl_i      => scl_i,
      sda_o      => sda_o,
      sda_i      => sda_i,
      sfp_scl_o  => sfp_scl_o,
      sfp_scl_i  => sfp_scl_i,
      sfp_sda_o  => sfp_sda_o,
      sfp_sda_i  => sfp_sda_i,
      sfp_det_i  => sfp_det_i,
      btn1_i     => btn1_i,
      btn2_i     => btn2_i,
      spi_sclk_o => spi_sclk_o,
      spi_ncs_o  => spi_ncs_o,
      spi_mosi_o => spi_mosi_o,
      spi_miso_i => spi_miso_i,
      uart_rxd_i => uart_rxd_i,
      uart_txd_o => uart_txd_o,

      owr_pwren_o => owr_pwren_o,
      owr_en_o    => owr_en_o,
      owr_i       => owr_i,

      slave_i.adr   => wb_adr_i,
      slave_i.dat   => wb_dat_i,
      slave_i.sel   => wb_sel_i,
      slave_i.we    => wb_we_i,
      slave_i.cyc   => wb_cyc_i,
      slave_i.stb   => wb_stb_i,
      slave_o.dat   => wb_dat_o,
      slave_o.ack   => wb_ack_o,
      slave_o.err   => wb_err_o,
      slave_o.rty   => wb_rty_o,
      slave_o.stall => wb_stall_o,

      aux_master_o.adr   => aux_adr_o,
      aux_master_o.dat   => aux_dat_o,
      aux_master_o.sel   => aux_sel_o,
      aux_master_o.cyc   => aux_cyc_o,
      aux_master_o.stb   => aux_stb_o,
      aux_master_o.we    => aux_we_o,
      aux_master_i.stall => aux_stall_i,
      aux_master_i.ack   => aux_ack_i,
      aux_master_i.dat   => aux_dat_i,
      aux_master_i.err   => '0',
      aux_master_i.rty   => '0',

      wrf_snk_i.adr   => ext_snk_adr_i,
      wrf_snk_i.dat   => ext_snk_dat_i,
      wrf_snk_i.sel   => ext_snk_sel_i,
      wrf_snk_i.cyc   => ext_snk_cyc_i,
      wrf_snk_i.we    => ext_snk_we_i,
      wrf_snk_i.stb   => ext_snk_stb_i,
      wrf_snk_o.ack   => ext_snk_ack_o,
      wrf_snk_o.err   => ext_snk_err_o,
      wrf_snk_o.stall => ext_snk_stall_o,
      wrf_snk_o.rty   => ext_snk_rty,

      wrf_src_o.adr   => ext_src_adr_o,
      wrf_src_o.dat   => ext_src_dat_o,
      wrf_src_o.sel   => ext_src_sel_o,
      wrf_src_o.cyc   => ext_src_cyc_o,
      wrf_src_o.stb   => ext_src_stb_o,
      wrf_src_o.we    => ext_src_we_o,
      wrf_src_i.ack   => ext_src_ack_i,
      wrf_src_i.err   => ext_src_err_i,
      wrf_src_i.stall => ext_src_stall_i,
      wrf_src_i.rty   => '0',

      timestamps_o.port_id(4 downto 0) => txtsu_port_id_o,
      timestamps_o.port_id(5) => txtsu_port_id_5,
      timestamps_o.frame_id   => txtsu_frame_id_o,
      timestamps_o.tsval      => txtsu_ts_value_o,
      timestamps_o.incorrect  => txtsu_ts_incorrect_o,
      timestamps_o.stb        => txtsu_stb_o,
      timestamps_ack_i        => txtsu_ack_i,

      abscal_txts_o        => abscal_txts_o,
      abscal_rxts_o        => abscal_rxts_o,

      fc_tx_pause_req_i    => fc_tx_pause_req_i,
      fc_tx_pause_delay_i  => fc_tx_pause_delay_i,
      fc_tx_pause_ready_o  => fc_tx_pause_ready_o,

      tm_link_up_o         => tm_link_up_o,
      tm_dac_value_o       => tm_dac_value_o,
      tm_dac_wr_o          => tm_dac_wr_o,
      tm_clk_aux_lock_en_i => tm_clk_aux_lock_en_i,
      tm_clk_aux_locked_o  => tm_clk_aux_locked_o,
      tm_time_valid_o      => tm_time_valid_o,
      tm_tai_o             => tm_tai_o,
      tm_cycles_o          => tm_cycles_o,
      pps_csync_o          => pps_csync_o,
      pps_valid_o          => pps_valid_o,
      pps_p_o              => pps_p_o,
      pps_led_o            => pps_led_o,

      --lock_sweep_i       => lock_sweep_i,
      --lock_sweep_phase_i => lock_sweep_phase_i,

      aux_timing_serdes_locked_i  => aux_timing_serdes_locked_i,

      utc_o.utc_year           => utc_year_o,
      utc_o.utc_diy            => utc_diy_o,
      utc_o.utc_month          => utc_month_o,
      utc_o.utc_day            => utc_day_o,
      utc_o.utc_hour           => utc_hour_o,
      utc_o.utc_min            => utc_min_o,
      utc_o.utc_sec            => utc_sec_o,
      utc_o.utc_sbs            => utc_sbs_o,
      utc_o.utc_valid          => utc_valid_o,
      utc_o.ls_val             => ls_val_o,
      utc_o.ls_flag            => ls_flag_o,
      utc_o.ls_valid           => ls_valid_o,
      aux_timing_o.irig        => irig_o,
      aux_timing_o.irig_valid  => irig_valid_o,
      aux_timing_o.nmea        => nmea_o,
      aux_timing_o.nmea_valid  => nmea_valid_o,
      aux_timing_o.serdes_in   => serdes_dat_o,
      aux_timing_o.serdes_out  => aux_timing_serdes_out,

      rst_aux_n_o => rst_aux_n_o,

      link_ok_o => link_ok_o,

      aux_diag_i => aux_diag_i,
      aux_diag_o => aux_diag_o
      );
end struct;
