-------------------------------------------------------------------------------
-- Title      : WRPC Wrapper for clbv3
-- Project    : WR PTP Core
-- URL        : http://www.ohwr.org/projects/wr-cores/wiki/Wrpc_core
-------------------------------------------------------------------------------
-- File       : xwrc_board_acorn.vhd
-- Author(s)  : Peter Jansweijer <peterj@nikhef.nl>
-- Company    : Nikhef
-- Created    : 2017-11-08
-- Last update: 2022-05-20
-- Standard   : VHDL'93
-------------------------------------------------------------------------------
-- Description: Top-level wrapper for WR PTP core including all the modules
-- needed to operate the core on the clbv3 board.
-------------------------------------------------------------------------------
-- Copyright (c) 2017 Nikhef
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

library work;
use work.gencores_pkg.all;
use work.wrcore_pkg.all;
use work.wishbone_pkg.all;
use work.wr_fabric_pkg.all;
use work.endpoint_pkg.all;
use work.streamers_pkg.all;
use work.wr_xilinx_pkg.all;
use work.wr_board_pkg.all;
use work.wr_clbv3_pkg.all;

library unisim;
use unisim.vcomponents.all;

entity xwrc_board_acorn is
  generic(
    -- Select whether to include external ref clock input
    g_with_external_clock_input : boolean              := TRUE;
    -- Number of aux clocks syntonized by WRPC to WR timebase
    g_aux_clks                  : integer              := 0;
    -- plain     = expose WRC fabric interface
    -- streamers = attach WRC streamers to fabric interface
    -- etherbone = attach Etherbone slave to fabric interface
    g_fabric_iface              : t_board_fabric_iface := plain;
    -- parameters configuration when g_fabric_iface = "streamers" (otherwise ignored)
    g_streamers_op_mode        : t_streamers_op_mode  := TX_AND_RX;
    g_tx_streamer_params       : t_tx_streamer_params := c_tx_streamer_params_defaut;
    g_rx_streamer_params       : t_rx_streamer_params := c_rx_streamer_params_defaut;
    -- memory initialisation file for embedded CPU
    g_dpram_initf               : string               := "default_xilinx";
    -- identification (id and ver) of the layout of words in the generic diag interface
    g_diag_id                   : integer              := 0;
    g_diag_ver                  : integer              := 0;
    -- size the generic diag interface
    g_diag_ro_size              : integer              := 0;
    g_diag_rw_size              : integer              := 0;
    -- GTPE2_CHANNEL TX Polarity Control Ports
    txpolarity                  : bit                  := '0';
    -- GTPE2_CHANNEL RX Polarity Control Ports
    rxpolarity                  : bit                  := '1'
    );
  port (
    ---------------------------------------------------------------------------
    -- Clocks/resets
    ---------------------------------------------------------------------------
    -- Reset input (active low, can be async)
    areset_n_i          : in  std_logic;
    -- Optional reset input active low with rising edge detection. Does not
    -- reset PLLs.
    areset_edge_n_i     : in  std_logic := '1';
    -- Clock inputs from the board
    clk_125m_dmtd_i     : in  std_logic;
    clk_125m_gtp_i      : in  std_logic;
    -- 10MHz ext ref clock input (g_with_external_clock_input = TRUE)
    clk_10m_ext_i       : in  std_logic                               := '0';
    -- External PPS input (g_with_external_clock_input = TRUE)
    pps_ext_i           : in  std_logic                               := '0';

    -- debug/test ggm 20240402
    clk_ref_62m5_o      : out std_logic;
    clk_ref_locked_o    : out std_logic;
    dbg_rdy_o           : out std_logic;
    ext_ref_rst_o       : out std_logic;
    ready_for_reset_o   : out std_logic;

    ---------------------------------------------------------------------------
    -- Shared SPI interface to DACs
    ---------------------------------------------------------------------------
    dac_refclk_cs_n_o : out std_logic;
    dac_refclk_sclk_o : out std_logic;
    dac_refclk_din_o  : out std_logic;

    dac_dmtd_cs_n_o   : out std_logic;
    dac_dmtd_sclk_o   : out std_logic;
    dac_dmtd_din_o    : out std_logic;

    ---------------------------------------------------------------------------
    -- SFP I/O for transceiver and SFP management info
    ---------------------------------------------------------------------------
    sfp_txp_o         : out std_logic;
    sfp_txn_o         : out std_logic;
    sfp_rxp_i         : in  std_logic;
    sfp_rxn_i         : in  std_logic;
    sfp_det_i         : in  std_logic := '1';
    sfp_sda           : inout  std_logic;
    sfp_scl           : inout  std_logic;
    sfp_tx_fault_i    : in std_logic;
    sfp_tx_los_i      : in std_logic;

    ---------------------------------------------------------------------------
    -- I2C EEPROM
    ---------------------------------------------------------------------------
    --eeprom_sda_i : in  std_logic;
    --eeprom_sda_o : out std_logic;
    --eeprom_scl_i : in  std_logic;
    --eeprom_scl_o : out std_logic;

    ---------------------------------------------------------------------------
    -- Onewire interface
    ---------------------------------------------------------------------------
    onewire_i     : in  std_logic;
    onewire_oen_o : out std_logic;

    ---------------------------------------------------------------------------
    -- UART
    ---------------------------------------------------------------------------
    uart_rxd_i : in  std_logic;
    uart_txd_o : out std_logic;

    ---------------------------------------------------------------------------
    -- Flash memory SPI interface
    ---------------------------------------------------------------------------
    spi_sclk_o : out std_logic;
    spi_ncs_o  : out std_logic;
    spi_mosi_o : out std_logic;
    spi_miso_i : in  std_logic := '0';

    ---------------------------------------------------------------------------
    -- No External WB interface
    ---------------------------------------------------------------------------

    ---------------------------------------------------------------------------
    -- WR fabric interface (when g_fabric_iface = "plainfbrc")
    ---------------------------------------------------------------------------
    wrf_src_o : out t_wrf_source_out;
    wrf_src_i : in  t_wrf_source_in := c_dummy_src_in;
    wrf_snk_o : out t_wrf_sink_out;
    wrf_snk_i : in  t_wrf_sink_in   := c_dummy_snk_in;

    ---------------------------------------------------------------------------
    -- WR streamers (when g_fabric_iface = "streamers")
    ---------------------------------------------------------------------------
    wrs_tx_data_i  : in  std_logic_vector(g_tx_streamer_params.data_width-1 downto 0) := (others => '0');
    wrs_tx_valid_i : in  std_logic                                        := '0';
    wrs_tx_dreq_o  : out std_logic;
    wrs_tx_last_i  : in  std_logic                                        := '1';
    wrs_tx_flush_i : in  std_logic                                        := '0';
    wrs_tx_cfg_i   : in  t_tx_streamer_cfg                                := c_tx_streamer_cfg_default;
    wrs_rx_first_o : out std_logic;
    wrs_rx_last_o  : out std_logic;
    wrs_rx_data_o  : out std_logic_vector(g_rx_streamer_params.data_width-1 downto 0);
    wrs_rx_valid_o : out std_logic;
    wrs_rx_dreq_i  : in  std_logic                                        := '0';
    wrs_rx_cfg_i   : in t_rx_streamer_cfg                                 := c_rx_streamer_cfg_default;
    ---------------------------------------------------------------------------
    -- No Etherbone WB master interface (when g_fabric_iface = "etherbone")
    ---------------------------------------------------------------------------

    ---------------------------------------------------------------------------
    -- Generic diagnostics interface (access from WRPC via SNMP or uart console
    ---------------------------------------------------------------------------
    aux_diag_i : in  t_generic_word_array(g_diag_ro_size-1 downto 0) := (others => (others => '0'));
    aux_diag_o : out t_generic_word_array(g_diag_rw_size-1 downto 0);

    ---------------------------------------------------------------------------
    -- Aux clocks control
    ---------------------------------------------------------------------------
    tm_dac_value_o       : out std_logic_vector(31 downto 0);
    tm_dac_wr_o          : out std_logic_vector(g_aux_clks-1 downto 0);
    tm_clk_aux_lock_en_i : in  std_logic_vector(g_aux_clks-1 downto 0) := (others => '0');
    tm_clk_aux_locked_o  : out std_logic_vector(g_aux_clks-1 downto 0);

    ---------------------------------------------------------------------------
    -- External Tx Timestamping I/F
    ---------------------------------------------------------------------------
    timestamps_o     : out t_txtsu_timestamp;
    timestamps_ack_i : in  std_logic := '1';

    -----------------------------------------
    -- Timestamp helper signals, used for Absolute Calibration
    -----------------------------------------
    --abscal_txts_o       : out std_logic;
    --abscal_rxts_o       : out std_logic;

    ---------------------------------------------------------------------------
    -- Pause Frame Control
    ---------------------------------------------------------------------------
    fc_tx_pause_req_i   : in  std_logic                     := '0';
    fc_tx_pause_delay_i : in  std_logic_vector(15 downto 0) := x"0000";
    --fc_tx_pause_ready_o : out std_logic;

    ---------------------------------------------------------------------------
    -- Timecode I/F
    ---------------------------------------------------------------------------
    --tm_link_up_o    : out std_logic;
    --tm_time_valid_o : out std_logic;
    --tm_tai_o        : out std_logic_vector(39 downto 0);
    --tm_cycles_o     : out std_logic_vector(27 downto 0);

    ---------------------------------------------------------------------------
    -- Buttons, LEDs and PPS output
    ---------------------------------------------------------------------------
    led_act_o  : out std_logic;
    led_link_o : out std_logic;
    btn1_i     : in  std_logic := '1';
    btn2_i     : in  std_logic := '1';
    -- 1PPS output
    pps_p_o    : out std_logic;
    pps_led_o  : out std_logic;
    -- Link ok indication
    link_ok_o  : out std_logic;

    debug : out std_logic_vector(31 downto 0);

    qpll_reset  : out std_logic;
    qpll_clk    : in  std_logic;
    qpll_refclk : in  std_logic;
    qpll_lock   : in  std_logic
    );

end entity xwrc_board_acorn;

architecture struct of xwrc_board_acorn is

  -----------------------------------------------------------------------------
  -- Signals
  -----------------------------------------------------------------------------

  -- IBUFDS
  --signal clk_125m_dmtd_i : std_logic;
  signal clk_dmtd     : std_logic;

  -- PLLs, clocks
  signal clk_pll_62m5 : std_logic;
  signal clk_ref_62m5 : std_logic;
  signal pll_locked   : std_logic;
  signal clk_10m_ext  : std_logic;

  -- Reset logic
  signal areset_edge_ppulse : std_logic;
  signal rst_62m5_n         : std_logic;
  signal rstlogic_arst_n    : std_logic;
  signal rstlogic_clk_in    : std_logic_vector(1 downto 0);
  signal rstlogic_rst_out   : std_logic_vector(1 downto 0);

  -- PLL DACs
  signal dac_dmtd_load   : std_logic;
  signal dac_dmtd_data   : std_logic_vector(15 downto 0);
  signal dac_refclk_load : std_logic;
  signal dac_refclk_data : std_logic_vector(15 downto 0);

  -- OneWire
  signal onewire_in : std_logic_vector(1 downto 0);
  signal onewire_en : std_logic_vector(1 downto 0);

  -- PHY
  signal phy16_to_wrc   : t_phy_16bits_to_wrc;
  signal phy16_from_wrc : t_phy_16bits_from_wrc;

  -- External reference
  signal ext_ref_mul         : std_logic;
  signal ext_ref_mul_locked  : std_logic;
  signal ext_ref_mul_stopped : std_logic;
  signal ext_ref_rst         : std_logic;

  signal sfp_sda_out         : std_logic;
  signal sfp_sda_in          : std_logic;
  signal sfp_scl_out         : std_logic;
  signal sfp_scl_in          : std_logic;

begin  -- architecture struct

  sfp_scl <= '0' when sfp_scl_out = '0' else 'Z';
  sfp_sda <= '0' when sfp_sda_out = '0' else 'Z';
  sfp_scl_in     <= sfp_scl;
  sfp_sda_in     <= sfp_sda;


  -----------------------------------------------------------------------------
  -- Platform-dependent part (PHY, PLLs, buffers, etc)
  -----------------------------------------------------------------------------

  cmp_xwrc_platform : entity work.xwrc_platform_xilinx
    generic map (
      g_fpga_family               => "artix7",
      g_direct_dmtd               => TRUE,
      g_with_external_clock_input => g_with_external_clock_input,
      g_use_default_plls          => TRUE,
      g_simulation                => 0,
      g_input_clk_single          => TRUE,
      g_gtp_enable_pll0           => '1',
      g_gtp_enable_pll1           => '0',
      txpolarity                  => txpolarity,
      rxpolarity                  => rxpolarity)
    port map (
      -- test/debug ggm 20240402
      clk_ref_locked_o      => clk_ref_locked_o,
      dbg_rdy_o             => dbg_rdy_o,
      ready_for_reset_o     => ready_for_reset_o,

      areset_n_i            => areset_n_i,
      clk_10m_ext_i         => clk_10m_ext_i,
      clk_125m_dmtd_i       => clk_125m_dmtd_i,
      clk_125m_gtp_p_i      => clk_125m_gtp_i,
      clk_125m_gtp_n_i      => '0', --clk_125m_gtp_n_i,
      sfp_txn_o             => sfp_txn_o,
      sfp_txp_o             => sfp_txp_o,
      sfp_rxn_i             => sfp_rxn_i,
      sfp_rxp_i             => sfp_rxp_i,
      sfp_tx_fault_i        => sfp_tx_fault_i,
      sfp_los_i             => sfp_tx_los_i,
      sfp_tx_disable_o      => Open,
      clk_62m5_sys_o        => clk_pll_62m5,
      clk_125m_ref_o        => clk_ref_62m5,  -- Note: This is a 62m5 Clock for 16 bit PHYs!
      clk_62m5_dmtd_o       => clk_dmtd,
      pll_locked_o          => pll_locked,
      clk_10m_ext_o         => clk_10m_ext,
      phy16_o               => phy16_to_wrc,
      phy16_i               => phy16_from_wrc,
      ext_ref_mul_o         => ext_ref_mul,
      ext_ref_mul_locked_o  => ext_ref_mul_locked,
      ext_ref_mul_stopped_o => ext_ref_mul_stopped,
      ext_ref_rst_i         => ext_ref_rst,
      debug                 => debug,
      qpll_reset            => qpll_reset,
      qpll_clk              => qpll_clk,
      qpll_refclk           => qpll_refclk,
      qpll_lock             => qpll_lock
    );

  clk_ref_62m5_o <= clk_ref_62m5;

  -- test/debug
  ext_ref_rst_o <= ext_ref_rst;

  -----------------------------------------------------------------------------
  -- Reset logic
  -----------------------------------------------------------------------------
  -- Detect when areset_edge_n_i goes high (end of reset) and use this edge to
  -- generate rstlogic_arst_n. This is needed to connect optional reset like PCIe
  -- reset. When baord runs standalone, we need to ignore PCIe reset being
  -- constantly low.
  cmp_arst_edge: gc_sync_ffs
    generic map (
      g_sync_edge => "positive")
    port map (
      clk_i    => clk_pll_62m5,
      rst_n_i  => '1',
      data_i   => areset_edge_n_i,
      ppulse_o => areset_edge_ppulse);

  -- logic AND of all async reset sources (active low)
  rstlogic_arst_n <= pll_locked and areset_n_i and (not areset_edge_ppulse);

  -- concatenation of all clocks required to have synced resets
  rstlogic_clk_in(0) <= clk_pll_62m5;
  rstlogic_clk_in(1) <= clk_ref_62m5;

  cmp_rstlogic_reset : gc_reset
    generic map (
      g_clocks    => 2,                           -- 62.5MHz, 125MHz
      g_logdelay  => 4,                           -- 16 clock cycles
      g_syncdepth => 3)                           -- length of sync chains
    port map (
      free_clk_i => clk_125m_dmtd_i,
      locked_i   => rstlogic_arst_n,
      clks_i     => rstlogic_clk_in,
      rstn_o     => rstlogic_rst_out);

  -- distribution of resets (already synchronized to their clock domains)
  rst_62m5_n <= rstlogic_rst_out(0);

  -----------------------------------------------------------------------------
  -- 2x SPI DAC
  -----------------------------------------------------------------------------

  cmp_dmtd_dac : gc_serial_dac
    generic map (
      g_num_data_bits => 16,
      g_num_extra_bits => 8,
      g_num_cs_select => 1,
      g_sclk_polarity => 0)
    port map (
      clk_i         => clk_pll_62m5,
      rst_n_i       => rst_62m5_n,
      value_i       => dac_dmtd_data,
      cs_sel_i      => "1",
      load_i        => dac_dmtd_load,
      sclk_divsel_i => "001",
      dac_cs_n_o(0) => dac_dmtd_cs_n_o,
      dac_sclk_o    => dac_dmtd_sclk_o,
      dac_sdata_o   => dac_dmtd_din_o);

  cmp_refclk_dac : gc_serial_dac
    generic map (
      g_num_data_bits => 16,
      g_num_extra_bits => 8,
      g_num_cs_select => 1,
      g_sclk_polarity => 0)
    port map (
      clk_i         => clk_pll_62m5,
      rst_n_i       => rst_62m5_n,
      value_i       => dac_refclk_data,
      cs_sel_i      => "1",
      load_i        => dac_refclk_load,
      sclk_divsel_i => "001",
      dac_cs_n_o(0) => dac_refclk_cs_n_o,
      dac_sclk_o    => dac_refclk_sclk_o,
      dac_sdata_o   => dac_refclk_din_o);

  -----------------------------------------------------------------------------
  -- The WR PTP core with optional fabric interface attached
  -----------------------------------------------------------------------------

  cmp_board_common : xwrc_board_common
    generic map (
      g_simulation                => 0,
      g_with_external_clock_input => g_with_external_clock_input,
      g_board_name                => "CLB3",
      g_phys_uart                 => TRUE,
      g_virtual_uart              => TRUE,
      g_aux_clks                  => g_aux_clks,
      g_ep_rxbuf_size             => 1024,
      g_tx_runt_padding           => TRUE,
      g_dpram_initf               => g_dpram_initf,
      g_dpram_size                => 147456/4, -- 131072/4,
      g_interface_mode            => PIPELINED,
      g_address_granularity       => BYTE,
      g_aux_sdb                   => c_wrc_periph3_sdb,
      g_softpll_enable_debugger   => TRUE,
      g_vuart_fifo_size           => 1024,
      g_pcs_16bit                 => TRUE,
      g_diag_id                   => g_diag_id,
      g_diag_ver                  => g_diag_ver,
      g_diag_ro_size              => g_diag_ro_size,
      g_diag_rw_size              => g_diag_rw_size,
      g_streamers_op_mode         => g_streamers_op_mode,
      g_tx_streamer_params        => g_tx_streamer_params,
      g_rx_streamer_params        => g_rx_streamer_params,
      g_fabric_iface              => g_fabric_iface
      )
    port map (
      clk_sys_i            => clk_pll_62m5,
      clk_dmtd_i           => clk_dmtd,
      clk_ref_i            => clk_ref_62m5,
      clk_aux_i            => (0 downto 0 => '0'),
      clk_10m_ext_i        => clk_10m_ext,
      clk_ext_mul_i        => ext_ref_mul,
      clk_ext_mul_locked_i => ext_ref_mul_locked,
      clk_ext_stopped_i    => ext_ref_mul_stopped,
      clk_ext_rst_o        => ext_ref_rst,
      pps_ext_i            => pps_ext_i,
      rst_n_i              => rst_62m5_n,
      dac_hpll_load_p1_o   => dac_dmtd_load,
      dac_hpll_data_o      => dac_dmtd_data,
      dac_dpll_load_p1_o   => dac_refclk_load,
      dac_dpll_data_o      => dac_refclk_data,
      phy16_o              => phy16_from_wrc,
      phy16_i              => phy16_to_wrc,
      scl_o                => Open, --eeprom_scl_o,
      scl_i                => '1', --eeprom_scl_i,
      sda_o                => Open, --eeprom_sda_o,
      sda_i                => '1', --eeprom_sda_i,
      sfp_scl_o            => sfp_scl_out,
      sfp_scl_i            => sfp_scl_in,
      sfp_sda_o            => sfp_sda_out,
      sfp_sda_i            => sfp_sda_in,
      sfp_det_i            => sfp_det_i,
      spi_sclk_o           => spi_sclk_o,
      spi_ncs_o            => spi_ncs_o,
      spi_mosi_o           => spi_mosi_o,
      spi_miso_i           => spi_miso_i,
      uart_rxd_i           => uart_rxd_i,
      uart_txd_o           => uart_txd_o,
      owr_pwren_o          => open,
      owr_en_o             => onewire_en,
      owr_i                => onewire_in,
      wrf_src_o            => wrf_src_o,
      wrf_src_i            => wrf_src_i,
      wrf_snk_o            => wrf_snk_o,
      wrf_snk_i            => wrf_snk_i,
      wrs_tx_data_i        => wrs_tx_data_i,
      wrs_tx_valid_i       => wrs_tx_valid_i,
      wrs_tx_dreq_o        => wrs_tx_dreq_o,
      wrs_tx_last_i        => wrs_tx_last_i,
      wrs_tx_flush_i       => wrs_tx_flush_i,
      wrs_tx_cfg_i         => wrs_tx_cfg_i,
      wrs_rx_first_o       => wrs_rx_first_o,
      wrs_rx_last_o        => wrs_rx_last_o,
      wrs_rx_data_o        => wrs_rx_data_o,
      wrs_rx_valid_o       => wrs_rx_valid_o,
      wrs_rx_dreq_i        => wrs_rx_dreq_i,
      wrs_rx_cfg_i         => wrs_rx_cfg_i,
      aux_diag_i           => aux_diag_i,
      aux_diag_o           => aux_diag_o,
      tm_dac_value_o       => tm_dac_value_o,
      tm_dac_wr_o          => tm_dac_wr_o,
      tm_clk_aux_lock_en_i => tm_clk_aux_lock_en_i,
      tm_clk_aux_locked_o  => tm_clk_aux_locked_o,
      timestamps_o         => timestamps_o,
      timestamps_ack_i     => timestamps_ack_i,
      abscal_txts_o        => open, --abscal_txts_o,
      abscal_rxts_o        => open, --abscal_rxts_o,
      fc_tx_pause_req_i    => fc_tx_pause_req_i,
      fc_tx_pause_delay_i  => fc_tx_pause_delay_i,
      fc_tx_pause_ready_o  => open, --fc_tx_pause_ready_o,
      tm_link_up_o         => open, --tm_link_up_o,
      tm_time_valid_o      => open, --tm_time_valid_o,
      tm_tai_o             => open, --tm_tai_o,
      tm_cycles_o          => open, --tm_cycles_o,
      led_act_o            => led_act_o,
      led_link_o           => led_link_o,
      btn1_i               => btn1_i,
      btn2_i               => btn2_i,
      pps_p_o              => pps_p_o,
      pps_led_o            => pps_led_o,
      link_ok_o            => link_ok_o);

  onewire_oen_o <= onewire_en(0);
  onewire_in(0) <= onewire_i;
  onewire_in(1) <= '1';

end architecture struct;
