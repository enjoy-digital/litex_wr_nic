--
-- This file is part of SPEC-A7.
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

library work;
use work.gencores_pkg.all;
use work.wrcore_pkg.all;
use work.wishbone_pkg.all;
use work.wr_fabric_pkg.all;
use work.endpoint_pkg.all;
use work.streamers_pkg.all;
use work.wr_xilinx_pkg.all;
use work.wr_board_pkg.all;

entity xwrc_board_spec_a7_wrapper is
  generic(
    -- Select whether to include external ref clock input
    g_with_external_clock_input : boolean := TRUE;
    -- Number of aux clocks syntonized by WRPC to WR timebase
    g_aux_clks                  : integer := 0;
    -- plain                    = expose WRC fabric interface
    -- streamers                = attach WRC streamers to fabric interface
    -- etherbone                = attach Etherbone slave to fabric interface
    g_fabric_iface              : t_board_fabric_iface := PLAIN;
    -- memory initialization file for embedded CPU
    g_dpram_initf               : string  := "default_xilinx";
    g_dpram_size                : integer := 131072/4;
    -- identification (id and ver) of the layout of words in the generic diag interface
    g_diag_id                   : integer := 0;
    g_diag_ver                  : integer := 0;
    -- size the generic diag interface
    g_diag_ro_size              : integer := 0;
    g_diag_rw_size              : integer := 0;
    -- GTPE2_CHANNEL TX Polarity Control Ports
    txpolarity                  : bit := '0';
    -- GTPE2_CHANNEL RX Polarity Control Ports
    rxpolarity                  : bit := '1'
  );
  port (
    -- Clocks/resets
    areset_n_i           : in  std_logic;
    areset_edge_n_i      : in  std_logic := '1';
    clk_62p5m_dmtd_i     : in  std_logic;
    clk_125m_gtp_i       : in  std_logic;
    clk_10m_ext_i        : in  std_logic := '0';
    pps_ext_i            : in  std_logic := '0';
    clk_ref_62m5_o       : out std_logic;
    clk_62m5_sys_o       : out std_logic;
    clk_ref_locked_o     : out std_logic;
    dbg_rdy_o            : out std_logic;
    ext_ref_rst_o        : out std_logic;
    ready_for_reset_o    : out std_logic;

    -- Serial DACs
    dac_refclk_load      : out std_logic;
    dac_refclk_data      : out std_logic_vector(15 downto 0);

    dac_dmtd_load        : out std_logic;
    dac_dmtd_data        : out std_logic_vector(15 downto 0);

    -- SFP I/O for transceiver and SFP management info
    sfp_txp_o            : out std_logic;
    sfp_txn_o            : out std_logic;
    sfp_rxp_i            : in  std_logic;
    sfp_rxn_i            : in  std_logic;
    sfp_det_i            : in  std_logic := '1';
    sfp_sda              : inout std_logic;
    sfp_scl              : inout std_logic;
    sfp_tx_fault_i       : in  std_logic;
    sfp_tx_los_i         : in  std_logic;
    sfp_tx_disable_o     : out std_logic;

    -- Onewire interface
    onewire_i            : in  std_logic;
    onewire_oen_o        : out std_logic;

    -- UART
    uart_rxd_i           : in  std_logic;
    uart_txd_o           : out std_logic;

    -- Flash memory SPI interface
    spi_sclk_o           : out std_logic;
    spi_ncs_o            : out std_logic;
    spi_mosi_o           : out std_logic;
    spi_miso_i           : in  std_logic := '0';

    -- WRF
    wrf_src_adr          : out std_logic_vector(1 downto 0);
    wrf_src_dat          : out std_logic_vector(15 downto 0);
    wrf_src_cyc          : out std_logic;
    wrf_src_stb          : out std_logic;
    wrf_src_we           : out std_logic;
    wrf_src_sel          : out std_logic_vector(1 downto 0);

    wrf_src_ack          : in std_logic;
    wrf_src_stall        : in std_logic;
    wrf_src_err          : in std_logic;
    wrf_src_rty          : in std_logic;

    wrf_snk_adr          : in std_logic_vector(1 downto 0);
    wrf_snk_dat          : in std_logic_vector(15 downto 0);
    wrf_snk_cyc          : in std_logic;
    wrf_snk_stb          : in std_logic;
    wrf_snk_we           : in std_logic;
    wrf_snk_sel          : in std_logic_vector(1 downto 0);

    wrf_snk_ack          : out std_logic;
    wrf_snk_stall        : out std_logic;
    wrf_snk_err          : out std_logic;
    wrf_snk_rty          : out std_logic;

    -- WB Slave
    wb_slave_cyc         : in  std_logic;
    wb_slave_stb         : in  std_logic;
    wb_slave_we          : in  std_logic;
    wb_slave_adr         : in  std_logic_vector(31 downto 0);
    wb_slave_sel         : in  std_logic_vector(3 downto 0);
    wb_slave_dat_i       : in  std_logic_vector(31 downto 0);
    wb_slave_dat_o       : out std_logic_vector(31 downto 0);
    wb_slave_ack         : out std_logic;
    wb_slave_err         : out std_logic;
    wb_slave_rty         : out std_logic;
    wb_slave_stall       : out std_logic;

    -- Generic diagnostics interface
    aux_diag_i           : in  t_generic_word_array(g_diag_ro_size-1 downto 0);
    aux_diag_o           : out t_generic_word_array(g_diag_rw_size-1 downto 0);

    -- Aux clocks control
    tm_dac_value_o       : out std_logic_vector(31 downto 0);
    tm_dac_wr_o          : out std_logic_vector(g_aux_clks-1 downto 0);
    tm_clk_aux_lock_en_i : in  std_logic_vector(g_aux_clks-1 downto 0);
    tm_clk_aux_locked_o  : out std_logic_vector(g_aux_clks-1 downto 0);

    -- External Tx Timestamping I/F
    timestamps_o         : out t_txtsu_timestamp;
    timestamps_ack_i     : in  std_logic := '1';

    -- Pause Frame Control
    fc_tx_pause_req_i    : in  std_logic := '0';
    fc_tx_pause_delay_i  : in  std_logic_vector(15 downto 0) := x"0000";

    -- Buttons, LEDs and PPS output
    led_act_o            : out std_logic;
    led_link_o           : out std_logic;
    btn1_i               : in  std_logic := '1';
    btn2_i               : in  std_logic := '1';
    pps_p_o              : out std_logic;
    pps_led_o            : out std_logic;
    link_ok_o            : out std_logic;

    -- QPLL Rst/Clk/Lock.
    gt0_ext_qpll_reset   : out std_logic;
    gt0_ext_qpll_clk     : in  std_logic;
    gt0_ext_qpll_refclk  : in  std_logic;
    gt0_ext_qpll_lock    : in  std_logic;

    -- TX PI Control.
    txpippmen            : in std_logic;
    txpippmstepsize      : in std_logic_vector(4 downto 0)
  );
end xwrc_board_spec_a7_wrapper;

architecture wrapper of xwrc_board_spec_a7_wrapper is

  signal wrf_src_o : t_wrf_source_out;
  signal wrf_src_i : t_wrf_source_in := c_dummy_src_in;
  signal wrf_snk_o : t_wrf_sink_out;
  signal wrf_snk_i : t_wrf_sink_in := c_dummy_snk_in;

  signal wb_slave_i : t_wishbone_slave_in  := cc_dummy_slave_in;
  signal wb_slave_o : t_wishbone_slave_out;

begin

  -- wrf_src Record -> Signals.
  wrf_src_adr     <= wrf_src_o.adr;
  wrf_src_dat     <= wrf_src_o.dat;
  wrf_src_cyc     <= wrf_src_o.cyc;
  wrf_src_stb     <= wrf_src_o.stb;
  wrf_src_we      <= wrf_src_o.we;
  wrf_src_sel     <= wrf_src_o.sel;

  -- wrf_src Signals -> Record.
  wrf_src_i.ack   <= wrf_src_ack;
  wrf_src_i.stall <= wrf_src_stall;
  wrf_src_i.err   <= wrf_src_err;
  wrf_src_i.rty   <= wrf_src_rty;

  -- wrf_snk Signals -> Record.
  wrf_snk_i.adr   <= wrf_snk_adr;
  wrf_snk_i.dat   <= wrf_snk_dat;
  wrf_snk_i.cyc   <= wrf_snk_cyc;
  wrf_snk_i.stb   <= wrf_snk_stb;
  wrf_snk_i.we    <= wrf_snk_we;
  wrf_snk_i.sel   <= wrf_snk_sel;

  -- wrf_snk Record -> Signals.
  wrf_snk_ack     <= wrf_snk_o.ack;
  wrf_snk_stall   <= wrf_snk_o.stall;
  wrf_snk_err     <= wrf_snk_o.err;
  wrf_snk_rty     <= wrf_snk_o.rty;

  -- wrf_slave Signals -> Record.
  wb_slave_i.cyc  <= wb_slave_cyc;
  wb_slave_i.stb  <= wb_slave_stb;
  wb_slave_i.adr  <= std_logic_vector(wb_slave_adr);
  wb_slave_i.sel  <= std_logic_vector(wb_slave_sel);
  wb_slave_i.we   <= wb_slave_we;
  wb_slave_i.dat  <= std_logic_vector(wb_slave_dat_i);

  -- wrf_slave Record -> Signals.
  wb_slave_dat_o  <= std_logic_vector(wb_slave_o.dat);
  wb_slave_ack    <= wb_slave_o.ack;
  wb_slave_err    <= wb_slave_o.err;
  wb_slave_rty    <= wb_slave_o.rty;
  wb_slave_stall  <= wb_slave_o.stall;

  -- xwrc_board_spec_a7 Instance.
  u_xwrc_board_spec_a7 : entity work.xwrc_board_spec_a7
    generic map (
      g_with_external_clock_input => g_with_external_clock_input,
      g_aux_clks                  => g_aux_clks,
      g_fabric_iface              => PLAIN,
      g_streamers_op_mode         => TX_AND_RX,
      g_tx_streamer_params        => c_tx_streamer_params_defaut,
      g_rx_streamer_params        => c_rx_streamer_params_defaut,
      g_dpram_initf               => g_dpram_initf,
      g_dpram_size                => g_dpram_size,
      g_diag_id                   => g_diag_id,
      g_diag_ver                  => g_diag_ver,
      g_diag_ro_size              => g_diag_ro_size,
      g_diag_rw_size              => g_diag_rw_size,
      txpolarity                  => txpolarity,
      rxpolarity                  => rxpolarity
    )
    port map (
      areset_n_i           => areset_n_i,
      areset_edge_n_i      => areset_edge_n_i,
      clk_62p5m_dmtd_i     => clk_62p5m_dmtd_i,
      clk_125m_gtp_i       => clk_125m_gtp_i,
      clk_10m_ext_i        => clk_10m_ext_i,
      pps_ext_i            => pps_ext_i,
      clk_ref_62m5_o       => clk_ref_62m5_o,
      clk_62m5_sys_o       => clk_62m5_sys_o,
      clk_ref_locked_o     => clk_ref_locked_o,
      dbg_rdy_o            => dbg_rdy_o,
      ext_ref_rst_o        => ext_ref_rst_o,
      ready_for_reset_o    => ready_for_reset_o,
      dac_refclk_load      => dac_refclk_load,
      dac_refclk_data      => dac_refclk_data,
      dac_dmtd_load        => dac_dmtd_load,
      dac_dmtd_data        => dac_dmtd_data,
      sfp_txp_o            => sfp_txp_o,
      sfp_txn_o            => sfp_txn_o,
      sfp_rxp_i            => sfp_rxp_i,
      sfp_rxn_i            => sfp_rxn_i,
      sfp_det_i            => sfp_det_i,
      sfp_sda              => sfp_sda,
      sfp_scl              => sfp_scl,
      sfp_tx_fault_i       => sfp_tx_fault_i,
      sfp_tx_los_i         => sfp_tx_los_i,
      sfp_tx_disable_o     => sfp_tx_disable_o,
      onewire_i            => onewire_i,
      onewire_oen_o        => onewire_oen_o,
      uart_rxd_i           => uart_rxd_i,
      uart_txd_o           => uart_txd_o,
      spi_sclk_o           => spi_sclk_o,
      spi_ncs_o            => spi_ncs_o,
      spi_mosi_o           => spi_mosi_o,
      spi_miso_i           => spi_miso_i,

      wrf_src_o            => wrf_src_o,
      wrf_src_i            => wrf_src_i,
      wrf_snk_o            => wrf_snk_o,
      wrf_snk_i            => wrf_snk_i,

      wb_slave_i           => wb_slave_i,
      wb_slave_o           => wb_slave_o,
      aux_diag_i           => aux_diag_i,
      aux_diag_o           => aux_diag_o,
      tm_dac_value_o       => tm_dac_value_o,
      tm_dac_wr_o          => tm_dac_wr_o,
      tm_clk_aux_lock_en_i => tm_clk_aux_lock_en_i,
      tm_clk_aux_locked_o  => tm_clk_aux_locked_o,
      timestamps_o         => timestamps_o,
      timestamps_ack_i     => timestamps_ack_i,
      fc_tx_pause_req_i    => fc_tx_pause_req_i,
      fc_tx_pause_delay_i  => fc_tx_pause_delay_i,
      led_act_o            => led_act_o,
      led_link_o           => led_link_o,
      btn1_i               => btn1_i,
      btn2_i               => btn2_i,
      pps_p_o              => pps_p_o,
      pps_led_o            => pps_led_o,
      link_ok_o            => link_ok_o,
      GT0_EXT_QPLL_RESET   => gt0_ext_qpll_reset,
      GT0_EXT_QPLL_CLK     => gt0_ext_qpll_clk,
      GT0_EXT_QPLL_REFCLK  => gt0_ext_qpll_refclk,
      GT0_EXT_QPLL_LOCK    => gt0_ext_qpll_lock,
      txpippmen            => txpippmen,
      txpippmstepsize      => txpippmstepsize
    );

end architecture;
