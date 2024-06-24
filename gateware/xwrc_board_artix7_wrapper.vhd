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
use work.wr_clbv3_pkg.all;

entity xwrc_board_artix7_wrapper is
  generic(
    -- Select whether to include external ref clock input
    g_with_external_clock_input : boolean := TRUE;
    -- Number of aux clocks syntonized by WRPC to WR timebase
    g_aux_clks                  : integer := 0;
    -- plain                    = expose WRC fabric interface
    -- streamers                = attach WRC streamers to fabric interface
    -- etherbone                = attach Etherbone slave to fabric interface
    g_fabric_iface              : t_board_fabric_iface := streamers;
    -- memory initialization file for embedded CPU
    g_dpram_initf               : string := "default_xilinx";
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
    areset_n_i                           : in  std_logic;
    areset_edge_n_i                      : in  std_logic := '1';
    clk_125m_dmtd_i                      : in  std_logic;
    clk_125m_gtp_i                       : in  std_logic;
    clk_10m_ext_i                        : in  std_logic := '0';
    pps_ext_i                            : in  std_logic := '0';
    clk_ref_62m5_o                       : out std_logic;
    clk_ref_locked_o                     : out std_logic;
    dbg_rdy_o                            : out std_logic;
    ext_ref_rst_o                        : out std_logic;
    ready_for_reset_o                    : out std_logic;

    -- Shared SPI interface to DACs
    dac_refclk_cs_n_o                    : out std_logic;
    dac_refclk_sclk_o                    : out std_logic;
    dac_refclk_din_o                     : out std_logic;
    dac_dmtd_cs_n_o                      : out std_logic;
    dac_dmtd_sclk_o                      : out std_logic;
    dac_dmtd_din_o                       : out std_logic;

    -- SFP I/O for transceiver and SFP management info
    sfp_txp_o                            : out std_logic;
    sfp_txn_o                            : out std_logic;
    sfp_rxp_i                            : in  std_logic;
    sfp_rxn_i                            : in  std_logic;
    sfp_det_i                            : in  std_logic := '1';
    sfp_sda                              : inout std_logic;
    sfp_scl                              : inout std_logic;
    sfp_tx_fault_i                       : in  std_logic;
    sfp_tx_los_i                         : in  std_logic;

    -- Onewire interface
    onewire_i                            : in  std_logic;
    onewire_oen_o                        : out std_logic;

    -- UART
    uart_rxd_i                           : in  std_logic;
    uart_txd_o                           : out std_logic;

    -- Flash memory SPI interface
    spi_sclk_o                           : out std_logic;
    spi_ncs_o                            : out std_logic;
    spi_mosi_o                           : out std_logic;
    spi_miso_i                           : in  std_logic := '0';

    -- WR streamers (when g_fabric_iface = "streamers")
    wrs_tx_data_i                        : in  std_logic_vector(31 downto 0);
    wrs_tx_valid_i                       : in  std_logic;
    wrs_tx_dreq_o                        : out std_logic;
    wrs_tx_last_i                        : in  std_logic;
    wrs_tx_flush_i                       : in  std_logic;
    wrs_tx_cfg_mac_local                 : in std_logic_vector(47 downto 0);
    wrs_tx_cfg_mac_target                : in std_logic_vector(47 downto 0);
    wrs_tx_cfg_ethertype                 : in std_logic_vector(15 downto 0);
    wrs_tx_cfg_qtag_ena                  : in std_logic;
    wrs_tx_cfg_qtag_vid                  : in std_logic_vector(11 downto 0);
    wrs_tx_cfg_qtag_prio                 : in std_logic_vector(2 downto 0);
    wrs_tx_cfg_sw_reset                  : in std_logic;

    wrs_rx_first_o                       : out std_logic;
    wrs_rx_last_o                        : out std_logic;
    wrs_rx_data_o                        : out std_logic_vector(31 downto 0);
    wrs_rx_valid_o                       : out std_logic;
    wrs_rx_dreq_i                        : in  std_logic;
    wrs_rx_cfg_mac_local                 : in std_logic_vector(47 downto 0);
    wrs_rx_cfg_mac_remote                : in std_logic_vector(47 downto 0);
    wrs_rx_cfg_ethertype                 : in std_logic_vector(15 downto 0);
    wrs_rx_cfg_accept_broadcasts         : in std_logic;
    wrs_rx_cfg_filter_remote             : in std_logic;
    wrs_rx_cfg_fixed_latency             : in std_logic_vector(27 downto 0);
    wrs_rx_cfg_fixed_latency_timeout     : in std_logic_vector(27 downto 0);
    wrs_rx_cfg_sw_reset                  : in std_logic;

    -- Generic diagnostics interface
    aux_diag_i                           : in  t_generic_word_array(g_diag_ro_size-1 downto 0);
    aux_diag_o                           : out t_generic_word_array(g_diag_rw_size-1 downto 0);

    -- Aux clocks control
    tm_dac_value_o                       : out std_logic_vector(31 downto 0);
    tm_dac_wr_o                          : out std_logic_vector(g_aux_clks-1 downto 0);
    tm_clk_aux_lock_en_i                 : in  std_logic_vector(g_aux_clks-1 downto 0);
    tm_clk_aux_locked_o                  : out std_logic_vector(g_aux_clks-1 downto 0);

    -- External Tx Timestamping I/F
    timestamps_o                         : out t_txtsu_timestamp;
    timestamps_ack_i                     : in  std_logic := '1';

    -- Pause Frame Control
    fc_tx_pause_req_i                    : in  std_logic := '0';
    fc_tx_pause_delay_i                  : in  std_logic_vector(15 downto 0) := x"0000";

    -- Buttons, LEDs and PPS output
    led_act_o                            : out std_logic;
    led_link_o                           : out std_logic;
    btn1_i                               : in  std_logic := '1';
    btn2_i                               : in  std_logic := '1';
    pps_p_o                              : out std_logic;
    pps_led_o                            : out std_logic;
    link_ok_o                            : out std_logic;

    GT0_EXT_QPLL_RESET                   : out std_logic;
    GT0_EXT_QPLL_CLK                     : in  std_logic;
    GT0_EXT_QPLL_REFCLK                  : in  std_logic;
    GT0_EXT_QPLL_LOCK                    : in  std_logic
  );
end xwrc_board_artix7_wrapper;

architecture wrapper of xwrc_board_artix7_wrapper is

  signal tx_streamer_cfg : t_tx_streamer_cfg;
  signal rx_streamer_cfg : t_rx_streamer_cfg;

begin

  -- Map the individual signals to the record fields
  tx_streamer_cfg.mac_local             <= wrs_tx_cfg_mac_local;
  tx_streamer_cfg.mac_target            <= wrs_tx_cfg_mac_target;
  tx_streamer_cfg.ethertype             <= wrs_tx_cfg_ethertype;
  tx_streamer_cfg.qtag_ena              <= wrs_tx_cfg_qtag_ena;
  tx_streamer_cfg.qtag_vid              <= wrs_tx_cfg_qtag_vid;
  tx_streamer_cfg.qtag_prio             <= wrs_tx_cfg_qtag_prio;
  tx_streamer_cfg.sw_reset              <= wrs_tx_cfg_sw_reset;

  rx_streamer_cfg.mac_local             <= wrs_rx_cfg_mac_local;
  rx_streamer_cfg.mac_remote            <= wrs_rx_cfg_mac_remote;
  rx_streamer_cfg.ethertype             <= wrs_rx_cfg_ethertype;
  rx_streamer_cfg.accept_broadcasts     <= wrs_rx_cfg_accept_broadcasts;
  rx_streamer_cfg.filter_remote         <= wrs_rx_cfg_filter_remote;
  rx_streamer_cfg.fixed_latency         <= wrs_rx_cfg_fixed_latency;
  rx_streamer_cfg.fixed_latency_timeout <= wrs_rx_cfg_fixed_latency_timeout;
  rx_streamer_cfg.sw_reset              <= wrs_rx_cfg_sw_reset;

  u_xwrc_board_artix7 : entity work.xwrc_board_artix7
    generic map (
      g_with_external_clock_input => g_with_external_clock_input,
      g_aux_clks                  => g_aux_clks,
      g_fabric_iface              => g_fabric_iface,
      g_streamers_op_mode         => TX_AND_RX,
      g_tx_streamer_params        => c_tx_streamer_params_defaut,
      g_rx_streamer_params        => c_rx_streamer_params_defaut,
      g_dpram_initf               => g_dpram_initf,
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
      clk_125m_dmtd_i      => clk_125m_dmtd_i,
      clk_125m_gtp_i       => clk_125m_gtp_i,
      clk_10m_ext_i        => clk_10m_ext_i,
      pps_ext_i            => pps_ext_i,
      clk_ref_62m5_o       => clk_ref_62m5_o,
      clk_ref_locked_o     => clk_ref_locked_o,
      dbg_rdy_o            => dbg_rdy_o,
      ext_ref_rst_o        => ext_ref_rst_o,
      ready_for_reset_o    => ready_for_reset_o,
      dac_refclk_cs_n_o    => dac_refclk_cs_n_o,
      dac_refclk_sclk_o    => dac_refclk_sclk_o,
      dac_refclk_din_o     => dac_refclk_din_o,
      dac_dmtd_cs_n_o      => dac_dmtd_cs_n_o,
      dac_dmtd_sclk_o      => dac_dmtd_sclk_o,
      dac_dmtd_din_o       => dac_dmtd_din_o,
      sfp_txp_o            => sfp_txp_o,
      sfp_txn_o            => sfp_txn_o,
      sfp_rxp_i            => sfp_rxp_i,
      sfp_rxn_i            => sfp_rxn_i,
      sfp_det_i            => sfp_det_i,
      sfp_sda              => sfp_sda,
      sfp_scl              => sfp_scl,
      sfp_tx_fault_i       => sfp_tx_fault_i,
      sfp_tx_los_i         => sfp_tx_los_i,
      onewire_i            => onewire_i,
      onewire_oen_o        => onewire_oen_o,
      uart_rxd_i           => uart_rxd_i,
      uart_txd_o           => uart_txd_o,
      spi_sclk_o           => spi_sclk_o,
      spi_ncs_o            => spi_ncs_o,
      spi_mosi_o           => spi_mosi_o,
      spi_miso_i           => spi_miso_i,
      wrs_tx_data_i        => wrs_tx_data_i,
      wrs_tx_valid_i       => wrs_tx_valid_i,
      wrs_tx_dreq_o        => wrs_tx_dreq_o,
      wrs_tx_last_i        => wrs_tx_last_i,
      wrs_tx_flush_i       => wrs_tx_flush_i,
      wrs_tx_cfg_i         => tx_streamer_cfg,
      wrs_rx_first_o       => wrs_rx_first_o,
      wrs_rx_last_o        => wrs_rx_last_o,
      wrs_rx_data_o        => wrs_rx_data_o,
      wrs_rx_valid_o       => wrs_rx_valid_o,
      wrs_rx_dreq_i        => wrs_rx_dreq_i,
      wrs_rx_cfg_i         => rx_streamer_cfg,
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
      GT0_EXT_QPLL_RESET   => GT0_EXT_QPLL_RESET,
      GT0_EXT_QPLL_CLK     => GT0_EXT_QPLL_CLK,
      GT0_EXT_QPLL_REFCLK  => GT0_EXT_QPLL_REFCLK,
      GT0_EXT_QPLL_LOCK    => GT0_EXT_QPLL_LOCK
    );

end architecture;
