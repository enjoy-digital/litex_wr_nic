-- SPDX-License-Identifier: BSD-2-Clause
-- Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
-- Flatten the upstream WR records for a LiteX-supplied 8-bit PHY.
-- Clocks, resets, SerDes and 8b/10b are provided by the caller.
library ieee;
use ieee.std_logic_1164.all;
use work.wishbone_pkg.all;
use work.wr_fabric_pkg.all;
use work.endpoint_pkg.all;
use work.wr_board_pkg.all;

entity xwrc_litex_phy8 is
  generic (
    g_dpram_initf : string := "";
    g_dpram_size : integer := 32768;
    g_external_cpu_memory : boolean := false;
    g_external_cpu : boolean := false;
    g_softpll_enable_debugger : boolean := false;
    g_board_name : string := "LTX8"
  );
  port (
    clk_sys_i : in std_logic;
    clk_ref_i : in std_logic;
    clk_dmtd_i : in std_logic;
    rst_n_i : in std_logic;
    uart_rxd_i : in std_logic;
    sfp_det_i : in std_logic;
    uart_txd_o : out std_logic;
    dac_refclk_load : out std_logic;
    dac_refclk_data : out std_logic_vector(15 downto 0);
    dac_dmtd_load : out std_logic;
    dac_dmtd_data : out std_logic_vector(15 downto 0);
    led_act_o : out std_logic;
    led_link_o : out std_logic;
    pps_valid_o : out std_logic;
    pps_csync_o : out std_logic;
    pps_p_o : out std_logic;
    pps_led_o : out std_logic;
    tm_link_up_o : out std_logic;
    tm_time_valid_o : out std_logic;
    tm_tai_o : out std_logic_vector(39 downto 0);
    tm_cycles_o : out std_logic_vector(27 downto 0);
    wb_slave_cyc : in std_logic;
    wb_slave_stb : in std_logic;
    wb_slave_we : in std_logic;
    wb_slave_adr : in std_logic_vector(31 downto 0);
    wb_slave_sel : in std_logic_vector(3 downto 0);
    wb_slave_dat_i : in std_logic_vector(31 downto 0);
    wb_slave_dat_o : out std_logic_vector(31 downto 0);
    wb_slave_ack : out std_logic;
    wb_slave_err : out std_logic;
    wb_slave_rty : out std_logic;
    wb_slave_stall : out std_logic;
    cpu_mem_cyc_o : out std_logic;
    cpu_mem_stb_o : out std_logic;
    cpu_mem_we_o : out std_logic;
    cpu_mem_adr_o : out std_logic_vector(31 downto 0);
    cpu_mem_sel_o : out std_logic_vector(3 downto 0);
    cpu_mem_dat_o : out std_logic_vector(31 downto 0);
    cpu_mem_dat_i : in std_logic_vector(31 downto 0);
    cpu_mem_ack_i : in std_logic;
    cpu_mem_err_i : in std_logic;
    cpu_mem_rty_i : in std_logic;
    cpu_mem_stall_i : in std_logic;
    cpu_ext_cyc_i : in std_logic;
    cpu_ext_stb_i : in std_logic;
    cpu_ext_we_i : in std_logic;
    cpu_ext_adr_i : in std_logic_vector(31 downto 0);
    cpu_ext_sel_i : in std_logic_vector(3 downto 0);
    cpu_ext_dat_i : in std_logic_vector(31 downto 0);
    cpu_ext_dat_o : out std_logic_vector(31 downto 0);
    cpu_ext_ack_o : out std_logic;
    cpu_ext_err_o : out std_logic;
    cpu_ext_rty_o : out std_logic;
    cpu_ext_stall_o : out std_logic;
    cpu_mem_ready_i : in std_logic;
    cpu_ext_irq_o : out std_logic;
    cpu_ext_reset_o : out std_logic;
    wrf_src_adr : out std_logic_vector(1 downto 0);
    wrf_src_dat : out std_logic_vector(15 downto 0);
    wrf_src_cyc : out std_logic;
    wrf_src_stb : out std_logic;
    wrf_src_we : out std_logic;
    wrf_src_sel : out std_logic_vector(1 downto 0);
    wrf_src_ack : in std_logic;
    wrf_src_stall : in std_logic;
    wrf_src_err : in std_logic;
    wrf_src_rty : in std_logic;
    wrf_snk_adr : in std_logic_vector(1 downto 0);
    wrf_snk_dat : in std_logic_vector(15 downto 0);
    wrf_snk_cyc : in std_logic;
    wrf_snk_stb : in std_logic;
    wrf_snk_we : in std_logic;
    wrf_snk_sel : in std_logic_vector(1 downto 0);
    wrf_snk_ack : out std_logic;
    wrf_snk_stall : out std_logic;
    wrf_snk_err : out std_logic;
    wrf_snk_rty : out std_logic;
    phy_tx_disparity_i : in std_logic;
    phy_tx_enc_err_i : in std_logic;
    phy_rx_data_i : in std_logic_vector(7 downto 0);
    phy_rx_clk_i : in std_logic;
    phy_rx_enc_err_i : in std_logic;
    phy_rx_bitslide_i : in std_logic_vector(3 downto 0);
    phy_rdy_i : in std_logic;
    phy_sfp_tx_fault_i : in std_logic;
    phy_sfp_los_i : in std_logic;
    phy_rx_k_i : in std_logic;
    phy_rst_o : out std_logic;
    phy_loopen_o : out std_logic;
    phy_tx_data_o : out std_logic_vector(7 downto 0);
    phy_sfp_tx_disable_o : out std_logic;
    phy_tx_k_o : out std_logic
  );
end entity;

architecture rtl of xwrc_litex_phy8 is
  signal host_in : t_wishbone_slave_in;
  signal host_out : t_wishbone_slave_out;
  signal cpu_mem_out : t_wishbone_master_out;
  signal cpu_mem_in : t_wishbone_master_in;
  signal cpu_ext_in : t_wishbone_master_out;
  signal cpu_ext_out : t_wishbone_master_in;
  signal wrf_src_req : t_wrf_source_out;
  signal wrf_src_rsp : t_wrf_source_in;
  signal wrf_snk_req : t_wrf_source_out;
  signal wrf_snk_rsp : t_wrf_source_in;
  signal phy_in : t_phy_8bits_to_wrc := c_dummy_phy8_to_wrc;
  signal phy_out : t_phy_8bits_from_wrc;
begin
  host_in.cyc <= wb_slave_cyc;
  host_in.stb <= wb_slave_stb;
  host_in.we <= wb_slave_we;
  host_in.adr <= wb_slave_adr;
  host_in.sel <= wb_slave_sel;
  host_in.dat <= wb_slave_dat_i;
  wb_slave_dat_o <= host_out.dat;
  wb_slave_ack <= host_out.ack;
  wb_slave_err <= host_out.err;
  wb_slave_rty <= host_out.rty;
  wb_slave_stall <= host_out.stall;
  cpu_mem_cyc_o <= cpu_mem_out.cyc;
  cpu_mem_stb_o <= cpu_mem_out.stb;
  cpu_mem_we_o <= cpu_mem_out.we;
  cpu_mem_adr_o <= cpu_mem_out.adr;
  cpu_mem_sel_o <= cpu_mem_out.sel;
  cpu_mem_dat_o <= cpu_mem_out.dat;
  cpu_mem_in.dat <= cpu_mem_dat_i;
  cpu_mem_in.ack <= cpu_mem_ack_i;
  cpu_mem_in.err <= cpu_mem_err_i;
  cpu_mem_in.rty <= cpu_mem_rty_i;
  cpu_mem_in.stall <= cpu_mem_stall_i;
  cpu_ext_in.cyc <= cpu_ext_cyc_i;
  cpu_ext_in.stb <= cpu_ext_stb_i;
  cpu_ext_in.we <= cpu_ext_we_i;
  cpu_ext_in.adr <= cpu_ext_adr_i;
  cpu_ext_in.sel <= cpu_ext_sel_i;
  cpu_ext_in.dat <= cpu_ext_dat_i;
  cpu_ext_dat_o <= cpu_ext_out.dat;
  cpu_ext_ack_o <= cpu_ext_out.ack;
  cpu_ext_err_o <= cpu_ext_out.err;
  cpu_ext_rty_o <= cpu_ext_out.rty;
  cpu_ext_stall_o <= cpu_ext_out.stall;
  wrf_src_adr <= wrf_src_req.adr;
  wrf_src_dat <= wrf_src_req.dat;
  wrf_src_cyc <= wrf_src_req.cyc;
  wrf_src_stb <= wrf_src_req.stb;
  wrf_src_we <= wrf_src_req.we;
  wrf_src_sel <= wrf_src_req.sel;
  wrf_src_rsp.ack <= wrf_src_ack;
  wrf_src_rsp.stall <= wrf_src_stall;
  wrf_src_rsp.err <= wrf_src_err;
  wrf_src_rsp.rty <= wrf_src_rty;
  wrf_snk_req.adr <= wrf_snk_adr;
  wrf_snk_req.dat <= wrf_snk_dat;
  wrf_snk_req.cyc <= wrf_snk_cyc;
  wrf_snk_req.stb <= wrf_snk_stb;
  wrf_snk_req.we <= wrf_snk_we;
  wrf_snk_req.sel <= wrf_snk_sel;
  wrf_snk_ack <= wrf_snk_rsp.ack;
  wrf_snk_stall <= wrf_snk_rsp.stall;
  wrf_snk_err <= wrf_snk_rsp.err;
  wrf_snk_rty <= wrf_snk_rsp.rty;
  phy_in.tx_disparity <= phy_tx_disparity_i;
  phy_in.tx_enc_err <= phy_tx_enc_err_i;
  phy_in.rx_data <= phy_rx_data_i;
  phy_in.rx_clk <= phy_rx_clk_i;
  phy_in.rx_enc_err <= phy_rx_enc_err_i;
  phy_in.rx_bitslide <= phy_rx_bitslide_i;
  phy_in.rdy <= phy_rdy_i;
  phy_in.sfp_tx_fault <= phy_sfp_tx_fault_i;
  phy_in.sfp_los <= phy_sfp_los_i;
  phy_in.rx_k(0) <= phy_rx_k_i;
  phy_in.ref_clk <= clk_ref_i;
  phy_in.rx_sampled_clk <= '0';
  phy_rst_o <= phy_out.rst;
  phy_loopen_o <= phy_out.loopen;
  phy_tx_data_o <= phy_out.tx_data;
  phy_sfp_tx_disable_o <= phy_out.sfp_tx_disable;
  phy_tx_k_o <= phy_out.tx_k(0);
  core : entity work.xwrc_board_common
    generic map (
      g_board_name => g_board_name,
      g_pcs_16bit => false,
      g_with_external_clock_input => false,
      g_dpram_initf => g_dpram_initf,
      g_dpram_size => g_dpram_size,
      g_external_cpu_memory => g_external_cpu_memory,
      g_external_cpu => g_external_cpu,
      g_softpll_enable_debugger => g_softpll_enable_debugger,
      g_interface_mode => PIPELINED,
      g_address_granularity => BYTE,
      g_fabric_iface => PLAIN
    )
    port map (
      clk_sys_i => clk_sys_i,
      clk_ref_i => clk_ref_i,
      clk_dmtd_i => clk_dmtd_i,
      rst_n_i => rst_n_i,
      uart_rxd_i => uart_rxd_i,
      sfp_det_i => sfp_det_i,
      uart_txd_o => uart_txd_o,
      dac_dpll_load_p1_o => dac_refclk_load,
      dac_dpll_data_o => dac_refclk_data,
      dac_hpll_load_p1_o => dac_dmtd_load,
      dac_hpll_data_o => dac_dmtd_data,
      led_act_o => led_act_o,
      led_link_o => led_link_o,
      pps_valid_o => pps_valid_o,
      pps_csync_o => pps_csync_o,
      pps_p_o => pps_p_o,
      pps_led_o => pps_led_o,
      tm_link_up_o => tm_link_up_o,
      tm_time_valid_o => tm_time_valid_o,
      tm_tai_o => tm_tai_o,
      tm_cycles_o => tm_cycles_o,
      wb_slave_i => host_in,
      wb_slave_o => host_out,
      cpu_mem_o => cpu_mem_out,
      cpu_mem_i => cpu_mem_in,
      cpu_ext_master_i => cpu_ext_in,
      cpu_ext_master_o => cpu_ext_out,
      cpu_mem_ready_i => cpu_mem_ready_i,
      cpu_ext_irq_o => cpu_ext_irq_o,
      cpu_ext_reset_o => cpu_ext_reset_o,
      wrf_src_o => wrf_src_req,
      wrf_src_i => wrf_src_rsp,
      wrf_snk_i => wrf_snk_req,
      wrf_snk_o => wrf_snk_rsp,
      phy8_i => phy_in,
      phy8_o => phy_out
    );
end architecture;
