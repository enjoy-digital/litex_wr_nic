#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import os
import subprocess

from litex.build import tools

# WR Core Init -------------------------------------------------------------------------------------

WR_CORES_URL    = "https://gitlab.com/ohwr/project/wr-cores.git"
WR_CORES_BRANCH = "wrpc-v5"
WR_CORES_SHA1   = "39825ec55291cb12492090093f27a50f9d0b73d9"

def wr_core_init():
    print("Cloning wr-cores repository...")
    subprocess.run(["git", "clone", WR_CORES_URL])
    os.chdir("wr-cores")
    print("Checking out the specified commit...")
    subprocess.run(["git", "checkout", WR_CORES_SHA1, "-b", WR_CORES_BRANCH])
    print("Fixing submodules URL...")
    tools.replace_in_file(".gitmodules", "ohwr.org", "gitlab.com/ohwr")
    print("Updating submodules...")
    subprocess.run(["git", "submodule", "update", "--init"])
    print("wr-cores initialization complete.")
    os.chdir("..")

# WR Core Files ------------------------------------------------------------------------------------

wr_core_files = [
    # WR Unmodified files.
    # ----------------------------------------------------------------------------------------------

    # Common Packages.
    "wr-cores/board/common/wr_board_pkg.vhd",
    "wr-cores/board/common/xwrc_board_common.vhd",

    # Common Modules.
    "wr-cores/ip_cores/general-cores/modules/common/gc_async_counter_diff.vhd",
    "wr-cores/ip_cores/general-cores/modules/common/gc_crc_gen.vhd",
    "wr-cores/ip_cores/general-cores/modules/common/gc_edge_detect.vhd",
    "wr-cores/ip_cores/general-cores/modules/common/gc_extend_pulse.vhd",
    "wr-cores/ip_cores/general-cores/modules/common/gc_pulse_synchronizer.vhd",
    "wr-cores/ip_cores/general-cores/modules/common/gc_pulse_synchronizer2.vhd",
    "wr-cores/ip_cores/general-cores/modules/common/gc_reset.vhd",
    "wr-cores/ip_cores/general-cores/modules/common/gc_reset_multi_aasd.vhd",
    "wr-cores/ip_cores/general-cores/modules/common/gc_serial_dac.vhd",
    "wr-cores/ip_cores/general-cores/modules/common/gc_sync.vhd",
    "wr-cores/ip_cores/general-cores/modules/common/gc_sync_ffs.vhd",
    "wr-cores/ip_cores/general-cores/modules/common/gc_sync_register.vhd",
    "wr-cores/ip_cores/general-cores/modules/common/gencores_pkg.vhd",

    # RAMs Modules.
    "wr-cores/ip_cores/general-cores/modules/genrams/common/generic_shiftreg_fifo.vhd",
    "wr-cores/ip_cores/general-cores/modules/genrams/common/inferred_async_fifo.vhd",
    "wr-cores/ip_cores/general-cores/modules/genrams/common/inferred_async_fifo_dual_rst.vhd",
    "wr-cores/ip_cores/general-cores/modules/genrams/common/inferred_sync_fifo.vhd",
    "wr-cores/ip_cores/general-cores/modules/genrams/generic/generic_async_fifo.vhd",
    "wr-cores/ip_cores/general-cores/modules/genrams/generic/generic_async_fifo_dual_rst.vhd",
    "wr-cores/ip_cores/general-cores/modules/genrams/generic/generic_sync_fifo.vhd",
    "wr-cores/ip_cores/general-cores/modules/genrams/genram_pkg.vhd",
    "wr-cores/ip_cores/general-cores/modules/genrams/memory_loader_pkg.vhd",
    "wr-cores/ip_cores/general-cores/modules/genrams/xilinx/gc_shiftreg.vhd",
    "wr-cores/ip_cores/general-cores/modules/genrams/xilinx/generic_dpram.vhd",
    "wr-cores/ip_cores/general-cores/modules/genrams/xilinx/generic_dpram_dualclock.vhd",
    "wr-cores/ip_cores/general-cores/modules/genrams/xilinx/generic_dpram_sameclock.vhd",
    "wr-cores/ip_cores/general-cores/modules/genrams/xilinx/generic_dpram_split.vhd",

    # Peripherals Modules.
    "wr-cores/ip_cores/general-cores/modules/wishbone/wb_clock_monitor/clock_monitor_wb.vhd",
    "wr-cores/ip_cores/general-cores/modules/wishbone/wb_clock_monitor/clock_monitor_wbgen2_pkg.vhd",
    "wr-cores/ip_cores/general-cores/modules/wishbone/wb_clock_monitor/xwb_clock_monitor.vhd",
    "wr-cores/ip_cores/general-cores/modules/wishbone/wb_crossbar/sdb_rom.vhd",
    "wr-cores/ip_cores/general-cores/modules/wishbone/wb_crossbar/xwb_crossbar.vhd",
    "wr-cores/ip_cores/general-cores/modules/wishbone/wb_crossbar/xwb_sdb_crossbar.vhd",
    "wr-cores/ip_cores/general-cores/modules/wishbone/wb_onewire_master/sockit_owm.v",
    "wr-cores/ip_cores/general-cores/modules/wishbone/wb_onewire_master/wb_onewire_master.vhd",
    "wr-cores/ip_cores/general-cores/modules/wishbone/wb_onewire_master/xwb_onewire_master.vhd",
    "wr-cores/ip_cores/general-cores/modules/wishbone/wb_slave_adapter/wb_slave_adapter.vhd",
    "wr-cores/ip_cores/general-cores/modules/wishbone/wb_uart/simple_uart_pkg.vhd",
    "wr-cores/ip_cores/general-cores/modules/wishbone/wb_uart/simple_uart_wb.vhd",
    "wr-cores/ip_cores/general-cores/modules/wishbone/wb_uart/uart_async_rx.vhd",
    "wr-cores/ip_cores/general-cores/modules/wishbone/wb_uart/uart_async_tx.vhd",
    "wr-cores/ip_cores/general-cores/modules/wishbone/wb_uart/uart_baud_gen.vhd",
    "wr-cores/ip_cores/general-cores/modules/wishbone/wb_uart/wb_simple_uart.vhd",
    "wr-cores/ip_cores/general-cores/modules/wishbone/wb_uart/xwb_simple_uart.vhd",
    "wr-cores/ip_cores/general-cores/modules/wishbone/wbgen2/wbgen2_eic.vhd",
    "wr-cores/ip_cores/general-cores/modules/wishbone/wbgen2/wbgen2_fifo_sync.vhd",
    "wr-cores/ip_cores/general-cores/modules/wishbone/wbgen2/wbgen2_pkg.vhd",
    "wr-cores/ip_cores/general-cores/modules/wishbone/wishbone_pkg.vhd",

    # uRV CPU Modules.
    "wr-cores/ip_cores/urv-core/rtl/urv_cpu.v",
    "wr-cores/ip_cores/urv-core/rtl/urv_csr.v",
    "wr-cores/ip_cores/urv-core/rtl/urv_decode.v",
    "wr-cores/ip_cores/urv-core/rtl/urv_divide.v",
    "wr-cores/ip_cores/urv-core/rtl/urv_ecc.v",
    "wr-cores/ip_cores/urv-core/rtl/urv_exceptions.v",
    "wr-cores/ip_cores/urv-core/rtl/urv_exec.v",
    "wr-cores/ip_cores/urv-core/rtl/urv_fetch.v",
    "wr-cores/ip_cores/urv-core/rtl/urv_multiply.v",
    "wr-cores/ip_cores/urv-core/rtl/urv_pkg.vhd",
    "wr-cores/ip_cores/urv-core/rtl/urv_regfile.v",
    "wr-cores/ip_cores/urv-core/rtl/urv_shifter.v",
    "wr-cores/ip_cores/urv-core/rtl/urv_timer.v",
    "wr-cores/ip_cores/urv-core/rtl/urv_writeback.v",

    # Fabric Modules.
    "wr-cores/modules/fabric/wr_fabric_pkg.vhd",
    "wr-cores/modules/fabric/xwb_fabric_sink.vhd",
    "wr-cores/modules/fabric/xwb_fabric_source.vhd",
    "wr-cores/modules/fabric/xwrf_loopback/lbk_pkg.vhd",
    "wr-cores/modules/fabric/xwrf_loopback/lbk_wishbone_controller.vhd",
    "wr-cores/modules/fabric/xwrf_loopback/xwrf_loopback.vhd",
    #"wr-cores/modules/fabric/xwrf_mux.vhd",

    # Timing Modules.
    "wr-cores/modules/timing/dmtd_phase_meas.vhd",
    "wr-cores/modules/timing/dmtd_sampler.vhd",
    "wr-cores/modules/timing/dmtd_with_deglitcher.vhd",
    "wr-cores/modules/timing/pulse_stamper.vhd",
    "wr-cores/modules/timing/pulse_stamper_sync.vhd",

    # WR Endpoint Modules.
    "wr-cores/modules/wr_endpoint/endpoint_pkg.vhd",
    "wr-cores/modules/wr_endpoint/endpoint_private_pkg.vhd",
    "wr-cores/modules/wr_endpoint/ep_1000basex_pcs.vhd",
    "wr-cores/modules/wr_endpoint/ep_autonegotiation.vhd",
    "wr-cores/modules/wr_endpoint/ep_clock_alignment_fifo.vhd",
    "wr-cores/modules/wr_endpoint/ep_crc32_pkg.vhd",
    "wr-cores/modules/wr_endpoint/ep_leds_controller.vhd",
    "wr-cores/modules/wr_endpoint/ep_mdio_regs.vhd",
    "wr-cores/modules/wr_endpoint/ep_packet_filter.vhd",
    "wr-cores/modules/wr_endpoint/ep_registers_pkg.vhd",
    "wr-cores/modules/wr_endpoint/ep_rtu_header_extract.vhd",
    "wr-cores/modules/wr_endpoint/ep_rx_buffer.vhd",
    "wr-cores/modules/wr_endpoint/ep_rx_crc_size_check.vhd",
    "wr-cores/modules/wr_endpoint/ep_rx_early_address_match.vhd",
    "wr-cores/modules/wr_endpoint/ep_rx_oob_insert.vhd",
    "wr-cores/modules/wr_endpoint/ep_rx_path.vhd",
    "wr-cores/modules/wr_endpoint/ep_rx_pcs_16bit.vhd",
    "wr-cores/modules/wr_endpoint/ep_rx_pcs_8bit.vhd",
    "wr-cores/modules/wr_endpoint/ep_rx_status_reg_insert.vhd",
    "wr-cores/modules/wr_endpoint/ep_rx_vlan_unit.vhd",
    "wr-cores/modules/wr_endpoint/ep_rx_wb_master.vhd",
    "wr-cores/modules/wr_endpoint/ep_sync_detect.vhd",
    "wr-cores/modules/wr_endpoint/ep_sync_detect_16bit.vhd",
    "wr-cores/modules/wr_endpoint/ep_timestamping_unit.vhd",
    "wr-cores/modules/wr_endpoint/ep_ts_counter.vhd",
    "wr-cores/modules/wr_endpoint/ep_tx_crc_inserter.vhd",
    "wr-cores/modules/wr_endpoint/ep_tx_header_processor.vhd",
    "wr-cores/modules/wr_endpoint/ep_tx_inject_ctrl.vhd",
    "wr-cores/modules/wr_endpoint/ep_tx_packet_injection.vhd",
    "wr-cores/modules/wr_endpoint/ep_tx_path.vhd",
    "wr-cores/modules/wr_endpoint/ep_tx_pcs_16bit.vhd",
    "wr-cores/modules/wr_endpoint/ep_tx_pcs_8bit.vhd",
    "wr-cores/modules/wr_endpoint/ep_tx_vlan_unit.vhd",
    "wr-cores/modules/wr_endpoint/ep_wishbone_controller.vhd",
    "wr-cores/modules/wr_endpoint/wr_endpoint.vhd",

    # WR Mini NIC Modules.
    "wr-cores/modules/wr_endpoint/xwr_endpoint.vhd",
    "wr-cores/modules/wr_mini_nic/minic_wb_slave.vhd",
    "wr-cores/modules/wr_mini_nic/minic_wbgen2_pkg.vhd",
    "wr-cores/modules/wr_mini_nic/wr_mini_nic.vhd",
    "wr-cores/modules/wr_mini_nic/xwr_mini_nic.vhd",

    # WR PPS Modules.
    "wr-cores/modules/wr_pps_gen/pps_gen_wb.vhd",
    "wr-cores/modules/wr_pps_gen/wr_pps_gen.vhd",
    "wr-cores/modules/wr_pps_gen/xwr_pps_gen.vhd",

    # WR SoftPLL Modules.
    "wr-cores/modules/wr_softpll_ng/softpll_pkg.vhd",
    "wr-cores/modules/wr_softpll_ng/spll_aligner.vhd",
    "wr-cores/modules/wr_softpll_ng/spll_wb_slave.vhd",
    "wr-cores/modules/wr_softpll_ng/spll_wbgen2_pkg.vhd",
    "wr-cores/modules/wr_softpll_ng/wr_softpll_ng.vhd",
    "wr-cores/modules/wr_softpll_ng/xwr_softpll_ng.vhd",

    # WR Streamers Modules.
    "wr-cores/modules/wr_streamers/dropping_buffer.vhd",
    "wr-cores/modules/wr_streamers/escape_detector.vhd",
    "wr-cores/modules/wr_streamers/escape_inserter.vhd",
    "wr-cores/modules/wr_streamers/fifo_showahead_adapter.vhd",
    "wr-cores/modules/wr_streamers/fixed_latency_delay.vhd",
    "wr-cores/modules/wr_streamers/fixed_latency_ts_match.vhd",
    "wr-cores/modules/wr_streamers/streamers_pkg.vhd",
    "wr-cores/modules/wr_streamers/streamers_priv_pkg.vhd",
    "wr-cores/modules/wr_streamers/ts_restore_tai.vhd",
    "wr-cores/modules/wr_streamers/wr_streamers_wb.vhd",
    "wr-cores/modules/wr_streamers/wr_streamers_wbgen2_pkg.vhd",
    "wr-cores/modules/wr_streamers/xrtx_streamers_stats.vhd",
    "wr-cores/modules/wr_streamers/xrx_streamer.vhd",
    "wr-cores/modules/wr_streamers/xrx_streamers_stats.vhd",
    "wr-cores/modules/wr_streamers/xtx_streamer.vhd",
    "wr-cores/modules/wr_streamers/xtx_streamers_stats.vhd",
    "wr-cores/modules/wr_streamers/xwr_streamers.vhd",
    "wr-cores/modules/wr_tbi_phy/disparity_gen_pkg.vhd",

    # WR Core Modules.
    #"wr-cores/modules/wrc_core/wr_core.vhd",
    "wr-cores/modules/wrc_core/wrc_cpu_csr_wb.vhd",
    "wr-cores/modules/wrc_core/wrc_cpu_csr_wbgen2_pkg.vhd",
    "wr-cores/modules/wrc_core/wrc_diags_dpram.vhd",
    "wr-cores/modules/wrc_core/wrc_periph.vhd",
    "wr-cores/modules/wrc_core/wrc_syscon_pkg.vhd",
    "wr-cores/modules/wrc_core/wrc_syscon_wb.vhd",
    "wr-cores/modules/wrc_core/wrc_urv_wrapper.vhd",
    "wr-cores/modules/wrc_core/wrcore_pkg.vhd",
    "wr-cores/modules/wrc_core/xwr_core.vhd",

    # WR PHY Modules.
    #"wr-cores/platform/xilinx/wr_gtp_phy/family7-gtp/whiterabbit_gtpe2_channel_wrapper.vhd",
    #"wr-cores/platform/xilinx/wr_gtp_phy/family7-gtp/whiterabbit_gtpe2_channel_wrapper_gt.vhd",
    #"wr-cores/platform/xilinx/wr_gtp_phy/family7-gtp/wr_gtp_phy_family7.vhd",
    "wr-cores/platform/xilinx/wr_gtp_phy/family7-gtp/whiterabbit_gtpe2_channel_wrapper_gtrxreset_seq.vhd",
    "wr-cores/platform/xilinx/wr_gtp_phy/gtp_bitslide.vhd",
    "wr-cores/platform/xilinx/wr_xilinx_pkg.vhd",

    # LiteX-WR NIC adapted files.
    # ----------------------------------------------------------------------------------------------

    # WR PHY Modules.
    "gateware/wr-cores/platform/xilinx/wr_gtp_phy/family7-gtp/whiterabbit_gtpe2_channel_wrapper.vhd",
    "gateware/wr-cores/platform/xilinx/wr_gtp_phy/family7-gtp/whiterabbit_gtpe2_channel_wrapper_gt.vhd",
    "gateware/wr-cores/platform/xilinx/wr_gtp_phy/family7-gtp/wr_gtp_phy_family7.vhd",

    # WR Fabric Modules.
    "gateware/wr-cores/modules/fabric/xwrf_mux.vhd",

    # WR Core Modules.
    "gateware/wr-cores/modules/wrc_core/wr_core.vhd",

    # WR Platform.
    "gateware/wr-cores/platform/xilinx/xwrc_platform_vivado.vhd",

    # WR Board.
    "gateware/wr-cores/board/spec_a7/xwrc_board_spec_a7.vhd",
    "gateware/wr-cores/board/spec_a7/xwrc_board_spec_a7_wrapper.vhd",

    # WR MMCM phase shift
    "gateware/wr-cores/modules/ps_gen/ps_gen.vhdl"
]
