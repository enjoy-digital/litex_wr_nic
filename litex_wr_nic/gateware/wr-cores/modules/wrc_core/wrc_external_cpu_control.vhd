-------------------------------------------------------------------------------
-- SPDX-FileCopyrightText: 2026 Enjoy-Digital
--
-- SPDX-License-Identifier: CERN-OHL-W-2.0+
-------------------------------------------------------------------------------
-- Keep the WRPC host CPU-control register block when the CPU itself is
-- instantiated by LiteX. Only RESET remains meaningful in this mode.
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;

use work.wishbone_pkg.all;
use work.wrc_cpu_csr_pkg.all;

entity wrc_external_cpu_control is
  port(
    clk_sys_i    : in  std_logic;
    rst_n_i      : in  std_logic;
    reset_o      : out std_logic;
    host_slave_i : in  t_wishbone_slave_in;
    host_slave_o : out t_wishbone_slave_out);
end wrc_external_cpu_control;

architecture arch of wrc_external_cpu_control is
  signal regs_in  : t_wrc_cpu_csr_regs_master_out;
  signal regs_out : t_wrc_cpu_csr_regs_master_in;
begin
  U_CPU_CSR : entity work.wrc_cpu_csr
    port map (
      rst_n_i            => rst_n_i,
      clk_i              => clk_sys_i,
      wb_i               => host_slave_i,
      wb_o               => host_slave_o,
      wrc_cpu_csr_regs_i => regs_out,
      wrc_cpu_csr_regs_o => regs_in);

  reset_o                 <= regs_in.reset(0);
  regs_out.udata          <= (others => '0');
  regs_out.dbg_status     <= (others => '0');
  regs_out.dbg_insn_ready <= (others => '0');
  regs_out.dbg_core0_mbx  <= (others => '0');
end architecture arch;
