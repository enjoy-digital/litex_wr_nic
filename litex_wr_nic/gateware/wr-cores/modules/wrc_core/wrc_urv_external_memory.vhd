-------------------------------------------------------------------------------
-- SPDX-FileCopyrightText: 2026 Enjoy-Digital
--
-- SPDX-License-Identifier: CERN-OHL-W-2.0+
-------------------------------------------------------------------------------
-- uRV wrapper for a WRPC whose low memory is provided by the enclosing SoC.
-- Instruction and data accesses below 1 MiB share cpu_mem_o.  The existing
-- WRPC peripheral Wishbone path is retained for data accesses above 1 MiB.
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.wishbone_pkg.all;
use work.wrc_cpu_csr_pkg.all;
use work.urv_pkg.all;

entity wrc_urv_external_memory is
  generic(
    g_CPU_ID : integer);
  port(
    clk_sys_i      : in  std_logic;
    rst_n_i        : in  std_logic;
    irq_i          : in  std_logic;
    memory_ready_i : in  std_logic;
    cpu_mem_o      : out t_wishbone_master_out;
    cpu_mem_i      : in  t_wishbone_master_in;
    dwb_o          : out t_wishbone_master_out;
    dwb_i          : in  t_wishbone_master_in;
    host_slave_i   : in  t_wishbone_slave_in;
    host_slave_o   : out t_wishbone_slave_out);
end wrc_urv_external_memory;

architecture arch of wrc_urv_external_memory is
  type t_mem_state is (MEM_IDLE, MEM_ACCESS_INSN, MEM_RESP_INSN,
    MEM_ACCESS_DATA, MEM_RESP_DATA, MEM_WAIT_DATA);

  constant c_INSN_NOP : std_logic_vector(31 downto 0) := x"0000_0013";

  signal cpu_rst : std_logic;

  signal im_addr  : std_logic_vector(31 downto 0);
  signal im_data  : std_logic_vector(31 downto 0);
  signal im_read  : std_logic;
  signal im_valid : std_logic;

  signal dm_addr, dm_data_s, dm_data_l                  : std_logic_vector(31 downto 0);
  signal dm_data_select                                 : std_logic_vector(3 downto 0);
  signal dm_load, dm_store, dm_load_done, dm_store_done : std_logic;
  signal dm_is_wishbone                                 : std_logic;
  signal dm_pending, dm_accept, dm_write                : std_logic;
  signal dm_request_addr, dm_request_data               : std_logic_vector(31 downto 0);
  signal dm_request_select                              : std_logic_vector(3 downto 0);

  signal dm_hi_cycle, dm_hi_write, dm_hi_wait : std_logic;
  signal dm_hi_load_done, dm_hi_store_done    : std_logic;
  signal dm_hi_rdata                          : std_logic_vector(31 downto 0);
  signal dwb_out                              : t_wishbone_master_out;

  signal mem_state                      : t_mem_state;
  signal mem_out                        : t_wishbone_master_out;
  signal mem_data_write                 : std_logic;
  signal mem_insn_rdata, mem_data_rdata : std_logic_vector(31 downto 0);

  signal dbg_insn : std_logic_vector(31 downto 0);
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

  -- The legacy UADDR/UDATA window addresses the removed private RAM.  Keep the
  -- registers readable and harmless; SoC-side access uses wr_cpu_mem instead.
  regs_out.udata <= (others => '0');

  U_CPU : urv_cpu
    generic map (
      g_with_hw_debug => 1,
      g_with_hw_mulh  => 1,
      g_with_hw_mul   => 1,
      g_with_hw_div   => 1)
    port map (
      clk_i            => clk_sys_i,
      rst_i            => cpu_rst,
      irq_i            => irq_i,
      im_addr_o        => im_addr,
      im_data_i        => im_data,
      im_valid_i       => im_valid,
      im_rd_o          => im_read,
      dm_addr_o        => dm_addr,
      dm_data_s_o      => dm_data_s,
      dm_data_l_i      => dm_data_l,
      dm_data_select_o => dm_data_select,
      dm_store_o       => dm_store,
      dm_load_o        => dm_load,
      dm_load_done_i   => dm_load_done,
      dm_store_done_i  => dm_store_done,
      fault_o          => open,
      dbg_force_i      => regs_in.dbg_force(0),
      dbg_enabled_o    => regs_out.dbg_status(0),
      dbg_insn_i       => dbg_insn,
      dbg_insn_set_i   => regs_in.dbg_core0_insn_wr,
      dbg_insn_ready_o => regs_out.dbg_insn_ready(0),
      dbg_mbx_data_i   => regs_in.dbg_core0_mbx,
      dbg_mbx_write_i  => regs_in.dbg_core0_mbx_wr,
      dbg_mbx_data_o   => regs_out.dbg_core0_mbx);

  cpu_rst        <= not rst_n_i or regs_in.reset(0) or not memory_ready_i;
  dm_is_wishbone <= '1' when dm_request_addr(31 downto 20) /= x"000" else '0';
  dwb_o          <= dwb_out;
  cpu_mem_o      <= mem_out;

  -- Completion does not depend combinationally on dm_addr: the CPU can derive
  -- dm_addr from load_done, so address-based response muxing forms a loop.
  dm_load_done <= '1' when dm_hi_load_done = '1' or
    (mem_state = MEM_RESP_DATA and mem_data_write = '0') else '0';
  dm_store_done <= '1' when dm_hi_store_done = '1' or
    (mem_state = MEM_RESP_DATA and mem_data_write = '1') else '0';
  dm_data_l <= dm_hi_rdata when dm_hi_load_done = '1' else mem_data_rdata;
  im_data   <= mem_insn_rdata;
  im_valid  <= '1' when mem_state = MEM_RESP_INSN else '0';

  -- uRV pulses load/store for one cycle, independently of instruction fetches.
  -- Preserve the complete request while the selected bus path is occupied.
  -- The CPU stalls until completion, so one pending entry is sufficient.
  dm_accept <= '1' when dm_pending = '1' and
    ((dm_is_wishbone = '0' and mem_state = MEM_IDLE) or
     (dm_is_wishbone = '1' and dm_hi_cycle = '0' and dm_hi_wait = '0')) else '0';

  p_data_request : process(clk_sys_i)
  begin
    if rising_edge(clk_sys_i) then
      if cpu_rst = '1' then
        dm_pending        <= '0';
        dm_write          <= '0';
        dm_request_addr   <= (others => '0');
        dm_request_data   <= (others => '0');
        dm_request_select <= (others => '0');
      else
        if dm_accept = '1' then
          dm_pending <= '0';
        end if;
        if dm_load = '1' or dm_store = '1' then
          dm_pending        <= '1';
          dm_write          <= dm_store;
          dm_request_addr   <= dm_addr;
          dm_request_data   <= dm_data_s;
          dm_request_select <= dm_data_select;
        end if;
      end if;
    end if;
  end process;

  p_debug_instruction : process(clk_sys_i)
  begin
    if rising_edge(clk_sys_i) then
      if rst_n_i = '0' then
        dbg_insn <= c_INSN_NOP;
      elsif regs_in.dbg_core0_insn_wr = '1' then
        dbg_insn <= regs_in.dbg_core0_insn;
      else
        dbg_insn <= c_INSN_NOP;
      end if;
    end if;
  end process;

  -- Existing WRPC peripheral path (data addresses >= 1 MiB).
  p_peripheral_wishbone : process(clk_sys_i)
  begin
    if rising_edge(clk_sys_i) then
      dm_hi_load_done  <= '0';
      dm_hi_store_done <= '0';
      if cpu_rst = '1' then
        dwb_out.cyc <= '0';
        dwb_out.stb <= '0';
        dwb_out.adr <= (others => '0');
        dwb_out.sel <= (others => '0');
        dwb_out.we  <= '0';
        dwb_out.dat <= (others => '0');
        dm_hi_cycle <= '0';
        dm_hi_write <= '0';
        dm_hi_wait  <= '0';
        dm_hi_rdata <= (others => '0');
      elsif dm_hi_wait = '1' then
        -- A following uRV request can arrive while completion is sampled.
        -- The pending register retains it until this path is available.
        if dm_load = '0' and dm_store = '0' then
          dm_hi_wait <= '0';
        end if;
      elsif dm_hi_cycle = '0' then
        if dm_is_wishbone = '1' and dm_pending = '1' then
          dwb_out.cyc <= '1';
          dwb_out.stb <= '1';
          dwb_out.adr <= dm_request_addr;
          dwb_out.sel <= dm_request_select;
          dwb_out.we  <= dm_write;
          dwb_out.dat <= dm_request_data;
          dm_hi_write <= dm_write;
          dm_hi_cycle <= '1';
        end if;
      else
        if dwb_i.stall = '0' then
          dwb_out.stb <= '0';
        end if;
        if dwb_i.ack = '1' or dwb_i.err = '1' or dwb_i.rty = '1' then
          dwb_out.cyc <= '0';
          dm_hi_cycle <= '0';
          dm_hi_wait  <= '1';
          if dm_hi_write = '1' then
            dm_hi_store_done <= '1';
          else
            dm_hi_load_done <= '1';
            if dwb_i.ack = '1' then
              dm_hi_rdata <= dwb_i.dat;
            else
              dm_hi_rdata <= (others => '0');
            end if;
          end if;
        end if;
      end if;
    end if;
  end process;

  -- Shared external instruction/data memory path. Data accesses are selected
  -- first whenever both interfaces request a transaction.
  p_external_memory : process(clk_sys_i)
  begin
    if rising_edge(clk_sys_i) then
      if cpu_rst = '1' then
        mem_state      <= MEM_IDLE;
        mem_out.cyc    <= '0';
        mem_out.stb    <= '0';
        mem_out.adr    <= (others => '0');
        mem_out.sel    <= (others => '0');
        mem_out.we     <= '0';
        mem_out.dat    <= (others => '0');
        mem_data_write <= '0';
        mem_insn_rdata <= c_INSN_NOP;
        mem_data_rdata <= (others => '0');
      else
        case mem_state is
          when MEM_IDLE =>
            if dm_is_wishbone = '0' and dm_pending = '1' then
              mem_out.cyc    <= '1';
              mem_out.stb    <= '1';
              mem_out.adr    <= dm_request_addr;
              mem_out.sel    <= dm_request_select;
              mem_out.we     <= dm_write;
              mem_out.dat    <= dm_request_data;
              mem_data_write <= dm_write;
              mem_state      <= MEM_ACCESS_DATA;
            elsif im_read = '1' then
              mem_out.cyc <= '1';
              mem_out.stb <= '1';
              mem_out.adr <= im_addr;
              mem_out.sel <= "1111";
              mem_out.we  <= '0';
              mem_out.dat <= (others => '0');
              mem_state   <= MEM_ACCESS_INSN;
            end if;

          when MEM_ACCESS_INSN =>
            if cpu_mem_i.stall = '0' then
              mem_out.stb <= '0';
            end if;
            if cpu_mem_i.ack = '1' or cpu_mem_i.err = '1' or cpu_mem_i.rty = '1' then
              mem_out.cyc <= '0';
              if cpu_mem_i.ack = '1' then
                mem_insn_rdata <= cpu_mem_i.dat;
              else
                mem_insn_rdata <= c_INSN_NOP;
              end if;
              mem_state <= MEM_RESP_INSN;
            end if;

          when MEM_RESP_INSN =>
            mem_state <= MEM_IDLE;

          when MEM_ACCESS_DATA =>
            if cpu_mem_i.stall = '0' then
              mem_out.stb <= '0';
            end if;
            if cpu_mem_i.ack = '1' or cpu_mem_i.err = '1' or cpu_mem_i.rty = '1' then
              mem_out.cyc <= '0';
              if cpu_mem_i.ack = '1' then
                mem_data_rdata <= cpu_mem_i.dat;
              else
                mem_data_rdata <= (others => '0');
              end if;
              mem_state <= MEM_RESP_DATA;
            end if;

          when MEM_RESP_DATA =>
            mem_state <= MEM_WAIT_DATA;

          when MEM_WAIT_DATA =>
            if dm_load = '0' and dm_store = '0' then
              mem_state <= MEM_IDLE;
            end if;
        end case;
      end if;
    end if;
  end process;
end architecture arch;
