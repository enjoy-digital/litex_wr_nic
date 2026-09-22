-------------------------------------------------------------------------------
-- SPDX-FileCopyrightText: 2026 Enjoy-Digital
--
-- SPDX-License-Identifier: CERN-OHL-W-2.0+
-------------------------------------------------------------------------------
-- uRV wrapper for a WRPC whose low memory is provided by the enclosing SoC.
-- Instruction and data accesses below 1 MiB share cpu_mem_o.  The existing
-- WRPC peripheral Wishbone path is retained for data accesses above 1 MiB.
--
-- cpu_mem_o is a pipelined Wishbone master: a request is issued in every cycle
-- where cpu_mem_i.stall is low and responses return in order.  A slave that
-- acknowledges each request one cycle later restores the native uRV memory
-- timing of one instruction fetch per cycle.  A slave that stalls while busy
-- limits the CPU to a single outstanding access.  Slaves must only act on
-- accepted requests (stb and not stall): a stalled request may be replaced.
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
  constant c_INSN_NOP : std_logic_vector(31 downto 0) := x"0000_0013";

  -- Accepted memory requests awaiting their in-order response.
  constant c_MAX_OUTSTANDING : natural := 4;

  signal cpu_rst : std_logic;

  signal im_addr  : std_logic_vector(31 downto 0);
  signal im_data  : std_logic_vector(31 downto 0);
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

  signal mem_request_data, mem_request, mem_full, mem_issue : std_logic;
  signal mem_response, mem_response_data                    : std_logic;
  signal mem_outstanding : unsigned(2 downto 0);
  -- Request types in issue order, oldest first; '1' marks a data access.
  signal mem_queue       : std_logic_vector(c_MAX_OUTSTANDING-1 downto 0);
  signal mem_out         : t_wishbone_master_out;

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
      im_rd_o          => open,
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

  -- uRV pulses load/store for one cycle, independently of instruction fetches.
  -- Preserve the complete request while the selected bus path is occupied.
  -- The CPU stalls until completion, so one pending entry is sufficient.
  dm_accept <= '1' when dm_pending = '1' and
    ((dm_is_wishbone = '0' and mem_issue = '1') or
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

  -- Shared external instruction/data memory path.  A pending data access is
  -- issued before the fetch of the cycle, which the CPU then repeats.  As with
  -- the native uRV instruction port, the program counter is fetched in every
  -- other cycle: a stalled pipeline keeps its address, so any later response
  -- remains valid and no bubble follows the stall.
  mem_request_data <= dm_pending and not dm_is_wishbone;
  mem_request      <= not cpu_rst;
  mem_full         <= '1' when mem_outstanding = c_MAX_OUTSTANDING else '0';
  mem_issue        <= mem_request and not mem_full and not cpu_mem_i.stall;
  mem_response     <= (cpu_mem_i.ack or cpu_mem_i.err or cpu_mem_i.rty)
                      when mem_outstanding /= 0 else '0';

  mem_out.cyc <= '1' when cpu_rst = '0' and (mem_outstanding /= 0 or mem_request = '1') else '0';
  mem_out.stb <= mem_request and not mem_full;
  mem_out.adr <= dm_request_addr   when mem_request_data = '1' else im_addr;
  mem_out.we  <= dm_write          when mem_request_data = '1' else '0';
  mem_out.sel <= dm_request_select when mem_request_data = '1' else "1111";
  mem_out.dat <= dm_request_data;

  p_memory_queue : process(clk_sys_i)
    variable v_queue : std_logic_vector(mem_queue'range);
    variable v_count : unsigned(mem_outstanding'range);
  begin
    if rising_edge(clk_sys_i) then
      if cpu_rst = '1' then
        mem_queue       <= (others => '0');
        mem_outstanding <= (others => '0');
      else
        v_queue := mem_queue;
        v_count := mem_outstanding;
        if mem_response = '1' then
          v_queue := '0' & v_queue(v_queue'high downto 1);
          v_count := v_count - 1;
        end if;
        if mem_issue = '1' then
          v_queue(to_integer(v_count)) := mem_request_data;
          v_count                      := v_count + 1;
        end if;
        mem_queue       <= v_queue;
        mem_outstanding <= v_count;
      end if;
    end if;
  end process;

  -- Completion does not depend combinationally on dm_addr: the CPU can derive
  -- dm_addr from load_done, so address-based response muxing forms a loop.
  mem_response_data <= mem_response and mem_queue(0);
  im_valid          <= mem_response and not mem_queue(0);
  im_data           <= cpu_mem_i.dat when cpu_mem_i.ack = '1' else c_INSN_NOP;
  dm_load_done      <= dm_hi_load_done or (mem_response_data and not dm_write);
  dm_store_done     <= dm_hi_store_done or (mem_response_data and dm_write);
  dm_data_l         <= dm_hi_rdata when dm_hi_load_done = '1' else
                       cpu_mem_i.dat when cpu_mem_i.ack = '1' else (others => '0');
end architecture arch;
