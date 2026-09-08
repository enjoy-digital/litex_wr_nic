library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.wishbone_pkg.all;

entity urv_external_tb is end;
architecture test of urv_external_tb is
  signal clk          : std_logic := '0';
  signal rst_n        : std_logic := '0';
  signal mem_o, per_o : t_wishbone_master_out;
  signal mem_i, per_i : t_wishbone_master_in := cc_dummy_master_in;
  signal host_i       : t_wishbone_slave_in := cc_dummy_slave_in;
  signal host_o       : t_wishbone_slave_out;

  type t_words is array(0 to 31) of std_logic_vector(31 downto 0);
  constant rom : t_words := (
    0      => x"0100006f", -- j 0x10, like the WRPC reset vector
    4      => x"000012b7", -- lui t0,1
    5      => x"12300313", -- li t1,0x123
    6      => x"0062a023", -- sw t1,0(t0)
    7      => x"00130313", -- addi t1,t1,1
    8      => x"0062a223", -- sw t1,4(t0)
    9      => x"0002a383", -- lw t2,0(t0)
    10     => x"0072a423", -- sw t2,8(t0)
    11     => x"001002b7", -- lui t0,0x100
    12     => x"0062a023", -- sw t1,0(t0), peripheral
    13     => x"0062a223", -- sw t1,4(t0), peripheral
    14     => x"0000006f", -- j .
    others => x"00000013");
  signal memory                    : t_words := (others => (others => '0'));
  signal writes, peripheral_writes : natural := 0;
begin
  clk   <= not clk after 8 ns;
  rst_n <= '1' after 100 ns;

  dut : entity work.wrc_urv_external_memory
    generic map (
      g_CPU_ID => 0)
    port map (
      clk_sys_i      => clk,
      rst_n_i        => rst_n,
      irq_i          => '0',
      memory_ready_i => '1',
      cpu_mem_o      => mem_o,
      cpu_mem_i      => mem_i,
      dwb_o          => per_o,
      dwb_i          => per_i,
      host_slave_i   => host_i,
      host_slave_o   => host_o);

  process(clk)
    variable remaining : integer := 0;
    variable active    : boolean := false;
    variable request   : t_wishbone_master_out;
    variable addr      : natural;
  begin
    if rising_edge(clk) then
      mem_i.ack   <= '0';
      mem_i.err   <= '0';
      mem_i.rty   <= '0';
      mem_i.stall <= '0';
      if remaining > 0 then
        remaining := remaining - 1;
        if remaining = 0 then
          addr := to_integer(unsigned(request.adr));
          if request.we = '1' then
            assert addr >= 4096 and addr < 4224 severity failure;
            memory((addr - 4096)/4) <= request.dat;
            writes                 <= writes + 1;
            report "MEM WRITE " & to_hstring(request.adr) & " = " & to_hstring(request.dat);
          elsif addr < 128 then
            mem_i.dat <= rom(addr/4);
            report "INSN " & to_hstring(request.adr) & " = " & to_hstring(rom(addr/4));
          elsif addr >= 4096 and addr < 4224 then
            mem_i.dat <= memory((addr - 4096)/4);
          else
            report "Unexpected address " & to_hstring(request.adr) severity failure;
          end if;
          mem_i.ack <= '1';
        end if;
      elsif active then
        if mem_o.cyc = '0' then
          active := false;
        end if;
      elsif mem_o.cyc = '1' and mem_o.stb = '1' then
        request   := mem_o;
        remaining := 4;
        active    := true;
      end if;
      per_i.ack   <= '0';
      per_i.err   <= '0';
      per_i.rty   <= '0';
      per_i.stall <= '0';
      per_i.dat   <= (others => '0');
      if per_o.cyc = '1' and per_o.stb = '1' then
        per_i.ack <= '1';
        if per_o.we = '1' then
          peripheral_writes <= peripheral_writes + 1;
        end if;
      end if;
    end if;
  end process;

  process
  begin
    wait for 20 us;
    assert writes = 3
      report "Expected 3 memory stores, observed " & integer'image(writes) severity failure;
    assert memory(0) = x"00000123" and memory(1) = x"00000124" and memory(2) = x"00000123"
      severity failure;
    assert peripheral_writes = 2
      report "Expected 2 peripheral stores, observed " & integer'image(peripheral_writes)
      severity failure;
    report "PASS";
    std.env.finish;
    wait;
  end process;
end;
