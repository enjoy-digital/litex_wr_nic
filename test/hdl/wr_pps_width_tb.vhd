-- Equivalence check for the pulse-width transformation applied to
-- xwr_pps_gen: the wide "counter is zero" comparison is replaced by a
-- registered flag. Both versions of the process run side by side on the
-- same stimulus, including reloads while counting and a zero width.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity wr_pps_width_tb is end;

architecture test of wr_pps_width_tb is
  signal clk       : std_logic := '0';
  signal rst_n     : std_logic := '0';
  signal overflow  : std_logic := '0';
  signal pwidth    : std_logic_vector(27 downto 0) := (others => '0');
  signal valid     : std_logic := '1';

  -- Upstream version.
  signal a_pps     : std_logic;
  signal a_cntr    : unsigned(27 downto 0);
  -- Patched version.
  signal b_pps     : std_logic;
  signal b_cntr    : unsigned(27 downto 0);
  signal b_zero    : std_logic;

  signal checks    : natural := 0;
begin
  clk <= not clk after 4 ns;

  p_original : process(clk)
  begin
    if rising_edge(clk) then
      if rst_n = '0' then
        a_pps  <= '0';
        a_cntr <= (others => '0');
      else
        if overflow = '1' then
          a_pps  <= valid;
          a_cntr <= unsigned(pwidth);
        else
          if a_cntr = to_unsigned(0, a_cntr'length) then
            a_pps <= '0';
          else
            a_cntr <= a_cntr - 1;
          end if;
        end if;
      end if;
    end if;
  end process;

  p_patched : process(clk)
  begin
    if rising_edge(clk) then
      if rst_n = '0' then
        b_pps  <= '0';
        b_cntr <= (others => '0');
        b_zero <= '1';
      else
        if overflow = '1' then
          b_pps  <= valid;
          b_cntr <= unsigned(pwidth);
          if unsigned(pwidth) = 0 then
            b_zero <= '1';
          else
            b_zero <= '0';
          end if;
        else
          if b_zero = '1' then
            b_pps <= '0';
          else
            b_cntr <= b_cntr - 1;
            if b_cntr = 1 then
              b_zero <= '1';
            end if;
          end if;
        end if;
      end if;
    end if;
  end process;

  p_check : process(clk)
  begin
    if falling_edge(clk) and rst_n = '1' then
      assert a_pps = b_pps
        report "PPS output differs" severity failure;
      checks <= checks + 1;
    end if;
  end process;

  p_stimulus : process
    procedure pulse(width : natural; cycles : natural) is
    begin
      pwidth   <= std_logic_vector(to_unsigned(width, 28));
      overflow <= '1';
      wait until rising_edge(clk);
      overflow <= '0';
      for i in 1 to cycles loop
        wait until rising_edge(clk);
      end loop;
    end procedure;
  begin
    wait for 40 ns;
    rst_n <= '1';
    wait until rising_edge(clk);
    -- Ordinary widths, including the shortest and a zero-width pulse.
    pulse(0, 8);
    pulse(1, 8);
    pulse(2, 8);
    pulse(7, 20);
    -- Reload while still counting, and back-to-back overflows.
    pulse(10, 4);
    pulse(3, 10);
    pulse(5, 1);
    pulse(0, 6);
    -- An output disabled by the valid input.
    valid <= '0';
    pulse(4, 10);
    valid <= '1';
    pulse(4, 10);
    -- A long pulse, checked after reset recovery.
    pulse(1000, 1010);
    rst_n <= '0';
    wait until rising_edge(clk);
    rst_n <= '1';
    pulse(6, 12);
    assert checks > 1100
      report "Too few compared cycles: " & integer'image(checks) severity failure;
    report "PASS";
    std.env.finish;
    wait;
  end process;
end;
