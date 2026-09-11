"""Replay the Python debug transport against the real uRV RTL in XSim."""

import importlib.util
from pathlib import Path
import shutil
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]


def test_debug_memory_access_preserves_registers_and_resumes_cpu(tmp_path):
    if any(shutil.which(tool) is None for tool in ("xvlog", "xvhdl", "xelab", "xsim")):
        pytest.skip("XSim is required for the real uRV debug protocol test")
    rtl = ROOT / "wr-cores/ip_cores/urv-core/rtl"
    if not rtl.exists():
        pytest.skip("Initialize the pinned WR-core submodules")
    spec = importlib.util.spec_from_file_location(
        "wr_urv_debug_test", ROOT / "tools/wr_urv_debug.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    class TraceBus:
        mems = type("Memories", (), {"wr_wb_slave": type("Memory", (), {"base": 0x20000000})})

        def __init__(self):
            self.trace = []
            self.debug = self.mailbox = 0
            self.registers = {10: 0x5A5, 11: 0x6B6}

        def read(self, address):
            offset = address - 0x20000B00
            value = {0: 0, 0x80: self.debug, 0x88: 1, 0x90: self.mailbox}[offset]
            self.trace.append(("read", offset, value))
            return value

        def write(self, address, value):
            offset = address - 0x20000B00
            self.trace.append(("write", offset, value))
            if offset == 0x84 and value:
                self.debug = 1
            elif offset == 0x90:
                self.mailbox = value
            elif offset == 0x8C:
                if value >> 20 == 0x7D0 and (value >> 12) & 7 == 1:
                    self.mailbox = self.registers[(value >> 15) & 31]
                elif value >> 20 == 0x7D0 and (value >> 12) & 7 == 2:
                    self.registers[(value >> 7) & 31] = self.mailbox
                elif value == 0x00052583:
                    self.registers[11] = 0xAABBCCDD
                elif value == 0x00100073:
                    self.debug = 0

    bus = TraceBus()
    with module.URVDebug(bus) as cpu:
        assert cpu.read(0x100004) == 0xAABBCCDD
        cpu.write(0x10012C, 0x80001940)
    # Verify preservation by re-entering, reading the actual registers and
    # resuming again. The simulated CPU executes the captured instructions.
    with module.URVDebug(bus) as cpu:
        assert cpu.read_register(10) == 0x5A5
        assert cpu.read_register(11) == 0x6B6

    text = (ROOT / "test/hdl/wr_urv_external_tb.vhd").read_text()
    text = text.replace(
        '14     => x"0000006f", -- j .',
        '14 => x"5a500513", -- li a0,0x5a5\n'
        '    15 => x"6b600593", -- li a1,0x6b6\n'
        '    16 => x"0000006f", -- j .',
    )
    text = text.replace(
        "signal writes, peripheral_writes : natural := 0;",
        "signal writes, peripheral_writes : natural := 0;\n"
        "  signal debug_write_seen : boolean := false;",
    )
    text = text.replace("per_i.dat   <= (others => '0');", 'per_i.dat <= x"AABBCCDD";')
    text = text.replace(
        "peripheral_writes <= peripheral_writes + 1;",
        "peripheral_writes <= peripheral_writes + 1;\n"
        '          if per_o.adr = x"0010012C" then\n'
        '            assert per_o.dat = x"80001940" severity failure;\n'
        "            debug_write_seen <= true;\n          end if;",
    )
    procedures = """
    procedure host_write(address:natural; value:std_logic_vector(31 downto 0)) is
    begin
      wait until falling_edge(clk);
      host_i.adr <= std_logic_vector(to_unsigned(address,32));
      host_i.dat <= value;host_i.sel <= "1111";
      host_i.we <= '1';host_i.cyc <= '1';host_i.stb <= '1';
      loop wait until rising_edge(clk);exit when host_o.ack='1';end loop;
      wait until falling_edge(clk);host_i <= cc_dummy_slave_in;
      wait for 320 ns;
    end;
    procedure host_read(address:natural; value:std_logic_vector(31 downto 0)) is
    begin
      wait until falling_edge(clk);
      host_i.adr <= std_logic_vector(to_unsigned(address,32));host_i.sel <= "1111";
      host_i.we <= '0';host_i.cyc <= '1';host_i.stb <= '1';
      loop wait until rising_edge(clk);exit when host_o.ack='1';end loop;
      if address=128 or address=136 then
        assert host_o.dat(0)=value(0) report "Debug handshake @" & integer'image(address) severity failure;
      else
        assert host_o.dat=value report "Debug read @" & integer'image(address) &
          " got " & to_hstring(host_o.dat) & " expected " & to_hstring(value) severity failure;
      end if;
      wait until falling_edge(clk);host_i <= cc_dummy_slave_in;
      wait for 320 ns;
    end;
"""
    text = text.replace(
        "  process\n  begin\n    wait for 20 us;",
        "  process\n" + procedures + "  begin\n    wait for 20 us;",
    )
    trace = "\n".join(
        f'    host_{op}({offset}, x"{value:08X}");' for op, offset, value in bus.trace
    )
    text = text.replace(
        '    report "PASS";',
        trace + '\n    assert debug_write_seen severity failure;\n    report "PASS";',
    )
    tb = tmp_path / "urv_debug_tb.vhd"
    tb.write_text(text)

    def run(tool, *args):
        result = subprocess.run(
            [tool, *map(str, args)], cwd=tmp_path, capture_output=True, text=True, timeout=120
        )
        output = result.stdout + result.stderr
        (tmp_path / (tool + "-output.log")).write_text(output)
        assert result.returncode == 0, output
        return output

    modules = (
        "cpu",
        "csr",
        "decode",
        "divide",
        "ecc",
        "exceptions",
        "exec",
        "fetch",
        "multiply",
        "regfile",
        "shifter",
        "timer",
        "writeback",
    )
    run("xvlog", "--work", "work", "--include", rtl, *[rtl / f"urv_{name}.v" for name in modules])
    run(
        "xvhdl",
        "--2008",
        "--work",
        "work",
        ROOT / "wr-cores/ip_cores/general-cores/modules/wishbone/wishbone_pkg.vhd",
        ROOT / "wr-cores/modules/wrc_core/wrc_cpu_csr.vhd",
        rtl / "urv_pkg.vhd",
        ROOT / "litex_wr_nic/gateware/wr-cores/modules/wrc_core/wrc_urv_external_memory.vhd",
        tb,
    )
    run("xelab", "work.urv_external_tb", "--snapshot", "urv_debug_tb")
    output = run("xsim", "urv_debug_tb", "--runall")
    assert "Note: PASS" in output and "Failure:" not in output, output
