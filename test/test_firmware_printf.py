"""Exercise the actual upstream full formatter with the project's buffer size."""
import os
from pathlib import Path
import re
import shutil
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]


def test_console_print_buffer_fits_version_and_monitor_lines(tmp_path):
    firmware = ROOT / 'litex_wr_nic/firmware'
    upstream = firmware / 'wrpc-sw'
    if not (upstream / 'pp_printf/printf.c').exists() or not shutil.which('cc'):
        pytest.skip('Upstream firmware sources and a host compiler are required')
    configured = int(re.search(r'^CONFIG_CUSTOM_PRINT_BUFSIZE=(\d+)$',
        (firmware / 'spec_a7_defconfig').read_text(), re.M)[1])
    main = tmp_path / 'main.c'
    main.write_text('''
#include <stdint.h>
#include <pp-printf.h>
uint32_t __div64_32(uint64_t *n, uint32_t base) {
    uint32_t rem = *n % base; *n /= base; return rem;
}
int main(void) {
    pp_printf("WR Core build: %s (%s)\\n", "13527cd6-dirty", "unsupported developer build");
    pp_printf("Built for RISCV, %u kB RAM, stack is %u bytes\\n", 128, 2048);
    pp_printf("%-9s/%-10s/%-7s", "MASTER", "IDLE", "EXT_ON");
    pp_printf("%20lld ps", (long long)-123456789012345LL);
    return 0;
}
''')
    env = dict(os.environ, ASAN_OPTIONS='detect_leaks=0')
    for size, must_pass in [(16, False), (configured, True)]:
        binary = tmp_path / ('printf-' + str(size))
        subprocess.run(['cc', '-fsanitize=address', '-fno-omit-frame-pointer',
            '-no-pie', '-g', '-DCONFIG_PRINT_BUFSIZE=' + str(size),
            '-I' + str(upstream / 'pp_printf'), '-I' + str(upstream / 'include'),
            str(upstream / 'pp_printf/printf.c'), str(upstream / 'pp_printf/vsprintf-full.c'),
            str(main), '-o', str(binary)], check=True, capture_output=True)
        result = subprocess.run([str(binary)], env=env, capture_output=True, text=True)
        if must_pass:
            assert result.returncode == 0, result.stderr
            assert 'PRINTF OVF' not in result.stdout
            assert '128 kB RAM' in result.stdout and 'EXT_ON' in result.stdout
        else:
            assert result.returncode != 0 and 'global-buffer-overflow' in result.stderr
