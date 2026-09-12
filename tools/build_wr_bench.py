#!/usr/bin/env python3
"""Build a pinned uRV/private-RAM WR profile with read-only storage."""

import argparse
import hashlib
import importlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
BOARDS = {
    'acorn': ('acorn_wr_nic.py', 'acorn', 'sqrl_acorn.bit'),
    'spec': ('spec_a7_wr_nic.py', 'spec_a7', 'spec_a7_wr_nic.bin'),
    'hyvision': ('hyvision_pcie_opt01_revf.py', 'hyvision', 'hyvision_pcie_opt01_revf.bit'),
}


def git(path, *args):
    return subprocess.check_output(['git', '-C', str(path), *args], text=True).strip()


def finish_build(board, output):
    # Accept older target outputs too. SPEC requires the converted 35T image.
    if not (output / 'csr.csv').is_file():
        shutil.copy2(ROOT / 'test/csr.csv', output / 'csr.csv')
    image = output / 'gateware' / BOARDS[board][2]
    if board == 'spec':
        subprocess.run([sys.executable, str(ROOT / 'litex_wr_nic/gateware/xilinx-bitstream.py'),
            str(image.with_suffix('.bit')), str(image)], cwd=ROOT, check=True)
    if not image.is_file() or not image.stat().st_size:
        raise RuntimeError('Missing or empty programming image: ' + str(image))
    timing = next((output / 'gateware').glob('*_timing.rpt')).read_text()
    if 'All user specified timing constraints are met.' not in timing:
        raise RuntimeError('Vivado timing failed; inspect the report before loading')
    return {str(p.relative_to(output)): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in output.rglob('*') if p.is_file() and p.suffix in ('.bit', '.bin', '.bram', '.boot', '.csv')}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--board', required=True, choices=BOARDS)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--wr-refclk-tuning', choices=['mmcm', 'txpi'], default='mmcm',
        help='Acorn main-clock actuator; TXPI is supported only on its Artix-7 GTP.')
    parser.add_argument('--pll-trace-decimation', type=int, default=0,
        help='Enable PLL trace FIFO, main/helper trace decimation and FPGA sensors (power of two, 1..1024).')
    args = parser.parse_args()
    if args.pll_trace_decimation < 0 or args.pll_trace_decimation > 1024 or args.pll_trace_decimation & (args.pll_trace_decimation - 1):
        parser.error('--pll-trace-decimation must be 0 or a power of two from 1 to 1024')
    if args.wr_refclk_tuning == 'txpi' and args.board != 'acorn':
        parser.error('--wr-refclk-tuning txpi requires --board acorn')
    pins = json.loads((ROOT / 'doc/wr_bench_sources.json').read_text())
    for name, pin in pins['python'].items():
        module = importlib.import_module(name)
        path = Path(module.__file__).resolve().parent
        if git(path, 'rev-parse', 'HEAD') != pin['revision'] or git(path, 'status', '--porcelain', '--untracked-files=no'):
            raise RuntimeError(f'{name}: expected clean source {pin["revision"]}, loaded {path}')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    manifest = dict(board=args.board, source=git(ROOT, 'rev-parse', 'HEAD'),
        source_dirty=bool(git(ROOT, 'status', '--porcelain', '--untracked-files=no')),
        dependencies=pins, profile='uRV/private 128 KiB; read-only SPI storage',
        pll_trace_decimation=args.pll_trace_decimation,
        refclk_tuning=args.wr_refclk_tuning if args.board == 'acorn' else ('dac' if args.board == 'spec' else 'mmcm'),
        pcie_nic=not (args.board == 'spec' and args.pll_trace_decimation), passed=False)
    def run(command):
        subprocess.run([sys.executable, *command], cwd=ROOT, check=True)
    try:
        run(['litex_wr_nic/firmware/build.py', '--target',
            BOARDS[args.board][1], '--wr-cpu-type', 'urv', '--read-only-storage',
            '--pll-trace-decimation', str(args.pll_trace_decimation)])
        firmware = ROOT / 'litex_wr_nic/firmware'
        for ext in ('bram', 'bin', 'boot'):
            shutil.copy2(firmware / ('spec_a7_wrc.' + ext), output / ('spec_a7_wrc.' + ext))
        command = [BOARDS[args.board][0],
            '--build', '--skip-firmware-build', '--skip-software-headers', '--wr-cpu-type', 'urv',
            '--wr-cpu-memory', 'private', '--output-dir', str(output)]
        if args.board == 'acorn':
            command += ['--wr-refclk-tuning', args.wr_refclk_tuning]
        if args.pll_trace_decimation:
            command.append('--with-wr-pll-debug')
        run(command)
        manifest['sha256'] = finish_build(args.board, output)
        manifest['passed'] = True
    finally:
        (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')


if __name__ == '__main__':
    main()
