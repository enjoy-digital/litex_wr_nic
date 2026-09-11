#!/usr/bin/env python3
"""Build the pinned uRV/private-RAM USB bench profile with read-only storage."""

import argparse
import hashlib
import importlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def git(path, *args):
    return subprocess.check_output(['git', '-C', str(path), *args], text=True).strip()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--board', required=True, choices=['acorn', 'spec'])
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
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
        dependencies=pins, profile='uRV/private 128 KiB; read-only SPI storage', passed=False)
    def run(command):
        subprocess.run([sys.executable, *command], cwd=ROOT, check=True)
    try:
        run(['litex_wr_nic/firmware/build.py', '--target',
            'acorn' if args.board == 'acorn' else 'spec_a7', '--wr-cpu-type', 'urv', '--read-only-storage'])
        firmware = ROOT / 'litex_wr_nic/firmware'
        for ext in ('bram', 'bin', 'boot'):
            shutil.copy2(firmware / ('spec_a7_wrc.' + ext), output / ('spec_a7_wrc.' + ext))
        command = ['acorn_wr_nic.py' if args.board == 'acorn' else 'spec_a7_wr_nic.py',
            '--build', '--skip-firmware-build', '--skip-software-headers', '--wr-cpu-type', 'urv',
            '--wr-cpu-memory', 'private', '--output-dir', str(output)]
        run(command)
        if args.board == 'acorn':
            shutil.copy2(ROOT / 'test/csr.csv', output / 'csr.csv')
        manifest['sha256'] = {str(p.relative_to(output)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in output.rglob('*') if p.is_file() and p.suffix in ('.bit', '.bin', '.bram', '.boot', '.csv')}
        timing = next((output / 'gateware').glob('*_timing.rpt')).read_text()
        if 'All user specified timing constraints are met.' not in timing:
            raise RuntimeError('Vivado timing failed; inspect the report before loading')
        manifest['passed'] = True
    finally:
        (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')


if __name__ == '__main__':
    main()
