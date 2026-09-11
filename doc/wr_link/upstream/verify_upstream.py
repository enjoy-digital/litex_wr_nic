#!/usr/bin/env python3
"""Replay the existing regressions on the exact upstream and contribution commits.

Requires git, a host C compiler, GHDL and the integration repository's test files.
This checks the real SFP C function, extracted VHDL host-write decoder, and CPU map;
it does not claim to build or qualify the entire latest upstream design.
"""
import argparse
import ast
import json
from pathlib import Path
import re
import resource
import subprocess
import tempfile


def git(repo, *args):
    return subprocess.check_output(['git', '-C', str(repo), *args], text=True).strip()


def source(repo, revision, path):
    return git(repo, 'show', revision + ':' + path)


def c_function(text, name):
    start = re.search(r'^int ' + name + r'\(', text, re.M).start()
    end = text.index('{', start) + 1
    depth = 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--wr-cores', type=Path, required=True)
    parser.add_argument('--wrpc-sw', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[3]
    # A failing unpatched C assertion is expected, but should not create a core dump.
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    revisions = {
        'wr_cores_base': '8f9b06565ef8d23712494314def63f6fc3813f8c',
        'wr_cores_fixed': '628cfa0792c74bab83fed378b54ac0dbaa288436',
        'wrpc_base': '13527cd68e1833214a89e4ee8c5b208188ff0e6a',
        'wrpc_address_fixed': '225231378741cd3ded7132778b2cde90633e1698',
        'wrpc_sfp_fixed': '7983b2c9ec5fbca4e4f9d799b61b2c80457d2c56',
    }
    report = {'revisions': revisions, 'checks': {}}
    for prefix, repo in [('wr_cores', args.wr_cores), ('wrpc', args.wrpc_sw)]:
        base = revisions[prefix + '_base']
        for name, revision in revisions.items():
            if name.startswith(prefix) and not name.endswith('_base'):
                assert git(repo, 'rev-parse', revision + '^') == base, name

    tree = ast.parse((root / 'test/test_firmware_fixes.py').read_text())
    fn = next(n for n in tree.body if isinstance(n, ast.FunctionDef)
              and n.name == 'test_sfp_storage_error_is_not_a_calibration_match')
    literals = {n.targets[0].id: ast.literal_eval(n.value) for n in fn.body
                if isinstance(n, ast.Assign) and isinstance(n.value, ast.Constant)
                and isinstance(n.targets[0], ast.Name)}
    with tempfile.TemporaryDirectory(prefix='wr-upstream-audit-') as directory:
        work = Path(directory)
        for label, key in [('unpatched', 'wrpc_base'), ('fixed', 'wrpc_sfp_fixed')]:
            text = source(args.wrpc_sw, revisions[key], 'dev/sfp.c')
            path = work / 'sfp.c'
            path.write_text(literals['prefix'] + c_function(text, 'sfp_match') + literals['checks'])
            subprocess.run(['cc', '-std=c99', '-Werror=implicit-function-declaration',
                            str(path), '-o', str(work / 'sfp')], check=True, capture_output=True)
            run = subprocess.run([str(work / 'sfp')], capture_output=True, text=True)
            report['checks']['sfp_' + label] = {'returncode': run.returncode,
                                               'stderr': run.stderr.strip()}
            assert (run.returncode == 0) == (label == 'fixed')

        tree = ast.parse((root / 'test/test_wr_diags.py').read_text())
        call = next(n for n in ast.walk(tree) if isinstance(n, ast.Call)
                    and isinstance(n.func, ast.Attribute) and n.func.attr == 'write_text'
                    and isinstance(n.args[0], ast.BinOp))
        prefix = ast.literal_eval(call.args[0].left.left)
        suffix = ast.literal_eval(call.args[0].right)
        for label, key in [('unpatched', 'wr_cores_base'), ('fixed', 'wr_cores_fixed')]:
            text = source(args.wr_cores, revisions[key], 'modules/wrc_core/wrc_diags_dpram.vhd')
            assignments = re.findall(r'^  (?:s_is_control_word|s_we_user) <= .*?;$', text, re.M)
            assert len(assignments) == 2
            tb = work / 'diags_tb.vhd'
            tb.write_text(prefix + '\n'.join(assignments) + suffix)
            for action, target in [('-a', str(tb)), ('-e', 'diags_tb')]:
                subprocess.run(['ghdl', action, '--std=08', target], cwd=work,
                               check=True, capture_output=True)
            run = subprocess.run(['ghdl', '-r', '--std=08', 'diags_tb', '--assert-level=error'],
                                 cwd=work, capture_output=True, text=True)
            report['checks']['host_decoder_' + label] = {
                'returncode': run.returncode, 'output': (run.stdout + run.stderr).strip()}
            assert (run.returncode == 0) == (label == 'fixed')

    header = source(args.wrpc_sw, revisions['wrpc_base'], 'include/hw/wrc_devices_map.h')
    expected = int(re.search(r'#define WRC_DEVICES_MAP_WDIAG (0x[0-9a-f]+)', header).group(1), 16)
    cheby = source(args.wr_cores, revisions['wr_cores_base'], 'modules/wrc_core/wrc_devices_map.cheby')
    mapped = int(re.search(r'name: wdiag\s+address: (0x[0-9a-f]+)', cheby).group(1), 16)
    assert expected == mapped == 0x800
    for label, key in [('unpatched', 'wrpc_base'), ('fixed', 'wrpc_address_fixed')]:
        text = source(args.wrpc_sw, revisions[key], 'include/board.h')
        actual = int(re.search(r'#define BASE_WDIAGS_PRIV\s+\(DEV_BASE \+ (0x[0-9a-f]+)\)', text).group(1), 16)
        report['checks']['cpu_address_' + label] = {'address': hex(actual), 'expected': hex(expected)}
        assert (actual == expected) == (label == 'fixed')
    report['passed'] = True
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
