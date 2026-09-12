#!/usr/bin/env python3
"""Manage dedicated WR JTAG servers with persistent Linux process ownership."""

import argparse
import fcntl
import json
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import sys
import time


def load_config(path):
    path = Path(path).resolve()
    config = json.loads(path.read_text())
    def resolve(value):
        return os.path.abspath(path.parent / value)
    config['state_dir'] = resolve(config['state_dir'])
    config['pythonpath'] = [resolve(p) for p in config.get('pythonpath', [])]
    ports = set()
    for name, board in config['boards'].items():
        if not re.fullmatch(r'[a-zA-Z0-9_-]+', name):
            raise ValueError('Invalid board name')
        for key in ('csr', 'bitstream', 'jtag_config', 'uart'):
            if key in board:
                board[key] = resolve(board[key])
        for key in ('jtag_port', 'stream_port'):
            port = board[key]
            if type(port) is not int or not 1024 <= port <= 65535 or port in ports:
                raise ValueError('JTAG and stream ports must be distinct integers in 1024..65535')
            ports.add(port)
        if 'master_trim' in board and (type(board['master_trim']) is not int
                                      or not 0 <= board['master_trim'] <= 65535):
            raise ValueError('Master trim must be a 16-bit code')
        if 'jtag_config' not in board:
            if not re.fullmatch(r'\d+-\d+(?:\.\d+)*', board['usb_location']):
                raise ValueError('Expected a physical USB location, for example 1-3')
    return config


def jtag_config(config, board):
    cfg = config['boards'][board]
    if 'jtag_config' in cfg:
        return cfg['jtag_config']
    path = Path(config['state_dir']) / (board + '.cfg')
    path.write_text('adapter driver ftdi\nadapter speed 5000\n'
        f'adapter usb location {cfg["usb_location"]}\n'
        'transport select jtag\nftdi_vid_pid 0x0403 0x6011\nftdi_channel 0\n'
        'ftdi_layout_init 0x00e8 0x60eb\nreset_config none\n'
        'tcl_port disabled\ntelnet_port disabled\ngdb_port disabled\n'
        'source [find cpld/xilinx-xc7.cfg]\n')
    return str(path)


def process(pid):
    try:
        fields = (Path('/proc') / str(pid) / 'stat').read_text().rsplit(')', 1)[1].split()
        return dict(pid=int(pid), state=fields[0], group=int(fields[2]), started=fields[19])
    except (FileNotFoundError, ProcessLookupError):
        return None


def members(group):
    result = []
    for path in Path('/proc').iterdir():
        if path.name.isdigit():
            item = process(path.name)
            if item and item['group'] == group and item['state'] != 'Z':
                result.append(item)
    return result


def write_record(path, record):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(record, indent=2) + '\n')
    temporary.replace(path)


def listening(port):
    with socket.socket() as probe:
        probe.settimeout(0.2)
        return probe.connect_ex(('127.0.0.1', port)) == 0


def probe_board(cfg):
    from litex import RemoteClient
    bus = RemoteClient(host='127.0.0.1', port=cfg['jtag_port'], csr_csv=cfg['csr'],
        timeout=2, raise_on_timeout=True)
    bus.open()
    try:
        if bus.read(bus.mems.wr_wb_slave.base) != 0x57525043:
            raise RuntimeError('Invalid WR host signature')
        identifier = bytes(value & 255 for value in bus.read(bus.bases.identifier_mem,
            length=128)).split(b'\0')[0].decode()
        if cfg['identifier'] not in identifier:
            raise RuntimeError('Unexpected FPGA identity: '+identifier)
        return identifier
    finally:
        bus.close()


def stop(path):
    if not path.exists():
        return
    record = json.loads(path.read_text())
    if record['boot_id'] != Path('/proc/sys/kernel/random/boot_id').read_text().strip():
        path.unlink()
        return
    known = record['members']
    group = record['group']
    for sig, grace in [(signal.SIGTERM, 1), (signal.SIGKILL, 3)]:
        live = members(group)
        if not live:
            break
        if not any(old['pid'] == new['pid'] and old['started'] == new['started']
                   for old in known for new in live):
            raise RuntimeError(f'Refusing to signal unverified process group {group}')
        try:
            os.killpg(group, sig)
        except ProcessLookupError:
            break
        deadline = time.monotonic() + grace
        while members(group) and time.monotonic() < deadline:
            time.sleep(0.05)
    if members(group):
        raise RuntimeError(f'Owned process group {group} did not stop')
    path.unlink()


def serve(config, board, path):
    # Record our identity before exec starts LiteX/OpenOCD. A controller killed
    # during startup can subsequently clean up this process and its children.
    identity = process(os.getpid())
    assert identity['group'] == os.getpid(), 'Worker must own its process group'
    write_record(path, dict(boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
        group=os.getpid(), members=[identity]))
    cfg = config['boards'][board]
    command = [sys.executable, '-m', 'litex.tools.litex_server', '--jtag',
        '--jtag-config', jtag_config(config, board), '--jtag-port', str(cfg['stream_port']),
        '--bind-ip', '127.0.0.1', '--bind-port', str(cfg['jtag_port'])]
    os.execv(sys.executable, command)


def start(config, config_path, board, state_dir):
    cfg = config['boards'][board]
    path = state_dir / (board + '.json')
    stop(path)
    for port in (cfg['jtag_port'], cfg['stream_port']):
        if listening(port):
            raise RuntimeError(f'Port {port} has an unmanaged listener')
    for attempt in range(3):
        stamp = time.strftime('%Y%m%d-%H%M%S') + f'-{attempt + 1}'
        log_path = state_dir / (board + '-' + stamp + '.log')
        environment = os.environ.copy()
        if config.get('pythonpath'):
            environment['PYTHONPATH'] = os.pathsep.join(config['pythonpath'])
        with log_path.open('w') as log:
            child = subprocess.Popen([sys.executable, str(Path(__file__).resolve()),
                '--config', str(config_path), '--board', board, '_serve'],
                env=environment, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline:
                if child.poll() is not None:
                    raise RuntimeError(f'{board}: startup exited with {child.returncode}; see {log_path}')
                if path.exists():
                    record = json.loads(path.read_text())
                    live = members(record['group'])
                    # Keep identities already seen, including a parent which
                    # exits while its OpenOCD child is still starting.
                    record['members'] += [item for item in live if not any(
                        item['pid'] == old['pid'] and item['started'] == old['started']
                        for old in record['members'])]
                    write_record(path, record)
                if path.exists() and listening(cfg['jtag_port']):
                    probe_board(cfg)
                    print(f'{board}: JTAG server ready on localhost:{cfg["jtag_port"]}', flush=True)
                    return
                time.sleep(0.1)
            raise TimeoutError(f'{board}: JTAG startup timed out; see {log_path}')
        except BaseException as error:
            stop(path)
            # Covers the short interval before the worker writes its record.
            if child.poll() is None:
                child.terminate()
            child.wait(timeout=3)
            if not isinstance(error, Exception) or attempt == 2:
                raise
            print(str(error) + '; retrying', file=sys.stderr, flush=True)
            time.sleep(0.2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--board')
    parser.add_argument('action', choices=['start', 'stop', 'status', '_serve'])
    args = parser.parse_args()
    config_path = args.config.resolve()
    config = load_config(config_path)
    sys.path[:0] = config['pythonpath']
    state_dir = Path(config['state_dir'])
    state_dir.mkdir(parents=True, exist_ok=True)
    names = [args.board] if args.board else list(config['boards'])
    if any(name not in config['boards'] for name in names):
        parser.error('Unknown board')
    if args.action == '_serve':
        serve(config, args.board, state_dir / (args.board + '.json'))
    with (state_dir / 'bench.lock').open('a') as lock:
        if args.action != 'status':
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        for name in names:
            path = state_dir / (name + '.json')
            if args.action == 'start':
                start(config, config_path, name, state_dir)
            elif args.action == 'stop':
                stop(path)
            else:
                record = json.loads(path.read_text()) if path.exists() else None
                cfg = config['boards'][name]
                print(json.dumps(dict(board=name, record=record,
                    jtag=listening(cfg['jtag_port']), stream=listening(cfg['stream_port']))))


if __name__ == '__main__':
    def interrupted(signum, frame):
        raise KeyboardInterrupt(f'signal {signum}')
    signal.signal(signal.SIGTERM, interrupted)
    main()
