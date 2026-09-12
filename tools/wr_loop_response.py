#!/usr/bin/env python3
"""Measure the Acorn/SPEC USB bench with optional PLL stimuli and automatic recovery."""
import argparse
import fcntl
import hashlib
import json
from pathlib import Path
import re
import signal
import threading
import time

from litex import RemoteClient
from wr_console import Console
from wr_jtag import load_config
from wr_pll_trace import Trace, read_sensors
from wr_status import diagnostic_snapshot


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--master', required=True, choices=['acorn', 'spec'])
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--passive', type=float, default=0)
    parser.add_argument('--repeats', type=int, default=4)
    parser.add_argument('--step-ps', type=int, default=500)
    parser.add_argument('--frequency-step-ppb', type=float, default=100)
    parser.add_argument('--master-sensitivity-ppb', type=float,
        help='Measured master main-actuator slope, in ppb per code; required for stimuli.')
    args = parser.parse_args()
    cfg = load_config(args.config)
    if set(cfg['boards']) != {'acorn', 'spec'}:
        parser.error('This experiment requires the acorn/spec bench profile')
    if args.passive < 0 or args.repeats < 1 or not 1 <= args.step_ps <= 500:
        parser.error('Use a nonnegative passive duration, positive repeats and a 1..500 ps step')
    if not args.passive:
        if args.master_sensitivity_ppb is None or args.master_sensitivity_ppb <= 0 or args.frequency_step_ppb <= 0:
            parser.error('Stimuli require positive --master-sensitivity-ppb and --frequency-step-ppb')
        trim = cfg['boards'][args.master].get('master_trim')
        if trim is None:
            parser.error('Set the selected board\'s calibrated master_trim in --config before applying stimuli')
        step = round(args.frequency_step_ppb / args.master_sensitivity_ppb)
        if not 0 < step <= min(trim, 65535-trim):
            parser.error('Frequency step must be nonzero and remain inside the DAC rails')
    out = args.output
    out.mkdir(parents=True, exist_ok=False)
    master = args.master
    slave = 'spec' if master == 'acorn' else 'acorn'
    consoles, buses, traces, workers = {}, {}, {}, []
    gains = {}
    delock_counts = {}
    modified = False
    halt = threading.Event()
    errors = []
    events = (out/'events.jsonl').open('w', buffering=1)
    state = dict(master=master, slave=slave, start_unix=time.time(), passed=False,
                 restored=False, decimation=16, config=cfg,
                 arguments={k: str(v) if isinstance(v, Path) else v for k,v in vars(args).items()})
    state['tool_sha256'] = {name: hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
        for name in ('wr_loop_response.py', 'wr_pll_trace.py', 'wr_console.py', 'wr_status.py')}
    state['input_sha256'] = {name: {key: hashlib.sha256(Path(board[key]).read_bytes()).hexdigest()
        for key in ('csr', 'bitstream') if board.get(key)} for name,board in cfg['boards'].items()}

    def interrupt(signum, frame):
        raise KeyboardInterrupt(f'signal {signum}')
    signal.signal(signal.SIGTERM, interrupt)
    signal.signal(signal.SIGINT, interrupt)

    def event(kind, **data):
        events.write(json.dumps(dict(time_unix=time.time(), kind=kind, **data))+'\n')

    def command(name, text):
        event('command_start', board=name, command=text)
        answer = consoles[name].command(text, timeout=15)
        event('command_done', board=name, command=text, response=answer)
        return answer

    def pause(seconds):
        if halt.wait(seconds):
            raise RuntimeError('Capture stopped: '+repr(errors))

    def check_locks(name):
        stats = command(name, 'pll stat')
        delocks = re.search(r'DelCnt:(\d+)', stats)
        if delocks is None or 'HL1' not in stats or (name == slave and 'MFL1 MPL1' not in stats):
            raise RuntimeError(name + ': PLL lock check failed')
        count = int(delocks.group(1))
        if delock_counts.setdefault(name, count) != count:
            raise RuntimeError(name + ': PLL lost lock during the experiment')

    def capture(name):
        try:
            sensor_due = 0
            diagnostic_due = 0
            with (out/(name+'-sensors.jsonl')).open('w', buffering=1) as sensors, (out/(name+'-diagnostics.jsonl')).open('w', buffering=1) as diagnostics:
                while not halt.is_set():
                    frames = traces[name].poll()
                    now = time.monotonic()
                    if now >= sensor_due:
                        sensors.write(json.dumps(dict(time_unix=time.time(), **read_sensors(buses[name])))+'\n')
                        sensor_due = now + 1
                    if now >= diagnostic_due:
                        diagnostics.write(json.dumps(dict(time_unix=time.time(), **diagnostic_snapshot(buses[name])))+'\n')
                        diagnostic_due = now + 5
                    if not frames:
                        halt.wait(0.002)
        except BaseException as error:
            errors.append(name+': '+repr(error))
            halt.set()

    with (Path(cfg['state_dir'])/'bench.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        try:
            for name, board in cfg['boards'].items():
                c = consoles[name] = Console(board['uart'], out/name)
                c.connect()
                command(name, 'verbose 0')
                command(name, 'ver')
                stats = command(name, 'pll stat')
                gains[name] = {}
                for loop in ('m', 'h'):
                    values = re.search(rf'{loop}_kp:(-?\d+) {loop}_ki:(-?\d+) {loop}_sh:(\d+)', stats)
                    if values is None:
                        raise RuntimeError(name + ': cannot save PI gains before experiment')
                    gains[name][loop] = tuple(map(int, values.groups()))
                b = buses[name] = RemoteClient(host='127.0.0.1', port=board['jtag_port'],
                    csr_csv=board['csr'], timeout=5, raise_on_timeout=True)
                b.open()
                identifier = bytes(v & 255 for v in b.read(b.bases.identifier_mem, length=128)).split(b'\0')[0].decode()
                if board['identifier'] not in identifier:
                    raise RuntimeError(name + ': unexpected FPGA identifier: ' + identifier)
                event('initial_diagnostics', board=name, values=diagnostic_snapshot(b))
            state['saved_gains'] = gains
            if not args.passive:
                modified = True
                for name in consoles:
                    command(name, 'ptp stop')
                command(master, 'pll init 2 0 0')
                command(master, f"pll sdac 0 {cfg['boards'][master]['master_trim']}")
                command(slave, 'pll init 3 0 0')
                deadline = time.monotonic()+60
                while time.monotonic() < deadline:
                    answer = command(slave, 'pll cl 0')
                    if re.search(r'\n1\n', answer):
                        break
                    time.sleep(1)
                else:
                    raise RuntimeError('Isolated slave PLL did not lock')
                for name in consoles:
                    check_locks(name)
                command(slave, 'pll sps 0 0')
            for name, b in buses.items():
                traces[name] = Trace(b, out/(name+'-trace'), 16)
                t = threading.Thread(target=capture, args=(name,), daemon=True)
                workers.append(t)
                t.start()
            pause(10)
            initial_trace = {name: trace.summary() for name, trace in traces.items()}
            event('capture_ready', traces=initial_trace)
            if args.passive:
                pause(args.passive)
            else:
                kp, ki, shift = gains[slave]['m']
                event('frequency_step', codes=step, ppb=step*args.master_sensitivity_ppb)
                for factor in (1, 0.5, 2, 1):
                    event('gain_stage', factor=factor, loop='main')
                    command(slave, f'pll gain 0 0 {int(kp*factor)} {int(ki*factor)} {shift}')
                    pause(2)
                    for repeat in range(args.repeats):
                        event('native_step_trial', factor=factor, repeat=repeat)
                        for phase in (args.step_ps, 0, -args.step_ps, 0):
                            command(slave, f'pll step {phase}')
                            pause(1)
                            deadline = time.monotonic()+15
                            while traces[slave].last_count > 128:
                                if time.monotonic() >= deadline:
                                    raise RuntimeError('Trace FIFO did not drain between native bursts')
                                pause(.1)
                        event('phase_trial', factor=factor, repeat=repeat)
                        for phase in (1024, 0, -1024, 0):
                            command(slave, f'pll sps 0 {phase}')
                            pause(0.7)
                        event('frequency_trial', factor=factor, repeat=repeat, master_delta_code=step)
                        for delta in (step, 0, -step, 0):
                            command(master, f'pll sdac 0 {trim+delta}')
                            pause(0.7)
                    answer = command(slave, 'pll cl 0')
                    if not re.search(r'\n1\n', answer):
                        raise RuntimeError('Slave PLL lost lock during a gain stage')
                    for name in consoles:
                        check_locks(name)
                for factor in (0.5, 2, 1):
                    event('gain_stage', factor=factor, loop='helper')
                    for name in consoles:
                        hkp, hki, hshift = gains[name]['h']
                        command(name, f'pll gain -1 0 {int(hkp*factor)} {int(hki*factor)} {hshift}')
                    pause(2)
                    for repeat in range(args.repeats):
                        event('helper_frequency_trial', factor=factor, repeat=repeat, master_delta_code=step)
                        for delta in (step, 0, -step, 0):
                            command(master, f'pll sdac 0 {trim+delta}')
                            pause(1)
                    for name in consoles:
                        check_locks(name)
                event('experiment_done')
                pause(3)
            final_trace = {name: trace.summary() for name, trace in traces.items()}
            state['active_capture_gaps'] = {name: final_trace[name]['gaps']-initial_trace[name]['gaps'] for name in traces}
            state['active_discarded_frames'] = {name: final_trace[name]['discarded']-initial_trace[name]['discarded'] for name in traces}
            state['passed'] = not errors and not any(state['active_capture_gaps'].values()) and not any(state['active_discarded_frames'].values())
            for name in traces:
                required_sources = (0,1) if name == slave else (0,)
                state['passed'] &= all(final_trace[name]['samples'].get(source,0)
                    > initial_trace[name]['samples'].get(source,0) for source in required_sources)
            if not args.passive:
                native = final_trace[slave]['native_samples'].get(1, 0) - initial_trace[slave]['native_samples'].get(1, 0)
                state['native_samples'] = native
                state['passed'] &= native == 16 * args.repeats * 640
        except BaseException as error:
            state['error'] = repr(error)
            raise
        finally:
            halt.set()
            for worker in workers:
                worker.join(timeout=12)
                if worker.is_alive():
                    errors.append('Capture worker did not stop within 12 seconds')
            state['passed'] &= not errors
            state['capture_errors'] = errors
            state['traces'] = {name: trace.summary() for name, trace in traces.items()}
            for trace in traces.values():
                trace.close()
            restore_errors = []
            if modified:
                for name in consoles:
                    try:
                        command(name, 'ptp stop')
                        for loop, index in (('m', 0), ('h', -1)):
                            kp, ki, shift = gains[name][loop]
                            command(name, f'pll gain {index} 0 {kp} {ki} {shift}')
                        command(name, 'pll sps 0 0')
                        command(name, 'mode '+('master' if name == master else 'slave'))
                        if name == master:
                            command(name, f"pll sdac 0 {cfg['boards'][name]['master_trim']}")
                        command(name, 'ptp start')
                    except BaseException as error:
                        restore_errors.append(name+': '+repr(error))
            state['restore_errors'] = restore_errors
            state['passed'] &= not restore_errors
            state['restored'] = not restore_errors and len(consoles) == 2
            for console in consoles.values():
                console.close()
            for bus in buses.values():
                bus.close()
            state['end_unix'] = time.time()
            (out/'summary.json').write_text(json.dumps(state, indent=2)+'\n')
            events.close()
            print(json.dumps({k:v for k,v in state.items() if k != 'config'}, indent=2), flush=True)
    if not state['passed']:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
