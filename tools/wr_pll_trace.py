#!/usr/bin/env python3
"""Read the upstream SoftPLL host FIFO without stopping the WR CPU.

Requires --with-wr-pll-debug gateware. Values are internal detector samples,
not an independent PPS measurement. Raw FIFO words remain in the capture.
"""

import argparse
import fcntl
import json
from pathlib import Path
import struct
import time

from litex import RemoteClient
from wr_jtag import load_config


FIELDS = {
    0: 'dac', 1: 'error', 2: 'tag', 3: 'ref', 4: 'period',
    5: 'sample_id', 6: 'event', 7: 'time_ms',
    8: 'phase_current', 9: 'phase_target', 10: 'src', 11: 'stride',
}
SIGNED = {'error', 'phase_current', 'phase_target'}
MODULUS = 1 << 24


class Decoder:
    def __init__(self, decimation):
        self.decimation = decimation
        self.pending = []
        self.previous = {}
        self.samples = {}
        self.native_samples = {}
        self.discarded = 0
        self.gaps = 0

    def feed(self, words):
        result = []
        for word in words:
            self.pending.append(word)
            if not word & 0x80000000:
                if len(self.pending) > 32:
                    raise ValueError('No SoftPLL end-of-sample marker in 32 records')
                continue
            records, self.pending = self.pending, []
            source = (records[0] >> 28) & 7
            frame = {'source': source}
            valid = True
            for value in records:
                field = FIELDS.get((value >> 24) & 15)
                if (value >> 28) & 7 != source or field is None or field in frame:
                    valid = False
                    break
                data = value & (MODULUS - 1)
                if field in SIGNED and data & (1 << 23):
                    data -= MODULUS
                frame[field] = data
            if valid and len(frame) == 2 and 'event' in frame:
                result.append(frame)
                continue
            required = {'time_ms', 'sample_id', 'dac', 'error'}
            if source == 1:
                required |= {'phase_current', 'phase_target', 'tag', 'ref'}
            if not valid or source not in (0, 1) or not required <= frame.keys():
                self.discarded += 1
                continue
            previous = self.previous.get(source)
            stride = frame.get('stride', self.decimation)
            if stride not in (1, self.decimation):
                raise ValueError('Unexpected trace stride: ' + str(stride))
            frame['stride'] = stride
            frame['gap'] = False
            frame['stride_transition'] = False
            if previous is None:
                frame['sample'] = 0
                frame['elapsed_ms'] = 0
            else:
                delta = (frame['sample_id'] - previous['sample_id']) % MODULUS
                frame['stride_transition'] = stride != previous['stride']
                frame['gap'] = (not 0 < delta <= max(stride, previous['stride'])
                    if frame['stride_transition'] else delta != stride)
                self.gaps += int(frame['gap'])
                frame['sample'] = previous['sample'] + delta
                frame['elapsed_ms'] = previous['elapsed_ms'] + (frame['time_ms'] - previous['time_ms']) % MODULUS
            self.previous[source] = frame
            self.samples[source] = self.samples.get(source, 0) + 1
            if stride == 1:
                self.native_samples[source] = self.native_samples.get(source, 0) + 1
            result.append(frame)
        return result


def read_sensors(bus):
    names = ('temperature', 'vccint', 'vccaux', 'vccbram')
    addresses = [getattr(bus.regs, 'xadc_' + name).addr for name in names]
    if addresses != list(range(addresses[0], addresses[0]+16, 4)):
        raise ValueError('Expected contiguous 32-bit XADC status registers')
    values = dict(zip(names, bus.read(addresses[0], length=4)))
    if any(not 0 < value < 4096 for value in values.values()):
        raise ValueError('Invalid XADC readings: ' + repr(values))
    return dict(raw=values, die_temperature_c=values['temperature'] * 503.975 / 4096 - 273.15,
        **{name + '_v': values[name] * 3 / 4096 for name in ('vccint', 'vccaux', 'vccbram')})


class Trace:
    def __init__(self, bus, output, decimation):
        self.bus = bus
        self.base = bus.mems.wr_wb_slave.base
        if bus.read(self.base) != 0x57525043:
            raise ValueError('WR host map signature mismatch')
        self.decoder = Decoder(decimation)
        self.raw = output.with_suffix('.words.bin').open('wb')
        self.frames = output.with_suffix('.samples.jsonl').open('w', buffering=1)
        self.high_water = 0
        self.last_count = 0

    def poll(self):
        status = self.bus.read(self.base + 0x278)
        if status & ~0x21fff:
            raise ValueError('Invalid SoftPLL FIFO status: ' + hex(status))
        self.last_count = status & 0x1fff
        if status & 0x20000:
            return []
        count = status & 0x1fff
        if not count:
            return []
        self.high_water = max(self.high_water, count)
        # Fixed-address bursts pop FIFO words. Incrementing reads access other
        # registers and silently corrupt the measurement.
        words = self.bus.read(self.base + 0x270, length=min(count, 2040), burst='fixed')
        self.raw.write(struct.pack('<' + 'I' * len(words), *words))
        received = time.time()
        frames = self.decoder.feed(words)
        for frame in frames:
            frame['received_unix'] = received
            self.frames.write(json.dumps(frame) + '\n')
        return frames

    def close(self):
        self.raw.close()
        self.frames.close()

    def summary(self):
        return dict(samples=dict(self.decoder.samples), gaps=self.decoder.gaps,
            native_samples=dict(self.decoder.native_samples),
            discarded=self.decoder.discarded, fifo_high_water_words=self.high_water,
            incomplete_tail_words=len(self.decoder.pending))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--board', required=True)
    parser.add_argument('--decimation', type=int, required=True, help='Must match the loaded firmware.')
    parser.add_argument('--duration', type=float, default=60)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.decimation < 1 or args.duration <= 0:
        parser.error('Decimation and duration must be positive')
    config = load_config(args.config)
    cfg = config['boards'][args.board]
    args.output.mkdir(parents=True, exist_ok=False)
    result = dict(board=args.board, decimation=args.decimation, start_unix=time.time(), passed=False)
    with (Path(config['state_dir']) / 'bench.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        bus = RemoteClient(host='127.0.0.1', port=cfg['jtag_port'], csr_csv=cfg['csr'], timeout=5, raise_on_timeout=True)
        bus.open()
        trace = None
        try:
            trace = Trace(bus, args.output / args.board, args.decimation)
            with (args.output / 'sensors.jsonl').open('w', buffering=1) as sensors:
                deadline, next_sensor = time.monotonic() + args.duration, 0
                while time.monotonic() < deadline:
                    frames = trace.poll()
                    now = time.monotonic()
                    if now >= next_sensor:
                        sensors.write(json.dumps(dict(time_unix=time.time(), **read_sensors(bus))) + '\n')
                        next_sensor = now + 1
                    if not frames:
                        time.sleep(0.002)
            result.update(trace.summary())
            result['passed'] = bool(result['samples']) and not result['gaps'] and not result['discarded']
        finally:
            if trace:
                result.update(trace.summary())
                trace.close()
            bus.close()
            result['end_unix'] = time.time()
            (args.output / 'summary.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    if not result['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
