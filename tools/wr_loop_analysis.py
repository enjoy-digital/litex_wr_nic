#!/usr/bin/env python3
"""Fit native PLL phase steps; all reported margins are model-derived."""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

RATE = 62500000 / 16384
QUANTUM_PS = 16000 / 16384


def model_response(count, gain, lag_samples, p, i):
    # Positive plant; upstream negative PI regulates e = y - reference.
    # First-order actuator with exact integration over a sampling interval.
    error, integrator, rate = -1., 0., 0.
    a = np.exp(-1 / lag_samples) if lag_samples > 1e-6 else 0.
    integral_factor = lag_samples * (1 - a)
    result = np.empty(count)
    for k in range(count):
        result[k] = error
        integrator -= i * error
        command = integrator - p * error
        error += gain * (command + (rate - command) * integral_factor)
        rate = a * rate + (1 - a) * command
    return result


def margins(gain, lag_samples, p, i, rate=RATE):
    w = np.geomspace(1e-5, np.pi, 100000)
    z = np.exp(1j*w)
    a = np.exp(-1/lag_samples) if lag_samples > 1e-6 else 0.
    h = lag_samples*(1-a)
    # x[k+1] = x[k] + g*((1-h)*u[k] + h*v[k]); v[k+1]=a*v[k]+(1-a)*u[k].
    plant = gain/(z-1) * ((1-h) + h*(1-a)/(z-a))
    loop = plant*(p+i*z/(z-1))
    magnitude = np.abs(loop)
    phase = np.unwrap(np.angle(loop))*180/np.pi
    crossing = np.argmin(abs(magnitude-1))
    phase_cross = np.where(phase <= -180 + 1e-7)[0]
    closed = loop/(1+loop)
    return dict(crossover_hz=float(w[crossing]*rate/(2*np.pi)),
        phase_margin_deg=float(180+phase[crossing]),
        gain_margin_db=None if not len(phase_cross) else float(-20*np.log10(magnitude[phase_cross[0]])),
        closed_loop_peak_db=float(20*np.log10(np.max(abs(closed)))))


def load(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def analyse_passive(directory, summary):
    start = next(e['time_unix'] for e in load(directory/'events.jsonl') if e['kind']=='capture_ready')
    result = dict(kind='passive', boards={},
        caveat='Uncalibrated FPGA die sensors and internal DAC commands; no independent PPS or oscillator-temperature measurement.')
    for board in ('acorn','spec'):
        sensors = [s for s in load(directory/(board+'-sensors.jsonl')) if s['time_unix']>=start]
        diagnostics = [s for s in load(directory/(board+'-diagnostics.jsonl')) if s['time_unix']>=start]
        samples = [s for s in load(directory/(board+'-trace.samples.jsonl')) if s['received_unix']>=start and 'dac' in s]
        if not sensors or not diagnostics or not samples:
            raise ValueError('Missing passive observation data for '+board)
        temperature = np.array([s['die_temperature_c'] for s in sensors])
        entry = dict(die_min_c=float(temperature.min()), die_max_c=float(temperature.max()),
            die_start_c=float(np.median(temperature[:30])), die_end_c=float(np.median(temperature[-30:])),
            sensor_samples=len(sensors), duration_seconds=sensors[-1]['time_unix']-sensors[0]['time_unix'],
            diagnostics=len(diagnostics),
            link_lock_time_valid=all(all(d[k] for k in ('link','locked','time_valid')) for d in diagnostics),
            servo_states=sorted(set(d['servo_state'] for d in diagnostics)),
            ptp_states=sorted(set(d['ptp_state'] for d in diagnostics)),
            rx_error_change=(diagnostics[-1]['rx_errors']-diagnostics[0]['rx_errors'])%(1<<32),
            servo_update_change=(diagnostics[-1]['updates']-diagnostics[0]['updates'])%(1<<32),
            dac={})
        end=sensors[-1]['time_unix']
        for source,name in ((0,'helper'),(1,'main')):
            rows=[s for s in samples if s['source']==source]
            if not rows:
                continue
            early=[s['dac'] for s in rows if s['received_unix']<start+30]
            late=[s['dac'] for s in rows if s['received_unix']>end-30]
            if not early or not late:
                raise ValueError('Incomplete DAC record for '+board)
            entry['dac'][name]=dict(start_mean=float(np.mean(early)),end_mean=float(np.mean(late)),
                min=min(s['dac'] for s in rows),max=max(s['dac'] for s in rows))
        result['boards'][board]=entry
    (directory/'analysis.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    return result


def analyse(directory, sensitivity_ppb, bootstrap=200):
    summary = json.loads((directory/'summary.json').read_text())
    if not summary['passed']:
        raise ValueError('Capture did not pass integrity checks')
    if summary['arguments'].get('passive'):
        return analyse_passive(directory,summary)
    if sensitivity_ppb is None or sensitivity_ppb<=0:
        raise ValueError('A positive slave main-actuator sensitivity is required for a loop fit')
    board = summary['slave']
    events = load(directory/'events.jsonl')
    capture_start = next(e['time_unix'] for e in events if e['kind']=='capture_ready')
    data = load(directory/(board+'-trace.samples.jsonl'))
    main = [f for f in data if f['source']==1 and 'sample_id' in f and f['received_unix']>=capture_start]
    bursts, current = [], []
    for index, f in enumerate(main):
        if f.get('stride') == 1:
            if not current:
                previous = main[max(0,index-40):index]
            current.append(f)
        elif current:
            bursts.append((previous,current)); current=[]
    if current:
        bursts.append((previous,current))
    commands = [e for e in events if e['kind']=='command_done' and e.get('command','').startswith('pll step ')]
    stages = [e for e in events if e['kind']=='gain_stage' and e['loop']=='main']
    if not bursts or len(bursts) != len(commands):
        raise ValueError('Native bursts and phase commands must match one to one')
    result = dict(board=board, bursts_seen=len(bursts), commands_seen=len(commands), bursts=[],
        sensitivity_ppb_per_code=sensitivity_ppb, bootstrap_repeats=bootstrap,
        caveat='Margins assume a discrete PI and integrating oscillator with first-order actuator lag. Bootstrap intervals resample whole bursts and exclude systematic/model error.')
    controllers = {}
    curves = {}
    for n,(previous,burst) in enumerate(bursts):
        errors = []
        if len(burst)!=640: errors.append('native sample count differs from 640')
        if any(f['gap'] for f in burst): errors.append('sample gap')
        if any(abs(f['ref']-16384)>1024 or abs(f['tag']-16384)>1024 for f in burst):
            errors.append('missing/abnormal DMTD tag period')
        if not previous or any(f['gap'] for f in previous[-20:]):
            errors.append('missing/gapped preceding baseline')
        expected_phase = int(int(commands[n]['command'].split()[-1]) / QUANTUM_PS)
        if any(f['phase_current'] != expected_phase for f in burst):
            errors.append('trace phase does not match command')
        if errors:
            result['bursts'].append(dict(index=n, valid=False, errors=errors)); continue
        step = burst[0]['phase_current']-previous[-1]['phase_current']
        if step==0:
            result['bursts'].append(dict(index=n, valid=False, errors=['zero phase step']));continue
        command_time = commands[n]['time_unix']
        stage = [e for e in stages if e['time_unix']<command_time][-1]
        factor = stage['factor']
        gain_command = [e for e in events if e['kind']=='command_done'
            and e.get('board')==board and e.get('command','').startswith('pll gain 0 0 ')
            and stage['time_unix']<e['time_unix']<command_time][-1]
        kp, ki, shift = map(int, gain_command['command'].split()[-3:])
        controller = (-kp / 2**shift, -ki / 2**shift)
        if min(controller)<=0 or controllers.setdefault(factor,controller)!=controller:
            raise ValueError('Unsupported or inconsistent PI gains within a stage')
        baseline = np.mean([f['error'] for f in previous[-20:]])
        normalized = (np.array([f['error'] for f in burst])-baseline)/step
        curves.setdefault(factor,[]).append(normalized)
        dac = np.array([f['dac'] for f in burst])
        interval = (burst[-1]['time_ms']-burst[0]['time_ms'])%(1<<24)
        result['bursts'].append(dict(index=n, valid=True, gain_factor=factor,
            step_ps=step*QUANTUM_PS, elapsed_ms=interval, first_error_ps=burst[0]['error']*QUANTUM_PS,
            peak_error_ps=float(max(abs(f['error']) for f in burst)*QUANTUM_PS),
            dac_min=int(dac.min()), dac_max=int(dac.max())))
    if any(not b['valid'] for b in result['bursts']):
        raise ValueError('Incomplete or invalid native burst: ' + repr([b for b in result['bursts'] if not b['valid']]))
    result['gain_stages']={}
    rng = np.random.default_rng(20260911)
    for factor, rows in curves.items():
        matrix=np.array(rows); mean=matrix.mean(axis=0)
        p,i=controllers[factor]
        initial_gain=sensitivity_ppb*1e-9*16384**2
        def residual(x):
            gain,lag,scale,offset=x
            return scale*model_response(len(mean),gain,lag,p,i)+offset-mean
        fit=least_squares(residual,[initial_gain,.3,1,0],
            bounds=([initial_gain*.25,0,.75,-.1],[initial_gain*3,8,1.25,.1]))
        gain,lag,scale,offset=fit.x
        predicted=scale*model_response(len(mean),gain,lag,p,i)+offset
        output=1+mean
        amplitude_ps=float(np.median([abs(b['step_ps']) for b in result['bursts'] if b.get('valid') and b['gain_factor']==factor]))
        smoothed=np.convolve(mean, np.ones(8)/8, mode='valid')
        unsettled=np.where(abs(smoothed)*amplitude_ps>50)[0]
        metrics=dict(bursts=len(rows), fitted_gain_counts_per_code_per_update=float(gain),
            static_gain_counts_per_code_per_update=initial_gain, actuator_lag_samples=float(lag),
            fitted_amplitude=float(scale), fitted_offset=float(offset),
            residual_rms_ps=float(np.sqrt(np.mean((mean-predicted)**2))*amplitude_ps),
            measured_overshoot_percent=float(100*(output.max()-1)),
            mean_50ps_settling_ms=None if len(unsettled) and unsettled[-1]==len(smoothed)-1 else float((unsettled[-1]+8 if len(unsettled) else 0)*1000/RATE),
            model_margins=margins(gain,lag,p,i))
        if bootstrap:
            fitted = []
            original_mean = mean
            for repeat in range(bootstrap):
                mean = matrix[rng.integers(0,len(matrix),len(matrix))].mean(axis=0)
                sampled = least_squares(residual, fit.x,
                    bounds=([initial_gain*.25,0,.75,-.1],[initial_gain*3,8,1.25,.1]))
                fitted.append(margins(sampled.x[0],sampled.x[1],p,i))
            mean = original_mean
            metrics['bootstrap_95_percent_interval'] = {key: np.percentile(
                [m[key] for m in fitted], [2.5,97.5]).tolist()
                for key in ('crossover_hz','phase_margin_deg')}
        result['gain_stages'][factor]=metrics
        np.savez(directory/(f'{board}-gain-{factor}-curves.npz'),mean=mean, individual=matrix,predicted=predicted,time_s=np.arange(len(mean))/RATE)
    result['temperature']={}
    for name in ('acorn','spec'):
        if not (directory/(name+'-sensors.jsonl')).exists():
            result['temperature'][name]={'available':False}
            continue
        sensors=load(directory/(name+'-sensors.jsonl'))
        temp=np.array([s['die_temperature_c'] for s in sensors])
        result['temperature'][name]=dict(samples=len(temp),die_min_c=float(temp.min()),die_max_c=float(temp.max()),
            die_start_c=float(np.median(temp[:10])),die_end_c=float(np.median(temp[-10:])))
    (directory/'analysis.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='bursts'},indent=2))
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('directory',type=Path)
    p.add_argument('--sensitivity-ppb', type=float, help='Previously measured slave main-actuator sensitivity, in ppb/code; required for a loop fit.')
    p.add_argument('--bootstrap', type=int, default=200)
    args=p.parse_args()
    if (args.sensitivity_ppb is not None and args.sensitivity_ppb<=0) or args.bootstrap<0:
        p.error('Sensitivity must be positive; bootstrap count must be nonnegative')
    analyse(args.directory,args.sensitivity_ppb,args.bootstrap)
