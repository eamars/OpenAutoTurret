#!/usr/bin/env python3
"""Summarize target-free numeric traces. Use the project venv with NumPy."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def stats(values):
    a=np.array(values,dtype=float)
    return None if not len(a) else dict(n=len(a),median=float(np.median(a)),
        p95=float(np.percentile(a,95)),maximum=float(np.max(a)))


def analyze(path):
    rows=[json.loads(line) for line in path.read_text().splitlines()]
    traces=[r for r in rows if r['kind']=='trace']
    acknowledgements=[r for r in rows if r['kind']=='trial_ack']
    trials=[]
    for ack in acknowledgements:
        axis,delta,omega,*gain=ack['arg'].split(':')
        axis_i=['pitch','yaw'].index(axis)
        delta=float(delta); direction=np.sign(delta)
        seq=ack['state']['cmd_ack_seq']
        samples=[r for r in traces if r['ack']==seq and r.get('omega',0)>0]
        if not samples:
            continue
        t=np.array([r['t'] for r in samples],dtype=np.int64)
        t=(t-t[0])/1e9
        goal=samples[0]['goal'][axis_i]*180/np.pi
        base=goal-delta
        q=np.array([r['q'][axis_i] for r in samples])*180/np.pi
        ref=np.array([r['ref'][axis_i] for r in samples])*180/np.pi
        cmd=np.array([r['cmd'][axis_i] for r in samples])*180/np.pi
        vref=np.array([r['vref'][axis_i] for r in samples])*180/np.pi
        # Cancellation ends the commanded trial, not physical motion. Include
        # post-cancel encoder evidence up to the next trial, including state
        # samples retained when the trace observer itself aborted.
        begin=samples[0]['t']; active_end=samples[-1]['t']
        later=[r['t'] for r in traces if r.get('omega',0)>0 and r['ack']>seq]
        end=min(later) if later else float('inf')
        observed=[(r['t'],r['q'][axis_i]*180/np.pi) for r in traces if begin<=r['t']<end]
        for row in rows:
            if row['kind']!='state':
                continue
            state=row['data']
            timestamp=state.get(f'feedback_timestamp_{axis}_ns',state['ts_ns'])
            if begin<=timestamp<end:
                observed.append((timestamp,state[f'q_{axis}_rad']*180/np.pi))
        post_active=[point for point in observed if point[0]>active_end]
        def crossing(signal,fraction):
            passed=direction*(signal-base) >= abs(delta)*fraction
            for i in np.flatnonzero(passed):
                j=np.searchsorted(t,t[i]+.05)
                if j<len(t) and np.all(passed[i:j+1]):
                    return round(float(t[i]*1000),2)
            return None
        settled=None
        inside=abs(q-goal)<=.2
        for i in np.flatnonzero(inside & (t<=t[-1]-1)):
            if np.all(inside[i:]):
                settled=round(float(t[i]*1000),2); break
        trial=dict(arg=ack['arg'],axis=axis,delta_deg=delta,omega=float(omega),
            # An omitted gain uses the deployed controller configuration; it
            # cannot be reconstructed as a constant from the command string.
            position_gain=float(gain[0]) if gain else None,
            seq=seq,samples=len(samples),duration_s=round(float(t[-1]),3),
            valid_full_trial=bool(t[-1]>=5.8 and all(r['safety']==0 for r in samples)),
            start_deg=round(base,4),goal_deg=round(goal,4),
            encoder_t50_ms=crossing(q,.5),encoder_t90_ms=crossing(q,.9),
            reference_t50_ms=crossing(ref,.5),reference_t90_ms=crossing(ref,.9),
            settling_within_point2_deg_ms=settled,
            overshoot_deg=round(float(max(0,np.max(direction*(q-goal)))),4),
            observed_overshoot_including_stop_deg=round(max(0,max(direction*(p[1]-goal) for p in observed)),4),
            observed_peak_excursion_deg=round(max(abs(p[1]-base) for p in observed),4),
            observation_end_s=round((max(p[0] for p in observed)-begin)/1e9,3),
            post_active_samples=len(post_active),
            post_active_overshoot_deg=round(max(0,max(direction*(p[1]-goal) for p in post_active)),4) if post_active else None,
            late_error_deg=round(float(np.median(q[t>=4]-goal)),4) if np.any(t>=4) else None,
            late_span_deg=round(float(np.ptp(q[t>=4])),4) if np.any(t>=4) else None,
            peak_command_deg_s=round(float(np.max(abs(cmd))),3),
            peak_reference_deg_s=round(float(np.max(abs(vref))),3),
            peak_effort_nm=max(abs(r['effort'][axis_i]) for r in samples))
        trials.append(trial)
    cameras=[r['data'] for r in rows if r['kind']=='camera']
    stages={}
    for camera in cameras:
        values=dict(camera.get('stages_ms',{}))
        values['sensor_to_wire_ms']=(camera['wire_done_ns']-camera['sensor_timestamp_ns'])/1e6
        values['sensor_to_request_ms']=(camera['metadata_receive_ns']-camera['sensor_timestamp_ns'])/1e6
        values['request_to_wire_ms']=(camera['wire_done_ns']-camera['metadata_receive_ns'])/1e6
        values.update(camera.get('camera',{}))
        if camera.get('imx500_kpi_ms'):
            values['dnn_ms'],values['dsp_ms']=camera['imx500_kpi_ms']
        for k,v in values.items():
            if isinstance(v,(int,float)): stages.setdefault(k,[]).append(v)
    timestamps=np.array([r['t'] for r in traces],dtype=np.int64)
    return dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        complete=any(r['kind']=='complete' for r in rows),
        errors=[r.get('error') for r in rows if r['kind']=='error'],
        trace_samples=len(traces),trace_interval_ms=stats(np.diff(timestamps)/1e6),
        control_period_ms=stats([r['period_us']/1000 for r in traces]),
        feedback_age_ms={axis:stats([(r['t']-r['rx'][i])/1e6 for r in traces])
                         for i,axis in enumerate(('pitch','yaw'))},
        camera={k:stats(v) for k,v in stages.items()},trials=trials)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('inputs',nargs='+',type=Path)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    results=[analyze(path) for path in args.inputs]
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(results,indent=2))
    for result in results:
        print(result['path'],result['trace_samples'],'samples')
        for trial in result['trials']:
            print(trial['arg'],'t50/t90',trial['encoder_t50_ms'],trial['encoder_t90_ms'],
                'reference',trial['reference_t50_ms'],trial['reference_t90_ms'],
                'overshoot',trial['overshoot_deg'],'late error',trial['late_error_deg'])


if __name__=='__main__':
    main()
