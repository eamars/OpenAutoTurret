#!/usr/bin/env python3
"""Offline analysis of measure_station_latency.py captures (NumPy/Matplotlib)."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def stats(values):
    a = np.asarray(values, dtype=float)
    return dict(n=len(a), **{k: float(v) for k,v in zip(
        ('min','p50','p95','p99','max'), np.percentile(a,[0,50,95,99,100]))}) if len(a) else dict(n=0)


def analyze(path):
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    states = [r['state'] for r in rows if r['kind']=='state']
    perceptions = {r['data']['frame_sequence']:r['data'] for r in rows if r['kind']=='perception'}
    cameras = [r['data'] for r in rows if r['kind']=='camera']
    pair = [(c,perceptions[c['frame_sequence']]) for c in cameras if c['frame_sequence'] in perceptions]
    timing = dict(
        sensor_to_publish_ms=stats([(p['publish_timestamp_ns']-p['sensor_timestamp_ns'])/1e6 for p in perceptions.values()]),
        sensor_to_request_return_ms=stats([(c['metadata_receive_ns']-c['sensor_timestamp_ns'])/1e6 for c in cameras]),
        request_return_to_publish_ms=stats([(p['publish_timestamp_ns']-c['metadata_receive_ns'])/1e6 for c,p in pair]),
        exposure_ms=stats([c['camera']['ExposureTime']/1000 for c in cameras]),
        frame_duration_ms=stats([c['camera']['FrameDuration']/1000 for c in cameras]),
        publish_to_controller_ms=stats([s['vision_publish_to_receive_ms'] for s in states]),
        controller_sensor_age_ms=stats([s['vision_sensor_age_ms'] for s in states]),
        feedback_age_ms=stats([s['feedback_age_ms'] for s in states]),
        control_cycle_ms=stats([s['control_cycle_us']/1000 for s in states]),
        telemetry_interval_ms=stats(np.diff([s['ts_ns'] for s in states])/1e6),
        camera_fps=stats([s['camera_fps'] for s in states]))
    summary = dict(source=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        counts={kind:sum(r['kind']==kind for r in rows) for kind in sorted({r['kind'] for r in rows})},
        timing=timing, faults=sorted({s['fault'] for s in states}),
        safety_actions=sorted({s['safety_action'] for s in states}),
        deadline_misses_max=max(s['control_deadline_misses'] for s in states),
        clock='All event/capture/control timestamps originate on the Pi; no SSH timing.',
        trials=[])
    plots=[]
    starts=[r for r in rows if r['kind']=='trial_start']
    for start in starts:
        end=next((r for r in rows if r['kind']=='trial_end' and r['index']==start['index']),None)
        if end is None:
            summary['trials'].append(dict(index=start['index'], direction=start['direction'], excluded='Run aborted before trial completion'))
            continue
        commands=[r for r in rows if r['kind']=='command' and start['observed_ns']<=r['sent_ns']<end['observed_ns']]
        begin=next(r for r in commands if r['command']=='manual_jog_start')
        finish=next(r for r in commands if r['command']=='manual_jog_stop')
        events={e['t_ns']:e for s in states for e in s.get('events',[]) if e['event']=='MANUAL_JOG_STARTED'
                and begin['sent_ns']<=e['t_ns']<finish['sent_ns']}
        t0=min(events) if events else begin['sent_ns']
        axis=start['direction'][:-1]
        sign=1 if start['direction'].endswith('+') else -1
        ss=[s for s in states if t0-1e9<=s['ts_ns']<=end['observed_ns']]
        t=(np.array([s['ts_ns'] for s in ss],dtype=np.int64)-t0)/1e9
        qt=(np.array([s[f'feedback_timestamp_{axis}_ns'] for s in ss],dtype=np.int64)-t0)/1e9
        q=np.degrees([s[f'q_{axis}_rad'] for s in ss])
        ref=np.degrees([s[f'q_ref_{axis}_rad'] for s in ss])
        rate=np.degrees([s[f'service_command_rate_{axis}_rad_s'] for s in ss])
        q0=np.median(q[t<0]); r0=np.median(ref[t<0])
        q=(q-q0)*sign; ref=(ref-r0)*sign; rate*=sign

        def crossing(times, values, threshold, after=0):
            for k in range(1,len(times)-1):
                if times[k]>=after and values[k]>=threshold and values[k+1]>=threshold:
                    return [float(max(after,times[k-1])*1000),float(times[k]*1000)]
            return None

        trial=dict(index=start['index'], direction=start['direction'],
            command_to_controller_event_ms=(t0-begin['sent_ns'])/1e6,
            controller_event_exact=bool(events), baseline_encoder_span_deg=float(np.ptp(q[t<0])),
            command_rate_025_onset_ms=crossing(t,rate,.25),
            reference_025_displacement_ms=crossing(t,ref,.25),
            encoder_025_displacement_ms=crossing(qt,q,.25),
            reference_050_displacement_ms=crossing(t,ref,.5),
            encoder_050_displacement_ms=crossing(qt,q,.5),
            encoder_peak_displacement_deg=float(max(q)),
            encoder_final_displacement_deg=float(np.median(q[t>t[-1]-.8])),
            final_reference_error_deg=float(np.median((q-ref)[t>t[-1]-.8])),
            stop_request_after_event_ms=(finish['sent_ns']-t0)/1e6)
        # Descriptive time-shift fit, not an identified pure motor dead time.
        # Uses encoder sample timestamps and integrated commanded velocity.
        integral=np.r_[0,np.cumsum((rate[1:]+rate[:-1])/2*np.diff(t))]
        mask=(qt>=0)&(qt<=3.5)
        candidates=[]
        for lag in np.arange(0,.601,.001):
            predicted=np.interp(qt[mask]-lag,t,integral)
            design=np.column_stack([predicted,np.ones(len(predicted))])
            gain,bias=np.linalg.lstsq(design,q[mask],rcond=None)[0]
            residual=q[mask]-(gain*predicted+bias)
            candidates.append((float(np.mean(residual**2)),lag,gain,bias))
        mse,lag,gain,bias=min(candidates)
        trial['command_integral_delay_fit']=dict(delay_ms=lag*1000,gain=gain,bias_deg=bias,
            rmse_deg=mse**.5, at_search_bound=bool(lag==0 or lag>=.599),
            limitation='Descriptive fit; feedback is downsampled and plant has dynamics, friction and load. A bound hit is not a valid delay estimate.')
        summary['trials'].append(trial)
        plots.append((start['direction'],t,qt,q,ref,rate,(finish['sent_ns']-t0)/1e9))
    return summary, plots


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('captures',nargs='+',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    reports=[]; plots=[]
    for path in args.captures:
        report,p=analyze(path); reports.append(report); plots.extend(p)
    (args.output/'summary.json').write_text(json.dumps(reports,indent=2))
    if plots:
        fig,axes=plt.subplots(len(plots),2,figsize=(12,2.6*len(plots)),squeeze=False)
        for row,(direction,t,qt,q,ref,rate,stop) in zip(axes,plots):
            row[0].plot(t,ref,label='Planned position',lw=1.7)
            row[0].plot(qt,q,'.-',label='Motor encoder',lw=1,ms=3)
            row[1].plot(t,rate,label='Commanded motor speed')
            for ax in row:
                ax.axvline(0,color='gray',ls=':'); ax.axvline(stop,color='red',ls=':',label='Stop request')
                ax.set_xlabel('Seconds after controller accepted jog'); ax.grid(alpha=.25)
                ax.legend(fontsize=8,loc='upper left')
            row[0].set_ylabel(direction+' displacement (deg)'); row[1].set_ylabel('Signed speed (deg/s)')
        fig.suptitle('Target-free physical response: 2 s FINE jogs, production service unchanged',y=1.001)
        fig.tight_layout(); fig.savefig(args.output/'physical-response.png',dpi=140,bbox_inches='tight')
        plt.close(fig)
        fig,axes=plt.subplots(2,2,figsize=(11,7),sharex=True)
        for ax,direction in zip(axes.flat,('yaw+','yaw-','pitch+','pitch-')):
            first=True
            for name,t,qt,q,ref,rate,stop in plots:
                if name!=direction:
                    continue
                ax.plot(t,ref,color='#2674b8',alpha=.65,label='Planned' if first else None)
                ax.plot(qt,q,color='#d05a28',alpha=.75,label='Encoder' if first else None)
                first=False
            ax.axvline(0,color='gray',ls=':'); ax.axvline(2.08,color='gray',ls=':')
            ax.set_title(direction); ax.grid(alpha=.2); ax.legend(); ax.set_ylabel('Signed displacement (deg)')
            ax.set_xlabel('Seconds after controller accepted jog')
        fig.suptitle('Physical motion: repeated 2 s FINE jogs without a target\nEach line is one trial; planned and measured positions are separate')
        fig.tight_layout(); fig.savefig(args.output/'physical-overview.png',dpi=160)
        plt.close(fig)
    print(json.dumps(reports,indent=2))


if __name__=='__main__':
    main()
