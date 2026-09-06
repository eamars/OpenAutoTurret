"""Plot actual yaw/pitch, reference error and motor commands from a live capture.

Usage: project-venv/python tools/analyze_closed_loop_capture.py capture.json[.gz]
This analyzes telemetry; it does not command the station. A varying person/anchor
is not ground truth for a fixed-target settling claim.
"""
import gzip,json,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
path=Path(sys.argv[1]);raw=json.loads(gzip.decompress(path.read_bytes()) if path.suffix=='.gz' else path.read_bytes())
if path.suffix=='.gz':path=path.with_suffix('')
source = raw['samples'] if isinstance(raw, dict) else [s for s in raw if s.get('probe_stage')=='tracking']
rows=sorted({s['ts_ns']:s for s in source}.values(), key=lambda s:s['ts_ns'])
if len(rows)<2:raise SystemExit('Capture has fewer than two telemetry samples')
t=np.array([s['ts_ns'] for s in rows],dtype=float);t=(t-t[0])/1e9
q=np.degrees([[s['q_yaw_rad'],s['q_pitch_rad']] for s in rows])
ref=np.degrees([[s['q_ref_yaw_rad'],s['q_ref_pitch_rad']] for s in rows])
cmd=np.degrees([[s.get('service_command_rate_yaw_rad_s',s.get('tracking_command_rate_yaw_rad_s',0)),
                 s.get('service_command_rate_pitch_rad_s',s.get('tracking_command_rate_pitch_rad_s',0))] for s in rows])
goal=np.degrees([[s['tracking_aim_yaw_rad'],s['tracking_aim_pitch_rad']]
                 if s.get('tracking_aim_joint_valid') else [np.nan,np.nan] for s in rows])
err=ref-q
valid=np.array([s['mode_phase']=='TRACKING' for s in rows])
stats={'duration_s':t[-1],'unique_samples':len(rows),'tracking_fraction':float(valid.mean()),'start_q_deg':q[0].tolist(),'end_q_deg':q[-1].tolist(),'selection_uuids':list(set(s.get('selected_uuid') for s in rows)),
       'faults':list(set(s['fault'] for s in rows if s['fault'])),
       'max_telemetry_gap_s':float(np.max(np.diff(t))),
       'damped_reference_samples':sum(s.get('tracking_reference_damped',False) for s in rows)}
for lo,hi in [(0,5),(5,15),(15,30),(30,45)]:
    use=(t>=lo)&(t<hi)&valid
    if use.any():stats[f'window_{lo}_{hi}']={'q_span_deg':np.ptp(q[use],axis=0).tolist(),'q_std_deg':np.std(q[use],axis=0).tolist(),'error_p95_deg':np.percentile(np.abs(err[use]),95,axis=0).tolist(),'error_rms_deg':np.sqrt(np.mean(err[use]**2,axis=0)).tolist(),'command_rms_deg_s':np.sqrt(np.mean(cmd[use]**2,axis=0)).tolist()}
fig,axs=plt.subplots(3,2,figsize=(12,10),layout='constrained')
gaps=np.r_[False,np.diff(t)>.25]
def broken(series):
    result=series.copy();result[gaps]=np.nan;return result
for k,a in enumerate(['Yaw','Pitch']):
    axs[0,k].plot(t,broken(q[:,k]),label='Measured');axs[0,k].plot(t,broken(ref[:,k]),alpha=.75,label='Position reference')
    if np.isfinite(goal[:,k]).any():axs[0,k].plot(t,broken(goal[:,k]),alpha=.7,label='Predicted aim')
    axs[0,k].set(ylabel=a+' (degrees)',xlabel='Seconds');axs[0,k].legend()
    axs[1,k].plot(t,broken(err[:,k]));axs[1,k].axhline(0,color='gray',lw=.7);axs[1,k].set(ylabel=a+' reference minus measured (deg)',xlabel='Seconds')
axs[2,0].plot(q[:,0],q[:,1],alpha=.7);axs[2,0].scatter(*q[0],label='Start',color='green');axs[2,0].scatter(*q[-1],label='End',color='red');axs[2,0].set(xlabel='Measured yaw (deg)',ylabel='Measured pitch (deg)');axs[2,0].axis('equal');axs[2,0].legend()
axs[2,1].plot(t,broken(cmd[:,0]),label='Yaw');axs[2,1].plot(t,broken(cmd[:,1]),label='Pitch');axs[2,1].set(xlabel='Seconds',ylabel='Motor velocity command (deg/s)');axs[2,1].legend()
for ax in axs.flat:ax.grid(alpha=.25)
fig.suptitle(path.stem+' — actual station telemetry')
fig.savefig(path.with_suffix('.png'),dpi=150)
path.with_suffix('.analysis.json').write_text(json.dumps(stats,indent=2));print(json.dumps(stats,indent=2))
