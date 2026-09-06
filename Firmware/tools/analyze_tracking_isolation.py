#!/usr/bin/env python3
"""Render the September 6 boundary-isolation captures and their source manifest."""
import argparse
import collections,hashlib,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('capture_directory',type=Path)
parser.add_argument('geometry_summary',type=Path)
parser.add_argument('geometry_fits',type=Path)
parser.add_argument('output_directory',type=Path)
args=parser.parse_args()
root=args.capture_directory
out=args.output_directory
out.mkdir(parents=True,exist_ok=True)
summary={'sources':{}}
def source(name):
    p=root/name;summary['sources'][name]=hashlib.sha256(p.read_bytes()).hexdigest();return p
def unique(name):
    rows=json.loads(source(name).read_text())
    return sorted({s['ts_ns']:s for s in rows}.values(),key=lambda s:s['ts_ns'])
states=unique('isolation-fixed-camera-01.json')
t=(np.array([s['ts_ns'] for s in states])-states[0]['ts_ns'])/1e9
q=np.degrees([[s['q_yaw_rad'],s['q_pitch_rad']] for s in states])
anchors=np.array([[s['tracks'][0]['anchor_x']*1920,s['tracks'][0]['anchor_y']*1080] for s in states])
summary['held_stationary']={'unique_states':len(states),'duration_s':float(t[-1]),'anchor_std_px':np.std(anchors,axis=0).tolist(),'anchor_span_px':np.ptp(anchors,axis=0).tolist(),'encoder_std_deg':np.std(q,axis=0).tolist(),'encoder_span_deg':np.ptp(q,axis=0).tolist(),'selected_track_id_count':len({tr['uuid'] for s in states for tr in s['tracks'] if tr['selected']}),'tentative_track_id_count':len({tr['uuid'] for s in states for tr in s['tracks'] if tr['state']=='TENTATIVE'})}
fig,ax=plt.subplots(2,1,figsize=(10,6),sharex=True)
ax[0].plot(t,anchors-np.median(anchors,axis=0),label=['Measured anchor X','Measured anchor Y']);ax[0].set_ylabel('Anchor displacement (px)');ax[0].legend()
ax[1].plot(t,q-np.median(q,axis=0),label=['Yaw encoder','Pitch encoder']);ax[1].set_ylabel('Joint displacement (deg)');ax[1].set_xlabel('Time (s)');ax[1].legend()
fig.suptitle('Camera held: measurement variation and independent encoder motion')
fig.tight_layout();fig.savefig(out/'isolation-held-stationary.png',dpi=150);plt.close(fig)
summary['planner']={}
fig,axes=plt.subplots(3,1,figsize=(10,8),sharex=True)
for name in ['before','after']:
    d=np.genfromtxt(source('isolation-clean-reference-'+name+'.csv'),delimiter=',',names=True)
    result=[]
    for ax,speed in zip(axes,[3,10,15]):
        a=d[d['target_speed_deg_s']==speed];sel=a['time_s']>=5
        e=a['reference_deg']-a['target_deg'];nonzero=np.sign(e[sel][abs(e[sel])>.05])
        result.append({'target_speed_deg_s':speed,'stop_overshoot_deg':float(max(e[sel])),'crossings_beyond_005deg':int(np.sum(np.diff(nonzero)!=0)),'final_error_deg':float(e[-1]),'last_2s_max_error_deg':float(max(abs(e[a['time_s']>=10])))})
        ax.plot(a['time_s']-5,e,label=name)
        ax.set_ylabel(f'{speed} deg/s\nReference error (deg)');ax.axvline(0,color='gray',ls=':');ax.set_xlim(-1,7);ax.legend()
    summary['planner'][name]=result
axes[-1].set_xlabel('Time after clean target stops (s)');fig.suptitle('Production reference planner only: detector, estimator and motor excluded')
fig.tight_layout();fig.savefig(out/'isolation-planner.png',dpi=150);plt.close(fig)
d=np.genfromtxt(source('isolation-estimator-held.csv'),delimiter=',',names=True)
t=(d['capture_ns']-d['capture_ns'][0])/1e9;valid=d['valid']==1;pred=d['prediction_valid']==1
fig,ax=plt.subplots(2,1,figsize=(10,6),sharex=True)
for i,(raw,pr,label) in enumerate([('u_px','predicted_u','Horizontal (px)'),('v_px','predicted_v','Vertical (px)')]):
    ax[i].plot(t,np.where(valid,d[raw],np.nan),'.',ms=2,label='Measured anchor')
    ax[i].plot(t,np.where(pred,d[pr],np.nan),label='Predicted anchor (valid only)')
    ax[i].set_ylabel(label);ax[i].legend()
ax[-1].set_xlabel('Time (s)');ax[-1].set_xlim(0,t[-1]);fig.suptitle('Held-camera move/stop replay: gaps retained, actual estimator and geometry')
fig.tight_layout();fig.savefig(out/'isolation-estimator-held.png',dpi=150);plt.close(fig)
distance=np.hypot(d['predicted_u']-d['u_px'],d['predicted_v']-d['v_px'])[valid&pred]
summary['estimator_replay']={'rows':len(d),'valid_measurements':int(sum(valid)),'accepted':int(sum(d['accepted'][valid])),'same_frame_prediction_displacement_px_p50_p95_max':np.percentile(distance,[50,95,100]).tolist(),'limitation':'Prediction displacement is not future-ground-truth accuracy; API pose interpolation and publication-time arrival are approximate.'}
frames=json.loads(source('isolation-held-move-stop-native-01.json').read_text())
summary['native_move_capture']={'frames':len(frames),'states':dict(collections.Counter(f['documents']['selected_target.json']['target_state_name'] for f in frames)),'limitation':'The subject left the fixed field of view; invalid observations are not a detector failure rate.'}
for name in ['isolation-dwell-geometry-02.json','dwell-projection-geometry.json','isolation-held-move-stop-01.json']:source(name)
summary['geometry']=json.loads(args.geometry_summary.read_text())
summary['geometry_cross_validation']=json.loads(args.geometry_fits.read_text())
summary['live_verification']={}
for name in ['isolation-planner-live-01.json','isolation-planner-auto-01.json']:
    rows=unique(name)
    summary['live_verification'][name]={'unique_states':len(rows),'modes':dict(collections.Counter(s['operating_mode'] for s in rows)),'safety':dict(collections.Counter(s['safety_action'] for s in rows)),'faults':sorted({s['fault'] for s in rows if s['fault']})}
summary['implementation_sources']={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in map(Path,['Firmware/control/src/control/reference_limiter.hpp','Firmware/control/tests/test_reference_limiter.cpp','Firmware/tools/probe_tracking_boundaries.cpp','Firmware/tools/prepare_tracking_replay.py','Firmware/config/turret.yaml','Firmware/calibration/camera_intrinsics.yaml','Firmware/calibration/camera_extrinsics.yaml'])}
(out/'isolation-analysis.json').write_text(json.dumps(summary,indent=2))
print(json.dumps({k:v for k,v in summary.items() if k not in ['sources','geometry','geometry_cross_validation']},indent=2))
