#!/usr/bin/env python3
"""Offline consistency check for two settled-scene correspondence sets. Writes no calibration."""
import argparse
import json
from pathlib import Path
import numpy as np

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('capture_directory',type=Path)
parser.add_argument('output',type=Path)
args=parser.parse_args()
root=args.capture_directory
datasets=[json.loads((root/name).read_text()) for name in ['dwell-projection-geometry.json','isolation-dwell-geometry-02.json']]
R0=np.array([[0.,1,0],[-1,0,0],[0,0,1]])
def ry(q):return np.array([[np.cos(q),0,np.sin(q)],[0,1,0],[-np.sin(q),0,np.cos(q)]])
def rz(q):return np.array([[np.cos(q),-np.sin(q),0],[np.sin(q),np.cos(q),0],[0,0,1]])
def residuals(p,rows):
    fx,fy=np.exp(p[:2]); K=np.array([[fx,0,960],[0,fy,540],[0,0,1.]])
    RPC=ry(p[2])@R0
    result=[]
    for row in rows:
        old,q=row['old_q'],row['q']
        Ro=rz(old[0])@ry(old[1])@RPC; Rn=rz(q[0])@ry(q[1])@RPC
        a=np.c_[row['a'],np.ones(len(row['a']))]
        projected=a@(K@Rn.T@Ro@np.linalg.inv(K)).T
        result.append(projected[:,:2]/projected[:,2:]-np.array(row['b']))
    return result
def fit(rows):
    p=np.array([np.log(1389),np.log(1467),-.86])
    def r(p):return np.concatenate(residuals(p,rows)).ravel()
    for _ in range(60):
        value=r(p); weight=np.minimum(1,4/np.maximum(np.abs(value),1e-9))
        jac=np.stack([(r(p+np.eye(3)[k]*1e-5)-value)/1e-5 for k in range(3)],axis=1)
        change=np.linalg.solve(jac.T@(jac*weight[:,None])+1e-6*np.eye(3),-jac.T@(value*weight))
        cost=lambda v: np.sum(np.where(abs(v)<4,.5*v*v,4*(abs(v)-2)))
        step=1.
        while step>1e-5 and cost(r(p+step*change))>cost(value):step*=.5
        p+=step*change
        if np.linalg.norm(step*change)<1e-8:break
    return p
out=[]
baseline=np.array([np.log(1389),np.log(1467),-.86])
for label,rows in [('old',datasets[0]),('new',datasets[1]),('both',sum(datasets,[]))]:
    p=fit(rows)
    record={'fit_on':label,'fx':float(np.exp(p[0])),'fy':float(np.exp(p[1])),'pitch_offset_rad':float(p[2]),'evaluations':[]}
    for name,data in zip(['old','new'],datasets):
        b=residuals(baseline,data);a=residuals(p,data)
        record['evaluations'].append({'dataset':name,'baseline_median_by_move_px':[float(np.median(np.linalg.norm(x,axis=1))) for x in b], 'candidate_median_by_move_px':[float(np.median(np.linalg.norm(x,axis=1))) for x in a]})
    out.append(record)
print(json.dumps(out,indent=2))
args.output.write_text(json.dumps(out,indent=2))
