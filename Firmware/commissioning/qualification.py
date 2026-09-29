"""Reproducible offline condition matrix. Results never qualify a physical device."""
from __future__ import annotations
import argparse
from concurrent.futures import ProcessPoolExecutor,as_completed
import fcntl
import json
from pathlib import Path
import time
import os
import numpy as np
from .calibrate import synthetic_dataset,fit_dataset
from .contracts import Rejected,write_immutable,digest
from .measurement import ObserverSpec
from .native import Native,Simulation
from .adaptation import Envelope
from .synthesis import solve
from .synthetic import CONDITIONS
from .provenance import method_identity


def case(output,axis,condition,reuse_fits):
    native=Native();started=time.monotonic()
    folder=output/axis/condition;folder.mkdir(parents=True,exist_ok=True)
    data=synthetic_dataset(axis,condition)
    dataset=write_immutable(folder/'datasets',data)
    def progress(stage,count):
        print(json.dumps({"axis":axis,"condition":condition,"stage":stage,"count":count,
                          "local_pid":os.getpid(),"elapsed_s":round(time.monotonic()-started,2)}),flush=True)
    progress('start',0)
    try:
        doc,snapshot,path=fit_dataset(native,dataset,folder,progress,reuse_fits=reuse_fits)
        truth=np.asarray(data["synthetic_truth"]);error=np.abs(snapshot.theta-truth)
        parameter_pass=(np.max(error[:3]/truth[:3])<.03 and np.max(error[3:6]/truth[3:6])<.08
                        and np.max(error[6:-1])<.005 and error[-1]<.002)
        candidate=solve(native,snapshot,ObserverSpec(**doc["observer"]),Envelope(**doc["envelope"]),
                        Simulation(**doc["simulation"]),latency_p99_s=doc["latency_p99_s"])
        candidate_path=write_immutable(folder/"candidates",candidate)
        row={"axis":axis,"condition":condition,"status":"PASS" if parameter_pass else "FAIL",
             "dataset":str(dataset),"snapshot":str(path),"candidate":str(candidate_path),
             "method_hash":candidate['method_hash'],"snapshot_hash":snapshot.identity_hash,
             "candidate_hash":digest(candidate),
             "a_max_relative_error":float(np.max(error[:3]/truth[:3])),
             "b_max_relative_error":float(np.max(error[3:6]/truth[3:6])),
             "h_max_error_A":float(np.max(error[6:-1])),"delay_error_s":float(error[-1]),
             "bootstrap_models":128,"case_evaluations":candidate["case_evaluations"],
             "point_id":candidate["point_id"],"omega_n_rad_s":candidate["omega_n_rad_s"],
             "phase_margin_deg":candidate["worst_phase_margin_deg"],
             "gain_margin_db":candidate["worst_gain_margin_db"]}
    except Rejected as exc:
        row={"axis":axis,"condition":condition,"status":"FAIL","reason":exc.reason.value,"detail":exc.detail}
    except Exception as exc:
        row={"axis":axis,"condition":condition,"status":"FAIL","reason":"DATA_INVALID",
             "detail":f"local software execution failed: {type(exc).__name__}: {exc}"}
    progress('result',row['status'])
    return row


def run(output,axes,conditions,reuse_fits=False,jobs=1):
    native=Native();results=[];started=time.monotonic()
    output.mkdir(parents=True,exist_ok=True)
    method=method_identity(native)
    write_immutable(output/'methods',method)
    tasks=[(output,axis,condition,reuse_fits) for axis in axes for condition in conditions]
    def record(row):
        results.append(row)
        report={"execution":"SYNTHETIC_LOCAL_ONLY","physical_access":False,
                "physical_qualification":"NOT_RUN","stage2_started":False,
                "method":method,"method_hash":digest(method),"expected_cases":len(tasks),
                "results":sorted(results,key=lambda r:(axes.index(r['axis']),conditions.index(r['condition']))),
                "elapsed_s":time.monotonic()-started}
        pending=output/'matrix.pending.json'
        pending.write_text(json.dumps(report,indent=2,allow_nan=False),encoding='utf-8')
        pending.replace(output/'matrix.json')
    if jobs==1:
        for task in tasks:record(case(*task))
    else:
        with ProcessPoolExecutor(max_workers=jobs) as pool:
            futures=[pool.submit(case,*task) for task in tasks]
            for future in as_completed(futures):record(future.result())
    return len(results)==len(tasks) and all(r['status']=='PASS' for r in results)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--axes',nargs='+',choices=('yaw','pitch'),default=['yaw','pitch'])
    parser.add_argument('--conditions',nargs='+',choices=CONDITIONS,default=list(CONDITIONS))
    parser.add_argument('--reuse-fits',action='store_true',help='reuse exact immutable assets after method/data/holdout verification')
    parser.add_argument('--jobs',type=int,choices=range(1,9),default=1,help='independent local mathematical workers')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    if len(set(args.axes))!=len(args.axes) or len(set(args.conditions))!=len(args.conditions):
        parser.error('duplicate cases are not independent evidence')
    with (args.output/'.qualification.lock').open('a') as guard:
        try:fcntl.flock(guard,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:parser.error('another local matrix owns this output directory')
        return 0 if run(args.output,args.axes,args.conditions,args.reuse_fits,args.jobs) else 2


if __name__=='__main__':raise SystemExit(main())
