"""Local analysis entry. There are deliberately no deploy/connect/move subcommands."""
from __future__ import annotations
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import time

import numpy as np

from .contracts import Identity, ModelSpec, PlantSnapshot, Reason, Rejected, write_immutable
from .identification import Run, identify, check_split, validation_report
from .measurement import ObserverSpec
from .native import Native, Simulation
from .adaptation import Envelope
from .synthesis import solve


def load_dataset(path):
    doc=json.loads(Path(path).read_text(encoding="utf-8"))
    spec=ModelSpec(**doc["model_spec"]);identity=Identity(**doc["identity"])
    if "raw_train" in doc or "raw_holdout" in doc:
        from .normalization import normalize_raw
        from .contracts import require
        require("train" not in doc and "holdout" not in doc,Reason.DATA_INVALID,
                "supply raw or normalized records, never competing copies")
        train=[normalize_raw(row,doc["calibration"],identity) for row in doc["raw_train"]]
        holdout=[normalize_raw(row,doc["calibration"],identity) for row in doc["raw_holdout"]]
        return doc,spec,identity,train,holdout
    groups=[]
    for group in ("train","holdout"):
        group_runs=[]
        for row in doc[group]:
            values={**row,"identity":identity}
            # Earlier synthetic records declare the same observation filter in
            # their simulation contract. Missing in BOTH places remains an error.
            if "gyro_filter_tau_s" not in values:
                values["gyro_filter_tau_s"]=doc["simulation"]["gyro_filter_tau"]
            for k in ("t","q","v","tx","z","direction","q_new","v_new"):
                values[k]=np.asarray(values[k],dtype=bool if k.endswith("_new") else float)
            group_runs.append(Run(**values))
        groups.append(group_runs)
    return doc,spec,identity,*groups


def synthetic_dataset(axis,condition):
    from .synthetic import fixture,runs
    from .breakaway import assemble_intervals
    from .probe_control import fixture_control
    spec,theta,identity=fixture(axis,condition)
    _,observer,envelope,simulation=fixture_control(axis)
    # Inject the same declared noise into causal feedback during qualification.
    # Additional quantization is explicit and independent of the fitter's noise.
    simulation.encoder_noise=2e-5;simulation.gyro_noise=8e-5;simulation.encoder_quantum=1e-5
    records=[]
    for di,d in enumerate((-1,1)):
        for iz in range(3):
            for iq in range(len(spec.q_nodes)):
                load=theta[6:-1].reshape(2,3,5)[di,iz,iq]
                t=np.arange(0,1,.005);u=load+d*.03*t
                moving=t>.5
                v=np.where(moving,d*.012,0.);q=np.cumsum(v)*.005
                records.append({"direction":d,"posture_index":iz,"position_index":iq,
                                "trace":{"time":t,"successful_tx":u,"q":q,"velocity":v,
                                         "sigma_velocity":8e-5,"encoder_quantum":1e-5}})
    intervals,censored=assemble_intervals(spec,records)
    def rows(runs):
        out=[]
        for run in runs:
            row=asdict(run);row.pop("identity")
            out.append({k:v.tolist() if isinstance(v,np.ndarray) else v for k,v in row.items()})
        return out
    return {"model_spec":asdict(spec),"identity":asdict(identity),
            "train":rows(runs(spec,theta,identity,seed=2202)),
            "holdout":rows(runs(spec,theta,identity,seed=2203,repetitions=2)),
            "breakaway_intervals":intervals.tolist(),"breakaway_censored":censored.tolist(),
            "delay_search_bound_s":.025,"frequency_band_hz":[.5,15.],
            "observer":asdict(observer),"envelope":asdict(envelope),
            "simulation":{k:getattr(simulation,k) for k,_ in Simulation._fields_},
            "latency_p99_s":.008,"synthetic_truth":theta.tolist()}


def fit_dataset(native,path,output,progress,*,reuse_fits=False):
    from .provenance import method_identity,identification_component
    from .contracts import digest
    method=method_identity(native);current_method=digest(method)
    component=identification_component(method)
    doc,spec,identity,train,holdout=load_dataset(path)
    if reuse_fits:
        check_split(train,holdout)
        for cached in sorted((output/"plants").glob("*.json")):
            try:
                snapshot=PlantSnapshot.bind(json.loads(cached.read_text()),spec,identity)
                previous=snapshot.fit_report.get('method_hash')
                same_fit=snapshot.fit_report.get('identification_component_hash')==component
                manifest_path=output.parent.parent/'methods'/f'{previous}.json'
                if not same_fit and manifest_path.is_file():
                    manifest=json.loads(manifest_path.read_text(encoding='utf-8'))
                    same_fit=(digest(manifest)==previous and identification_component(manifest)==component)
                if (same_fit and
                    snapshot.train_hashes==tuple(r.hash for r in train) and
                    snapshot.holdout_hashes==tuple(r.hash for r in holdout) and
                    np.array_equal(snapshot.start_intervals,np.asarray(doc["breakaway_intervals"])) and
                    snapshot.fit_report.get("bootstrap_runs")==128 and
                    all(r["passed"] for r in validation_report(native,spec,snapshot.theta,holdout))):
                    progress("reuse_exact_verified_whole_run_asset",cached.name)
                    return doc,snapshot,cached
            except (Rejected,KeyError,ValueError):
                continue
    result=identify(native,spec,train,holdout,delay_bound_s=doc["delay_search_bound_s"],progress=progress)
    result["report"]["method_hash"]=current_method
    result["report"]["identification_component_hash"]=component
    snapshot=PlantSnapshot(spec,identity,result["theta"],result["uncertainty"],
                           tuple(r.hash for r in train),tuple(r.hash for r in holdout),
                           tuple(doc["frequency_band_hz"]),doc["breakaway_intervals"],
                           np.asarray(doc["breakaway_censored"],bool),result["report"])
    asset=write_immutable(output/"plants",snapshot.document())
    return doc,snapshot,asset


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest="command",required=True)
    synth=sub.add_parser("synthetic-data")
    synth.add_argument("--axis",choices=("yaw","pitch"),required=True)
    synth.add_argument("--condition",required=True)
    synth.add_argument("--output",type=Path,required=True)
    fit=sub.add_parser("fit");fit.add_argument("--dataset",type=Path,required=True)
    fit.add_argument("--output",type=Path,required=True)
    solve_cmd=sub.add_parser("solve");solve_cmd.add_argument("--dataset",type=Path,required=True)
    solve_cmd.add_argument("--snapshot",type=Path,required=True)
    solve_cmd.add_argument("--output",type=Path,required=True)
    args=parser.parse_args();start=time.monotonic()
    try:
        if args.command=="synthetic-data":
            from .synthetic import CONDITIONS
            if args.condition not in CONDITIONS:parser.error("unknown synthetic operating condition")
            args.output.parent.mkdir(parents=True,exist_ok=True)
            args.output.write_text(json.dumps(synthetic_dataset(args.axis,args.condition),allow_nan=False),encoding="utf-8")
            print(json.dumps({"dataset":str(args.output),"provenance":"SYNTHETIC"}));return 0
        native=Native()
        def progress(stage,count):
            print(json.dumps({"stage":stage,"count":count,"elapsed_s":round(time.monotonic()-start,2)}),flush=True)
        if args.command=="fit":
            _,snapshot,path=fit_dataset(native,args.dataset,args.output,progress)
            print(json.dumps({"snapshot":str(path),"hash":snapshot.identity_hash,"elapsed_s":time.monotonic()-start}));return 0
        doc,spec,identity,_,_=load_dataset(args.dataset)
        snapshot=PlantSnapshot.bind(json.loads(args.snapshot.read_text()),spec,identity)
        candidate=solve(native,snapshot,ObserverSpec(**doc["observer"]),Envelope(**doc["envelope"]),
                        Simulation(**doc["simulation"]),latency_p99_s=doc["latency_p99_s"])
        path=write_immutable(args.output/"candidates",candidate)
        print(json.dumps({"candidate":str(path),"elapsed_s":time.monotonic()-start}));return 0
    except Rejected as exc:
        print(json.dumps({"status":"REJECTED","reason":exc.reason.value,"detail":exc.detail,
                          "physical_qualification":"NOT_RUN"}));return 2
    except (KeyError,TypeError,ValueError,OSError) as exc:
        print(json.dumps({"status":"REJECTED","reason":"DATA_INVALID","detail":str(exc),
                          "physical_qualification":"NOT_RUN"}));return 2


if __name__=="__main__":raise SystemExit(main())
