"""Calculate a provisional yaw feedback probe from measured data; never drive CAN."""
from __future__ import annotations

import argparse
from dataclasses import asdict
from copy import copy
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from Firmware.commissioning.adaptation import Envelope
from Firmware.commissioning.contracts import Identity, Reason, Rejected, require
from Firmware.commissioning.identification import integral_design
from Firmware.commissioning.measurement import estimate_observer
from Firmware.commissioning.metrics import shaped_step
from Firmware.commissioning.native import Native, Simulation, CONTROL_FIELDS, ACCELERATION_FIELDS, acceleration_values
from Firmware.commissioning.parameter_catalog import core_registry, bind_core_profile
from Firmware.commissioning.synthesis import solve
from adr0022_yaw_information import fitted_training_runs,read_stream,REFERENCE_COUNT


def save(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+"\n", encoding="utf-8")


def export_control_manifest(fit, runtime, envelope, candidate, output_dir, *, pose_journal=None):
    """Bind calculated parameters to one bounded measured out/back experiment."""
    pose_journal=Path(pose_journal or fit["training_journals"][-1])
    with pose_journal.open() as source:
        pose_rows=[json.loads(line) for line in source]
        last=next(row for row in reversed(pose_rows)
                  if row.get("kind")=="yaw_feedback")
    pitch_rows=[row for row in pose_rows if row.get("kind")=="register_read" and
                row.get("index")==0x7019 and row.get("value") is not None]
    posture=float(pitch_rows[-1]["value"]) if pitch_rows else float(fit["fixed_pitch_rad"])
    initial=float(((last["encoder_raw"]-REFERENCE_COUNT+4096)%8192-4096)*2*np.pi/8192)
    distance=float(np.deg2rad(5.))
    _,_,out=shaped_step(distance,envelope,position=initial,
                        posture=posture)
    _,_,back=shaped_step(-distance,envelope,position=initial+distance,
                         posture=posture)
    manifest=json.loads((Path(fit["training_journals"][0]).parent/"manifest.json").read_text())
    for key in ("current_segments","initial_level","signal_selection","acquisition_target",
                "output","imu_fd"):
        manifest.pop(key,None)
    manifest.update(schema="adr0022.yaw-control/1",purpose="yaw_shared_core_3a",
        session_label=runtime["session_label"],candidate_label=runtime["session_label"]+"-point-"+str(candidate["point_id"]),
        source_description="calculated provisional feedback; workstation ARM64 shared core; one physical out/back probe",
        controller_parameters=runtime["controller_parameters"],
        gyro_calibration=runtime["gyro_calibration"],
        other_axis_posture_rad=posture,
        yaw_position_offset_rad=initial,
        reference_segments=[
            {"duration_s":out["zero_reference_time"]-out["command_time"],"target_position_rad":initial+distance},
            {"duration_s":3.,"target_position_rad":initial+distance},
            {"duration_s":back["zero_reference_time"]-back["command_time"],"target_position_rad":initial},
            {"duration_s":3.,"target_position_rad":initial}],
        reference_frame={"encoder_reference_count":REFERENCE_COUNT,
            "source_journal":str(pose_journal),"last_encoder_raw":last["encoder_raw"],
            "last_receipt_ns":last["kernel_monotonic_ns"],
            "scope":"last measured yaw phase; fresh first raw encoder is recorded for alignment assessment"},
        qualification={"candidate":"OFFLINE_PROVISIONAL_ONLY","physical":"NOT_RUN",
            "formal_promotion_eligible":False,"open_loop_prediction_qualified":False})
    from adr0022_capture_launch import validate_yaw_control_contract
    validate_yaw_control_contract(manifest)
    save(Path(output_dir)/"control-manifest.json",manifest)
    return manifest


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fit-json", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--session-label", required=True)
    parser.add_argument("--pose-journal", type=Path,
                        help="latest measured raw yaw journal for the reference frame")
    parser.add_argument("--acceleration-cap-rad-s2", type=float,
                        help="explicit owner acceleration cap; otherwise use the fit context binding")
    parser.add_argument("--acceleration-noise-sigma-rad-s2", type=float,
                        help="actual measured acceleration noise sigma; otherwise use the fit context observation")
    parser.add_argument("--acceleration-sample-period-s", type=float,
                        help="actual measured gyro sample interval; otherwise use the fit context observation")
    parser.add_argument("--acceleration-current-window-enabled", type=float, choices=(0.,1.),
                        help="owner guidance defaults to 0; reference shaping and derivative telemetry remain active")
    args=parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    fit=json.loads(args.fit_json.read_text())
    supplied_guard={"acceleration_cap":args.acceleration_cap_rad_s2,
                    "acceleration_noise_sigma":args.acceleration_noise_sigma_rad_s2,
                    "acceleration_sample_period_s":args.acceleration_sample_period_s,
                    "acceleration_current_window_enabled":args.acceleration_current_window_enabled}
    measured=fit.get("acceleration_guard_observation",{})
    measured_guard={"acceleration_cap":measured.get("acceleration_cap_rad_s2"),
                    "acceleration_noise_sigma":measured.get("acceleration_noise_sigma_rad_s2"),
                    "acceleration_sample_period_s":measured.get("acceleration_sample_period_s"),
                    "acceleration_current_window_enabled":0.}
    guard={key:(supplied_guard[key] if supplied_guard[key] is not None else measured_guard[key])
           for key in ACCELERATION_FIELDS}
    if any(value is None for value in guard.values()):
        parser.error("yaw synthesis needs the explicit owner acceleration cap and actual measured noise/sample period in the fit context or CLI")
    guard,guard_provenance=acceleration_values(guard)
    if guard["acceleration_cap"]<=0:
        parser.error("live yaw synthesis requires a positive acceleration cap")
    guard_sources={key:("EXPLICIT_CLI" if supplied_guard[key] is not None else
                        "OWNER_GUIDANCE_MODE" if key=="acceleration_current_window_enabled" else "MEASURED_FIT_CONTEXT")
                   for key in ACCELERATION_FIELDS}
    acceleration_binding={"binding":guard_provenance,"sources":guard_sources,
                          "actual_observation_context":measured,
                          "included_in_offline_native_simulations":True}
    spec,runs,descriptions=fitted_training_runs(fit,args.fit_json)
    theta=np.asarray(fit["theta"],float)
    model_scope={"source_context":str(args.fit_json),"local_only":fit.get("local_only",False),
        "local_q_support_rad":fit.get("local_q_support_rad"),
        "actual_posture_support_rad":fit.get("actual_posture_support_rad"),
        "correction_domain":fit.get("correction_domain"),
        "failure_classification":fit.get("failure_classification"),
        "source_prediction_reports":fit.get("update_report",{}).get("raw_prediction_errors"),
        "full_qualification":False,"formal_qualification":False,
        "gyro_calibration_refit":False,"holdout_used_for_fit_or_stress":False}
    evaluation_domain=({"q_min_rad":float(fit["local_q_support_rad"][0]),
        "q_max_rad":float(fit["local_q_support_rad"][1]),"fixed_pitch_rad":float(fit["fixed_pitch_rad"]),
        "scope":"Actual measured local q range and latest actual pitch; unseen positions/postures unqualified"}
        if fit.get("local_only") else None)
    # Align the integral input with the same fitted command-to-motion delay.
    # Residuals are load-equivalent stress observations, not a friction attribution.
    delayed=[]
    for run in runs:
        indices=np.searchsorted(run.tx_history_t,run.t-theta[-1],side="right")-1
        require(np.all(indices>=0),Reason.DATA_INVALID,
                "actual pre-window successful TX history does not cover the fitted command delay")
        aligned=copy(run)
        aligned.tx=run.tx_history_A[indices]
        delayed.append(aligned)
    integral_window_s=float(fit.get("update_report",{}).get("initializer_integral_window_s",.15))
    X,y,groups=integral_design(spec,delayed,window_s=integral_window_s)
    durations=X[:,6:].sum(axis=1)
    load_error=(y-X@theta[:-1])/durations
    directions=np.asarray([runs[index].direction[0] for index in groups])
    bounds={str(direction):[float(load_error[directions==direction].min()),
                           float(load_error[directions==direction].max())]
            for direction in (-1,1)}
    stress=[theta.copy()]
    count=3*len(spec.q_nodes)
    for negative in bounds["-1"]:
        for positive in bounds["1"]:
            row=theta.copy()
            row[6:6+count]+=negative
            row[6+count:-1]+=positive
            stress.append(row)
    measured_alternatives=fit.get("measured_model_alternatives",[])
    for alternative in measured_alternatives:
        if alternative["same_frozen_model_and_gyro_calibration"]:
            stress.append(np.asarray(alternative["theta"],float))
    identity=Identity("rpi-turret-yaw-GM6020","raw-encoder-5768-fixed-pitch-gyro-column",
                      "current-payload-fixed-pitch", "MEASURED",provisional_labels=True)
    # Static covariance uses only actual training receipts before excitation.
    # Dynamic innovation fitting evaluates encoder interpolation at actual gyro times.
    static_q,static_v=[],[]
    dynamic=[]
    calibration=fit["gyro_calibration"]
    column=np.asarray(calibration["yaw_column"])
    projection=column/(column@column)
    bias=np.asarray(calibration["baseline_sensor_bias"])
    for journal in fit["training_journals"]:
        stream=read_stream(Path(journal))
        yy,gg,cc=stream["yaw"],stream["gyro"],stream["commands"]
        yt=np.asarray([row["kernel_monotonic_ns"] for row in yy],np.int64)
        gt=np.asarray([row["sample_ns"] for row in gg],np.int64)
        ct=np.asarray([row["kernel_accepted_ns"] for row in cc],np.int64)
        current=np.asarray([row["successful_tx_A"] for row in cc])
        first=int(ct[np.flatnonzero(np.abs(current)>0)[0]])
        q=np.asarray([row["q_relative_rad"]+stream["q_offset"] for row in yy])
        v=(np.asarray([row["values"] for row in gg])-bias)@projection
        restq=q[yt<first]
        static_q.extend(restq-restq.mean())
        static_v.extend(v[gt<first])
        valid=(gt>=yt[0])&(gt<=yt[-1])
        tt=(gt[valid]-gt[valid][0])/1e9
        dynamic.append((str(journal),tt,np.interp(gt[valid],yt,q),v[valid]))
    observer_journal,dynamic_t,dynamic_q,dynamic_v=max(dynamic,key=lambda row:len(row[1]))
    observer=estimate_observer(dynamic_t,dynamic_q,dynamic_v,static_q,static_v,
                               measurement_hash=identity.measurement,provenance="MEASURED")
    # These are requested motion limits from the installed station configuration.
    # They do not certify the motor's attainable acceleration or sensor bandwidth.
    import yaml
    config=yaml.safe_load(Path("Firmware/config/turret_mixed.yaml").read_text())
    requested=config["axes"]["yaw"]
    capture=json.loads((Path(fit["training_journals"][0]).parent/"manifest.json").read_text())
    cap=float(capture["yaw_current_bound_A"])
    if "current_segments" in capture:
        slew=max(abs(float(s["end_A"])-float(s["start_A"]))/float(s["duration_s"])
                 for s in capture["current_segments"])
    else:
        slew=float(fit["actual_controller_parameter_readbacks"][0]["slew"])
    envelope=Envelope(cap,slew,np.deg2rad(requested["max_velocity_deg_s"]),
                      np.deg2rad(requested["max_acceleration_deg_s2"]),
                      np.deg2rad(requested["max_jerk_deg_s3"]),None,None,40.,False,"MEASURED")
    sensor_path=Path("run/adr0022-stage2/yaw-sensor-capability-01/sensor-capability-report.json")
    sensor=json.loads(sensor_path.read_text())
    band=sensor.get("measured_coherent_band",sensor.get("bandwidth",{}))
    # The existing report's exact key is checked before any candidate calculation.
    if "valid_band_hz" not in band:
        for value in sensor.values():
            if isinstance(value,dict) and "valid_band_hz" in value:
                band=value;break
    band=tuple(band["valid_band_hz"])
    if fit.get("supplemental_bandwidth_file"):
        band=tuple(json.loads(Path(fit["supplemental_bandwidth_file"]).read_text())["spectral_result"]["valid_band_hz"])
    latency=float(sensor["sensor_timing"]["gyro"]["producer_receipt_minus_sample_s"]["p99"])
    # Unmeasured startup cells retain censoring. The finite cap is a requested
    # startup hypothesis; it is never labelled a measured threshold map.
    starts=np.empty((2,3,len(spec.q_nodes),2))
    starts[...,0]=0.
    starts[...,1]=np.asarray([-cap,cap])[:,None,None]
    startup_basis="bounded requested current hypothesis; cells remain censored"
    if fit.get("startup_empirical_prior") and fit.get("startup_spatial_coverage_qualified") is True:
        for index,direction in enumerate((-1,1)):
            starts[index,...,1]=fit["startup_empirical_prior"][str(direction)]["signed_total_current_A"]
        startup_basis="spatially qualified observed training startup endpoint per direction"
    snapshot=SimpleNamespace(spec=spec,identity=identity,theta=theta,
        uncertainty=np.asarray(stress),frequency_band_hz=band,start_intervals=starts,
        start_censored=np.ones((2,3,len(spec.q_nodes)),bool),
        train_hashes=tuple(fit["training_journals"]),holdout_hashes=(fit["holdout_journal"],),
        fit_report={"source":str(args.fit_json),"open_loop_prediction_qualified":False,
                    "uncertainty_method":"empirical training-window load-equivalent residual extrema",
                    "statistical_confidence":None,"stress_cases":len(stress),
                    "integral_window_s":integral_window_s,
                    "input_history":"Actual successful pre-window TX events; ZOH at observation time minus fitted command delay",
                    "load_equivalent_residual_bounds_A":bounds,
                    "measured_model_alternatives":measured_alternatives,
                    "dynamic_uncertainty_method":"Complete exact earlier measured theta/delay alternatives plus current empirical load-equivalent stress; no bootstrap confidence",
                    "startup_basis":startup_basis,
                    "fixed_pitch_only":True})
    simulation=Simulation(.005,2*np.pi/8192,np.sqrt(observer.encoder_variance),
        np.sqrt(observer.gyro_variance),latency,0.,1,4,22)
    inputs={"schema":"adr0022.yaw-feedback-probe-inputs/1","session_label":args.session_label,
        "model_spec":asdict(spec),"theta":theta.tolist(),"stress_models":np.asarray(stress).tolist(),
        "stress_basis":snapshot.fit_report,"observer":asdict(observer),"envelope":asdict(envelope),
        "training_runs":descriptions,"holdout_used_for_fit_or_stress":False,
        "static_encoder_receipts":len(static_q),"static_gyro_receipts":len(static_v),
        "observer_dynamic_journal":observer_journal,
        "observer_scope":"one actual continuous receipt sequence; static covariance pooled from training baselines only",
        "acceleration_binding":guard,"acceleration_binding_provenance":acceleration_binding,
        "source_model_scope":model_scope,
        "evaluation_domain":evaluation_domain,
        "physical_qualification":"NOT_RUN","model_prediction_qualified":False}
    save(args.output_dir/"synthesis-inputs.json",inputs)
    native=Native(Path(fit["native_library"]))
    try:
        candidate=solve(native,snapshot,observer,envelope,simulation,
                        latency_p99_s=latency,session_label=args.session_label,
                        acceleration=guard,evaluation_domain=evaluation_domain)
    except Rejected as exc:
        save(args.output_dir/"synthesis-result.json",{"status":"REJECTED","reason":exc.reason.value,
             "detail":exc.detail,"physical_execution":False})
        print(json.dumps({"status":"REJECTED","reason":exc.reason.value,"detail":exc.detail}))
        return
    candidate["acceleration_binding_provenance"]=acceleration_binding
    candidate["source_model_scope"]=model_scope
    save(args.output_dir/"candidate.json",candidate)
    fields,current=core_registry(snapshot,observer,candidate["runtime_values"])
    bound=bind_core_profile(native,snapshot,observer,current)
    scalar={key:float(getattr(bound,key)) for key in CONTROL_FIELDS+ACCELERATION_FIELDS}
    parameters={"model":{"n":bound.model.n,"periodic":bound.model.periodic,
                           "q":list(spec.q_nodes),"z":list(spec.z_nodes),"theta":list(bound.model.theta)[:spec.size]},
        "observer":{key:getattr(bound.observer,key) for key,_ in bound.observer._fields_},
        **scalar,"start_total":list(bound.start_total)[:6*len(spec.q_nodes)],
        "start_censored":list(bound.start_censored)[:6*len(spec.q_nodes)]}
    runtime={"session_label":args.session_label,
        "controller_parameters":parameters,"gyro_calibration":fit["gyro_calibration"],
        "other_axis_posture_rad":fit["fixed_pitch_rad"],
        "registry_fields":{key:asdict(field) for key,field in fields.items()},
        "registry_values":current,"native_configuration_accepted":True,
        "acceleration_binding_provenance":candidate["acceleration_binding_provenance"],
        "source_model_scope":model_scope,
        "physical_parameter_readback":"NOT_RUN","physical_qualification":"NOT_RUN"}
    save(args.output_dir/"runtime-parameters.json",runtime)
    export_control_manifest(fit,runtime,envelope,candidate,args.output_dir,pose_journal=args.pose_journal)
    print(json.dumps({"status":"OFFLINE_CANDIDATE_CALCULATED","point_id":candidate["point_id"],
                      "runtime_values":candidate["runtime_values"],"physical_qualification":"NOT_RUN"}))


if __name__=="__main__":
    main()
