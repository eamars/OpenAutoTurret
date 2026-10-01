"""Recorded yaw velocity validation using the existing frozen metrics."""
from __future__ import annotations

from collections import Counter
import json
import math
import numpy as np
from scipy.spatial.transform import Rotation

from .contracts import Rejected
from .metrics import motion_metrics

TRACE_FIELDS = ("elapsed_s", "encoder_q_rad", "projected_gyro_rad_s",
                "encoder_local_velocity_rad_s", "reference_q_rad", "reference_v_rad_s",
                "reference_a_rad_s2", "core_motion", "core_status", "requested_A",
                "successful_tx_A")


def numerical_values(value):
    """Use the existing captured-numeric-string comparison convention."""
    if isinstance(value, dict):
        return {key:numerical_values(item) for key,item in value.items()}
    if isinstance(value, list):
        return [numerical_values(item) for item in value]
    if isinstance(value, str):
        try:
            return float(value)
        except ValueError:
            return value
    return value


def _motion_transitions(cycles, tx, observed, manifest, params, begin, zero, half):
    """The recorded13 transition/quiet observation, with unchanged60ms support."""
    names = {0:"REST", 1:"START", 2:"MOVE", 3:"STOP", 4:"REVERSE"}
    anchor = float(manifest["yaw_position_offset_rad"])
    trace = []
    for index,(cycle,sent,row) in enumerate(zip(cycles,tx,observed)):
        core,reference = cycle["core"],cycle["reference"]
        proportional = core["requested_A"]-core["integral_A"]-core["feedforward_A"]-core["start_increment_A"]
        trace.append({
            "sequence":cycle["sequence"], "time_ns":cycle["time_ns"], "elapsed_s":float(row[0]),
            "motion":names[core["motion"]], "phase":sent["phase"],
            "actual_encoder_from_start_deg":math.degrees(row[1]-anchor),
            "actual_gyro_deg_s":math.degrees(row[2]),
            "encoder200ms_velocity_deg_s":math.degrees(row[3]) if half<=index<len(observed)-half else None,
            "encoder200ms_velocity_valid":half<=index<len(observed)-half,
            "core_velocity_deg_s":math.degrees(core["velocity_rad_s"]),
            "fresh_gyro_input_deg_s":math.degrees(cycle["observation"]["gyro_rate_rad_s"]),
            "reference_velocity_deg_s":math.degrees(reference["velocity_rad_s"]),
            "reference_encoder_error_deg":math.degrees(reference["position_rad"]-row[1]),
            "effective_core_reference_velocity_deg_s":math.degrees(proportional/params["kp"]+core["velocity_rad_s"]),
            "integral_A":core["integral_A"], "feedforward_A":core["feedforward_A"],
            "proportional_A":proportional, "startup_increment_A":core["start_increment_A"],
            "requested_A":core["requested_A"], "limited_A":core["limited_A"],
            "actual_successful_TX_A":sent["successful_tx_A"], "TX_kernel_accepted_ns":sent["kernel_accepted_ns"]})
    groups = []
    for row in trace:
        if not groups or groups[-1][0]["motion"] != row["motion"]:
            groups.append([])
        groups[-1].append(row)
    releases = []
    for index,group in enumerate(groups):
        if index and group[0]["motion"] == "MOVE" and groups[index-1][0]["motion"] == "START":
            releases.append({"prior_START":groups[index-1][-1], "release":group[0], "end":group[-1],
                "next":groups[index+1][0] if index+1<len(groups) else None,
                "MOVE_support_s":(group[-1]["time_ns"]-group[0]["time_ns"])*1e-9,
                "actual_encoder_displacement_deg":group[-1]["actual_encoder_from_start_deg"]-group[0]["actual_encoder_from_start_deg"]})
    rest = math.degrees(params["rest_speed"])
    floor = math.degrees(3*math.sqrt(2*params["observer"]["encoder_variance"]))
    times = np.asarray([row["time_ns"] for row in trace],np.int64)
    positions = np.asarray([row["actual_encoder_from_start_deg"] for row in trace])
    quiet = []
    for row in trace:
        earlier = np.interp(row["time_ns"]-round(params["sustained_s"]*1e9),times,positions)
        if (row["phase"] == "control" and row["motion"] in ("START","MOVE")
                and abs(row["actual_gyro_deg_s"]) <= rest
                and abs(row["actual_encoder_from_start_deg"]-earlier) <= floor):
            quiet.append(row)
    profile = manifest["reference_profile"]
    pb,pe = float(profile["plateau_begin_s"]),float(profile["plateau_end_s"])
    plateau = [row for row in trace if pb<=row["elapsed_s"]<pe]
    quiet_plateau = [row for row in quiet if pb<=row["elapsed_s"]<pe]
    selected = {row["sequence"] for row in quiet_plateau}
    exposure = sum((b["time_ns"]-a["time_ns"])*1e-9 for a,b in zip(trace,trace[1:]) if a["sequence"] in selected)
    stop = [row for row in trace if row["phase"] == "controlled_stop"]
    return {"schema":"adr0022.actual-velocity-motion-transitions/1",
        "starts":[group[0] for group in groups if group[0]["motion"] == "START"],
        "START_to_MOVE_releases":releases,
        "all_motion_groups":[{"motion":group[0]["motion"],"begin":group[0],"end":group[-1],"cycles":len(group)} for group in groups],
        "plateau_quiet_samples":quiet_plateau, "plateau_quiet_timestamp_exposure_s":exposure,
        "plateau_fastest_200ms_encoder_velocity_sample":max(plateau,key=lambda row:row["encoder200ms_velocity_deg_s"]),
        "plateau_fastest_independent_gyro_sample":max(plateau,key=lambda row:row["actual_gyro_deg_s"]),
        "sampled_low_motion_convention":{"existing_gyro_rest_deg_s":rest,
            "existing_raw_encoder_travel_floor_deg":floor,"window_s":params["sustained_s"],"qualification_gate":False},
        "controlled_stop":{"first":stop[0] if stop else None,"last":stop[-1] if stop else None,
            "cycles":len(stop),"zero_observation_begin_elapsed_s":(zero-begin)*1e-9},
        "local_velocity_scope":"The unchanged200ms convolution has full support away from capture endpoints. Unsupported transition samples are null; raw convolution remains in CSV. Plateau metrics are unchanged.",
        "scope":"Recorded references/current components, successful TX times, raw encoder and frozen independent gyro. Existing60ms noise convention retained; no fit or qualification."}


def analyze_yaw_velocity(rows, *, manifest=None, bandwidth_context, expected_parameters=None):
    """Return (report, CSV rows) with the actual12/13 analysis definitions.

    Bandwidth context and expected parameters are explicit existing evidence.
    No model, calibration, gains, thresholds or physical qualification are made.
    """
    if manifest is None:
        manifest = json.loads(rows[0]["manifest_yaml"])
    profile = manifest["reference_profile"]
    begin = next(row["time_ns"] for row in rows if row["kind"] == "yaw_control_begin")
    zero = next(row["time_ns"] for row in rows if row["kind"] == "stop_observation_begin")
    params = next(row["parameters"] for row in rows if row["kind"] == "controller_parameters_readback")
    cycles = [row for row in rows if row["kind"] == "yaw_control_cycle"]
    tx = [row for row in rows if row["kind"] == "yaw_current_tx" and row.get("phase") in ("control","controlled_stop")]
    if len(cycles) != len(tx) or not all(row["success"] for row in tx):
        raise ValueError("recorded control cycles require one actual successful TX each")
    if not all(sent["begin_ns"]>=cycle["time_ns"] for cycle,sent in zip(cycles,tx)):
        raise ValueError("recorded TX must follow its paired control decision")
    yaw = [row for row in rows if row["kind"] == "yaw_feedback"]
    wire = [row for row in rows if row["kind"] == "can_rx" and row.get("axis") == "yaw"]
    imu = [json.loads(row["raw_json"]) for row in rows if row["kind"] == "imu_raw"]
    gyro = [row for row in imu if row.get("kind") == "sample" and row.get("sensor") == "gyro"]
    game = [row for row in imu if row.get("kind") == "sample" and row.get("sensor") == "game_rv"]
    raw = np.asarray([row["encoder_raw"] for row in yaw])
    q = float(manifest["yaw_position_offset_rad"])+np.r_[0,np.cumsum((np.diff(raw)+4096)%8192-4096)]*(2*np.pi/8192)
    yt = (np.asarray([row["kernel_monotonic_ns"] for row in yaw],np.int64)-begin)*1e-9
    ct = (np.asarray([row["time_ns"] for row in cycles],np.int64)-begin)*1e-9
    gt = (np.asarray([row["sample_ns"] for row in gyro],np.int64)-begin)*1e-9
    column = np.asarray(manifest["gyro_calibration"]["yaw_column"],float)
    bias = np.asarray(manifest["gyro_calibration"]["baseline_sensor_bias"],float)
    v = (np.asarray([row["values"] for row in gyro])-bias)@column/(column@column)
    cq,cv = np.interp(ct,yt,q),np.interp(ct,gt,v)
    refs = np.asarray([[row["reference"][key] for key in
                       ("position_rad","velocity_rad_s","acceleration_rad_s2","posture_rad")] for row in cycles])
    core = [row["core"] for row in cycles]
    trace = np.asarray([[cq[index],cv[index],row["position_rad"],row["velocity_rad_s"],
        row["requested_A"],row["limited_A"],row["integral_A"],row["feedforward_A"],
        row["start_increment_A"],row["motion"],row["status"],tx[index]["successful_tx_A"]]
        for index,row in enumerate(core)])
    band = bandwidth_context["measured_report"]["spectral_result"]["valid_band_hz"]
    timing = {"command_time":float(profile["command_time_s"]),
              "zero_reference_time":float(profile["zero_reference_time_s"])}
    try:
        metrics = motion_metrics(ct,refs,trace,gyro_bandwidth_hz=float(band[-1]),**timing)
    except Rejected as exc:
        metrics = {"passed":False,"evaluation_error":str(exc)}
    steady = ((abs(refs[:,2])<1e-9)&(abs(refs[:,1])>np.deg2rad(.1))
              &(ct>=timing["command_time"])&(ct<timing["zero_reference_time"]-.05))
    dt = float(np.median(np.diff(ct)))
    half = max(3,round(.2/dt))//2
    centered = np.arange(-half,half+1)*dt
    local = np.convolve(cq,centered[::-1]/(centered@centered),mode="same")
    pb,pe = float(profile["plateau_begin_s"]),float(profile["plateau_end_s"])
    quantities = {}
    qt = (np.asarray([row["sample_ns"] for row in game],np.int64)-begin)*1e-9
    rotations = Rotation.from_quat(np.asarray([row["values"] for row in game]))
    anchor = int(np.flatnonzero(qt<=0)[-1])
    body = (rotations[anchor].inv()*rotations).as_rotvec()@column/np.linalg.norm(column)
    for name,times,positions in (("encoder",yt,q),("independent_game_rv",qt,body)):
        quantities[name+"_plateau_change_deg"] = float(np.rad2deg(np.interp(pe,times,positions)-np.interp(pb,times,positions)))
    gyro_mask = (gt>=pb)&(gt<=pe)
    quantities["independent_gyro_plateau_integral_deg"] = float(np.rad2deg(np.trapezoid(v[gyro_mask],gt[gyro_mask])))
    quantities["encoder_mean_velocity_ratio"] = float(np.mean(local[steady])/np.mean(refs[steady,1]))
    quantities["encoder_local_active_fraction"] = float(np.mean(np.sign(refs[steady,1])*local[steady]>.05*abs(refs[steady,1])))
    quantities["independent_gyro_mean_velocity_deg_s"] = float(np.rad2deg(np.mean(cv[steady])))
    quantities["encoder_local_mean_velocity_deg_s"] = float(np.rad2deg(np.mean(local[steady])))
    quantities["reference_mean_velocity_deg_s"] = float(np.rad2deg(np.mean(refs[steady,1])))
    quantities["encoder_local_velocity_deg_s_range"] = [float(np.rad2deg(local[steady].min())),float(np.rad2deg(local[steady].max()))]
    zero_elapsed = (zero-begin)*1e-9
    final = (yt>=zero_elapsed)&(yt<=zero_elapsed+2)
    final_drift = float(np.max(np.abs(q[final]-q[final][0])))
    zero_tx = [row for row in rows if row["kind"] == "yaw_current_tx" and row.get("phase") == "stop" and row["success"]]
    selected = np.flatnonzero(steady)
    accepted = np.asarray([tx[index]["kernel_accepted_ns"] for index in selected],np.int64)
    observed = list(zip(ct,cq,cv,local,refs[:,0],refs[:,1],refs[:,2],trace[:,9],trace[:,10],trace[:,4],trace[:,11]))
    transitions = _motion_transitions(cycles,tx,observed,manifest,params,begin,zero,half)
    transitions["controlled_stop"].update(successful_zero_TX_count=len(zero_tx),
        all_stop_TX_zero=all(row["successful_tx_A"] == 0 for row in zero_tx))
    report = {
        "schema":"adr0022.actual-yaw-velocity-analysis/1", "analysis_status":"COMPLETE",
        "capture_footer":rows[-1], "reference_profile":profile, "exact_parameters":params,
        "numerical_readback_matches_captured_manifest":numerical_values(params)==numerical_values(manifest["controller_parameters"]),
        "exact_parameters_match_expected_export":numerical_values(params)==numerical_values(expected_parameters) if expected_parameters is not None else None,
        "guidance_flag":params["acceleration_current_window_enabled"],
        "raw_encoder_matches_wire":len(wire)==len(yaw) and all(((packet["bytes"][0]<<8)|packet["bytes"][1])==row["encoder_raw"] for packet,row in zip(wire,yaw)),
        "decoded_encoder_matches_raw":bool(np.allclose(q,[row["q_relative_rad"] for row in yaw],atol=1e-12)),
        "actual_reference_plateau":{
            "decision_time_support_s":[float(ct[selected[0]]),float(ct[selected[-1]])],
            "actual_successful_TX_time_support_ns":[int(accepted[0]),int(accepted[-1])],
            "actual_successful_TX_duration_s":float((accepted[-1]-accepted[0])*1e-9),
            "decision_count":len(selected), "current_A_range":[float(trace[steady,11].min()),float(trace[steady,11].max())],
            "motion_counts":dict(Counter(int(value) for value in trace[steady,9])),
            "status_counts":dict(Counter(int(value) for value in trace[steady,10]))},
        "frozen_motion_metrics":metrics,
        "frozen_motion_metrics_degrees":{key.replace("_rad_s","_deg_s").replace("_rad","_deg"):float(np.rad2deg(value))
            for key,value in metrics.items() if key.endswith(("_rad","_rad_s"))},
        "independent_actual_motion":quantities, "gyro_band_context":bandwidth_context,
        "actual_observation_support":{
            "encoder_receipts":len(yaw), "gyro_samples":len(gyro), "game_rv_samples":len(game),
            "control_cycles":len(cycles), "median_cycle_period_s":dt,
            "cycle_period_s_range":[float(np.diff(ct).min()),float(np.diff(ct).max())],
            "fixed_local_velocity_window_s":2*half*dt, "gyro_status_counts":dict(Counter(row["status"] for row in gyro))},
        "final_zero_observation":{
            "begin_elapsed_s":zero_elapsed, "encoder_sample_time_support_s":[float(yt[final][0]),float(yt[final][-1])],
            "drift_deg":float(np.rad2deg(final_drift)), "existing_drift_limit_deg":.15,
            "passed":bool(final_drift<=np.deg2rad(.15)), "successful_zero_TX_count":len(zero_tx),
            "all_successful_stop_TX_zero":all(row["successful_tx_A"] == 0 for row in zero_tx)},
        "motion_transitions":transitions,
        "scope":"Actual recorded sampled references and successful TX history; unchanged frozen velocity metrics. Gyro projection and prior excited bandwidth remain pose-limited hypotheses. Fixed200ms convolution uses measured median cycle cadence; actual timestamps/raw streams retained. No model/gain update or full map qualification.",
        "motor_action":"NONE", "physical_qualification":False}
    return report,observed


def compare_recorded_metrics(report, previous):
    """Exact replay comparison of the already-recorded metric fields."""
    fields = ("capture_footer","reference_profile","exact_parameters",
              "numerical_readback_matches_captured_manifest","guidance_flag",
              "raw_encoder_matches_wire","decoded_encoder_matches_raw",
              "actual_reference_plateau","frozen_motion_metrics","frozen_motion_metrics_degrees",
              "independent_actual_motion","gyro_band_context","actual_observation_support",
              "final_zero_observation")
    current = json.loads(json.dumps(report,allow_nan=False))
    matches = {key:current[key]==previous[key] for key in fields}
    return {"comparison":"exact_saved_metric_values", "all_fields_match":all(matches.values()),
            "field_matches":matches}
