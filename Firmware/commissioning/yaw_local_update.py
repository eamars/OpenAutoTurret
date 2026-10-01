"""Deterministic measured yaw update with an explicitly local fallback."""
from __future__ import annotations

from copy import copy, deepcopy
import json
from pathlib import Path
import time

import numpy as np

from .breakaway import estimate_interval
from .contracts import ModelSpec, Reason, Rejected, require
from .identification import integral_design, bounded_initializer, output_error
from .model import coefficients
from Firmware.tools.adr0022_yaw_data import regression

QUANTUM = 2*np.pi/8192


def feedback_windows(stream, rows, calibration):
    """Retain every actual MOVE support and its existing noise-window decision."""
    begin = next(row["time_ns"] for row in rows if row["kind"] == "yaw_control_begin")
    cycles = [row for row in rows if row["kind"] == "yaw_control_cycle"]
    readback = next(row["parameters"] for row in rows if row["kind"] == "controller_parameters_readback")
    duration = estimate_interval.__kwdefaults__["sustained_s"]
    width = round(duration*1e9)
    yt = np.asarray([row["kernel_monotonic_ns"] for row in stream["yaw"]], np.int64)
    q = np.asarray([row["q_relative_rad"]+stream["q_offset"] for row in stream["yaw"]])
    gt = np.asarray([row["sample_ns"] for row in stream["gyro"]], np.int64)
    column = np.asarray(calibration["yaw_column"])
    velocity = (np.asarray([row["values"] for row in stream["gyro"]])-calibration["baseline_sensor_bias"])@column/(column@column)
    sigma_v = calibration["noise_sigma_rad_s"]
    baseline = (yt >= begin-int(2e9))&(yt < begin)
    baseline_fit = regression((yt[baseline]-begin)/1e9, q[baseline])
    sigma_q = max(baseline_fit["detrended_sigma"], QUANTUM/np.sqrt(12))
    baseline_rates, quantization_sigmas = [], []
    for i in np.flatnonzero(baseline & (yt >= yt[0]+width)):
        first = np.searchsorted(yt, yt[i]-width, side="right")
        t = (yt[first:i+1]-yt[i])/1e9
        baseline_rates.append(regression(t, q[first:i+1])["slope"])
        quantization_sigmas.append(QUANTUM/np.sqrt(12*np.sum((t-t.mean())**2)))
    scatter = float(np.std(baseline_rates))
    sigma_rate = scatter if scatter > 0 else float(np.median(quantization_sigmas))
    groups = []
    for cycle in cycles:
        if not groups or groups[-1][0]["core"]["motion"] != cycle["core"]["motion"]:
            groups.append([])
        groups[-1].append(cycle)
    selected, decisions, core_support = [], [], []
    for index, group in enumerate(groups):
        start, end = group[0]["time_ns"], group[-1]["time_ns"]
        motion = group[0]["core"]["motion"]
        core_support.append({"group":index, "motion":motion, "start_ns":start, "end_ns":end,
                             "first_sequence":group[0]["sequence"], "last_sequence":group[-1]["sequence"],
                             "included_phase":motion == 2, "raw_retained_in":stream["journal"]})
        if motion != 2:
            continue
        sign = int(np.sign(np.interp(end,yt,q)-np.interp(start,yt,q)))
        decisions.append({"group":index, "start_ns":start, "end_ns":min(start+width,end),
                          "excluded_reasons":["existing_60ms_confirmation_trim"], "raw_retained_in":stream["journal"]})
        active = None
        for block, first in enumerate(range(start+width, end, width)):
            last = min(first+width,end)
            dt = (last-first)/1e9
            gi = (gt >= first)&(gt <= last)
            stamps = np.unique(np.r_[first,gt[gi],last])
            rotation = float(np.trapezoid(np.interp(stamps,gt,velocity),(stamps-first)/1e9))
            rate = float((np.interp(last,yt,q)-np.interp(first,yt,q))/dt)
            reasons = []
            if last-first < width:
                reasons.append("remaining_support_shorter_than_existing_motion_window")
            if not sign or sign*rate <= 3*sigma_rate or sign*rotation <= 3*sigma_v*dt:
                reasons.append("resolved_reversal" if sign and sign*rate < -3*sigma_rate and sign*rotation < -3*sigma_v*dt
                               else "rest_or_unresolved_motion_in_existing_noise_window")
            decision = {"group":index,"block":block,"direction":sign,"start_ns":first,"end_ns":last,
                        "actual_encoder_rate_rad_s":rate,"frozen_projected_gyro_rotation_rad":rotation,
                        "actual_gyro_samples":int(gi.sum()),"excluded_reasons":reasons,"raw_retained_in":stream["journal"]}
            decisions.append(decision)
            if reasons:
                if active:
                    selected.append(active)
                    active = None
            elif active:
                active["end_ns"] = last
                active["confirmation_windows"].append(decision)
            else:
                active = {"journal":stream["journal"],"run_id":f"{Path(stream['journal']).parent.parent.name}-MOVE-{index}-block-{block}",
                          "direction":sign,"start_ns":first,"end_ns":last,"sigma_q_rad":sigma_q,
                          "confirmation_windows":[decision],"source_core_group":index}
        if active:
            selected.append(active)
    supported, excluded = [], []
    for window in selected:
        anchors = {"encoder":int(((yt >= window["start_ns"])&(yt <= window["end_ns"])).sum()),
                   "gyro":int(((gt >= window["start_ns"])&(gt <= window["end_ns"])).sum())}
        reasons = [f"fewer_than_two_actual_{key}_anchors" for key,n in anchors.items() if n < 2]
        window["actual_observation_anchors"] = anchors
        window["closed_loop_identification"] = True
        window["support_controller_source_label"] = stream["config"]["candidate_label"]
        if reasons:
            excluded.append({**window,"excluded_reasons":reasons})
        else:
            supported.append(window)
    # Describe the actual sample timing/noise in SI. It does not calibrate
    # the sensor's undocumented internal dynamics or select a controller gain.
    bg = (gt >= begin-int(2e9))&(gt < begin)
    same_generation = np.asarray([a["generation"] == b["generation"] for a,b in zip(stream["gyro"],stream["gyro"][1:])])
    intervals = np.diff(gt)/1e9
    valid = same_generation & (intervals > 0)
    derivatives = np.diff(velocity)[valid]/intervals[valid]
    baseline_derivatives = np.diff(velocity)[valid & bg[:-1] & bg[1:]]/intervals[valid & bg[:-1] & bg[1:]]
    return supported, {"journal":stream["journal"],"all_core_support":core_support,"all_motion_decisions":decisions,
        "excluded_resolved_windows":excluded,"actual_controller_parameters":readback,
        "reference_source":"Every yaw_control_cycle reference is retained in the source journal; selection uses actual body motion, not desired movement or error magnitude",
        "measured_baseline_encoder_sigma_rad":sigma_q,"measured_baseline_encoder_rate_sigma_rad_s":sigma_rate,
        "baseline_time_support_ns":[int(yt[baseline][0]),int(yt[baseline][-1])],"existing_motion_window_s":duration,
        "gyro_observation_metadata":{"sample_period_s":float(np.median(intervals[valid])),
            "sample_period_range_s":[float(intervals[valid].min()),float(intervals[valid].max())],
            "baseline_acceleration_sigma_rad_s2":float(np.std(baseline_derivatives)),
            "actual_acceleration_range_rad_s2":[float(derivatives.min()),float(derivatives.max())],
            "frozen_gyro_noise_sigma_rad_s":sigma_v,"added_filter_tau_s":0.,"internal_filter":"unknown",
            "source":"Actual unique gyro sample times and unchanged frozen yaw projection; baseline last two seconds before control"},
        "source_capture_footer":stream["footer"],"qualification":False}


def information(matrix):
    matrix = np.asarray(matrix)
    scale = np.linalg.norm(matrix,axis=0)
    observed = scale > 1e-12
    singular = np.linalg.svd(matrix[:,observed]/scale[observed],compute_uv=False) if observed.any() else np.array([])
    rank = int(np.sum(singular > singular[0]/1e6)) if len(singular) else 0
    full = bool(observed.all() and rank == matrix.shape[1])
    condition = float(singular[0]/singular[-1]) if full else None
    return {"equations":matrix.shape[0],"unknowns":matrix.shape[1],"observed_columns":np.flatnonzero(observed).tolist(),
            "normalized_rank":rank,"normalized_condition":condition,
            "initializer_supported":bool(full and matrix.shape[0] >= 2*matrix.shape[1]),
            "rank_rule":"Existing constrained initializer: nonzero columns and normalized singular values above largest/1e6"}


def predictions(native,spec,theta,runs):
    reports = []
    for run in runs:
        predicted = native.rollout(spec,theta,run.t,run.tx,run.z,run.direction,(run.q[0],run.v[0]),
                                   tx_history_t=run.tx_history_t,tx_history_A=run.tx_history_A)
        eq = predicted[run.q_new,0]-run.q[run.q_new]
        ev = predicted[run.v_new,1]-run.v[run.v_new]
        qrms, vrms = float(np.sqrt(np.mean(eq**2))), float(np.sqrt(np.mean(ev**2)))
        limit_v = max(float(np.deg2rad(.5)),float(np.mean(np.abs(run.v[run.v_new])))*.1)
        failures = []
        if qrms > np.deg2rad(.15):
            failures.append("angle_rms")
        if vrms > limit_v:
            failures.append("velocity_rms")
        reports.append({"run_id":run.run_id,"direction":int(run.direction[0]),"duration_s":float(run.t[-1]),
            "q_rms_rad":qrms,"q_rms_deg":float(np.rad2deg(qrms)),"v_rms_rad_s":vrms,"v_rms_deg_s":float(np.rad2deg(vrms)),
            "q_endpoint_error_deg":float(np.rad2deg(eq[-1])),"v_endpoint_error_deg_s":float(np.rad2deg(ev[-1])),
            "velocity_rms_limit_deg_s":float(np.rad2deg(limit_v)),"metric_failures":failures,"qualified":False})
    return reports


def measured_model_alternatives(prior,source_path,additional_paths=()):
    """Retain exact recorded models and failures from this measured prior chain."""
    retained=deepcopy(prior.get("measured_model_alternatives",[]))
    sources=[(Path(source_path),prior)]
    if prior.get("prior_model_path"):
        path=Path(prior["prior_model_path"])
        sources.append((path,json.loads(path.read_text())))
    sources.extend((Path(path),json.loads(Path(path).read_text())) for path in additional_paths)
    for path,model in sources:
        if any(row["source_context"]==str(path) for row in retained):
            continue
        calibration=model["gyro_calibration"]
        same=(model["model_spec"]==prior["model_spec"] and all(calibration[key]==prior["gyro_calibration"][key]
              for key in ("yaw_column","baseline_sensor_bias")))
        retained.append({"source_context":str(path),"theta":model["theta"],"model_spec":model["model_spec"],
            "gyro_calibration":calibration,"same_frozen_model_and_gyro_calibration":same,
            "training_journals":model["training_journals"],"local_q_support_rad":model.get("local_q_support_rad"),
            "actual_posture_support_rad":model.get("actual_posture_support_rad"),"local_only":model.get("local_only",True),
            "correction_domain":model.get("correction_domain"),"failure_classification":model.get("failure_classification"),
            "raw_prediction_errors":model.get("update_report",{}).get("raw_prediction_errors"),
            "provenance":"Exact earlier measured-data fit; conflicting dynamics retained as provisional stress hypotheses",
            "statistical_confidence":None,"formal_qualification":False,"global_qualification":False})
    return retained


def update(prior,runs,descriptions,native,*,delay_bound_s,source_path,native_path,progress=None,additional_model_paths=()):
    """Choose supported periodic, local dynamic, or frozen-dynamic coordinates."""
    start = time.perf_counter()
    spec = ModelSpec(**prior["model_spec"])
    fixed = np.asarray(prior["theta"])
    aligned = []
    for run in runs:
        indices = np.searchsorted(run.tx_history_t,run.t-fixed[-1],side="right")-1
        require(np.all(indices >= 0),Reason.DATA_INVALID,"actual pre-window successful TX history does not cover the frozen command delay")
        row = copy(run)
        row.tx = run.tx_history_A[indices]
        aligned.append(row)
    # The existing sustained motion support is also an existing integral window.
    window_s = estimate_interval.__kwdefaults__["sustained_s"]
    X,y,groups = integral_design(spec,aligned,window_s=window_s)
    full_map = np.asarray(prior.get("periodic_parameter_map",prior["parameter_map"]))
    full_design = X@full_map[:-1,:-1]
    full_information = information(full_design)
    observed_columns = full_information["observed_columns"]
    observed_information = information(full_design[:,observed_columns])
    block = 3*len(spec.q_nodes)
    local_map = np.zeros((spec.size,2))
    local_map[6:6+block,0] = 1.
    local_map[6+block:-1,1] = 1.
    local_information = information(X@local_map[:-1])
    dynamic_map = np.zeros((spec.size,4))
    dynamic_map[:3,0] = 1.
    dynamic_map[3:6,1] = 1.
    dynamic_map[:,2:] = local_map
    dynamic_information = information(X@dynamic_map[:-1])
    branch = ("PERIODIC_Z_TIED" if full_information["initializer_supported"] else
              "OBSERVED_PERIODIC_Z_TIED" if observed_information["initializer_supported"] else
              "LOCAL_A_B_DIRECTIONAL_H_OFFSETS" if dynamic_information["initializer_supported"] else
              "LOCAL_DIRECTIONAL_H_OFFSETS")
    report = {"schema":"adr0022.yaw-running-update/1","model_spec":prior["model_spec"],"branch":branch,"periodic_information":full_information,
        "local_information":local_information,"local_dynamic_information":dynamic_information,"initializer_integral_window_s":window_s,
        "observed_periodic_columns":observed_columns,"observed_periodic_information":observed_information,
        "initializer_command_delay_s":float(fixed[-1]),"delay_search_bound_s":delay_bound_s,
        "equations_by_direction":{str(d):sum(int(runs[int(g)].direction[0]) == d for g in groups) for d in (-1,1)},
        "actual_pre_window_input":"Recorded successful TX events, ZOH at t-delay; measured initial q/v",
        "prior_theta":fixed.tolist(),"training_run_descriptions":descriptions,"formal_qualification":False}
    if progress:
        progress("information",report)
    if branch == "PERIODIC_Z_TIED":
        mapping = full_map
        guess, condition = bounded_initializer(X@mapping[:-1,:-1],y,
            lower_bounds=np.r_[1e-8,0.,np.full(mapping.shape[1]-3,-np.inf)])
        coordinates = np.r_[guess,fixed[-1]]
        offset = np.zeros(spec.size)
        bounds = (np.r_[1e-8,0.,np.full(mapping.shape[1]-3,-np.inf),0.],
                  np.r_[np.full(mapping.shape[1]-1,np.inf),delay_bound_s])
    elif branch == "OBSERVED_PERIODIC_Z_TIED":
        mapping, offset = full_map[:,observed_columns], fixed.copy()
        offset[np.any(mapping!=0,axis=1)] = 0.
        lower=np.asarray([1e-8 if column==0 else 0. if column==1 else -np.inf for column in observed_columns])
        guess, condition = bounded_initializer(X@mapping[:-1],y-X@offset[:-1],lower_bounds=lower)
        coordinates = guess
        bounds = (lower,np.full(len(observed_columns),np.inf))
    elif branch == "LOCAL_A_B_DIRECTIONAL_H_OFFSETS":
        mapping, offset = dynamic_map, fixed.copy()
        offset[:6] = 0.
        guess, condition = bounded_initializer(X@mapping[:-1],y-X@offset[:-1],
                                               lower_bounds=np.r_[1e-8,0.,-np.inf,-np.inf])
        coordinates = guess
        bounds = (np.r_[1e-8,0.,-np.inf,-np.inf],np.full(4,np.inf))
    else:
        mapping, offset = local_map, fixed
        guess, condition = bounded_initializer(X@mapping[:-1],y-X@fixed[:-1],lower_bounds=np.full(2,-np.inf))
        coordinates = guess
        bounds = (np.full(2,-np.inf),np.full(2,np.inf))
    initial = offset+mapping@coordinates
    report.update(initializer_theta=initial.tolist(),initializer_reduced_theta=coordinates.tolist(),
                  parameter_map=mapping.tolist(),parameter_offset=offset.tolist(),normalized_condition=condition)
    if progress:
        progress("initializer",report)
    tick = time.perf_counter()
    fitted = output_error(native,spec,initial,runs,delay_bound_s,parameter_map=mapping,parameter_offset=offset,
                          reduced_initial=coordinates,reduced_bounds=bounds)
    report.update(theta=fitted.x.tolist(),reduced_theta=fitted.reduced_x.tolist(),
        optimizer_evaluations=fitted.nfev,optimizer_max_evaluations=2000,optimizer_message=str(fitted.message),
        normalized_residual_rms=float(np.sqrt(np.mean(fitted.fun**2))),output_error_s=time.perf_counter()-tick)
    old = predictions(native,spec,fixed,runs)
    new = predictions(native,spec,fitted.x,runs)
    report["raw_prediction_errors"] = [{"prior":a,"updated":b} for a,b in zip(old,new)]
    report["failure_classification"] = "MODEL_INADEQUATE_ON_ACTUAL_LOCAL_OBSERVATIONS" if any(r["metric_failures"] for r in new) else "LOCAL_PREDICTIONS_WITHIN_RECORDED_METRICS_UNQUALIFIED"
    allq, allz = np.concatenate([r.q[r.q_new] for r in runs]), np.concatenate([r.z for r in runs])
    report.update(local_q_support_rad=[float(allq.min()),float(allq.max())],
                  actual_posture_support_rad=[float(allz.min()),float(allz.max())],total_s=time.perf_counter()-start)
    if branch == "LOCAL_DIRECTIONAL_H_OFFSETS":
        report["local_h_offsets_negative_positive_A"] = fitted.reduced_x.tolist()
        report["frozen_parameters"] = "prior a/b/delay and directional spatial shape; three posture rows remain ties"
    elif branch == "LOCAL_A_B_DIRECTIONAL_H_OFFSETS":
        report["local_h_offsets_negative_positive_A"] = fitted.reduced_x[2:].tolist()
        report["local_a_b"] = fitted.reduced_x[:2].tolist()
        report["frozen_parameters"] = "prior delay and directional spatial shape; fitted common a/b and directional offsets are local only, three posture rows remain ties"
    elif branch == "OBSERVED_PERIODIC_Z_TIED":
        report["local_a_b"] = [float(fitted.x[0]),float(fitted.x[3])]
        report["frozen_parameters"] = "prior unobserved spatial coordinates and delay; only actual supported periodic coordinates fitted, three posture rows remain ties"
    alternatives=measured_model_alternatives(prior,source_path,additional_model_paths)
    report["measured_model_alternatives"] = alternatives
    context = deepcopy(prior)
    context.update(model_spec=prior["model_spec"],theta=fitted.x.tolist(),native_library=str(native_path),
        training_journals=list(dict.fromkeys(row["journal"] for row in descriptions)),
        training_run_descriptions=descriptions,prior_model_path=str(source_path),update_report=report,
        parameter_map=mapping.tolist(),periodic_parameter_map=full_map.tolist(),parameter_offset=offset.tolist(),delay_search_bound_s=delay_bound_s,
        reduced_theta=fitted.reduced_x.tolist(),optimizer_evaluations=fitted.nfev,optimizer_max_evaluations=2000,
        optimizer_message=str(fitted.message),normalized_condition=condition,
        normalized_residual_rms=report["normalized_residual_rms"],failure_classification=report["failure_classification"],
        local_q_support_rad=report["local_q_support_rad"],actual_posture_support_rad=report["actual_posture_support_rad"],
        local_only=branch != "PERIODIC_Z_TIED",formalfalse=True,formal_qualification=False,
        measured_model_alternatives=alternatives,
        full_qualification=False,unqualified=True,source_provisional=True,provisional=True,
        holdout_used_for_fit=False,holdout_used_for_information=False,holdout_read_or_evaluated=False,
        prior_holdout_results=deepcopy(prior.get("holdout_residuals")),
        scope="Actual independently confirmed local MOVE supports only. Deterministic existing initializer/native Huber OE; actual successful pre-window TX and received MechPos. No unseen local correction, independent posture sensitivity, global map, gain search or physical qualification.")
    # Prior errors are retained as historical results and are never passed off as
    # evaluation of the new model. Unknown outside-domain correction is explicit.
    context.pop("holdout_residuals",None)
    context["prior_training_residuals"] = deepcopy(prior.get("training_residuals"))
    context["training_residuals"] = report["raw_prediction_errors"]
    context["prior_training_capture_support"] = deepcopy(prior.get("training_capture_support"))
    context.pop("training_capture_support",None)
    context["correction_domain"] = {"position_rad":report["local_q_support_rad"],"posture_rad":report["actual_posture_support_rad"],
        "outside_domain":"UNKNOWN; repeated ABI parameter rows do not establish measured correction there",
        "per_direction":[{"direction":d,"q_rad":[float(min(r.q.min() for r in runs if r.direction[0] == d)),
            float(max(r.q.max() for r in runs if r.direction[0] == d))]} for d in (-1,1) if any(r.direction[0] == d for r in runs)]}
    return report, context
