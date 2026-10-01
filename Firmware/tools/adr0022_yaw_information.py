"""Select the next finite yaw measurement through the existing native fit and selector."""
from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from Firmware.commissioning.adaptation import Envelope, choose_information
from Firmware.commissioning.contracts import Identity, ModelSpec
from Firmware.commissioning.identification import Run, bounded_initializer, integral_design, output_error, residuals
from Firmware.commissioning.native import Native
from adr0022_yaw_data import load_capture
from adr0022_yaw_plan import initial_plan

QUANTUM = 2*np.pi/8192
REFERENCE_COUNT = 5768


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+"\n", encoding="utf-8")


def read_stream(path):
    journal, config, rows = load_capture(path)
    yaw = [row for row in rows if row.get("kind") == "yaw_feedback"]
    commands = [row for row in rows if row.get("kind") == "yaw_current_tx" and row.get("success")]
    gyro = [json.loads(row["raw_json"]) for row in rows if row.get("kind") == "imu_raw"]
    gyro = [row for row in gyro if row.get("kind") == "sample" and row.get("sensor") == "gyro"]
    pitch = [row for row in rows if row.get("kind") == "register_read" and
             row.get("axis") == "pitch" and row.get("index") == 0x7019]
    offset = ((yaw[0]["encoder_raw"]-REFERENCE_COUNT+4096)%8192-4096)*QUANTUM-yaw[0]["q_relative_rad"]
    return {"journal": str(journal), "config": config, "yaw": yaw, "commands": commands,
            "gyro": gyro, "pitch": pitch, "q_offset": offset, "footer": rows[-1]}


def encoder_rate(stream, stamps, width_ns):
    yaw = stream["yaw"]
    t = np.array([row["kernel_monotonic_ns"] for row in yaw], dtype=np.int64)
    q = np.array([row["q_relative_rad"] for row in yaw])+stream["q_offset"]
    rates = []
    for stamp in stamps:
        left, right = np.searchsorted(t, [stamp-width_ns, stamp+width_ns])
        tt = (t[left:right]-stamp)/1e9
        qq = q[left:right]
        centered = tt-tt.mean()
        rates.append(float(centered @ (qq-qq.mean())/(centered @ centered)))
    return np.array(rates)


def gyro_calibration(streams, baseline):
    bias = np.array(baseline["sensor_frame_baseline"]["gyro"]["mean"])
    sigma_components = np.array(baseline["sensor_frame_baseline"]["gyro"]["sigma"])
    gyro, velocities = [], []
    period = baseline["sensor_frame_baseline"]["gyro"]["native_interval_s"]["median"]
    width_ns = int(2*period*1e9)
    for stream in streams:
        yt = [row["kernel_monotonic_ns"] for row in stream["yaw"]]
        samples = [row for row in stream["gyro"] if yt[0]+width_ns < row["sample_ns"] < yt[-1]-width_ns]
        stamps = np.array([row["sample_ns"] for row in samples], dtype=np.int64)
        local = encoder_rate(stream, stamps, width_ns)
        motion = np.abs(local) > 3*baseline["encoder_velocity_sigma_rad_s"]
        gyro.extend(np.array([row["values"] for row in samples])[motion]-bias)
        velocities.extend(local[motion])
    velocities, gyro = np.asarray(velocities), np.asarray(gyro)
    column = (velocities @ gyro)/(velocities @ velocities)
    projection = column/(column @ column)
    sigma_v = float(np.sqrt(np.sum((projection*sigma_components)**2)))
    return column, projection, bias, sigma_v, period, {
        "scope": "single yaw column at the current fixed pitch; full mounting calibration unknown",
        "yaw_column": column.tolist(), "baseline_sensor_bias": bias.tolist(),
        "motion_gyro_samples": len(velocities), "encoder_local_regression_half_width_s": width_ns/1e9,
        "noise_sigma_rad_s": sigma_v,
        "added_gyro_filter_tau_s": 0., "sensor_internal_filter": "unknown; unseparated end-to-end dynamics",
    }


def normalized_run(stream, start_ns, end_ns, direction, identity, bias, projection, sigma_v, baseline, label):
    yy = [row for row in stream["yaw"] if start_ns <= row["kernel_monotonic_ns"] <= end_ns]
    gg = [row for row in stream["gyro"] if start_ns <= row["sample_ns"] <= end_ns]
    cc = stream["commands"]
    ys = np.array([row["kernel_monotonic_ns"] for row in yy], dtype=np.int64)
    gs = np.array([row["sample_ns"] for row in gg], dtype=np.int64)
    cs = np.array([row["kernel_accepted_ns"] for row in cc], dtype=np.int64)
    stamps = np.unique(np.r_[ys, gs, cs[(cs >= start_ns)&(cs <= end_ns)]])
    q = np.interp(stamps, ys, [row["q_relative_rad"]+stream["q_offset"] for row in yy])
    rates = (np.array([row["values"] for row in gg])-bias) @ projection
    v = np.interp(stamps, gs, rates)
    tx = np.array([row["successful_tx_A"] for row in cc])[np.searchsorted(cs, stamps, side="right")-1]
    bandwidth = .2/max(float(np.max(np.diff(ys))/1e9), float(np.max(np.diff(gs))/1e9))
    run = Run(label, identity, (stamps-stamps[0])/1e9, q, v, tx,
              np.full(len(stamps), baseline["pitch_mechpos_rad"]["mean"]), np.full(len(stamps), direction),
              np.isin(stamps, ys), np.isin(stamps, gs),
              max(baseline["encoder_detrended_sigma_rad"], QUANTUM/np.sqrt(12)), sigma_v,
              bandwidth, gyro_filter_tau_s=0., tx_history_t=(cs-stamps[0])/1e9,
              tx_history_A=np.array([row["successful_tx_A"] for row in cc]))
    return run, {"run_id": label, "source_journal": stream["journal"], "start_ns": int(start_ns),
                 "end_ns": int(end_ns), "direction": direction, "encoder_observations": len(ys),
                 "gyro_observations": len(gs), "tx_events": int(np.sum((cs>=start_ns)&(cs<=end_ns))),
                 "position_range_rad": [float(q.min()), float(q.max())], "bandwidth_hz": bandwidth,
                 "TX_history_events":len(cs), "TX_history_coordinate":"actual accepted timestamps relative to measured window start"}


def tying(spec):
    # One a/b and one load per direction. Repeated slots are ABI representation,
    # never evidence of other positions/postures having been measured.
    matrix = np.zeros((spec.size, 5))
    matrix[:3, 0] = 1
    matrix[3:6, 1] = 1
    half = 3*len(spec.q_nodes)
    matrix[6:6+half, 2] = 1
    matrix[6+half:-1, 3] = 1
    matrix[-1, 4] = 1
    return matrix


class NumericalObservations:
    validate = Run.validate


def fitted_training_runs(fit, fit_path, *, journals=None):
    """Reconstruct actual full moving observations with frozen yaw calibration."""
    spec = ModelSpec(**fit["model_spec"])
    theta = np.asarray(fit["theta"])
    calibration = fit["gyro_calibration"]
    column = np.asarray(calibration["yaw_column"])
    projection = column/(column @ column)
    bias = np.asarray(calibration["baseline_sensor_bias"])
    identity = Identity("rpi-turret-yaw-GM6020", "encoder-5768-fixed-pitch-yaw-column",
                        "current-payload-fixed-pitch-initial-coverage", "MEASURED", provisional_labels=True)
    runs, inputs = [], []
    for journal in journals or [Path(path) for path in fit["training_journals"]]:
        journal = Path(journal)
        stream = read_stream(journal)
        directory = journal.parent.parent
        windows=[row for row in fit.get("training_run_descriptions",[]) if Path(row["journal"])==journal]
        if fit.get("training_run_descriptions") and not windows:
            continue
        closed_loop=stream["config"].get("schema")=="adr0022.yaw-control/1"
        evidence={} if closed_loop else json.loads((directory/"movement-evidence-full.json").read_text())
        baseline={} if closed_loop else json.loads((directory/"baseline-observations.json").read_text())
        yt = np.asarray([row["kernel_monotonic_ns"] for row in stream["yaw"]], dtype=np.int64)
        gt = np.asarray([row["sample_ns"] for row in stream["gyro"]], dtype=np.int64)
        ct = np.asarray([row["kernel_accepted_ns"] for row in stream["commands"]], dtype=np.int64)
        q = np.asarray([row["q_relative_rad"]+stream["q_offset"] for row in stream["yaw"]])
        velocity = (np.asarray([row["values"] for row in stream["gyro"]])-bias) @ projection
        current = np.asarray([row["successful_tx_A"] for row in stream["commands"]])
        pt = np.asarray([row["receive_ns"] for row in stream["pitch"]], dtype=np.int64)
        pv = np.asarray([row["value"] for row in stream["pitch"]], dtype=float)
        if not windows:
            for direction in (1,-1):
                interval=max((row for row in evidence["sustained_motion_intervals"] if row["direction"]==direction),
                             key=lambda row:row["encoder_motion_duration_s"])
                windows.append({"direction":direction,
                    "start_ns":interval["first_encoder_motion_ns"]+int(evidence["encoder_uncertainty"]["window_s"]*1e9),
                    "end_ns":interval["last_encoder_motion_ns"]})
        for window_index,window in enumerate(windows):
            direction=window["direction"]
            begin,end=window["start_ns"],window["end_ns"]
            qm, vm, cm = (yt>=begin)&(yt<=end), (gt>=begin)&(gt<=end), (ct>=begin)&(ct<=end)
            pm = (pt>=begin)&(pt<=end)
            stamps = np.unique(np.r_[yt[qm], gt[vm], ct[cm], pt[pm]])
            run = NumericalObservations()
            run.identity = identity
            run.run_id = window.get("run_id",f"{directory.name}-full-direction-{direction}-window-{window_index}")
            run.t = (stamps-stamps[0])/1e9
            run.q = np.interp(stamps, yt, q)
            run.v = np.interp(stamps, gt, velocity)
            run.tx = current[np.searchsorted(ct, stamps, side="right")-1]
            run.tx_history_t = (ct-stamps[0])/1e9
            run.tx_history_A = current.copy()
            run.tx_history_accepted_ns = ct.copy()
            posture_indices = np.searchsorted(pt, stamps, side="right")-1
            if not len(pt) or np.any(posture_indices<0):
                raise ValueError("moving observations have no preceding actual pitch readback")
            run.z = pv[posture_indices]
            run.pitch_receive_ns = pt[pm]
            run.pitch_receipt_age_s = (stamps-pt[posture_indices])/1e9
            run.direction = np.full(len(stamps), direction)
            run.q_new, run.v_new = np.isin(stamps, yt[qm]), np.isin(stamps, gt[vm])
            run.sigma_q = window["sigma_q_rad"] if closed_loop else max(baseline["yaw_baseline"]["encoder_linear_observation"]["detrended_sigma"], QUANTUM/np.sqrt(12))
            run.sigma_v = calibration["noise_sigma_rad_s"]
            run.bandwidth_hz = np.nextafter(min(1/np.max(np.diff(run.t[run.q_new])),
                                              1/np.max(np.diff(run.t[run.v_new])))/5, 0.)
            run.generation = 1
            run.acquisition_verified = True
            run.closed_loop_identification = closed_loop
            run.support_controller_hash = stream["config"]["candidate_label"] if closed_loop else None
            run.gyro_filter_tau_s = 0.
            run.source_journal = str(journal)
            run.gyro_sample_ns = gt[vm]
            run.gyro_receive_ns = np.asarray([row["rx_ns"] for row in stream["gyro"]], dtype=np.int64)[vm]
            run.encoder_receipt_ns = yt[qm]
            run.tx_accepted_ns = ct[cm]
            run.raw_baseline = baseline
            run.validate()
            runs.append(run)
            inputs.append({"journal":str(journal), "direction":direction, "start_ns":int(stamps[0]),
                           "end_ns":int(stamps[-1]), "encoder_events":int(run.q_new.sum()),
                           "gyro_events":int(run.v_new.sum()),
                           "actual_pitch_range_rad":[float(run.z.min()),float(run.z.max())],
                           "actual_pitch_receive_events":int(pm.sum()),
                           "actual_pitch_max_receipt_age_s":float(run.pitch_receipt_age_s.max()),
                           "actual_TX_history_events":len(ct),
                           "actual_TX_history_start_ns":int(ct[0]),
                           "actual_TX_history_end_ns":int(ct[-1]),
                           "native_pre_window_input":"actual successful TX ZOH; measured initial q/v unchanged",
                           "posture_basis":"actual receive-time MechPos ZOH; device sample time unknown"})
    return spec, runs, inputs


def select_frozen_fit(args):
    fit = json.loads(args.fit_json.read_text())
    spec, runs, inputs = fitted_training_runs(fit, args.fit_json, journals=args.journal)
    theta = np.asarray(fit["theta"])
    calibration = fit["gyro_calibration"]
    baseline = json.loads(args.baseline.read_text())
    failure_evidence = [{"source":str(path), "observation":json.loads(path.read_text())}
                        for path in args.failure_evidence]
    noise = float(baseline["yaw_baseline"]["protocol_current_A"]["standard_deviation"])
    X, y, groups = integral_design(spec, runs)
    prior = np.zeros((spec.size, spec.size))
    prior[:-1,:-1] = X.T @ X/noise**2
    eigenvalues = np.linalg.eigvalsh(prior)
    write_json(args.output_dir/"prior-diagnostics.json", {"minimum_eigenvalue":float(eigenvalues[0]),
        "maximum_eigenvalue":float(eigenvalues[-1]), "maximum_absolute_entry":float(np.max(np.abs(prior))),
        "shape":list(prior.shape), "equations":len(y), "mathematical_form":"X.T @ X / measured_noise_sigma**2",
        "noise_sigma_A":noise, "model_or_calibration_refit":False})
    position_stream = read_stream(args.position_journal)
    position = position_stream["yaw"][-1]["q_relative_rad"]+position_stream["q_offset"]
    gt = np.asarray([row["sample_ns"] for row in position_stream["gyro"]], dtype=np.int64)
    sample_hz = 1/float(np.median(np.diff(gt))/1e9)
    posture = float(position_stream["pitch"][-1]["value"])
    seed_plan = initial_plan(label=args.session_label, approved_minimum_a=.25,
        current_bound_a=.9, factor_index=0, pulse_s=.5, baseline_s=2., zero_between_s=2.,
        startup_s=20., duration_s=40.)
    maximum_hold_s = seed_plan["current_segments"][1]["duration_s"]
    # Include the existing maximum-first preparation in prediction and scoring.
    # It is a finite stimulus, not a certificate that the body will move.
    envelope = Envelope(.9, 1.8, None, None, None, None, None,
        4.+maximum_hold_s+4*.9/1.8, False, "MEASURED")
    native = Native(args.library)
    options = []
    for direction in (-1, 1):
        rows = choose_information(native, spec, theta, theta[None,:], envelope, prior,
            initial_position=position, posture=posture, direction=direction,
            noise_sigma=noise, sample_hz=sample_hz, maximum_first_hold_s=maximum_hold_s,
            _all_templates=True)
        options.extend({**row, "direction":direction} for row in rows)
    # Same prescribed marginal-information rule, comparing both directions at
    # the actual reachable pose rather than choosing a direction by hand.
    chosen = []
    information = prior+np.eye(spec.size)*1e-12
    while options and len(chosen)<3:
        score_before = np.linalg.slogdet(information)[1]
        for row in options:
            row["score"] = float((np.linalg.slogdet(information+row["information"])[1]-score_before)/row["time"][-1])
        winner = min(options, key=lambda row:(-row["score"], row["direction"], row["case_id"]))
        chosen.append(winner)
        options.remove(winner)
        information += winner["information"]
    selected = []
    for index, row in enumerate(chosen):
        t, current = row["time"], row["successful_tx"]
        segments = [{"duration_s":float(t[k+1]-t[k]), "start_A":float(current[k]),
                     "end_A":float(current[k+1])} for k in range(len(t)-1)]
        transition_s = 0.
        if current[0] != 0:
            duration = abs(float(current[0]))/envelope.slew_a_s
            segments.insert(0, {"duration_s":duration, "start_A":0., "end_A":float(current[0])})
            transition_s += duration
        if current[-1] != 0:
            duration = abs(float(current[-1]))/envelope.slew_a_s
            segments.append({"duration_s":duration, "start_A":float(current[-1]), "end_A":0.})
            transition_s += duration
        direction_label = "positive" if row["direction"]>0 else "negative"
        plan = initial_plan(label=args.session_label+f"-case-{row['case_id']}-{direction_label}", approved_minimum_a=.25,
            current_bound_a=.9, factor_index=0, pulse_s=.5, baseline_s=2., zero_between_s=2., startup_s=20., duration_s=40.)
        plan["current_segments"] = segments
        plan["signal_selection"] = {"method":"choose_information", "case_id":row["case_id"],
            "direction":row["direction"], "direction_selection":"same marginal Fisher information across both directions at measured pose",
            "score_logdet_per_s":row["score"], "template_count":32, "fit_source":str(args.fit_json),
            "prior_source":"actual .90/.45 A full-motion integral-design equations",
            "plant_prediction_available":True, "plant_qualified":False, "uncertainty_reliable":False,
            "stop_prediction":row.get("stop_prediction"), "stop_verified":False,
            "initial_position_rad":position, "raw_encoder_reference_count":REFERENCE_COUNT,
            "posture_rad":posture, "posture_source":str(args.position_journal),
            "posture_measurement_ns":position_stream["pitch"][-1]["receive_ns"],
            "kinematic_or_angle_constraints":None,
            "capability_basis":"owner-authorized finite .90 A yaw authority; measured requested 1.8 A/s ramp shape",
            "gyro_internal_filter":"unknown; frozen current-pose yaw column, gyro accuracy0 retained",
            "prior_scale_basis":"integral-design Fisher metric with measured protocol-current baseline scatter; planning score, not confidence",
            "observed_030A_response":"brief body rotation followed by limited motion; not a qualified starting-current threshold",
            "candidate_peak_above_030A":bool(np.max(np.abs(current))>.3),
            "zero_to_load_and_back_transition_s":transition_s,
            "transition_rule":"abs(endpoint_current)/recorded input slew; first and last commands are zero",
            "starting_or_sustained_motion_predicted_by_model_only":True,
            "maximum_first_prefix":row["maximum_first_prefix"],
            "observed_motion_evidence_sources":[row["source"] for row in failure_evidence],
            "actual_motion_verification_required_after_capture":True,
            "failed_start_observations_are_censored_not_thresholds":True}
        from adr0022_capture_launch import validate_yaw_contract
        validate_yaw_contract(plan)
        path = args.output_dir/("manifest.json" if index == 0 else f"manifest-case-{row['case_id']}-{direction_label}.json")
        write_json(path, plan)
        selected.append({"case_id":row["case_id"], "direction":row["direction"], "manifest":str(path), "score":row["score"],
            "duration_s":float(t[-1])+transition_s, "template_duration_s":row["template_duration_s"],
            "maximum_first_prefix":row["maximum_first_prefix"],
            "transition_duration_s":transition_s,
            "current_min_A":min(0.,float(current.min())), "current_max_A":max(0.,float(current.max())),
            "current_first_A":0., "current_last_A":0.,
            "maximum_segment_slew_A_s":max(abs(row["end_A"]-row["start_A"])/row["duration_s"] for row in segments),
            "stop_prediction":row.get("stop_prediction"), "time_s":t.tolist(), "current_A":current.tolist()})
    report = {"schema":"adr0022.yaw-information-selection/2", "unqualified":True,
        "fit_source":str(args.fit_json), "model_refit":False, "gyro_calibration_refit":False,
        "holdout_used_for_fit_or_information":False, "training_inputs":inputs,
        "prior_information_equations":len(y), "prior_information":prior.tolist(),
        "position_source":str(args.position_journal), "current_position_rad":position,
        "current_posture_rad":posture, "posture_measurement_ns":position_stream["pitch"][-1]["receive_ns"],
        "envelope":asdict(envelope), "frozen_gyro_calibration":calibration,
        "nominal_parameter_samples":1, "uncertainty_reliable":False, "selected":selected,
        "scope":"existing program scorer compares 32 fixed templates in both directions and selects up to three cases including maximum-first preparation at the measured current fixed-pitch raw-count pose",
        "maximum_first_preparation_basis":"Existing owner-approved initial-stimulus amplitude and hold; complete ramp/hold/template/zero profile is included in the information calculation. Real movement and information remain postcapture evidence, not an arming qualification gate.",
        "observed_motion_evidence":failure_evidence,
        "observed_030A_response":"No reliable sustained-motion region; selected currents and native predictions are acquisition hypotheses, not physical success."}
    write_json(args.output_dir/"information-selection.json", report)
    print(json.dumps({"status":"NEXT_INFORMATION_SELECTED", "selected":[{key:row[key] for key in
        ("case_id","direction","manifest","score","duration_s","current_min_A","current_max_A","maximum_segment_slew_A_s")}
        for row in selected], "unqualified":True}))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--journal", type=Path, action="append", required=True)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--session-label", default="yaw-program-information-20260930")
    parser.add_argument("--fit-json", type=Path)
    parser.add_argument("--position-journal", type=Path)
    parser.add_argument("--failure-evidence", type=Path, action="append", default=[],
                        help="retain actual failed/limited motion observations in the next measurement plan")
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    if args.fit_json:
        return select_frozen_fit(args)
    baseline = json.loads(args.baseline.read_text())
    streams = [read_stream(path) for path in args.journal]
    identity = Identity("rpi-turret-yaw-GM6020", "raw-encoder-5768-fixed-pitch-gyro-column",
                        "current-payload-pitch-fixed-20260930", "MEASURED", provisional_labels=True)
    column, projection, bias, sigma_v, period, calibration = gyro_calibration(streams, baseline)
    write_json(args.output_dir/"gyro-column.json", calibration)
    posture = baseline["pitch_mechpos_rad"]["mean"]
    # These knots are only the native computation representation of a constant
    # provisional seed. Measured applicability is recorded separately below.
    spec = ModelSpec("yaw", tuple(np.linspace(-2*np.pi, 2*np.pi, 5)),
                     (posture-.1, posture, posture+.1))
    matrix = tying(spec)
    runs, descriptions = [], []
    for stream_index, stream in enumerate(streams):
        excitation = [row for row in stream["commands"] if row["phase"] == "excitation"]
        start = excitation[0]["kernel_accepted_ns"]
        intervals = baseline["breakaway_intervals"] if stream_index == 0 else []
        offset_s = 0.
        for index, segment in enumerate(stream["config"]["current_segments"]):
            duration = float(segment["duration_s"])
            interval = next((row for row in intervals if row["segment_index"] == index), None)
            if interval is not None and not interval["censored"]:
                begin_ns = start+int((offset_s+interval["sustained_confirmed_s"])*1e9)
                # The adjoining downward ramp remains in the same motion direction.
                end_s = offset_s+duration
                if index+1 < len(stream["config"]["current_segments"]):
                    following = stream["config"]["current_segments"][index+1]
                    if float(following["start_A"])*interval["direction"] > 0:
                        end_s += float(following["duration_s"])
                run, description = normalized_run(stream, begin_ns, start+int(end_s*1e9),
                    interval["direction"], identity, bias, projection, sigma_v, baseline,
                    f"yaw-seed-capture-{stream_index}-direction-{interval['direction']}")
                runs.append(run)
                descriptions.append(description)
            offset_s += duration
    write_json(args.output_dir/"normalized-runs.json", {"runs": descriptions,
        "limited_response_captures": [{"journal": stream["journal"], "footer": stream["footer"],
            "actual_displacement_rad": stream["yaw"][-1]["q_relative_rad"],
            "actual_peak_displacement_rad": max(row["q_relative_rad"] for row in stream["yaw"]),
            "used_for_gyro_column": True, "used_as_running_dynamic_fit": False}
            for stream in streams[1:]]})
    X, y, groups = integral_design(spec, runs, window_s=2*period)
    reduced_X = X @ matrix[:-1, :-1]
    guess, condition = bounded_initializer(reduced_X, y,
                                          lower_bounds=[1e-8, 0., -np.inf, -np.inf])
    delay_bound = float(streams[0]["config"]["limits"]["can_gap_s"])
    reduced_initial = np.r_[guess, delay_bound/4]
    native = Native(args.library)
    result = output_error(native, spec, matrix @ reduced_initial, runs, delay_bound,
                          parameter_map=matrix, reduced_initial=reduced_initial,
                          reduced_bounds=([1e-8,0.,-np.inf,-np.inf,0.],
                                          [np.inf,np.inf,np.inf,np.inf,delay_bound]))
    rr = residuals(native, spec, result.x, runs)
    seed = {"schema": "adr0022.yaw-provisional-seed/1", "identity": asdict(identity),
            "model_spec": asdict(spec), "theta": result.x.tolist(), "reduced_theta": result.reduced_x.tolist(),
            "parameter_labels": ["a", "b", "h_negative", "h_positive", "delay"],
            "parameter_map": matrix.tolist(), "normalized_condition": condition,
            "optimizer_evaluations": result.nfev, "normalized_residual_rms": float(np.sqrt(np.mean(rr**2))),
            "qualification": False, "independent_holdout": False, "uncertainty_reliable": False,
            "posture_applicability_rad": posture,
            "measured_position_range_rad": [min(row["position_range_rad"][0] for row in descriptions),
                                             max(row["position_range_rad"][1] for row in descriptions)],
            "scope": "Provisional information-selection seed only; repeated native slots are not measured coverage",
            "gyro_column": calibration, "delay_search_bound_s": delay_bound,
            "delay_bound_basis": "declared capture feedback deadline; not an identified delay confidence bound"}
    write_json(args.output_dir/"provisional-seed.json", seed)
    information = reduced_X.T @ reduced_X/baseline["current_sigma_A"]**2
    prior = np.zeros((spec.size, spec.size))
    prior[:-1,:-1] = np.linalg.pinv(matrix[:-1,:-1]).T @ information @ np.linalg.pinv(matrix[:-1,:-1])
    envelope = Envelope(.8, 1., None, None, None, None, None, 4., False, "MEASURED")
    chosen = choose_information(native, spec, result.x, result.x[None,:], envelope, prior,
        initial_position=streams[-1]["yaw"][-1]["q_relative_rad"]+streams[-1]["q_offset"],
        posture=posture, direction=1, noise_sigma=baseline["current_sigma_A"], sample_hz=1/period)
    selected = chosen[0]
    plan = initial_plan(label=args.session_label, approved_minimum_a=.25, current_bound_a=.8,
        factor_index=0, pulse_s=.5, baseline_s=2., zero_between_s=2., startup_s=20., duration_s=40.)
    t, current = selected["time"], selected["successful_tx"]
    segments = [{"duration_s": float(t[k+1]-t[k]), "start_A": float(current[k]),
                 "end_A": float(current[k+1])} for k in range(len(t)-1)]
    plan["current_segments"] = segments
    plan["signal_selection"] = {"method": "choose_information", "case_id": selected["case_id"],
        "score_logdet_per_s": selected["score"], "template_count": 32,
        "plant_prediction_available": True, "plant_qualified": False,
        "scope": "one finite next measurement at current fixed pitch, provisional constant local plant",
        "capability_basis": "existing 0.8 A production allowance; measured 1 A/s requested ramp basis",
        "stop_verified": False, "kinematic_safety_bounds": None,
        "seed": "provisional-seed.json"}
    from adr0022_capture_launch import validate_yaw_contract
    validate_yaw_contract(plan)
    write_json(args.output_dir/"manifest.json", plan)
    write_json(args.output_dir/"information-selection.json", {"schema": "adr0022.yaw-information-selection/1",
        "selected": [{"case_id": row["case_id"], "score": row["score"],
                      "current_range_A": [float(row["successful_tx"].min()),float(row["successful_tx"].max())],
                      "duration_s": float(row["time"][-1])} for row in chosen],
        "envelope": asdict(envelope), "seed_qualified": False})
    print(json.dumps({"status": "NEXT_INFORMATION_SELECTED", "manifest": str(args.output_dir/"manifest.json"),
                      "case_id": selected["case_id"], "reduced_theta": result.reduced_x.tolist()}))


if __name__ == "__main__":
    main()
