"""TRAIN-only censored breakaway intervals and bounded native hybrid fitting.

This first method has an explicit constant-load gauge and known algebraic input
and sensor nuisances.  It estimates directional *total* loads; neither a local
zero derivative nor a grid representative identifies exact physical friction.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, replace
from itertools import product
import time

import numpy as np

from .contracts import Reason, require
from .model_family import _errors, fit_family, moving_integral_initializer


MOVING_COORDINATES = ("a", "viscous", "coulomb_negative", "coulomb_positive")


@dataclass(frozen=True)
class PlateauTrial:
    run_id: str
    trial_id: str
    direction: int
    command_time: float
    end_time: float
    rest_start: float


@dataclass(frozen=True)
class RestSupport:
    """Independent support for actual rest; quiet sensor bands are insufficient.

    Physical qualification must supply its own evidence. An independent synthetic
    oracle certificate is accepted only for a SYNTHETIC acquisition and cannot
    become physical rest evidence through otherwise matching sensor values.
    """
    run_id: str
    trial_id: str
    provenance: str
    basis: str
    rest_from_s: float
    rest_until_s: float
    actual_rest_supported: bool
    evidence_reference: str

    def validate(self):
        require(bool(self.run_id) and bool(self.trial_id) and bool(self.evidence_reference)
            and self.provenance in ("SYNTHETIC","MEASURED") and type(self.actual_rest_supported) is bool
            and np.isfinite([self.rest_from_s,self.rest_until_s]).all()
            and self.rest_from_s < self.rest_until_s,Reason.DATA_INVALID,
            "typed independent rest support with finite coverage and evidence required")
        require(self.basis in ("INDEPENDENT_SYNTHETIC_ORACLE","QUALIFIED_PHYSICAL_REST_EVIDENCE",
            "BOUNDED_PREHISTORY_STOP_PROOF"),Reason.DATA_INVALID,
            "quiet encoder/gyro observations cannot certify actual rest")
        require(self.basis != "INDEPENDENT_SYNTHETIC_ORACLE" or self.provenance == "SYNTHETIC",
            Reason.DATA_INVALID,"synthetic rest certificate cannot qualify physical rest")
        require(self.basis != "QUALIFIED_PHYSICAL_REST_EVIDENCE" or self.provenance == "MEASURED",
            Reason.DATA_INVALID,"physical rest support must refer to measured evidence")
        return self


@dataclass(frozen=True)
class ThresholdPolicy:
    """Supplied detection/uncertainty settings, never a new physical quality gate."""
    minimum_rest_s: float = .15
    persistence_s: float = .04
    velocity_sigma_multiple: float = 5.
    minimum_velocity_rad_s: float = .025
    displacement_sigma_multiple: float = 6.
    effective_input_uncertainty_A: float = 0.
    grid_fractions: tuple = (.25, .5, .75)
    max_inner_nfev: int = 120

    def validate(self):
        require(np.isfinite([self.minimum_rest_s, self.persistence_s,
            self.velocity_sigma_multiple, self.minimum_velocity_rad_s,
            self.displacement_sigma_multiple, self.effective_input_uncertainty_A]).all()
            and self.minimum_rest_s > 0 and self.persistence_s > 0
            and self.velocity_sigma_multiple > 0 and self.minimum_velocity_rad_s > 0
            and self.displacement_sigma_multiple > 0 and self.effective_input_uncertainty_A >= 0
            and 1 <= self.max_inner_nfev <= 2000, Reason.DATA_INVALID,
            "finite positive supplied detection rules and bounded fit budget required")
        require(0 < len(self.grid_fractions) <= 5 and len(set(self.grid_fractions)) == len(self.grid_fractions)
            and all(np.isfinite(x) and 0 < x < 1 for x in self.grid_fractions), Reason.DATA_INVALID,
            "declare unique bounded interior threshold fractions")
        return self


def _supported(model, bounds):
    model.validate()
    require(model.actuator == "algebraic" and model.friction == "coulomb"
        and model.load == "constant" and model.load_offset == 0,
        Reason.INSUFFICIENT_EXCITATION,
        "early threshold method requires algebraic Coulomb constant total-load gauge load_offset=0")
    require(set(bounds) == set(MOVING_COORDINATES), Reason.DATA_INVALID,
        "early method fits four moving totals with known supplied nuisance parameters")
    require(all(len(pair)==2 and np.isfinite(pair).all() and 0 <= pair[0] < pair[1]
        for pair in bounds.values()) and bounds["a"][0] > 0, Reason.DATA_INVALID,
        "finite physically admissible moving-total fit bounds required")


def threshold_outcome_support(interval, directional_effective_A, *, input_uncertainty_A=0.):
    """Conditional rest outcome supported by the *whole* identified interval.

    Sensor/actuator and rest-state qualification are still separate preconditions.
    An interior interval command remains ambiguous; a chosen grid representative
    must never silently turn that ambiguity into an identified start prediction.
    """
    require(np.isfinite([directional_effective_A,input_uncertainty_A]).all()
        and directional_effective_A >= 0 and input_uncertainty_A >= 0,
        Reason.DATA_INVALID,"finite directional input and nonnegative uncertainty required")
    lower,upper=interval["lower_A"],interval["upper_A"]
    require(np.isfinite([lower,upper]).all() and 0 <= lower < upper,
        Reason.DATA_INVALID,"consistent directional interval required")
    if directional_effective_A+input_uncertainty_A <= lower:
        return "SUPPORTED_STATIC_HOLD"
    release_boundary = directional_effective_A-input_uncertainty_A
    if release_boundary > upper or (release_boundary == upper and not interval["upper_inclusive"]):
        return "SUPPORTED_STATIC_RELEASE"
    return "AMBIGUOUS_INSIDE_THRESHOLD_INTERVAL"


def _motion_onset(times, velocity, threshold, persistence, direction):
    active = direction * velocity > threshold
    starts = np.flatnonzero(active & np.r_[True, ~active[:-1]])
    for start in starts:
        stops = np.flatnonzero(~active[start:])
        end = start + stops[0] if len(stops) else len(times)
        if times[end - 1] - times[start] >= persistence - 1e-12:
            return float(times[start])
    return None


def censored_threshold_intervals(model, train, trials, *, bounds, threshold_bounds,
                                policy=ThresholdPolicy(),rest_support=()):
    """Infer conservative directional intervals from declared constant plateaus.

    A no-start only constrains a static threshold when a hypothetical sliding
    response exceeds the supplied detection bands for *every* allowed moving
    parameter. Otherwise it remains dynamically censored and adds no bound.
    Actual rest needs separate independent support. Native quiet bands only
    check observation consistency; creeping can pass them. Unsupported/ambiguous
    trials remain in the record and add no static inequality.
    """
    _supported(model, bounds); policy.validate()
    for run in train: run.validate()
    runs = {run.run_id: run for run in train}
    require(len(runs) == len(train) and bool(runs), Reason.DATA_INVALID,
            "unique nonempty whole TRAIN acquisitions required")
    require(bool(trials) and len({(x.run_id,x.trial_id) for x in trials}) == len(trials),
        Reason.DATA_INVALID,"unique declared TRAIN plateau identities required")
    require(all(isinstance(support,RestSupport) for support in rest_support),Reason.DATA_INVALID,
        "typed independent rest support required")
    support_by_trial={}
    trial_ids={(x.run_id,x.trial_id) for x in trials}
    for support in rest_support:
        support.validate(); key=(support.run_id,support.trial_id)
        require(key in trial_ids and key not in support_by_trial and
            support.provenance == runs[support.run_id].provenance,Reason.DATA_INVALID,
            "independent rest support must uniquely match its TRAIN acquisition and provenance")
        support_by_trial[key]=support
    require(set(threshold_bounds) == {"negative", "positive"} and all(
        np.isfinite(pair).all() and 0 <= pair[0] < pair[1] for pair in threshold_bounds.values()),
        Reason.DATA_INVALID, "finite directional threshold search intervals required")
    intervals = {direction: {"lower_A": float(pair[0]), "upper_A": float(pair[1]),
        "lower_inclusive": True, "upper_inclusive": True,
        "no_start_constraints": 0, "start_constraints": 0} for direction,pair in threshold_bounds.items()}
    records = []
    for trial in trials:
        require(trial.run_id in runs and trial.direction in (-1, 1)
            and np.isfinite([trial.command_time, trial.end_time, trial.rest_start]).all()
            and trial.rest_start < trial.command_time < trial.end_time,
            Reason.DATA_INVALID, "declared trial must reference TRAIN and a finite rest/hold window")
        run = runs[trial.run_id]
        require(run.t[0] <= trial.rest_start and trial.end_time <= run.t[-1], Reason.DATA_INVALID,
                "trial window outside its whole acquisition")
        index = np.searchsorted(run.tx_t, trial.command_time + 1e-12, side="right") - 1
        end_index=np.searchsorted(run.tx_t,trial.end_time-1e-12,side="left")
        require(index >= 0 and np.all(run.tx_A[index:end_index] == run.tx_A[index]),
                Reason.DATA_INVALID, "trial requires a constant successful-TX plateau")
        command = float(model.actuator_gain * run.tx_A[index] + model.actuator_bias)
        magnitude = trial.direction * command
        require(magnitude > 0, Reason.DATA_INVALID, "plateau direction disagrees with effective command")
        velocity_limit = max(policy.minimum_velocity_rad_s,policy.velocity_sigma_multiple*run.sigma_v)
        q_limit = policy.displacement_sigma_multiple * (run.sigma_q + run.encoder_quantum / 2)
        # Rest is assessed before the command, allowing the known sensor delay.
        rest = run.v_new & (run.t >= trial.rest_start) & (run.t < trial.command_time)
        rest_q = run.q_new & (run.t >= trial.rest_start) & (run.t < trial.command_time)
        quiet = rest.sum() >= 3 and rest_q.sum() >= 3 and \
            trial.command_time-trial.rest_start >= policy.minimum_rest_s and \
            np.max(np.abs(run.v[rest]-model.gyro_bias)) <= velocity_limit and \
            np.ptp(run.q[rest_q]) <= q_limit
        support=support_by_trial.get((trial.run_id,trial.trial_id))
        established = support is not None and support.actual_rest_supported and \
            support.rest_from_s <= trial.rest_start+1e-12 and support.rest_until_s >= trial.command_time-1e-12
        start_time = trial.command_time + model.transport_delay + model.gyro_delay
        observed = run.v_new & (run.t >= start_time) & (run.t < trial.end_time)
        observed_q = run.q_new & (run.t >= trial.command_time) & (run.t < trial.end_time)
        require(observed.sum() >= 3 and observed_q.sum() >= 3, Reason.DATA_INVALID,
                "native gyro and encoder observations needed during plateau")
        onset = _motion_onset(run.t[observed],run.v[observed]-model.gyro_bias,
                             velocity_limit,policy.persistence_s,trial.direction)
        displacement = trial.direction * (run.q[observed_q][-1]-run.q[observed_q][0])
        duration = max(0., trial.end_time-start_time-policy.persistence_s)
        # Worst supported inertia, drag and moving total minimize this sliding
        # response. Filter lag allowance leaves a conservative usable duration.
        d = "positive" if trial.direction > 0 else "negative"
        inertia = bounds["a"][1]; drag = bounds["viscous"][1]
        net = magnitude - policy.effective_input_uncertainty_A - bounds["coulomb_"+d][1]
        usable = max(0.,duration-5*model.gyro_tau)
        if drag > 0:
            v_min = max(0.,net)/drag * (-np.expm1(-drag*usable/inertia))
            q_min = max(0.,net)/drag * (usable+np.expm1(-drag*usable/inertia)/(drag/inertia))
        else:
            v_min = max(0.,net)*usable/inertia; q_min = .5*max(0.,net)*usable**2/inertia
        # Over the last five filter constants, monotone sliding velocity is at
        # least v_min. The zero-initialized positive filter response is therefore
        # at least this attenuated value; latent v_min alone is not gyro evidence.
        gyro_min=v_min*(-np.expm1(-5)) if model.gyro_tau > 0 else v_min
        detectable = gyro_min > velocity_limit and q_min > q_limit
        record = {**asdict(trial),"effective_plateau_A":command,"directional_total_magnitude_A":magnitude,
            "observation_window_quiet":bool(quiet),"actual_rest_established":bool(established),
            "independent_rest_support":asdict(support) if support is not None else None,"observed_onset_s":onset,
            "observed_directional_displacement_rad":float(displacement),
            "hypothetical_sliding_detectable_over_allowed_moving_bounds":bool(detectable),
            "conservative_sliding_velocity_rad_s":float(v_min),"conservative_sliding_displacement_rad":float(q_min),
            "conservative_filtered_gyro_rad_s":float(gyro_min),
            "velocity_detection_rad_s":float(velocity_limit),"displacement_detection_rad":float(q_limit)}
        if not established:
            record["outcome"]="AMBIGUOUS_REST_NOT_ESTABLISHED"
            record["reason"]="STATIC_INEQUALITY_REQUIRES_INDEPENDENT_ACTUAL_REST_SUPPORT"
        elif not quiet:
            record["outcome"]="UNKNOWN_OBSERVATIONS_CONTRADICT_REST_SUPPORT"
        elif onset is not None and displacement > q_limit:
            record["outcome"]="START"; interval=intervals[d]
            upper=magnitude+policy.effective_input_uncertainty_A
            if upper <= interval["upper_A"]:
                interval.update(upper_A=float(upper),upper_inclusive=False)
            interval["start_constraints"] += 1
        elif onset is None and abs(displacement) <= q_limit and \
                np.max(np.abs(run.v[observed]-model.gyro_bias)) <= velocity_limit and detectable:
            record["outcome"]="NO_START_CENSORED"; interval=intervals[d]
            interval["lower_A"]=max(interval["lower_A"],magnitude-policy.effective_input_uncertainty_A)
            interval["no_start_constraints"] += 1
        else:
            record["outcome"]="DYNAMICALLY_OR_OBSERVATION_CENSORED"
        records.append(record)
    inconsistent = [d for d,x in intervals.items() if x["lower_A"] >= x["upper_A"]]
    for d,interval in intervals.items():
        interval["width_A"]=interval["upper_A"]-interval["lower_A"]
        interval["status"] = "INCONSISTENT" if d in inconsistent else "BOUNDED_BY_START_AND_NO_START" if \
            interval["start_constraints"] and interval["no_start_constraints"] else "ONE_SIDED_OR_PRIOR_ONLY"
    return {"status":"INCONSISTENT" if inconsistent else "COMPUTED_TRAIN_ONLY",
        "intervals":intervals,"trials":records,"selection_or_holdout_used":False,
        "gauge":"load_offset=0; Fc/Fs are directional total input-equivalent loads, not unique physical friction",
        "input_uncertainty_A":policy.effective_input_uncertainty_A,
        "rest_support":"independent actual-rest evidence required; quiet sensor bands alone never establish sticking",
        "physical_rest_qualification_from_synthetic":False,
        "interval_confidence_calibrated":False,
        "exact_static_friction_identified":False}


def fit_threshold_family(native, model, train, trials, *, bounds, threshold_bounds,
                         policy=ThresholdPolicy(), progress=None,rest_support=()):
    """Bounded automatic outer interval/grid + native inner moving-total fit."""
    began=time.monotonic()
    interval_record=censored_threshold_intervals(model,train,trials,bounds=bounds,
        threshold_bounds=threshold_bounds,policy=policy,rest_support=rest_support)
    require(interval_record["status"] != "INCONSISTENT", Reason.MODEL_INADEQUATE,
            "censored threshold inequalities inconsistent; route to input/state/structure diagnosis")
    intervals=interval_record["intervals"]
    require(all(x["status"] == "BOUNDED_BY_START_AND_NO_START" for x in intervals.values()),
            Reason.INSUFFICIENT_EXCITATION,"both signs require detectable censored no-start and start evidence")
    require(all(bounds["coulomb_"+d][1] <= intervals[d]["lower_A"] for d in intervals),
            Reason.INSUFFICIENT_EXCITATION,"moving-total bounds overlap static interval; joint bounded continuation required")
    seed,seed_record=moving_integral_initializer(model,train,bounds)
    if seed is None: seed=model
    grid={d:[x["lower_A"]+fraction*x["width_A"] for fraction in policy.grid_fractions]
          for d,x in intervals.items()}
    candidates=[]; selected_fit=None; best_key=None
    for negative,positive in product(grid["negative"],grid["positive"]):
        candidate=replace(seed,static_negative=negative,static_positive=positive)
        fit=fit_family(native,candidate,train,bounds=bounds,max_nfev=policy.max_inner_nfev,progress=progress)
        predictions=[native.rollout(fit["model"],r.t,r.tx_t,r.tx_A,np.asarray(initial))
                     for r,initial in zip(train,fit["optimizer"]["initial_latent_states"],strict=True)]
        raw=np.concatenate([np.concatenate([error/scale for error,scale in zip(_errors(r,p),
            (r.sigma_q,r.sigma_v,r.sigma_current))]) for r,p in zip(train,predictions,strict=True)])
        cost=float(np.sum(np.where(np.abs(raw)<=1,.5*raw**2,np.abs(raw)-.5)))
        row={"static_negative_A":negative,"static_positive_A":positive,"training_huber_cost":cost,
            "optimizer_converged":fit["optimizer"]["success"],
            "optimizer_termination_reason":fit["optimizer"]["message"],
            "optimizer_evaluations":fit["optimizer"]["evaluations"],
            "residual_evaluations":fit["optimizer"]["residual_evaluations_including_jacobian_and_diagnostics"],
            "model":fit["model"].document(),"training":fit["training"]}
        candidates.append(row)
        # Equivalent plateau likelihoods choose central representation, not a
        # false precision claim. Failed optimizers never silently win promotion.
        middle_distance=sum(abs(value-(intervals[d]["lower_A"]+intervals[d]["upper_A"])/2)
                            for d,value in (("negative",negative),("positive",positive)))
        key=(not row["optimizer_converged"],round(cost,6),middle_distance)
        if best_key is None or key < best_key: selected_fit=fit;best_key=key
    selected_fit["threshold_identification"]={"schema":"adr0022.censored-threshold-fit/1",
        "interval_evidence":interval_record,"policy":asdict(policy),"moving_seed":seed_record,
        "outer_candidates":candidates,"outer_candidates_evaluated":len(candidates),
        "training_outer_cost_span":max(x["training_huber_cost"] for x in candidates)-min(x["training_huber_cost"] for x in candidates),
        "total_inner_residual_evaluations":sum(x["residual_evaluations"] for x in candidates),
        "selection_rule":"TRAIN native Huber cost; sub-micro objective ties use central interval representation",
        "representative_is_exact_static_estimate":False,"selected_thresholds_A":{
            d:getattr(selected_fit["model"],"static_"+d) for d in intervals},
        "outer_wall_time_s":time.monotonic()-began,"selection_or_holdout_used":False,
        "scope":"known supplied nuisance, constant total-load gauge, algebraic Coulomb, declared settled plateaus",
        "physical_stage3a":"NOT_RUN","physical_stage3b":"NOT_RUN","deployment_authorized":False}
    return selected_fit
