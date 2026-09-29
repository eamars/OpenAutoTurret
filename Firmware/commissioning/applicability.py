"""Bounded operating domains, protection facts, and selective asset invalidation."""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from .contracts import Reason,digest,require


@dataclass(frozen=True)
class OperatingDomain:
    hardware_hash: str
    measurement_hash: str
    categories: dict
    intervals: dict
    unknown_permitted_with_response: tuple[str,...] = ()

    def check(self,hardware,measurement,point,*,independent_prediction_passed):
        require(hardware==self.hardware_hash and measurement==self.measurement_hash,
                Reason.OPERATING_POINT_CHANGED,"hardware or measurement domain changed")
        for name,allowed in self.categories.items():
            require(point.get(name) in allowed,Reason.OPERATING_POINT_CHANGED,
                    f"uncovered operating category: {name}")
        for name,bounds in self.intervals.items():
            value=point.get(name)
            if value is None and name in self.unknown_permitted_with_response:
                require(independent_prediction_passed is True,Reason.OPERATING_POINT_CHANGED,
                        f"unknown {name} requires independent current response validation")
                continue
            require(type(value) in (int,float) and np.isfinite(value),Reason.DATA_INVALID,
                    f"unknown/nonfinite required applicability variable: {name}")
            require(len(bounds)==2 and np.isfinite(bounds).all() and bounds[0]<=value<=bounds[1],
                    Reason.OPERATING_POINT_CHANGED,f"outside identified {name} interval; no extrapolation")
        return True


def protection_decision(observation,bounds,*,elapsed_s):
    """Danger and inability to claim continuous qualification are different results.

    The result specifies a response to the output owner. It never invents a zero-current
    'hold' or sends an independent motor command.
    """
    require(np.isfinite(elapsed_s) and elapsed_s>=0,Reason.DATA_INVALID,"invalid elapsed time")
    if observation.get("estop") is True or observation.get("drive_fault") is True:
        return {"reason":"HARD_ABORT","response":"OWNER_VERIFIED_STOP","continuous_qualified":False}
    if observation.get("feedback_valid") is not True:
        return {"reason":"HARD_ABORT","response":"OWNER_VERIFIED_STOP","continuous_qualified":False}
    unknown=[]
    for name in ("current_A","temperature_C","bus_V","position_rad"):
        require(name in bounds,Reason.DATA_INVALID,f"missing injected protection bound {name}")
        value=observation.get(name);limits=bounds[name]
        require(len(limits)==2 and np.isfinite(limits).all() and limits[0]<limits[1],
                Reason.DATA_INVALID,f"invalid protection interval {name}")
        if value is None:
            unknown.append(name);continue
        require(type(value) in (int,float) and np.isfinite(value),Reason.DATA_INVALID,
                f"invalid supervision reading {name}")
        if not limits[0]<=value<=limits[1]:
            return {"reason":"HARD_ABORT","response":"OWNER_VERIFIED_STOP","continuous_qualified":False}
    if elapsed_s>bounds["approved_duration_s"]:
        return {"reason":"ENVELOPE_LIMITED","response":"END_QUALIFICATION_KEEP_VERIFIED_SUPPORT",
                "continuous_qualified":False}
    if unknown:
        finite=(unknown==["temperature_C"] and elapsed_s<=bounds.get("approved_unknown_temperature_window_s",-1))
        return {"reason":"MEASUREMENT_LIMITED","response":"CONTINUE_APPROVED_FINITE_WINDOW" if finite else
                "END_QUALIFICATION_KEEP_VERIFIED_SUPPORT","continuous_qualified":False}
    return {"reason":None,"response":"CONTINUE_WITHIN_APPROVED_ENVELOPE","continuous_qualified":False}


def thermal_equilibrium(time_s,temperature_C,*,temperature_max_C,required_margin_C):
    t=np.asarray(time_s,float);temperature=np.asarray(temperature_C,float)
    require(t.ndim==1 and len(t)>=30 and temperature.shape==t.shape and np.isfinite(t).all()
            and np.isfinite(temperature).all() and np.all(np.diff(t)>0),
            Reason.MEASUREMENT_LIMITED,"valid temperature samples required for thermal qualification")
    require(t[-1]-t[0]>=1800,Reason.MEASUREMENT_LIMITED,"less than minimum 30 minute observation")
    require(np.max(np.diff(t))<=60,Reason.MEASUREMENT_LIMITED,"temperature coverage has gaps over one minute")
    slopes=[]
    for end in (t[-1]-600,t[-1]):
        mask=(t>=end-600)&(t<=end)
        require(mask.sum()>=10,Reason.MEASUREMENT_LIMITED,"insufficient samples in thermal windows")
        slopes.append(float(np.polyfit((t[mask]-end)/60,temperature[mask],1)[0]))
    passed=all(abs(s)<.1 for s in slopes) and temperature.max()<=temperature_max_C-required_margin_C
    return {"passed":bool(passed),"window_slopes_C_per_min":slopes,
            "observed_margin_C":float(temperature_max_C-temperature.max())}


def invalidated_assets(changed_fields):
    affected=set()
    for field in changed_fields:
        if field.startswith("hardware.yaw"):
            affected.update(("yaw_capability","yaw_plant","dual_axis_validation","3a","3b"))
        elif field.startswith("hardware.pitch"):
            affected.update(("pitch_capability","pitch_plant","yaw_posture_family","dual_axis_validation","3a","3b"))
        elif field.startswith("measurement.installation"):
            affected.update(("measurement_calibration","observer","affected_plant_observations","3a","3b"))
        elif field.startswith("measurement.session"):
            affected.update(("session_mapping","session_readiness"))
        elif field.startswith("software.core"):
            affected.update(("core_build","observer","controller_candidate","3a","3b"))
        elif field.startswith("software.production"):
            affected.update(("end_to_end_latency","3b"))
        elif field.startswith("operating_point"):
            affected.update(("applicability_check","parameter_snapshot_if_residual_changed","3a","3b"))
        else:
            require(False,Reason.DATA_INVALID,f"unclassified identity change {field}")
    return {"invalidate":sorted(affected),"preserve":["immutable_raw_history","model_method","identification_solver"]}


def applicable_rollback(snapshots,domain,point,*,hardware,measurement,prediction_checks):
    domain.check(hardware,measurement,point,independent_prediction_passed=True)
    matching=[s for s in snapshots if s.identity.hardware==hardware and s.identity.measurement==measurement
              and prediction_checks.get(s.identity_hash) is True]
    require(bool(matching),Reason.OPERATING_POINT_CHANGED,
            "no applicable historical snapshot; retain verified support/stop instead of blind rollback")
    return sorted(matching,key=lambda s:s.identity_hash)[0]
