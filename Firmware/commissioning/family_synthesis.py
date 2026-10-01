"""Bounded automatic offline synthesis for a declared selected-family slice.

Candidates use the existing PI structure. The complete selected-family sampled
lift decides feasibility; the analytic bandwidth curve is only an initializer.
No candidate is applied, promoted, or physically qualified by this module.
"""
from dataclasses import asdict, dataclass
from enum import Enum
import math

from .contracts import Reason, Rejected, require
from .family_analysis import SlidingPoint, SlidingSupport
from .family_sampled_analysis import FamilySampledAnalysis, LocalGains, SampleSchedule
from .model_family import FamilyModel


class FamilySynthesisStatus(str, Enum):
    SELECTED = "SELECTED_OFFLINE_LOCAL_CANDIDATE"
    UNSUPPORTED = "UNSUPPORTED_SELECTED_FAMILY_SYNTHESIS_SLICE"
    NO_FEASIBLE = "NO_FEASIBLE_IN_DECLARED_GRID"


@dataclass(frozen=True)
class FamilySynthesisResult:
    status: FamilySynthesisStatus
    reason: Reason | None
    detail: str
    selected_wn_rad_s: float | None
    gains: LocalGains | None
    candidates: tuple
    minimum_incremental_damping_A_s_rad: float | None
    selected_damping_ratio: float | None = None
    phase_required_deg: float = 50.
    gain_required_db: float = 6.

    def document(self):
        return {"schema": "adr0022.selected-family-synthesis/1",
            "status": self.status.value, "reason": self.reason.value if self.reason else None,
            "detail": self.detail, "selected_wn_rad_s": self.selected_wn_rad_s,
            "selected_damping_ratio": self.selected_damping_ratio,
            "phase_required_deg": self.phase_required_deg, "gain_required_db": self.gain_required_db,
            "gains": asdict(self.gains) if self.gains else None,
            "minimum_incremental_damping_A_s_rad": self.minimum_incremental_damping_A_s_rad,
            "candidates": list(self.candidates),
            "qualification": "OFFLINE_LOCAL_CANDIDATE_ONLY",
            "nonlinear_qualification": "NOT_RUN", "physical_qualification": "NOT_RUN",
            "limitations": ["supplied selected Coulomb or Stribeck family and declared signed sliding points",
                "one declared finite bandwidth curve; not a proof of global infeasibility",
                "unquantized periodic local sliding and inactive limits/AW",
                "steady-state reference FF leaves actuator dynamics and delays in the forward plant",
                "native observable and nonlinear forecast checks remain separate"]}


def solve_selected_family(model, points, supports, observer, nominal_gains, schedule, *,
                          wn_grid, ff_policy="SHARED_POSTERIOR_SLIDE_ALGEBRAIC",
                          damping_ratio_grid=(1.,), phase_required_deg=50., gain_required_db=6.):
    """Return fastest locally feasible candidate, or an explicit typed failure.

    The positive finite bandwidth grid is supplied before evaluation. In command
    units, Kp=(2*zeta*a*wn-min(B+dF/dv))/gain and Ki=a*wn**2/gain; Kpos=wn/5.
    Kaw stays at the supplied nominal value because AW is inactive in this lift.
    All declared candidates and operating-point outcomes are retained. Dynamic
    FF uses an explicit steady-state reference policy, never actuator inversion.
    """
    require(isinstance(model, FamilyModel) and isinstance(nominal_gains, LocalGains)
            and isinstance(schedule, SampleSchedule), Reason.DATA_INVALID,
            "typed selected family, nominal gains and sampling required")
    model.validate(); nominal_gains.validate()
    require(all(isinstance(x,(int,float)) and not isinstance(x,bool) and math.isfinite(x) and x>0
                for x in (phase_required_deg,gain_required_db)), Reason.DATA_INVALID,
            "explicit positive finite phase and gain margin requirements required")
    requirements = dict(phase_required_deg=float(phase_required_deg),gain_required_db=float(gain_required_db))
    require(isinstance(wn_grid, (tuple, list)) and 0 < len(wn_grid) <= 32 and
            all(isinstance(x, (int, float)) and not isinstance(x, bool) and
                math.isfinite(x) and x > 0 for x in wn_grid) and
            all(b > a for a,b in zip(wn_grid, wn_grid[1:])),
            Reason.DATA_INVALID, "explicit increasing positive finite bandwidth grid of at most 32 points required")
    require(isinstance(damping_ratio_grid,(tuple,list)) and 0<len(damping_ratio_grid)<=8 and
            all(isinstance(x,(int,float)) and not isinstance(x,bool) and math.isfinite(x) and x>0
                for x in damping_ratio_grid) and
            all(b>a for a,b in zip(damping_ratio_grid,damping_ratio_grid[1:])) and
            len(wn_grid)*len(damping_ratio_grid)<=128, Reason.DATA_INVALID,
            "explicit increasing positive damping-ratio curve and at most 128 total candidates required")
    policies = ("SHARED_POSTERIOR_SLIDE_ALGEBRAIC", "SHARED_POSTERIOR_STEADY_STATE_REFERENCE")
    if model.friction not in ("coulomb","stribeck") or ff_policy not in policies or \
            (ff_policy == policies[0] and (model.actuator != "algebraic" or model.actuator_tau != 0. or model.transport_delay != 0.)):
        return FamilySynthesisResult(FamilySynthesisStatus.UNSUPPORTED, Reason.INTEGRATION_MISMATCH,
            "automatic slice requires selected Coulomb or Stribeck and an explicit supported shared posterior FF policy",
            None, None, (), None, **requirements)
    require(isinstance(points, (tuple, list)) and isinstance(supports, (tuple, list)) and
            2 <= len(points) == len(supports) <= 32 and
            all(isinstance(p, SlidingPoint) for p in points) and
            all(isinstance(s, SlidingSupport) for s in supports), Reason.DATA_INVALID,
            "two to 32 typed declared local points and corresponding sliding supports required")
    require(all(isinstance(p.v_rad_s, (int, float)) and not isinstance(p.v_rad_s, bool) and
                math.isfinite(p.v_rad_s) for p in points), Reason.DATA_INVALID, "finite declared sliding speeds required")
    if not any(p.v_rad_s < 0 for p in points) or not any(p.v_rad_s > 0 for p in points) or \
            any(p.v_rad_s == 0 for p in points):
        return FamilySynthesisResult(FamilySynthesisStatus.UNSUPPORTED, Reason.OPERATING_POINT_CHANGED,
            "automatic sliding synthesis requires declared nonzero points in both directions; rest is nonlinear",
            None, None, (), None, **requirements)
    try:
        prerequisites = [FamilySampledAnalysis(model,p,s,observer,nominal_gains,schedule,ff_policy=ff_policy)
                         for p,s in zip(points,supports)]
    except Rejected as exc:
        if exc.reason == Reason.DATA_INVALID: raise
        return FamilySynthesisResult(FamilySynthesisStatus.UNSUPPORTED,exc.reason,
            "selected-family local-analysis prerequisite failed: "+exc.detail,None,None,(),None,**requirements)
    damping = min(analysis.local.incremental_damping_A_s_rad for analysis in prerequisites)
    records, best = [], None
    for wn,zeta in ((w,z) for w in wn_grid for z in damping_ratio_grid):
        gains = LocalGains((2*zeta*model.a*wn-damping)/model.actuator_gain,
            model.a*wn**2/model.actuator_gain, wn/5., nominal_gains.kaw)
        record = {"wn_rad_s":float(wn), "damping_ratio":float(zeta), "gains":asdict(gains), "points":[], "passed":True}
        try:
            gains.validate()
        except Rejected as exc:
            record.update(passed=False, reason=exc.reason.value, detail=exc.detail)
        if record["passed"]:
            for point,support in zip(points,supports):
                point_record = {"q_rad":point.q_rad, "v_rad_s":point.v_rad_s}
                try:
                    analysis = FamilySampledAnalysis(model,point,support,observer,gains,schedule,ff_policy=ff_policy)
                    margins = analysis.margin_diagnostics(**requirements)
                    point_record.update(passed=bool(margins["passed"]), diagnostics=margins)
                    if not margins["passed"]:
                        point_record.update(reason=Reason.ENVELOPE_LIMITED.value,
                            detail=f"selected-family poles or {phase_required_deg:g}-degree/{gain_required_db:g}-dB local margins failed")
                except Rejected as exc:
                    point_record.update(passed=False,reason=exc.reason.value,detail=exc.detail)
                record["points"].append(point_record)
                record["passed"] &= point_record["passed"]
        records.append(record)
        # The first passing damping ratio at the fastest bandwidth wins ties.
        if record["passed"] and (best is None or wn>best[0]): best = (float(wn),gains,float(zeta))
    if best is None:
        return FamilySynthesisResult(FamilySynthesisStatus.NO_FEASIBLE, Reason.ENVELOPE_LIMITED,
            "no candidate passes all declared selected-family local pole/margin cases in this bandwidth grid",
            None,None,tuple(records),damping,**requirements)
    return FamilySynthesisResult(FamilySynthesisStatus.SELECTED,None,
        f"fastest candidate passing all declared local pole and {phase_required_deg:g}-degree/{gain_required_db:g}-dB checks; nonlinear/native gates remain separate",
        best[0],best[1],tuple(records),damping,best[2],**requirements)
