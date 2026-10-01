"""Deterministic offline controller calculation; never drives hardware candidates."""
from __future__ import annotations
from dataclasses import asdict
import hashlib
import numpy as np
from scipy.optimize import brentq

from .contracts import Reason, Rejected, digest, require
from .model import coefficients
from .native import parameters, Simulation
from .metrics import motion_metrics, shaped_velocity, shaped_step


def selected_family_linearization(model, point, support):
    """Offline selected-family plant/measurement tangent, with explicit delays.

    This additive interface supplies no gains or closed-loop margins. Dynamic
    actuator/sensor states and friction derivatives enter the separate
    selected_family_sampled_analysis interface with explicit timing and FF policy.
    Rest/reversal remain nonlinear hybrid validation tasks.
    """
    from .family_analysis import linearize_sliding_family
    return linearize_sliding_family(model, point, support)


def selected_family_sampled_analysis(model, point, support, observer, gains, schedule, *, ff_policy):
    """Unconstrained offline sliding lift; physical/native qualification separate."""
    from .family_sampled_analysis import FamilySampledAnalysis
    return FamilySampledAnalysis(model, point, support, observer, gains, schedule, ff_policy=ff_policy)


def selected_family_solve(model, points, supports, observer, nominal_gains, schedule, *, wn_grid,
                          ff_policy="SHARED_POSTERIOR_SLIDE_ALGEBRAIC", damping_ratio_grid=(1.,),
                          phase_required_deg=50., gain_required_db=6.):
    """Bounded declared sliding synthesis with explicit shared posterior FF policy.

    Dynamic steady-state-reference compensation leaves selected actuator lag and
    transport in the forward plant. This never applies or promotes a candidate.
    """
    from .family_synthesis import solve_selected_family
    return solve_selected_family(model, points, supports, observer, nominal_gains, schedule,
        wn_grid=wn_grid, ff_policy=ff_policy,damping_ratio_grid=damping_ratio_grid,
        phase_required_deg=phase_required_deg,gain_required_db=gain_required_db)


def observer_gain(observer, dt):
    F=np.array([[1.,dt],[0.,1.]])
    Q=observer.process_variance*np.array([[dt**4/4,dt**3/2],[dt**3/2,dt**2]])
    P=np.diag([observer.initial_position_variance,observer.initial_velocity_variance])
    K=np.eye(2)
    for _ in range(2000):
        before=P.copy();P=F@P@F.T+Q;M=np.eye(2)
        for index,var in ((0,observer.encoder_variance),(1,observer.gyro_variance)):
            gain=P[:,index]/(P[index,index]+var)
            H=np.zeros(2);H[index]=1
            update=np.eye(2)-np.outer(gain,H)
            P=update@P;M=update@M
        K=np.eye(2)-M
        if np.max(np.abs(P-before))<1e-14:break
    return F,K


def linear_system(a,b,load_slope,gains,observer,dt,delay,feedforward_slope=None):
    """Linearization includes the exact core update order, causal observer and output history."""
    from .sampled_analysis import delayed_plant
    A,Bnew,Bold,whole,extra=delayed_plant(a,b,load_slope,dt,delay)
    F,K=observer_gain(observer,dt);M=np.eye(2)-K
    kp,ki,kpos=gains["kp"],gains["ki"],gains["kpos"]
    ff_slope=load_slope if feedforward_slope is None else feedforward_slope
    output=np.array([ff_slope-kp*kpos,-kp]);error=np.array([-kpos,-1.])
    delay_steps=whole+extra
    require(delay_steps<=200,Reason.ENVELOPE_LIMITED,"delay too large for the 200Hz control structure")
    def step(state):
        plant=state[:2];estimated=state[2:4];integral=state[4];old_error=state[5]
        u=output@estimated+integral
        applied=state[6+whole-1] if whole else u
        old=state[6+whole] if extra else 0.
        nextplant=A@plant+Bnew*applied+Bold*old
        nextestimated=M@F@estimated+K@nextplant
        e=error@estimated
        tail=np.r_[u,state[6:-1]] if delay_steps else np.empty(0)
        return np.r_[nextplant,nextestimated,integral+ki*dt*(e+old_error)/2,e,tail]
    matrix=np.column_stack([step(e) for e in np.eye(6+delay_steps)])
    return matrix,(A,Bnew,Bold,F,K,output,error,whole)


def all_crossing_margins(parts,ki,dt,*,points=2048):
    A,Bnew,Bold,F,K,output,error,m=parts;M=np.eye(2)-K
    def loop(omega):
        z=np.exp(1j*np.asarray(omega)*dt)
        # Batched 2x2 solves, no favourable single crossover is selected.
        eye=np.eye(2)
        drive=Bnew*z[...,None]**(-m)+Bold*z[...,None]**(-m-1)
        plant=np.linalg.solve(z[...,None,None]*eye-A,drive[...,None])[...,0]
        observed=np.linalg.solve(z[...,None,None]*eye-M@F,
                                  (z[...,None]*(plant@K.T))[...,None])[...,0]
        ci=ki*dt/2*(1+1/z)/(z-1)
        return -np.sum((output+ci[...,None]*error)*observed,axis=-1)
    w=np.geomspace(1e-5/dt,(np.pi/dt)*(1-1e-8),points)
    L=loop(w);magnitude=np.abs(L);phase=np.unwrap(np.angle(L))
    gain_crossings=[];phase_margins=[];gain_margins=[]
    for k in np.flatnonzero((magnitude[:-1]-1)*(magnitude[1:]-1)<=0):
        root=brentq(lambda x: float(abs(loop(x))-1),w[k],w[k+1])
        unwrapped=float(np.interp(np.log(root),np.log(w),phase))
        phase_at=float(np.angle(loop(root)))
        phase_at+=round((unwrapped-phase_at)/(2*np.pi))*2*np.pi
        gain_crossings.append(root);phase_margins.append(float(180+np.rad2deg(phase_at)))
    # Every odd pi crossing, including those beyond the first Nyquist winding.
    for multiple in range(int(np.floor(phase.min()/np.pi))-1,int(np.ceil(phase.max()/np.pi))+2):
        if multiple%2==0:continue
        target=multiple*np.pi
        for k in np.flatnonzero((phase[:-1]-target)*(phase[1:]-target)<=0):
            root=brentq(lambda x: float(np.imag(loop(x))),w[k],w[k+1])
            if np.real(loop(root))<0:gain_margins.append(float(-20*np.log10(abs(loop(root)))))
    return {"phase_margin_deg":min(phase_margins) if phase_margins else -180.,
            # A stable nominal loop can have both lower and upper critical gains.
            # Measure distance from unity in either direction, retaining signed crossings.
            "gain_margin_db":min(abs(g) for g in gain_margins) if gain_margins else 300.,
            "signed_gain_crossings_db":gain_margins,
            "gain_crossings_rad_s":gain_crossings,"frequency_samples":points}


def runtime_values(theta,wn,envelope,observer,dt,*,acceleration=None):
    # One candidate schedules a/b with posture by using the smallest nominal a for
    # synthesis and validates every posture. Gains themselves remain fixed per axis.
    anchor=int(np.argmin(theta[:3]));a=float(theta[anchor]);b=float(theta[3+anchor])
    kp=2*wn*a-b
    require(kp>0,Reason.ENVELOPE_LIMITED,"nonpositive Kp is infeasible, never clamped")
    values={"kp":kp,"ki":a*wn**2,"kpos":wn/5,"kaw":wn,
            "current_cap":envelope.current_a,"slew":envelope.slew_a_s,
            "integral_cap":envelope.current_a+float(np.max(np.abs(theta[6:-1]))),
            "velocity_cap":envelope.velocity_rad_s,"dt_min":dt/2,"dt_max":dt*2,
            "intent_threshold":3*np.sqrt(observer.gyro_variance),
            "rest_speed":3*np.sqrt(observer.gyro_variance),"sustained_s":.06,"start_timeout_s":.2}
    if acceleration is not None:
        from .native import ACCELERATION_FIELDS, acceleration_values
        require(set(acceleration)==set(ACCELERATION_FIELDS),Reason.DATA_INVALID,
                "complete explicit acceleration fields required")
        guard,_=acceleration_values(acceleration)
        values.update(guard)
    return {k:float(v) for k,v in values.items()}


def candidate_grid(snapshot,observer,envelope,*,sample_hz=200.,latency_p99_s,observation_band_hz=None):
    require(np.isfinite(latency_p99_s) and latency_p99_s>=0,Reason.DATA_INVALID,
            "unknown delay cannot be treated as zero")
    # Calculating an offline bandwidth grid does not qualify physical stopping.
    fid=snapshot.frequency_band_hz[1]
    if observation_band_hz is not None:fid=min(fid,observation_band_hz)
    maximum=min(2*np.pi*sample_hz/20,2*np.pi*fid/5,
                .35/latency_p99_s if latency_p99_s>0 else np.inf)
    require(maximum>0,Reason.MEASUREMENT_LIMITED,"no identifiable control bandwidth")
    return np.geomspace(maximum/100,maximum,256)


def linear_check(snapshot,observer,gains,dt,theta,posture,position,direction,simulation=None):
    a,b,h=coefficients(snapshot.spec,theta,position,posture,direction)
    span=(snapshot.spec.q_nodes[-1]-snapshot.spec.q_nodes[0])*1e-5
    _,_,left=coefficients(snapshot.spec,theta,position-span,posture,direction)
    _,_,right=coefficients(snapshot.spec,theta,position+span,posture,direction)
    slope=float((right-left)/(2*span))
    _,_,ff_left=coefficients(snapshot.spec,snapshot.theta,position-span,posture,direction)
    _,_,ff_right=coefficients(snapshot.spec,snapshot.theta,position+span,posture,direction)
    ff_slope=float((ff_right-ff_left)/(2*span))
    if simulation is not None and (simulation.encoder_period!=1 or simulation.gyro_period!=1 or
                                  simulation.measurement_delay>0 or simulation.gyro_filter_tau>0):
        from .sampled_analysis import lifted_loop,sampled_margins
        return sampled_margins(lifted_loop(float(a),float(b),slope,ff_slope,gains,observer,
                                          simulation,float(theta[-1])),dt)
    matrix,parts=linear_system(float(a),float(b),slope,gains,observer,dt,float(theta[-1]),ff_slope)
    radius=float(np.max(np.abs(np.linalg.eigvals(matrix))))
    if radius>=1:return {"passed":False,"spectral_radius":radius}
    margins=all_crossing_margins(parts,gains["ki"],dt)
    return {"passed":margins["phase_margin_deg"]>=50 and margins["gain_margin_db"]>=6,
            "spectral_radius":radius,**margins}


def solve(native,snapshot,observer,envelope,simulation,*,latency_p99_s,session_label=None,
          acceleration=None,evaluation_domain=None):
    if session_label is None:
        require(envelope.stop_verified,Reason.ENVELOPE_LIMITED,"stopping envelope not supplied")
    else:
        require(isinstance(session_label,str) and bool(session_label.strip()),Reason.DATA_INVALID,
                "a descriptive provisional session label is required")
    require(snapshot.identity.measurement==observer.measurement_hash and
            snapshot.identity.provenance==observer.provenance==envelope.provenance,
            Reason.INTEGRATION_MISMATCH,"model/observer/envelope measurement or provenance differs")
    require(simulation.encoder_period>=1 and simulation.gyro_period>=1 and
            simulation.encoder_period*simulation.dt<=observer.max_encoder_age_s and
            simulation.gyro_period*simulation.dt<=observer.max_gyro_age_s,
            Reason.MEASUREMENT_LIMITED,"declared sensor cadence exceeds observer freshness domain")
    observation_band=1/(simulation.dt*max(simulation.encoder_period,simulation.gyro_period)*5)
    grid=candidate_grid(snapshot,observer,envelope,latency_p99_s=latency_p99_s,
                        observation_band_hz=observation_band)
    require(abs(simulation.dt-.005)<1e-12,Reason.DATA_INVALID,"frozen controller cadence is 200Hz")
    dt=simulation.dt;spec=snapshot.spec
    if session_label is None:
        require(not snapshot.start_censored.any(),Reason.ENVELOPE_LIMITED,
                "censored startup cells cannot qualify an unmeasured sustained-motion current")
    starts=snapshot.start_intervals[...,1]
    decisions=[]
    # All 256 analytic points are calculated. Expensive robustness evaluation is lazy
    # in descending bandwidth order: lower points cannot outrank a passing higher one.
    viable=[]
    for index,wn in enumerate(grid):
        try:
            values=runtime_values(snapshot.theta,float(wn),envelope,observer,dt,
                                  acceleration=acceleration)
            viable.append((index,float(wn),values))
        except Rejected as exc:decisions.append({"id":index,"reason":exc.detail})
    cases=[];deferred_cases=[]
    # The largest required step permits 0.5deg overshoot; stopping permits
    # another 0.15deg. Reserve both INSIDE the injected physical/model domain.
    # Requested grid cells remain in the report even when their inward test
    # path must start away from an end boundary. No acceptance metric changes.
    reserve=np.deg2rad(.5+.15)+3*np.sqrt(observer.encoder_variance)
    bounded_angles=envelope.angle_min_rad is not None
    if bounded_angles:
        require(envelope.angle_max_rad-envelope.angle_min_rad>2*reserve,
                Reason.ENVELOPE_LIMITED,"no room for the frozen response and stop error bounds")
    if evaluation_domain is None:
        postures=spec.z_nodes;positions=spec.q_nodes
        linear_positions=(np.asarray(spec.q_nodes[:-1])+np.asarray(spec.q_nodes[1:]))/2
        low=envelope.angle_min_rad+reserve if bounded_angles else None
        high=envelope.angle_max_rad-reserve if bounded_angles else None
    else:
        low=float(evaluation_domain["q_min_rad"])+reserve
        high=float(evaluation_domain["q_max_rad"])-reserve
        postures=[float(evaluation_domain["fixed_pitch_rad"])]
        positions=np.unique(np.r_[evaluation_domain["q_min_rad"],
            [q for q in spec.q_nodes if evaluation_domain["q_min_rad"]<=q<=evaluation_domain["q_max_rad"]],
            evaluation_domain["q_max_rad"]])
        linear_positions=positions
        if bounded_angles:
            low=max(low,envelope.angle_min_rad+reserve);high=min(high,envelope.angle_max_rad-reserve)
    for z in postures:
        for position in positions:
            for speed in (3,5,10,-3,-5,-10):
                t,refs,kwargs=shaped_velocity(np.deg2rad(speed),envelope,dt,posture=z)
                if evaluation_domain is not None and np.ptp(refs[:,0])>high-low:
                    deferred_cases.append({"requested_position_rad":float(position),"posture_rad":float(z),
                        "speed_deg_s":speed,"reference_travel_rad":float(np.ptp(refs[:,0])),
                        "reason":"prescribed reference travel exceeds measured local range with existing response/stop reserve"})
                    continue
                offset=float(np.clip(position,low-min(refs[:,0]),high-max(refs[:,0]))) if low is not None else float(position)
                refs[:,0]+=offset
                cases.append((t,refs,kwargs))
            for step in (.5,1,5,-.5,-1,-5):
                if evaluation_domain is not None and abs(np.deg2rad(step))>high-low:
                    deferred_cases.append({"requested_position_rad":float(position),"posture_rad":float(z),
                        "step_deg":step,"reference_travel_rad":float(abs(np.deg2rad(step))),
                        "reason":"prescribed reference travel exceeds measured local range with existing response/stop reserve"})
                    continue
                offset=float(np.clip(position,low-min(0,np.deg2rad(step)),high-max(0,np.deg2rad(step)))) if low is not None else float(position)
                cases.append(shaped_step(np.deg2rad(step),envelope,dt,position=offset,posture=z))
    for index,wn,values in reversed(viable):
        params=parameters(spec,snapshot.theta,observer,values,starts,snapshot.start_censored)
        accepted=True;worst_pm=300.;worst_gm=300.;worst_radius=0.;case_reports=[]
        linear_reports=[];formal_margins_passed=True;performance_passed=not bool(deferred_cases)
        # Nominal model first rejects grossly infeasible points before the supplied stress models.
        models=[snapshot.theta,*snapshot.uncertainty]
        for model_index,theta in enumerate(models):
            for z in postures:
                for d in (-1,1):
                    for position in linear_positions:
                        linear=linear_check(snapshot,observer,values,dt,theta,z,float(position),d,simulation)
                        if session_label is not None and ("phase_margin_deg" not in linear or
                                                         "gain_margin_db" not in linear):
                            linear.update(phase_margin_deg=None,gain_margin_db=None,
                                margin_status="UNDEFINED_AFTER_FAILED_POLE_CHECK")
                        linear_reports.append({"plant_index":model_index,"posture_rad":float(z),
                            "position_rad":float(position),"direction":d,**linear})
                        formal_margins_passed &= bool(linear["passed"])
                        if session_label is None:
                            linear_accepted=linear["passed"]
                        else:
                            # A labelled bounded development probe retains failed model checks
                            # as uncertainty evidence rather than requiring model qualification.
                            linear_accepted=True
                        if not linear_accepted:accepted=False;break
                        worst_pm=(min(worst_pm,linear["phase_margin_deg"])
                                  if worst_pm is not None and linear["phase_margin_deg"] is not None else None)
                        worst_gm=(min(worst_gm,linear["gain_margin_db"])
                                  if worst_gm is not None and linear["gain_margin_db"] is not None else None)
                        worst_radius=max(worst_radius,linear["spectral_radius"])
                    if not accepted:break
                if not accepted:break
            if not accepted:break
        # Formal candidates require the checks; provisional probes retain them and evaluate motion.
        for model_index,theta in enumerate(models if accepted else []):
            plant=parameters(spec,theta,observer,values,starts,snapshot.start_censored)
            for t,refs,kwargs in cases:
                try:
                    trace=native.closed_rollout(params,plant,simulation,refs,(float(refs[0,0]),0.))
                    metrics=motion_metrics(t,refs,trace,**kwargs,
                        gyro_bandwidth_hz=min(snapshot.frequency_band_hz[1],1/(simulation.dt*simulation.gyro_period*5)))
                    metrics.update(plant_index=model_index,
                                   posture_rad=float(refs[0,3]),initial_position_rad=float(refs[0,0]),
                                   reference_peak_rad_s=float(np.max(np.abs(refs[:,1]))),
                                   step_rad=kwargs.get("step_rad"))
                    if evaluation_domain is not None:
                        outside=bool(np.min(trace[:,0])<evaluation_domain["q_min_rad"] or
                                     np.max(trace[:,0])>evaluation_domain["q_max_rad"])
                        metrics["prediction_outside_measured_local_range"]=outside
                        performance_passed &= not outside
                except Rejected as exc:
                    accepted=False;case_reports.append({"reason":str(exc),
                        "initial_position_rad":float(refs[0,0]),"final_position_rad":float(refs[-1,0]),
                        "posture":float(refs[0,3]),"case":kwargs});break
                case_reports.append(metrics)
                performance_passed &= bool(metrics["passed"])
                if not metrics["passed"] and session_label is None:accepted=False;break
            if not accepted:break
        if accepted:
            if session_label is None:
                from .provenance import method_hash
                identity_fields={"plant_hash":snapshot.identity_hash,
                    "model_spec_hash":spec.identity,"observer_hash":observer.hash,
                    "core_build_hash":hashlib.sha256(native.path.read_bytes()).hexdigest(),
                    "method_hash":method_hash(native),"envelope_hash":digest(asdict(envelope))}
            else:
                identity_fields={"session_label":session_label,"identity_kind":"DESCRIPTIVE_SESSION_LABELS",
                    "formal_promotion_eligible":False,
                    "startup_threshold_coverage_qualified":not bool(snapshot.start_censored.any()),
                    "source_labels":asdict(snapshot.identity),
                    "plant_label":session_label+":measured-approximate-plant",
                    "plant_document":{"theta":snapshot.theta.tolist(),
                        "uncertainty":snapshot.uncertainty.tolist(),
                        "frequency_band_hz":list(snapshot.frequency_band_hz),
                        "start_intervals":snapshot.start_intervals.tolist(),
                        "start_censored":snapshot.start_censored.tolist(),
                        "fit_report":snapshot.fit_report},
                    "model_label":session_label+":"+spec.axis+"-model","model_document":asdict(spec),
                    "observer_label":session_label+":measured-observer","observer_document":asdict(observer),
                    "envelope_document":asdict(envelope),"core_label":str(native.path)}
            return {"version":"adr0022.candidate/2",
                    "qualification":"OFFLINE_CANDIDATE_ONLY" if session_label is None else "OFFLINE_PROVISIONAL_ONLY",
                    "provenance":snapshot.identity.provenance,**identity_fields,
                    "simulation":{k:getattr(simulation,k) for k,_ in simulation._fields_},
                    "latency_p99_s":latency_p99_s,
                    "point_id":index,"omega_n_rad_s":wn,"runtime_values":values,
                    "grid_points":len(grid),"uncertainty_models":len(snapshot.uncertainty),
                    "total_plant_models":1+len(snapshot.uncertainty),
                    "worst_phase_margin_deg":worst_pm,"worst_gain_margin_db":worst_gm,
                    "worst_spectral_radius":worst_radius,"case_evaluations":len(case_reports),
                    "formal_margin_requirements_passed":bool(formal_margins_passed),
                    "offline_performance_passed":bool(performance_passed),
                    "linear_case_reports":linear_reports,
                    "case_reports":case_reports,"rejected_points":decisions,
                    "evaluation_domain":evaluation_domain,
                    "evaluation_positions_rad":[float(q) for q in positions],
                    "evaluation_postures_rad":[float(z) for z in postures],
                    "deferred_prescribed_cases":deferred_cases,
                    "lower_points_not_selected":"cannot outrank highest feasible bandwidth",
                    "physical_qualification":"NOT_RUN"}
        decisions.append({"id":index,"reason":"offline stability/margin/performance constraint",
                          "last_case":case_reports[-1] if case_reports else None,
                          "last_linear_case":linear_reports[-1] if linear_reports else None})
    raise Rejected(Reason.ENVELOPE_LIMITED,f"no offline feasible controller in 256 points: {decisions}")
