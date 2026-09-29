"""Complete parameter contract and pending-value document generator.

The catalog describes *what must be supplied*, and never manufactures station values.
"""
from __future__ import annotations
from dataclasses import asdict
import numpy as np
from .contracts import Reason, digest, require
from .native import CONTROL_FIELDS, parameters, Controller
from .protocol import Field


def catalog(nq=5):
    result={}
    def add(name,meaning,unit,shape,classification,term,signals,method,conditions,bounds,
            dependencies=(),uncertainty="joint whole-run bootstrap",reuse="same H/C and covered O"):
        result[name]={"meaning":meaning,"unit":unit,"coordinate":"output_shaft / calibrated sensor frames",
                      "shape":list(shape),"classification":classification,"model_term":term,
                      "dependencies":list(dependencies),"raw_signals":signals,"estimator":method,
                      "identifiability":conditions,"constraints":bounds,"uncertainty":uncertainty,
                      "applicability_and_reuse":reuse,
                      "unknown_invalid_stale":"PENDING; block dependent calculation, never substitute zero"}
    add("model.a","Current-equivalent inertia, not kg*m^2","A*s^2/rad",(3,),"estimated","dynamics",
        "successful TX, calibrated encoder and gyro at each posture","constrained integral initialization then bounded Huber output error",
        "bidirectional acceleration and independent repeated runs; normalized rank/condition <= 1e6","strictly positive")
    add("model.b","Velocity-dependent resistance","A*s/rad",(3,),"estimated","dynamics",
        "successful TX and independently varying velocity at each posture","same joint output-error fit",
        "velocity not confounded with spatial load; whole-run holdout","nonnegative")
    add("model.h","Total directional position load; no separate unidentifiable bias/gravity columns","A",(2,3,nq),
        "estimated","load","bidirectional motion covering spatial/posture basis","same joint fit with continuous linear basis",
        "both directions and all basis columns observed; no additional free bias column","finite, signed",
        dependencies=("coordinates.q_nodes","coordinates.z_nodes"))
    add("model.delay","End-to-end successful TX to calibrated motion delay","s",(),"estimated","actuation",
        "TX timestamps, encoder/gyro timestamps, dynamic excitation","joint bounded output-error delay fit",
        "known clock map and excited resolvable band; delay must not trade against uncalibrated filtering","0 <= delay <= injected search bound")
    add("model.uncertainty","Correlated a/b/h/delay samples","mixed SI",(128,7+6*nq),"derived","robustness",
        "independent repeated whole training runs","128 seeded, stratified whole-run bootstrap output-error refits",
        ">=3 runs per posture/direction; no data leakage","128 finite physically constrained vectors",
        dependencies=("model.a","model.b","model.h","model.delay"))
    add("start.intervals","Directed total current endpoints: not moving and sustained motion","A",(2,3,nq,2),
        "estimated","hybrid start","monotone bounded current ramp, TX, encoder and gyro","continuous-duration and displacement confirmation",
        "full position/direction/posture coverage; no three-count shortcut","directed ordered endpoints; censored upper is null")
    add("start.censored","Threshold not established before current/time/displacement boundary","bool",(2,3,nq),
        "derived","hybrid start","same startup runs and applied boundaries","censor unsuccessful starts",
        "endpoint coverage and valid TX","boolean; censored never qualifies motion",uncertainty="directed interval/censoring")
    for name,unit,shape,meaning in (
        ("encoder.counts_per_motor_turn","count/turn",(),"Encoder modulus"),
        ("encoder.motor_turns_per_output_turn","1",(),"Gear ratio including internal gearbox"),
        ("encoder.sign","1",(),"Output shaft direction"),
        ("encoder.physical_zero_rad","rad",(),"Recoverable mechanical zero"),
        ("encoder.session_offset_rad","rad",(),"Session mapping; does not erase physical table identity"),
        ("current.scale_A_per_count","A/count",(),"Signed current feedback unit conversion"),
        ("current.bias_A","A",(),"Current measurement bias"),
        ("current.filter_tau_s","s",(),"Iq/iqf filtering, not infinite-bandwidth truth"),
        ("imu.body_to_sensor","1",(3,3),"Proper IMU mounting rotation"),
        ("imu.gyro_bias_rad_s","rad/s",(3,),"Stationary gyro bias"),
        ("imu.accel_bias_m_s2","m/s^2",(3,),"Accelerometer bias"),
        ("imu.lever_arm_m","m",(3,),"IMU lever arm in body frame"),
        ("clock.scale","1",(),"Clock drift scale"),
        ("clock.offset_s","s",(),"Clock/session offset"),
        ("clock.residual_p99_s","s",(),"Unresolved synchronization uncertainty"),
        ("measurement.encoder_quantum_rad","rad",(),"Output quantization"),
        ("measurement.effective_rates_hz","Hz",(3,),"Actual encoder/gyro/current RX rates"),
        ("measurement.valid_band_hz","Hz",(2,),"Joint identifiable valid frequency interval"),
        ("measurement.gyro_filter_tau_s","s",(),"Declared causal gyro observation response")):
        method=("independent-axis Wahba mounting + stationary bias" if name.startswith("imu.") else
                "matched event affine clock least squares" if name.startswith("clock.") else
                "unit mapping, stationary/repeated records and timestamp/response calibration")
        add(name,meaning,unit,shape,"estimated","measurement","raw encoder, current, gyro, accel; sample/RX times and generations",
            method,"excited full-rank calibration with independently checked scale/frame; valid stationary/repeated runs",
            "proper rotation / positive scales and rates / signed +/-1 encoder; null lever arm disallows angular-accel claim",
            uncertainty="calibration residual covariance, time residual or quantization interval",
            reuse="same sensor installation, mapping and valid clock/generation session")
    for name in ("encoder_variance","gyro_variance","process_variance","max_encoder_age_s","max_gyro_age_s",
                 "initial_position_variance","initial_velocity_variance","encoder_only_verified"):
        unit=("bool" if name.endswith("verified") else "s" if name.endswith("_s") else
              "rad^2" if "position" in name or "encoder_variance"==name else
              "rad^2/s^4" if name=="process_variance" else "rad^2/s^2")
        add("observer."+name,"Causal q/omega observer "+name,unit,(),"derived","observer",
            "stationary/repeated encoder/gyro and timestamped innovations","noise covariance; bounded innovation likelihood for process variance",
            "positive measured covariance; fallback requires its own validation","positive finite; fallback boolean",
            dependencies=("measurement.valid_band_hz",),uncertainty="held-out innovations and calibration identity")
    gain_units={"kp":"A*s/rad","ki":"A/rad","kpos":"1/s","kaw":"1/s","current_cap":"A",
                "slew":"A/s","integral_cap":"A","velocity_cap":"rad/s","dt_min":"s","dt_max":"s",
                "intent_threshold":"rad/s","rest_speed":"rad/s","sustained_s":"s","start_timeout_s":"s"}
    for name in CONTROL_FIELDS:
        classification="external_constraint" if name in ("current_cap","slew","velocity_cap") else "derived"
        add("controller."+name,"Runtime core field "+name,gain_units[name],(),classification,"controller",
            "bound model, observer, approved envelope","256 log-spaced offline bandwidth points; shared C++ simulation and all-model checks",
            "identified model and all offline performance/pole/margin gates","Kp>0, Ki>=0, other values positive; never expand approved limits",
            dependencies=("model.a","model.b","model.h","model.uncertainty"),uncertainty="128 joint model robustness outcomes")
    for name,unit in (("current_a","A"),("slew_a_s","A/s"),("velocity_rad_s","rad/s"),
                      ("acceleration_rad_s2","rad/s^2"),("jerk_rad_s3","rad/s^3"),
                      ("angle_min_rad","rad"),("angle_max_rad","rad"),("duration_s","s")):
        add("envelope."+name,"Approved operational limit "+name,unit,(),"external_constraint","actuation/protection",
            "approved capability, stopping, thermal and supply evidence","injected external bound, never optimized upward",
            "verified applicable operating point; unknown blocks stimulus","finite and ordered, positive except signed angle bounds",
            uncertainty="approval domain and measurement margin",reuse="current operating point and capability identity")
    for name,unit,shape in (("q_nodes","rad",(nq,)),("z_nodes","rad",(3,))):
        add("coordinates."+name,"Interpolation knots in recoverable physical coordinates",unit,shape,
            "derived","domain","approved output-shaft range and session mapping","5 equal finite nodes or 8 periodic yaw nodes; 3 posture levels",
            "full-turn permission/mapping for periodic yaw","increasing; no unmeasured cell or extrapolation",
            uncertainty="domain coverage")
    add("method.control_hz","Controller cadence","Hz",(),"constant","method","software timing contract",
        "fixed at 200, actual timing recorded in Stage 2","software constant is not a measured sensor rate","200",
        uncertainty="actual jitter measured separately",reuse="same method/core ABI")
    add("context.hardware_signature","Motor UID/firmware, drive interface, topology, gearing and installation","identity",(),
        "external_constraint","identity","hardware inventory and verified interface contract","canonical content hash",
        "verified exact asset identity","64 lowercase hexadecimal",uncertainty="explicit unknown fields block dependent hardware use")
    add("context.operating_point","Payload, mass/CG when known, friction/preload, other-axis posture, temperature and supply","identity",(),
        "external_constraint","applicability","operator description plus independent response/temperature/supply records",
        "canonical identity; residual checks decide reuse","same label is not same physical state; uncovered changes require update",
        "no extrapolation; unknown optional physical descriptors remain null",uncertainty="covered domain and model residual distribution")
    result['imu.body_to_sensor']['coordinate']='rotation mapping pitch-body vectors into IMU sensor coordinates'
    result['imu.gyro_bias_rad_s']['coordinate']='IMU sensor axes'
    result['imu.accel_bias_m_s2']['coordinate']='IMU sensor axes'
    result['imu.lever_arm_m']['coordinate']='pitch-body axes, from body origin to IMU; transform gravity into this frame per sample'
    for name in ('clock.scale','clock.offset_s','clock.residual_p99_s'):
        result[name]['coordinate']='source sample clock to host monotonic clock; per source/session/generation'
    return result


def pending_document(nq=5):
    return {"version":"adr0022.parameters/2","provenance":"PENDING",
            "parameters":{name:{"value":None,"state":"PENDING","unit":p["unit"],
                                 "shape":p["shape"],"source_hash":None,"uncertainty":None}
                          for name,p in catalog(nq).items()}}


def validate_parameter_document(document,*,physical=False,nq=5):
    require(document.get("version")=="adr0022.parameters/2",Reason.DATA_INVALID,"parameter version differs")
    provenance=document.get("provenance")
    require(provenance in ("SYNTHETIC","MEASURED") and (not physical or provenance=="MEASURED"),
            Reason.DATA_INVALID,"pending/synthetic parameters cannot masquerade as physical measurements")
    declared=catalog(nq);params=document.get("parameters",{})
    require(set(params)==set(declared),Reason.DATA_INVALID,"parameter contract is incomplete or has unknown fields")
    for name,entry in params.items():
        definition=declared[name]
        require(entry.get("state")=="VALID" and entry.get("value") is not None and
                entry.get("unit")==definition["unit"] and entry.get("shape")==definition["shape"] and
                isinstance(entry.get("source_hash"),str) and len(entry["source_hash"])==64 and
                entry.get("uncertainty") is not None,Reason.DATA_INVALID,f"unbound, stale or incompatible parameter {name}")
        values=np.asarray(entry["value"])
        require(list(values.shape)==definition["shape"],Reason.DATA_INVALID,f"wrong dimensions: {name}")
        if values.dtype.kind in "iuf":
            require(np.isfinite(values).all(),Reason.DATA_INVALID,f"nonfinite parameter: {name}")
    return digest(document)


def core_registry(snapshot,observer,values):
    current={"model.theta":snapshot.theta.tolist(),"model.q_nodes":list(snapshot.spec.q_nodes),
             "model.z_nodes":list(snapshot.spec.z_nodes),"start.total":snapshot.start_intervals[...,1].tolist(),
             **{"controller."+k:v for k,v in values.items()},
             **{"observer."+k:v for k,v in asdict(observer).items() if k not in
                ("version","measurement_hash","provenance")}}
    definitions=catalog(len(snapshot.spec.q_nodes));fields={}
    for name,value in current.items():
        info=definitions.get(name,{"unit":"mixed SI","classification":"derived"})
        protected=name in ("controller.current_cap","controller.slew","controller.velocity_cap")
        fields[name]=Field(info["unit"],not protected,info["classification"],np.asarray(value).shape)
    return fields,current


def bind_core_profile(native,snapshot,observer,current):
    fields,expected=core_registry(snapshot,observer,{k:current["controller."+k] for k in CONTROL_FIELDS})
    require(set(current)==set(expected),Reason.DATA_INVALID,"unknown or missing core registry binding")
    require(current["model.q_nodes"]==list(snapshot.spec.q_nodes) and
            current["model.z_nodes"]==list(snapshot.spec.z_nodes),Reason.OPERATING_POINT_CHANGED,
            "coordinate domains require a new compatible model spec")
    obs=asdict(observer)
    for key in obs:
        if "observer."+key in current:obs[key]=current["observer."+key]
    from .measurement import ObserverSpec
    p=parameters(snapshot.spec,current["model.theta"],ObserverSpec(**obs),
                 {k:current["controller."+k] for k in CONTROL_FIELDS},
                 current["start.total"],snapshot.start_censored)
    with Controller(native,p):pass
    return p
