"""Calibration and normalization algorithms; values come from supplied data only."""
from __future__ import annotations
from dataclasses import asdict, dataclass
import numpy as np
from scipy.optimize import minimize_scalar
from scipy.spatial.transform import Rotation
from scipy.signal import welch, coherence

from .contracts import Reason, Rejected, array, digest, require


@dataclass(frozen=True)
class ObserverSpec:
    encoder_variance: float
    gyro_variance: float
    process_variance: float
    max_encoder_age_s: float
    max_gyro_age_s: float
    initial_position_variance: float
    initial_velocity_variance: float
    encoder_only_verified: bool
    measurement_hash: str
    provenance: str
    version: str = "adr0022.observer/2"

    def __post_init__(self):
        for name in ("encoder_variance", "gyro_variance", "process_variance", "max_encoder_age_s",
                     "max_gyro_age_s", "initial_position_variance", "initial_velocity_variance"):
            v = getattr(self, name)
            require(type(v) in (float, int) and np.isfinite(v) and v > 0,
                    Reason.DATA_INVALID, f"{name} must be measured/derived positive")
        require(type(self.encoder_only_verified) is bool and self.provenance in ("SYNTHETIC", "MEASURED")
                and self.version == "adr0022.observer/2", Reason.DATA_INVALID,
                "invalid observer identity/encoder fallback qualification")

    @property
    def hash(self):
        return digest(asdict(self))


@dataclass(frozen=True)
class EncoderMapping:
    counts_per_motor_turn: int
    motor_turns_per_output_turn: float
    sign: int
    physical_zero_rad: float
    session_offset_rad: float

    def convert(self, counts):
        require(type(self.counts_per_motor_turn) is int and self.counts_per_motor_turn > 1 and
                self.motor_turns_per_output_turn > 0 and self.sign in (-1, 1) and
                np.isfinite([self.motor_turns_per_output_turn, self.physical_zero_rad,
                             self.session_offset_rad]).all(), Reason.DATA_INVALID,
                "verified encoder scale/sign/zero/session mapping required")
        values = np.asarray(counts)
        require(np.isfinite(values).all() and np.all(values == np.floor(values)) and
                np.all((values >= 0) & (values < self.counts_per_motor_turn)), Reason.DATA_INVALID,
                "invalid raw encoder counts")
        phase = np.unwrap(values*2*np.pi/self.counts_per_motor_turn)
        return (self.sign*phase/self.motor_turns_per_output_turn+
                self.physical_zero_rad+self.session_offset_rad)


@dataclass(frozen=True)
class BoundedEncoderMapping:
    """Finite protocol endpoint encoding (e.g. CyberGear type-2 position).

    Endpoint units must be verified against mechPos before a measured calibration
    is qualified. This mapping has no modulo unwrap and never turns a finite
    endpoint transition into a small wraparound motion.
    """
    raw_min: int
    raw_max: int
    shaft_min_rad: float
    shaft_max_rad: float
    motor_turns_per_output_turn: float
    sign: int
    physical_zero_rad: float
    session_offset_rad: float

    def convert(self, counts):
        require(type(self.raw_min) is int and type(self.raw_max) is int and self.raw_min < self.raw_max and
                self.sign in (-1, 1) and type(self.sign) is int and self.motor_turns_per_output_turn > 0 and
                np.isfinite([self.shaft_min_rad, self.shaft_max_rad, self.motor_turns_per_output_turn,
                             self.physical_zero_rad, self.session_offset_rad]).all() and
                self.shaft_max_rad > self.shaft_min_rad, Reason.DATA_INVALID,
                "verified finite encoder endpoints/ratio/sign/session mapping required")
        values = np.asarray(counts)
        require(values.ndim == 1 and values.dtype.kind in 'iuf' and np.isfinite(values).all() and
                np.all(values == np.floor(values)) and np.all((values >= self.raw_min) & (values <= self.raw_max)),
                Reason.DATA_INVALID, "invalid bounded encoder counts")
        angle = self.shaft_min_rad + (values-self.raw_min)*(self.shaft_max_rad-self.shaft_min_rad)/(self.raw_max-self.raw_min)
        return self.sign*angle/self.motor_turns_per_output_turn + self.physical_zero_rad + self.session_offset_rad


def convert_encoder(counts, mapping, *, measured=False):
    fields = dict(mapping)
    encoding = fields.pop("encoding", None)
    require(encoding is not None or not measured, Reason.DATA_INVALID,
            "measured encoder calibration must declare its encoding")
    if encoding in (None, "modulo_count"):
        kind = EncoderMapping
    elif encoding == "bounded_count":
        kind = BoundedEncoderMapping
    else:
        require(False, Reason.INTEGRATION_MISMATCH, "unsupported encoder encoding")
    try:
        return kind(**fields).convert(counts)
    except (TypeError, KeyError) as exc:
        raise Rejected(Reason.DATA_INVALID, "encoder fields do not match the declared encoding") from exc


def verify_stream(time, sequence, generation, valid, *, max_gap_s):
    t = np.asarray(time, dtype=float); seq = np.asarray(sequence); gen = np.asarray(generation)
    mask = np.asarray(valid)
    require(t.ndim == 1 and len(t) >= 3 and np.isfinite(t).all() and
            seq.shape == gen.shape == mask.shape == t.shape and mask.dtype == bool,
            Reason.DATA_INVALID, "stream fields/dimensions invalid")
    require(np.all(np.diff(t) > 0) and np.all(np.diff(seq) > 0) and np.all(seq == np.floor(seq))
            and np.all(seq > 0), Reason.DATA_INVALID, "duplicate/out-of-order time or RX sequence")
    require(np.all(gen == gen[0]) and gen[0] > 0, Reason.DATA_INVALID,
            "generation/tare changed: split and recalibrate this session")
    require(mask.all(), Reason.DATA_INVALID, "invalid samples cannot be promoted by last-value hold")
    require(max_gap_s > 0 and np.max(np.diff(t)) <= max_gap_s+1e-12, Reason.MEASUREMENT_LIMITED,
            "stream gap exceeds calibrated sampling ability")
    return {"samples": len(t), "effective_hz": float((len(t)-1)/(t[-1]-t[0])),
            "largest_gap_s": float(np.max(np.diff(t)))}


def clock_calibration(source_times, host_times, *, maximum_residual_s):
    src, host = np.asarray(source_times, float), np.asarray(host_times, float)
    require(src.ndim == 1 and src.shape == host.shape and len(src) >= 8 and
            np.isfinite(src).all() and np.isfinite(host).all() and
            np.all(np.diff(src)>0) and np.all(np.diff(host)>0), Reason.DATA_INVALID,
            "clock calibration needs matched monotonic event pairs")
    x = np.c_[src-src[0], np.ones(len(src))]
    scale, shift = np.linalg.lstsq(x, host-host[0], rcond=None)[0]
    error = host-(host[0]+x@np.array([scale, shift]))
    require(scale > 0 and np.max(np.abs(error)) <= maximum_residual_s,
            Reason.MEASUREMENT_LIMITED, "clock offset/drift/jitter is unresolved")
    return {"scale": float(scale), "offset_s": float(host[0]+shift-scale*src[0]),
            "residual_p99_s": float(np.quantile(np.abs(error), .99)),
            "source_range_s": [float(src[0]), float(src[-1])]}


def calibrated_times(source_times, mapping):
    source_times = np.asarray(source_times, float)
    lo, hi = mapping["source_range_s"]
    require(np.isfinite(source_times).all() and np.all((source_times>=lo)&(source_times<=hi)),
            Reason.DATA_INVALID, "clock calibration is stale or outside its session")
    return source_times*mapping["scale"]+mapping["offset_s"]


def kinematic_axes(pitch):
    """Yaw/pitch rates to pitch-body angular rate before the IMU mounting rotation."""
    pitch = np.asarray(pitch, float)
    H = np.zeros(pitch.shape+(3, 2))
    H[..., 0, 0] = -np.sin(pitch); H[..., 2, 0] = np.cos(pitch); H[..., 1, 1] = 1.
    return H


def imu_calibration(pitch, rates, gyro, stationary_gyro):
    rates, gyro = np.asarray(rates, float), np.asarray(gyro, float)
    static = np.asarray(stationary_gyro, float)
    require(rates.ndim == 2 and rates.shape[1] == 2 and gyro.shape == (len(rates), 3) and
            static.ndim == 2 and static.shape[1] == 3 and len(static) >= 100 and
            np.isfinite(rates).all() and np.isfinite(gyro).all() and np.isfinite(static).all(),
            Reason.DATA_INVALID, "raw stationary and independent-axis gyro data required")
    bias = np.mean(static, axis=0)
    expected = np.einsum("nij,nj->ni", kinematic_axes(pitch), rates)
    singular = np.linalg.svd(expected, compute_uv=False)
    require(singular[1] > 1e-3*singular[0], Reason.INSUFFICIENT_EXCITATION,
            "at least two independent angular-rate directions required for mounting")
    rotation, _ = Rotation.align_vectors(gyro-bias, expected)
    residual = gyro-bias-rotation.apply(expected)
    noise = np.std(static-bias, axis=0, ddof=1)
    require(np.sqrt(np.mean(residual**2)) <= max(3*np.max(noise), 1e-7),
            Reason.MODEL_INADEQUATE, "gyro coordinate/scale does not fit a rigid mounting rotation")
    return {"body_to_sensor": rotation.as_matrix().tolist(), "gyro_bias_rad_s": bias.tolist(),
            "gyro_noise_rad_s": noise.tolist(), "mount_residual_rms": float(np.sqrt(np.mean(residual**2)))}


def axis_rates(pitch, gyro, calibration):
    R = array(calibration["body_to_sensor"], (3, 3), "mount rotation")
    require(np.allclose(R.T@R, np.eye(3), atol=1e-6) and np.linalg.det(R)>0,
            Reason.DATA_INVALID, "IMU mounting is not a proper rotation")
    corrected = (np.asarray(gyro)-np.asarray(calibration["gyro_bias_rad_s"]))@R
    H = kinematic_axes(pitch)
    rates = np.einsum("nki,nk->ni", H, corrected)
    residual = corrected-np.einsum("nki,ni->nk", H, rates)
    require(np.sqrt(np.mean(residual**2)) <= max(4*max(calibration["gyro_noise_rad_s"]), 1e-7),
            Reason.DATA_INVALID, "gyro axes inconsistent with the calibrated kinematics")
    return rates


def accelerometer_model(rotation_body_to_sensor, origin_acceleration_body, angular_velocity_body,
                        angular_acceleration_body, lever_arm_m, gravity_body, bias):
    """All kinematic vectors/lever arm are expressed in body coordinates; bias is sensor-frame.

    Transform world gravity and origin acceleration into the body at the sample
    orientation before calling. The mounting rotation then maps body to sensor.
    """
    require(lever_arm_m is not None, Reason.MEASUREMENT_LIMITED,
            "unknown lever arm: accelerometer remains vibration evidence, not angular acceleration")
    R = array(rotation_body_to_sensor, (3, 3), "body-to-sensor rotation")
    require(np.allclose(R.T@R,np.eye(3),atol=1e-6) and np.linalg.det(R)>0,
            Reason.DATA_INVALID,"accelerometer mounting must be a proper rotation")
    r = array(lever_arm_m, (3,), "lever arm")
    omega, alpha = np.asarray(angular_velocity_body), np.asarray(angular_acceleration_body)
    acceleration = (np.asarray(origin_acceleration_body)+np.cross(alpha, r)+
                    np.cross(omega, np.cross(omega, r))-np.asarray(gravity_body))
    return acceleration@R.T+np.asarray(bias)


def lever_arm_calibration(rotations, origin_acc, omega, alpha, gravity, accelerometer):
    """Body-frame kinematics and gravity, body-to-sensor mounting, sensor-frame readings."""
    rows, values = [], []
    n=len(omega);gravity=np.asarray(gravity,float)
    if gravity.shape==(3,):gravity=np.tile(gravity,(n,1))
    require(gravity.shape==(n,3) and all(len(x)==n for x in (rotations,origin_acc,alpha,accelerometer)),
            Reason.DATA_INVALID,"all calibration vectors must describe the same samples in the declared frames")
    for R, a0, w, a, g, measured in zip(rotations, origin_acc, omega, alpha, gravity, accelerometer):
        R = np.asarray(R)
        columns = np.column_stack([np.cross(a, e)+np.cross(w, np.cross(w, e)) for e in np.eye(3)])
        rows.append(np.c_[R@columns, np.eye(3)])
        values.extend(measured-R@(a0-g))
    X, y = np.vstack(rows), np.asarray(values)
    require(np.isfinite(X).all() and np.isfinite(y).all(), Reason.DATA_INVALID, "nonfinite accel design")
    scale = np.linalg.norm(X, axis=0)
    require(np.all(scale>1e-12) and np.linalg.cond(X/scale)<1e6,
            Reason.INSUFFICIENT_EXCITATION, "lever arm/bias are not separately identifiable")
    fit = np.linalg.lstsq(X/scale, y, rcond=None)[0]/scale
    return {"lever_arm_m": fit[:3].tolist(), "accel_bias_m_s2": fit[3:].tolist(),
            "residual_rms_m_s2": float(np.sqrt(np.mean((X@fit-y)**2)))}


def observer_likelihood(t, q, v, rq, rv, process):
    state = np.array([q[0], v[0]], float); P = np.diag([rq, rv]); score = 0.
    for k, dt in enumerate(np.diff(t), 1):
        F = np.array([[1., dt], [0., 1.]])
        state = F@state
        P = F@P@F.T+process*np.array([[dt**4/4, dt**3/2], [dt**3/2, dt**2]])
        for index, value, variance in ((0, q[k], rq), (1, v[k], rv)):
            cov = P[:, index].copy(); S = P[index, index]+variance
            error = value-state[index]
            score += np.log(S)+error**2/S
            state += cov/S*error; P -= np.outer(cov, cov)/S
    return float(score)


def estimate_observer(t, q, v, static_q, static_v, *, measurement_hash, provenance,
                      encoder_only_verified=False):
    t, q, v = np.asarray(t), np.asarray(q), np.asarray(v)
    require(t.shape == q.shape == v.shape and t.ndim==1 and len(t)>=100 and np.all(np.diff(t)>0)
            and np.isfinite(t).all() and np.isfinite(q).all() and np.isfinite(v).all(),
            Reason.DATA_INVALID, "observer innovations need valid synchronized records")
    require(len(static_q)>=100 and len(static_v)>=100, Reason.INSUFFICIENT_EXCITATION,
            "stationary noise acquisition too short")
    require(np.isfinite(static_q).all() and np.isfinite(static_v).all(),Reason.DATA_INVALID,
            "stationary observations contain invalid values")
    rq, rv = float(np.var(static_q, ddof=1)), float(np.var(static_v, ddof=1))
    require(rq>0 and rv>0 and np.isfinite([rq, rv]).all(), Reason.MEASUREMENT_LIMITED,
            "zero/no noise characterization is not a usable covariance")
    fit = minimize_scalar(lambda p: observer_likelihood(t,q,v,rq,rv,np.exp(p)),
                          bounds=(np.log(1e-10), np.log(1e4)), method="bounded")
    require(fit.success, Reason.MODEL_INADEQUATE, "innovation likelihood fit failed")
    gap = float(np.max(np.diff(t)))
    return ObserverSpec(rq, rv, float(np.exp(fit.x)), 3*gap, 3*gap, rq, rv,
                        encoder_only_verified, measurement_hash, provenance)


def vibration(t, gyro_rate, acceleration, calibrated_band_hz):
    t = np.asarray(t)
    require(np.all(np.diff(t)>0), Reason.DATA_INVALID, "vibration samples not monotonic")
    fs = (len(t)-1)/(t[-1]-t[0])
    require(np.max(np.abs(np.diff(t)-1/fs)) <= .05/fs, Reason.MEASUREMENT_LIMITED,
            "PSD needs uniform sampling or a declared resampling calibration")
    high = min(20., fs/5, calibrated_band_hz)
    require(high>.5, Reason.MEASUREMENT_LIMITED, "no resolvable vibration band")
    f, psd = welch(gyro_rate, fs=fs, nperseg=min(len(t), int(fs*2)))
    band = (f>=.5)&(f<=high)
    return {"gyro_band_hz": [.5, high], "gyro_rms_rad_s": float(np.sqrt(np.trapezoid(psd[band],f[band]))),
            "frequency_hz": f.tolist(), "psd": psd.tolist(),
            "peak_frequency_hz": float(f[band][np.argmax(psd[band])]),
            "accel_peak_m_s2": float(np.max(np.linalg.norm(acceleration, axis=-1)))}


def calibrated_bandwidth(time, reference, measured, noise_sigma):
    """A measured reference is required; sample cadence alone is not bandwidth."""
    t,reference,measured=(np.asarray(x,float) for x in (time,reference,measured))
    require(t.ndim==1 and len(t)>=256 and t.shape==reference.shape==measured.shape and
            all(np.isfinite(x).all() for x in (t,reference,measured)) and noise_sigma>0,
            Reason.DATA_INVALID,"bandwidth calibration requires finite reference/measurement/noise records")
    dt=float(np.median(np.diff(t)))
    require(dt>0 and np.max(np.abs(np.diff(t)-dt))<=dt*.05,Reason.MEASUREMENT_LIMITED,
            "bandwidth calibration clock/sampling is not uniform")
    fs=1/dt;segment=min(len(t)//4,int(4*fs))
    require(np.std(reference)>3*noise_sigma and np.std(measured)>noise_sigma,
            Reason.MEASUREMENT_LIMITED,"no resolved calibration excitation")
    f,c=coherence(reference,measured,fs=fs,nperseg=segment)
    _,power=welch(reference,fs=fs,nperseg=segment)
    usable=(f>=.5)&(f<=fs/5)&(c>=.9)&(power>9*noise_sigma**2/fs)
    indices=np.flatnonzero(usable)
    require(len(indices)>=2,Reason.MEASUREMENT_LIMITED,"no coherent excited frequency band above noise")
    # Never bridge unexcited/low-coherence holes and advertise the endpoints as a band.
    breaks=np.flatnonzero(np.diff(indices)>1)
    groups=np.split(indices,breaks+1);best=max(groups,key=lambda group:(len(group),-group[0]))
    require(len(best)>=2,Reason.INSUFFICIENT_EXCITATION,"spectral holes require targeted excitation")
    return {"valid_band_hz":[float(f[best[0]]),float(f[best[-1]])],
            "coherence_floor":float(np.min(c[best])),"frequency_hz":f.tolist(),"coherence":c.tolist()}


def calibrate_causal_filter(time, reference_rate, measured_rate, *, noise_sigma, tau_bound_s):
    """Identify a declared first-order causal sensor response from a timed reference."""
    from .identification import observe_gyro
    t,reference,measured=(np.asarray(x,float) for x in (time,reference_rate,measured_rate))
    require(t.ndim==1 and len(t)>=30 and t.shape==reference.shape==measured.shape and
            np.all(np.diff(t)>0) and all(np.isfinite(x).all() for x in (t,reference,measured)) and
            noise_sigma>0 and tau_bound_s>0,Reason.DATA_INVALID,"filter calibration data/bounds invalid")
    fit=minimize_scalar(lambda tau:float(np.mean((observe_gyro(reference,t,tau)-measured)**2)),
                        bounds=(0.,tau_bound_s),method="bounded",options={"xatol":1e-10})
    require(fit.success and np.sqrt(fit.fun)<=3*noise_sigma,Reason.MODEL_INADEQUATE,
            "measurement response is not the declared causal filter/time model")
    return {"filter_tau_s":float(fit.x),"residual_rms_rad_s":float(np.sqrt(fit.fun))}
