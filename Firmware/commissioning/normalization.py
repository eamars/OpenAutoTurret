"""Raw asynchronous data to calibrated SI runs without fabricating RX samples."""
from __future__ import annotations
import numpy as np
from .contracts import Reason, array, digest, require
from .identification import Run
from .measurement import convert_encoder, calibrated_times, verify_stream, axis_rates


def current_si(raw,scale_A_per_count,bias_A,*,source,filter_tau_s):
    require(source in ("Iq","iqf") and np.isfinite([scale_A_per_count,bias_A,filter_tau_s]).all()
            and scale_A_per_count!=0 and filter_tau_s>=0,Reason.DATA_INVALID,
            "current source, unit conversion and causal filter must be declared")
    values=np.asarray(raw,float)
    require(np.isfinite(values).all(),Reason.DATA_INVALID,"invalid raw current")
    return values*scale_A_per_count+bias_A


def normalize_raw(raw,calibration,identity):
    require(raw.get("axis") in ("yaw","pitch"),Reason.DATA_INVALID,"axis must be explicit")
    require(digest({k:v for k,v in calibration.items() if k!="identity_hash"})==calibration.get("identity_hash"),
            Reason.DATA_INVALID,"calibration content hash differs from its declared identity")
    require(raw["identity"]==identity.__dict__ and calibration["identity_hash"]==identity.measurement,
            Reason.INTEGRATION_MISMATCH,"raw/calibration identity differs")
    require(raw.get("mode")=="current" and raw.get("readback_verified") is True and
            raw.get("capture_ready") is True,Reason.DATA_INVALID,"current mode/readback/capture not confirmed")
    data={}
    for name in ("encoder","gyro","current","tx"):
        stream=raw[name]
        t=calibrated_times(stream["sample_time_s"],calibration["clocks"][name])
        verify_stream(t,stream["sequence"],stream["generation"],stream["valid"],
                      max_gap_s=calibration["max_gap_s"][name])
        receive=np.asarray(stream["receive_time_s"],float)
        require(receive.shape==t.shape and np.isfinite(receive).all() and np.all(receive>=t),
                Reason.DATA_INVALID,"RX time precedes calibrated sample time")
        data[name]=(t,stream)
    qt,enc=data["encoder"];gt,gyro=data["gyro"];it,iq=data["current"];ut,tx=data["tx"]
    q=convert_encoder(enc["raw_count"],calibration["encoder"],measured=identity.provenance=="MEASURED")
    require(len(q)==len(qt),Reason.DATA_INVALID,"encoder length differs")
    rates=axis_rates(np.asarray(gyro["pitch_rad"]),np.asarray(gyro["raw_rad_s"]),calibration["imu"])
    axis=0 if raw["axis"]=="yaw" else 1;v=rates[:,axis]
    currents=current_si(iq["raw"],**calibration["current"])
    require(len(currents)==len(it) and np.max(np.abs(currents))<=calibration["current_feedback_cap_A"],
            Reason.HARD_ABORT,"actual current exceeds injected supervision boundary")
    successful=np.asarray(tx["successful_A"],float)
    limited=np.asarray(tx["limited_A"],float);requested=np.asarray(tx["requested_A"],float)
    require(successful.shape==limited.shape==requested.shape==ut.shape and
            np.isfinite(successful).all() and np.isfinite(limited).all() and np.isfinite(requested).all() and
            np.asarray(tx["success"]).dtype==bool and np.asarray(tx["success"]).all() and
            np.allclose(successful,limited,atol=calibration["tx_quantum_A"],rtol=0),
            Reason.DATA_INVALID,"failed or mismatched successful TX must not be fitted as an applied command")
    require(len(set(tx["owner_epoch"]))==1 and tx["owner_epoch"][0]>0,Reason.HARD_ABORT,
            "queued commands cross output owner epochs")
    begin=max(qt[0],gt[0],ut[0]);end=min(qt[-1],gt[-1],ut[-1])
    require(end>begin,Reason.DATA_INVALID,"no common calibrated observation window")
    grid=np.unique(np.r_[qt,gt,ut]);grid=grid[(grid>=begin)&(grid<=end)]
    qnew=np.isin(grid,qt);vnew=np.isin(grid,gt)
    # Interpolation initializes the model; residuals use only qnew/vnew observations.
    position=np.interp(grid,qt,q);velocity=np.interp(grid,gt,v)
    index=np.searchsorted(ut,grid,side="right")-1
    posture=np.interp(grid,np.asarray(raw["posture_time_s"]),np.asarray(raw["other_axis_rad"]))
    direction=np.asarray(tx["direction"])[index]
    return Run(raw["run_id"],identity,grid,position,velocity,successful[index],posture,direction,qnew,vnew,
               calibration["sigma_q_rad"],calibration["sigma_v_rad_s"],calibration["valid_band_hz"][1],
               generation=int(enc["generation"][0]),acquisition_verified=True,
               closed_loop_identification=raw.get("closed_loop_identification",False),
               support_controller_hash=raw.get("support_controller_hash"),
               gyro_filter_tau_s=calibration["gyro_filter_tau_s"]).validate()
