"""Export existing shaped yaw references for the normal commissiond session."""
from __future__ import annotations

from copy import deepcopy
import numpy as np

from .metrics import shaped_velocity


def velocity_reference_manifest(config, speed_rad_s, envelope, *, position_rad, dt=.005):
    """Return a manifest containing the existing shaper's absolute q/v/a table.

    Envelope values are supplied planning settings, not measured motor limits.
    The session evaluates this table using its actual control elapsed time.
    """
    if not np.isfinite((speed_rad_s, position_rad, dt)).all() or dt <= 0:
        raise ValueError("finite speed/position and positive reference sample period required")
    if any(value is None or not np.isfinite(value) or value <= 0 for value in
           (envelope.acceleration_rad_s2, envelope.jerk_rad_s3)):
        raise ValueError("declared positive acceleration and planning jerk required by shaped_velocity")
    posture = float(config["other_axis_posture_rad"])
    t, references, timing = shaped_velocity(speed_rad_s, envelope, dt,
                                           position=position_rad, posture=posture)
    samples = [{"time_s": 0., "position_rad": float(position_rad),
                "velocity_rad_s": 0., "acceleration_rad_s2": 0.}]
    samples.extend({"time_s": float(time), "position_rad": float(row[0]),
                    "velocity_rad_s": float(row[1]), "acceleration_rad_s2": float(row[2])}
                   for time, row in zip(t, references))
    result = deepcopy(config)
    result.pop("reference_segments", None)
    result["reference_samples"] = samples
    ramp = (timing["zero_reference_time"] - timing["command_time"] - 2.5) / 2
    result["reference_profile"] = {
        "generator": "Firmware.commissioning.metrics.shaped_velocity",
        "position_coordinate": "absolute manifest yaw coordinate; caller supplies measured initial position",
        "speed_rad_s": float(speed_rad_s), "sample_period_s": float(dt),
        "acceleration_shaping_rad_s2": float(envelope.acceleration_rad_s2),
        "planning_jerk_rad_s3": float(envelope.jerk_rad_s3),
        "physical_kinematic_qualification": False,
        "command_time_s": float(timing["command_time"]),
        "zero_reference_time_s": float(timing["zero_reference_time"]),
        "plateau_begin_s": float(timing["command_time"] + ramp),
        "plateau_end_s": float(timing["zero_reference_time"] - ramp),
        "plateau_duration_s": 2.5, "reference_duration_s": float(t[-1]),
    }
    return result
