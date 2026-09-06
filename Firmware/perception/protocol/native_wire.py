"""Atomic perception frame: selection authority plus the matching candidate list.

Little endian, version 1. Mirror of tracks/perception_wire.hpp. The embedded v3
list remains a display projection; only the observation authorizes measurement.
"""
import struct
import uuid

from .wire import encode_track_set

HEADER = struct.Struct('<4sHHQQQQQQBBH10f')
MAGIC = b'OTP1'


def uuid_parts(value):
    number = uuid.UUID(str(value)).int if value else 0
    return number >> 64, number & ((1 << 64) - 1)


def encode_perception_frame(track_set, observation):
    observation.validate()
    if (track_set.session_uuid != observation.session_uuid or
            track_set.frame_sequence != observation.frame_sequence or
            track_set.sensor_timestamp_ns != observation.sensor_timestamp_ns):
        raise ValueError('selection and track list must describe the same frame/session')
    if not observation.session_uuid:
        raise ValueError('native frame requires a session UUID')
    bbox = observation.bbox
    anchor = observation.measured_anchor
    return HEADER.pack(
        MAGIC, 1, HEADER.size, *uuid_parts(observation.session_uuid),
        *uuid_parts(observation.track_uuid), observation.selection_generation,
        track_set.track_set_sequence,
        int(observation.target_state), int(observation.measurement_valid),
        int(observation.just_reacquired),
        bbox.x_min, bbox.y_min, bbox.x_max, bbox.y_max, anchor.x, anchor.y,
        observation.detector_score, observation.association_quality,
        observation.identity_confidence, observation.ambiguity,
    ) + encode_track_set(track_set.tracks, frame_sequence=track_set.frame_sequence,
                        sensor_timestamp_ns=track_set.sensor_timestamp_ns,
                        publish_timestamp_ns=track_set.publish_timestamp_ns,
                        width=track_set.stream_width, height=track_set.stream_height)
