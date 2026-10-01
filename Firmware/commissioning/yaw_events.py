"""Lossless, native-rate yaw journals for offline whole-run identification.

Receipt time is never renamed device sample time. The caller supplies a frozen
gyro projection and a shaft datum; neither is learned from these records.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Sequence
import json

import numpy as np

from .contracts import Reason, require


ENCODER_COUNTS = 8192
ENCODER_QUANTUM_RAD = 2 * np.pi / ENCODER_COUNTS


@dataclass
class YawJournal:
    source_journal: str
    archive_member: str | None
    physical_run_id: str
    configuration_id: str
    calibration_revision: str
    provenance: str
    origin_ns: int
    manifest: dict
    registration: dict
    gyro_calibration: dict
    events: list[dict]
    streams: dict[str, list[dict]]

    def fitter_arrays(self) -> dict:
        """Return separate native observations, including all regimes/history.

        Unknown clocks/filter semantics remain metadata, not a zero-latency
        assertion. Consumers must bound or qualify them before physical use.
        """
        result = {"physical_run_id": self.physical_run_id,
                  "source_journal": self.source_journal,
                  "configuration_id": self.configuration_id,
                  "calibration_revision": self.calibration_revision,
                  "origin_ns": self.origin_ns, "registration": self.registration,
                  "gyro_calibration": self.gyro_calibration}
        fields = {"encoder": ("encoder_t_s", "encoder_q_rad", "registered_rad"),
                  "gyro": ("gyro_t_s", "gyro_rad_s", "projected_rate_rad_s"),
                  "successful_tx": ("tx_t_s", "tx_A", "value_A"),
                  "decoded_current": ("current_t_s", "current_A", "value_A"),
                  "pitch": ("pitch_t_s", "pitch_rad", "value_rad")}
        for channel, (time_name, value_name, field) in fields.items():
            observations = self.streams[channel]
            stamps = np.asarray([e["timestamp"]["time_ns"] for e in observations], dtype=np.int64)
            require(len(stamps) > 0 or channel == "pitch", Reason.DATA_INVALID,
                    f"whole-run journal is missing {channel}")
            require(np.all(np.diff(stamps) > 0), Reason.DATA_INVALID,
                    f"{channel} has duplicate or nonmonotonic native times")
            result[time_name] = (stamps - self.origin_ns) * 1e-9
            result[value_name] = np.asarray([e[field] for e in observations], float)
            result[channel + "_valid"] = np.asarray([e["validity"]["valid"] for e in observations], bool)
            result[channel + "_events"] = observations
        result["encoder_winding_rad"] = np.asarray([e["winding_rad"] for e in self.streams["encoder"]])
        result["encoder_session_relative_rad"] = np.asarray([e["session_relative_rad"] for e in self.streams["encoder"]])
        result["encoder_shaft_phase_rad"] = np.asarray([e["shaft_phase_rad"] for e in self.streams["encoder"]])
        result["gyro_native_values_rad_s"] = np.asarray([e["native_values"] for e in self.streams["gyro"]])
        result["generation"] = {channel: np.asarray([e["generation"] for e in self.streams[channel]], object)
                                for channel in ("encoder", "gyro")}
        result["clock_status"] = {channel: sorted({e["timestamp"]["mapping_status"] for e in observations})
                                  for channel, observations in self.streams.items() if observations}
        result["calibration_support"] = {
            "gyro_projection": "frozen_caller_supplied_fixed_posture",
            "gyro_internal_filter": self.gyro_calibration.get("sensor_internal_filter", "UNKNOWN"),
            "gyro_added_filter_tau_s": self.gyro_calibration.get("added_gyro_filter_tau_s", self.gyro_calibration.get("added_filter_tau_s")),
            "clock_status": result["clock_status"],
            "gyro_physical_sample_timing": "UNKNOWN",
            "gyro_independent_device_clock": False,
            "physical_current_semantics": "UNQUALIFIED_REPORTED_PROTOCOL_CURRENT",
            "registered_shaft_coordinate": self.registration,
            "physical_qualification": False}
        result["all_regimes_retained"] = True
        return result

    def union_observations(self, *, delay_bound_s: float = 0., initial_after_s: float = 0.) -> dict:
        """Union native timestamps with masks; no interpolated measurement values.

        Delay coverage can move initialization forward within the baseline. The
        earlier timeline stays in ``events`` and all successful TX prehistory is
        supplied. Initial latent currents/filter states are seeds, not physics.
        """
        require(np.isfinite([delay_bound_s, initial_after_s]).all() and delay_bound_s >= 0,
                Reason.DATA_INVALID, "finite nonnegative input-delay coverage required")
        arrays = self.fitter_arrays()
        for channel in ("encoder", "gyro", "decoded_current", "successful_tx"):
            require(arrays[channel + "_valid"].all(), Reason.DATA_INVALID,
                    f"invalid {channel} observations retained; qualification cannot silently remove them")
        for channel, generations in arrays["generation"].items():
            require(len(set(generations)) == 1, Reason.DATA_INVALID,
                    f"{channel} generation changed; one initialized run cannot cross a reset")
        minimum = max(initial_after_s, arrays["encoder_t_s"][0], arrays["gyro_t_s"][0],
                      arrays["current_t_s"][0], arrays["tx_t_s"][0] + delay_bound_s)
        union = np.unique(np.concatenate([arrays[key] for key in
                         ("encoder_t_s", "gyro_t_s", "current_t_s", "tx_t_s")]))
        last_observation = max(arrays[key][-1] for key in ("encoder_t_s", "gyro_t_s", "current_t_s"))
        union = union[(union >= minimum) & (union <= last_observation)]
        require(len(union) >= 2, Reason.DATA_INVALID, "no complete observation support after input-delay initialization")
        start = union[0]
        result = {key: arrays[key] for key in ("physical_run_id", "source_journal", "configuration_id",
                                             "calibration_revision", "registration", "calibration_support")}
        result["t"] = union - start
        seeds, supports = [], {}
        for stem, value_key, output_key, mask_key in (("encoder", "encoder_q_rad", "q_obs", "q_new"),
                                                     ("gyro", "gyro_rad_s", "v_obs", "v_new"),
                                                     ("current", "current_A", "current_obs", "current_new")):
            stamps, values = arrays[stem + "_t_s"], arrays[value_key]
            included = (stamps >= start) & (stamps <= union[-1])
            indices = np.searchsorted(union, stamps[included])
            output = np.full(len(union), np.nan)
            mask = np.zeros(len(union), dtype=bool)
            output[indices], mask[indices] = values[included], True
            result[output_key], result[mask_key] = output, mask
            previous = int(np.searchsorted(stamps, start, side="right") - 1)
            seeds.append(float(values[previous]))
            supports[stem] = {"time_s_from_session_begin": float(stamps[previous]),
                              "age_s": float(start - stamps[previous]),
                              "initializer": "last_native_observation_available_at_start"}
        result["tx_t"], result["tx_A"] = arrays["tx_t_s"] - start, arrays["tx_A"]
        result["initial"] = np.array([seeds[0], seeds[1], seeds[2], seeds[1], seeds[2]])
        result["initial_support"] = {"start_s_from_session_begin": float(start),
            "required_pre_window_input_s": float(delay_bound_s), "observations": supports,
            "latent_states": "single-run estimation seeds; effective current and filter state are not measured equality",
            "earlier_raw_events_retained": True, "state_resets_after_initialization": 0}
        result["all_regimes_retained"] = True
        return result


def _timestamp(record, field, *, clock, provenance, mapping_status, dequeue_ns=None):
    return {"time_ns": int(record[field]), "field": field, "clock_identity": clock,
            "timestamp_provenance": provenance, "mapping_status": mapping_status,
            "device_sample_time_ns": record.get("device_sample_ns"),
            "native_sh2_us": record.get("sh2_us"), "receive_ns": record.get("rx_ns", record.get("kernel_monotonic_ns", record.get("receive_ns"))),
            "dequeue_ns": record.get("dequeue_ns", dequeue_ns),
            "clock_uncertainty_ns": record.get("clock_uncertainty_ns")}


def imu_report_clock(record: Mapping) -> dict:
    """Check the inspected producer's epoch lift for this particular record.

    ``sh2_us`` is reconstructed SDK report time from host poll time plus SH-2
    corrections. This arithmetic check supplies no physical latency/drift bound.
    Other producers may use different timestamps; their records stay untouched.
    """
    result = {"mapping_status": "HOST_POLL_REPORT_TIME_EPOCH_LIFT_UNVERIFIED",
              "source_algorithm": "Firmware/tools/imu_bno085.c:sensor",
              "sh2_us": record.get("sh2_us"),
              "sh2_us_description": "SDK reconstructed report time; host poll time plus SH-2 report corrections",
              "independent_device_clock": False, "physical_sample_timing": "UNKNOWN",
              "physical_offset_bound_ns": None, "physical_drift_bound_ppm": None,
              "sensor_filter_latency_bound_ns": None,
              "epoch_lift_verified": False, "epoch_lift_residual_ns": None}
    if not all(type(record.get(name)) is int and record[name] >= 0 for name in ("sample_ns", "rx_ns", "sh2_us")):
        result["verification_detail"] = "complete integer report/sample/receive fields unavailable"
        return result
    receive_us = record["rx_ns"] // 1000
    signed_delta = ((record["sh2_us"] - receive_us + 2**31) % 2**32) - 2**31
    expected = (receive_us + signed_delta) * 1000
    residual = record["sample_ns"] - expected
    result.update(epoch_lift_expected_sample_ns=expected, epoch_lift_residual_ns=residual,
                  epoch_lift_verified=residual == 0,
                  epoch_lift_half_range_us=2**31,
                  epoch_lift_quantization_ns=1000)
    if residual == 0:
        result["mapping_status"] = "HOST_POLL_REPORT_TIME_EPOCH_LIFT_VERIFIED"
    else:
        result["verification_detail"] = "reported sample time differs from inspected producer epoch lift"
    return result


def load_yaw_journal(path: Path | str, *, gyro_calibration: Mapping,
                     encoder_datum_count: int, physical_run_id: str | None = None,
                     configuration_id: str | None = None,
                     calibration_revision: str | None = None,
                     archive_member: str | None = None,
                     clock_provenance: Mapping | None = None) -> YawJournal:
    """Normalize a complete capture, keeping original records and derived parents.

    ``clock_provenance`` supplies already established mapping descriptions keyed
    by encoder/gyro/current/pitch. It changes metadata only; no receive timestamp
    is substituted for an absent device timestamp. A datum is shaft registration,
    never an assertion of world attitude or accumulated cable winding.
    """
    path = Path(path).resolve()
    require(type(encoder_datum_count) is int and 0 <= encoder_datum_count < ENCODER_COUNTS,
            Reason.DATA_INVALID, "explicit encoder datum in 0..8191 required")
    column = np.asarray(gyro_calibration["yaw_column"], float)
    bias = np.asarray(gyro_calibration["baseline_sensor_bias"], float)
    require(column.shape == bias.shape == (3,) and np.isfinite(column).all() and
            np.isfinite(bias).all() and column @ column > 0,
            Reason.DATA_INVALID, "frozen finite gyro column and bias required")
    with path.open(encoding="utf-8") as source:
        originals = [(number, json.loads(line)) for number, line in enumerate(source, 1) if line.strip()]
    require(bool(originals), Reason.DATA_INVALID, "empty journal")
    header = originals[0][1]
    manifest = json.loads(header.get("manifest_yaml", "{}"))
    physical_run_id = physical_run_id or manifest.get("session_label") or str(path)
    configuration_id = configuration_id or manifest.get("configuration_id") or manifest.get("candidate_label") or "UNKNOWN_UNVERSIONED"
    calibration_revision = calibration_revision or gyro_calibration.get("revision") or "CALLER_SUPPLIED_UNVERSIONED"
    origin = next((int(row["time_ns"]) for _, row in originals if row.get("kind") == "session_begin"), None)
    require(origin is not None, Reason.DATA_INVALID, "physical session_begin is required")
    clock_provenance = dict(clock_provenance or {})
    streams = {key: [] for key in ("encoder", "gyro", "requested_current", "limited_current",
                                   "successful_tx", "decoded_current", "pitch")}
    events, wire, encoder_state = [], {}, {}
    calibration_parent = {"kind": "frozen_gyro_calibration", "revision": calibration_revision}
    registration = {"encoder_datum_count": encoder_datum_count, "counts_per_turn": ENCODER_COUNTS,
                    "quantum_rad": ENCODER_QUANTUM_RAD, "coordinate": "registered_output_shaft_rad",
                    "registration_source": "caller_supplied", "world_frame_angle": None,
                    "cross_session_registration_qualified": False,
                    "winding_scope": "encoder displacement within generation; cable winding unknown"}

    def base(number, row, channel, parents=()):
        reference = {"source_journal": str(path), "archive_member": archive_member, "original_line": number}
        return {"schema": "adr0022.yaw-event/1", **reference, "event_ref": {**reference, "channel": channel},
                "channel": channel, "physical_run_id": physical_run_id,
                "configuration_id": configuration_id, "calibration_revision": calibration_revision,
                "provenance": header.get("provenance", manifest.get("provenance", "UNKNOWN")),
                "generation": row.get("generation"), "sequence": row.get("sequence"),
                "status": row.get("status"), "phase": row.get("phase"),
                "validity": {"valid": row.get("valid", True) is not False,
                             "flags": []}, "parents": list(parents), "raw": row}

    def add(number, row, channel, parent):
        event = base(number, row, channel, (parent,))
        events.append(event); streams[channel].append(event)
        return event

    def clock(channel, default):
        declared = clock_provenance.get(channel)
        return (declared.get("status", "DECLARED") if isinstance(declared, Mapping) else str(declared)) if declared else default

    for number, row in originals:
        original = base(number, row, "raw")
        events.append(original)
        parent = original["event_ref"]
        kind = row.get("kind")
        if kind == "can_rx" and row.get("axis") == "yaw":
            wire[int(row["kernel_monotonic_ns"])] = original
        elif kind == "yaw_feedback":
            raw = int(row["encoder_raw"])
            require(0 <= raw < ENCODER_COUNTS, Reason.DATA_INVALID, f"invalid encoder at line {number}")
            packet = wire.get(int(row["kernel_monotonic_ns"]))
            generation = packet["generation"] if packet else None
            generation_key = generation if generation is not None else "UNKNOWN"
            state = encoder_state.setdefault(generation_key, {"first": raw, "last": raw, "delta": 0})
            state["delta"] += (raw - state["last"] + 4096) % ENCODER_COUNTS - 4096
            state["last"] = raw
            first_phase = (state["first"] - encoder_datum_count + 4096) % ENCODER_COUNTS - 4096
            event = add(number, row, "encoder", parent)
            event.update(units={"encoder_raw": "count", "registered_rad": "rad", "winding_rad": "rad", "session_relative_rad": "rad"},
                         generation=generation, encoder_raw=raw,
                         shaft_phase_rad=((raw - encoder_datum_count) % ENCODER_COUNTS) * ENCODER_QUANTUM_RAD,
                         session_relative_rad=state["delta"] * ENCODER_QUANTUM_RAD,
                         winding_rad=state["delta"] * ENCODER_QUANTUM_RAD,
                         registered_rad=(first_phase + state["delta"]) * ENCODER_QUANTUM_RAD,
                         logged_position_rad=row.get("q_relative_rad"),
                         timestamp=_timestamp(row, "kernel_monotonic_ns", clock="host_monotonic",
                                              provenance="kernel_receive_realtime_bracket_mapping",
                                              mapping_status=clock("encoder", "RECEIVE_ONLY_DEVICE_TIME_UNKNOWN")))
            event["parents"].append({"kind": "shaft_registration", **registration})
            if packet:
                event["parents"].append(packet["event_ref"])
                event["sequence"] = packet["sequence"]
                event["timestamp"]["clock_uncertainty_ns"] = packet["raw"].get("clock_uncertainty_ns")
                payload = packet["raw"].get("bytes")
                if payload is not None:
                    payload = bytes(payload)
                    expected = (raw, row.get("speed_rpm"), row["current_raw"])
                    decoded = (int.from_bytes(payload[:2], "big"),
                               int.from_bytes(payload[2:4], "big", signed=True),
                               int.from_bytes(payload[4:6], "big", signed=True))
                    if expected[1] is not None and decoded != expected:
                        event["validity"]["valid"] = False
                        event["validity"]["flags"].append("raw_can_decoder_mismatch")
            else:
                event["validity"]["flags"].append("unpaired_raw_can")
            current = add(number, row, "decoded_current", parent)
            current.update(units={"value_A": "A", "current_raw": "protocol_count"}, value_A=float(row["current_A"]),
                           current_raw=row["current_raw"], timestamp={**event["timestamp"], "mapping_status": clock("current", "RECEIVE_ONLY_DEVICE_TIME_UNKNOWN")},
                           semantics="reported_protocol_current; torque/Iq meaning and filter unqualified", generation=generation)
            current["validity"] = {"valid": event["validity"]["valid"], "flags": list(event["validity"]["flags"])}
            if packet:
                current["parents"].append(packet["event_ref"])
        elif kind == "imu_raw":
            native = json.loads(row["raw_json"])
            if native.get("kind") == "sample" and native.get("sensor") == "gyro":
                event = add(number, native, "gyro", parent)
                values = np.asarray(native["values"], float)
                require(values.shape == (3,) and np.isfinite(values).all(), Reason.DATA_INVALID, f"invalid gyro at line {number}")
                event.update(units={"native_values": "rad/s", "projected_rate_rad_s": "rad/s"},
                             native_values=values.tolist(), projected_rate_rad_s=float((values - bias) @ column / (column @ column)),
                             timestamp=_timestamp(native, "sample_ns", clock="producer_host_monotonic_sample",
                                                  provenance="host_poll_time_plus_SH2_report_corrections",
                                                  mapping_status="HOST_POLL_REPORT_TIME_EPOCH_LIFT_UNVERIFIED", dequeue_ns=row.get("dequeue_ns")))
                event["timestamp"].update(imu_report_clock(native))
                event["timestamp"]["clock_uncertainty_ns"] = None
                event["parents"].append(calibration_parent)
        elif kind == "yaw_current_tx":
            for channel, field in (("requested_current", "requested_A"), ("limited_current", "limited_A"), ("successful_tx", "successful_tx_A")):
                if field not in row or (channel == "successful_tx" and row.get("success") is not True):
                    continue
                event = add(number, row, channel, parent)
                stamp = "kernel_accepted_ns" if channel == "successful_tx" else "begin_ns"
                event.update(units={"value_A": "A"}, value_A=float(row[field]),
                             success=row.get("success"), timestamp=_timestamp(row, stamp, clock="host_monotonic",
                             provenance="kernel_socket_acceptance" if channel == "successful_tx" else "command_request_begin",
                             mapping_status="HOST_CLOCK;HARDWARE_APPLICATION_TIME_UNKNOWN"))
        elif kind == "register_read" and row.get("axis") == "pitch" and row.get("index") == 0x7019:
            event = add(number, row, "pitch", parent)
            event.update(units={"value_rad": "rad"}, value_rad=float(row["value"]),
                         timestamp=_timestamp(row, "receive_ns", clock="host_monotonic", provenance="register_receive",
                                              mapping_status=clock("pitch", "RECEIVE_ONLY_DEVICE_TIME_UNKNOWN")))
    for channel, observations in streams.items():
        declaration = clock_provenance.get("current" if channel == "decoded_current" else channel)
        for event in observations:
            event["timestamp"]["mapping_declaration"] = dict(declaration) if isinstance(declaration, Mapping) else declaration
            if not np.isfinite(float(event.get("value_A", event.get("value_rad", 0.)))):
                event["validity"]["valid"] = False
                event["validity"]["flags"].append("nonfinite_value")
    return YawJournal(str(path), archive_member, str(physical_run_id), str(configuration_id), str(calibration_revision),
                      header.get("provenance", manifest.get("provenance", "UNKNOWN")), origin,
                      manifest, registration, dict(gyro_calibration), events, streams)


def check_physical_run_split(train: Sequence[YawJournal], selection: Sequence[YawJournal],
                             validation: Sequence[YawJournal]) -> dict:
    """Reject physical-journal leakage even when a caller relabels windows/runs."""
    groups = {"train": list(train), "selection": list(selection), "validation": list(validation)}
    require(bool(groups["train"]) and bool(groups["validation"]), Reason.DATA_INVALID,
            "whole physical training and validation journals required")
    seen_ids, seen_sources, seen_captures, seen_members = {}, {}, {}, {}
    for group, journals in groups.items():
        for journal in journals:
            source = str(Path(journal.source_journal).resolve()).casefold()
            capture = (journal.manifest.get("session_label"), journal.origin_ns)
            identities = [(journal.physical_run_id, seen_ids), (source, seen_sources)]
            if capture[0]:
                identities.append((capture, seen_captures))
            if journal.archive_member:
                identities.append((journal.archive_member, seen_members))
            for value, seen in identities:
                require(value not in seen, Reason.DATA_INVALID,
                        f"physical journal overlap: {value} in {seen.get(value)} and {group}")
                seen[value] = group
    return {"split_unit": "complete_physical_journal", "prospective_validation": False,
            "groups": {group: [journal.physical_run_id for journal in journals] for group, journals in groups.items()},
            "interpretation": "Historical blocked validation; unseen prospective runs remain necessary."}
