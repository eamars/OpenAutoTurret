#!/usr/bin/env python3
"""``visiond`` — the perception subsystem's entry point (§43's ``visiond --replay``).

The CLI is where the document's rules become behaviour that cannot be argued with:

* **Nothing starts before the configuration is validated.** ``--production`` runs §50's
  validation, which refuses the shipped file's ``COMMISSION`` thresholds. Without the flag the
  daemon still runs, but it prints the unresolved items as work rather than pretending they were
  never there (§50's objection to the retired code was exactly that: demo thresholds in
  production clothing).
* **The station is described before it is used.** Capture mode writes the §9.1 environment
  manifest — OS, Python, kernel, the six IMX500 packages, the model's hash — before the first
  inference, so an upgrade that breaks something can be *located* rather than guessed at.
* **A model is admitted by measurement, not by filename.** ``--probe-model`` runs Appendix D and
  exits; capture mode refuses to infer until the probe and the manifest agree (§9.3).
* **Replay is a first-class mode, not a debug hack.** ``--replay`` drives the same pipeline from
  a recording (§43 Level B), prints §45's metrics, and with ``--gates`` exits non-zero when §46
  is not met — so the same command line works as a CI step.

Selection is not taken from the command line in capture mode (an operator chooses targets through
the UI, §28) except as an explicit ``--select-uuid`` for testing; in replay it is the mechanism
for re-asking §46's "requesting UUID A selects A or rejects, never B".

Nothing here imports ``picamera2``, ``CAN``, or the retired ``vision``/``control`` modules: the
camera path is reached through :mod:`perception.camera`, which is import-guarded, and the
subsystem's output is two JSON documents (§38) that controld reads.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Any, Dict, List, Optional

from .camera import CameraOwner, open_picamera2
from .config import VisionConfig
from .errors import ConfigError, ConfigPlaceholderError, ModelRejected, PerceptionError
from .events import EventLog
from .model import (OFFLINE_ADAPTERS, EnvironmentManifest, build_adapter, manifest_for,
                    probe_model,
                    resolve_artifact)
from .model.adapter import MockAdapter
from .pipeline import JsonPublisher, PerceptionPipeline, PreviewTap
from .protocol.jsonio import atomic_write_text, dumps as json_dumps
from .protocol.selected_target import TargetState
from .protocol.wire import (SocketPublisher, encode_track_set,
                            measurement_from_selection)
import json as _json
import urllib.request as _urllib
from .replay import (Recorder, ReplaySource, compare_ground_truth, compare_runs,
                     engineering_gates, run_level_b)
from .selection.protocol import SelectTargetRequest

#: Which selected identities have already been handed to the controller (per-process). Re-issuing
#: `select_target` every frame would spam the ack channel; the selection persists until the
#: identity is retired and re-selected, so one issuance per identity is enough.
_issued_selections: set = set()


def _selected_display_index(track_set: Any, track_uuid: str) -> int:
    """The display index controld's `select_target` wants, for a selected track uuid."""
    if track_set is None or not track_uuid:
        return 0
    for track in getattr(track_set, "tracks", ()):
        if getattr(track, "track_uuid", "") == track_uuid:
            return int(getattr(track, "display_index", 0) or 0)
    return 0
from .tracking.diagnostics import AssociationDiagnostics

DEFAULT_CONFIG = "configs/perception_v1.json"

EXIT_OK = 0
EXIT_CONFIG = 2
EXIT_MODEL = 3
EXIT_GATES = 4
EXIT_REPLAY = 5


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="visiond",
        description="OpenAutoTurret perception and target-selection daemon (Vision 1.0).")
    parser.add_argument("--config", default=DEFAULT_CONFIG,
                        help=f"configuration file (default: the shipped {DEFAULT_CONFIG})")
    parser.add_argument("--profile", default="",
                        help="model profile to run (default: the config's active profile)")
    parser.add_argument("--production", action="store_true",
                        help="enforce §50: refuse to start with COMMISSION thresholds")
    parser.add_argument("--replay", default="", metavar="RECORDING",
                        help="Level-B replay of a recorded directory (§43) instead of the camera")
    parser.add_argument("--record-dataset", default="", metavar="DIR",
                        help="record this run (§43: manifest, detections, camera metadata)")
    parser.add_argument("--record-events", default="", metavar="PATH",
                        help="persist critical events (§42) to this JSONL file")
    parser.add_argument("--record-images", action="store_true",
                        help="also record encoded frames (Level A, large; default off)")
    parser.add_argument("--disable-preview", action="store_true",
                        help="do not offer frames to the preview tap (§39)")
    parser.add_argument("--preview-fps", type=float, default=0.0,
                        help="override preview rate (default: the config's)")
    parser.add_argument("--publish-dir", default="", metavar="DIR",
                        help="write track_set.json / selected_target.json here (§38)")
    parser.add_argument("--select-controller", default="", metavar="URL",
                        help="controld's HTTP API (e.g. http://127.0.0.1:8080): when the subsystem "
                             "auto-selects a confirmed person, issue `select_target <display_index>` "
                             "once so the turret leaves LOST_HOLD and follows it")
    parser.add_argument("--publish-socket", default="", metavar="PATH",
                        help="publish the selected target to controld's SOCK_SEQPACKET socket "
                             "each frame as a 58-byte TargetMeasurement (§6.2). This is the "
                             "live-tracking path; without it visiond only records/publishes JSON")
    parser.add_argument("--session-uuid", default="",
                        help="stamp the published TrackSets with this session id (§33)")
    parser.add_argument("--probe-model", default="", metavar="RPK",
                        help="run Appendix D's compatibility probe on a model file and exit")
    parser.add_argument("--emit-manifest", default="", metavar="PATH",
                        help="with --probe-model: write a draft manifest drafted from the runtime "
                             "facts (§9.3). It is a draft — _needs names what a human must still "
                             "decide, and §50 refuses it until they have")
    parser.add_argument("--environment-manifest", default="", metavar="PATH",
                        help="write §9.1's environment manifest here")
    parser.add_argument("--select-uuid", default="",
                        help="issue one selection request by identity UUID (capture mode)")
    parser.add_argument("--select-label", default="", metavar="LABEL",
                        help='replay: select the identity shown as e.g. "Person #2". UUIDs are '
                             f'per-run (§17), so a recorded UUID names nothing in a replay')
    parser.add_argument("--clear-selection", action="store_true",
                        help="start with no selection even if §28's auto policy would pick one")
    parser.add_argument("--max-frames", type=int, default=0,
                        help="stop after N frames (0 = run until interrupted)")
    parser.add_argument("--diagnostics", default="", metavar="PATH",
                        help="dump §41's association ring here on exit/fault")
    parser.add_argument("--report", default="", metavar="PATH",
                        help="write the run report as JSON here")
    parser.add_argument("--gates", action="store_true",
                        help="evaluate §46's engineering gates and exit non-zero if unmet")
    parser.add_argument("--dedup-iou", type=float, default=None, metavar="IOU",
                        help="override §16.1's NMS IoU for this run (offline experiments only)")
    parser.add_argument("--dedup-containment", type=float, default=None, metavar="RATIO",
                        help="override §16.2's containment ratio for this run")
    parser.add_argument("--dedup-center-distance", type=float, default=None,
                        metavar="NORM", help="override §16.2's centre-distance for this run")
    parser.add_argument("--quiet", action="store_true", help="suppress the progress line")
    return parser


def load_config(args: argparse.Namespace) -> VisionConfig:
    """Read the configuration and refuse what cannot run, before anything is opened.

    Everything the operator can get wrong on the command line is answered here, in
    :data:`EXIT_CONFIG` terms: a missing file, a typo in a profile name, a threshold override
    paired with ``--production``. Downstream code is allowed to assume the profile exists.
    """
    try:
        path = resolve_artifact(args.config) or args.config
        config = VisionConfig.from_file(path)
    except (OSError, PerceptionError) as exc:
        raise ConfigError(f"cannot read configuration {args.config!r}: {exc}") from exc
    if args.profile:
        if args.profile not in config.models:
            raise ConfigError(
                f"profile {args.profile!r} is not in {path}. Available profiles: "
                + ", ".join(sorted(config.models))
                + ". A profile name is not a free-form label: it selects the model, its "
                  "manifest and its thresholds (§9.3).")
        config.profile = args.profile
    if args.emit_manifest and not args.probe_model:
        raise ConfigError(
            "--emit-manifest only means something with --probe-model: a manifest drafted "
            "without probing the artefact would be a file of placeholders with a timestamp on "
            "it (§9.3)")
    overrides = {"nms_iou": args.dedup_iou, "containment_ratio": args.dedup_containment,
                 "center_distance_norm": args.dedup_center_distance}
    requested = {key: value for key, value in overrides.items() if value is not None}
    if requested:
        if args.production:
            raise ConfigError(
                "--dedup-* overrides are not accepted with --production (§50). A production "
                "run's thresholds come from the commissioned configuration; if these numbers "
                "are right, put them in the file and commit them.")
        for key, value in requested.items():
            setattr(config.dedup, key, float(value))
        print("visiond: §16 dedup thresholds overridden for this run "
              + ", ".join(f"{key}={value}" for key, value in sorted(requested.items()))
              + " — an experiment, not a commissioned configuration", file=sys.stderr)

    for problem in config.validate(production=bool(args.production)):
        print(f"visiond: configuration note: {problem}", file=sys.stderr)
    if args.clear_selection and hasattr(config.selection, "policy"):
        # The CLI can only *narrow* a policy: running with auto-select forced on would let a
        # command-line flag decide what §28 reserves for the operator.
        from .config import SelectionPolicy
        if config.selection.policy is not SelectionPolicy.EXPLICIT_ONLY:
            config.selection.policy = SelectionPolicy.EXPLICIT_ONLY
            print("visiond: --clear-selection forced policy to explicit_only (§28)",
                  file=sys.stderr)
    return config


def _event_log(args: argparse.Namespace, *, station: bool) -> EventLog:
    return EventLog(persist_path=args.record_events or "",
                    persist_critical=station)


def _environment_manifest(args: argparse.Namespace, config: VisionConfig,
                          manifest: Any) -> EnvironmentManifest:
    """§9.1: record the whole compatibility set, including the parts that are missing."""
    label_names = [str(name) for name in getattr(manifest, "label_map", lambda: None)().names] \
        if hasattr(manifest, "label_map") else []
    record = EnvironmentManifest.collect(
        model_path=getattr(manifest, "path", "") or "",
        model_id=getattr(manifest, "model_id", "") or "",
        task=getattr(manifest, "task", "") or "",
        labels=label_names or None)
    missing = record.missing_required_packages()
    if missing:
        record.collector_notes.append(
            "packages not installed: " + ", ".join(missing) +
            " — the IMX500 path cannot be expected to work on this interpreter (§9.1)")
    if args.environment_manifest:
        written = record.write(args.environment_manifest)
        print(f"visiond: wrote §9.1 environment manifest to {written}", file=sys.stderr)
    return record


def _print_environment(record: EnvironmentManifest) -> None:
    print(f"visiond: station {record.hostname} ({record.machine}) python {record.python} "
          f"picamera2={record.packages.get('python:picamera2') or record.packages.get('python3-picamera2')} "
          f"imx500-models={record.packages.get('imx500-models')}", file=sys.stderr)


def run_probe(model_path: str, config: VisionConfig, profile: str,
              emit_manifest: str = "") -> int:
    """Appendix D, as a command. Prints findings and exits with §9.2's verdict."""
    model = config.model_for(profile) if profile else config.active_model
    item = manifest_for(config, profile or config.profile)
    result = probe_model(model_path, model_id=item.model_id or model.model_id)
    print(json.dumps(result.to_dict(), indent=2))
    if emit_manifest:
        if not result.probed:
            # A draft from a model that never opened would be all COMMISSION and a filename,
            # and it would look like a manifest to the next person. Say no, and say why.
            print(f"visiond: refusing to draft a manifest for {model_path!r}: the model never "
                  f"opened, so there is nothing measured to draft from", file=sys.stderr)
            return EXIT_MODEL
        document = result.to_manifest_document(model_id=item.model_id or model.model_id)
        atomic_write_text(emit_manifest, json_dumps(document, indent=2) + "\n")
        print(f"visiond: drafted {emit_manifest} from runtime facts. {len(document['_needs'])} "
              f"field(s) still need a human (_needs); §50 will refuse it until they are filled",
              file=sys.stderr)
    if not result.usable:
        print("visiond: model NOT admitted (§9.2)", file=sys.stderr)
        return EXIT_MODEL
    try:
        from .model import admit
        warnings = admit(item.with_runtime(path=model_path), result)
    except ModelRejected as exc:
        print(str(exc), file=sys.stderr)
        return EXIT_MODEL
    for warning in warnings:
        print(f"visiond: {warning}", file=sys.stderr)
    print("visiond: model admitted (§9.2/§9.3)", file=sys.stderr)
    return EXIT_OK


def run_replay(args: argparse.Namespace, config: VisionConfig) -> int:
    """§43's Level B: recorded detections through filter → dedup → tracker → selection."""
    try:
        source = ReplaySource(args.replay, expected_model_id=config.active_model.model_id)
    except PerceptionError as exc:
        print(f"visiond: {exc}", file=sys.stderr)
        return EXIT_REPLAY
    for note in source.summary.notes:
        print(f"visiond: recording: {note}", file=sys.stderr)
    for mismatch in source.summary.config_mismatches:
        print(f"visiond: CONFIGURATION DIFFERS FROM RECORDING: {mismatch}", file=sys.stderr)

    events = _event_log(args, station=False)
    request = None
    if args.select_uuid:
        request = SelectTargetRequest(request_id="visiond-replay",
                                      track_uuid=args.select_uuid,
                                      track_set_sequence_seen_by_ui=0)
    recorder = None
    if args.record_dataset:
        recorder = Recorder(args.record_dataset, config=config,
                            notes=["re-derived from a recording; not a station run"])
    run = run_level_b(source.detection_sets(), config, event_log=events,
                      select_request=request, select_label=args.select_label,
                      frame_limit=args.max_frames, record=recorder)
    if recorder is not None:
        recorder.close()
    summary = source.finish()
    for note in summary.notes:
        print(f"visiond: recording: {note}", file=sys.stderr)
    if summary.malformed_lines:
        print(f"visiond: {summary.malformed_lines} malformed lines in the recording",
              file=sys.stderr)

    report = run.report
    failures = engineering_gates(report)
    payload: Dict[str, Any] = {"mode": "replay", "report": report.to_dict(),
                               "canonical_frames": len(run.canonical),
                               "source": summary.to_dict(),
                               "events": events.counts()}
    truth = source.ground_truth()
    if truth:
        payload["ground_truth"] = compare_ground_truth(run.track_sets, run.observations, truth)
        print(json.dumps(payload["ground_truth"], indent=2))
    second = run_level_b(ReplaySource(args.replay).detection_sets(), config,
                         select_request=request, select_label=args.select_label,
                         frame_limit=args.max_frames)
    diff = compare_runs(run.canonical, second.canonical)
    payload["determinism"] = diff.to_dict()
    payload["gates"] = failures
    # One JSON document on stdout, at the end. Printing the report and then the ground truth and
    # then the determinism result makes the stream unparseable, which is the difference between
    # a tool a CI job can read and a tool a person has to squint at.
    if not args.quiet:
        print(json.dumps(payload, indent=2))
    print(f"visiond: replay determinism: "
          f"{'identical' if diff.identical else 'DIFFERENT'}", file=sys.stderr)
    for difference in diff.differences[:3]:
        print(f"  {difference['path']}: {difference['reference']!r} vs "
              f"{difference['candidate']!r}", file=sys.stderr)
    _write_report(args, payload)      # written last: the report is the artefact of this run
    if args.gates and failures:
        for failure in failures:
            print(f"visiond: GATE FAILED: {failure}", file=sys.stderr)
        return EXIT_GATES
    return EXIT_OK


def run_capture(args: argparse.Namespace, config: VisionConfig) -> int:
    """The station path: one camera owner, one adapter, one publish target."""
    manifest = manifest_for(config, config.profile)
    try:
        manifest.validate()
    except PerceptionError as exc:
        print(f"visiond: {exc}", file=sys.stderr)
        return EXIT_CONFIG
    offline_adapter = (config.active_model.adapter or "") in OFFLINE_ADAPTERS
    gaps = manifest.commissioning_gaps(requires_artifact=not offline_adapter)
    if gaps and args.production:
        print("visiond: manifest is not commissioned (§50):\n  - " + "\n  - ".join(gaps),
              file=sys.stderr)
        return EXIT_CONFIG
    for gap in gaps:
        print(f"visiond: commissioning gap: {gap}", file=sys.stderr)
    if offline_adapter:
        # Announced even under --quiet: it says a check was skipped, which is not progress noise.
        print("visiond: offline profile — §50's artifact questions are waived (there is no "
              f"{config.active_model.adapter} file to hash); the score and §16 thresholds are "
              "not", file=sys.stderr)

    if config.dedup.nms_iou is None or config.dedup.containment_ratio is None \
            or config.dedup.center_distance_norm is None:
        # A stage that will decline has to be said out loud at start-up. The pipeline counts it
        # every frame, but nobody scrolls through 3000 frames to discover that dedup was off;
        # and §25's resolver declines on the same numbers, so duplicates will not be merged
        # either (§50's placeholders are load-bearing, and this is what that costs).
        print("visiond: §16 dedup is OFF (its thresholds are COMMISSION): class-aware NMS, "
              "containment suppression and §25's duplicate resolver will all decline, and the "
              "run will count that per frame", file=sys.stderr)

    environment = _environment_manifest(args, config, manifest)
    _print_environment(environment)

    events = _event_log(args, station=True)
    preview = PreviewTap(enabled=not args.disable_preview,
                         fps=(args.preview_fps or config.preview.fps),
                         latest_queue_depth=config.preview.latest_queue_depth)
    diagnostics = AssociationDiagnostics(
        capacity=int(config.tracking.diagnostics_capacity),
        # Asking for a dump on disk is an instruction to fill it: `--diagnostics` overrides the
        # config's disabled switch rather than writing an empty ring and calling it done (§41).
        enabled=bool(config.tracking.diagnostics_enabled or bool(args.diagnostics)))
    recorder = None
    if args.record_dataset:
        recorder = Recorder(args.record_dataset, config=config, model_manifest=manifest,
                            environment=environment.to_dict(),
                            record_detections=config.record.record_detections,
                            record_images=bool(args.record_images or
                                              config.record.record_images),
                            flush_every=max(1, int(config.record.flush_every)))
        recorder.open()
        print(f"visiond: recording dataset to {args.record_dataset}", file=sys.stderr)

    try:
        adapter = build_adapter(config, manifest=manifest)
    except ConfigError as exc:
        print(f"visiond: {exc}", file=sys.stderr)
        return EXIT_CONFIG

    publisher = JsonPublisher(args.publish_dir) if args.publish_dir else None
    wire_publisher = SocketPublisher(args.publish_socket) if args.publish_socket else None
    pipeline = PerceptionPipeline(config, adapter=adapter, event_log=events,
                                  diagnostics=diagnostics, recorder=recorder,
                                  publisher=publisher, preview=preview,
                                  session_uuid=args.session_uuid or "")
    camera: Optional[CameraOwner] = None
    try:
        if isinstance(adapter, MockAdapter):
            # No camera on a mock profile: synthetic frames, so the daemon's own wiring can be
            # exercised on a machine with no sensor attached (§55.18's offline acceptance run).
            return _run_synthetic(args, pipeline, adapter, config, wire_publisher=wire_publisher)
        requested_stream = ((int(config.camera.width), int(config.camera.height))
                            if config.camera.width and config.camera.height else None)
        imx500, picam2, info = open_picamera2(manifest.path, stream_size=requested_stream)
        adapter.open()
        stream = (int(info["stream_size"][0]), int(info["stream_size"][1]))
        adapter.configure_stream(*stream)
        camera = CameraOwner(picam2, stream_size=stream, preview=preview, events=events)
        pipeline.start()
        return _run_camera(args, pipeline, adapter, camera, info,
                        wire_publisher=wire_publisher)
    except ModelRejected as exc:
        print(f"visiond: model refused (§9.3):\n{exc}", file=sys.stderr)
        return EXIT_MODEL
    except ConfigError as exc:
        print(f"visiond: {exc}", file=sys.stderr)
        return EXIT_CONFIG
    finally:
        report = pipeline.report()
        if camera is not None:
            report["camera"] = camera.stats.to_dict()
            camera.close()
        pipeline.stop()
        if args.diagnostics:
            _dump_diagnostics(args.diagnostics, diagnostics)
        payload = {"mode": "capture", **report}
        _write_report(args, payload)
        if not args.quiet:
            print(json.dumps(payload, indent=2))
        events.close()


def _run_camera(args: argparse.Namespace, pipeline: PerceptionPipeline, adapter: Any,
                camera: CameraOwner, info: Dict[str, Any],
                wire_publisher: Optional[SocketPublisher] = None) -> int:
    print(f"visiond: camera {info['camera_num']} stream "
          f"{info['stream_size'][0]}x{info['stream_size'][1]} model {info['task']} "
          f"@ {info['inference_rate_hz']} Hz", file=sys.stderr)
    camera.start()
    delivered = 0
    for frame in camera.frames(max_frames=args.max_frames):
        outcome = pipeline.process_frame(frame.image, frame.metadata,
                                         frame_sequence=frame.frame_sequence,
                                         sensor_timestamp_ns=frame.sensor_timestamp_ns,
                                         capture_started_ns=frame.metadata_receive_ns)
        delivered += 1
        if not args.quiet and delivered % 30 == 0:
            tracks = len(outcome.track_set.tracks) if outcome.track_set else 0
            print(f"visiond: frame {frame.frame_sequence} tracks {tracks} "
                  f"target {outcome.observation.target_state.name if outcome.observation else 'NO_TARGET'}",
                  file=sys.stderr)
        if not outcome.published and not args.quiet:
            print(f"visiond: frame {frame.frame_sequence} failed in {outcome.stage}: "
                  f"{outcome.failure}", file=sys.stderr)
        if wire_publisher is not None:
            # The live-tracking handoff. TWO messages, because they do two different jobs:
            #   * the v3 TrackSet populates controld's SelectionManager (tracks + display indices),
            #     which is what `select_target` and the AutoTrack controller actually read;
            #   * the v1 measurement keeps the estimator fed and is the legacy path.
            # The v1 message alone never sets controld's has_selection — the exact reason the
            # camera→motor loop never closed before.
            stream_w, stream_h = int(info["stream_size"][0]), int(info["stream_size"][1])
            if outcome.track_set is not None:
                ts = encode_track_set(
                    outcome.track_set.tracks, frame_sequence=frame.frame_sequence,
                    sensor_timestamp_ns=frame.sensor_timestamp_ns,
                    publish_timestamp_ns=frame.metadata_receive_ns or frame.sensor_timestamp_ns,
                    width=stream_w, height=stream_h)
                if ts:
                    wire_publisher.send(ts)
            sent = wire_publisher.send(measurement_from_selection(
                outcome.observation, frame_sequence=frame.frame_sequence,
                sensor_timestamp_ns=frame.sensor_timestamp_ns,
                stream_size=(stream_w, stream_h)).encode())
            if not sent and not args.quiet:
                print("visiond: publish-socket send failed (controld down?)", file=sys.stderr)
            # When the subsystem auto-selects a confirmed person, tell controld to track it. A
            # v3 publisher is supposed to hand the target to the operator; for the autonomous
            # core feature the subsystem selects and the turret follows, so we issue the one
            # `select_target` command per identity (the selection survives until the identity is
            # retired, and re-issuing it every frame would just spam the ack channel).
            obs = outcome.observation
            if obs is not None and obs.target_state is not TargetState.NO_TARGET                     and obs.track_uuid and args.select_controller is not None:
                displayed = _selected_display_index(outcome.track_set, obs.track_uuid)
                if displayed and obs.track_uuid not in _issued_selections:
                    try:
                        req = _urllib.Request(args.select_controller,
                                              data=_json.dumps({"command": "select_target",
                                                                "arg": str(displayed)}).encode(),
                                              headers={"Content-Type": "application/json"})
                        _urllib.urlopen(req, timeout=2).read()
                        _issued_selections.add(obs.track_uuid)
                        if not args.quiet:
                            print(f"visiond: select_target {displayed} -> control", file=sys.stderr)
                    except Exception:                                            # noqa: BLE001
                        pass
    return EXIT_OK


def _run_synthetic(args: argparse.Namespace, pipeline: PerceptionPipeline,
                   adapter: MockAdapter, config: VisionConfig,
                   wire_publisher: Optional[SocketPublisher] = None) -> int:
    """Drive the pipeline with the mock adapter: no sensor, same code path.

    The stream size is the configured one, or 1920x1080 — the point of the exercise is the
    daemon's own wiring (recorder, publisher, preview, timings), so it must use the same
    geometry numbers the station would, not something invented from the model's input size.
    """
    frames = args.max_frames or 60
    width = int(config.camera.width or 1920)
    height = int(config.camera.height or 1080)
    adapter.configure_stream(width, height)
    adapter.open()
    pipeline.start()
    interval_ns = int(1_000_000_000 / max(1, int(adapter.manifest.inference_rate_hz or 16)))
    # The synthetic clock advances at the model's cadence, not the host's — otherwise a run that
    # finishes in 3 ms of real time would never let §18's confirmation windows elapse, and the
    # offline acceptance path would report an identity that never became selectable. It is
    # anchored *behind* the host clock by a whole run, so §40's sensor→publish span stays
    # comparable instead of turning into a clock-domain mismatch on every frame.
    base = int(pipeline.clock()) - frames * interval_ns
    for index in range(frames):
        pipeline.process_frame(None, None, frame_sequence=index,
                               sensor_timestamp_ns=base + index * interval_ns)
    counters = pipeline.counters
    if not args.quiet:
        print(f"visiond: synthetic {frames} frames, "
              f"{counters.frames_with_tracks} with tracks, "
              f"{counters.documents_written} document pairs written"
              + (f", {counters.failures} frame failures" if counters.failures else ""),
              file=sys.stderr)
    return EXIT_OK


def _dump_diagnostics(path: str, diagnostics: AssociationDiagnostics) -> None:
    try:
        written = diagnostics.dump(path)
        print(f"visiond: wrote {written} §41 diagnostic records to {path}", file=sys.stderr)
    except OSError as exc:
        print(f"visiond: could not write diagnostics: {exc}", file=sys.stderr)


def _write_report(args: argparse.Namespace, payload: Dict[str, Any]) -> None:
    if not args.report:
        return
    directory = os.path.dirname(os.path.abspath(args.report))
    os.makedirs(directory, exist_ok=True)
    with open(args.report, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, default=str)
        handle.write("\n")


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        config = load_config(args)
    except ConfigPlaceholderError as exc:
        print(f"visiond: {exc}", file=sys.stderr)
        return EXIT_CONFIG
    except ConfigError as exc:
        print(f"visiond: {exc}", file=sys.stderr)
        return EXIT_CONFIG

    if args.probe_model:
        return run_probe(args.probe_model, config, args.profile or config.profile,
                         emit_manifest=args.emit_manifest)
    if args.replay:
        return run_replay(args, config)
    return run_capture(args, config)


if __name__ == "__main__":                                       # pragma: no cover
    raise SystemExit(main())
