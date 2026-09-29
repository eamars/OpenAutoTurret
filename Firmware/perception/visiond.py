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

Selection in capture mode uses the UUID API (§28); ``--select-uuid`` is replay-only, the mechanism
for re-asking §46's "requesting UUID A selects A or rejects, never B".

Nothing here imports ``picamera2``, ``CAN``, or the retired ``vision``/``control`` modules: the
camera path is reached through :mod:`perception.camera`, which is import-guarded, and the
subsystem publishes an atomic native observation/candidate datagram to controld.
Optional JSON snapshots are diagnostics, written outside the capture thread.
"""
from __future__ import annotations

from perception.camera_id import derive_camera_id, resolve_durable_id
import argparse
import json
import os
import sys
import threading
import time
from typing import Any, Dict, List, Optional

from .camera import CameraOwner, open_picamera2, open_picamera2_sensor
from .config import VisionConfig
from .errors import ConfigError, ConfigPlaceholderError, ModelRejected, PerceptionError
from .events import EventLog
from .model import (OFFLINE_ADAPTERS, EnvironmentManifest, build_adapter, manifest_for,
                    probe_model,
                    resolve_artifact)
from .model.adapter import MockAdapter
from .pipeline import LatestJsonPublisher, PerceptionPipeline, PreviewTap
from .preview import JpegPreviewWorker
from .detail_stream import DetailFrame, DetailStreamAnnouncer, SecondaryCameraStream
from .pipeline import PreviewTap
from .stream_manifest import StreamDescriptor, publish, publish_merged
from .protocol.jsonio import atomic_write_text, dumps as json_dumps
from .protocol.wire import SocketPublisher, encode_track_set
from .protocol.native_wire import encode_perception_frame
from .selection.service import SelectionService
from .selection.control_context import ControllerContext
from .replay import (Recorder, ReplaySource, compare_ground_truth, compare_runs,
                     engineering_gates, run_level_b)
from .selection.protocol import SelectTargetRequest

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
    parser.add_argument("--input-tensor-probe", default=os.environ.get('OTA_VISION_INPUT_TENSOR_PROBE', ''),
                        metavar="JPEG", help="capture one paired network input/ISP frame, then disable tensor output")
    parser.add_argument("--disable-preview", action="store_true",
                        help="do not offer frames to the preview tap (§39)")
    parser.add_argument("--preview-fps", type=float, default=0.0,
                        help="override preview rate (default: the config's)")
    parser.add_argument("--publish-dir", default="", metavar="DIR",
                        help="write track_set.json / selected_target.json here (§38)")
    parser.add_argument("--select-controller", default="", metavar="URL",
                        help="retired; use the native UUID selection API")
    parser.add_argument("--publish-socket", default="", metavar="PATH",
                        help="publish atomic native observation and candidate-list frames to controld")
    parser.add_argument("--selection-socket", default="/tmp/ota-selection.sock",
                        help="local UUID selection/ACK socket; empty disables the service")
    parser.add_argument('--controller-state-url', default='',
                        help='read-only controller /api/state for optional AUTO_SELECT_SINGLE')
    parser.add_argument("--legacy-track-wire", action="store_true",
                        help="compatibility bridge: publish only the old TrackSet contract")
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
                        help="replay-only UUID selection request; live selection uses the UUID API")
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
    if args.select_controller:
        print('visiond: --select-controller is retired; use the UUID selection API', file=sys.stderr)
        return EXIT_CONFIG
    if args.select_uuid:
        print('visiond: --select-uuid is replay-only; use the live UUID selection API', file=sys.stderr)
        return EXIT_CONFIG
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
    preview_worker = None
    tap_path = os.environ.get("OTA_VISION_FRAME_TAP", "").strip()
    if preview.enabled and tap_path:
        preview_worker = JpegPreviewWorker(preview, tap_path)
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
                            asynchronous=True,
                            flush_every=max(1, int(config.record.flush_every)))
        recorder.open()
        print(f"visiond: recording dataset to {args.record_dataset}", file=sys.stderr)

    try:
        adapter = build_adapter(config, manifest=manifest)
        # "What I cannot see has not been updated": the backend publishes its own self-report once a
        # second so the web surface can say, in one glance, which network is producing the tracks.
        # The path defaults to beside the stream manifest, so a station that set one has the other.
        from .inference_health import HealthPublisher
        health_path = (os.environ.get("OTA_INFERENCE_HEALTH", "").strip()
                       or (os.path.dirname(os.environ.get("OTA_VISION_STREAM_MANIFEST", "").strip())
                           + "/inference_health.json"))
        # The second camera's adapter does not exist yet at this line -- it is created once its own
        # sensor has opened -- so the health beat asks a function for its companions every second.
        # An absent key then means "there is no second leg in this boot", which is a different claim
        # from a key that says 0 Hz, and the difference is the whole point of publishing this file.
        health_extras: Dict[str, Any] = {"fn": None}
        if health_path.strip("/"):
            adapter_health = HealthPublisher(
                adapter=adapter, path=health_path,
                companions=lambda: (health_extras["fn"]() if callable(health_extras["fn"]) else {})
            ).start()
            print(f"visiond: publishing inference health to {health_path}", file=sys.stderr)
    except ConfigError as exc:
        print(f"visiond: {exc}", file=sys.stderr)
        return EXIT_CONFIG

    # ``info`` only becomes real once a sensor is actually open, but the report path at the end of
    # this function reads it unconditionally. Binding it before the try means a camera that never
    # opened still produces that report -- which is the one thing telling an operator why nothing
    # is streaming. An empty dict degrades the identity to ``/dev/video?`` with source ``index`` and
    # durable=False: the honest claim for "we could not open a camera, so we cannot name it".
    info: Dict[str, Any] = {}
    publisher = LatestJsonPublisher(args.publish_dir) if args.publish_dir else None
    wire_publisher = SocketPublisher(args.publish_socket) if args.publish_socket else None
    from common.image_corrections import read_install_orientation
    orientation, source = read_install_orientation()
    print(f"visiond: camera orientation {orientation} ({source})", file=sys.stderr)
    pipeline = PerceptionPipeline(config, adapter=adapter, event_log=events,
                                  diagnostics=diagnostics, recorder=recorder,
                                  publisher=publisher, preview=preview,
                                  session_uuid=args.session_uuid or "", orientation=orientation)
    if args.selection_socket and not args.legacy_track_wire:
        pipeline.selection_service = SelectionService(args.selection_socket)
    if args.controller_state_url and pipeline.selector.auto.enabled:
        pipeline.controller_context = ControllerContext(args.controller_state_url)
    lores: Optional[Tuple[int, int]] = None      # set by the profile that asks for a second leg
    camera: Optional[CameraOwner] = None
    try:
        if pipeline.selection_service is not None:
            pipeline.selection_service.start()
        if pipeline.controller_context is not None:
            pipeline.controller_context.start()
        if preview_worker is not None:
            preview_worker.start()
        if isinstance(adapter, MockAdapter):
            # No camera on a mock profile: synthetic frames, so the daemon's own wiring can be
            # exercised on a machine with no sensor attached (§55.18's offline acceptance run).
            return _run_synthetic(args, pipeline, adapter, config, wire_publisher=wire_publisher)
        model = config.active_model
        if (model.adapter or "").strip().lower() in ("hailo", "hailo8"):
            # The Hailo profile names its independent camera and capture geometry. The
            # current IMX500 installation orientation is not presumed to describe IMX477.
            adapter.open()
            width = int(model.camera_width or 640)
            height = int(model.camera_height or 480)
            rate_hz = float(model.camera_frame_rate_hz or 15.0)
            lores_w, lores_h = int(model.camera_lores_width or 0), int(model.camera_lores_height or 0)
            if lores_w > 0 and lores_h > 0:
                lores = (lores_w, lores_h)
            picam2, info = open_picamera2_sensor(
                model.camera_model,
                stream_size=(width, height),
                frame_rate_hz=rate_hz,
                orientation=model.camera_orientation,
                lores_size=lores,
            )
            # The configured libcamera transform applies to pixels before Hailo inference.
            pipeline.orientation = 'none'
        else:
            requested_stream = ((int(config.camera.width), int(config.camera.height))
                                if config.camera.width and config.camera.height else None)
            imx500, picam2, info = open_picamera2(manifest.path, stream_size=requested_stream,
                                                external_manifest=manifest, orientation=orientation)
            # Sensor orientation also corrects the neural-network input. Both its
            # boxes and the image now arrive upright; do not rotate either again.
            pipeline.orientation = 'none'
        stream = (int(info["stream_size"][0]), int(info["stream_size"][1]))
        # Inference is configured for the leg it will actually be handed -- the ISP's small picture
        # when the profile asked for one. Reporting the model input without this would let the
        # surface claim 640x360 while the network quietly ate a downscaled 1080p.
        # The leg is what inference reads; the declaration is the picture the station publishes and the
        # control layer validates a TrackSet against. Naming the leg here is what kept every TrackSet
        # out of the control loop while the tracker was confirming tracks just fine.
        adapter.configure_stream(*(lores or stream), declared=stream)
        camera = CameraOwner(picam2, stream_size=stream, events=events,
                             inference_stream=("lores" if lores else None),
                             inference_size=(lores or stream))
        if (model.adapter or "").strip().lower() not in ("hailo", "hailo8"):
            adapter.open(device=imx500, camera=picam2)
        pipeline.start()
        primary_ident = resolve_durable_id(str(info.get("device_path")
                                              or f"/dev/video{info.get('camera_num', '?')}"))
        merge_view = None
        if lores is not None and (int(config.secondary.lores_width or 0) > 0):
            from .tracking.camera_registry import MergedTrackSetView
            # The canvas is the picture the station publishes -- not either camera's own stream
            # size. Both TrackSets declare it, so the merged document states one geometry and every
            # box inside it is a fraction of a full-frame view of its own optics.
            merge_canvas = (int(stream[0]), int(stream[1]))
            merge_view = MergedTrackSetView(
                max_age_ns=int(float(config.merge_max_age_ms) * 1_000_000))

            # One physical Hailo-8 carries one HailoRT context, so the second camera joins the first
            # adapter's device instead of opening its own. H5 did it the other way round and the chip
            # answered HAILO_DEVICE_IN_USE(73) — see the post-mortem in the work order. A non-Hailo
            # primary has no device to lend, and build_adapter refuses a device it would ignore.
            shared_device = getattr(adapter, "device", None)

            def detail_factory(camera_id, _config=config, _manifest=manifest,
                               _device=shared_device, _events=None):
                from .model import build_adapter as _build
                second = _build(_config, manifest=_manifest, device=_device)
                second_events = EventLog(capacity=256)
                second_pipeline = PerceptionPipeline(
                    _config, adapter=second, event_log=second_events,
                    session_uuid=args.session_uuid or "", orientation='none')
                return second_pipeline, second

            detail = _start_detail_stream(primary_ident=primary_ident,
                                          secondary=config.secondary,
                                          pipeline_factory=detail_factory,
                                          merge_view=merge_view,
                                          merge_canvas=merge_canvas)
            if detail is not None and detail.get("adapter") is not None:
                def _health_companions(_a=adapter, _d=detail["adapter"],
                                       _stream=detail["stream"], _view=merge_view):
                    report = {"cameras": {str(_a.camera_id or "unbound"): _a.describe(),
                                          str(_d.camera_id): _d.describe()},
                              "detail_stream": _stream.stats()}
                    if _view is not None:
                        report["merge"] = {
                            "max_age_ms": _view.max_age_ns / 1e6,
                            "per_camera": _view.stats(), "offers": _view.offers,
                            "dropped_stale": _view.dropped_stale,
                            "refusals": _view.refusals, "last_refusal": _view.last_refusal}
                    return report
                health_extras["fn"] = _health_companions
        else:
            detail = _start_detail_stream(primary_ident=primary_ident,
                                          secondary=config.secondary)
        try:
            return _run_camera(args, pipeline, adapter, camera, info,
                            wire_publisher=wire_publisher, preview=preview_worker,
                            merge_view=merge_view)
        finally:
            _stop_detail_stream(detail)
    except ModelRejected as exc:
        print(f"visiond: model refused (§9.3):\n{exc}", file=sys.stderr)
        return EXIT_MODEL
    except ConfigError as exc:
        print(f"visiond: {exc}", file=sys.stderr)
        return EXIT_CONFIG
    finally:
        if pipeline.controller_context is not None:
            pipeline.controller_context.close()
        if pipeline.selection_service is not None:
            pipeline.selection_service.close()
        if preview_worker is not None:
            preview_worker.stop()
        if wire_publisher is not None:
            wire_publisher.close()
        if publisher is not None:
            publisher.close()
        if camera is not None:
            camera.close()
        adapter.close()
        pipeline.stop()
        report = pipeline.report()
        if publisher is not None:
            report['json_publisher'] = publisher.stats()
        if wire_publisher is not None:
            report["wire_publisher"] = wire_publisher.stats()
        if preview_worker is not None:
            report["preview_worker"] = preview_worker.stats()
        if camera is not None:
            report["camera"] = camera.stats.to_dict()
        if args.diagnostics:
            _dump_diagnostics(args.diagnostics, diagnostics)
        # WP3: every frame says which camera it came from, and how much that claim is worth.
        # Today the platform hands us `camera_num`, a kernel number, so the source tag reads
        # "index" and durable=False -- the HUD shows that instead of my promising durability.
        # Upgrading it means handing open_picamera2 a /dev/v4l/by-path node, which is a config
        # change, not a code change; the tag is what makes that difference visible on the page.
        ident = derive_camera_id(str(info.get("device_path")
                                   or f"/dev/video{info.get('camera_num', '?')}"))
        payload = {"mode": "capture", "camera_id": ident.id,
                   "camera_identity_source": ident.source, **report}
        _write_report(args, payload)
        if not args.quiet:
            print(json.dumps(payload, indent=2))
        events.close()


def _publish_wire(outcome, publisher: Optional[SocketPublisher], *, legacy=False) -> bool:
    """One measurement authority per frame, shared by live and offline daemon runs."""
    if publisher is None or outcome.track_set is None:
        return False
    track_set = outcome.track_set
    if not legacy:
        if outcome.observation is None:
            return False
        return publisher.send(encode_perception_frame(track_set, outcome.observation))
    return publisher.send(encode_track_set(
        track_set.tracks, frame_sequence=track_set.frame_sequence,
        sensor_timestamp_ns=track_set.sensor_timestamp_ns,
        publish_timestamp_ns=track_set.publish_timestamp_ns,
        width=track_set.stream_width, height=track_set.stream_height))


#: The role a physical sensor is allowed to publish under (§ architect's (b) binding). This is a
#: binding by *model + by-path identity*, never by probe order: a station that grows a third
#: sensor gets a new name here rather than a renumbering of the old ones.
STREAM_ROLES = {"imx500": "wide", "imx477": "detail"}


def _role_for_stream(info: Dict[str, Any]) -> Optional[str]:
    model = str(info.get("sensor_model") or info.get("camera_model")
                or info.get("model") or "").strip().lower()
    role = STREAM_ROLES.get(model)
    if role is None:
        # An unmapped sensor gets no name rather than a borrowed one: `wide` on an unknown sensor
        # would let a UI label pixels it cannot attribute.
        print(f"visiond: sensor model {model!r} has no stream role; publishing no manifest",
              file=sys.stderr)
    return role


class _StreamAnnouncer:
    """Republishes the named-stream manifest at 1 Hz while the camera is open.

    `delivered_fps` is measured from the encoder's own counter across the last window, and stays
    ``None`` until one whole window has samples. A rate nobody measured must not sit in the
    manifest as 0, or the first second of every boot renders as "this stream is dead".
    """

    def __init__(self, *, path: str, role: str, ident: Any, size, preview=None,
                 interval_s: float = 1.0) -> None:
        self.path = str(path)
        self.role = role
        self.ident = ident
        self.size = (int(size[0]), int(size[1]))
        self.preview = preview
        self.interval_ns = max(1, int(float(interval_s) * 1_000_000_000))
        self._stop = threading.Event()
        self._thread = None
        self._last_count = 0
        self._last_ns = time.monotonic_ns()
        self.failures = 0
        self.last_error = ""

    @classmethod
    def from_environment(cls, *, role, ident, size, preview):
        path = os.environ.get("OTA_VISION_STREAM_MANIFEST", "").strip()
        if not path or role is None:
            return None
        return cls(path=path, role=role, ident=ident, size=size, preview=preview)

    def start(self) -> None:
        self.publish_once()          # an empty manifest beats a consumer guessing at absence
        self._thread = threading.Thread(target=self._run, name="stream-manifest", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.wait(self.interval_ns / 1_000_000_000.0):
            self.publish_once()

    def publish_once(self) -> None:
        count = int(getattr(self.preview, "published", 0)) if self.preview is not None else 0
        now = time.monotonic_ns()
        elapsed_ns = now - self._last_ns
        fps = None
        if elapsed_ns >= self.interval_ns and count > self._last_count:
            fps = round((count - self._last_count) * 1_000_000_000.0 / elapsed_ns, 2)
        if elapsed_ns >= self.interval_ns:
            self._last_count, self._last_ns = count, now
        try:
            publish_merged(path=self.path, descriptor=StreamDescriptor(
                role=self.role, camera_id=self.ident.id, identity_source=self.ident.source,
                durable=bool(self.ident.durable), path=str(self.preview.path) if self.preview
                else "", width=self.size[0], height=self.size[1], delivered_fps=fps,
                dropped=int(getattr(self.preview, "failures", 0)) if self.preview else None,
                updated_ns=now))
            self.last_error = ""
        except (OSError, ValueError) as exc:
            # A manifest that cannot be written is a degraded UI, not a lost camera: the capture
            # thread must not die because a file could not be replaced. Loud, counted, continuing.
            self.failures += 1
            self.last_error = f"{type(exc).__name__}: {exc}"

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)


DETAIL_STREAM_ENV = "OTA_VISION_DETAIL_SENSOR"


def _start_detail_stream(*, primary_ident=None, secondary=None, pipeline_factory=None,
                         merge_view=None, merge_canvas=None):
    """Open the secondary sensor as a *preview-only* stream, if the launcher asked for one.

    The switch names a **sensor model**, not an index: ``/dev/videoN`` is a lease for this boot.
    What the stream then *publishes* is a by-path identity resolved from the node we actually
    opened, so the claim the manifest makes is port-derived, not order-derived. A station with two
    identical sensors would need the port in configuration; that limitation is written down rather
    than hidden behind a number.
    """
    configured = (secondary.model if secondary is not None else "")
    # The env var is an operator override for one boot; the document is the durable answer. Both
    # name a **sensor model**, never a node number.
    model = (os.environ.get(DETAIL_STREAM_ENV, "").strip().lower() or configured)
    if not model:
        return None
    if model not in STREAM_ROLES:
        print(f"visiond: secondary sensor {model!r} maps to no stream role; opening no "
              "secondary stream", file=sys.stderr)
        return None
    width = int(getattr(secondary, "width", 1280) or 1280)
    height = int(getattr(secondary, "height", 720) or 720)
    lores_w = int(getattr(secondary, "lores_width", 0) or 0)
    lores_h = int(getattr(secondary, "lores_height", 0) or 0)
    lores = (lores_w, lores_h) if lores_w > 0 and lores_h > 0 else None
    rate = float(getattr(secondary, "frame_rate_hz", 30.0) or 30.0)
    preview_fps = float(getattr(secondary, "preview_fps", 10.0) or 10.0)
    queue_depth = max(1, int(getattr(secondary, "queue_depth", 1) or 1))
    tap_path = os.environ.get("OTA_VISION_FRAME_TAP", "").strip()
    manifest_path = os.environ.get("OTA_VISION_STREAM_MANIFEST", "").strip()
    run_dir = os.path.dirname(tap_path) or os.path.dirname(manifest_path)
    if not run_dir:
        print("visiond: no run dir for the secondary preview; opening no secondary stream",
              file=sys.stderr)
        return None
    try:
        # One mechanism for both sensors, one value per role: the transform happens at the sensor so
        # the preview, the neural network and any saved frame all see the same upright pixels. The
        # value is configuration because which sensor is mounted upside down is a fact about the
        # station -- and an unsupported value is a named refusal, not a silent "none".
        want = (getattr(secondary, "orientation", "none") or "none")
        if want not in ("none", "rotate_180", "flip_horizontal", "flip_vertical"):
            print(f"visiond: secondary.orientation {want!r} is not supported; using 'none' "
                  "(legal: none, rotate_180, flip_horizontal, flip_vertical)", file=sys.stderr)
            want = "none"
        picam2, info = open_picamera2_sensor(model, stream_size=(width, height),
                                             frame_rate_hz=rate, orientation=want,
                                             lores_size=lores)
    except Exception as exc:                                                  # noqa: BLE001
        # The wide camera is already open and serving: a secondary sensor that will not open is
        # one stream down, said out loud, not a daemon that takes the station's video with it.
        print(f"visiond: secondary {model} did not open: {type(exc).__name__}: {exc}",
              file=sys.stderr)
        return None
    # Identity comes from the camera stack's own firmware-node path (`.../i2c@80000/imx477@1a`),
    # not from a node number: libcamera's camera index is NOT a /dev/videoN number, and on this
    # station deriving it that way made the IMX477 claim the IMX500's identity.
    fwnode = str(info.get("fwnode") or "").strip()
    ident = derive_camera_id(fwnode) if fwnode else resolve_durable_id(
        f"/dev/video{info['camera_num']}")
    if fwnode == "" :
        print("visiond: secondary sensor reported no firmware node path; falling back to the "
              "kernel node for its identity (less durable, said out loud)", file=sys.stderr)
    if primary_ident is not None and ident.id == getattr(primary_ident, "id", None):
        # Two named streams claiming one camera is worse than one stream: every consumer that
        # attributes detections by camera_id would be silently wrong. Refuse, say why, and keep
        # the wide stream serving.
        print(f"visiond: secondary sensor resolved to the primary's identity {ident.id}; "
              "publishing no secondary stream", file=sys.stderr)
        try:
            picam2.stop()
            picam2.close()
        except Exception:                                             # noqa: BLE001
            pass
        return None
    stream_size = (int(info["stream_size"][0]), int(info["stream_size"][1]))
    # Where this camera's boxes will be declared. With a merge, that is the published canvas --
    # normally the primary's picture -- because the merged document states exactly one geometry.
    # Without a merge, a camera declares its own picture, which is what a preview-only role does.
    merge_canvas = ((int(merge_canvas[0]), int(merge_canvas[1])) if merge_canvas else stream_size)
    print(f"visiond: secondary stream {model} node /dev/video{info['camera_num']} identity "
          f"{ident.id} source={ident.source} orientation={want!r} durable={ident.durable} "
          f"{stream_size[0]}x{stream_size[1]}", file=sys.stderr)
    try:
        picam2.start()
    except Exception as exc:                                                  # noqa: BLE001
        print(f"visiond: secondary {model} failed to start: {type(exc).__name__}: {exc}",
              file=sys.stderr)
        return None
    tap = PreviewTap(enabled=True, fps=preview_fps, latest_queue_depth=1)
    preview = JpegPreviewWorker(tap, os.path.join(run_dir, "preview_detail.jpg"))
    preview.start()

    def poll():
        request = picam2.capture_request()
        try:
            image = request.make_array("main").copy()      # the buffer dies with the request
            # The leg is copied for the same reason the main stream is: both arrays belong to the
            # request being released below, and an adapter that reads the buffer afterwards reads
            # whatever the next frame put there.
            leg = (request.make_array("lores").copy() if lores is not None else None)
            metadata = dict(request.get_metadata())   # 名字里带下划线：getmetadata() 不存在
        finally:
            request.release()
        sequence = int(metadata.get("sequence", 0))
        stamp = metadata.get("SensorTimestamp")
        tap.offer(image, now_ns=None, metadata={"camera": "detail",
                                               "sensor_timestamp_ns": stamp,
                                               "frame_sequence": sequence})
        return DetailFrame(camera_id=ident.id, frame_sequence=sequence,
                           sensor_timestamp_ns=None if stamp is None else int(stamp),
                           image=image, metadata=metadata,
                           inference_image=leg,
                           inference_size=(lores or stream_size))

    stream = SecondaryCameraStream(role="detail", ident=ident, poll=poll,
                                   queue_depth=queue_depth)
    stream.start()
    detail_pipeline = detail_adapter = detail_worker = None
    if pipeline_factory is not None and merge_view is not None:
        # A second pipeline over the same profile: its own adapter (one adapter, one camera), its
        # own event log (one camera's bad day stays its own), and no publisher of its own -- the
        # wide camera's loop is the single publisher, so a document appears at one cadence and not
        # at whichever camera got there first.
        from .camera_worker import CameraWorker
        detail_pipeline, detail_adapter = pipeline_factory(ident.id)
        detail_adapter.bind_camera(ident.id)
        detail_adapter.configure_stream(*(lores or stream_size), declared=merge_canvas)
        detail_adapter.open()
        detail_pipeline.start()

        def infer_step():
            frame = stream.latest(timeout_s=0.2)
            if frame is None:
                return None                      # no frame yet: the worker keeps waiting, alive
            if frame.inference_image is None:
                return None
            outcome = detail_pipeline.process_frame(
                frame.inference_image, frame.metadata,
                frame_sequence=frame.frame_sequence,
                sensor_timestamp_ns=frame.sensor_timestamp_ns or 0,
                camera_id=ident.id)
            if outcome.track_set is not None:
                merge_view.offer(outcome.track_set)
            return outcome

        detail_worker = CameraWorker(ident, infer_step)
        detail_worker.start()
        print(f"visiond: detail inference on {ident.id} leg "
              f"{(lores or stream_size)[0]}x{(lores or stream_size)[1]} declared "
              f"{merge_canvas[0]}x{merge_canvas[1]}", file=sys.stderr)
    announcer = None
    if manifest_path:
        announcer = DetailStreamAnnouncer(path=manifest_path, stream=stream, preview=preview,
                                          size=stream_size)
        announcer.start()
    return {"picam2": picam2, "stream": stream, "preview": preview, "announcer": announcer,
            "pipeline": detail_pipeline, "adapter": detail_adapter, "worker": detail_worker}


def _stop_detail_stream(detail) -> None:
    if not detail:
        return
    # The worker first, then the pipeline it was feeding, then the stream that fed it: stopping in
    # the other order lets a step hand a frame to a pipeline that is already closed, which is a
    # counted error nobody asked for.
    for key in ("worker", "pipeline", "announcer", "stream", "preview"):
        part = detail.get(key)
        if part is not None:
            part.stop()
    try:
        detail["picam2"].stop()
        detail["picam2"].close()
    except Exception as exc:                                                  # noqa: BLE001
        print(f"visiond: secondary sensor released badly: {type(exc).__name__}: {exc}",
              file=sys.stderr)


def _run_camera(args: argparse.Namespace, pipeline: PerceptionPipeline, adapter: Any,
                camera: CameraOwner, info: Dict[str, Any],
                wire_publisher: Optional[SocketPublisher] = None,
                preview: Optional[JpegPreviewWorker] = None,
                merge_view: Optional[Any] = None) -> int:
    # WP3: the owner announces which camera it owns, and how durable that claim is. This is the
    # layer where ownership lives -- carrying it into controld needs a v3 wire-schema bump (the
    # perception report is a typed structure, not a dict), which belongs to the dual-worker cut
    # that reworks that header anyway. Saying it here, now, means every boot leaves a record of
    # the identity it was given -- including when that identity is only a kernel number.
    _node = str(info.get("device_path") or f"/dev/video{info.get('camera_num', '?')}")
    _ident = resolve_durable_id(_node)   # prefers a by-path name for the same node, when present
    # file=sys.stderr, like every other line here: the launcher captures stderr into vision.log,
    # so a plain stdout print in this daemon is a print into the void. That is why this line was
    # missing from the station's log even though the code demonstrably ran (frames were flowing).
    print(f"visiond: camera identity {_ident.id} source={_ident.source} "
          f"durable={_ident.durable}", file=sys.stderr)
    print(f"visiond: camera {info['camera_num']} stream "
          f"{info['stream_size'][0]}x{info['stream_size'][1]} model {info['task']} "
          f"@ {info['inference_rate_hz']} Hz", file=sys.stderr)
    # One adapter serves one camera: it holds the geometry, the letterbox pad and every counter
    # that only makes sense per leg. Binding happens here, where ownership is decided, so a frame
    # routed to the wrong worker is refused by name instead of quietly degrading the other leg's
    # statistics. Adapters without a binding (replay, offline) stay unattributed and keep working.
    bind = getattr(adapter, "bind_camera", None)
    if callable(bind):
        try:
            bind(_ident.id)
        except Exception as exc:                                          # noqa: BLE001
            print(f"visiond: the inference adapter would not bind to camera {_ident.id}: "
                  f"{type(exc).__name__}: {exc}", file=sys.stderr)
            raise
    # One publisher for the whole station: the document on disk is the merge, while the control
    # layer keeps receiving *this* camera's set. That asymmetry is deliberate and it is said out
    # loud below -- controld's wire header has no camera field until v3, so handing it a merged set
    # would make a box from the narrow optic aim the turret as if it came from the wide one.
    publish_hook = None
    if merge_view is not None and pipeline.publisher is not None:
        def publish_hook(track_set, observation, _view=merge_view, _pub=pipeline.publisher,
                         _pipeline=pipeline):
            _view.offer(track_set)
            document = _view.merge() or track_set
            _pub.publish(document, observation)
            if isinstance(_pub, LatestJsonPublisher):
                _pipeline.counters.documents_enqueued += 1
            else:
                _pipeline.counters.documents_written += 1
        print("visiond: publishing the merged TrackSet to the document path; the control wire "
              "carries only this camera's set until the wire carries camera attribution (v3)",
              file=sys.stderr)
    camera.start()
    # (b): visiond owns the physical sensor, so visiond is also the only process allowed to say
    # which named stream came out of it. The manifest is what webd reads instead of a filename it
    # guessed; nothing here changes what webd can do with the pixels.
    announcer = _StreamAnnouncer.from_environment(role=_role_for_stream(info), ident=_ident,
                                                  size=info["stream_size"], preview=preview)
    if announcer is not None:
        announcer.start()
        print(f"visiond: publishing stream manifest -> {announcer.path}", file=sys.stderr)
    tensor_probe = None
    try:
        if args.input_tensor_probe:
            from .tensor_probe import InputTensorProbe
            tensor_probe = InputTensorProbe(args.input_tensor_probe, camera.device, adapter.device)
            tensor_probe.start()
        delivered = 0
        for frame in camera.frames(max_frames=args.max_frames):
            # `is not None`, never `or`: the leg is a numpy array, and `array or x` asks numpy for a
            # truth value -- which is exactly the ValueError that killed visiond six seconds into
            # the first Hailo boot. A pixel buffer is never falsy, so the intent has to be spelled.
            outcome = pipeline.process_frame(
                frame.inference_image if frame.inference_image is not None else frame.image,
                frame.metadata,
                                             frame_sequence=frame.frame_sequence,
                                             sensor_timestamp_ns=frame.sensor_timestamp_ns,
                                             camera_id=_ident.id,
                                             capture_started_ns=frame.metadata_receive_ns,
                                             publish=publish_hook)
            delivered += 1
            if tensor_probe is not None:
                tensor_probe.offer(frame, outcome)
                if tensor_probe.done:
                    tensor_probe.close()
                    tensor_probe = None
            if not args.quiet and delivered % 30 == 0:
                tracks = len(outcome.track_set.tracks) if outcome.track_set else 0
                print(f"visiond: frame {frame.frame_sequence} tracks {tracks} "
                      f"target {outcome.observation.target_state.name if outcome.observation else 'NO_TARGET'}",
                      file=sys.stderr)
            if not outcome.published and outcome.stage != 'inference_pending' and not args.quiet:
                print(f"visiond: frame {frame.frame_sequence} failed in {outcome.stage}: "
                      f"{outcome.failure}", file=sys.stderr)
            if wire_publisher is not None and outcome.stage != 'inference_pending':
                if not _publish_wire(outcome, wire_publisher, legacy=args.legacy_track_wire) and not args.quiet:
                    print("visiond: TrackSet publish failed", file=sys.stderr)
            if isinstance(pipeline.publisher, LatestJsonPublisher):
                wire_done_ns = time.monotonic_ns()
                kpi = None
                try:
                    kpi = adapter.device.get_kpi_info(frame.metadata)
                except (AttributeError, KeyError, TypeError):
                    pass
                pipeline.publisher.publish_timing(dict(
                    frame_sequence=frame.frame_sequence, sensor_timestamp_ns=frame.sensor_timestamp_ns,
                    metadata_receive_ns=frame.metadata_receive_ns, wire_done_ns=wire_done_ns,
                    published=outcome.published, stages_ms=outcome.timings_ms,
                    detections=len(outcome.detection_set.detections) if outcome.detection_set else 0,
                    tracks=len(outcome.track_set.tracks) if outcome.track_set else 0,
                    batch_geometry_verified=getattr(adapter,'_batch_geometry_verified',False),
                    batch_geometry_rejected=getattr(adapter,'_batch_geometry_rejected',False),
                    camera={k: frame.metadata[k] for k in ('ExposureTime','FrameDuration',
                        'AnalogueGain','DigitalGain','_ota_image_copy_ms') if k in frame.metadata},
                    imx500_kpi_ms=kpi))
    finally:
        if announcer is not None:
            announcer.stop()
        if tensor_probe is not None:
            tensor_probe.close()
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
    # WP3: the frames are driven by a per-camera worker rather than by this function's own stack,
    # so the offline path exercises the same lifecycle the second camera will use -- start, retire,
    # die alone -- instead of only the real-sensor path. The identity is derived from the configured
    # device, so the identity below is a label rather than a durable path.
    from perception.camera_id import derive_camera_id
    from perception.camera_worker import CameraWorker, WorkerSupervisor

    state = {"index": 0}
    synthetic_ident = derive_camera_id("mock://synthetic")

    def step():
        index = state["index"]
        if index >= frames:
            return None
        state["index"] += 1
        outcome = pipeline.process_frame(None, None, frame_sequence=index,
                                         sensor_timestamp_ns=base + index * interval_ns,
                                         camera_id=synthetic_ident.id)
        _publish_wire(outcome, wire_publisher, legacy=args.legacy_track_wire)
        return index

    # The mock profile has no device to name -- ``CameraConfig`` carries geometry, not a path -- so
    # the identity is a label. ``source=label`` and ``durable=False`` in the report are the truth
    # about a synthetic run, not a shortfall to apologise for.
    worker = CameraWorker(synthetic_ident, step)
    supervisor = WorkerSupervisor().add(worker)
    supervisor.start_all()
    deadline = time.monotonic() + max(10.0, frames * (interval_ns / 1e9) * 20)
    while worker.frames < frames and time.monotonic() < deadline:
        time.sleep(0.002)
    worker.stop(join_s=1.0)
    if worker.frames < frames:
        print(f"visiond: synthetic worker delivered {worker.frames} of {frames} frames "
              f"(state {worker.state}) before the deadline", file=sys.stderr)
    synthetic_worker_status = supervisor.status()
    counters = pipeline.counters
    if not args.quiet:
        print(f"visiond: synthetic {frames} frames, "
              f"{counters.frames_with_tracks} with tracks, "
              f"{counters.documents_written} document pairs written"
              + (f", {counters.failures} frame failures" if counters.failures else ""),
              file=sys.stderr)
        # One line per camera worker: what it delivered, what it dropped, and how it ended. The
        # state is the point -- "44 of 60 frames" with a dead worker is a different incident than
        # "60 of 60" with a clean stop, and the difference is only visible if it is said.
        for key, status in sorted(synthetic_worker_status.items()):
            print(f"visiond: worker {key[:12]} state={status['state']} "
                  f"frames={status['frames']} dropped={status['dropped']} "
                  f"generation={status['generation']}"
                  + (f" error={status['error']}" if status['error'] else ""),
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
    import signal
    def _shutdown(*_):
        raise SystemExit(EXIT_OK)
    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)
    raise SystemExit(main())
