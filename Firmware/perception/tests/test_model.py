"""§9's model layer: the manifest, the environment record, the probe, the adapters.

Nothing here needs a camera. That is the point of §9.2's second clause — "the adapter produces
identical normalized detections" is a claim about *this code*, and a claim that could only be
checked on a station with an IMX500 attached could not be checked at all by the offline suite
§55.18 requires. So the device is always injected, and the real import is reached only through
the default factory.
"""
from __future__ import annotations

import json
import os
import tempfile
import unittest

import perception
from perception.config import ModelConfig, ScoreThresholds, VisionConfig
from perception.errors import (ConfigError, ConfigPlaceholderError, ModelRejected,
                           ValidationError)
from perception.model import (EnvironmentManifest, LabelMap, MockAdapter, ModelManifest,
                              ProbeResult, admit, build_adapter, labels_hash, manifest_for,
                              probe_model, resolve, resolve_artifact, rows_for_moving_target,
                              sha256_file)
from perception.model.imx500_yolo import Imx500YoloAdapter
from perception.model.label_maps import COCO

PACKAGE_ROOT = os.path.dirname(os.path.abspath(perception.__file__))
GOOD_INTRINSICS = {
    "task": "object detection", "inference_rate": 16.0, "bbox_order": "xy",
    "bbox_normalization": True, "preserve_aspect_ratio": True,
    "labels": list(COCO), "postprocess": "on_sensor",
}


def manifest(**overrides) -> ModelManifest:
    values = {"model_id": "test-model", "input_width": 640, "input_height": 640,
              "inference_rate_hz": 16.0}
    values.update(overrides)
    return ModelManifest(**values)


class FakeDevice:
    """Enough of picamera2's ``IMX500`` to exercise the adapter and the probe."""

    def __init__(self, intrinsics=None, rows=None, input_size=(640, 640),
                 shapes=((1, 4, 6),)) -> None:
        self.network_intrinsics = dict(GOOD_INTRINSICS) if intrinsics is None \
            else dict(intrinsics)
        self.camera_num = 0
        self._rows = [[0.9, 0.0, 0.28, 0.35, 0.38, 0.65]] if rows is None else rows
        self._input_size = input_size
        self._shapes = shapes
        self.get_outputs_calls = 0

    def get_input_size(self):
        return self._input_size

    def get_output_shapes(self, metadata):
        return [list(shape) for shape in self._shapes]

    def get_outputs(self, metadata, add_batch=False):
        self.get_outputs_calls += 1
        return [self._rows]


class _SplitDevice(FakeDevice):
    """A fake whose ``get_outputs`` returns the on-sensor _pp parallel tensors."""

    def __init__(self, boxes, scores, classes, count):
        import numpy as _np
        super().__init__()
        self._tensors = [_np.asarray(boxes, dtype=float),
                         _np.asarray(scores, dtype=float),
                         _np.asarray(classes, dtype=float),
                         _np.asarray(count, dtype=float)]

    def get_outputs(self, metadata, add_batch=False):
        self.get_outputs_calls += 1
        return list(self._tensors)


class _SingleRowsDevice(FakeDevice):
    """A fake whose output is one row-major ``[score, class, box…]`` tensor."""

    def __init__(self, rows):
        super().__init__()
        self._rows = rows

    def get_outputs(self, metadata, add_batch=False):
        self.get_outputs_calls += 1
        return [list(self._rows)]


class TestManifest(unittest.TestCase):
    def test_a_declared_manifest_loads_and_validates(self):
        item = manifest(model_id="imx500-yolo11n-pp-coco")
        self.assertIs(item.validate(), item)
        self.assertEqual(item.label_map().name(0), "person")
        self.assertEqual(item.label_map().index_of("Person"), 0)

    def test_the_manifest_survives_a_round_trip(self):
        item = manifest(notes=["one", "two"]).with_runtime(path="/tmp/a.rpk")
        payload = item.to_dict()
        self.assertEqual(payload["task"], "object_detection")
        restored = ModelManifest.from_dict(payload)
        self.assertEqual(restored.model_id, item.model_id)
        self.assertEqual(restored.notes, "one\ntwo")
        self.assertEqual(restored.score_indices(), (0, 1, 2))

    def test_the_documented_spelling_of_a_task_is_normalised(self):
        # Appendix D's intrinsics say "object detection"; a hand-written manifest says
        # "object_detection". Both must land on one value or §9.3 compares spellings.
        self.assertEqual(ModelManifest.from_dict({"task": "object detection"}).task,
                         "object_detection")
        self.assertEqual(ModelManifest.from_dict({"task": "Pose Estimation"}).task,
                         "pose_estimation")

    def test_an_uncommissionable_field_is_reported_with_every_problem_at_once(self):
        item = ModelManifest(model_id="", task="segmentation", input_width=0,
                             bbox_order="xyz", postprocess="host", inference_rate_hz=0.0,
                             labels="not-a-map", permitted_classes=())
        with self.assertRaises(ValidationError) as caught:
            item.validate()
        text = str(caught.exception)
        for fragment in ("model_id", "not supported by Vision 1.0", "input_width",
                         "bbox_order", "label map", "permitted_classes"):
            self.assertIn(fragment, text)

    def test_a_row_layout_that_names_one_column_twice_is_refused(self):
        # Two fields reading the same column never crashes: it produces a model whose scores
        # are class ids and whose boxes start one coordinate early — plausible-looking output
        # with every number slightly wrong, which is the worst kind to notice downstream.
        with self.assertRaises(ValidationError) as caught:
            manifest(class_index=2, box_index=2).validate()
        self.assertIn("distinct columns", str(caught.exception))
        with self.assertRaises(ValidationError):
            manifest(score_index=-1).validate()
        # A declared tensor narrower than the box it is supposed to carry is also refused.
        with self.assertRaises(ValidationError):
            manifest(row_columns=5).validate()
        # Declaring nothing is allowed: the adapter checks the real width at runtime.
        self.assertEqual(manifest(row_columns=6).validate().row_width, 6)

    def test_geometry_carries_the_declared_conventions(self):
        item = manifest(bbox_order="cxcywh", bbox_normalized=False,
                        preserve_aspect_ratio=False)
        geometry = item.geometry(1920, 1080, roi=(0, 0, 960, 540))
        self.assertEqual(geometry.bbox_order, "cxcywh")
        self.assertFalse(geometry.bbox_normalized)
        self.assertEqual(geometry.roi, (0, 0, 960, 540))

    def test_a_manifest_without_input_dimensions_cannot_map_anything(self):
        with self.assertRaises(ValidationError):
            ModelManifest(model_id="x").geometry(1920, 1080)


class TestDisagreement(unittest.TestCase):
    def test_an_agreeing_model_produces_no_findings(self):
        item = manifest()
        self.assertEqual(item.disagreements_with_intrinsics(dict(GOOD_INTRINSICS)), [])

    def test_every_fatal_convention_difference_is_named(self):
        item = manifest()
        runtime = {"task": "pose estimation", "bbox_order": "yxyx",
                   "bbox_normalization": False, "preserve_aspect_ratio": False,
                   "input_size": [320, 320], "inference_rate": 16.0, "labels": list(COCO)}
        found = {item_.field: item_ for item_ in
                 item.disagreements_with_intrinsics(runtime)}
        self.assertEqual(set(found), {"task", "bbox_order", "bbox_normalized",
                                      "preserve_aspect_ratio", "input_size"})
        self.assertTrue(all(item_.fatal for item_ in found.values()))
        self.assertIn("640", str(found["bbox_normalized"]))

    def test_the_declared_rate_may_differ_but_not_by_threefold(self):
        item = manifest(inference_rate_hz=16.0)
        self.assertEqual(item.disagreements_with_intrinsics(
            {"inference_rate": 17.5}), [])
        found = item.disagreements_with_intrinsics({"inference_rate": 30.0})
        self.assertEqual([f.field for f in found], ["inference_rate_hz"])
        self.assertFalse(found[0].fatal)          # nominal capability vs measured reality

    def test_a_label_list_that_shifts_person_is_fatal_but_a_tail_change_is_not(self):
        item = manifest()
        shifted = ["background", "person"] + list(COCO[2:])
        self.assertTrue(item.disagreements_with_intrinsics({"labels": shifted})[0].fatal)
        tail = list(COCO[:-1]) + ["tooth brush"]
        found = item.disagreements_with_intrinsics({"labels": tail})
        self.assertEqual(len(found), 1)
        self.assertFalse(found[0].fatal)
        self.assertIn("index 79", str(found[0]))   # actionable: where they part

    def test_require_agreement_raises_listing_every_fatal_item(self):
        item = manifest()
        with self.assertRaises(ValidationError) as caught:
            item.require_agreement({"bbox_order": "cxcywh", "inference_rate": 30.0})
        text = str(caught.exception)
        self.assertIn("bbox_order", text)
        self.assertIn("refuse to run", text)
        # Warnings survive the call rather than being printed by it (§40's owner reports them).
        self.assertEqual([f.field for f in item.require_agreement({"inference_rate": 40.0})],
                         ["inference_rate_hz"])

    def test_commissioning_gaps_are_the_work_items_in_the_artefact(self):
        item = manifest(sha256="COMMISSION", license="COMMISSION", path="")
        self.assertEqual(len(item.commissioning_gaps()), 3)
        self.assertEqual(manifest(sha256="ab" * 32, license="Apache-2.0",
                                  path="/tmp/a.rpk").commissioning_gaps(), [])


class TestLabelMap(unittest.TestCase):
    def test_a_missing_index_is_not_invented_as_a_name(self):
        mapping = LabelMap(["person"])
        self.assertIsNone(mapping.name(3))
        self.assertEqual(mapping.name_or_unknown(3), "")
        with self.assertRaises(IndexError):
            mapping[3]

    def test_a_mapping_form_fills_the_gaps_it_declares(self):
        self.assertEqual(resolve({0: "person", 2: "dog"}).names, ("person", "", "dog"))
        self.assertEqual(resolve("coco").source, "coco")
        self.assertEqual(resolve(["a", "b"]).ids_for(["B"]), (1,))
        with self.assertRaises(ValueError):
            resolve("coco-plus")
        with self.assertRaises(ValueError):
            resolve(None)

    def test_membership_is_case_insensitive_like_the_filter(self):
        self.assertIn("PERSON", LabelMap(list(COCO)))
        self.assertNotIn("dog person", LabelMap(list(COCO)))


class TestEnvironmentManifest(unittest.TestCase):
    def test_the_station_is_described_not_assumed(self):
        record = EnvironmentManifest.collect(model_path="/proc/self/cmdline",
                                             model_id="m", task="object_detection",
                                             labels=list(COCO))
        self.assertTrue(record.os and record.python and record.kernel)
        self.assertEqual(record.machine, os.uname().machine)
        for package in EnvironmentManifest.collect().missing_required_packages():
            self.assertIn(package, ("python3-picamera2", "imx500-all", "imx500-firmware",
                                    "imx500-models", "imx500-tools",
                                    "rpicam-apps-imx500-postprocess"))
        self.assertIsNotNone(record.model["sha256"])
        self.assertIsNotNone(record.model["labels_hash"])

    def test_a_hash_is_stable_and_an_unreadable_model_is_recorded_as_unknown(self):
        with tempfile.TemporaryDirectory() as folder:
            path = os.path.join(folder, "a.rpk")
            with open(path, "wb") as handle:
                handle.write(b"network bytes")
            self.assertEqual(sha256_file(path), sha256_file(path))
            self.assertNotEqual(sha256_file(path), sha256_file(path + ".missing"))
        self.assertIsNone(sha256_file("/definitely/not/here.rpk"))
        self.assertEqual(labels_hash(["person", "dog"]), labels_hash(("person", "dog")))
        self.assertIsNone(labels_hash(None))

    def test_yaml_quoting_survives_a_version_with_a_colon(self):
        # dpkg reports epoch-bearing versions ("1:1.0.0-1"); unquoted, that is a mapping.
        record = EnvironmentManifest(os="Linux", python="3.13", kernel="6", hostname="h",
                                     machine="aarch64", collected_at_ns=1,
                                     packages={"imx500-models": "1:1.0.0-1"},
                                     model={"path": "a:b.rpk"})
        text = record.to_yaml()
        self.assertIn('imx500-models: "1:1.0.0-1"', text)
        self.assertIn('path: "a:b.rpk"', text)

    def test_the_manifest_is_written_whole_or_not_at_all(self):
        record = EnvironmentManifest.collect(model_id="m")
        with tempfile.TemporaryDirectory() as folder:
            for name in ("env.json", "env.yaml"):
                path = record.write(os.path.join(folder, name))
                self.assertTrue(os.path.exists(path))
                self.assertGreater(os.path.getsize(path), 40)
            path = record.write(os.path.join(folder, "env2.json"))
            with open(path, encoding="utf-8") as handle:
                payload = json.load(handle)
        self.assertIn("packages", payload)
        self.assertIn("os", payload)


class TestProbe(unittest.TestCase):
    def test_the_probe_reads_what_the_device_declares(self):
        device = FakeDevice()
        result = probe_model("/tmp/a.rpk", model_id="m",
                             imx500_factory=lambda path: device, metadata={"seq": 1})
        self.assertTrue(result.usable, result.findings)
        self.assertEqual(result.task, "object_detection")
        self.assertEqual(result.inference_rate_hz, 16.0)
        self.assertEqual((result.input_width, result.input_height), (640, 640))
        self.assertEqual((result.row_count, result.row_width), (1, 6))
        self.assertEqual(result.output_shapes, ((1, 4, 6),))
        self.assertTrue(result.usable)
        self.assertEqual(result.intrinsics()["task"], "object detection")

    def test_appending_d_asserts_become_findings_reported_together(self):
        result = probe_model("/tmp/b.rpk", imx500_factory=lambda path: FakeDevice(
            {"task": "semantic segmentation", "inference_rate": None, "labels": None}))
        joined = "\n".join(result.findings)
        self.assertIn("segmentation", joined)
        self.assertIn("inference_rate", joined)
        self.assertIn("labels", joined)
        self.assertFalse(result.usable)

    def test_a_device_with_no_intrinsics_is_reported_not_raised_through(self):
        class Silent(FakeDevice):
            network_intrinsics = None

            def __init__(self):
                self.camera_num = 0

        result = probe_model("/tmp/c.rpk", imx500_factory=lambda path: Silent())
        self.assertTrue(result.probed)
        self.assertIn("network_intrinsics is None", result.findings[0])

    def test_a_device_that_cannot_be_opened_records_the_reason(self):
        def factory(path):
            raise RuntimeError("firmware upload failed")
        result = probe_model("/tmp/d.rpk", imx500_factory=factory)
        self.assertFalse(result.probed)
        self.assertIn("firmware upload failed", result.error)

    def test_the_probe_can_draft_a_manifest_from_what_it_measured(self):
        """§9.3's only honest direction of writing: measurement → file → human.

        The draft is asserted to be *incomplete on purpose*: a probe cannot see column order, so
        a tool that filled ``score_index`` in would be guessing with a timestamp attached, and the
        next person would have no way to tell the measured fields from the invented ones.
        """
        import hashlib

        from perception.model.label_maps import COCO_RPK

        with tempfile.TemporaryDirectory() as folder:
            artefact = os.path.join(folder, "candidate.rpk")
            with open(artefact, "wb") as handle:
                handle.write(b"not-a-real-rpk-but-a-real-hash")
            device = FakeDevice({"task": "object detection", "inference_rate": 26,
                                 "bbox_order": "yxyx", "bbox_normalization": True,
                                 "labels": list(COCO_RPK)})
            result = probe_model(artefact, model_id="candidate",
                                 imx500_factory=lambda path: device, metadata={"seq": 1})
            self.assertTrue(result.usable, result.findings)
            draft = result.to_manifest_document(model_id="candidate")

            self.assertEqual(draft["sha256"], hashlib.sha256(
                b"not-a-real-rpk-but-a-real-hash").hexdigest())
            self.assertEqual(draft["labels"], "coco_rpk",
                             "the measured 90-entry list matches the built-in map, so the draft "
                             "should name the map rather than paste 90 strings")
            self.assertEqual(draft["bbox_order"], "yxyx")
            self.assertEqual(draft["input_width"], 640)
            self.assertEqual(draft["inference_rate_hz"], 26.0)
            self.assertEqual(draft["license"], "COMMISSION")
            self.assertEqual(set(draft["row_layout"]),
                             {"score_index", "class_index", "box_index"})
            self.assertEqual(set(draft["row_layout"].values()), {"COMMISSION"})
            joined = "\n".join(draft["_needs"])
            self.assertIn("license", joined)
            self.assertIn("row_layout", joined)
            # A draft must not be loadable as it stands. If it were, `--emit-manifest` would be a
            # way to install an unfinished manifest by accident, and the field that the probe
            # could not see — column order — would silently parse boxes out of scores.
            with self.assertRaises(ValidationError) as caught:
                ModelManifest.from_dict(draft)
            self.assertIn("row_layout", str(caught.exception))
            self.assertIn("postprocess", draft["_provisional"],
                          "this device declared no postprocess mode, and the draft has to say "
                          "which of its fields are unmeasured rather than quietly defaulting")
            finished = dict(draft)
            finished["row_layout"] = {"score_index": 0, "class_index": 1, "box_index": 2}
            finished["license"] = "Apache-2.0"
            finished["postprocess"] = "on_sensor"
            ModelManifest.from_dict(finished).validate()

    def test_a_draft_from_a_model_that_never_opened_names_nothing(self):
        with tempfile.TemporaryDirectory() as folder:
            missing = os.path.join(folder, "absent.rpk")
            result = probe_model(missing, imx500_factory=lambda path: FakeDevice())
            # The file does not exist, so there is no hash to record: an empty string, not
            # zeros — a zero hash would match nothing and read like a measurement.
            self.assertEqual(result.sha256(), "")
            self.assertEqual(result.to_manifest_document()["sha256"], "")

    def test_the_off_hardware_default_factory_says_what_is_missing(self):
        # This interpreter has picamera2 but no libcamera; the error must name the remedy.
        result = probe_model("/usr/share/imx500-models/does_not_exist.rpk")
        self.assertIn("libcamera", result.error + " ".join(result.findings))

    def test_admit_returns_warnings_and_raises_on_findings(self):
        item = manifest()
        clean = ProbeResult(probed=True, task="object_detection", inference_rate_hz=30.0,
                            labels=list(COCO), input_width=640, input_height=640,
                            bbox_order="xy", bbox_normalized=True)
        self.assertEqual(admit(item, clean), [
            "warning: inference_rate_hz: manifest says 16.0, model reports 30.0 "
            "(timing budgets were derived from the declared rate)"])
        dirty = ProbeResult(probed=True, task="object_detection", inference_rate_hz=16.0,
                            labels=list(COCO), input_width=640, input_height=640,
                            bbox_order="yxyx", bbox_normalized=True)
        with self.assertRaises(ModelRejected) as caught:
            admit(item, dirty)
        self.assertIn("bbox_order", str(caught.exception))


class TestRowShapes(unittest.TestCase):
    def test_every_shape_the_runtime_has_been_seen_becomes_rows(self):
        from perception.model.compatibility_probe import _as_rows
        self.assertEqual(_as_rows([[1.0, 0.0, 1, 2, 3, 4]]), [[1.0, 0.0, 1, 2, 3, 4]])
        self.assertEqual(_as_rows([[[1.0, 0.0, 1, 2, 3, 4]]]), [[1.0, 0.0, 1, 2, 3, 4]])
        self.assertEqual(_as_rows([[1.0, 0.0, 1, 2, 3, 4], [0.5, 0.0, 1, 2, 3, 4]]),
                         [[1.0, 0.0, 1, 2, 3, 4], [0.5, 0.0, 1, 2, 3, 4]])
        self.assertEqual(_as_rows([1.0, 0.0, 1, 2, 3, 4]), [[1.0, 0.0, 1, 2, 3, 4]])
        self.assertIsNone(_as_rows([[1.0, 0.0, 1], [0.5, 0.0, 2]]))          # too short
        self.assertIsNone(_as_rows([[1.0, 0.0, 1, 2, 3], [0.5, 0.0, 1]]))    # ragged
        self.assertEqual(_as_rows([]), [])

    def test_a_single_detection_frame_is_still_rows(self):
        # Shape (1, 6) used to be stripped as a batch dimension, so the only person in view
        # became "unexpected output layout" — a detection lost to a shape guess.
        result = probe_model("/tmp/e.rpk", imx500_factory=lambda path: FakeDevice(
            rows=[[0.9, 0.0, 0.28, 0.35, 0.38, 0.65]]), metadata={})
        self.assertEqual((result.row_count, result.row_width), (1, 6))
        self.assertEqual(result.findings, [])


class TestImx500Adapter(unittest.TestCase):
    def _adapter(self, device=None, **overrides):
        item = manifest(path="/usr/share/imx500-models/fake.rpk", **overrides)
        device = device if device is not None else FakeDevice()
        adapter = Imx500YoloAdapter(item, imx500_factory=lambda path: device)
        adapter.configure_stream(1920, 1080)
        return adapter

    def test_open_checks_the_manifest_before_the_first_inference(self):
        adapter = self._adapter()
        adapter.open()
        self.assertTrue(adapter.opened)
        self.assertEqual(adapter.camera_num, 0)
        self.assertEqual(adapter.warnings, [])

    def test_a_missing_model_path_stops_startup(self):
        adapter = Imx500YoloAdapter(manifest(), imx500_factory=lambda path: FakeDevice())
        with self.assertRaises(ModelRejected) as caught:
            adapter.open()
        self.assertIn("no model path", str(caught.exception))

    def test_a_disagreeing_model_is_refused_and_stays_closed(self):
        device = FakeDevice(dict(GOOD_INTRINSICS, bbox_order="cxcywh"))
        adapter = self._adapter(device)
        with self.assertRaises(ModelRejected):
            adapter.open()
        self.assertFalse(adapter.opened)
        self.assertIsNone(adapter.device)

    def test_a_warning_does_not_stop_the_model_from_running(self):
        # 20 Hz against a declared 16 Hz is inside the ±35% tolerance; 30 Hz is not, and the
        # tolerance is the point — the declared figure is nominal, not a measurement.
        device = FakeDevice(dict(GOOD_INTRINSICS, inference_rate=30.0))
        adapter = self._adapter(device)
        adapter.open()
        self.assertTrue(adapter.opened)
        self.assertEqual(len(adapter.warnings), 1)

    def test_the_input_size_the_device_reports_wins_for_the_geometry(self):
        device = FakeDevice(input_size=(320, 320))
        adapter = self._adapter(device)
        adapter.open()
        self.assertEqual((adapter.manifest.input_width, adapter.manifest.input_height),
                         (320, 320))
        self.assertEqual(len(adapter.warnings), 1)
        self.assertIn("input_size", adapter.warnings[0])
        self.assertIn("using the device's value", adapter.warnings[0])

    def test_split_outputs_are_assembled_identically_to_single_tensor_rows(self):
        # The on-sensor _pp firmware emits boxes/scores/classes/count as parallel tensors; the
        # adapter must assemble them into the same rows §13's single-tensor contract uses, so the
        # rest of the subsystem has one behaviour to test, not two that can diverge.
        rows = [[0.9, 0.0, 0.28, 0.35, 0.38, 0.65],
                [0.4, 0.0, 0.50, 0.10, 0.60, 0.90]]
        boxes = [row[2:6] for row in rows]
        scores = [row[0] for row in rows]
        classes = [row[1] for row in rows]
        split_adapter = self._adapter(
            _SplitDevice(boxes, scores, classes, [2.0]),
            output_tensors={"format": "imx500_pp_split", "boxes": 0, "scores": 1,
                            "classes": 2, "count": 3, "max_rows": 100})
        split_adapter.open()
        plain_adapter = self._adapter(_SingleRowsDevice(rows))
        plain_adapter.open()

        split_set = split_adapter.infer(None, {"seq": 1}, frame_sequence=1,
                                        sensor_timestamp_ns=1_000_000_000,
                                        publish_timestamp_ns=1_000_000_000)
        plain_set = plain_adapter.infer(None, {"seq": 1}, frame_sequence=1,
                                        sensor_timestamp_ns=1_000_000_000,
                                        publish_timestamp_ns=1_000_000_000)

        def key(det):
            return (det.bbox.x_min, det.bbox.y_min, det.bbox.x_max, det.bbox.y_max,
                    det.detector_score, det.class_name)

        self.assertEqual([key(d) for d in split_set.detections],
                         [key(d) for d in plain_set.detections],
                         "assembling split tensors must not change the detections")

    def test_the_count_tensor_trims_the_sensor_padding(self):
        # The sensor pads its buffer to max_rows; only the count tensor says how many are real.
        # Reading padded rows would surface as phantom identities (§23), so the count is what
        # caps the set.
        # Boxes are sized like the equivalence test's, i.e. large enough that §36's
        # measurement-quality floor does not reject them — the point here is the count trim,
        # not the minimum-size gate, and mixing the two would make the failure ambiguous.
        device = _SplitDevice([[0.28, 0.35, 0.38, 0.65], [0.40, 0.10, 0.52, 0.45],
                               [0.55, 0.30, 0.68, 0.70]], [0.9, 0.8, 0.7], [0.0, 0.0, 0.0],
                              [1.0])
        adapter = self._adapter(device, output_tensors={"format": "imx500_pp_split", "boxes": 0,
                                                       "scores": 1, "classes": 2, "count": 3,
                                                       "max_rows": 100})
        adapter.open()
        result = adapter.infer(None, {"seq": 1}, frame_sequence=1,
                               sensor_timestamp_ns=1_000_000_000,
                               publish_timestamp_ns=1_000_000_000)
        self.assertEqual(len(result.detections), 1,
                         "only the detection the count tensor vouches for may be kept")
        self.assertAlmostEqual(result.detections[0].detector_score, 0.9)

    def test_a_malformed_split_descriptor_is_refused_not_guessed(self):
        item = manifest(output_tensors={"format": "imx500_pp_split", "boxes": 0, "scores": 1,
                                        "count": 3, "max_rows": 100})
        with self.assertRaises(ValidationError) as caught:
            item.validate()
        self.assertIn("classes", str(caught.exception))

    def test_infer_before_open_or_without_metadata_is_a_counted_failure(self):
        adapter = self._adapter()
        with self.assertRaises(ModelRejected):
            adapter.infer(None, None, frame_sequence=0, sensor_timestamp_ns=1,
                          publish_timestamp_ns=2)
        adapter.open()
        with self.assertRaises(ModelRejected) as caught:
            adapter.infer(None, None, frame_sequence=0, sensor_timestamp_ns=1,
                          publish_timestamp_ns=2)
        self.assertIn("metadata", str(caught.exception))
        self.assertEqual(adapter.failures, 1)

    def test_the_produced_set_carries_the_model_generation(self):
        adapter = self._adapter()
        adapter.open()
        first = adapter.infer(None, {"seq": 1}, frame_sequence=1,
                              sensor_timestamp_ns=1_000_000_000,
                              publish_timestamp_ns=1_100_000_000)
        self.assertEqual(first.model_generation, 1)
        self.assertEqual(first.detections[0].class_name, "person")
        adapter.note_model_generation(2)                     # §33: a reload happened
        second = adapter.infer(None, {"seq": 2}, frame_sequence=2,
                               sensor_timestamp_ns=1_200_000_000,
                               publish_timestamp_ns=1_300_000_000)
        self.assertEqual(second.model_generation, 2)

    def test_an_unexpected_output_layout_is_a_failure_not_an_empty_frame(self):
        device = FakeDevice(rows=[[0.9, 0.0, 0.28], [0.8, 0.0, 0.4]])
        adapter = self._adapter(device)
        adapter.open()
        with self.assertRaises(ModelRejected):
            adapter.infer(None, {"seq": 1}, frame_sequence=1, sensor_timestamp_ns=1,
                          publish_timestamp_ns=2)
        self.assertEqual(adapter.failures, 1)
        self.assertEqual(adapter.inferences, 0)

    def test_no_tensors_is_a_failure_not_an_empty_frame(self):
        class Nothing(FakeDevice):
            def get_outputs(self, metadata, add_batch=False):
                return []
        adapter = self._adapter(Nothing())
        adapter.open()
        with self.assertRaises(ModelRejected):
            adapter.infer(None, {"seq": 1}, frame_sequence=1, sensor_timestamp_ns=1,
                          publish_timestamp_ns=2)


class TestMockAdapter(unittest.TestCase):
    def test_scripted_rows_reach_the_real_normalization_path(self):
        adapter = MockAdapter(manifest(), {0: rows_for_moving_target(2),
                                           1: rows_for_moving_target(1)},
                              default_rows=[[0.5, 0.0, 0.4, 0.35, 0.5, 0.6]])
        adapter.configure_stream(1920, 1080)
        adapter.open()
        for sequence, expected in ((0, 2), (1, 1), (2, 1), (3, 1)):
            out = adapter.infer(None, None, frame_sequence=sequence,
                                sensor_timestamp_ns=sequence * 60_000_000 + 1,
                                publish_timestamp_ns=sequence * 60_000_000 + 2)
            self.assertEqual(len(out.detections), expected, sequence)
        self.assertEqual(adapter.requested_frames, [0, 1, 2, 3])

    def test_a_callable_source_is_asked_per_frame(self):
        adapter = MockAdapter(manifest(), lambda sequence: [[0.9, 0, 0.4, 0.35, 0.5, 0.6]]
                              if sequence % 2 == 0 else [])
        adapter.configure_stream(1280, 720)
        adapter.open()
        counts = [len(adapter.infer(None, None, frame_sequence=index,
                                    sensor_timestamp_ns=index + 1,
                                    publish_timestamp_ns=index + 2).detections)
                  for index in range(4)]
        self.assertEqual(counts, [1, 0, 1, 0])

    def test_the_synthetic_boxes_land_inside_the_letterbox(self):
        # A square input on a 16:9 stream leaves only the vertical middle visible. Fixtures
        # that ignore that are "rejected as out of frame", which reads as a broken tracker.
        adapter = MockAdapter(manifest(), {0: rows_for_moving_target(3)})
        adapter.configure_stream(1920, 1080)
        adapter.open()
        out = adapter.infer(None, None, frame_sequence=0, sensor_timestamp_ns=1,
                            publish_timestamp_ns=2)
        self.assertEqual(len(out.detections), 3)
        self.assertEqual(out.counters.malformed_rejected, 0)
        for detection in out.detections:
            self.assertGreater(detection.bbox.y_min, 0.1)
            self.assertLess(detection.bbox.y_max, 0.9)

    def test_inference_before_the_stream_size_is_known_is_refused(self):
        adapter = MockAdapter(manifest(), {0: []})
        adapter.open()
        with self.assertRaises(ConfigError) as caught:
            adapter.infer(None, None, frame_sequence=0, sensor_timestamp_ns=1,
                          publish_timestamp_ns=2)
        self.assertIn("configure_stream", str(caught.exception))
        with self.assertRaises(ConfigError):
            adapter.configure_stream(0, 1080)


class TestFactory(unittest.TestCase):
    def _config(self) -> VisionConfig:
        from perception.config import CameraConfig
        models = {
            "mock_profile": ModelConfig(profile_name="mock_profile", model_id="mock",
                                        adapter="mock",
                                        thresholds=ScoreThresholds(0.1, 0.3, 0.5, 0.5)),
            "imx_profile": ModelConfig(profile_name="imx_profile", model_id="imx",
                                       adapter="imx500", path="/tmp/a.rpk",
                                       thresholds=ScoreThresholds(0.1, 0.3, 0.5, 0.5)),
        }
        return VisionConfig(profile="mock_profile", models=models,
                            camera=CameraConfig())

    def test_the_profile_names_the_adapter_and_a_typo_stops_startup(self):
        config = self._config()
        self.assertIsInstance(build_adapter(config), MockAdapter)
        self.assertIsInstance(build_adapter(config, profile="imx_profile"),
                              Imx500YoloAdapter)
        config.models["typo"] = ModelConfig(profile_name="typo", adapter="yclo")
        config.profile = "typo"
        with self.assertRaises(ConfigError) as caught:
            build_adapter(config)
        self.assertIn("yclo", str(caught.exception))

    def test_the_manifest_file_wins_and_the_config_overlays_what_it_declares(self):
        with tempfile.TemporaryDirectory() as folder:
            path = os.path.join(folder, "m.json")
            with open(path, "w", encoding="utf-8") as handle:
                json.dump({"model_id": "from-file", "input_width": 416,
                           "input_height": 416, "inference_rate_hz": 24.0,
                           "labels": "coco", "path": "/from/file.rpk",
                           "sha256": "ff" * 32}, handle)
            config = self._config()
            config.models["mock_profile"].manifest_path = path
            config.models["mock_profile"].input_width = 640          # a stale default
            config.models["mock_profile"].permitted_classes = ("person", "dog")
            config.models["mock_profile"].path = "/relocated/on/this/station.rpk"
            item = manifest_for(config)
            self.assertEqual(item.model_id, "from-file")
            self.assertEqual(item.inference_rate_hz, 24.0)
            # The artefact is authoritative about the artefact: a config default that was
            # never written down must not silently rewrite a reviewed manifest (§9.3).
            self.assertEqual(item.input_width, 416)
            self.assertIn("input_width=640 ignored", item.notes)
            # What the config legitimately owns: where the file lives, which classes pass.
            self.assertEqual(item.path, "/relocated/on/this/station.rpk")
            self.assertEqual(item.permitted_classes, ("person", "dog"))
            self.assertEqual(item.sha256, "ff" * 32)

    def test_a_shipped_artefact_is_found_without_depending_on_the_working_directory(self):
        relative = "model/manifests/imx500_yolo11n_pp_coco.json"
        previous = os.getcwd()
        try:
            os.chdir(tempfile.gettempdir())
            resolved = resolve_artifact(relative)
        finally:
            os.chdir(previous)
        self.assertTrue(os.path.isabs(resolved))
        self.assertTrue(os.path.exists(resolved))
        self.assertEqual(resolve_artifact("model/manifests/not-there.json"),
                         "model/manifests/not-there.json")
        self.assertEqual(resolve_artifact(""), "")

    def test_the_shipped_config_and_manifests_are_coherent(self):
        config = VisionConfig.from_file(os.path.join(PACKAGE_ROOT, "configs",
                                                     "perception_v1.json"))
        self.assertEqual(sorted(config.models),
                         ["bakeoff_efficientdet320", "bakeoff_nanodet416", "offline_replay",
                          "person_detect", "person_detect_available"])
        for name in config.models:
            item = manifest_for(config, name)
            item.validate()
            # Labels must name a real map: a manifest whose label list cannot be resolved turns
            # every class id into an unknown class, and §15's filter would then reject a person
            # it was configured to permit.
            self.assertTrue(len(item.label_map()), name)
        self.assertEqual(config.validate(production=False), [])

    def test_a_manifest_naming_an_installed_artefact_agrees_with_the_file(self):
        """§9.3, checked against the station's own disk rather than against prose.

        The three ``_pp`` detectors are measured from their .rpk, not transcribed. If the package
        is upgraded, this test fails — which is the same outcome the runtime probe produces at
        start-up, just earlier and with a traceback that names the file.
        """
        import hashlib

        config = VisionConfig.from_file(os.path.join(PACKAGE_ROOT, "configs",
                                                     "perception_v1.json"))
        measured = 0
        for name in config.models:
            item = manifest_for(config, name)
            if not item.path or not os.path.isfile(item.path):
                continue                     # offline dev box, or a model not installed here
            measured += 1
            digest = hashlib.sha256()
            with open(item.path, "rb") as handle:
                for block in iter(lambda: handle.read(1 << 20), b""):
                    digest.update(block)
            self.assertEqual(item.sha256, digest.hexdigest(),
                             f"{name}: {item.path} does not match its manifest's sha256")
            self.assertNotEqual(item.license, "COMMISSION",
                                f"{name}: the artefact's licence is a work item, and §50 is "
                                f"right to refuse it")
            self.assertFalse([gap for gap in item.commissioning_gaps()
                              if "sha256" in gap or "license" in gap],
                             f"{name}: installed artefact still carries artifact work items")
        self.assertGreaterEqual(measured, 1,
                                "no installed artefact was checked; this station should have at "
                                "least one _pp detector under /usr/share/imx500-models")

    def test_no_shipped_profile_may_start_in_production_yet(self):
        """§50's guard, phrased so it survives commissioning the manifests.

        Asserting "the manifests have gaps" would flip to a failure the moment a hash and a
        licence are recorded — and the obvious way to silence it would be to delete the test.
        What must actually stay true today is that no profile can start a production run: the
        score bands and §16's numbers are still unset.
        """
        config = VisionConfig.from_file(os.path.join(PACKAGE_ROOT, "configs",
                                                     "perception_v1.json"))
        # §50 stays alive for the profile we could not commission (yolo11n is not installed),
        # and the live profiles were provisionally commissioned for the tracking loop.
        config.profile = "person_detect"
        with self.assertRaises(ConfigPlaceholderError) as caught:
            config.validate(production=True)
        self.assertIn("COMMISSION", str(caught.exception))
        for name in ("person_detect_available", "bakeoff_nanodet416", "bakeoff_efficientdet320"):
            config.profile = name
            self.assertEqual(config.validate(production=False), [], name)


if __name__ == "__main__":                                   # pragma: no cover
    unittest.main()
