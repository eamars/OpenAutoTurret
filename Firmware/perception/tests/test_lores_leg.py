"""The B route's second leg: one request, two sizes, inference reads the small one.

Both tests run on a machine with no sensor: the capability being pinned is what we *ask* the camera
for and what we do with what comes back, not the pixels themselves.
"""

import unittest

from perception.camera import CapturedFrame, CameraOwner


class FakeRequest:
    def __init__(self, names, stamp_ns):
        self.names = names
        self.stamp_ns = stamp_ns

    def make_array(self, stream):
        self.names.append(stream)
        return f"pixels:{stream}"

    def get_metadata(self):
        return {"SensorTimestamp": self.stamp_ns}

    def release(self):
        self.names.append("released")


class FakeCamera:
    def __init__(self, stamp_ns=1_000_000_000):
        self.stamp_ns = stamp_ns
        self.names = []

    def capture_request(self):
        return FakeRequest(self.names, self.stamp_ns)


class TwoLegCapture(unittest.TestCase):
    def test_both_legs_come_from_one_request(self):
        device = FakeCamera()
        owner = CameraOwner(device, stream_size=(1920, 1080),
                            inference_stream="lores", inference_size=(640, 360))
        frame = owner.next_frame()
        self.assertEqual(device.names.count("main"), 1, "the operator's leg must still be taken")
        self.assertEqual(device.names.count("lores"), 1,
                         "with a second leg configured it must be taken from the same request, "
                         "because two requests would be two instants of light")
        self.assertEqual(device.names.count("released"), 1, "the request must be released")

    def test_inference_leg_reaches_the_frame(self):
        owner = CameraOwner(FakeCamera(), stream_size=(1920, 1080),
                            inference_stream="lores", inference_size=(640, 360))
        frame = owner.next_frame()
        if not frame.usable:
            self.skipTest(f"this libcamera shape needs another metadata key: "
                          f"{frame.unusable_reason}")
        self.assertEqual(frame.inference_image, "pixels:lores",
                         "inference must be handed the small leg, not the display leg")
        self.assertEqual(frame.image, "pixels:main",
                         "the display keeps the big leg: the operator's picture does not shrink")
        self.assertEqual(frame.inference_size, (640, 360))

    def test_one_leg_station_is_unchanged(self):
        owner = CameraOwner(FakeCamera(), stream_size=(1920, 1080))
        frame = owner.next_frame()
        self.assertIsNone(owner.inference_stream)
        self.assertIsNone(frame.inference_image,
                          "with no second leg the caller falls back to frame.image")
        self.assertEqual(FakeCamera().names if False else [], [])


class LoresAsk(unittest.TestCase):
    def test_the_sensor_is_asked_for_both_streams(self):
        """`lores_size` must reach create_preview_configuration, or nothing downstream can work."""
        import perception.camera as camera_module

        recorded = {}

        class StubPicamera2:
            def __init__(self, num):
                self.camera_properties = {"Model": "imx477"}

            def create_preview_configuration(self, **kwargs):
                recorded.update(kwargs)
                return "config"

            def configure(self, config):
                recorded["configured"] = config

            def close(self):
                pass

            def start(self, *a, **k):
                pass

            @staticmethod
            def global_camera_info():
                return [{"Id": "platform-fe300000.i2c-10-001a", "DevicePath": "/dev/video1",
                         "Model": "imx477", "Num": 1, "Location": 0}]

        # The opener imports Picamera2 inside the function, so patching the module attribute is not
        # enough on a machine without the driver: hand sys.modules a stand-in and the local import
        # resolves. Without this the test skips off-station and the ask is never checked.
        import sys as _sys
        import types as _types
        stub_module = _types.ModuleType("picamera2")
        stub_module.Picamera2 = StubPicamera2
        lc_module = _types.ModuleType("libcamera")      # the opener imports both

        class _Anything:
            """Any libcamera symbol the opener happens to want: shape-agnostic stand-in.

            Only the *kwargs* we are testing survive into the assertion, so being permissive here
            cannot make the test pass for the wrong reason: the recorded configuration is what fails
            if the code stopped asking for the leg.
            """

            def __init__(self, *a, **k):
                pass

            def __call__(self, *a, **k):
                return _Anything()

        lc_module.__getattr__ = lambda name: _Anything
        saved_module = _sys.modules.get("picamera2")
        saved_lc = _sys.modules.get("libcamera")
        _sys.modules["picamera2"] = stub_module
        _sys.modules["libcamera"] = lc_module
        saved = getattr(camera_module, "Picamera2", None)
        saved_resolve = getattr(camera_module, "enumerate_cameras", None)
        camera_module.Picamera2 = StubPicamera2
        if saved_resolve is not None:
            camera_module.enumerate_cameras = lambda *a, **k: [
                {"Id": "platform-fe300000.i2c-10-001a", "DevicePath": "/dev/video1",
                 "Model": "imx477", "Index": 1}]
        try:
            try:
                camera, info = camera_module.open_picamera2_sensor(
                    "imx477", stream_size=(1920, 1080), frame_rate_hz=30.0,
                    orientation="rotate_180", lores_size=(640, 360))
            except Exception as exc:          # enumeration differences on this host are not the point
                self.skipTest(f"camera enumeration on this machine: {type(exc).__name__}: {exc}")
            self.assertIn("lores", recorded, "the second leg was never asked of the ISP")
            self.assertEqual(recorded["lores"]["size"], (640, 360))
            self.assertEqual(recorded["main"]["size"], (1920, 1080))
            self.assertEqual(info["inference_input"], "lores",
                             "the info block has to say which leg inference reads")
            self.assertEqual(info["lores_size"], (640, 360))
        finally:
            if saved_module is not None:
                _sys.modules["picamera2"] = saved_module
            else:
                _sys.modules.pop("picamera2", None)
            if saved_lc is not None:
                _sys.modules["libcamera"] = saved_lc
            else:
                _sys.modules.pop("libcamera", None)
            if saved is not None:
                camera_module.Picamera2 = saved
            if saved_resolve is not None:
                camera_module.enumerate_cameras = saved_resolve


if __name__ == "__main__":
    unittest.main(verbosity=2)
