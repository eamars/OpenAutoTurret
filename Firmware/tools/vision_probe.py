#!/usr/bin/env python3
"""Camera readiness probe (system python3 ONLY — picamera2 lives there).

Run with the sensor free (stop webd first). Does two things:
  stream : configure 1920x1080 preview, start/stop — proves the IMX500 path.
  survey : dump the detection-related API surface actually installed
           (detect_objects attr, postprocessors dir, dnf/hailo SDKs).
Docs: docs/research_vision_readiness_p7.md
"""
import sys, time

def stream():
    from picamera2 import Picamera2
    import math
    p = Picamera2()
    print("model:", p.camera_properties.get("Model", "?"))
    p.configure(p.create_preview_configuration(
        main={"size": (1920, 1080), "format": "XRGB8888"}))
    p.start(); time.sleep(1.0)
    req = p.capture_request(); arr = req.make_array("main")
    print("frame:", arr.shape, "lum:", int(arr.mean()))
    req.release(); p.stop(); p.close()

def survey():
    from importlib.metadata import version
    import picamera2, os
    print("picamera2", version("picamera2"))
    from picamera2 import Picamera2
    print("detect attrs:", [m for m in dir(Picamera2)
          if any(k in m.lower() for k in ("object", "detect", "ai", "rpk"))] or "NONE")
    pp = os.path.join(os.path.dirname(picamera2.__file__), "postprocessing")
    print("postprocessors:", sorted(os.listdir(pp)) if os.path.isdir(pp) else "none")
    for mod in ("dnf", "hailo", "hilortc"):
        try:
            __import__(mod); print(mod, "available")
        except Exception:
            print(mod, "MISSING")

if __name__ == "__main__":
    what = sys.argv[1] if len(sys.argv) > 1 else "all"
    if what in ("all", "survey"): survey()
    if what in ("all", "stream"): stream()
