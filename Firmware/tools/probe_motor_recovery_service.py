"""HTTP recovery/cancel/re-home probe using real services and simulated motors.

No camera or physical motor is opened. Simulator success does not verify the
CyberGear fault-clear behavior; probe_motor_recovery.cpp covers UART framing.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request

import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controld", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--port", type=int, default=8811)
    parser.add_argument("--inspect-seconds", type=int, default=0)
    args = parser.parse_args()
    firmware = Path(__file__).resolve().parents[1]
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    config = yaml.safe_load((firmware / "config/turret.yaml").read_text())
    config["alignment"]["mode"] = "off"
    config["v3"]["default_mode"] = "MANUAL"
    config["homing"]["contact"].update(coarse_speed_deg_s=60, fine_speed_deg_s=20,
                                          contact_dwell_ms=150)
    config["homing_plan"] = [{"action": "home_full_range", "axis": axis}
                             for axis in ("pitch", "yaw")]
    config["payload"]["auto_verify"] = False
    config_path = output / "simulator.yaml"
    config_path.write_text(yaml.safe_dump(config))
    with tempfile.TemporaryDirectory(prefix="ota-recovery-") as sockets:
        env = dict(os.environ, OTA_WEB_SOCKET=str(Path(sockets) / "control.sock"),
                   OTA_VISION_SOCKET=str(Path(sockets) / "vision.sock"),
                   OTA_WEB_HOST="127.0.0.1", OTA_WEB_PORT=str(args.port),
                   OTA_VIDEO_ENABLE="0", PYTHONPATH=str(firmware))
        base = f"http://127.0.0.1:{args.port}"

        def api(path, body=None):
            req = urllib.request.Request(base + path,
                data=None if body is None else json.dumps(body).encode(),
                headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(req, timeout=2) as response:
                return json.load(response)

        def command(name):
            return api("/api/command", {"command": name, "arg": ""})

        with (output / "controller.log").open("w") as cl, (output / "web.log").open("w") as wl:
            controller = subprocess.Popen([str(args.controld.resolve()), str(config_path), "--sim"],
                                          cwd=firmware, env=env, stdout=cl, stderr=cl)
            web = subprocess.Popen([sys.executable, "-m", "web.webd.app"], cwd=firmware,
                                   env=env, stdout=wl, stderr=wl)
            states = []

            def wait_phase(phase, timeout=60):
                until = time.monotonic() + timeout
                while time.monotonic() < until:
                    if controller.poll() is not None or web.poll() is not None:
                        raise RuntimeError("service exited; inspect logs")
                    try:
                        state = api("/api/state")
                        if state.get("phase") == phase:
                            states.append(state)
                            return state
                    except (OSError, urllib.error.HTTPError):
                        pass
                    time.sleep(.05)
                raise TimeoutError(f"phase {phase} not reached")

            try:
                wait_phase("homing")
                assert command("stop_motion")["ok"]
                wait_phase("idle")
                assert command("recover_motors")["ok"]
                wait_phase("recovering", 5)
                assert not command("start_homing")["ok"]
                assert not command("recover_motors")["ok"]
                assert command("stop_motion")["ok"]
                cancelled = wait_phase("fault", 5)
                assert "cancelled" in cancelled["fault"]
                assert command("recover_motors")["ok"]
                wait_phase("recovering", 5)
                recovered = wait_phase("idle", 5)
                assert not recovered["fault"] and not recovered["soft_limits_valid"]
                assert recovered["operating_mode"] == "MANUAL"
                assert command("start_homing")["ok"]
                wait_phase("homing", 5)
                homed = wait_phase("hold", 120)
                assert not homed["fault"] and homed["soft_limits_valid"]
                assert api("/api/health")["controld_connected"]
                (output / "states.json").write_text(json.dumps(states, indent=2))
                print("PASS: HTTP recovery, rejection during recovery, cancellation, retry, and re-homing; services alive", flush=True)
                if args.inspect_seconds:
                    # Provide a disabled Fault state for browser validation.
                    assert command("start_homing")["ok"]
                    wait_phase("homing", 5)
                    assert command("stop_motion")["ok"]
                    wait_phase("idle", 5)
                    assert command("recover_motors")["ok"]
                    wait_phase("recovering", 5)
                    assert command("stop_motion")["ok"]
                    wait_phase("fault", 5)
                    print(f"Browser inspection: {base} for {args.inspect_seconds} seconds", flush=True)
                    time.sleep(args.inspect_seconds)
            finally:
                controller.terminate()
                controller.wait(timeout=30)
                web.terminate()
                web.wait(timeout=5)


if __name__ == "__main__":
    main()
