"""Exercise real controld (--sim) and webd processes through control APIs.

No camera, video request, browser, or hardware backend. Fast homing overrides
exist only in the disposable simulator configuration, never in deployment.
"""
import argparse
import json
import os
from pathlib import Path
import socket
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
    args = parser.parse_args()
    firmware = Path(__file__).resolve().parents[1]
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    config = yaml.safe_load((firmware / "config/turret.yaml").read_text())
    # PyYAML's YAML 1.1 resolver reads the unquoted enum "off" as False.
    config["alignment"]["mode"] = "off"
    config["v3"]["default_mode"] = "MANUAL"
    config["homing"]["contact"].update(coarse_speed_deg_s=60,
        fine_speed_deg_s=20, contact_dwell_ms=150)
    config["homing_plan"] = [
        {"action": "home_full_range", "axis": "pitch"},
        {"action": "home_full_range", "axis": "yaw"}]
    config["payload"]["auto_verify"] = False
    config_path = output / "simulator.yaml"
    config_path.write_text(yaml.safe_dump(config))
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    env = os.environ.copy()
    # UDS cannot bind on WSL's Windows-mounted filesystem. Keep only socket
    # paths on native tmpfs; logs/config remain in the requested output folder.
    socket_dir = tempfile.TemporaryDirectory(prefix="ota-park-api-")
    env.update(OTA_WEB_SOCKET=str(Path(socket_dir.name) / "controller.sock"),
               OTA_VISION_SOCKET=str(Path(socket_dir.name) / "vision.sock"),
               OTA_WEB_HOST="127.0.0.1", OTA_WEB_PORT=str(port),
               OTA_VIDEO_ENABLE="0", PYTHONPATH=str(firmware))
    base = f"http://127.0.0.1:{port}"

    def api(path, body=None):
        request = urllib.request.Request(base + path,
            data=None if body is None else json.dumps(body).encode(),
            headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(request, timeout=2) as response:
            return json.load(response)

    def command(name, arg=""):
        return api("/api/command", {"command": name, "arg": arg})

    def wait_for(predicate, timeout=180):
        until = time.monotonic() + timeout
        latest = {}
        while time.monotonic() < until:
            if controller.poll() is not None or web.poll() is not None:
                raise RuntimeError("service exited; inspect probe logs")
            try:
                latest = api("/api/state")
                if predicate(latest):
                    return latest
            except (OSError, urllib.error.HTTPError):
                pass
            time.sleep(.1)
        raise TimeoutError(f"state deadline: phase={latest.get('phase')} fault={latest.get('fault')}")

    with (output / "controller.log").open("w") as cl, (output / "web.log").open("w") as wl:
        controller = subprocess.Popen([str(args.controld.resolve()), str(config_path), "--sim"],
                                      cwd=firmware, env=env, stdout=cl, stderr=cl)
        web = subprocess.Popen([sys.executable, "-m", "web.webd.app"],
                               cwd=firmware, env=env, stdout=wl, stderr=wl)
        try:
            for expect_success in (True, False):
                state = wait_for(lambda s: s.get("phase") == "hold" and s.get("at_ready"))
                time.sleep(3)
                if expect_success:
                    before = api("/api/state")["q_yaw_rad"]
                    assert command("manual_step", "yaw+5")["ok"]
                    wait_for(lambda s: s.get("cmd_ack_command") == "manual_step" and
                             s.get("cmd_ack_accepted") == 1, timeout=5)
                    wait_for(lambda s: abs(s["q_yaw_rad"]-before) > .06, timeout=15)
                    time.sleep(3)
                assert command("request_shutdown")["ok"]
                wait_for(lambda s: s.get("phase") == "parking", timeout=5)
                rejected = command("start_homing")
                assert not rejected["ok"], rejected
                state = wait_for(lambda s: s.get("phase") in ("parked", "fault"), timeout=90)
                assert state["phase"] == ("parked" if expect_success else "fault"), state.get("fault")
                assert api("/api/health")["controld_connected"]
                assert controller.poll() is None and web.poll() is None
                time.sleep(1)
                assert command("start_homing")["ok"]
                wait_for(lambda s: s.get("phase") == "homing", timeout=5)
                print(f"PASS: {state['phase']} keeps both services alive; Home rejected during parking and accepted afterward", flush=True)
        finally:
            controller.terminate()
            controller.wait(timeout=30)
            web.terminate()
            web.wait(timeout=5)
    print(f"Evidence: {output}")


if __name__ == "__main__":
    main()
