"""Read-only checks for the supported station launcher; never opens camera/CAN."""
from pathlib import Path
import importlib
import os
import sys

def main():
    import yaml
    firmware = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(firmware))
    config_path = Path(sys.argv[1]).resolve()
    mode = sys.argv[2]
    config = yaml.safe_load(config_path.read_text())
    if mode == "imu":
        if not os.access("/dev/i2c-1", os.R_OK | os.W_OK):
            raise RuntimeError("BNO085 requires unprivileged read/write access to /dev/i2c-1")
        if not os.access(firmware / "build/imu-bno085", os.X_OK):
            raise RuntimeError("IMU executable missing; deploy --probe-build --probe-imu")
        print("Preflight: BNO085 capture only; no motor or camera process")
        return
    if mode == "commission":
        probe_config = Path(os.environ.get("OTA_HARDWARE_PROBE_CONFIG", firmware / "config/hardware_probe.yaml"))
        probe = yaml.safe_load(probe_config.read_text())
        if probe.get("schema_version") != 1:
            raise RuntimeError("Unsupported commissioning schema")
        if probe["yaw"]["interface"] == probe["pitch"]["interface"]:
            raise RuntimeError("Commissioning requires two independent CAN interfaces")
        for axis in ("yaw", "pitch"):
            interface = probe[axis]["interface"]
            parent = Path("/sys/class/net") / interface / "device"
            if not parent.exists() or parent.resolve().name != probe[axis]["spi_parent"]:
                raise RuntimeError(f"{axis}: interface/SPI mapping mismatch: {interface}")
        binary = firmware / "build/probe-mixed-hardware"
        if not binary.is_file() or not os.access(binary, os.X_OK):
            raise RuntimeError("Commissioning probe missing; run deploy --probe-build --commission-hardware")
        print(f"Preflight: bounded mixed-hardware commissioning; {probe_config}; {sys.executable}")
        return
    if mode == "hardware" and Path("/sys/class/net/can1/device").exists() and config.get("can", {}).get("backend") == "yousee":
        raise RuntimeError("Legacy dual-CyberGear configuration cannot start on the split-bus station; use the explicit commissioning mode until adaptation is complete")
    default = config.get("v3", {}).get("default_mode", "MANUAL")
    if config_path == firmware / "config/turret.yaml" and default != "AUTO_ROAM":
        raise RuntimeError("Normal station config must set v3.default_mode: AUTO_ROAM")
    for module in ("numpy", "PIL", "fastapi", "uvicorn", "websockets.sync.client",
                   "picamera2", "libcamera"):
        importlib.import_module(module)
    from picamera2.devices.imx500 import IMX500  # noqa: F401
    from perception.visiond import build_parser, load_config
    args = ["--config", str(firmware / "perception/configs/perception_v1.json"),
            "--profile", sys.argv[3]]
    if sys.argv[4] == "1":
        args.append("--production")
    load_config(build_parser().parse_args(args))
    if mode != "perception":
        binary = firmware / "build/control/controld"
        if not binary.is_file() or not os.access(binary, os.X_OK):
            raise RuntimeError("Controller missing; run deploy first")
        for key in ("intrinsics_file", "extrinsics_file"):
            path = Path(config["camera"][key])
            if not path.is_absolute():
                path = firmware / path
            if not path.is_file():
                raise RuntimeError(f"Camera calibration missing: {path}")
    print(f"Preflight: {config_path}; startup mode {default}; {sys.executable}")

if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        raise SystemExit(f"Preflight failed: {error}") from error
