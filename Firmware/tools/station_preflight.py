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
    default = config.get("v3", {}).get("default_mode", "MANUAL")
    if config_path == firmware / "config/turret.yaml" and default != "AUTO_ROAM":
        raise RuntimeError("Normal station config must set v3.default_mode: AUTO_ROAM")
    for module in ("numpy", "PIL", "fastapi", "uvicorn", "picamera2", "libcamera"):
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
