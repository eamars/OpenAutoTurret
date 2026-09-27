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
    mixed_backend_check = len(sys.argv) > 5 and sys.argv[5] == "1"
    mixed_controller_commission = len(sys.argv) > 6 and sys.argv[6] == "1"
    config = yaml.safe_load(config_path.read_text())
    if mode == "imu":
        if not os.access("/dev/i2c-1", os.R_OK | os.W_OK):
            raise RuntimeError("BNO085 requires unprivileged read/write access to /dev/i2c-1")
        if not os.access(firmware / "build/imu-bno085", os.X_OK):
            raise RuntimeError("IMU executable missing; deploy --probe-build --probe-imu")
        print("Preflight: BNO085 capture only; no motor or camera process")
        return
    if mode == "commission":
        if mixed_backend_check:
            profile_path = Path(os.environ.get(
                "OTA_MIXED_HARDWARE_CONFIG", firmware / "config/mixed_hardware.yaml")).resolve()
            profile = yaml.safe_load(profile_path.read_text())
            _validate_mixed_profile(profile, profile_path)
            for axis in ("yaw", "pitch"):
                _validate_can_spi_mapping(profile["buses"][axis], axis, require_up=True)
            binary = firmware / "build/probe-mixed-backend"
            if not binary.is_file() or not os.access(binary, os.X_OK):
                raise RuntimeError("Mixed-backend probe missing; run deploy --probe-build --commission-hardware --with-imu --mixed-backend-check")
            print(f"Preflight: mixed-backend observe-only check; {profile_path}; both CAN links mapped and up")
            return
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
    if mode == "mixed-controller-commission":
        if not mixed_controller_commission or config_path != firmware / "config/turret_mixed.yaml":
            raise RuntimeError("Mixed controller commissioning requires the explicit launcher mode and config/turret_mixed.yaml")
        profile_path = Path(config.get("hardware_profile", ""))
        if not profile_path.is_absolute():
            profile_path = firmware / profile_path
        profile_path = profile_path.resolve()
        profile = yaml.safe_load(profile_path.read_text())
        _validate_mixed_profile(profile, profile_path)
        for axis in ("yaw", "pitch"):
            _validate_can_spi_mapping(profile["buses"][axis], axis, require_up=True)
        if not os.access("/dev/i2c-1", os.R_OK | os.W_OK):
            raise RuntimeError("Mixed commissioning needs unprivileged read/write access to /dev/i2c-1 for the BNO085 observer")
        if not os.access(firmware / "build/imu-bno085", os.X_OK):
            raise RuntimeError("BNO085 executable missing; run deploy --probe-build")
        if not os.access(firmware / "build/control/controld", os.X_OK):
            raise RuntimeError("Controller missing; run deploy --probe-build --commission-mixed-controller")
        print("Preflight: explicit manual mixed-controller commissioning; both CAN links mapped/up, BNO085 available; no vision/web")
        return
    hardware_profile = config.get("hardware_profile")
    split_bus = Path("/sys/class/net/can1/device").exists()
    if mode == "hardware" and split_bus and not hardware_profile:
        raise RuntimeError("Legacy configuration cannot start on a split-bus station; select config/turret_mixed.yaml with OTA_CONTROL_CONFIG")
    if mode == "hardware" and hardware_profile:
        profile_path = Path(hardware_profile)
        if not profile_path.is_absolute():
            profile_path = firmware / profile_path
        profile_path = profile_path.resolve()
        profile = yaml.safe_load(profile_path.read_text())
        _validate_mixed_profile(profile, profile_path)
        for axis in ("yaw", "pitch"):
            _validate_can_spi_mapping(profile["buses"][axis], axis, require_up=True)
        if not os.access("/dev/i2c-1", os.R_OK | os.W_OK):
            raise RuntimeError("Mixed station needs unprivileged read/write access to /dev/i2c-1 for the BNO085 host reference")
        if not os.access(firmware / "build/imu-bno085", os.X_OK):
            raise RuntimeError("BNO085 executable missing; deploy --probe-build")
        raise RuntimeError("Normal mixed startup remains gated pending qualified pitch homing/stop and continuous-yaw runtime validation; use --mixed-backend-check for observe-only commissioning")
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

def _validate_mixed_profile(profile, path):
    if not isinstance(profile, dict) or profile.get("schema_version") != 1:
        raise RuntimeError(f"Unsupported or invalid mixed hardware profile: {path}")
    buses = profile.get("buses")
    axes = profile.get("axes")
    if not isinstance(buses, dict) or not isinstance(axes, dict):
        raise RuntimeError(f"Mixed profile needs buses and axes mappings: {path}")
    if set(buses) != {"yaw", "pitch"} or set(axes) != {"yaw", "pitch"}:
        raise RuntimeError(f"Mixed profile must define exactly yaw and pitch: {path}")
    for name in ("yaw", "pitch"):
        bus = buses[name]
        if not isinstance(bus, dict) or not bus.get("interface") or not bus.get("spi_parent"):
            raise RuntimeError(f"Mixed profile buses.{name} needs interface and spi_parent")
        if bus.get("bitrate") != 1000000:
            raise RuntimeError(f"Mixed profile buses.{name}.bitrate must be 1000000")
        axis = axes[name]
        if not isinstance(axis, dict) or axis.get("bus") != name:
            raise RuntimeError(f"Mixed profile axes.{name} must reference its named bus")
    yaw, pitch = axes["yaw"], axes["pitch"]
    if yaw.get("protocol") != "gm6020" or yaw.get("topology") != "continuous":
        raise RuntimeError("Mixed yaw must be a continuous GM6020")
    if pitch.get("protocol") != "cybergear" or pitch.get("topology") != "bounded":
        raise RuntimeError("Mixed pitch must be a bounded CyberGear")
    try:
        pitch_current = float(pitch["current_limit_a"])
    except (KeyError, TypeError, ValueError) as error:
        raise RuntimeError("Mixed pitch must specify its 5 A current ceiling") from error
    if pitch_current != 5.0:
        raise RuntimeError("Mixed pitch current_limit_a must remain exactly 5 A")
    if buses["yaw"]["interface"] == buses["pitch"]["interface"]:
        raise RuntimeError("Mixed profile requires independent yaw and pitch CAN interfaces")

def _validate_can_spi_mapping(bus, axis, require_up):
    interface = bus["interface"]
    net = Path("/sys/class/net") / interface
    device = net / "device"
    if not device.exists() or device.resolve().name != bus["spi_parent"]:
        raise RuntimeError(f"{axis}: {interface} does not map to SPI parent {bus['spi_parent']}")
    try:
        if (net / "type").read_text().strip() != "280":
            raise RuntimeError(f"{axis}: {interface} is not a CAN network interface")
        flags = int((net / "flags").read_text().strip(), 16)
    except OSError as error:
        raise RuntimeError(f"{axis}: cannot inspect {interface}: {error}") from error
    if require_up and (flags & 0x1) == 0:
        raise RuntimeError(f"{axis}: {interface} is down; bring it up at 1 Mbit/s before the probe/station")

if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        raise SystemExit(f"Preflight failed: {error}") from error
