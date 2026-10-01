"""Export the existing shaped_velocity profile into a yaw-control manifest."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from Firmware.commissioning.yaw_reference import velocity_reference_manifest
from adr0022_capture_launch import validate_yaw_control_contract


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True, help="base yaw-control manifest")
    parser.add_argument("--output", type=Path, required=True, help="new exported manifest path")
    parser.add_argument("--speed-deg-s", type=float, required=True)
    parser.add_argument("--position-rad", type=float, required=True,
                        help="initial position in the manifest's raw-count coordinate")
    parser.add_argument("--station-config", type=Path,
                        default=Path(__file__).resolve().parents[1]/"config/turret_mixed.yaml",
                        help="existing configured planning jerk source")
    args = parser.parse_args()
    config = json.loads(args.manifest.read_text())
    requested = yaml.safe_load(args.station_config.read_text())["axes"]["yaw"]
    envelope = SimpleNamespace(
        acceleration_rad_s2=float(config["controller_parameters"]["acceleration_cap"]),
        jerk_rad_s3=math.radians(float(requested["max_jerk_deg_s3"])),
        angle_min_rad=None, angle_max_rad=None)
    result = velocity_reference_manifest(config, math.radians(args.speed_deg_s), envelope,
                                         position_rad=args.position_rad)
    result["reference_profile"]["planning_jerk_source"] = str(args.station_config)
    validate_yaw_control_contract(result)
    with args.output.open("x", encoding="utf-8") as output:
        output.write(json.dumps(result, indent=2, allow_nan=False)+"\n")
    print(json.dumps({"manifest": str(args.output), "reference_profile": result["reference_profile"]}, indent=2))


if __name__ == "__main__":
    main()
