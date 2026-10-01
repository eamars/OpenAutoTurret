#!/bin/bash
# Workstation helper (Git Bash on the Windows workstation; WSL does the ARM64 build and numerics):
#   build manifest -> cross-build -> pack -> deploy release -> run on station -> fetch journal -> score
#   run.sh yaw   LABEL [manifests.py options...]     e.g. run.sh yaw yaw-usecase-01 --script usecase
#   run.sh pitch LABEL [manifests.py options...]
# Station access uses this machine's own ssh identity (Host rpi-turret). Environment overrides:
#   OTA_WSL_REPO  the repo as WSL sees it        OTA_PY  WSL python with numpy/scipy
#   OTA_XBUILD    WSL path of the configured ARM64 cross-build tree;  OTA_XLIB  host libs for its assembler
#   OTA_OUT       output directory (ignored run/ tree)
set -euo pipefail
export MSYS_NO_PATHCONV=1
axis=$1; label=$2; shift 2
here=$(cd "$(dirname "$0")" && pwd); repo=$(cd "$here/../../.." && pwd)
wrepo=${OTA_WSL_REPO:-/mnt/c/workspace/OpenAutoTurret}
OTA_PY=${OTA_PY:-$wrepo/run/adr0022-local/.venv/bin/python}
OTA_XBUILD=${OTA_XBUILD:-$wrepo/run/adr0022-debian13/firmware-make}
OTA_XLIB=${OTA_XLIB:-$wrepo/run/adr0022-debian13/cross-host-lib}
out=${OTA_OUT:-run/servo-commission}/$label
mkdir -p "$repo/$out"; cd "$repo"
# Never ship a stale binary: a failed cross build stops here.
wsl.exe -e bash -c "export LD_LIBRARY_PATH=$OTA_XLIB; cd $OTA_XBUILD && make -j\$(nproc) commissiond imu-bno085 > $wrepo/$out/xbuild.log 2>&1"
wsl.exe -e bash -c "cd $wrepo/Firmware/tools/servo_commission && $OTA_PY manifests.py $axis $label $wrepo/$out/manifest.json $*" | tr -d '\0'
python Firmware/tools/adr0022_baseline_bundle.py pack --build run/adr0022-debian13/firmware-make --manifest "$out/manifest.json" \
  --session-label "$label" --output "$out/bundle.tar" > /dev/null
release=$(python Firmware/tools/deploy_station.py --baseline-bundle "$out/bundle.tar" 2>&1 | tee "$out/deploy.log" \
  | sed -n 's/^Acquisition release prepared; devices unopened: //p')
[ -n "$release" ] || { tail -5 "$out/deploy.log"; exit 1; }
if [ "$axis" = yaw ]; then option=--control-yaw; dir=yaw-control; journal=yaw-control.jsonl
else option=--establish-homing; dir=sensorless-homing; journal=sensorless-homing.jsonl; fi
ssh rpi-turret "OTA_RUN_DIR=$release/run/stack bash $release/Firmware/scripts/run_application.sh run $option $release/run/$dir/manifest.json" \
  2>&1 | grep -E '"kind":"footer"' | head -1 | cut -c1-200 || true
scp -q "rpi-turret:$release/run/$dir/$journal" "$out/$journal"
wsl.exe -e bash -c "cd $wrepo/Firmware/tools/servo_commission && $OTA_PY score.py $axis $wrepo/$out/$journal $wrepo/$out/manifest.json" | tr -d '\0'
