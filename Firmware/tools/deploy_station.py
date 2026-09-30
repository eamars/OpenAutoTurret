"""Deploy committed source from Windows/Linux using git, ssh, scp and remote Bash.

Builds a separate release; --activate opts into stopping the old stack and
starting the new one. Never resets, cleans or overwrites the target checkout.
"""
import argparse
import ipaddress
import pathlib
import os
import sys
from pathlib import Path
import shlex
import subprocess
import tempfile


def run(args, **kwargs):
    return subprocess.run(args, check=True, **kwargs)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="eamars@rpi-turret")
    parser.add_argument("--connect-address", type=ipaddress.ip_address,
                        help="optional discovered IP; retains --host's known SSH identity")
    parser.add_argument("--root", default="/home/eamars/workspace/OpenAutoTurret")
    parser.add_argument("--activate", action="store_true",
                        help="after successful build/check, park/stop the old stack and start this release")
    parser.add_argument("--ready-timeout", type=int, default=420,
                        help="seconds to wait for automatic readiness after --activate (default: 420)")
    parser.add_argument("--probe-build", action="store_true",
                        help="build only the runtime controller and preflight; defer regression tests")
    parser.add_argument("--known-hosts", type=pathlib.Path,
                        default=(pathlib.Path(os.environ["OTA_KNOWN_HOSTS"])
                                 if os.environ.get("OTA_KNOWN_HOSTS") else None),
                        help="explicit known_hosts for the station. The container's home is "
                             "not durable (measured 2026-09-28: an image rebuild took the "
                             "ambient ~/.ssh/known_hosts with it and every ssh here died with "
                             "status 255), so the station's identity is passed in, not hoped "
                             "for. Env OTA_KNOWN_HOSTS does the same for scripts.")
    parser.add_argument("--identity", type=pathlib.Path,
                        default=(pathlib.Path(os.environ["OTA_SSH_IDENTITY"])
                                 if os.environ.get("OTA_SSH_IDENTITY") else None),
                        help="private key for the station, with IdentitiesOnly. The container's "
                             "home is not durable: when the image was rebuilt 2026-09-28 the "
                             "ambient key went with it and ssh answered 255 -- an auth failure "
                             "that looks exactly like a host-key failure. Passed in, like the "
                             "known_hosts. Env OTA_SSH_IDENTITY does the same for scripts.")
    parser.add_argument("--prebuilt", action="store_true",
                        help="cross-compile here with tools/cross_build.py and ship the "
                             "binaries: the station runs the suite rather than building it. "
                             "Compiling needs no hardware; only running the tests does.")
    parser.add_argument("--baseline-bundle", type=pathlib.Path,
                        help="ship a committed ADR-002.2 baseline or neutral-current acquisition bundle into a separate release; "
                             "validate with launcher check, without starting devices, installing packages or compiling")
    parser.add_argument("--commission-hardware", action="store_true",
                        help="build/check the bounded mixed-hardware probe; does not start motors")
    parser.add_argument("--commission-mixed-controller", action="store_true",
                        help="build/check the manual mixed controller commissioning path; does not start motors")
    parser.add_argument("--probe-imu", action="store_true",
                        help="build/check only the timestamped BNO085 acquisition probe")
    args = parser.parse_args()
    if args.host.startswith("-") or not args.root.startswith("/"):
        parser.error("host must not be an option; root must be an absolute remote path")
    if args.ready_timeout <= 0:
        parser.error("--ready-timeout must be positive")
    if (args.commission_hardware or args.commission_mixed_controller or args.probe_imu) and args.activate:
        parser.error("commissioning activation uses an explicit bounded launcher run, not --activate")
    if sum((args.commission_hardware, args.commission_mixed_controller, args.probe_imu)) > 1:
        parser.error("choose one commissioning or IMU-only deployment mode")
    if args.baseline_bundle and any((args.activate, args.prebuilt, args.probe_build,
                                    args.commission_hardware, args.commission_mixed_controller, args.probe_imu)):
        parser.error("baseline-bundle is a separate non-activating deployment mode")
    repo = Path(__file__).resolve().parents[2]
    requirements = repo / "Firmware" / "requirements-station.txt"
    if not requirements.is_file():
        parser.error(f"Missing station dependency manifest: {requirements}")
    status = run(["git", "status", "--porcelain", "--untracked-files=normal"],
                 cwd=repo, capture_output=True, text=True).stdout
    if status.strip():
        parser.error("Commit source changes before deployment; run/ artifacts are ignored")
    revision = run(["git", "rev-parse", "HEAD"], cwd=repo,
                   capture_output=True, text=True).stdout.strip()
    if args.baseline_bundle:
        from adr0022_baseline_bundle import validate
        acquisition_record = validate(args.baseline_bundle, revision, repo / "Firmware")
    quote = shlex.quote
    connection = []
    if args.connect_address:
        connection = ["-o", f"HostName={args.connect_address}",
                      "-o", f"HostKeyAlias={args.host.rsplit('@', 1)[-1]}"]
    if args.identity is not None:
        # IdentitiesOnly: without it ssh also offers anything an agent is holding, and an
        # offered-but-wrong key can itself be the reason the station says no.
        connection += ["-i", str(args.identity), "-o", "IdentitiesOnly=yes"]
    if args.known_hosts is not None:
        # Pinned identity, checked strictly: a silent yes would let a different box wearing
        # this address -- or an ARP neighbour -- take over the station's role mid-deploy.
        connection += ["-o", f"UserKnownHostsFile={args.known_hosts}",
                       "-o", "StrictHostKeyChecking=yes", "-o", "GlobalKnownHostsFile=/dev/null"]

    def remote(command, **kwargs):
        return run(["ssh", "-o", "ConnectTimeout=10", *connection, args.host, command], **kwargs)

    releases = args.root.rstrip("/") + "/run/releases"
    # Reuse the station's existing project-local runtime, including libcamera
    # system-site-packages. Python packages are installed from the committed
    # manifest; no OS package installation or root shell is needed here.
    venv = args.root.rstrip("/") + "/run/station-venv"
    remote(f"test -x {quote(venv + '/bin/python')}")
    release = remote(f"mkdir -p {quote(releases)} && mktemp -d {quote(releases + '/' + revision[:12] + '.XXXXXX')}",
                     capture_output=True, text=True).stdout.strip()
    if not release.startswith(releases + "/") or "\n" in release:
        raise RuntimeError("Unexpected release path from target")
    with tempfile.TemporaryDirectory(prefix="ota-deploy-") as temporary:
        archive = Path(temporary) / "source.tar"
        run(["git", "archive", "--format=tar", f"--output={archive}", revision], cwd=repo)
        run(["scp", *connection, str(archive), f"{args.host}:{release}/source.tar"])
    remote(f"tar -xf {quote(release + '/source.tar')} -C {quote(release)} && "
           f"rm -- {quote(release + '/source.tar')} && "
           f"mkdir -p {quote(release + '/run')} && "
           f"ln -s {quote(venv)} {quote(release + '/run/station-venv')} && "
           f"printf '%s\\n' {quote(revision)} > {quote(release + '/REVISION')}")
    if args.baseline_bundle:
        # Dedicated acquisition release. Existing production venv/configuration
        # and active services are untouched; the check opens no device transport.
        bundle = release + "/baseline-bundle.tar"
        run(["scp", *connection, str(args.baseline_bundle), f"{args.host}:{bundle}"])
        launch_option = acquisition_record.get("launch_option", "--capture-baseline")
        capture_directory = release + ("/run/current-preparation" if launch_option == "--prepare-current" else "/run/baseline")
        helper = release + "/Firmware/tools/adr0022_baseline_bundle.py"
        remote(f"{quote(venv + '/bin/python')} {quote(helper)} install --bundle {quote(bundle)} "
               f"--revision {quote(revision)} --firmware {quote(release + '/Firmware')} "
               f"--output-directory {quote(capture_directory)}")
        manifest = capture_directory + "/manifest.json"
        script = release + "/Firmware/scripts/run_application.sh"
        remote(f"OTA_RUN_DIR={quote(release + '/run/stack')} bash {quote(script)} "
               f"check {launch_option} {quote(manifest)}")
        print(f"Acquisition release prepared; devices unopened: {release}\nRevision: {revision}\n"
              f"Manifest: {manifest}\n"
              f"Capture: OTA_RUN_DIR={quote(release + '/run/stack')} bash {quote(script)} "
              f"run {launch_option} {quote(manifest)}", flush=True)
        return
    # Model binaries stay outside Git/release source. The adapter checks the
    # pinned SHA before opening the shared artifact.
    models = args.root.rstrip("/") + "/run/hailo-probe"
    remote(f"if [ -d {quote(models)} ]; then "
           f"ln -s {quote(models)} {quote(release + '/run/hailo-probe')}; fi")
    remote(f"{quote(venv + '/bin/python')} -m pip install --disable-pip-version-check --no-input "
           f"-r {quote(release + '/Firmware/requirements-station.txt')}")
    if args.prebuilt:
        # The build machine is this one; see tools/cross_build.py for what it links against and
        # why that is the station's own library set rather than an approximation of it.
        run([sys.executable, repo / "Firmware" / "tools" / "cross_build.py"], cwd=repo)
        # Beside the build tree, not in a temporary directory: the deploying sandbox gave a
        # freshly created /tmp path to the parent and ENOENT to tar for the same string, which is
        # the kind of failure that reads like a broken toolchain and is really a writable-path.
        artifacts = repo / "Firmware" / "build-arm64.tar"
        run(["tar", "-C", str(repo / "Firmware"), "-cf", str(artifacts),
             "--exclude=*.o", "--exclude=.ninja_deps", "--exclude=.ninja_log",
             "--exclude=_deps", "build-arm64"])
        run(["scp", *connection, str(artifacts), f"{args.host}:{release}/build-arm64.tar"])
        remote(f"tar -xf {quote(release + '/build-arm64.tar')} -C {quote(release + '/Firmware')} "
               f"&& rm -- {quote(release + '/build-arm64.tar')}")
    script = release + "/Firmware/scripts/run_application.sh"
    smoke = release + "/Firmware/tools/station_smoke.py"
    remote(("OTA_PREBUILT=1 " if args.prebuilt else "")
           + f"bash {quote(script)} deploy" + (" --probe-build" if args.probe_build else "")
           + (" --commission-hardware" if args.commission_hardware else "")
           + (" --commission-mixed-controller" if args.commission_mixed_controller else "")
           + (" --probe-imu" if args.probe_imu else ""))
    label = "Probe-ready release (regression tests deferred)" if args.probe_build else "Verified release"
    print(f"{label}: {release}\nRevision: {revision}", flush=True)
    if args.activate:
        # stop uses common PID+start-time ownership, even across release paths.
        remote(f"bash {quote(script)} stop && bash {quote(script)} start")
        remote(f"run_dir=/tmp/ota-stack-$(id -u) && port=$(cat \"$run_dir/web.port\") && "
               f"{quote(venv + '/bin/python')} {quote(smoke)} "
               f"--url http://127.0.0.1:$port --wait-ready {args.ready_timeout}")
        print(f"Active and ready: {release}", flush=True)
    else:
        print("Build only; the running station was not changed.")
        if args.commission_hardware:
            print(f"Receive/discovery probe: ssh {args.host} \"bash {script} run --commission-hardware\"")
        elif args.commission_mixed_controller:
            print(f"Mixed controller commissioning: ssh {args.host} \"bash {script} start --commission-mixed-controller\"")
        elif args.probe_imu:
            print(f"IMU capture: ssh {args.host} \"bash {script} run --probe-imu\"")
        else:
            print("Activate: rerun the deploy command with --activate to perform the "
                  "HTTP/WebSocket smoke test and readiness wait.")
    print(f"Status: ssh {args.host} \"bash {script} status\"")


if __name__ == "__main__":
    main()
