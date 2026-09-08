"""Deploy committed source from Windows/Linux using git, ssh, scp and remote Bash.

Builds a separate release; --activate opts into stopping the old stack and
starting the new one. Never resets, cleans or overwrites the target checkout.
"""
import argparse
from pathlib import Path
import shlex
import subprocess
import tempfile


def run(args, **kwargs):
    return subprocess.run(args, check=True, **kwargs)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="eamars@rpi-turret")
    parser.add_argument("--root", default="/home/eamars/workspace/OpenAutoTurret")
    parser.add_argument("--activate", action="store_true",
                        help="after successful build/check, park/stop the old stack and start this release")
    args = parser.parse_args()
    if args.host.startswith("-") or not args.root.startswith("/"):
        parser.error("host must not be an option; root must be an absolute remote path")
    repo = Path(__file__).resolve().parents[2]
    status = run(["git", "status", "--porcelain", "--untracked-files=normal"],
                 cwd=repo, capture_output=True, text=True).stdout
    if status.strip():
        parser.error("Commit source changes before deployment; run/ artifacts are ignored")
    revision = run(["git", "rev-parse", "HEAD"], cwd=repo,
                   capture_output=True, text=True).stdout.strip()
    quote = shlex.quote

    def remote(command, **kwargs):
        return run(["ssh", "-o", "ConnectTimeout=10", args.host, command], **kwargs)

    releases = args.root.rstrip("/") + "/run/releases"
    # Reuse the station's existing project-local runtime, including libcamera
    # system-site-packages. No package installation or root shell is needed.
    venv = args.root.rstrip("/") + "/run/station-venv"
    remote(f"test -x {quote(venv + '/bin/python')}")
    release = remote(f"mkdir -p {quote(releases)} && mktemp -d {quote(releases + '/' + revision[:12] + '.XXXXXX')}",
                     capture_output=True, text=True).stdout.strip()
    if not release.startswith(releases + "/") or "\n" in release:
        raise RuntimeError("Unexpected release path from target")
    with tempfile.TemporaryDirectory(prefix="ota-deploy-") as temporary:
        archive = Path(temporary) / "source.tar"
        run(["git", "archive", "--format=tar", f"--output={archive}", revision], cwd=repo)
        run(["scp", str(archive), f"{args.host}:{release}/source.tar"])
    remote(f"tar -xf {quote(release + '/source.tar')} -C {quote(release)} && "
           f"rm -- {quote(release + '/source.tar')} && "
           f"mkdir -p {quote(release + '/run')} && "
           f"ln -s {quote(venv)} {quote(release + '/run/station-venv')} && "
           f"printf '%s\\n' {quote(revision)} > {quote(release + '/REVISION')}")
    script = release + "/Firmware/scripts/run_application.sh"
    remote(f"bash {quote(script)} deploy")
    print(f"Verified release: {release}\nRevision: {revision}", flush=True)
    if args.activate:
        # stop uses common PID+start-time ownership, even across release paths.
        remote(f"bash {quote(script)} stop && bash {quote(script)} start")
    else:
        print("Build only; the running station was not changed.")
        print(f"Activate: ssh {args.host} \"bash {script} stop && bash {script} start\"")
    print(f"Status: ssh {args.host} \"bash {script} status\"")


if __name__ == "__main__":
    main()
