# Deploy: put committed source onto the station

## What this is for

Moving committed source into a runnable release on the station. Not for experiments with uncommitted
source — the tool refuses a dirty tree, and it should: a release whose contents nobody can name is
not reproducible.

## Where the work happens

**Compilation happens on the deployment host, not on the station.** The normal path cross-compiles
every binary for aarch64 on the host, ships an archive, and the station only unpacks, verifies and
runs. The station compiles nothing in this path; if you find yourself reaching for a compiler over
ssh, you have left the documented route.

Two consequences worth knowing before you are surprised by them:

- The build tree inside a shipped release was **configured on the machine that built it** and
  contains that machine's absolute paths. Running `cmake --build` against a release directory on the
  station fails with something like `ninja: error: mkdir(/workspace): Permission denied`. That is
  not a permissions problem to fight; it is the release telling you it was built elsewhere.
- Test binaries that bake in `__FILE__` carry the **compile host's** paths, which do not exist on the
  station. This project's fix is the `OTA_FIRMWARE_ROOT` environment variable, with the compiled-in
  path kept only as a fallback for native in-tree runs. Any new test that reads a data file must use
  it, or it will pass here and fail there.

Check what your host can actually do before choosing a route:

```bash
uname -m                                  # this host's architecture
command -v aarch64-linux-gnu-g++ || echo "no cross toolchain"
ssh <station> 'uname -m'                  # the station is aarch64
```

## The two things named "deploy" — do not conflate them

| Command | What it is | Where it builds |
|---|---|---|
| `Firmware/tools/deploy_station.py` | the workstation handover: archives `HEAD`, creates a fresh `run/releases/<sha>.XXXXXX` on the station, ships artifacts, optionally restarts the stack | **here**, cross-compiling |
| `bash Firmware/scripts/run_application.sh deploy` | build, test and preflight **the checkout you are standing in** | **there**, natively, on that machine |

They are not alternatives picked by taste: the first is how a release gets a revision identity and a
separate directory; the second is how you check out a tree in place. Both leave motors alone unless
you ask for more.

## How hard to build

Builds are expected to use **every core**: `cross_build.py` builds with `--parallel $(nproc)`
already, and the launcher's native path defaults to `-j$(nproc)` as of 2026-09-29 (it was pinned to
`-j2` when a native build beside a running stack risked the station's 5 V headroom -- lower it with
`OTA_BUILD_JOBS=2` if you are compiling on the Pi *while* the cameras, the Hailo and the axes are all
loaded, which is still an unmeasured combination). A cold arm64 tree is a full rebuild of the
controller and takes minutes; that is expected, not a hang, and it is the reason `--prebuilt` exists.

## The command

```bash
# resolve the station's address rather than restating it (see "Address" below)
OTA_STATION_ADDRESS=<observed-ip> scripts/station_address.sh deploy -- \
    --probe-build --ready-timeout 420            # build + preflight, regression deferred
scripts/station_address.sh deploy -- --activate --ready-timeout 420    # …then restart through the launcher
```

Three things a handover needs, and each has already cost this project a session:

1. **`--prebuilt`** — reuse artifacts built on this host instead of rebuilding. A cold arm64 tree is
   a full rebuild of the controller; if you have just built it, say so.
2. **`OTA_FIRMWARE_ROOT`** — so a test binary can find its data on a machine that is not the one that
   compiled it.
3. **An explicit SSH identity** — `--identity` and `--known-hosts`, with `StrictHostKeyChecking=yes`
   and `GlobalKnownHostsFile` blanked. The container's `~/.ssh` is not durable, and a deploy that only
   works while one container's home directory survives is not a deploy.

## Address

The station's address is **observed, not static** — it changed from `192.168.2.100` to
`192.168.2.103` without anyone deciding to move it, and every place that had copied the old number
went on confidently diagnosing the wrong box. So: ask the box (`scripts/station_address.sh print`),
which verifies each candidate **by connecting through the pinned host key under its alias** — which
also means an address change is a non-event and a different box wearing the address is not. If
nothing answers, it fails and names what to do. Do not paste a literal address into a command, a
script or a document and call that a fix.

If the pinned fingerprint ever differs from the box's, that is a finding to investigate, not a line
to edit.

## What it proves

ADR-002.2 baseline acquisition has a separate `deploy_station.py --baseline-bundle FILE`
path, described in [the capture card](adr0022-capture.md). It ships only the two
identified acquisition executables with committed source, uses the existing project
venv, and runs launcher `check` without opening devices. It does not activate the
production stack, install dependencies, compile or run regression tests on the station.
This is an acquisition release, not a verified production deployment. Normal deployment
and its registered CTest requirements are unchanged.

A successful `deploy_station.py` run proves: this exact revision built, its test suite passed on the
station where the hardware is, preflight passed, and (with `--activate`) the launcher stopped and
started it and reached readiness.

## What it does not prove

Nothing about behaviour. A green deploy does not qualify stop behaviour, tracking accuracy, or the
mechanical acceptance items recorded as `DEFERRED_TO_ADR002` in
[`../ADR-001/reports/WP9_LEDGER_2026-09-29.md`](../ADR-001/reports/WP9_LEDGER_2026-09-29.md).
And `--activate` is a motor event: pitch homes, so stand clear of the travel.

## A host with no cross-toolchain (read this before improvising)

Nothing here assumes your host can cross-compile — the next operator may arrive on a machine that
cannot. Choose one of these, and record the choice and its cost in the report or ADR you are writing:

- **A. Build on the station, natively.** `run_application.sh deploy` in the station's checkout.
  Correct and self-contained; the cost is a full native compile on the Pi, which is minutes slower
  than the cross path and runs beside the live stack. Tests run where they belong.
- **B. Install a cross-toolchain on this host.** `aarch64-linux-gnu-g++` at the same version as the
  station's compiler, plus arm64 copies of the third-party deps. Note that arm64 `libgtest` only
  affects *cross-built tests*: the deploy path runs the suite on the station, so a missing gtest arm64
  package is a limitation to state, not a blocker to hide.
- **C. Take the artifacts from wherever they were built.** Build elsewhere, hand the release over with
  `--prebuilt`, and record who built what revision — an artifact with no named producer is the same
  class of problem as an uncommitted tree.

What is *not* an option: starting from "I don't know how this works" and trying things on the running
station. The station is an instrument, not a scratch pad.

## Known failure modes, with their reasons

- **The remote step exits 2 right after a successful cross build** → the deploy ran without
  `--prebuilt`, so `run_application.sh deploy` on the station tried a **native** build and this station
  has no native dependency set. A cross build that already produced the artifacts must be handed over
  with `--prebuilt`; that flag is not an optimisation, it is the half of the route that says "the
  station compiles nothing".
- **A stale release directory that never activated** is normal after a failed deploy: the tool creates
  the directory before it builds. Remove it (`run/releases/<sha>.XXXXXX`) rather than letting the pile
  grow -- this station accumulated 106 release directories and 20 GB. Deleting the release a document
  names as qualified removes that document's rollback artifact, so say which ones you removed.
- **`ninja: error: mkdir(/workspace)`** on the station → you tried to build a release tree that was
  configured on another machine. Use the cross path or route A.
- **Four YAML-reading tests fail only on the station** → a test baked `__FILE__` from the compile
  host; it needs `OTA_FIRMWARE_ROOT`.
- **Green with suspiciously few tests** → an artifact archive that shipped fewer binaries than were
  built. The suite-count gate ("not green below 40 test binaries") exists because this happened.
- **`status 255` from ssh** → check which half failed: host-key or authentication. Both come from the
  container's home not being durable, and they look identical from the exit code alone.
- **Pruned `run/releases/` and the running stack is now executing a deleted directory** → there is **no
  pointer file**. "Which release is active" is answered by the running processes, not by a `latest`
  symlink or a `run/active_release` (neither exists; inventing one is how this happened on 2026-09-29:
  `cat run/active_release` returned empty, the prune loop therefore treated *every* directory as stale,
  and the stack kept serving from `.../0c3a917b7c62.mYPvxo/Firmware (deleted)` — visible as `(deleted)`
  in `readlink /proc/<pid>/cwd`). Ask the processes instead: `pgrep -af visiond` shows the release path
  in its own argv, and that is the one directory to keep. Recovery is a re-deploy of the same revision;
  the stack keeps serving meanwhile, which is the only mercy in this failure mode.
