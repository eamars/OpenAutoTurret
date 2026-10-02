"""ADR-003 tracking: the stage programs (one command each, every decision a fixed rule).

    tracking.py stage1 [--params FILE] [--out DIR]
        the 14 scenarios in simulation with the production tracking core and the ADR-002.2
        servos and plant models; checks, the Level-1 FF/FB comparison and performance.
        Exit status 1 if any check fails.

    tracking.py timing [--no-build] [--no-write] [--out DIR]
        stage 2, station: the tracking camera's optical time on the control clock. One yaw
        session (commissioned servo, sines up to ~15 deg/s, pitch disabled at its centre)
        while the camera records the scene; timing.py fits the offset and the rolling-shutter
        row time from the parts of the image that are static in the world. Writes
        config/tracking/camera_timing.json only when its gates hold. No subject needed.

    tracking.py assemble
        config/tracking/tracking.json: the stage-1 prior with every stage-2 measurement that
        exists substituted (camera_timing.json so far), each field labelled measured or prior.
        Production (turret_mixed.yaml tracking.core) reads it.

    tracking.py timing --resume RUN_DIR [--analyse-only]
        reuse the sessions recorded in RUN_DIR and run only the missing ones (or none).

Reports go to run/tracking/<id>/ (report.json, report.md). stage1 defaults to the stage-1 prior
(config/tracking/tracking_prior.json); production reads the assembled config/tracking/tracking.json.
"""
import argparse
import datetime
import json
import math
import sys
from pathlib import Path

import sim
import stage1

DEG = math.pi / 180

# Stage 1 is the synthetic world: its camera stamps the start of exposure, so it runs on the
# prior. tracking.json carries the station's measured camera semantics (tracking.py assemble).
DEFAULT_PARAMS = ("tracking_prior.json",)


def load_params(path):
    if path:
        return json.load(open(path, encoding="utf-8")), str(path)
    for name in DEFAULT_PARAMS:
        f = sim.FIRMWARE / "config/tracking" / name
        if f.exists():
            return json.load(open(f, encoding="utf-8")), str(f.relative_to(sim.REPO))
    raise SystemExit("no tracking parameters")


def out_dir(args, kind):
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    d = Path(args.out or sim.REPO / "run/tracking" / f"{kind}-{stamp}")
    d.mkdir(parents=True, exist_ok=True)
    return d


def stage1_command(args):
    params, source = load_params(args.params)
    results = stage1.run_all(params)
    report = {"stage": 1, "parameters": source, "applied": next(iter(results.values()))["applied"],
              "scenarios": {}, "comparison_level1": stage1.comparison(results)}
    failures = 0
    for sid, r in results.items():
        checks = stage1.checks(sid, r, params)
        failures += sum(not c["pass"] for c in checks)
        report["scenarios"][sid] = {"name": stage1.scenarios.SCENARIOS[sid]["name"], "checks": checks,
                                    "performance": stage1.performance(sid, r)}
    report["result"] = "PASS" if not failures else f"FAIL ({failures} checks)"
    d = out_dir(args, "stage1")
    (d / "report.json").write_text(json.dumps(report, indent=1, default=float), encoding="utf-8")
    lines = [f"# ADR-003 stage 1: {report['result']}", "", f"Parameters: `{source}`.", "",
             "| Scenario | Checks | Framing RMS / peak, FF+FB (px) | FB only RMS (px) | Ideal actuator RMS (px) |", "|---|---|---|---|---|"]
    for sid, s in report["scenarios"].items():
        p = s["performance"]
        ok = sum(c["pass"] for c in s["checks"])
        lines.append(f"| {sid} {s['name']} | {ok}/{len(s['checks'])} | {p['ff_fb']['framing_rms_px']:.1f} / {p['ff_fb']['framing_peak_px']:.1f} | "
                     f"{p['fb_only']['framing_rms_px']:.1f} | {p['ideal_actuator']['framing_rms_px']:.1f} |")
    lines += ["", "## Failed checks", ""]
    lines += [f"- {sid}: {c['check']}: {c['value']} (limit {c['limit']})" for sid, s in report["scenarios"].items()
              for c in s["checks"] if not c["pass"]] or ["none"]
    lines += ["", "## Level-1 FF+FB against FB only (steady windows)", "", "| Scenario | Rate (deg/s) | Framing FF+FB / FB only (px) | Lag FF+FB / FB only (s) |", "|---|---|---|---|"]
    for sid, c in report["comparison_level1"].items():
        lines.append(f"| {sid} | {c['target_rate_dps']} | {c['framing_rms_px']['ff_fb']} / {c['framing_rms_px']['fb_only']} | "
                     f"{c['lag_s']['ff_fb']} / {c['lag_s']['fb_only']} |")
    (d / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))
    print(f"\nreport: {d}")
    return 0 if not failures else 1


def timing_script():
    """Rest (the reference frames), then two sines both ways (0.35 Hz 4 deg + 1.1 Hz 0.8 deg:
    ~14 deg/s peak, inside the Level-1 envelope), then rest."""
    return [(3.0, "hold", 0, "rest"), (30.0, "sine", ((4.0 * DEG, 0.8 * DEG), (2 * math.pi * 0.35, 2 * math.pi * 1.1)), "sines"),
            (3.0, "hold", 0, "final")]


# The fixed plan: the camera's own auto-exposure, then a short fixed exposure; two exposures
# separate the exposure term of the optical time (timing.combine).
TIMING_SESSIONS = (("auto", None), ("fixed8ms", {"exposure_us": 8000, "analogue_gain": 12.0}))


def _session_kind(session_dir):
    m = json.load(open(session_dir / "manifest.json", encoding="utf-8"))
    fixed = (m.get("camera_capture") or {}).get("exposure_us")
    return "auto" if not fixed else f"fixed{int(fixed) // 1000}ms"


def timing_command(args):
    sys.path.insert(0, str(sim.FIRMWARE / "tools/servo_commission"))
    import manifests
    import station as stations
    import timing
    asset = json.load(open(sim.FIRMWARE / "config/servo/yaw_servo.json", encoding="utf-8"))
    d = Path(args.resume) if args.resume else out_dir(args, "timing")
    recorded = {_session_kind(p.parent): p.parent for p in sorted(d.glob("*/yaw-control.camera.jsonl"))}
    station = None
    sessions = []
    for kind, fixed in TIMING_SESSIONS:
        if kind in recorded:
            print(f"[recorded] {kind}: {recorded[kind].name}")
            session = recorded[kind]
        elif args.analyse_only:
            print(f"{kind}: not recorded")
            continue
        else:
            label = f"timing-{kind}-{d.name}"
            m = manifests.yaw(label, asset["servo_parameters"], timing_script())
            m["camera_capture"] = dict({"seconds": m["limits"]["duration_s"]}, **(fixed or {}))
            station = station or stations.Station(d, log=print, build=not args.no_build)
            j = station.run("yaw", m, label)
            footer = j.get("footer") or {}
            print(f"{kind}: session footer {footer.get('status')} {footer.get('detail', '')}")
            if footer.get("status") != "COMPLETE":
                return 1
            session = d / label
        r = timing.analyse(session)
        r["session"] = session.name
        print(f"{kind}: exposure {r['exposure_s'] * 1e3:.1f} ms, delay at row 0 {r['delta_row0_s'] * 1e3:.2f} ms, "
              f"row time {r['row_time_s'] * 1e6:.1f} us, scatter {r['row_fit_residual_ms']:.2f} ms, R^2 {r['r2']:.2f}"
              + (f" -- {'; '.join(r['fails'])}" if r["fails"] else ""))
        sessions.append(r)
    result = timing.combine(sessions)
    result.update(sessions=sessions, servo_asset=asset["provenance"]["run"], measured=datetime.date.today().isoformat(),
                  method="tools/tracking/timing.py: ego-motion, consecutive-frame flow against encoder increments, row bands, two exposures")
    (d / "camera_timing.json").write_text(json.dumps(result, indent=1), encoding="utf-8")
    print(json.dumps({k: v for k, v in result.items() if k != "sessions"}, indent=1))
    if not result["valid"]:
        print("NOT WRITTEN: " + "; ".join(result["fails"]))
        return 1
    if not args.no_write:
        (sim.FIRMWARE / "config/tracking/camera_timing.json").write_text(json.dumps(result, indent=1) + "\n", encoding="utf-8")
        print("wrote config/tracking/camera_timing.json")
    return 0


def assemble_command(args):
    cfg = sim.FIRMWARE / "config/tracking"
    asset = json.load(open(cfg / "tracking_prior.json", encoding="utf-8"))
    sources = {"estimator": "prior (stage 1): R and q_a need a moving subject (stage 2)",
               "level1": "prior: lambda from the stage-2 solve; limits by owner ruling (100 RPM, physical capability)",
               "pixel_sigma_px": "prior (stage 1): measured from a still subject in stage 2"}
    timing_file = cfg / "camera_timing.json"
    if timing_file.exists():
        t = json.load(open(timing_file, encoding="utf-8"))
        if t.get("valid"):
            asset["timing"] = t["timing"]
            sources["timing"] = (f"measured {t['measured']} (tracking.py timing, sessions "
                                 f"{', '.join(x['session'] for x in t['sessions'])}): uncertainty "
                                 f"{t['timestamp_uncertainty_s'] * 1e3:.1f} ms")
    asset["purpose"] = ("ADR-003 tracking asset read by production (turret_mixed.yaml tracking.core) and the 3a host. "
                        "Assembled by tools/tracking/tracking.py assemble; do not edit by hand.")
    asset["stage2"] = sources
    asset["assembled"] = datetime.datetime.now().isoformat(timespec="seconds")
    (cfg / "tracking.json").write_text(json.dumps(asset, indent=1) + "\n", encoding="utf-8")
    print(json.dumps(sources, indent=1))
    print("wrote config/tracking/tracking.json")
    return 0


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="command", required=True)
    c = sub.add_parser("stage1", help="the 14 scenarios in simulation")
    c.add_argument("--params", help="tracking parameter file (default: the stage-1 prior)")
    c.add_argument("--out", help="report directory (default run/tracking/stage1-<time>)")
    c = sub.add_parser("timing", help="stage 2: camera optical time from ego-motion (station)")
    c.add_argument("--no-build", action="store_true", help="reuse the current ARM64 build")
    c.add_argument("--no-write", action="store_true", help="do not write config/tracking/camera_timing.json")
    c.add_argument("--out", help="run directory (default run/tracking/timing-<time>)")
    c.add_argument("--resume", help="a timing run directory: reuse its recorded sessions, run the missing ones")
    c.add_argument("--analyse-only", action="store_true", help="with --resume: never run a session")
    sub.add_parser("assemble", help="config/tracking/tracking.json from the prior and the stage-2 measurements")
    args = p.parse_args()
    sys.exit({"stage1": stage1_command, "timing": timing_command, "assemble": assemble_command}[args.command](args))


if __name__ == "__main__":
    main()
