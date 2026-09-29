# ADR-002 implementation and acceptance ledger

Updated 2026-09-30 (runtime surface section by 小满). Work is in progress; no physical qualification is claimed.

## Identity and authorized scope

- Source baseline: `main@2e789c59dc9038228ac2aa91caab0f07d26d8137`, initially clean.
- Observed running release: `90aa1f5d4c9155310940813e3933ad49fa36901c`, directory
  `/home/eamars/workspace/OpenAutoTurret/run/releases/90aa1f5d4c91.uZvIsv`.
- Pi checkout HEAD is separately `6a47f1dd696d75b878b8138dcaceb9f455ab9147`.
- Initial live state: MANUAL / READY / hold, no fault. Preserve this operator intent.
- Owner authorized implementation, deployment and bounded motor/encoder/IMU trials.
  Actual pitch payload is **camera plus Raspberry Pi**, not an empty axis.
- Owner requested functional verification first, thermal testing last using motor
  reports; no external thermometer is available. Active cooling is a possible
  later hardware change, not present qualification.
- Owner's subsequent execution rule: **never roll back after a build or release
  failure**. The Pi has no critical service. Preserve failure evidence, fix the
  defect and deploy the next version; do not spend time restoring older releases.
- Unchanged initial limits: yaw current `0x1FE`, 0.8 A, Kp 1 A/(rad/s), Ki 0.6 A/rad;
  pitch LimitCur <=5 A; host 200 Hz. No new calibrated gains or friction values.

## Acceptance matrix

| Capability | Source/offline | Single-axis physical | Thermal/two-axis | Current scope |
|---|---|---|---|---|
| Output arbitration/recovery | PR1 native PASS; Pi 80/80 | Ordinary-TX continuity observed; physical fault injection NOT_RUN | NOT_RUN | 4387411 Manual commissioning |
| Yaw current velocity loop | PR2 math replay and native PASS | Baseline measured; latency/smoothness targets not met | NOT_RUN | Existing 0.8 A bound |
| Yaw friction compensation | Bounded state/current integration tests PASS | NOT_RUN | NOT_RUN | Disabled until calibration |
| Pitch native speed tuning | Hot update PASS (§6: prepare→apply→read-back→restore against drive registers) | NOT_RUN | NOT_RUN | Existing native mode and 5 A bound |
| Pitch homing | Existing native/Pi tests PASS | Two starts completed; repeatability not yet reduced | NOT_RUN | Do not inherit old mechanism results |
| Typed payload qualification | Profile binding PASS (a campaign bound to one profile refuses to run under another) | NOT_RUN | NOT_RUN | No new qualified profile |

## PR1 evidence so far

The public `gm6020::VelocityLoop` probe was compiled and executed with WSL G++.
At timestamps 6/31/36 ms with constant fresh-position input, baseline output was
0.1003/0/0 A and validity was true/false/false. The thin fix gave
0.1003/0.1003/0.1006 A and remained valid: the late cycle freezes integration,
then normal integration resumes. This verifies the deterministic PI boundary;
it does not verify Linux/CAN timing or physical response.

Changes under test:

- No-progress is a performance episode; the guard no longer inserts routine
  zero-current frames. Emergency trips latch under the same mutex as normal TX.
- Current link state and error deltas replace cumulative error-count latching.
  Yaw command health uses its own bus; sustained TX failure is bounded at 20 ms.
- Watchdog action identifies the affected axis, retaining the healthy axis's
  controlled support instead of automatically disabling both axes.
- Trace records carry RX sequence/raw encoder/current, successful TX sequence/time,
  encoded output, requested output, command kind/reason and effective PI values.
  Transport success is not a motor acknowledgement. Pitch host current command
  remains unknown; its command is SpdRef in rad/s.
- Raw RX timestamps are preserved separately from the legacy cycle-bounded
  supervisor timestamp, and the trace exports raw RX time.
- Legacy mixed-station payload loading is explicitly unqualified; the historical
  file remains a conservative cap while typed identity binding is implemented.

Offline package tests: 45 passed. Synthetic metrics and illustrative arithmetic
ran successfully. These tools establish no mechanism stability.

The old running release was observed read-only for 10.529 s (2,080 unique control
cycles). Average reported host period was 5,064 us. Pitch's trace-visible RX
updates averaged 55.59 Hz, with feedback age 3.45/11.74/24.37 ms min/mean/max.
Yaw feedback age was 0/0.49/2.48 ms; its trace-visible update rate of 197.46 Hz is
limited by controller sampling and **is not the true CAN RX rate**. Yaw raw
temperature was 33 (unverified degrees scaling); pitch temperature was absent
from that trace. Peak-to-peak encoder position range was 0.1093 degrees pitch
and 0.04395 degrees yaw. Raw capture is ignored at
`run/adr002/baseline-90aa/trace-10s-and-imu.txt`; the stack stayed MANUAL/READY.

Native builds use WSL Ubuntu, GCC 15 and CMake 4.2, in ignored
`run/adr002/native-build`. Full compilation succeeded. The final native suite
`ctest --test-dir run/adr002/native-build -E retained_homing --output-on-failure`
passed 78 of 79 entries initially; the remaining watchdog-event test expected
no-progress to outrank a lost heartbeat. Its assertion now requires the actual
fatal condition first, then no-progress after the heartbeat condition clears.
`cmake --build run/adr002/native-build --target test_watchdog_trip_events --parallel "$(nproc)"`
and `ctest --test-dir run/adr002/native-build -R "^test_watchdog_trip_events$" --output-on-failure`
passed. Combined result: 79/79 entries, including nine new recovery/arbitration
cases executing the production backend and control loop. Evidence logs:
`run/adr002/native-ctest-final.log`, `native-watchdog-rebuild.log`, and
`native-watchdog-rerun.log`. Earlier assertion-only failures are superseded;
neither simulated energized state nor mocked transport establishes physical hold.
No retained-homing test is claimed: the native suite excludes it as directed by
AGENTS because that test writes `/dev/shm`.

PR1 source commit: `02d5e8ce120eccda2a755b8815dbbb10ab4dedb8`.
An inactive release was created at `run/releases/02d5e8ce120e.yA9dLm` on the Pi.
Its native compile was an execution mistake: the owner had cleared WSL resources
specifically for local compilation. After the owner's correction, all build
processes with that exact release working directory were stopped; a subsequent
process check found none remaining. No new controller was activated. Preserve
this incomplete directory as evidence; it is not a usable/verified release.
The route is now **WSL native tests and ARM64 cross-compilation only**, then
binary transfer and station tests. Missing cross dependencies are to be installed
locally, not used as a reason to compile on the Pi again.

A real local Unix SOCK_SEQPACKET probe found a missing PR1 integration case:
256 live rows transmitted (188,615 bytes), but the 1024-row frozen fault window
(753,862 bytes) failed with EMSGSIZE under the old 256 KiB sender buffer.
The follow-up sizes the sender for 2 MiB and reuses the shared MAX_FRAME in the
response capture client. Repeated real socket probe passed both 256 rows
(188,615 bytes) and 1024 rows (753,862 bytes). The rebuilt `test_web_server`
passed, including a full frozen-window regression over a real packet socket;
logs are `run/adr002/trace-socket-build.log` and `trace-socket-test.log`.
No activation, calibrated parameter change, or hardware acceptance is included
in the native gate above.

## Local cross-build and inactive deployment

WSL Ubuntu hosts a signature-verified Debian 13 amd64 chroot at
`/home/rba90/.cache/ota-adr002/debian13-verified`, with the workspace bound at
`/workspace/OpenAutoTurret`. AArch64 GCC 14.2 and fmt 10.1.1, spdlog 1.15.2,
yaml-cpp 0.8 and GTest 1.16 are installed there. The first cross attempt needed
the missing `make` package for GTest bootstrapping; installing it and building
forward succeeded. No rollback was performed. The rootfs target libc is 2.41
with a newer Debian security patch than the Pi; target execution is verified
by the station tests, not inferred from that version similarity.

`cross_build.py` succeeded on `8838b1d62186dd106f9b47800e8d642ae9e3e16f`.
Its reported 152 ELF artifacts include build objects and are not a test count.
`deploy_station.py --prebuilt --commission-mixed-controller` shipped the local
build into `/home/eamars/workspace/OpenAutoTurret/run/releases/8838b1d62186.n6vz4e`.
The station ran 67 test binaries with zero failures and passed the manual
mixed-controller CAN/IMU preflight. The 13 additional CMake integration entries
were then run from relocated CTest metadata: 13/13 passed, 33.37 seconds.
All 80 registered entries, including retained homing, are therefore covered.
The deployment path now uses the complete CTest manifest rather than a filename
glob. Its two focused local regressions and seven real relocated CTest probes
passed; two pre-existing detached-process launcher tests timed out in WSL.
Logs: `pi-ctest-missing-13.log`, `pr1-cross-build.log`
and `pr1-deploy-final.log` under ignored `run/adr002/`.

## Old release incident before activation

The old `90aa1f5` release independently entered watchdog fault at 22:38:01 while
the mistaken Pi build was underway. Its frozen trace ends at a 21.108 ms
hold/DERATE cycle with fresh yaw feedback; about 90 ms later the log records
fault and an 84.487 ms cycle. This is consistent with the old late-cycle failure
path but does not establish scheduling attribution or the exact first trigger.
Captured evidence is in `run/adr002/old-release-trip/`; analysis is in
`run/adr002/analysis/old-release-trip-report.md`. At 22:47 the launcher stopped
the old stack and cleaned up its processes. It reported `STOP FAILED` because
the controller was already faulted; logs confirm STOP/zero requests and clean
process exit, not normal parking qualification or confirmed GM disable.

## Failure handling (owner override)

Release `8838b1d62186.n6vz4e` was started through the launcher in Manual mixed
commissioning at 23:02 NZDT. Pitch completed mandatory homing and reached hold.
Actual pitch readbacks: RunMode 2, LimitCur 5 A, SpdKp 4, SpdKi 0.05.
Yaw current-mode configuration remains 0x1FE, 0.8 A, Kp 1 A/(rad/s), Ki 0.6 A/rad.
The final 12-second stationary window measured yaw RX 1000.04 Hz and pitch
49.50 Hz, feedback-age p99 0.9992/19.73 ms respectively, host period p99 5.065 ms.
Yaw successful TX sequence advanced every cycle with ordinary reason; one raw
RX age was -0.834 microseconds (RX arrived after the cycle clock sample).
Pitch temperature was 25.9 C; yaw raw byte 31 has unverified degree scaling.
Pi throttling flags 0x50000 indicate historical events, no active low-bit flags.
Captures and analysis reside under ignored `run/adr002/8838` and `analysis`.

The first yaw fine jog was accepted but moved only about 0.75 degrees in twelve
seconds: this is a baseline observation, not a performance pass. An existing
response_probe was rejected because it required physical two-axis homing even
for continuous yaw. Its gate now uses position readiness and the declared
runtime envelope (including explicitly unbounded yaw); bounded-axis clearance
and stationary checks remain. The production command-gate simulation passed.

Future releases use locally cross-built
committed source and a separate release directory, with launcher-controlled
stop/start and manual commissioning. Build/deployment failures are fixed forward:
preserve diagnostics and publish the next corrected version, with no rollback.
This owner rule overrides the ADR's generic rollback procedure. Retained geometry
remains intact. A failed physical trial still preserves traces before the ring
wraps and uses the launcher for stopping as necessary; it does not trigger
restoration of an old release. Do not switch GM back to voltage, overwrite
calibration, or interpret a GM zero-current request as confirmed disable.

## PR2: executable path and bounded compensation

Source now sends service yaw through the shared position-P / velocity-feedforward
path. Previously finalizing pitch homing selected yaw's host position interface
unconditionally. Leased yaw jogs retain their explicit velocity even while their
position waypoint is bounded relative to feedback. The pitch service path is
unchanged in this PR2 slice. PI gains and the 0.8 A / 5 A caps are unchanged.

The new directional friction state machine defaults disabled. Explicit moving
intent permits one bounded attempt per direction; repeated lease refreshes do
not restart it. Directional fresh-RX displacement confirms motion; reversing
waits for stationary feedback. Timeout is a performance observation. Final
current and slew limits govern PI integration, transition handoffs use the last
delivered effort, quiet hold retains its integral, and a reduced current cap
retains signed braking authority. Unknown calibration is not promoted to a
measured production profile. New trace fields expose assist/state/exhaustion.

A fixed 128-sample raw-RX history supports 20/30/40 ms measurement windows and
ignores repeated/backwards timestamps. Default estimator selection remains the
legacy 50 ms filter until physical A/B data select a window. All three candidate
window observations are temporarily traced to compare against actual CAN RX.

Before regression expansion, standalone WSL C++ probes replayed the captured
8838 normal-jog samples through the production estimator and combined current
loop. The latter preserved finite <=0.8 A output and exactly one start attempt
after a tiny reference-sign crossing was given a direction deadband. These
replays use recorded feedback, not a simulated claim about changed mechanics.
The native suite passed 79/80 entries initially; two new test expectations
observed their 10 ms attempt after its deadline. After correcting those test
intervals, the rebuilt transport tests passed. The combined native result is
80/80 CTest entries excluding retained homing. Final focused current/estimator,
parser and transport tests passed 3/3; `pr2-ctest.log` and
`pr2-final-focused-tests.log` retain the evidence and initial failure.

Release `4387411e8be9.PLkMoF` passed all 80 station CTest entries and preflight,
then completed its mandatory pitch homing. Five +/- normal yaw pairs were
captured at the initial pitch pose, followed by three accepted +5-degree pitch
responses and another bounded yaw series. Baseline fine +/-3 deg/s requests
showed roughly 1.521 s / 9.713 s three-count motion-confirmation delays; positive
normal motion used up to 0.632 A and drifted about 0.395 degrees after stop.
The baseline therefore does not meet response/hold targets. Every observed
active yaw row had ordinary output reason and advancing successful TX sequence;
this confirms no interleaved guard zero in those trials, not emergency-stop
qualification. Detailed raw captures/analysis remain ignored under `run/adr002`.

## Stop clock correction and session-only yaw trials

Release `eaef375f9046.bzBrVq` was cross-built in local WSL; all 81 registered
station CTest entries and preflight passed. It was not started. Stopping the
previous `4387411` session at 23:35 NZDT exposed a readiness defect: shutdown
passed the previous cycle clock to a newer yaw RX snapshot, so 5.145 ms of
future feedback was rejected before the fresh-clock check. A production-backend
WSL probe reproduced `old_clock_feedback=0 q=nan fresh_clock_feedback=1 q=0.4`.
Shutdown now obtains its snapshot clock at entry. An accepted unverified stop
also no longer resumes ordinary Hold for the 55-second parking budget: only an
actual Parking phase runs that state machine. The old session exited at 23:36:53
with STOP FAILED; this is not counted as parking qualification. The station
remained stopped while the next fix-forward release was prepared. No rollback
was performed or is authorized after build/deployment failures.

`yaw_control_trial` uses the existing command queue only in an explicit Manual
commissioning launch, with fresh stationary axes, no jog/probe and ALLOW state.
Its eight colon-separated values are Kp [A/(rad/s)], Ki [A/rad], RX window [ms],
positive/negative breakaway [A], positive/negative running assist [A], and final
output slew [A/s]. RX windows are 0 (legacy), 20, 30 or 40 ms. Four zero assist
values disable compensation. Changes are volatile, retain the 0.8 A cap, preserve
quiet effort across gain application, and explicitly remain unqualified. Normal
startup and stored configuration are unchanged. Physical A/B of this interface
is pending deployment; it is not a calibration result.

The second baseline series achieved +15.235 degrees pitch displacement and five
yaw direction pairs at that pose. Hold yaw current remained roughly 0.44–0.55 A,
comparable to moving current, with post-stop drift up to 0.835 degrees. Thus the
observed moving total current must not be copied into a friction feedforward
term. The +/-1-degree yaw probes also failed to reach their requested travel.
These are performance failures without runtime faults, retained for comparison.

Validation for this correction: the WSL native build and all 80 CTest entries
excluding retained homing passed (`session-trial-build.log`,
`session-trial-tests.log`). Fresh-stop-clock, bounded trial settings and quiet
gain-change effort regressions passed in the two focused CTest entries after
their final rebuild (`session-trial-focused-tests.log`). Document links passed.
The command's real socket/backend application and physical stop are still
pending the next station run; local tests do not establish those results.

## Owner-requested stop and work-in-progress snapshot (2026-09-30)

The owner stopped further tuning and implementation, then requested a commit
of the current work and removal of the WSL environment installed for this task.
The detailed operation history, parameter trials, errors and remaining work are
in [the agent behavior review](AGENT_BEHAVIOR_REVIEW_2026-09-30.md).

The PR3 snapshot adds six asynchronous pitch register observations with request
and actual RX timestamps, transaction cancellation before mode changes, verified
pitch gain application, trace fields, and a commissioning-only command. Payload
response checks now measure installed gains rather than silently writing gains
through the old void setter. Homing jitter diagnostics no longer infer that
current must be increased. Hardware-clock shutdown checks are also corrected.

This is **unfinished work, not a release qualification**. The transport probe
passed; the last complete native run passed 79/81 entries excluding retained
homing. A subsequent focused run passed 5/6 entries, with `park_power_probe`
still failing on stale/untrusted feedback. No further repair or build was run
after the owner stopped work. The PR3 snapshot has not been deployed.

The last deployed revision remains `bad742dddba6a95d3d055e43998adfd0506b0a6e`.
At 00:12 NZDT its launcher reported STOPPED, pitch disable confirmed, and yaw
zero requested with disable state unavailable. No rollback occurred. Trial
gains were volatile; production configuration still has its original defaults.
Runtime captures and unintegrated PR4 drafts remain under ignored `run/adr002`;
they are not included as runtime artifacts in this commit.

## 2026-09-30 · ADR-002.1 runtime surface (小满)

Identity this section speaks about: branch `ADR-002` at `479857b`, deployed release
`0b1b4b2b3ba7.AJVn7p`, aarch64 `controld` `3f2cabc4fe78d0de…`, station-generated
`parameter_inventory.json` `498b4e80204a22…`, frozen design `f761cbec39f356db…`. The inventory's
`source_rev` comes from the release's own `REVISION`; the Pi checkout at `6a47f1dd…` is a different
tree and is no longer allowed to masquerade as the built source (`git -C` walked up to it until
`77f71a5`).

### The four gates docs/ADR-002.1/00_CODEX_START.md:58 asks for

| Gate | Status | What was actually run |
|---|---|---|
| Parameter hot update on real hardware | PASS | `tools/adr0021_acceptance.py` walked every `experiment_writable` entry: **19/19** prepare→apply→read-back→restore, binary digest unchanged across the set, zero compiles, zero redeploys; `yaw.host_current_limit_a` written as `protected_read_only` and refused **server-side**. Then a real campaign: 16 candidates, 32 applied writes, 16 restores accepted, `refused: []`, `blocked: []` |
| Apply failure blocks the trial | PASS | The gate now stands in front of both trial and `param_apply`; measured refusal on the station: `param_apply refuses: manual_commissioning_off+mode_not_manual`, snapshot stayed `revision=0`, candidate left staged. A refused restore is `BLOCKED_restore_failed_*`, not a shrug |
| Experiment freeze | PASS | `adr0021_plan.py` refuses under-sized/oversized grids, missing `coarse_count_reason`, `confirm.repeats < 2`, stop rules without bounds, and names D7 when a dimension may not move; `--check` refuses post-freeze edits and inventory/binary drift. Binding carries four legs: source rev, binary digest, inventory digest, config/hardware profile |
| Reversal and prescribed-pose re-verification | **NOT_RUN** | Not attempted yet; no cell in this table may be read as physical qualification until it is |

### What this section deliberately does not claim

- The campaign's levels were the **sample grid** (`manifests/campaign.example.json`), authorised by the
  owner's `跑！` without levels. It is mechanism acceptance, **not** a tuning result: no scorer took part,
  so no candidate may be described as better, and `metrics` is absent rather than zero.
- `RUN` and `SCORE` (`00_CODEX_START.md:46`) were **exercised on the hardware** on 2026-09-30 with the
  campaign's own runner (`--run-trials --score`, 4×2 grid = 16 candidates, bound to the same release,
  commission mode, homing waited for before starting): 32 parameter exchanges applied and all 16 candidates
  restored, `blocked = none`. The frozen scorer awarded **`PASS_SCOPE` to 15 of 16** candidates, and the
  classification is honest about its scope: only `feedback` computed (RX age within threshold), while the
  other eight metrics abstained with a stated reason — a current-loop step trial does not exercise homing,
  position, reversal or start latency, and there is no valid temperature reading to grade. One candidate
  (`c02`) never ran because the station's own guard said `yaw tuning requires fresh stationary axes`; the
  runner recorded that as a gate speaking rather than a quality verdict, and restored the candidate anyway.
  **What this does not establish:** the matrix above stays NOT_RUN for the §5 reversal and prescribed-pose
  re-verification, and `PASS_SCOPE` on one metric is not a §7 quality pass for a candidate.
- Trace identity is per-record and measured: 16/16 trials, 256 records per window, 66 carrying the
  candidate tag, contiguous from announcement to newest. The check is contiguity to the newest record,
  not "every row tagged" — the window is rolling and its head predates the candidate.
- Two host-side python tests remain red and are named, not hidden: `test_install_station` validates
  `User=eamars` against the local user database (correct on the Pi, cannot pass as `dsh`); the
  `test_station_launcher` log-path assertion is still open.


## 2026-09-30 深夜：正反与规定姿态复验（docs/03 §5）——**部分通过，两处如实留着**

新工具 `Firmware/tools/adr0021_pose.py`（`--selftest` 不碰硬件；符号逻辑、按轴索引、接触判读都过）。
真站跑了两遍，结论不同，两遍都记：

| 检查 | 第一遍 | 第二遍 |
|---|---|---|
| 归零完整循环 ×3（§5 要求至少 3 次） | **PASS**：三轮都 `homing → hold`（每轮约 66 秒） | **FAIL**：`homing → fault` |
| 正反（命令步 → 编码器读数） | **FAIL（测法错）**：在滚动窗内取首尾差，窗里大半是踏步之前的历史 | **INCONCLUSIVE**：命令被拒，`encoder 4298 → 4298`，轴根本没动 |
| 中段摩擦平台（§5 先证明不当端点） | `BLOCKED_friction_plateau_needs_hand_resistance` | 同 |

**我这两个错都是测法的错，不是轴的错**，而且都被我自己的工具当场揭出来：

1. **滚动窗不能取首尾差。** trace 是约 256 行的滚动窗，窗内历史远多于踏步之后的行；首尾差量到的是"窗里恰好装了什么"，
   不是那一步。改成**踏步前最后一条读数 vs 踏步后最后一条读数**之后，第一遍那个看似"反向"的结论就消失了——
   取而代之的是 `delta=0`：轴没动，因为命令被拒。
2. **命令形状是我猜的。** `command_validation.hpp:212` 的例子写的是 `yaw+1`——**整名轴 + 符号 + 整度**；
   我发的是 `y+2.0`。形状校验只查形状，"哪个步长被批准"是 controld 的决定（§38–§41、§52），
   所以我不替它猜；被拒也**没有**被读成"轴没反应=质量差"。
   另外我原来只读 `reason` 字段，被拒的话在那里是空的——**拒得没声音就是白拒**，现在 `error` 与 `reason` 都收。

**没做的事，不遮**：第二遍 `homing → fault` 我**还不知道原因**——需要站自己的话（telemetry 的 fault 字段/日志），
下一轮先查这个再谈 §5 通过与否。摩擦平台那一格要人手在轴上加阻力，主人在睡觉，不叫醒。

⇒ **矩阵里的"正反与规定姿态复验"仍记 NOT_RUN**：归零三次完整通过是真的，正反一次都没量到（命令被拒），
一个 `BLOCKED_friction_plateau_*` 也是真的。基础设施工具链自此**齐了**，这一格**没到 PASS 就不写 PASS**。

## 2026-10-01：判据「能动、不堵转」——**两判据在 Manual/Hold 下都过了**，AUTO_ROAM 那条还没量

主人把判据降到"能动、不堵转"。真机跑出来的答案：

| 候选（实发串） | 静止电流 | 占包线 | `manual_step yaw±1` 位移 |
|---|---|---|---|
| **baseline** `1:0.6:20:0:0:0:0:0.001` | **0.0493 A** | **6.2%** | **+5 / −6 counts ⇒ 能动** |
| half_ki `1.0:0.3:…` | 0.1069 A | 13.4% | +2 / −2 |
| half_kp `0.5:0.6:…` | 0.158 A | 19.8% | +3 / −1 |
| half_both `0.5:0.3:…` | 0.2205 A | 27.6% | +3 / 0 |

- **判据②不堵转：过。** Manual/Hold 下静止电流是包线的 **6%**（不是早上在 AUTO_ROAM 量到的 100%）。
- **判据①能动：过。** 命令步之后编码器位移**非零且正反两向符号相反**（+5 / −6 counts）。

**三处必须跟着这张表一起读，否则它会被误读：**

1. **"2° 不合法"是我自己造的哑局。** 站的判决一直是 `"step size must be one of 0.5, 1, or 5 degrees"`，
   我却连着几轮把**回执**当判决读，于是报成"命令被拒、原因未知"。改成批准的 1° 之后，
   `manual_jog_start yaw+` 也当场被批准并让编码器 4099 → 4109。**轴一直是会动的。**
   工具现在硬性拒绝非常准步长（`SANCTIONED_STEP_DEGREES`），不再拿一条被拒的命令去量运动。
2. **后三行"位移小"不等于"不动"。** 我的判决线取在 >2 counts，是**我拍的**，不是文档给的；
   被拒命令的位移是精确的 `0.0`，所以非零本身就是运动的证据。诚实的写法是：
   **四个候选都动**，baseline 位移最大且对称性最好；阈值这条我要重定，不许拿它冒充判据。
3. **"降额反而电流更高"这个趋势我不敢当真。** 静止读数是在上一候选的踏步/恢复之后 2 秒取的，
   候选之间不独立（余温、位置、滚动窗都串味）。要拿它下结论，得改成每个候选前先静置并复量。

**还没做完的两条**（不遮）：①早上那个 **0.80 A = 100% 包线**是 **AUTO_ROAM** 下量到的，本报告只在
Manual/Hold 下验过——**AUTO_ROAM 的 yaw 路径还没复量**，而主人抱怨的堵转正是那个模式；
②`enabled_state = -1`（使能状态未知仍在灌流）仍未查清。

## 2026-10-01 补：我把"能动"撤回重测；文档里本来就有路

主人提醒之后回读文档，三条我凭空造的假设作废：`angle_count` 是**13 位一圈 0..8191**（22.75 计数/度，
`references/gm6020/GM6020_AI_Reference.md:72`），**滑环无约束、不计圈数**（`STATION_OPERATIONS.md:307`），
`axes.yaw.position_envelope: none` ⇒ yaw **没有位置包线**（`:524`）。所以"走软限位定标""长 jog 必须限时"
都是我编的；`adr0021_sweep.py` 的回绕展开按 65536 写也**是错的**。

由此**重算**：我先前报的"能动 +5/−6 counts"按真单位是 **0.22°/0.26°**——**那个 PASS 我撤回**，
它没到"命令 1°"的程度，也不满足主人要的长脉冲证明。

**现成路径**（launcher 自带、护栏齐备、IMU 独立测角）：

```
run --commission-hardware --with-imu --yaw-step-deg N      # 文档上界 15..45 ⇒ 90° 超出，我不擅自放宽
run --commission-hardware --with-imu --yaw-sweep-deg N --yaw-sweep-ff-a A   # drag sweep：正对"各角度阻力不均"
```

真机跑了一趟 `--yaw-step-deg 45`：**校验通过、会话正常结束**（`COMMISSIONING FINISHED; zero output requested`），
但**没有报出任何位移**。首要怀疑（**未验证**，不当结论）：usage 明写"yaw 推力单位跟随 `axes.yaw.control_mode`：
电流驱动用 `--yaw-current-a`，电压驱动用 `--yaw-voltage`，探针会拒绝用错的那个"——我两者都没给，
**在这条 current-mode profile 上探针可能是"零推力"于是根本没推**。下一轮先读 `axes.yaw.control_mode`，
再按 `:482` 那次成功记录的量级给显式推力重跑。

`enabled_state=-1` **就地结案，不是故障**：账本写着 `GM6020 zero voltage is not a verified disable`
（`:495`）——断开状态本来就不可确认，每次停栈那句 `disable state unavailable` 是预期行为。

## 2026-10-01：电流模式 drag sweep 真机结果——**命令 45°，实测只走 2.46°**

现成路径逐层被探针纠正后才跑通：`--yaw-step-deg` 那支探针**命令电压**、电流环开着 ⇒ 驱动器不理它
（`exit=2`，探针自己的 NOTE）；`--yaw-sweep-deg` 还必须配 `--yaw-speed-deg-s`（文档：整数、±5、≤1500 raw）。
最终命令与探针原话（`/tmp/adr/sweep45.log`）：

```
run --commission-hardware --with-imu --yaw-sweep-deg 45 --yaw-sweep-ff-a 0.6 --yaw-speed-deg-s 5
RESULT reason=completed sweep_target_deg=45 sweep_ff_a=0.6 yaw_mode=current
       current_ceiling_a=3 yaw_speed_target_deg_s=5 peak_speed_deg_s=30
       peak_travel_deg=2.59 final_displacement_deg=2.46 stationary_observed=1
       yaw_frames=2807 yaw_tx=857 yaw_errors=0 zero_tx_failed=0
```

**读数**：探针自己按**度**报，不需要我拿编码器换算。**命令 45°、走完 2.46°、结束时观察到静止**、
CAN 无错误、护栏判定 `completed`。**这不是"没动"，是"只动了应有行程的 5%"**——
量级差一个数量级，方向没有异议。

**首要怀疑（下一轮验，不当结论）**：0.6 A 脱阻力前馈对付不了交叉滚子轴承的静阻力，
且 trace 里 yaw 的运行时 `current_cap` 只有 **0.8 A**（探针上限 3 A 是探针的，不是环的）
⇒ 速度环一顶到电流上限就再也推不动。可验证动作：**把 sweep 前馈抬到 1.5 A**（仍在探针 3 A 与厂商 ±3 A 之内）
复跑；若行程随之前馈明显增长，"能动不堵转"的正解就是**抬运行时 yaw 电流上限**（可写参数，0 改源码/0 重编/0 重部），
而不是继续动 kp/ki。注意 D7：**调参不得自己抬高电流包线**——这一抬属于**主人授权的诊断/资格**，须单独记账。

## 2026-10-01：drag sweep 两点对照——**行程跟着可用的脱阻力电流走**

同一支探针、同一速度目标（5 °/s）、同一 45° 命令，只改脱阻力前馈：

| `--yaw-sweep-ff-a` | `peak_travel_deg` | `final_displacement_deg` | `peak_speed_deg_s` | 判定 |
|---|---|---|---|---|
| **0.6** | 2.59 | **2.46** | 30 | `completed`、`stationary_observed=1`、`yaw_errors=0` |
| **1.5** | 11.38 | **11.21** | **84** | 同上 |

前馈 ×2.5 ⇒ 行程 ×4.6，**方向无争议：限制行程的是能用到环里的电流，不是 kp/ki**。
另两条读数值得记：`peak_speed_deg_s=84` 而速度目标只有 5 °/s，且 RESULT 里 `yaw_current_a=0` ——
**速度环没有把速度按住在 5 °/s**，脱阻力前馈在主导；轴是先窜起来、再在某个角度被阻力停下来
（`stationary_observed=1`）。**这正是主人说的"各角度阻力不均"的形状**，不是我此前任何一版猜测。

**还没到"能动"的定论**：11.2° 仍只有命令的 25%。下一手：前馈再抬一档（2.5 A，仍在探针 3 A 与厂商 ±3 A 之内），
看行程是否继续增长、以及**停在哪个角度**——那才是把"能动不堵转"落到一组参数上的依据。
D7 提醒仍然生效：抬高电流包线属**主人授权的资格动作**，不是调参自己开门，须单独记账。

## 2026-10-01：前馈 2.5 A ——**yaw 真的走大角度了（40°/50°），但会话 exit=2，速度环没在管**

同一支探针，`--yaw-sweep-ff-a 2.5 --yaw-speed-deg-s 5`：

| 命令 | `reason` | `peak_travel_deg` | `final_displacement_deg` | `peak_speed_deg_s` | launcher |
|---|---|---|---|---|---|
| 15° | `sweep_target_reached` | 15.25 | **40.65** | **186** | **exit=2** |
| 45° | `sweep_target_reached` | 45.04 | **50.36** | **222** | **exit=2** |

三条读法，好的坏的都写：

1. **能动成立（在量级上）**：实测位移 40.6° / 50.4°——比我报过的 0.26° 完全是另一个世界，
   也超过主人"能动"的字面要求。**轴不是被阻力摁死的。**
2. **但速度环显然没在干活**：目标 5 °/s，峰值 **186 / 222 °/s**，且 RESULT 里 `yaw_current_a=0`；
   `final_displacement` 大于 `peak_travel` ⇒ **越过反向点还在惯性滑行**。也就是说**全部权威都来自脱阻力前馈**，
   PI 那一项是空的——这与三轮 sweep 一致（0.6/1.5/2.5 A 时 `yaw_current_a` 都是 0）。
3. **`exit=2` 不是干净通过**：探针判 `sweep_target_reached`，launcher 仍以 2 退出（后续护栏/返回段的问题，
   原因未查）。**所以这一格我只能写"能动＝是；干净通过＝否"**，不拿 `target_reached` 冒充全程合格。

**由此得出的下一个真问题（也是调参的正题）**：为什么 yaw 速度环输出恒为 0 ——
是速度环增益被写成 0、还是被某个上限/开关掐住。若那一环活过来，"用 5 °/s 走完 45° 且不受阻力角度影响"
才是可达成的，`不堵转` 也才有意义（现在是前馈硬顶，不是闭环在跟）。

## 2026-10-01 更正与结论：**限制是 yaw 的电流包线本身，而它是受保护只读参数**

**先更正我自己**：上一轮我写"yaw 速度环输出恒为 0"，证据是 RESULT 里的 `yaw_current_a=0`。
读了 `tools/probe_mixed_hardware.cpp:506` 才知道那印的是**探针自己的推力电流变量**——我没传
`--yaw-current-a`，它就是 0。**那句话是误读，撤回。**
同一处还写着（`:419` 附近）探针作者引的主人 09-29 裁定：**"这是扭矩测试，不是速度测试"**，
任何形如运动极限的东西都不许结束一次扫掠。所以 **186/222 °/s 的峰值不是控制失效**，
是我拿速度测试的尺子去量扭矩测试——第二次因为没先读代码而挨这一刀。

**再看行程**：探针在 `travel >= |sweep_deg|` 时才判 `sweep_target_reached`，
所以 `peak_travel_deg=45.04` 是真的**拖着走完了 45°**（15° 那次 15.25°）。合上前两轮：

| 脱阻力前馈 | 拖过的角度 | 探针判定 |
|---|---|---|
| 0.6 A | 2.59° | `completed`（未达目标，时间窗用完） |
| 1.5 A | 11.38° | `completed`（同上） |
| **2.5 A** | **45.04°** | **`sweep_target_reached`** |

**最后是权限层的事实**（`parameter_inventory.json`）：

```
yaw.current_kp_a_per_rad_s   experiment_writable
yaw.current_ki_a_per_rad_s   experiment_writable
yaw.host_current_limit_a     protected_read_only     <-- 我量到的 0.8 A 上限就是它
yaw.current_limit_register   unsupported
```

⇒ **轴承要 ~2.5 A 才拖得动 45°，而生产环的包线是 0.8 A，且这个参数是受保护只读**——
调参面动不了它，ADR-002.1 D7 也不许调参自己抬包线。**这不是 bug，是一个需要主人点头的资格决定**
（profile 改动 + 重启，restart-class；厂商满量程 ±3 A，探针上限 3 A，2.5 A 在其中）。

**所以本目标的诚实结论**：`能动` 在**调参探针路径**下已证明（拖过 45°）；
`能动` 在**生产环 + 0.8 A 包线**下**未证明**；`不堵转` 在 Manual/Hold 下已过（静止 2–5% 包线），
AUTO_ROAM 的满包线保持仍未复量。要把 yaw 交给主人用，缺的是**抬 `yaw.host_current_limit_a`（建议 2.5 A）并重启**，
然后重跑扫掠与 AUTO_ROAM 保持电流。**这一步我不擅自做：它是包线资格，不是调参。**
