# 事故验证报告：整栈在无人操作时下线（2026-09-28）

状态：站点已恢复运行（release `8b0f4d4`，本次调查后重启）。
本报告是日期化测量记录，不是平行事实来源；事发后读数以站内为准。

## 1. 一句话结论

01:53:04（本地，UTC+13）这台站在**没有人调用 stop、没有定时任务、不是 ssh 会话断掉**的前提下自己走了清理路径：
launcher 结束 → controld 受控停车 → `MIXED STOPPED`（pitch disable 已确认，yaw 零请求、disable 态不可得）。
**最可能的触发者是 visiond 先退出**（内核在同一秒报相机 I2C 锁死），
但**这条没有证实**：旧 `vision.log` / `web.log` 在我开始调查之前已被重启截断。
**原因不可读本身是缺陷**，本切片把它变成可读的（见 §5）。

## 2. 时间线（本地时区 = UTC+13；`Z` 列为 UTC）

| 本地 | Z | 事件 | 来源 |
|---|---|---|---|
| 00:32:46 | 11:32 | `New session 196`（部署用的 ssh 会话） | journalctl |
| 00:32:50 | 11:32 | `Session 196 logged out. Waiting for processes to exit.` ⇒ **会话已退，站台留在里面** | journalctl |
| 00:36:02 | 11:36 | 热实验 logger 第一条：`station={"phase":"hold","mode":"MANUAL"}` ⇒ 栈活着 | `/tmp/ota-stack-1000/thermal_experiment.jsonl` |
| 00:41:50 | 11:41 | `New session 213`（挂 logger 的 ssh） | journalctl |
| 01:52:52 | 12:52 | logger 最后一条仍带 `station` 段 | 同上 |
| **01:53:03** | 12:53 | `kernel: i2c_designware 1f00074000.i2c: i2c_dw_handle_tx_abort: SDA stuck at low` | journalctl / dmesg |
| **01:53:04** | 12:53 | `kernel: i2c_designware …: controller timed out`；**同秒** controld `shutdown requested; controlled stop` | 同上 + controller.log |
| 01:53:05 | 12:53 | `MIXED STOPPED`；`controld stopped cleanly`；`session-196.scope: Deactivated successfully` / `Removed session 196` | controller.log + journalctl |
| 02:17:52 | 13:17 | logger 仍在写，但 `station=null`（HTTP 不可达） | jsonl |

`wait -n` 之前的 launcher **不记录是哪个子进程先退**，脚本末尾直接落出 → EXIT trap → 清理。
所以现场只留下"清了"，没留下"谁引起的"。

## 3. 排除项（每条都可复跑）

| 假设 | 判据 | 结论 |
|---|---|---|
| 人按的 stop | 他说不是他；`web.log` 无 `POST /api/stop`（但该文件已被重启截断，证据不完整） | 存疑，倾向否 |
| ssh 会话断掉连带 | `Removed session 196` 发生在 **01:53:05**，而 `logged out` 在 **00:32:50**；站台在会话之后又活了 80 分钟 | **否**（会话是被站台之死回收的，不是相反） |
| logind 清理用户进程 | `/etc/systemd/logind.conf`：`#KillUserProcesses=no`（默认） | **否** |
| 定时任务 | `crontab -l`＝无；`atq`＝命令不存在 | **否** |
| 代码里的空闲/租约自动停机 | 通读 `run_application.sh`、`web/webd/app.py`、`perception/visiond.py`：无空闲/租约到期触发停止 | **否** |
| 我自己 02:31 的重启造成的 | 停机时间在 01:53，重启在 02:31 | **否** |
| 相机 I2C 锁死拖死 visiond | 内核两行在 controld 报停机**之前** 1–2 秒；`i2c_designware 1f00074000` 是相机那条总线 | **最可能，未证实** |

## 4. 我自己毁掉的证据（记在这里，不藏）

我在 02:31 为了让 WP1 继续跑而重启了站台，**而 `$RUN=/tmp/ota-stack-1000` 的日志在 start 时被复用/截断**：
旧 `vision.log`、`web.log` 没能保留。正确顺序是先归档再重启——**当时没有"先归档"这个动作，所以现在只能给"最可能"而不是"已证实"**。
这条已经变成代码（§5 第 2 条），不是"下次注意"。

## 5. 本切片改了什么（原因可读）

1. **停机凭证**：`stop` 在发 TERM 之前写 `$RUN/stop.request`（调用者 pid/uid/UTC/目标 launcher），清理时消费并删除。
2. **日志不覆盖**：`run` 启动前把上一轮 `*.log`、`shutdown.result`、`shutdown.cause`、`stack.info` 移到 `$RUN/logs-history/<utc>Z-launcher<pid>/`，只保留最新 10 轮。
3. **死因归因**：launcher 结束前写 `$RUN/shutdown.cause`，取值 `operator_stop` / `child_exit` / `external_signal` / `unattributed`，并带 `exited_child`、`exited_pid`、`wait_status`（`137(signal 9)`＝SIGKILL；`wait_status=0`＝该子进程自己跑完，例如有界采集）。`wait -n` 的返回码以前被丢掉，现在带出来。
4. **可读出口**：`status` 在停机态先打 `shutdown.cause`，再打停车结果。
5. 运行流程写进 `docs/STATION_OPERATIONS.md`「Who stopped it」；验收工具 `tools/rehearse_stop_cause.py`。

## 6. 四类验收（诚实登记）

| 类别 | 命令 | 结果 |
|---|---|---|
| 静态/单元 | `python3 tools/rehearse_stop_cause.py --selftest` | **8/8**（判据逻辑，含"凭证优先于信号"这条钉住） |
| mock/replay | 同上 | 同上（容器内无站台即可跑） |
| camera-only | — | **NOT_RUN**：本切片不动相机路径 |
| 现场 motion | `tools/rehearse_stop_cause.py`（三场景各含一次归零） | 见 §7 |

## 7. 现场排练结果

**已跑（本地 03:12–03:27，release `9ee0285fab37`，站点被我起停 4 次，每次含归零）**：

| 场景 | 触发 | 站点写下的 `shutdown.cause`（摘录） | 判定 |
|---|---|---|---|
| 1 | `run_application.sh stop` | `cause=operator_stop … operator="who=operator pid=3615811 uid=1000 …" signal=TERM` | PASS |
| 2 | 裸 `kill -TERM launcher` | `cause=external_signal … signal=TERM`（**没有 operator 凭证**） | PASS |
| 3 | `kill -KILL visiond` | `cause=child_exit … exited_child=visiond exited_pid=3678233 wait_status=137(signal 9)` | PASS |
| 4 | 连续重启后查归档 | `logs-history/` 存 5 轮，每轮带自己那份 cause | PASS |

四次停车结果都是 `STOPPED (pitch disable confirmed; GM6020 yaw zero requested, disable state unavailable)`——
**包括子进程被 SIGKILL 那次**。⇒ 本切片的主张成立：**归因变清楚了，停车行为一点没动。**
另有一次真实部署停机也留下了完整凭证（`uptime_s=653`，pid/uid 全在）。

**我自己不老实的一处（已改）**：排练里的 ready 判定打的是 `/api/health`，而那份回答**不含 `phase`** ⇒
三个场景各把 300 s 超时**等满**才继续——排练"通过"了，但"栈确实起来了"当时并没被证明。
已改打 `/api/state`，并把 ready 变成一条真断言（selftest 10/10）；**带断言的重跑记 NOT_RUN，随下一次部署。**

**顺带的复发观察**：这 4 轮合计约 15 分钟的 AUTO_ROAM/AUTO_TRACK 里，`no_progress` **一次都没复现**。
见 `OBS_NO_PROGRESS_TRIP_2026-09-28.md`。

## 8. 剩余限制与下一步

- **`child_exit` 只到"哪个子进程先死"这一层**。visiond 为什么死（I2C 锁死）需要 visiond 自己的退出原因落到 `shutdown.cause` 可指向的地方——归 WP3（worker 生命周期/隔离）与 A3。
- `external_signal` 拿不到发送者 pid（bash trap 里没有 `siginfo`）。要更细得让 `stop` 之外的发送者留凭证，或把 launcher 监督逻辑挪进 systemd——本切片不发明。
- 本报告**不改变**温度公案的结论（见 `INCIDENT_YAW_GUARD_TRIP_2026-09-27.md` 与其后修订）：那次是守卫跳闸，这次是整栈清理，两件事不同因。
- 现场还欠一条：**相机 I2C 锁死是否会复发**。`logs-history` 从现在起会替我留着现场。
