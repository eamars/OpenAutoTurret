# 外部审阅简报（2026-09-28 19:4x）

给正在读我们代码的审阅者。目的不是自述成绩，而是把**证据边界**标清楚：哪些是现场量到的，
哪些只是代码写了、哪些是还没做的。按仓库规矩，未执行写 `NOT_RUN` 并给原因。

## 已被硬件证据钉住

| 主张 | 证据 |
|---|---|
| 停止请求不再被 readiness 拒绝（含"归零途中收到停止"） | 三模式下 `stop_motion` 全部 `ok:true`、零拒绝；站上 `stop-evidence.ndjson` |
| 停机证据可按 `stop_id` 拼接 | `stop-9236596751477`：`requested(unverified, 点名缺项)` → `parked(verified)` |
| 重复 stop 幂等（不后推 deadline、不换身份） | 运动中双发 `request_shutdown` ⇒ 新增行里 `stop_id` 数＝1 |
| yaw 现在能动 | 驱动上限抬到 15000 后 `vout` 达 -11452、`q_yaw` 由 -0.697 走到 7.75 rad |
| 相机身份可耐久 | `camera identity cam-baa28c2a source=by-path durable=True`；`realpath(/dev/video0)` 唯一命中该 by-path 名 |
| 站端零编译可行 | `--prebuilt` 连续多次 `65 binaries / 0 failed`（arm64 在本机交叉编） |

## 代码写了、但**尚未**在硬件上证明

- `axes.yaw.max_output_counts` 这条链（YAML → 校验 → `set_yaw_output_ceiling` → 速度环）。
  已证：键进了 release、构建与 77/77 通过。**未证**：改这个值会改变观察到的 `vout` 上限。
  A/B 做过一次**不成立**：上限压到 6000 后 roam 25 s 内没出现顶满，故无证据（不是"通过"）。
- 归零为何被强制的启动说明（`homing retention: unavailable on this profile`）——日志已见，
  但 `zero_source` 遥测字段还没做（条目 **WP2-N1**）。

## 我认为值得外部审阅者优先质疑的四处

1. **`vout` 的物理含义取决于 GM6020 固件代次**：我们从没读过设备固件版本；本地指南 v1.4 记 ±25000，
   而文档自述 v1.0/v1.2 记的是另一范围（-30000..）。所以"15000＝满量程 60%"这句话**可能是错的**。
   电流模式是否为该固件所支持，也未验证。
2. **堵转守卫可能在低驱动时也判 stall**：现场有 `stall #3` 而 `vout` 仅 -568/+1232/-1921 的行。
   若守卫在"驱动本来就没到位"或"驱动器被断电"情形下仍计一次 stall，它会在不该 hold 的时候 hold。
   （`kNoProgressLimitNs=1.5s`、`kStationaryToleranceRad=0.5°`、`kNoProgressCommandRadS=5 deg/s`）
3. **yaw 阻力是滑环固有的、且随角度不均匀**（主人手测，非今日新增）。因此"单一驱动上限"这个设计本身
   可能就偏薄：WP6 要的是 `vout` vs 角度的曲线，而不是再猜一个数。
4. **`RetainedHoming` 不能给连续旋转 yaw 免归零**（`control_loop.cpp:581` 直接 `return false`）。
   免归零需要新凭证：落盘会话零点 + 通电连续性 + 与当前 encoder 一致。这个设计我欢迎外部挑刺。

## 目前 NOT_RUN 的项与原因

- **WP2 ③**（CAN1 断、CAN0 有效时停止语义）：需要 `ip link set can1 down`＝sudo，执行环境无 sudo。
- **S0 30 次 / M0 10 次 / A3 / T2**：主人裁决"测试押后到 WP3–WP9 的 MVP 之后"。S0 现有 3/30，记 IN_PROGRESS。
