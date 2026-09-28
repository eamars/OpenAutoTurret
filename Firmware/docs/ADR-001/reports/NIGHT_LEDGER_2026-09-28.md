# 09-28 夜班总账（写给下一个干净会话；也是 WP9 收口的起点）

**硬约束（主人 09-28 19:0x–20:0x 令）**：所有开发本地进行；**不连 rpi-turret、不做任何实机测试**；
不停、不滥用 goal、**每做完一个 WP 再回 goal**。（他在刷 GM6020 固件。）

## 各包状态（详情见同目录资格报告）

| 包 | 状态 | 已证 | NOT_RUN 及原因 |
|---|---|---|---|
| WP0–WP2 | ✓（见各自历史报告） | 停止永不被 readiness 拒绝；归零途中停止被接受；四层同 `stop_id`；重复 stop 幂等**已在硬件验证**；相机身份耐久化上线 | WP2③ 需 sudo；WP2-N1 需落盘零点＋通电连续性 |
| WP3 | PARTIAL | worker/supervisor 自检 9/9、registry 10/10、synthetic 走 worker 的断言（变异检验过） | A3（worker 退出/卡死/backpressure）推到 MVP 后 |
| WP4 | PARTIAL | manifest 自检 8/8 **逐条点名拒绝**；真清单**今天不给资格**；预览半程闭合 3 例 | **C1** 跨镜留出集（需真机）；`input_size==stream_size` 绊线（归宿在检测器路径） |
| WP5 | PARTIAL | 两份清单事实**实测已分叉**→ 已对齐＋**防再分叉绊线**（变异检验过） | Hailo 真机、NMS/label/色序、离线分数分布（本容器无 `hailort`） |
| WP6 | PARTIAL | 电流准入六道条件＋**3 条离线用例**（**当场抓到"只校验不存值"的字段说谎**）；`1.62` 单一出处 | 单轴辨识、停止包络、S0 30 次、M0 归零 10 次 ≤0.5°（**全需通电运动**）；layer 4 探针 `--yaw-current-a` 未做 |
| WP7 | 仅清点 | AprilTag **绿地**；仲裁**已在** `reference_manager`；`fiducial` 已在标定/运动学头里 ⇒ AprilTag 可成 WP4 闭合真路径；关联**已在** `track_manager.update`（18 处 iou/overlap/gate） | 行为验证全部依赖真机真目标 |
| WP8 | 仅清点 | `imu_trace_ingest.{hpp,cpp}` **已存在且已接 `main.cpp`** | IMU 参与控制、UI 全部未动 |
| WP9 | 未开始 | 本账本即起点 | 发布收口需前面各包收口 |

## 等主人的三件裁决

> **本节已被 `OWNER_RULINGS_2026-09-29.md` 取代（09-29 03:4x 他睡前逐条答完，五条）。** 下面留原文，只作历史。

1. **F-WP4-1**：朝向词表宽于实现（图像路径无 90/270）——收窄词表，还是补实现？（我倾向收窄）
2. **电流模式何时切**：代码层已就绪且**不改变现网行为**（YAML 仍 `voltage`）；切换前提是**固件 ≥ v1.0.11.2 ＋ Current Ring 已开**——09-28 实测该驱动对 ±0.6 A 电流帧**统计无响应**。
3. **部署队列**（一次 `--prebuilt --activate` 能带走全部）：`visiond` 的 `info` 崩溃修复、yaw `max_output_counts` 配置管道（**A/B 至 6000 无饱和事件 ⇒ 未定论，按发现记录**）、电流模式配置层。

## 测试基线（勿跨 boot 拼统计）

- C++：`ctest -E "retained_homing"` **77/77**（`retained_homing` 写 `/dev/shm`，容器内按仓库 AGENTS.md 排除）。
- Python：`cd Firmware && .venv/bin/python -m pytest -q --ignore=legacy` → **17 failed / 991 passed / 24 skipped**（**09-29 更正：今天同一棵树两次连跑都是 18 failed / 990 passed，两次 FAILED 集合完全相同**。多出来的那条就是下面这条具名飘红——它单跑 3/3 全红，在套件里也跟着红。**别再把它当「偶尔红」，它现在是常红**，随测试轮一起结）（17 为既有红；**跑子集会因测试内 `os.chdir` 给出假计数**）。
- 已具名飘红：`vision/tests/test_ipc_publisher.py::test_reconnect_after_the_daemon_dies_is_caller_policy`（跨运行翻转，推到 MVP 后的测试轮）。

## 这一晚的三条方法教训（已写进各报告，这里只留骨头）

1. **红先复现、再二分，因果是跑出来的**：我两次凭半截阅读给错因果（`check_keys` 必填性、站点 4 嫌疑）。
2. **`ctest` 在旧二进制上会全绿**：构建判定用 `grep -c "error:"`，不看测试计数。
3. **读函数要读到 `return`**；**没落到文件里的事等于没发生过**。
