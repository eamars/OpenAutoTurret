# 开发包本身的验证报告

日期：2026-09-27。验证对象是本包，不是 GitHub 工作区或 Pi 站台。

## 实际完成

| 检查 | 结果 | 证据 |
|---|---|---|
| 标准库离线单元测试 | **107 项全部通过** | `unit_tests.txt` |
| JSON Schema Draft 2020-12 | **5 份 schema 有效** | `schema_validation.json` |
| 合成样例结构 | **27 条正例通过，6 条反例正确拒绝** | `schema_validation.json` |
| CLI 合成 trace 汇总 | PASS，缺失/不可信时间未补零 | `synthetic_trace_summary.json` |
| CLI 选定观测语义样例 | PASS，但资格仍 NOT_EVALUATED | `synthetic_observation_validation.json` |
| 0x50005 位解码样例 | 当前欠压/节流与历史位分别保留 | `power_bits_example.json` |
| 合成停止证据 | limited_complete；yaw disable/整机断能均为 null | `synthetic_stop_evidence.json` |
| Python 语法检查 | 3 个 Python 文件通过 | AST parse |
| 原始交接书复制与 SHA256 | 与上传文件字节一致 | `sources/provenance.json` |

测试环境：Python 3.13.5；可选 schema 验证器 jsonschema 4.26.0。核心参考工具和107项测试只依赖标准库。没有自动安装依赖。

运行命令：

```bash
python -m unittest discover -s tests -v
python tools/offline_checks.py summarize examples/synthetic_trace.ndjson
python tools/offline_checks.py check-observation examples/selected_observation.json examples/validation_context.json
python tools/offline_checks.py decode-power 0x50005
# 仅已安装 jsonschema 时执行；不会自动安装。
python tools/validate_schemas_optional.py
```

覆盖包括：时钟/相机 generation 隔离，过期/倒退/重复帧，selection 撤销与重新选择，显式目标丢失后不自动替换，Hailo letterbox反变换，180°变换，未知标定/时间质量拒绝，停止请求与禁能证明分离，NaN/Infinity/数字溢出与重复JSON键拒绝。

## 明确没有完成

本次没有运行当前仓库的完整回归套件，没有针对真实native ABI编译/集成，没有打开相机/模型/CAN/I2C或SSH，也没有供电、pitch归零、制动、停止、人物精度、双摄标定、跨摄身份或IMU融合的新增现场结果。**107项通过只适用于参考工具，不等于WP0–WP9已实现。**

官方网页与分支文档选择性阅读；没有clone/整库归档或权重下载。未独立取得分支当前完整HEAD；`8901808`是交接书报告的最近激活版本，不是本包锁定的源码commit。

所有样例使用SYNTHETIC身份；空资格报告保持NOT_RUN。schema验证不能证明机械安全、统计可靠性或运动授权。
