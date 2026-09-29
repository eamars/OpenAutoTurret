# 开发包离线验证记录

日期：2026-09-29。Python：3.13.5。

| 检查 | 结果 | 范围 |
|---|---|---|
| `python -m unittest discover -s tests -v` | 45 / 45 PASS | 数学辅助与CSV分析工具，不是固件测试 |
| control_math CLI | PASS | 简化PI计算；5°/s示例约5.97s，不是实机测量 |
| trace_metrics CLI | PASS | 402条合成记录、4个连续分组；未知电流/温度保留null |
| JSON解析 | PASS | 来源manifest、记录模板与工具输出 |
| 文档相对链接/代码围栏 | PASS | 13个本地链接检查 |
| 仓库C++编译/测试 | NOT_RUN | 未下载全库或构建仓库 |
| Pi/CAN/电机实机验证 | NOT_RUN | 未连接硬件、未发送命令 |
| payload/热平衡/最大电流资格 | NOT_RUN | 模板保持未合格 |

测试中一个温度斜率断言最初使用浮点精确相等而失败，已改为数值近似相等；计算实现未为测试改成硬编码结果。最终完整测试输出见`offline_tests.log`。

这些PASS不代表新控制器已集成、已部署，亦不代表任何实际电机/温度/起动电流参数合格。
