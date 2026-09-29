# 离线参考工具范围

`adr0021.py` 是一个**离线合同参考**，不是完整自动整定器，也不是可直接驱动站台的 runner。

已实现：固定种子分层候选生成、有界细化、plan 哈希、参数 registry 写入校验、应用回执阻断 mock、固定编码器 jitter/跟随/漂移评价、完整 case 聚合、确定性排序/下一阶段与重试规则。

未实现：真实 UDS adapter、电调寄存器事务、固件插桩、实时质量淘汰、原始 CAN 解析/编码器去回绕、10 秒静止噪声资格、姿态/轨迹安全几何检查、硬件温度资格、完整实机 campaign 生命周期和生产晋升。以上由 Codex 依本包接口在现有控制路径实现，不得因本工具运行成功便宣称实机自动整定器完成。

工具的 CSV 输入必须是已经去重、去回绕、单位明确的 RX 样本。`be_cmd_dps` 必须来自最终内环输入；工具不能仅凭字段名字证明其物理来源。

```bash
python tools/adr0021.py plan --spec manifests/search_space.example.json --output reports/plan.json
python tools/adr0021.py demo --output reports/demo
python -m unittest discover -s tests -v
```

默认域文件明确为 OFFLINE_ONLY，并拒绝以 physical_execution_authorized=true 运行参考 planner。将来实机授权应来自现有站台许可链，不是把这个布尔值改成 true。
