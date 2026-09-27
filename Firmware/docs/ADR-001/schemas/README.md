# 拟议契约，不是现有 ABI

采用 JSON Schema Draft 2020-12。本包离线工具无需第三方依赖；安装了 `jsonschema` 的环境可以另做结构检查。语义检查仍需检测帧时序、int64上界、不同字段的一致性和资格来源，不能只依赖schema。

所有absolute ns与64位帧序列必须是十进制字符串。数字形式即使数值相同也必须拒绝。shadow观测可以schema合法但time/geometry不合格；结构正确不授权运动。

`selected_observation` 是拟议的完整选定观测语义；`trace_event`/`stop_evidence` 是离线参考子集，完整站台日志仍须保留设计文档列出的额外阶段/证据；`validation_context` 只用于合成离线测试；`qualification_template` 只校验未运行的空模板。

文档里的 `selector_policy` 概念在schema中统一写为 `selection.policy`。
