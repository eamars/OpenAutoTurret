# 样例使用边界

本目录全部为**合成样例或空模板**，不是 Pi 实测捕获。`SYNTHETIC-*` 身份没有硬件含义。

`selected_observation.json` 与 `validation_context.json` 故意构造一个语义可接受的观测；其中 `geometry_valid=true`、`motion_qualified=true`、`verified` 只属于合成世界。`hardware_authorization=false` 不可删除后用作现场许可。

`synthetic_trace.ndjson` 有4帧：2帧可计算可信曝光到controller的28ms/34ms；1帧曝光时间是modelled，不能进入该指标；另1帧缺少controller_receive。不得把后两帧记作0ms或拿合成指标宣称硬件性能。

`stop_evidence.json` 仅演示新鲜反馈与pitch disable确认完整、yaw零命令已请求的有限完成情况。输出仍是 `limited_complete`，yaw禁能和整机断电确认仍为null。

`qualification_report_template.json` 保持NOT_RUN；其schema仅校验空模板。实际资格报告需要Codex结合所实现的证据字段扩展，不能只把status改成PASS绕过证据。

`n1_policy_proposal.json` 是人/agent可读的设计输入，不适配现有runtime parser。
