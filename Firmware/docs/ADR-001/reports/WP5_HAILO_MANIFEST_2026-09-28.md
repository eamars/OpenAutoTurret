# WP5 起点（2026-09-28 深夜，本地；站离线）

范围（`07_WORK_PACKAGES.md:24` 任务表第 5 行）：Hailo manifest **单一事实**、NMS/label/色序检测、
离线精度评估；触及 `perception/model/hailo_yolo.py`、`model/manifests`、`config/hailo_yolov8n_manifest.json`；
**Done/不做：实际返回分数分布可解释；不切最新 YOLO、不安装全套 Hailo。**

## F-WP5-1：清单**确实有两份事实**，且**今天已经分叉**（本轮量出来的）

比对 `config/hailo_yolov8n_manifest.json` 与 `perception/model/manifests/*.json` 的同名标量字段：

| 字段 | config 侧 | 包内侧（四个清单一致） |
|---|---|---|
| `model_id` | `hailo_yolov8n_coco_hailo8_v2_17_0` | `hailo-yolov8n-coco-hailo8-…`（**连字符**） |
| `task` | `"COCO object detection"`（散文） | `object_detection`（**枚举**） |

⇒ 对**四个**包内清单都是同样的两项不一致。这不是"可能有重复"，是**两份都在被使用、值已经不同**：

- `config/hailo_yolov8n_manifest.json` 由 **`tools/probe_hailo_camera.py`** 读取（活的，不是死文件）；
- 包内清单是感知管线路径读的（`model/manifests/…`，被 `perception` 配置以相对包路径引用）。

## 因此 WP5 第一刀的形状（需要清醒的一轮来做，勿仓促）

1. **定唯一真相**：包内 `model/manifests/*.json` 为准（它被运行时读取）；config 侧要么**删除**、
   要么**只留引用**（例如只写 `manifest: model/manifests/hailo_yolov8n_hailo8_coco.json`）。
2. **`task` 词表统一为枚举**（`object_detection`），散文值视为错误——否则任何按 `task` 分支的代码
   都会在一份副本上走错路而**不报错**。
3. 改之前必须先看 `tools/probe_hailo_camera.py` **怎么用** `model_id`/`task`：
   若它拿这两个值做断言或匹配，统一真相会改变工具行为（那是好事，但要说出来，不能悄悄变）。
4. 收口用 WP3/WP4 同款做法：**自检必须包含一条会红的断言**（例如"两份事实不得同时存在"），
   否则下次再分叉没人红。

## 本轮不做也不声称

Hailo 真机运行、NMS/label/色序行为、离线精度分布（都需要真模型/Hailo，站离线）；
`model_id` 命名风格的裁决（下划线 vs 连字符，等主人或等运行时真相统一后自然消失）。
