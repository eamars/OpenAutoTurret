# 合成样例

运行 `python tools/adr0021.py demo --output reports/demo` 生成六份 CSV：

smooth、stalled、creep、oscillating、drifting、spike。

数据完全由解析函数合成，不是原始站台 trace。这里使用近似 0.044° 编码器量化、50 Hz 独立采样；这只是测试样例，不是对真实设备采样频率或编码器的重新核定。

已生成样例随包放在 `reports/demo/`，并附完整 metrics 和 summary。
