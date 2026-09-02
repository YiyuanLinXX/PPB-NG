# HSI Internal 连续采集硬件验证记录（2026-09-01）

## 安全范围

本轮使用真实 FX10e（Pleora GigE）与 SWIR3（NI Camera Link/PCU），不连接 RGB、thermal、
GNSS、RSM400 或 Arduino。每次测试均执行 `prepare -> arm -> start -> begin_dark ->
start_sample -> stop`，停止时关闭快门、注销 callback、关闭 handle 并释放 SDK。测试数据保留
在 workspace 的 `data/` 下，不覆盖既有数据。

## FX10e 单机

首次真实采集正确保存了像素，但校验暴露暗场结束后 SDK frame number 从 1 重启，而旧代码
仍把 dark 与 sample 放在同一逻辑 segment。该批次保留为失败证据：

- `data/fx10e_d3_20260901_1840`
- 250 dark lines；2347 sample lines；约 2.38 GB。
- dark 实测约 50.61 Hz；sample 实测约 50.09 Hz。
- 失败原因：dark/sample 边界的 frame sequence reset 未建立新 segment。

修复边界语义后重测：

- `data/fx10e_d3_retry_20260901_1846`
- 2 segments，1314 lines，1,205,600,256 bytes。
- dark 和 sample 分离；ENVI header、raw extent、index/timestamp 对齐、逐行 CRC32、frame
  sequence evidence 全部通过。
- `ppbng_verify_dataset ... --hsi-only fx10e` 返回 `valid=true`。

## SWIR3 单机

- `data/swir_d3_20260901_1848`
- 2 segments，512 lines，113,246,208 bytes。
- profile `SWIR3 with NI`、serial `462111`、geometry `384 x 288 x 2`、工厂标定包均读回
  正确。
- ENVI header、raw extent、index/timestamp 对齐、逐行 CRC32、frame sequence evidence
  全部通过。
- `ppbng_verify_dataset ... --hsi-only swir` 返回 `valid=true`。

## 验证工具修复

真实数据测试发现并修复了验证工具的两个问题：

1. CLI 原来只在 session 根目录寻找 ENVI index，但生产 writer 正确写在 `segments/`；现同时
   识别目录并从 `segments/` 验证四文件集合。
2. 旧 verifier 强制每行必须有非零 external trigger/controller evidence，会错误拒绝 Internal
   连续行。现只在 `trigger_sequence/pps/controller/UTC` 全为零、time status 为 unsynced、
   association 为 unverified 时接受，并新增回归测试；不会把缺失外部时间证据伪装为已同步。

新增 `--hsi-only fx10e|swir`，可验证尚未生成完整 manifest/FrameContext 的单设备 D3 数据。

## 第一轮双机结果与根因

两个独立 `hsi_production_node` 进程同时初始化时，FX10e 成功而 SWIR 首次 start 失败；顺序
初始化则两台均成功。顺序初始化后并行采集可以产生两路数据，但发生非必要 recovery：

- `data/dual_hsi_d4_seqinit_20260901_1854`
- 各个已完成 segment 内部均无 SDK frame gap。
- FX10e 出现 1/3/5 等不连续的已写 segment 编号；SWIR/FX 均触发恢复。

确认的两个软件问题：

1. 两个独立进程同时执行完整 SDK 初始化会互相干扰，因此初始化事务必须跨进程串行化；
   但这不等于两台相机应共享进程，后续真实测试证明共享进程反而会触发 Pleora 崩溃。
2. watchdog 在一次最多 32 帧的 CRC+同步写盘批次开始时缓存 `now_ms`，随后每个健康帧都用
   这个过期值重设 deadline；高负载下会把刚保存的帧误判超时。现改为每帧成功持久化后重新
   读取 monotonic time。

同时把 recovery segment 改为 lazy commit：只有新 epoch 真正产生第一帧时才递增 segment；
连续多次无帧 recovery 不再制造 1/3/5 的磁盘编号空洞。

## 当前结论

- FX10e 单机数据路径：PASS-HW（短测）。
- SWIR3 单机数据路径：PASS-HW（短测）。
- 同进程双 handle：FAIL-HW，已禁用。两次分别在 `PvPersistence64.dll_unloaded` 与
  `PtConvertersLib64.dll` 内以 `0xc0000005` 崩溃；第二次崩溃前两路已并行采样，排除仅是
  初始化顺序问题。
- 最终候选结构：FX10e、SWIR 各自独立进程，完整初始化/停止/恢复生命周期用 Windows named
  mutex 跨进程串行化，采集阶段并行。这样避免 Pleora/NI callback 共用地址空间，也让单路
  SDK 阻塞或崩溃不会拖垮另一相机。
- `data/dual_hsi_isolated_d7_20260901_1932` 已证明双进程并行采集、正常 stop、Ctrl+C 干净退出；
  SWIR 首帧门限过短造成一次非必要恢复，已通过独立的 10 s 首帧宽限修复。
- `data/swir_worker_restart_d9_20260901_1946` 已证明 SWIR worker 被终止后，无需重启 Windows
  即可在新进程重新识别 462111、完成 dark/sample，并通过 CRC/序列校验：2 segments、
  1350 lines、298,598,400 bytes。
- `data/dual_hsi_soak_d10_20260901_1952` 在约 13 min 暴露 FX callback queue 容量不足：
  128 行缓冲在同步 flush/8 GiB rollover 压力下产生 overflow。故障被明确发布，SWIR 未中断；
  该批次保留为失败证据。FX queue 随后提高到 512 行（约 10.24 s、448 MiB）。
- `data/dual_hsi_soak_d11_20260901_2005` 完成 31.4 min 干净双机采集：FX 观测
  50.00 Hz，SWIR 20.59 Hz；两者始终只有 dark segment 0 与 sample segment 1，零 recovery、
  零 ROS fault、零新增 WER，正常 stop 后快门/handle/SDK 均释放，Ctrl+C 进程干净退出。
- D11 完整只读 verifier（逐行 CRC，非抽样）通过：FX 2 segments / 12 parts /
  94,467 lines / 86,673,850,368 bytes；SWIR 2 segments / 2 parts / 38,916 lines /
  8,607,596,544 bytes。
- GPS/RSM/RGB/thermal 未连接，本轮不能宣称完成跨设备同步。
- Internal 模式当前保存 SDK frame number 与 callback host monotonic time；尚无逐行曝光硬件
  时间戳，不能宣称曝光时刻已精确锁定 GPS。

## 120 分钟任务提前停止后的诊断与加固

`data/hsi_long_test_01_20260901_214915` 由用户在约 8 分钟时正常提前停止。数据本身没有被
覆盖或删除。只读分析得到：

- FX10e 为 segment 0–7，20,177 行、18,512,478,208 bytes；SWIR 为 segment 0–1，
  10,987 行、2,430,148,608 bytes。
- 每个 FX segment 内部仍约为 50 Hz，但 segment 之间存在约 17–21.5 s 的真实采集空档；按
  机器人 0.2 m/s 计算，每次对应约 3.4–4.3 m 未覆盖距离。因此这些 segment 不能被解释为
  普通的文件分片。
- `C:\ProgramData\Specim` 的日志时间与 FX segment 边界一致，显示 SDK 在每个边界发生
  unload/reload/re-enumeration；不是 8 GiB `part` rollover，也不是网卡收包错误。
- 该旧脚本没有把瞬时 ROS `FaultEvent` 持久化，无法在事后严格区分 callback queue overflow
  与 invalid callback frame。结合多个 segment 在 32 行批次边界结束，当前最强假设是消费/
  写盘停顿导致 callback queue 健康边界，但必须由下一轮新增计数器证据确认，不能写成已证实。

针对该事件已实施：

1. sample 阶段出现 timeout、callback overflow 或 invalid frame 时默认 fail-fast：立即结束当前
   segment、停止该相机并关闭快门，不再自动重连后继续制造看似连续的数据。
2. 每台 HSI 在 dataset `segments/` 中创建独占的 `<camera>_events.ndjson`，逐次 flush 保存状态、
   fault、队列 capacity/depth/high-water、overflow/invalid 累计值，以及 writer 最近/最大调用
   延迟。目标已存在时拒绝覆盖。
3. 长测脚本每秒检查 fail-fast 证据；一旦发现会用红色 CRITICAL 提示并安全停止两台相机，
   同时保存配置副本、配置/可执行文件 SHA-256 与本轮新增 Specim 日志。
4. 校验器新增明确区分的 `--quick` 模式：全量检查 index/timestamp/header/文件 extent，CRC
   检查首行、尾行及每 128 行；输出 `payload_crc_complete=false`，不会冒充逐行完整 CRC。
   默认不加 `--quick` 仍逐行 CRC。旧 19.5 GiB 数据的 FX 与 SWIR 快速校验分别约 9.6 s 与
   5.2 s 完成并返回 `valid=true`；这只证明现存部分内部结构和抽样 payload 正确，不消除
   segment 间的采集空档。

离线构建、PowerShell 语法检查及 `ppbng_hsi + ppbng_bringup` 回归测试通过：441 tests，
0 failures。

## 加固后的真实硬件复测

### FX10e 单机 15 分钟

数据集：`data/hsi_fx10e_diag_after_failfast_05_20260901_222933`

- 全部通过 ROS 2 `prepare -> arm -> start -> begin_dark -> start_sample -> stop` 完成；没有使用
  eBUS GUI。
- 250 dark lines；45,661 sample lines；sample sequence 1–45,661 连续无缺号。
- 首末 sample callback 跨度 913.113 s，观测 50.004752 Hz。
- 只有预期 dark segment 0 和 sample segment 1；sample 被正常拆为 5 个 8 GiB part 加最后
  一个 part，rollover 未关闭快门、未重载 SDK。
- queue high-water 289/512；writer 最大一次调用 849.006 ms；overflow、invalid、dropped、
  lost、FaultEvent 均为 0。
- 42,123,526,144 bytes；快速结构/extent/抽样 CRC 校验通过。stop 明确确认快门关闭、handle
  与 SDK 释放，且无残留进程。

### 双 HSI 30 分钟

数据集：`data/hsi_dual_30min_after_failfast_02_20260901_224811`

| 指标 | FX10e | SWIR3 |
| --- | ---: | ---: |
| sample lines | 90,950 | 37,540 |
| sample 时长 | 1,818.879 s | 1,823.145 s |
| 观测频率 | 50.002775 Hz | 20.590247 Hz |
| segments（含 dark） | 2 | 2 |
| parts（含 dark） | 11 | 2 |
| queue high-water | 140 / 512 | 1 / 128 |
| writer 最大调用延迟 | 1,234.263 ms | 1,171.313 ms |
| overflow / invalid / dropped / lost / faults | 0 / 0 / 0 / 0 / 0 | 0 / 0 / 0 / 0 / 0 |

- FX 83,676,364,800 bytes、SWIR 8,326,029,312 bytes；总 dataset 约 85.7 GiB。
- 两路快速结构/extent/首尾及间隔 CRC 校验均返回 `valid=true`；快速模式明确标记
  `payload_crc_complete=false`，没有伪装成逐行 CRC。
- 两个 ROS stop 均确认快门关闭、handle/SDK 释放；保存两份 vendor 日志；没有残留 HSI 或
  verifier 进程。
- SWIR vendor 日志在初始化串口探测时记录一次 `AIM SWIR error: 0 bytes returned`，随后立即
  在 COM2 找到相机、切换 Integrate-then-read，并连续完成 37,540 行，未对应 ROS fault、
  丢行或恢复。保留该原始日志，不把这条孤立初始化 warning 当作采集失败。

复测过程中还修复了四个只在干净终端/SSH 式启动中暴露的脚本问题：不再依赖
`Get-FileHash` cmdlet；`ros2.cmd` 显式加入 ROS Scripts PATH；后台 launch 完成环境加载前不再
并发启动第二个 ROS setup；双机选择参数不再被 PowerShell 大小写不敏感的循环变量覆盖。

## CRC32 热路径优化与后台运行复测（2026-09-02）

用户批次 `data/my_hsi_test_03_20260902_000530` 在 sample 约 85 s 后由 fail-fast 正确停止：

- FX callback queue 达到 512/512，新增 overflow 2、`samples_dropped=2`；没有 invalid frame，
  因此不能把这次故障解释为相机或 GigE 数据包损坏。
- 已落盘的 FX 3,994 行和 SWIR 2,043 行结构/extent/抽样 CRC 均有效；这只证明现存数据有效，
  不会掩盖两条未入队的 FX 行。
- 两台 ROS stop 均确认快门关闭并释放 SpecSensor handle/SDK。

根因位于存储热路径：原 IEEE CRC32 对每个 payload byte 再循环 8 个 bit。FX 每行
917,504 bytes、50 Hz，相当于约 3.67 亿次 bit step/s；故障批次最近 writer latency 为
22.602 ms，长期超过 20 ms 行周期，512 行缓冲只能延迟而不能消除 overflow。

`ppbng_storage::payload_crc32` 已替换为完全等价的 256-entry table-driven IEEE CRC32：

- 文件格式、polynomial、初值、反相规则和现有 checksum 不变；标准向量
  `123456789 -> 0xCBF43926` 加入单元测试。
- 新校验器对故障批次 3,994 行、3.664 GB 执行逐行完整 CRC 成功，约 18.6 s，证明可读旧数据。
- `ppbng_storage + ppbng_hsi + ppbng_bringup` 共 442 tests，0 failures。

真实 ROS 双机验收批次：`data/hsi_crc_optimized_regression_03_20260902_002348`：

| 指标 | FX10e | SWIR3 |
| --- | ---: | ---: |
| sample lines | 16,287 | 6,598 |
| sample 时长 | 325.720 s | 320.390 s |
| 观测频率 | 50.00 Hz | 20.59 Hz |
| queue high-water | 7 / 512 | 1 / 128 |
| writer 最大调用延迟 | 290.239 ms | 40.521 ms |
| overflow / invalid / dropped / lost | 0 / 0 / 0 / 0 | 0 / 0 / 0 / 0 |

FX 共保存 15,172,763,648 bytes，正常跨过一次 8 GiB part rollover；SWIR 保存
1,482,153,984 bytes。两路 quick verifier 均 `valid=true`，退出码 0；stop 确认快门关闭并释放
SDK，且无残留 HSI/verifier 进程。

同轮还修正 `-Unattended` 的控制台依赖：没有 Win32 console 的 SSH/后台任务不再访问
`Console.KeyAvailable` 或 `TreatControlCAsInput`。交互模式没有真实 console 时会明确拒绝并提示
使用 `-Unattended`，而不是抛出 `The handle is invalid`。

因此，双 HSI 自身的 30 分钟机器人候选数据路径现可判定为 PASS-HW。尚不能据此宣称完整
机器人 ready：还需接入 GNSS/RSM/RGB/thermal 验证 FrameContext/共同时间轴，并做全系统
2 小时验收。HSI Internal 模式仍只有 SDK sequence 与 callback monotonic time，不等同于已
验证的硬件曝光时间戳。
