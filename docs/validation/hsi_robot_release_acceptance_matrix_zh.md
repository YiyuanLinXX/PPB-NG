# PPB-NG HSI 机器人交付验收矩阵

更新日期：2026-09-01

## 1. 结论与放行规则

目前软件契约和 FX10e 无界面初始化已经具备证据，但两台 HSI **尚不能标记为机器人交付
Ready**。正式放行至少需要完成本表中的 H1–H10，并解决 T1 的时间精度边界。任何一项
Required 未通过，都只能标记为开发验证版。

状态定义：

- `PASS-SW`：纯软件自动测试已通过；不代表真实硬件行为。
- `PASS-HW`：已有真实硬件证据。
- `PARTIAL`：只有部分相机、模式或阶段有硬件证据。
- `BLOCKED`：必须等待相机、GNSS、RSM、RGB/thermal 或人工操作。
- `REQUIRED`：机器人放行前必须通过。

## 2. 已有证据

| ID | 验收项 | 当前状态 | 证据与边界 |
| --- | --- | --- | --- |
| S1 | 配置 fail-closed、明确硬件使能、prepare/arm/start 门控 | PASS-SW | `HsiProductionGate`、session binding 自动测试；441 tests 全通过。 |
| S2 | 两台 HSI 使用独立几何、行频、曝光、队列与文件 stem | PASS-SW | `DualHsiCoordinator` 和统一配置测试。 |
| S3 | Internal 连续采集不依赖 Arduino 外部逐行触发 | PASS-SW | `HsiInternalTiming.ContinuousFramesDoNotRequireExternalTriggerEvents`。 |
| S4 | 原始格式为 little-endian uint16 ENVI BIL | PASS-SW | `.raw + .hdr + index.csv + timestamps.csv` writer/verifier 测试。 |
| S5 | 每行 byte offset、长度、CRC32、SDK frame number、host monotonic time可审计 | PASS-SW | writer、part verifier 与 corruption/truncation 测试。 |
| S6 | dark/sample 明确分离，暗场不发布 scene SampleStamp | PASS-SW | dark workflow、writer 和 SampleStamp mapper 测试。 |
| S7 | 分卷、恢复边界、跨 part 连续性及禁止覆盖已有目标 | PASS-SW | ENVI dataset verifier 系列测试。 |
| S8 | 队列容量固定、溢出/非法回调可检测 | PASS-SW | `SpecSensorFrameQueue` 测试。 |
| S9 | transport/timeout 有界退避；storage/integrity/timing 不盲目重试 | PASS-SW | bounded recovery 与 fault policy 测试。 |
| S10 | 无 PPS 时不伪造锁定时间，保持 UNSYNCED/UNVERIFIED | PASS-SW | timestamp writer、SampleStamp mapper 测试。 |
| D1 | FX10e profile/序列号/几何/包长/标定包/快门读回 | PASS-HW | 2026-09-01 D2：`1024 x 448 x 2`、50 Hz、18000 us、8228 bytes。 |
| D2 | FX10e SSH/headless 固定 IP 初始化 | PASS-HW | `Grabber.Channel=192.168.10.2`，约 9 秒，无 eBUS 选择窗口。 |
| D3 | SWIR profile/序列号/真实几何/曝光行频读回 | PASS-HW | `384 x 288 x 2`，约 20.59 Hz / 17800 us；Internal dark/sample 原始数据已通过 CRC/序列验证。 |
| D4 | 正常 stop 后关闭快门、释放 SDK/handle | PASS-HW | 单机与双机采集中 stop 均成功；两个独立 worker 随后 Ctrl+C 干净退出。 |

2026-09-01 重新执行：

```powershell
.\tools\test.cmd --packages-select ppbng_hsi ppbng_bringup
```

结果为 `441 tests, 0 errors, 0 failures, 0 skipped`。

## 3. 机器人放行必须完成的硬件矩阵

| ID | Required 验收 | 建议工况 | 通过标准 | 当前 |
| --- | --- | --- | --- | --- |
| H1 | FX10e 独立 dark + sample 短采集 | 5 s dark，至少 2 min sample | 暗场行数正确；原始字节数、ENVI 几何和 BIL 方向正确；逐行 CRC、frame number 连续；正常 stop 快门关闭 | PASS-HW（D3 retry） |
| H2 | SWIR 独立 Internal 短采集 | 同 H1 | 同 H1，且读回几何/rate/exposure/trigger 均符合容差 | PASS-HW（D3、D9） |
| H3 | 暗场有效性 | 镜头盖好，重复 3 次任务 | 每次 line count 正确；均值/噪声稳定；确认快门动作后的 settling/discard 行数；暗场与 sample 不混淆 | BLOCKED |
| H4 | 双 HSI 同时采集 | 至少 30 min | 两流均无 frame gap、queue overflow、invalid callback、写盘错误；SDK 共存稳定 | PASS-HW（D11：31.4 min 完整 CRC；`hsi_dual_30min_after_failfast_02_20260901_224811`：加固后 30 min、持久诊断全零、快速校验通过） |
| H5 | 目标长稳 | 全系统目标参数 2 h | 无不可解释丢行；温度、SSD 吞吐、队列水位稳定；最终文件均通过 verifier；快门关闭 | BLOCKED |
| H6 | FX10e 断网与恢复 | 双机采集中拔/恢复 FX 网线一次 | 明显 fault；FX 有界恢复并开新 segment；SWIR 按策略继续；无静默数据拼接 | BLOCKED |
| H7 | SWIR/NI 故障恢复 | 双机采集中模拟允许的 NI/PCU 链路故障 | 同 H6；确认 `SI_Unload`/多进程行为不会破坏另一台相机 | BLOCKED |
| H8 | 正常与异常退出 | stop、Ctrl+C、受控强制终止各一次 | stop/Ctrl+C 数据可验证且快门关闭；强制终止不假定能执行清理，行为必须被记录，并形成明确的人工恢复 SOP | PARTIAL：正常退出与单 SWIR worker 重启已通过 |
| H9 | 曝光/行频边界 | 室外光照下选择正式参数 | 厂商软件可用；ROS 应用值读回在容差内；连续采集无超时/丢行 | BLOCKED |
| H10 | 全设备并行 | HSI + RGB + thermal + GNSS + RSM | 2 h 吞吐通过；每个 scene sample 有唯一 context；任一设备故障符合隔离/全局停止策略 | BLOCKED |

H3 特别重要：当前程序会保存所请求的全部 dark lines，但尚无真实证据说明机械快门动作后前几行
是否需要丢弃。这个值不能仅凭 SDK 文档推定，必须看真实暗场统计。

## 4. 时间同步验收

### T1：当前能力边界（机器人放行阻塞项）

两台 HSI 采用各自内部 line clock，不共享外部逐行触发。当前 SpecSensor 接口保留：

- SDK frame number；
- SDK 回调进入主机时的 monotonic timestamp；
- GNSS/RSM 上下文及其时间质量状态。

但当前没有得到“每一行实际曝光时刻”的相机硬件时间戳。因此 callback arrival 只能作为主机
到达时间证据，不能未经测量就宣称是微秒级或毫秒级 GPS 曝光时间。两台 HSI 也不会逐行同时
曝光；系统所能做的是把两条独立时间序列映射到共同时间轴，再对 10 Hz GNSS 和 2 Hz RSM
插值/最近邻关联。

机器人放行前必须：

1. 在单机和双机负载下测量相邻 callback interval 的分布、长时漂移、frame gap 与调度抖动；
2. 接入 UM982 后验证主机 monotonic 到 GNSS UTC 的映射、断 RTK/NTRIP 时的质量降级记录；
3. 定义并实测可接受的 HSI 行时间不确定度；
4. 若 callback 抖动超过科学目标容差，应重新评估硬件触发、相机硬件 timestamp/PTP 或额外
   timing capture。不能通过软件把缺失的曝光时刻“推算成已验证事实”。

### T2：同步数据通过标准

- 每个 sample line 都有唯一 SDK frame ID、host monotonic time、segment/part/offset/CRC；
- frame gap、回退、回调溢出不能静默；
- 每个 sample line 恰好对应一个 FrameContext；dark lines 不要求 scene context；
- GNSS/RSM 缺失或质量差时继续采集，但记录明确的 UNSYNCED/UNVERIFIED/quality 状态；
- 后处理不得把非确认状态当作 RTK Fixed 同步数据。

## 5. 数据量与存储门槛

按当前已读回参数估算，不含文件系统和 sidecar 开销：

| 数据流 | 速率 |
| --- | ---: |
| FX10e：917,504 B/line × 50 Hz | 45,875,200 B/s |
| SWIR：221,184 B/line × 20.590113 Hz | 约 4,554,204 B/s |
| HSI 合计 | 约 50,429,404 B/s；2 h 约 363.1 GB |
| 再加当前 RGB/thermal 2 Hz 原始 payload | 全系统约 76,318,684 B/s；2 h 约 549.5 GB |

SSD 持续落盘资格测试已于 2026-09-01 完成：write-through + `FlushFileBuffers` 写入 8 GiB，
实测 1,016,145,214 B/s（969.07 MiB/s），约为当前全系统 1.25 倍门槛的 10.6 倍。证据为
`data/ppbng_durable_evidence_311c41ad-c3e3-4a15-9437-3f53e220ad33.json`，统一配置已引用并由
production launch 校验卷序列号、路径和参数边界。

## 6. 可以无人值守自动执行的项目

以下项目不接触硬件，可在每次代码修改后执行：

1. 构建与 `ppbng_hsi ppbng_bringup` 全部测试；
2. 统一 YAML schema、禁止 FX `grabber_channel: ui`、Internal 模式和几何约束；
3. mock 双流、dark/sample 状态机、恢复/故障注入；
4. ENVI writer/verifier、CRC corruption、truncation、part gap、禁止覆盖；
5. SampleStamp/FrameContext 一一对应与时间质量降级；
6. 对真实采集结果运行只读 verifier 和统计报告（前提是已有数据目录）。

## 7. 必须等待用户或其他设备的项目

- H1–H9：需要相机保持上电、镜头盖/场景切换、允许的插拔故障以及必要的现场观察；
- H10/T2：需要 RGB、thermal、UM982、RSM400 和 timing controller 接入；
- 室外曝光参数：需要葡萄园等效光照、工作距离、速度约 0.2 m/s 与真实镜头状态；
- 精确同步容差：需要研究目标给出允许的空间/时间误差，不能由软件自行决定；
- 断电安全：软件无法在整机掉电后主动关快门，必须结合相机默认状态、供电顺序和 SOP 验证。

## 8. 建议执行顺序

`H1 FX短采集 → H2 SWIR短采集 → H3暗场统计 → H4双机30 min → H6/H7故障恢复
→ T1/GNSS时间验证 → H10全设备短测 → H5全系统2 h → 数据集离线完整性报告`。

只有最后一轮全部 Required 项通过、产出可追溯报告后，才把配置中的
`safety.configured` 和对应 evidence flags 改为 true。
