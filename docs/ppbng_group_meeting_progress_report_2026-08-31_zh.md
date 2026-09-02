# PPB-NG 多传感器采集系统阶段进展报告

日期：2026-08-31  
工程：`ppbng_ros2_ws`  
运行平台：Windows industrial PC + ROS 2 Jazzy  
当前重点硬件：FLIR A6701 MWIR、FLIR Blackfly S BFS-U3-123S6C、Arduino UNO R4 WiFi  
文档用途：组会汇报、阶段复盘、后续开发与田间测试参考

---

## 1. 一页结论

本项目的目标是在一台 Windows industrial PC 上建立一个命令行式 ROS 2 多传感器采集系统，统一控制并记录：

- 两台连续线扫描 HSI 相机；
- 一台 RGB snapshot 相机；
- 一台 FLIR A6701 thermal snapshot 相机；
- RSM400 稳定平台及其姿态和状态；
- UM982 双天线 GNSS 数据；
- 外部触发、PPS 和所有可追溯时间证据。

截至 2026-08-31，软件基础架构已经形成，RGB + thermal 子系统已经完成真实硬件台架验证，并达到一个明确的阶段里程碑：

> **RGB 与 thermal 相机已在 Windows/ROS 2 下由同一块 UNO R4 的双路外部触发同步采集，能够保存完整原始数据、逐帧元数据和触发证据，并能离线生成可视化图像。Thermal 已通过约一小时连续采集；RGB + thermal 已通过 60 秒、2 Hz 联合采集，得到 119 + 119 帧和 119 个逻辑同步对。**

但“阶段里程碑”不等于“完整机器人系统已可直接下地”。目前的验证边界是：

| 子系统 | 当前状态 | 结论边界 |
|---|---|---|
| ROS 2 软件框架 | 已完成主要骨架和硬件无关测试 | 13 个 package 可在 Windows 构建；早期完整检查点 427 tests 全通过 |
| Thermal A6701 | 已完成实机连接、采集、外触发、恢复和一小时稳定性测试 | 数据采集链路可靠；在线 NUC 标记和断网/断电恢复仍需补齐 |
| RGB Blackfly S | 已完成实机外触发、RAW、Chunk Data、自动曝光边界验证 | 台架采集可靠；强日照、高位深和运动场景仍需田间测试 |
| RGB + thermal 联合触发 | 已完成实机联合测试 | 已证明逻辑同步；尚未用示波器证明真实曝光起点差 |
| PPS / GNSS | 软件支持 PPS 可选和无 PPS 降级模式 | 本轮相机测试没有 GPS/PPS，因此绝对 UTC 尚未验证 |
| RSM400 | 驱动、协议和状态机已开发 | 尚未连接本机实物验证 |
| 两台 HSI | 驱动、数据结构和验证框架已开发 | 尚未连接本机实物验证和联合连续扫描 |
| 完整一键生产 launch | 架构与组件已存在 | 当前 RGB + thermal launch 是测试入口，不是最终全系统生产入口 |

当前最关键的工程成果并不只是“能拍到图”，而是已经建立了以下原则：

1. 原始数据是权威来源，PNG 只是派生预览；
2. 相机身份必须按序列号/DeviceID 精确绑定；
3. 外部触发必须有 ARM、START、STOP 和 watchdog 的完整状态机；
4. PPS 是可选增强，不是采集的硬前提；
5. 所有时间、曝光、增益、校正和故障状态都必须留证；
6. 单设备故障应隔离，不能无条件拖垮其他传感器；
7. “帧同时到达电脑”与“曝光同时发生”是两件不同的事。

---

## 2. 项目需求与系统边界

### 2.1 运行与操作方式

- 全部采集软件运行在 Windows industrial PC 上；
- 采用一个 ROS 2 workspace，内部按设备、接口、运行时、存储和启动配置拆分多个 package；
- 操作员最终只需在终端执行一个 launch 命令，并提供数据集名称；
- 不开发 GUI；
- 设备由用户手动上电和预热，软件不控制电源；
- 单次任务通常约 2 小时；
- 本机有 2 TB SSD，但每次任务仍必须按实际剩余空间进行启动前检查。

### 2.2 传感器布局与速率

| 设备 | 形式 | 安装位置 | 目标频率/行为 |
|---|---|---|---|
| Specim FX10e HSI | line scan | RSM400 | 连续扫描，约 120 Hz，可配置 |
| Specim SWIR HSI | line scan | RSM400 | 连续扫描，约 120 Hz，可配置 |
| FLIR Blackfly S RGB | snapshot | 机器人固定机架 | 当前测试 2 Hz，可配置 |
| FLIR A6701 MWIR | snapshot | 机器人固定机架 | 当前测试 2 Hz，可配置 |
| UM982 GNSS | 串口消息 | 机器人固定机架 | 约 10 Hz，Windows 只读 |
| RSM400 | 姿态与控制 | 承载两台 HSI | 遥测约 2 Hz；稳定控制连续运行 |

两台 HSI 的扫描线机械上平行并同时连续扫描。RGB 和 thermal 是 snapshot 相机，不安装在 RSM400 上。

### 2.3 与机器人底盘的边界

机器人底盘由独立 Raspberry Pi 控制，并使用另一套独立 GPS。工业 PC 的采集系统不会因为 RTK Fixed 暂时丢失、NTRIP 断线或定位质量下降而停止采集，而是：

- 继续保存所有传感器数据；
- 记录当时的 RTK 状态、定位质量、差分龄期等；
- 在终端给出明显警告；
- 由 Raspberry Pi 自己决定机器人是否停车。

这样可以避免两个控制域耦合，也能保留定位质量下降期间的数据供后处理检查。

---

## 3. 软件总体架构

### 3.1 数据和控制流

```text
                    UM982 TX/GND -> TTL-USB -> GNSS raw + position/heading
                                            |
PPS（可选） -> timing controller -----------+----> time authority
                  |                         |          |
                  +--> HSI trigger ---------+          +--> FrameContext
                  +--> RGB trigger ---------+          |    GNSS/RSM/time quality
                  +--> thermal trigger -----+          |
                  +--> trigger event log ----+----------+

FX10e --------> ppbng_hsi ----------- raw / ENVI / timestamps / index
SWIR ---------> ppbng_hsi ----------- raw / ENVI / timestamps / index
RGB ----------> ppbng_rgb ----------- raw Bayer / Chunk metadata
A6701 --------> ppbng_thermal ------- full radiometric payload / calibration metadata
RSM400 -------> ppbng_rsm400 -------- roll / pitch / status / fault events

                         ppbng_orchestrator + ppbng_runtime
                                      |
                                      v
                         segmented storage + manifest
```

### 3.2 13 个 ROS 2 package 的职责

| Package | 主要职责 |
|---|---|
| `ppbng_interfaces` | 公共 ROS 消息与服务，包括帧元数据、设备状态、故障和时间质量 |
| `ppbng_core` | 时间模型、插值、关联、会话和通用硬件无关逻辑 |
| `ppbng_hsi` | 双 HSI 获取、序号/时间证据和 ENVI 写入 |
| `ppbng_rgb` | Blackfly S/Spinnaker、原始 Bayer 和 Chunk Data |
| `ppbng_thermal` | A6701/Spinnaker、完整 radiometric payload 和独立恢复 |
| `ppbng_gnss` | UM982 只读解析、GGA/UNIHEADINGA 和质量验证 |
| `ppbng_rsm400` | MCP 2.0、姿态、状态、目标角和故障复位 |
| `ppbng_timing` | 触发控制器、PPS 锚定和本地硬件 tick |
| `ppbng_storage` | 分段、CRC、checkpoint、manifest 和恢复验证 |
| `ppbng_orchestrator` | 数据集状态机和跨设备故障策略 |
| `ppbng_runtime` | 生产运行时节点、关联和写盘编排 |
| `ppbng_bringup` | launch 与可版本管理的安全默认配置 |
| `ppbng_sim` | 模拟设备和硬件无关集成测试 |

厂商 SDK 对象被封装在 adapter/backend 后面，ROS 节点尽量保持薄层。这样可以把“SDK 生命周期”“设备协议”“文件格式”和“ROS 通讯”分开测试。

### 3.3 数据集状态机

```text
IDLE
  -> PREFLIGHT
  -> WAITING_FOR_DARK
  -> CAPTURING_DARK
  -> WAITING_FOR_SAMPLE
  -> WAITING_FOR_PPS（仅 PPS-required）
  -> RECORDING
  -> STOPPING
  -> FINALIZED
```

PPS 可选时不会卡在 `WAITING_FOR_PPS`。系统直接从本地硬件时钟开始触发，并把时间质量标为 `UNSYNCED`，而不是伪造 UTC。

---

## 4. 开发与验证方法

整个项目没有采用“一次把所有设备都接上，然后调一个巨大的程序”的做法，而是使用逐级验证：

1. **只读资料和旧代码梳理**：旧 Ubuntu/Orin 工程、HyperFusion 和 PPBv2 只作为参考，不直接覆盖；
2. **纯软件单元测试**：协议、时间、插值、关联、状态机、存储恢复；
3. **虚拟长时间测试**：在不碰真实设备的情况下模拟 2 小时任务；
4. **只读设备枚举**：先确认 DeviceID、型号、接口和可读状态；
5. **单设备短采集**：先不落盘或只保存少量原始帧；
6. **外部触发测试**：先验证电气输入和一脉冲一帧；
7. **故障与恢复测试**：暂停触发、watchdog、重复 Init/DeInit；
8. **连续稳定性测试**：thermal 约一小时；
9. **联合同步测试**：RGB + thermal 同源触发；
10. **离线转换与人工看图**：确认格式、Bayer 排列、热图、对焦和异常帧；
11. **配置收敛与逐帧元数据验证**：检查 AE/AWB 和 Chunk Data；
12. **未来现场验收**：强日照、运动、振动、PPS/GPS、RSM 和 HSI。

这种顺序的核心思想是：每一步只引入一个新的不确定因素，并保留原始证据。异常时能够判断是电气、网络、相机配置、SDK、时间关联还是写盘问题。

---

## 5. 软件基础与硬件无关测试成果

在连接真实相机之前，Windows ROS 2 Jazzy workspace 的 13 个 package 已成功构建。2026-08-27 的硬件无关完整检查点为：

- 427 tests；
- 0 errors；
- 0 failures；
- 0 skipped。

虚拟两小时 soak test 为：

- 模拟时长 7,200 秒；
- 1,468,800 个 trigger events；
- 计数和分段正确；
- 最终关闭干净。

后续 RGB Chunk Data 修改后，另一轮报告的测试集合为 425 tests 全通过。两个数字来自不同开发检查点/测试发现集合，不应简单相加；共同说明是各自检查点均无失败。

纯软件测试中特别覆盖了：

- 巨大、稀疏或溢出的帧序号不能导致按编号分配巨量内存；
- 在线和离线 FrameContext 证据必须逐字段一致；
- HSI 的 `.raw/.hdr/.index.csv/.timestamps.csv` 必须成套存在；
- 恢复、SDK segment 变化和帧 gap 同时发生时只建立一个新 segment；
- PPS 缺失时保留 trigger sequence、absolute controller tick 和 tick frequency；
- 没有可靠 UTC 时，canonical UTC 必须为零且状态为 `UNSYNCED`；
- GNSS 可以用主机 monotonic time 做近似关联，但只能标为 `DEGRADED`。

---

## 6. Thermal A6701：从不稳定旧代码到可重复采集

### 6.1 为什么 thermal 被单独谨慎处理

旧 thermal 程序曾出现“有时能工作、大多数时候不稳定”的情况。因此正式实现采用 clean-room 思路：

- 主要依赖 A6701、GenICam、GigE Vision 和 Spinnaker 官方资料；
- 旧代码只用于了解历史症状和设备身份，不复制其初始化、配置和恢复流程；
- thermal 与 RGB 在生产架构中使用独立 ROS 进程；
- 所有关键节点都执行存在性、可读/可写、写后读回检查；
- 配置被视为事务，任一关键读回不一致就拒绝进入 ARMED；
- 不加载来源不明的 camera user set；
- 不执行 factory reset、firmware update 或 manufacturing 节点。

### 6.2 网络与设备身份

本机 thermal 连接在独立网卡：

- 接口：`Ethernet 3`；
- 网卡：Intel X550 #2；
- 链路：1 Gbit/s；
- 主机原有地址：`192.168.4.150/24`；
- 为相机增加的 link-local 地址：`169.254.10.1/16`。

相机初始地址为 `169.254.10.244/16`。增加主机的 link-local 地址之前，Spinnaker 报 `Camera is on a wrong subnet [-1015]`，说明相机能被发现，但主机和相机不在可通讯的 IPv4 子网。

这里的 `192.168.4.150` 只是这块网卡上的一个主机地址，并不是相机地址，也不是相机必须使用的固定地址。A6701 在执行一次 DeviceReset 后地址变成了 `169.254.17.35`，因此程序必须按以下身份选择设备：

- TL `DeviceID`: `00111C0408CD`；
- 初始化后 `CameraModel`: `A6701`；
- MAC: `00:11:1c:04:08:cd`；
- 接口：GigEVision。

不能把 IP 或“第一个枚举到的相机”作为唯一身份。

### 6.3 只读基线与自由运行测试

实机只读基线：

- `Width=640`；
- `Height=513`；
- `PayloadSize=656640` bytes；
- `PixelFormat=Mono16`；
- `IRFormat=Radiometric`；
- `Ready=1`；
- `FPACold=1`；
- 初始 `FrameSyncSource=Internal`；
- 初始 `TriggerMode=FreeRun`。

自由运行获取 20 帧的结果：

- 20/20 complete；
- Frame ID 连续 1–20；
- 每帧 656,640 bytes；
- 每帧完整 payload hash 不同；
- 相机时间戳间隔约 16.6666 ms，即约 60 Hz。

诊断程序在主机侧观测到的 23.76 FPS 不是相机真实帧率，因为当时同步计算了每帧完整 hash。这个例子说明：主机处理吞吐不能直接当成相机曝光频率。

### 6.4 为什么是 513 行，而图像只有 512 行

A6701 的完整 transport payload 是：

```text
640 × 513 × 2 bytes = 656,640 bytes
```

其中：

- 第 1 行：`640 × 2 = 1,280 bytes`，是 FLIR 的不透明帧辅助数据/元数据行；
- 后 512 行：`640 × 512 × 2 = 655,360 bytes`，是辐射图像计数。

因此必须同时遵守两条规则：

1. **落盘时保留完整 513 行，bit-for-bit 不裁剪**；
2. **显示和常规图像处理时只使用后 512 行**。

如果直接把 513 行全部当图像显示，第一行会污染拉伸范围或产生明显条纹；如果只保存 512 行，又会永久丢失厂商辅助信息。当前代码用 `transport_height=513` 和 `image_height=512` 明确区分两者。

### 6.5 FPA 的 -199 °C 不是错误值

初次读到 `DeviceTemperature≈-199.37 °C` 时容易误以为是无效 sentinel。实际上当 `DeviceTemperatureSelector=FPA` 时，这代表制冷焦平面阵列温度。A6701 的冷却探测器标称工作点约 77 K，即约 -196.15 °C，因此 -199 °C 在物理上合理，并与 `FPACold=1` 一致。

这也是一个重要经验：任何看似异常的数值都要先确认 selector、单位和物理对象，不能仅凭常温直觉判断。

### 6.6 外部触发验证

A6701 使用后面板 `SYNC IN`。已确认的输入要求为：

- rising-edge TTL/LVCMOS；
- nominal 0–5.5 V；
- absolute -0.5–6.5 V；
- `Vih=2.0 V`；
- `Vil=0.8 V`；
- 最小 pulse width 160 ns。

测试使用 UNO R4 D12，2 Hz、1 ms active-high。成功结果：

- 10/10 complete；
- Frame ID 连续 1–10；
- 10/10 full-payload hashes unique；
- 相机时间戳平均间隔 500.302962 ms；
- 最小 500.284530 ms；
- 最大 500.326920 ms；
- 标准差 0.013322 ms。

这些结果证明相机响应了外部脉冲并按 2 Hz 输出完整帧，但 13 µs 的区间标准差不能直接称为“触发物理抖动”，因为 Arduino serial tick 和相机 timestamp 还没有被同一个已校准时钟关联。

### 6.7 软件恢复问题：为什么相机核心节点一度不可读

开发中最严重的软件问题之一是：一次探索性 GenICam “遍历所有节点”之后，A6701 特有节点变得不可读，甚至 `CameraModel` 出现异常字符串。

我们进行了分层排查：

1. 检查 Windows，没有发现仍持有相机的正常 PPBNG/SpinView/Research Studio 进程；
2. 执行标准 DeviceReset，相机重新枚举，但 A6701 节点仍不可读；
3. 使用全新的 `GENICAM_CACHE_V3_0`，问题仍存在，排除旧缓存；
4. 显式执行 `AcquisitionStop` 并 DeInit，问题仍存在，排除简单的 orphaned acquisition；
5. 完整断电重启 A6701 后恢复；
6. 再次执行探索性全节点扫描，问题被复现。

结论：**全节点探索扫描是最可能的软件触发条件**。它已经从生产路径删除，改为只访问明确 allow-list 中的节点。

另外，PowerShell 不应在 native 程序还没完成清理时截断管道输出或中止进程。相机程序必须自然完成，释放 Image、EndAcquisition、AcquisitionStop、DeInit 和 system handle。

恢复后完成了：

- 3 次 Init/DeInit 健康检查；
- 20/20 自由运行帧；
- 额外 5 个 open/acquire/stop/close 循环，共 100/100 帧；
- 再次 10/10 外触发采集；
- 每次结束后 `Ready=1`、`FPACold=1`，配置恢复为 Internal/FreeRun。

### 6.8 故障恢复测试

ROS 2 thermal + UNO 测试节点完成了两类可安全自动执行的故障注入：

1. 命令 UNO 停止触发 4 秒，再恢复；
2. 故意停止发送 KEEPALIVE，使 UNO 在 3 秒 watchdog 超时后停触发，再重新 START。

结果为 22 帧恢复测试通过，A6701 无需重新打开即可恢复采集，最后正常停止并恢复原配置。

尚未自动执行的高风险故障包括：物理拔网线、相机断电、带电拔插同步线。这些测试必须有操作员在场，并在生产节点的分段/重连逻辑完整后单独进行。

### 6.9 一小时稳定性测试

数据集：`hardware_test_data/thermal_ros2_thermal_soak_01_20260827_185418`

结果：

- 采集 7,195 帧；
- 约一小时；
- 没有进程崩溃；
- 没有发现传输链路中断；
- 平均相机时间戳间隔约 500.303445 ms；
- 所有原始帧均可进行离线质量扫描和转换。

2 Hz 下 thermal 原始数据约 4.73 GB/hour（4.40 GiB/hour）。

### 6.10 每约 30 分钟的自动 NUC 到底是什么

一小时数据中发现两段显著异常、近均匀或大量零值的连续帧：

- frame 2544–2607；
- frame 6144–6207。

两段各约 64 帧，起点相差 3,600 帧；在 2 Hz 下正好约 1,800 秒，也就是 30 分钟。

相机节点读回：

- `CorrectionAutoEnabled=1`；
- `CorrectionAutoUseDeltaTemp=1`；
- `CorrectionAutoDeltaTemp=1`；
- `CorrectionAutoUseDeltaTime=1`；
- `CorrectionAutoDeltaTime=30`。

这构成了配置证据和数据证据的闭环：相机正在执行自动 NUC（Non-Uniformity Correction，非均匀性校正）。

#### NUC 的原理

红外焦平面阵列的每个像素在偏置、响应和温漂上都不完全相同。如果不校正，即使观察一个均匀温度目标，也会出现固定图样噪声。NUC 通常让内部温度均匀的 shutter/flag 暂时遮住外部场景，测量每个像素当前的 offset，并更新校正参数。

因此 NUC 是维持热成像质量的正常机制，不是相机故障。但是内部 shutter 关闭及随后稳定期间的帧并不代表外部场景，不能当成葡萄叶或环境温度使用。

#### 当前处理原则

- 原始帧继续保留，不无痕删除；
- 离线转换器根据零像素比例、均匀度和标定范围生成 `quality_flags.ndjson`；
- 后处理中排除 NUC/shutter/settling 影响帧；
- 生产节点下一步需要逐帧记录 `CorrectionAutoInProgress`，或结合状态和图像质量生成可靠在线标记；
- 是否保持自动 30 分钟 NUC，还是在机器人计划停车点主动安排 NUC，仍需做策略试验。

当前建议是先保持相机自动 NUC，不牺牲探测器长期稳定性，同时把受影响时间段明确记录。未来如果要控制 NUC 时机，必须验证禁用自动校正后温漂和图像质量，不应仅为了避免短暂数据空窗而关闭它。

### 6.11 从原始计数到物体真实温度

当前保存的 Mono16 radiometric count 是最可追溯的一级数据。转换器可以依据会话中保存的工厂标定生成：

- 固定尺度的 count PNG；
- count 伪彩色图；
- 工厂标定意义下的 apparent temperature 图。

但 apparent blackbody temperature 不等于所有物体的真实表面温度。完整物温反演还需要：

- 目标发射率 emissivity；
- 反射表观温度；
- 相机到目标距离；
- 大气温度和湿度；
- 大气透过率；
- 外部光学件温度和透过率；
- 匹配当前 17 mm 镜头、Empty filter 位置和温度范围的工厂标定。

葡萄叶、枝条、土壤、金属和塑料的发射率不同，因此不可能用一个全局 emissivity 让整幅图的所有像素都成为准确真温度。合理流程是：

1. 采集时保存完整 raw counts 和配置/环境快照；
2. 对研究对象按类别或掩膜设置发射率；
3. 估计或测量反射温度与大气参数；
4. 后处理计算 radiance 和 corrected object temperature；
5. 最终用可追溯黑体在两个或更多温度点验证整条链。

黑体不是“能不能采集”的必要条件，但如果要对外声称绝对温度精度，它非常重要。HSI 使用的 Spectralon 是反射率标准，不是 thermal 黑体。

---

## 7. Arduino UNO R4 外部触发系统

### 7.1 为什么两台相机不用同一个 GPIO 直接并联

早期尝试曾让 D12 线路同时带有 RGB 分支。使用 UNO 的弱内部 pull-up 做 `LOADTEST` 时，线路被拉低。移除 RGB 分支后：

- D12 悬空：32/32 high；
- D12 + 未接相机的 BNC：32/32 high；
- D12 + A6701 SYNC IN：32/32 high。

这说明被动并联会让一个设备的输入电路影响另一个设备，故障也难以定位。因此采用：

- D11 -> RGB Line0 / OPTOIN；
- D12 -> A6701 SYNC IN；
- Arduino GND -> RGB Opto GND；
- Arduino GND -> thermal BNC shell。

生产级硬件理想上使用独立 buffer/isolator fan-out；在现有条件下，UNO R4 两个独立引脚是更合理且已经实测可用的方案。

### 7.2 两个引脚如何保证尽量同步

UNO R4 的 D11 和 D12 对应 RA4M1 的同一个 Port 4：

- D11 = P411；
- D12 = P410。

固件没有执行两次顺序 `digitalWrite()`，而是使用一次 masked `R_IOPORT_PortWrite` 同时改变两个 bit。因此 MCU 软件层面不会人为加入“先写 D11，再写 D12”的延迟。

这能保证两个电气输出由同一个定时事件、同一次寄存器写产生。但仍不能声称两台相机的曝光严格同一纳秒开始，因为还存在：

- 两个 GPIO 的电气传播差；
- RGB opto input 和 thermal TTL input 的内部路径差；
- 相机内部触发同步、读出和 exposure pipeline 差；
- 线缆长度和阈值差。

要测真实 exposure-start skew，需要示波器/逻辑分析仪同时测 D11、D12，最好再测两台相机的 Exposure Active 输出。

### 7.3 固件参数与状态机

当前固件：`firmware/arduino_uno_r4_rgb_thermal_trigger/arduino_uno_r4_rgb_thermal_trigger.ino`

固定测试参数：

- 频率：2 Hz；
- 周期：500,000 µs；
- 高电平宽度：1,000 µs；
- START 后首脉冲延迟：1 s；
- serial：115200 baud；
- host watchdog：3,000 ms。

支持命令：

- `START`：清零计数、保持输出低、等待 1 秒后开始；
- `KEEPALIVE`：更新主机心跳；
- `STOP`：立即把两路输出拉低；
- `STATUS`：报告运行状态、pulse count 和 watchdog。

每个脉冲结束后串口输出：

```text
PULSE,<sequence>,<rise_us>
```

其中 `rise_us` 是 UNO 本地 `micros()` tick，可作为顺序和相对周期证据，但尚不是 UTC。

### 7.4 为什么需要 watchdog

如果 Windows 节点崩溃、终端被关闭或 USB 串口异常，而 Arduino 继续永久发触发，相机会继续曝光和传输，导致：

- 数据与主程序状态脱节；
- 相机 buffer 累积；
- 重启程序时无法判断 trigger sequence；
- 故障后继续产生无主数据。

所以正常顺序是：

```text
UNO outputs LOW
  -> 配置 RGB 和 thermal
  -> 两台相机都 BeginAcquisition/ARMED
  -> 发送 START
  -> 周期发送 KEEPALIVE
  -> 停止时先发送 STOP
  -> 再 EndAcquisition、恢复配置和 DeInit
```

如果超过 3 秒没有 KEEPALIVE，UNO 自动执行 `STOPPED,host_watchdog_timeout` 并把两路输出拉低。

### 7.5 接错 VIDEO OUT 的事故与结论

一次测试中 BNC 被误接到 A6701 的 `VIDEO OUT`，而不是 `SYNC IN`。Arduino 已发送 11 个脉冲才停止。这是 output-to-output 连接，不是允许的接法。

拔除后做了以下检查：

- Arduino 串口和 D12 low 状态正常；
- A6701 能正常初始化；
- 自由运行 20/20；
- 正确接 SYNC IN 后外触发 10/10；
- GigE 图像路径未观察到损坏。

这只能说明没有观察到 Arduino GPIO 和 A6701 GigE acquisition path 的损坏，不能证明独立 HD-SDI VIDEO OUT 驱动仍完全正常。如果以后要用 VIDEO OUT，必须用合适的 75 Ω HD-SDI monitor/capture device 验证，绝不能再接 GPIO。

最实用的硬件教训是：

- 接线前对照 rear-panel label；
- 触发输出默认保持 LOW；
- 未确认接线前不要 START；
- RGB 和 thermal 不共用同一被动并联 GPIO；
- Arduino Serial Monitor 和 ROS 2 不能同时占用 COM 口；
- 不要把相机输出端当输入端测试。

---

## 8. RGB Blackfly S 开发与改进

### 8.1 实机身份与采集格式

当前相机：

- 型号：Blackfly S BFS-U3-123S6C；
- serial：`22209867`；
- interface：USB3Vision；
- 分辨率：4096 × 3000；
- 当前格式：BayerRG8；
- shutter：global shutter；
- 外部触发：Line0 rising edge，由 UNO D11 提供。

完整 BayerRG8 帧为：

```text
4096 × 3000 × 1 byte = 12,288,000 bytes
```

程序按 serial 选择相机，而不是按 USB 枚举顺序。

### 8.2 RAW、PGM 和 PNG 的区别

| 格式 | 是否保留原始 Bayer 像素 | 是否有尺寸/格式头 | 是否去马赛克 | 主要用途 |
|---|---|---|---|---|
| `.raw` | 是 | 否 | 否 | 权威原始数据、最快直接写入 |
| `.pgm` | 是 | 有很小头部 | 否 | 方便普通软件查看单通道 Bayer |
| RGB `.png` | 否，已派生 | 有 | 是 | 人工彩色预览 |

对于 BayerRG8，RAW 和 PGM 的有效像素信息相同。PGM 只是增加一个很小的头部，文件大小几乎相同，但写入和解析略多一步。彩色 PNG 需要去马赛克和压缩，计算更慢，而且已经不再是原始传感器样本。

因此当前策略是：

- 采集时保存 RAW + 每帧 metadata；
- 需要检查时无损封装 PGM；
- 使用双线性 demosaic 生成快速 RGB PNG；
- 最终科研处理可以重新选择更高质量 demosaic、颜色标定和镜头校正。

实测预览确认 Bayer pattern 按 RGGB 解释合理，没有发现红蓝通道颠倒。

### 8.3 RGB + thermal 联合测试

数据集：`hardware_test_data/rgb_thermal_ros2_rgb_thermal_test_01_20260827_213016`

测试参数：

- 时长 60 秒；
- 2 Hz；
- UNO D11/D12 同时产生逻辑触发；
- thermal 与 RGB 各自保存 raw 和 metadata。

结果：

- thermal：119 帧；
- RGB：119 帧；
- Arduino pulse 对应逻辑 frame pairs：119；
- 配置正常恢复；
- 无残留相机进程。

这里少于理论 120 帧是因为启动、首脉冲延迟和有界运行结束条件的边界，不代表中间漏掉一帧。序号和对应关系是判断漏帧的主要证据，不能只用 `duration × rate` 粗略相减。

### 8.4 自动曝光和自动白平衡最初的问题

最初使用 Continuous Exposure/Gain/WB。早期帧明显偏暗、偏绿：均值从约 2.79 逐步上升到 13.96、36、58，约第 9 帧才超过 100，约 10–12 秒后趋于稳定。

原因不是图像写错，而是相机自动控制环需要多帧观察图像后逐步调整。2 Hz 意味着每秒只有两次反馈机会，因此相同控制算法在低帧率下收敛更慢。

连续自动白平衡还会让相邻帧 RGB channel ratio 持续变化，不利于葡萄园颜色定量比较。因此目前选择：

- `ExposureAuto=Continuous`；
- `GainAuto=Continuous`；
- `BalanceWhiteAuto=Once`；
- `BalanceWhiteAutoProfile=Outdoor`；
- `AutoExposureControlPriority=Gain`；
- `AutoExposureExposureTimeUpperLimit=5000 µs`；
- `AutoExposureGainUpperLimit=12 dB`。

这里的“Gain priority”按相机实际控制语义使用，并通过读回和逐帧结果验证，而不是只相信参数名。

### 8.5 为什么葡萄园采用有边界的自动曝光

田间光照会随云、行间阴影、树冠遮挡、机器人朝向和叶面反光明显变化。完全固定曝光可能在一段任务中出现严重过曝或欠曝；完全无边界自动曝光又可能拉长曝光造成运动模糊，并提高增益造成噪声。

机器人速度为 0.2 m/s：

```text
5 ms 曝光期间平台位移 = 0.2 m/s × 0.005 s = 0.001 m = 1 mm
15 ms 曝光期间平台位移 = 3 mm
```

因此 5 ms 是一个有物理意义的初始上限。真实像素模糊还取决于安装高度、镜头焦距、视场、工作距离和振动，未来需要田间图像验证。

2 Hz 下机器人每两帧前进：

```text
0.2 m/s ÷ 2 frame/s = 0.1 m/frame
```

即相邻 RGB/thermal snapshot 的纵向采样间距约 10 cm。

实机 bounded-auto 短测中，旧状态从约 `14990 µs / 15.3 dB` 逐帧收敛为：

```text
14990, 9593, 7367, 6289, 5725, 5417,
5243, 5140, 5079, 5048, 5027 µs
```

gain 收敛至约 12.1 dB，AWB Once 在第 10 帧左右变为 Off。设置的上限为 5000 µs / 12 dB，实际读回 5027 µs / 12.1 dB 是控制环步进、量化和读回精度造成的小幅边界余量。

用户不希望程序强制丢弃 10–15 秒 pre-roll，因此当前做法是：

- 所有早期帧仍保存；
- 每帧保存真实 exposure/gain/WB；
- 操作员 launch 后让机器人静止等待约 5–10 秒，再开始移动；
- 后处理可以根据逐帧参数识别收敛段。

### 8.6 每帧 Chunk Data

为了让每张 RGB 图像可解释，当前逐帧记录：

- Image API FrameID；
- Image API Timestamp；
- Chunk FrameID；
- Chunk Timestamp；
- Chunk ExposureTime；
- Chunk Gain；
- Chunk BlackLevel；
- post-frame BalanceRatio Red；
- post-frame BalanceRatio Blue；
- ExposureAuto/GainAuto/BalanceWhiteAuto 状态；
- host monotonic receive time；
- Arduino pulse sequence 和 tick。

该型号的白平衡比不是 image chunk，因此是在收到帧后立即从 BalanceRatio 节点读回。元数据中的 `settings_source` 明确写为 `SPINNAKER_CHUNK_PLUS_POST_FRAME_WHITE_BALANCE_READBACK`，避免把两种来源混称为同一时刻的硬件 chunk。

### 8.7 Chunk FrameID 不等于 Image FrameID 的坑

第一版实现错误地假设 `image->GetFrameID()` 必须等于 `chunk.GetFrameID()`，不相等就拒绝帧。实机发现两者存在稳定偏移：

- Chunk FrameID = Image FrameID + 121；
- 两者各自连续；
- Chunk Timestamp 与 Image Timestamp 相等。

这说明两个 API 暴露的计数域/起点可能不同。正确做法是：

- 两个原始值都保存；
- 分别检查单调性和 gap；
- 不要求绝对数值相等；
- 可以记录并监测稳定 offset，但不要把 offset 硬编码为永久常量。

第一次 chunk validation 因这个过强假设安全失败，而且配置成功恢复；修改后第二次验证得到 19 + 19 帧和 19 个逻辑 pair，全部 Chunk/WB metadata 有效。

### 8.8 开启 Chunk 后 payload size 改变

RGB 纯像素数据是 12,288,000 bytes。开启 Chunk 后 transport payload 包含尾部元数据，实测 PayloadSize 增加到约 12,288,088 bytes，而 `Image::GetImageSize()` 仍代表图像像素区域。

如果代码错误地要求 GenICam `PayloadSize == width × height`，会把正常的 Chunk payload 当成格式错误。修正原则是：

- 原始像素文件只写 `GetImageSize()` 指向的 Bayer 数据；
- transport payload 允许大于 image byte count；
- Chunk 通过 SDK API 读取并写 metadata；
- 不把 Chunk 尾部混入 Bayer RAW 文件。

### 8.9 RGB 位深仍需验证

当前使用 BayerRG8。相机还支持 BayerRG10p、BayerRG12p、BayerRG16 等。强太阳光下同时存在高亮天空、叶面镜面反射和深阴影，8 bit 可能更容易饱和或损失暗部层次。

在 4096 × 3000、2 Hz 下：

| 格式 | 每帧 | 数据率 | 两小时原始量 |
|---|---:|---:|---:|
| BayerRG8 | 12.288 MB | 24.576 MB/s | 约 177 GB |
| BayerRG12p | 18.432 MB | 36.864 MB/s | 约 265 GB |
| BayerRG16 | 24.576 MB | 49.152 MB/s | 约 354 GB |

下一步应单独测试 BayerRG12p 的打包、有效位、USB 稳定性、解码和田间动态范围。当前转换器只支持 BayerRG8，不能在未扩展解码器时直接更换生产格式。

---

## 9. 时间同步：已经证明了什么，尚未证明什么

### 9.1 三个不同层级

必须区分：

1. **逻辑同步**：相同 Arduino pulse sequence 对应 RGB 和 thermal 各一帧；
2. **物理曝光同步**：两台相机真实曝光开始时间差是多少；
3. **绝对时间同步**：每个曝光能否映射到 GNSS UTC，误差和不确定度是多少。

本阶段已经证明第 1 层。第 2 层需要示波器/Exposure Active。第 3 层需要接入 UM982/PPS 并验证完整 timing chain。

### 9.2 为什么 host receive time 不能表示曝光差

联合节点顺序调用两台相机的 `GetNextImage()`，帧还经历：

- 相机内部 readout；
- GigE 或 USB3 packetization；
- 网卡/USB 控制器；
- SDK buffer；
- Windows 调度；
- 程序拷贝和写盘。

因此 `host_receive_delta_ns` 中约 18–20 ms 的差主要是传输和软件取帧顺序，不能解释为两台相机曝光差。

### 9.3 PPS 为什么是可选的

没有 PPS 时系统仍保存：

- Arduino pulse sequence；
- Arduino local rise tick；
- 相机 frame ID；
- 相机 timestamp；
- Windows monotonic receive time；
- 配置和时间质量。

这些足以做顺序、周期、丢帧和逻辑 pairing。此时时间状态必须为 `UNSYNCED`，不能把 Windows wall clock 冒充精确 GNSS UTC。

接入 PPS 后，timing controller 用硬件 capture 把本地 tick 轴锚定到每个 UTC 秒；GGA/UNIHEADINGA 提供秒标签和 GNSS 内容。PPS 不是简单“接到相机上就自动拥有 UTC”，而是需要：

```text
PPS edge
  -> hardware capture tick
  -> 与 GNSS 秒标签匹配
  -> trigger event tick 映射到 UTC
  -> camera frame ID 与 trigger sequence 关联
  -> 生成带 uncertainty/time quality 的 FrameContext
```

### 9.4 GPS 10 Hz 与 HSI 120 Hz 如何关联

HSI 频率远高于 GPS，因此不能要求每条 HSI line 都有一个同时到达的独立 GPS 消息。合理做法是：

- 保存所有原始 GNSS 消息；
- 对每个 frame/line 找前后两个合格 GNSS observation；
- 在合适的 Earth/local Cartesian frame 中插值位置；
- heading 使用 circular interpolation；
- 记录 bracket age、fix quality、RTK state 和 uncertainty；
- 超出最大允许年龄时拒绝外推；
- 没有 PPS 时可按 Windows monotonic receive time 近似 bracket，但必须标 `DEGRADED`。

RGB/thermal 虽然只有 2 Hz，也采用同一个 FrameContext 机制，这样每帧能够携带当时可追溯的 GNSS/RSM 状态，而不是把“最近一条 GPS”无条件复制过去。

---

## 10. 数据保存、完整性与转换工具

### 10.1 原始优先

- RGB：raw Bayer；
- thermal：完整 640 × 513 Mono16 payload；
- HSI：ENVI-compatible raw/hdr + index/timestamps；
- GNSS：raw message + parsed sample；
- RSM：raw protocol + attitude/status；
- timing：raw trigger event + summary；
- 全局：session manifest、events、faults 和配置快照。

PNG、伪彩色、温度图和 debayer 图都是可重建的派生物，不能替换 raw。

### 10.2 分段和异常关机恢复

生产 writer 设计包括：

- 可配置时间/大小 segment；
- CRC；
- 周期 checkpoint；
- 临时文件完成后原子 rename；
- 截断文件恢复；
- reconnect 后创建新 segment；
- manifest 记录 requested/actual settings、serial、SDK/firmware、warnings。

台架测试为了便于逐帧检查，仍有 one-file-per-frame 数据。最终两小时生产采集应以 segmented writer 为主，否则大量小文件会增加目录和文件系统压力。

### 10.3 两小时理论存储预算

默认全系统示例：

| Stream | 数据率 |
|---|---:|
| FX10e 1024 × 448 × 16 bit × 120 Hz | 110,100,480 B/s |
| SWIR 384 × 288 × 16 bit × 120 Hz | 26,542,080 B/s |
| RGB BayerRG8 × 2 Hz | 24,576,000 B/s |
| A6701 full payload × 2 Hz | 1,313,280 B/s |
| 合计 | 162,531,840 B/s，约 155 MiB/s |

两小时原始 payload 约 1.064 TiB。加 25% headroom 和固定 100 GiB finalization/OS reserve 后，任务开始时建议至少有约 1.428 TiB 实际空闲空间。

2 TB 是标称容量，不能只看“磁盘总容量”。程序必须读取目标卷的实时空闲空间并按配置计算；还应完成不低于约 203,164,800 B/s 的 durable-write qualification。

### 10.4 转换工具

- `tools/convert_thermal_raw_to_png.py/.cmd`
  - 跳过 thermal 第 1 transport row；
  - 输出 count 图、伪彩色和 apparent-temperature 图；
  - streaming 处理，适用于多小时数据；
  - 生成 `quality_flags.ndjson`。
- `tools/convert_rgb_bayer_raw.py/.cmd`
  - RAW 无损封装 PGM；
  - BayerRG8 去马赛克为 RGB PNG；
  - 不修改权威 raw。

---

## 11. 主要踩坑、原因和修正

| 现象/事故 | 根因 | 修正 | 可迁移经验 |
|---|---|---|---|
| A6701 被发现但初始化报 wrong subnet | 主机和 link-local 相机不在同一 IPv4 子网 | 给专用网卡增加 `169.254.10.1/16` | “能枚举”不等于“能建立 GigE 控制/流连接” |
| 把 `192.168.4.150` 误认为相机地址 | 它是主机网卡地址 | 用 DeviceID 识别相机，IP 仅用于网络路由 | 不硬编码 DHCP/link-local IP |
| DeviceReset 后相机 IP 改变 | link-local 地址可重新分配 | 精确绑定 DeviceID + model readback | 身份和地址必须分离 |
| A6701 节点突然不可读 | 探索性全 GenICam 节点扫描触发异常状态 | 删除全节点扫描，改 allow-list | 诊断程序也可能改变设备行为，尤其 vendor node |
| 怀疑残留进程占用相机 | 一部分失败路径清理不完整/输出被截断 | 完整 EndAcquisition、AcquisitionStop、Release、DeInit；让 native 程序自然退出 | 资源生命周期与正常采图同等重要 |
| A6701 高度 513 看似多一行 | 第一行是 1,280-byte opaque auxiliary row | raw 保留 513，preview 只显示后 512 | transport geometry 不等于 image geometry |
| FPA -199 °C 看似无效 | 读的是制冷探测器，不是机身/环境温度 | 检查 selector 和物理工作点 | 任何遥测值都必须带 selector、单位和语义 |
| 每 30 分钟出现连续异常 thermal 帧 | 自动 NUC shutter/settling | 保留 raw，标记并在后处理排除 | 校正过程是测量时间线的一部分 |
| D12 同时接两个相机时弱 pull-up 被拉低 | 被动并联输入互相加载 | D11/D12 分开，单次 port write | 同源触发不等于必须共用同一物理线 |
| BNC 误接 VIDEO OUT | 接口识别错误，形成 output-to-output | 立即拔除，完成两端功能回归 | 带电接线前必须核对端口方向 |
| Arduino 程序退出后可能继续触发 | MCU 独立于 Windows 运行 | 3 s host watchdog，正常 STOP-first shutdown | 外触发源必须有 fail-safe low |
| ROS launch 把 `rgb_device_id` 当整数 | 纯数字序列号在 launch 参数中发生类型推断 | 强制按 string 传递/声明 | 硬件 ID 永远按不透明字符串处理 |
| RGB 前几帧暗/绿 | 低帧率下 AE/AWB 尚未收敛 | bounded AE + AWB Once + 每帧元数据 + 操作员静止等待 | Auto 不是瞬时完成，也不是无需记录 |
| 第一版 Chunk validation 拒绝正常帧 | 错误假设 Image FrameID == Chunk FrameID | 两套计数分别保存和检查 | 相关计数器不一定有同一个零点 |
| 开启 Chunk 后 PayloadSize 比像素数大 88 bytes | transport 尾部含 chunk | 像素按 GetImageSize 写，chunk 走 API | payload、image 和 file byte count 要分开 |
| 联合节点 host receive 相差约 20 ms | USB/GigE/SDK/顺序 GetNextImage | 不用 arrival delta 推断曝光差 | 测曝光同步必须看硬件 edge/exposure signal |
| PGM 和 RAW 被误认为信息量不同 | PGM 只是对相同像素加头 | RAW 为权威，PGM 为方便查看 | 区分封装变化和像素处理 |
| thermal 伪彩色被误当成真温度 | display AGC/标定/物理修正概念混合 | 区分 count、apparent temp 和 true object temp | 可视化好看不代表物理可追溯 |

---

## 12. 田间操作需要特别注意的细节

### 12.1 采集前

- 确认所有设备上电并充分预热，A6701 必须 `Ready=1`、`FPACold=1`；
- 检查 thermal 镜头、filter、temperature range 和 calibration pack 匹配；
- 确认 A6701 BNC 接 `SYNC IN`，不是 `VIDEO OUT`；
- 确认 D11 只接 RGB，D12 只接 thermal，共地可靠；
- 关闭 SpinView、Research Studio、Arduino Serial Monitor 和其他占用程序；
- 按 serial/DeviceID 检查两台相机身份；
- 确认 SSD 输出目录、实时空闲空间和 durable write 资格；
- 启动后保持机器人静止约 5–10 秒，让 RGB AE/AWB 收敛；
- 查看 RGB exposure/gain/WB、饱和/欠曝情况；
- 查看 thermal focus、画面、NUC 状态和 calibration metadata；
- 如进行 HSI，按提示遮盖镜头、关闭 shutter、采 dark reference；
- 确认两条 HSI scan line 内 Spectralon reference bar 始终可见。

### 12.2 采集中

- 不要打开 Arduino Serial Monitor；
- 不要在同一 segment 内无记录地修改 rate、exposure、gain、pixel format 或 calibration range；
- 终端持续监控每台设备 frame rate、last frame ID、gap、timeout 和 recovery；
- 监控 RGB exposure/gain/WB 和未来的 saturation/dark fraction；
- 监控 A6701 NUC、Ready、FPACold 和异常帧数；
- 监控磁盘写入速率和剩余空间；
- RTK 下降时继续采集并记录，不要让工业 PC 擅自控制底盘；
- 一台非关键设备故障时，其他设备继续，故障设备进入独立恢复和新 segment；
- 若 timing controller 或磁盘发生全局故障，执行受控全局停止。

### 12.3 采集结束

- 操作员按一次 Ctrl+C 或使用正式 stop service；
- 等待软件先向 UNO 发送 STOP；
- 等待相机 EndAcquisition、配置恢复和 DeInit；
- 等待 writer checkpoint、flush、manifest 和 final rename；
- 不要直接关终端或断电；
- 检查 `capture_complete.json/session.json` 和终端 final summary；
- 抽检起始、收敛后、中间、NUC 附近和结束帧；
- 在离开现场前确认 raw 可读，而不是只确认目录存在。

---

## 13. 当前未完成项与下一阶段计划

### P0：进入完整生产候选版前必须完成

1. **Thermal 在线 NUC 标记**  
   将 `CorrectionAutoInProgress` 和图像质量 evidence 写入每帧元数据，验证 NUC 前后边界。

2. **RGB 每帧图像质量摘要**  
   记录 saturation fraction、dark fraction、p1/p50/p99，持续异常时终端报警。

3. **独立生产节点联合运行**  
   当前 RGB + thermal 联合节点是测试工具。正式系统需要两个独立进程、独立 recovery、共享 trigger/session 核心。

4. **完整 dataset 闭环**  
   统一 dataset name、preflight、磁盘检查、配置快照、分段、CRC、manifest、验证和安全停止。

5. **真实故障测试**  
   在操作员在场时验证拔网线、USB 断连、相机短暂掉电、Arduino 断开；确认其他设备继续、故障设备新 segment 恢复。

6. **两小时 RGB + thermal 联合运行**  
   检查 frame/pulse counts、内存、句柄、磁盘、NUC 和关闭行为。

### P1：强烈建议

1. BayerRG12p 与 BayerRG8 的日照动态范围对比；
2. 实际葡萄园强光、阴影、反光和 0.2 m/s 运动测试；
3. 示波器测 D11/D12 edge skew 和 Exposure Active skew；
4. thermal 黑体多温点验证；
5. RGB color chart/gray card 色彩和曝光验证；
6. 完成 PPS + UM982 实机 UTC 关联和 uncertainty budget；
7. 连接 RSM400，确认 RS-232/RS-422 转换链、MCP 版本、STAB、目标姿态、roll/pitch 遥测和 fault reset；
8. 分别连接两台 HSI，完成 dark reference、continuous line scan、120 Hz trigger、丢线和写盘测试；
9. 最后做所有设备 2 小时全速、约 155 MiB/s raw payload 的系统级验收。

---

## 14. 建议组会汇报顺序

如果把本报告整理成口头汇报或 slides，建议按下面顺序：

1. **问题与目标**：为什么需要 Windows 上统一 ROS 2 多传感器采集；
2. **整体架构**：设备、频率、触发、时间和存储；
3. **验证方法**：为什么坚持单设备、只读、短测、恢复、soak、联合；
4. **Thermal 重点**：网络、513 行、外触发、一小时稳定性、30 分钟 NUC；
5. **Arduino 重点**：D11/D12、atomic port write、watchdog、接错端口和并联输入的教训；
6. **RGB 重点**：raw Bayer、PGM/PNG、AE/AWB、Chunk Data、FrameID offset；
7. **同步边界**：已经实现逻辑同步，还没有声称物理/UTC 同步；
8. **量化结果**：7,195 thermal frames、119 + 119 联合帧、虚拟 2 小时测试；
9. **下一里程碑**：NUC 在线标记、田间图像、PPS/GNSS、RSM、HSI、全系统 2 小时。

组会上应特别强调三句话：

> 1. 当前 RGB + thermal 已经是“可重复的 ROS 2 台架采集链路”，但还不是“完成田间验收的整机系统”。  
> 2. 相同 Arduino 序号证明逻辑同步，不代表已经测得两台相机的真实曝光时间差或 UTC 误差。  
> 3. A6701 每约 30 分钟的异常段是自动 NUC 的正常副作用，必须标记和排除，而不是简单当成随机坏帧或直接关闭校正。

---

## 15. 关键文件与证据索引

### 设计与总体文档

- `docs/architecture.md`
- `docs/time_authority.md`
- `docs/timing_hardware.md`
- `docs/storage_budget.md`
- `docs/storage_qualification.md`
- `docs/hardware_validation_plan.md`
- `docs/device_by_device_validation_zh.md`
- `docs/CONTINUATION_CHECKPOINT_2026-08-27.md`

### RGB + thermal 阶段文档

- `docs/rgb_thermal_milestone_2026-08-27_zh.md`
- `docs/rgb_thermal_uno_r4_test.md`
- `docs/thermal_ros2_uno_r4_test.md`
- `docs/thermal_a6701_contract.md`
- `docs/validation/thermal_a6701_stage1_2026-08-27.md`
- `docs/validation/thermal_a6701_external_trigger_2026-08-27.md`
- `docs/references/rgb_thermal_official_reference_index_2026-08-27_zh.md`

### 主要代码

- `src/ppbng_thermal/launch/rgb_thermal_uno_r4_test.launch.py`
- `src/ppbng_thermal/src/a6701_uno_r4_test_node.cpp`
- `src/ppbng_thermal/src/thermal_camera_node.cpp`
- `src/ppbng_rgb/src/rgb_camera_node.cpp`
- `src/ppbng_rgb/src/spinnaker_rgb_backend.cpp`
- `src/ppbng_interfaces/msg/FrameMetadata.msg`
- `firmware/arduino_uno_r4_rgb_thermal_trigger/arduino_uno_r4_rgb_thermal_trigger.ino`
- `tools/convert_thermal_raw_to_png.py`
- `tools/convert_rgb_bayer_raw.py`

### 关键实测数据集

- Thermal 一小时：`hardware_test_data/thermal_ros2_thermal_soak_01_20260827_185418`
- Thermal 对焦：`hardware_test_data/thermal_focus_focus_trial_03_20260827_181628`
- RGB + thermal 60 秒：`hardware_test_data/rgb_thermal_ros2_rgb_thermal_test_01_20260827_213016`
- Chunk metadata 成功验证：`hardware_test_data/rgb_thermal_ros2_chunk_metadata_validation_02_20260827_220555`
- Bounded auto 验证：`hardware_test_data/rgb_thermal_ros2_bounded_auto_validation_01_20260827_221329`

---

## 16. 最终阶段判断

目前可以把项目分成两个层级看待：

### 已经达到的里程碑

- 建立了可构建、可测试的多 package ROS 2 软件基础；
- 形成了保守、可追溯的时间和数据模型；
- A6701 从不稳定历史状态发展到可重复初始化、外触发、恢复和一小时采集；
- RGB 完成 raw Bayer、外触发、Chunk metadata 和有边界自动曝光验证；
- UNO R4 实现双路同事件触发、状态控制和 host watchdog；
- RGB + thermal 完成 119 对逻辑同步实机采集；
- 提供 raw 到可视化/表观温度的离线工具；
- 把若干高风险误区转化为代码约束和操作规范。

### 尚未达到的最终目标

- 没有完成 RGB + thermal 的田间光照与运动验收；
- 没有用仪器测量真实 exposure skew；
- 没有完成本机 UM982/PPS 的 UTC 硬件验证；
- 没有完成 RSM400 和双 HSI 的实机联调；
- 没有完成所有设备同时运行的两小时全系统验收；
- 没有完成绝对物体温度的黑体验证；
- 当前联合相机 launch 仍是测试工具，完整生产入口还需收口。

因此，最准确的表述是：

> **PPB-NG 已完成软件骨架和 RGB/thermal 子系统的关键台架里程碑，已经从“概念和旧代码参考”进入“有真实数据证据、可重复测试、知道剩余风险”的工程阶段。下一阶段应从增加功能转向逐项硬件验收、田间质量验证和全系统收口。**
