# PPBNG 单设备逐级测试手册

本项目禁止把“第一次连接设备”和“整机联合采集”合并进行。每个设备必须独立完成
软件、身份、只读通信、短采集、故障恢复和数据审计，才允许进入组合测试。任何硬件
阶段仍受 `safety.md` 约束；拥有管理员权限不等于获得硬件操作许可。

## 1. 通用测试阶梯

每个设备都按以下顺序推进，失败后停留在当前级别：

| 级别 | 内容 | 是否接触硬件 | 通过证据 |
| --- | --- | --- | --- |
| D0 | 指定 package 的单元测试和 mock 故障注入 | 否 | 测试结果 XML 与终端摘要 |
| D1 | 只读身份清点 | 是，仅枚举 | 型号、序列号、接口、SDK/驱动版本记录 |
| D2 | 单设备打开或 receive-only 读取 | 是 | 只包含该设备的有界日志，不发送未授权命令 |
| D3 | 单设备低速、短时功能试验 | 是 | 原始数据、状态、配置 readback、停止结果 |
| D4 | 单设备断线/超时/恢复试验 | 是 | 新 segment、故障事件、无静默换机或覆盖 |
| D5 | 单设备目标速率和持续时间验收 | 是 | 完整性验证、吞吐、温度、丢帧统计 |
| C1 | 有直接依赖的双设备测试 | 是 | 共享触发或同步证据 |
| C2 | 全系统低速测试，最后才进行两小时全速测试 | 是 | 最终 manifest 与数据集验证报告 |

D0 可随时执行。D1 及以后每一级都需要针对该设备和动作的单独明确授权，授权不会
自动扩展到其他设备或下一等级。

## 2. 当前可直接运行的纯软件测试

先在 workspace 根目录执行一次构建，然后只测试需要的模块：

```bat
tools\build.cmd
tools\test_device.cmd rgb
tools\test_device.cmd thermal
tools\test_device.cmd fx10e
tools\test_device.cmd swir
tools\test_device.cmd gnss
tools\test_device.cmd rsm400
tools\test_device.cmd timing
```

`fx10e` 和 `swir` 当前共享 `ppbng_hsi` package，因此两条命令运行同一套 HSI 公共
契约测试；硬件阶段仍必须分别执行。脚本只接受一个固定模块名，不透传任意命令，
并且不会枚举、打开或配置硬件。

## 3. 每个设备的独立验收内容

### UM982 GNSS（Windows 只接收）

1. D0：NMEA/UNIHEADINGA 分帧、校验、UTC 日期解析、午夜跨越、RTK 状态和恢复测试。
2. D1：只记录 TTL-to-USB 的 COM、VID/PID、USB serial 和驱动；不打开端口。
3. D2：只以 receive-only 句柄读取，确认 Windows 端绝不发送字节；先运行 30 秒。
4. D3：记录 GGA、UNIHEADINGA、定位质量下降和恢复；RTK 下降不得停止采集。
5. D5：至少覆盖 PPS/串口输出延迟及临时断线；保留原始状态和不确定度。

### RSM400

1. D0：MCP2 解析、CRC/边界、状态就绪门、存储和命令白名单测试。
2. D1：确认 USB 转换器身份以及 RS-232/RS-422 类型，不打开串口。
3. D2：仅监听遥测；不发送 `RC 1`、`ST`、`OF002`、`OF005` 或 reset。
4. D3：必须建立物理隔离区并单独授权运动后，才验证自动水平和停止。
5. D4：再分别验证目标 roll/pitch、故障复位、通信中断和 watchdog。

### RGB（FLIR BFS-U3-122S6C-C）

1. D0：原始 Bayer、精确 serial、Spinnaker 生命周期、buffer 释放、队列和恢复测试。
2. D1：只读列出 serial/model/interface/MAC/IP；不 `Init()`、不改 GenICam node。
3. D2：单独打开相机并读回配置，触发线保持物理禁用；不采集。
4. D3：先用低速外部触发采 10–30 帧，验证 BayerRG8、4096×3000、payload、帧号、
   相机时间戳和主机时间戳。
5. D4/D5：验证漏触发、拔线恢复、新 segment、2 Hz 长时稳定性及数据 CRC。

### Thermal（FLIR A6701）

1. D0：优先使用重新实现的 A6701 契约测试，不以旧代码行为作为真值。
2. D1：只读确认精确 DeviceID/model/interface/MAC/IP 和实际 GenICam node 名。
3. D2：触发输入硬禁用，仅打开并读取 Ready、FPACold、640×513 Mono16 和 sync 设置。
4. D3：低速采集时完整保存 656,640 字节；前 1,280 字节保持不透明 metadata 行，
   后 655,360 字节是 640×512 辐射图像。不得裁掉第 513 行。
5. D4：用示波器证明 SYNC IN 的 FSSI/FSSR 语义后，才允许把同步证据标记为 confirmed。
6. D5：验证超时、重连、温度/Ready 状态、2 Hz 稳定性和完整 payload CRC。

### FX10e 与 SWIR HSI

两台相机必须分别完成 D1–D5，然后才做双 HSI 同时扫描：

1. D0：SDK 回调、bounded queue、dark/sample 区分、ENVI 分段和逐行 CRC。
2. D1：分别确认 SDK index、传感器 serial、license 和相机类型，禁止按“第一个相机”选取。
3. D2：单台打开、读回几何/曝光/line rate；确认关闭 session 后 shutter 关闭。
4. D3：用户盖镜头并确认后采 dark reference，再用低速外触发采短样本。
5. D4：分别验证丢 trigger、callback gap、断线、重连和新 segment。
6. D5：各自 120 Hz 验收；随后 C1 才允许两台相机同时连续扫描，并确认扫描线并行
   的机械前提不会被软件错误地当成时间同步证据。

### PPS/trigger controller

1. D0：协议 CRC、冻结 schedule、通道序列、PPS holdover、reset 和环形缓冲溢出。
2. D1：只读记录 COM、VID/PID/serial；输出端保持硬件 gate-off。
3. D2：只接 USB、不开相机线，观察状态和 PPS capture。
4. D3：示波器逐通道测试；每次只启用 FX10e、SWIR、RGB 或 thermal 一个物理输出。
5. C1：最后才测试 RGB/thermal 同一逻辑 snapshot 时刻、不同受保护驱动的双输出。

## 4. 每次硬件测试前必须给出的信息

执行者必须先向用户说明并等待确认：设备和接口、是否会发送命令、预期物理动作、
会创建的文件、停止/回滚方式、最长时限。建议第一次 D2 不超过 30 秒，第一次 D3
不超过 60 秒；出现身份不唯一、配置 readback 不一致、意外运动、异常温度或无法停止
时立即终止。

每次硬件测试使用唯一的 `validation_<device>_<UTC>` 数据集名称。不得写入生产数据集，
不得覆盖既有文件。每一级结束后运行只读数据验证，并把命令、配置快照、终端摘要和
通过/失败结论保存到该验证数据集。

## 5. 单设备启动入口的实现约束

正式硬件验证入口必须满足以下条件后才会加入：

- `device` 参数只接受固定枚举，恰好创建一个硬件驱动；
- 不借用整机 `production.launch.py` 来“顺便关闭其他节点”；
- 单设备 session manager 只期待该设备，不会因其他设备缺失而失败；
- 需要 trigger 的设备使用独立低速测试 schedule，默认输出 gate-off；
- `inventory`、`observe`、`acquire`、`motion/control` 是不同授权等级；
- 所有时限、最大帧数和最大写入字节数都有硬上限；
- Ctrl+C、超时和故障都走相同的安全停止与最终化路径。

在 Stage 1 身份尚未确认前，不提供一个看似方便但可能误开设备的通用硬件命令。

