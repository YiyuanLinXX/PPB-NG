# PPB-NG 统一配置与无界面 HSI 运行说明

## 1. 唯一用户配置文件

正式系统的用户配置统一放在：

`src/ppbng_bringup/config/ppbng_config.yaml`

修改 YAML 后只需结束并重新启动 ROS 2 launch，不需要重新编译 package。文件已经集中
包含 session、时间基准、RGB、thermal、FX10e、SWIR、Arduino timing、GNSS 和 RSM400
参数。尚未完成硬件确认的字段使用明显的 `TO_BE_CONFIRMED` 或 `REQUIRED` 占位值，完整
生产 launch 会因此保持 fail-closed，避免误启动。

`src/ppbng_hsi/config/fx10e_hardware.yaml` 和 `swir_hardware.yaml` 是单设备开发/诊断基线，
不是日常操作员要维护的第二套配置。后续用户只修改上述统一文件。

## 2. FX10e 无界面连接

FX10e 的 `grabber_channel` 固定为相机持久地址 `192.168.10.2`。不要把它改成 `ui`：
`ui` 会要求 eBUS 弹出 Device Selection 窗口，在 SSH 会话中无人点击时初始化会一直等待。
节点在调用 SpecSensor `Initialize` 前主动写入并读回该固定连接标识，因此不依赖 eBUS Player
或任何图形界面。

2026-09-01 已用真实 FX10e 验证：从 ROS 服务发出 `start` 后约 9 秒完成初始化，没有出现
设备选择窗口；profile、序列号、图像几何、包长和工厂标定包均读回正确。测试保持快门关闭，
没有开始采集。

## 3. HSI 采集方式

FX10e 和 SWIR 都设置为：

```yaml
trigger_mode: "Internal"
```

这里的“软件控制”是 ROS 通过 SDK 控制一次 `Acquisition.Start` 和
`Acquisition.Stop`，相机在两者之间依靠内部 line clock 连续产生扫描线；不是由 ROS 为
每一行发送一个软件触发命令，也不使用 Arduino 外部脉冲。RGB 和 thermal 仍是 snapshot
相机，继续由 Arduino 的独立输出通道作外部触发。

内部连续模式下，每行保存 SpecSensor SDK frame number 和主机收到回调时的 monotonic
timestamp。没有外部 PPS/触发硬件证据时，软件不会虚假宣称每行已锁定到 GPS 时间；状态保留
为 `UNSYNCED/UNVERIFIED`，供后处理结合主机—GPS 时钟关系和相邻 GNSS 样本插值。

## 4. 曝光时间与行频

`line_rate_hz` 和 `exposure_us` 受读出周期约束，不能任意组合。推荐流程是：

1. 在室外实际光照和机器人工作几何下，用厂商软件找到稳定组合。
2. 停止并退出厂商软件，避免占用相机。
3. 同时更新统一 YAML 中对应相机的两个参数。
4. 重启 launch。节点会应用参数并读回；超出有限硬件时钟量化容差时拒绝采集。

当前经硬件读回的开发基线为 FX10e `50 Hz / 18000 us`，SWIR 请求值
`20.59 Hz / 17800 us`。SWIR 实际量化值曾读回为 `20.590113 Hz / 17799.200058 us`。

## 5. 暗场与白参考

任务开始前只采暗场：操作员盖好镜头并确认后，ROS 关闭快门并按
`session.hsi_dark_duration_seconds` 采集两台相机的 dark lines。launch 根据各自 line rate
自动换算暗场行数，修改行频时不需要手工同步修改暗场行数。

两条扫描线上始终可见 Spectralon reference bar，因此不在采集程序内另做独立 white
reference。dark reference、场景中的 white bar 和原始 HSI 数据全部保留，flat-field
correction 在后处理完成。

## 6. 单相机 HSI 调试启动

FX10e：

```powershell
.\tools\ros2.cmd launch ppbng_bringup hsi_camera.launch.py camera:=fx10e hardware_enabled:=true
```

SWIR：

```powershell
.\tools\ros2.cmd launch ppbng_bringup hsi_camera.launch.py camera:=swir hardware_enabled:=true
```

在当前 workspace 内，这两个 launch 默认优先读取源目录中的统一 YAML，因此修改后只需
重启 launch，不需要 build。部署到没有源码的目标机时会自动回退到安装副本，也可以通过
`config_file:=C:/绝对路径/ppbng_config.yaml` 指定外部配置。

单设备节点启动后仍处于 inert 状态。完整采集必须依次完成 session prepare、arm、start、
暗场确认和 sample 确认；这条安全门防止仅执行 launch 就意外开流或覆盖数据。

推荐使用可复刻的任务脚本，而不是手工逐条调用服务：

```powershell
# 交互式双机 30 分钟测试；程序会提示盖/取镜头盖
.\tools\run_dual_hsi_long_test.cmd my_hsi_test 30 both

# FX10e 或 SWIR 单机测试
.\tools\run_dual_hsi_long_test.cmd my_fx_test 15 fx10e
.\tools\run_dual_hsi_long_test.cmd my_swir_test 15 swir
```

脚本也支持 `-Unattended`。该模式利用相机内部机械快门采集 dark，随后立即打开快门采集
当前场景，不等待人员盖/取镜头盖：

```powershell
.\tools\run_dual_hsi_long_test.cmd my_unattended_test 30 both -Unattended
```

每次任务都会保存配置副本、配置与可执行文件 SHA-256、ROS stdout/stderr、两路
`*_events.ndjson`、本轮 Specim 日志，并在安全 stop 后执行带进度的快速校验。因此从普通终端
或 SSH 调用时无需 eBUS 图形界面或额外点击。没有 Win32 console 的 SSH/后台任务必须使用
`-Unattended`；脚本不会读取不可用的 console handle。正式科学暗场仍推荐使用交互式流程并
盖好镜头，无人值守模式主要用于稳定性测试。

Windows 上每条 `ros2 service call` 需要重新加载 CLI 并完成 DDS discovery，通常需 5–10 秒；
两台相机的 `start` 还包含 SDK 枚举、身份确认、配置与标定加载，单台可能需 20–30 秒。脚本
现每 5 秒显示一次 `Still waiting ...`，并把服务 CLI 放到隐藏进程中，避免键盘 Ctrl+C 字符
破坏 `ros2.cmd`。看到 `Calling ...` 后应等待明确的 `response:`；只有在盖/取镜头盖提示处
按一次 Enter，或进入正式 sample 循环后按一次 Enter/Ctrl+C 请求提前停止。服务事务进行中
的额外按键会被安全忽略并显示黄色提示。

## 7. 当前边界

- FX10e 无界面连接已通过真实硬件验证；SWIR 不使用 eBUS，走 NI `img0`，不存在同类弹窗。
- 两台 HSI 各自初始化、FX 15 分钟单机及双相机 30 分钟 Internal 连续采集均已通过真实
  硬件验证；完整机器人跨传感器同步与全系统 2 小时验收仍需继续。
- 完整生产配置暂不启用，GNSS/RSM 串口、吞吐证据等占位项确认后再设置
  `safety.configured: true`。
