# PPB-NG 四相机 ROS 2 联合采集测试

本文档适用于以下四台相机：Specim FX10e、Specim SWIR、FLIR Blackfly S RGB 和
FLIR A6701 thermal。GPS 与 RSM400 暂不参与此阶段测试。

## 设计要点

- FX10e 与 SWIR 使用各自独立的 `hsi_production_node.exe` 进程。这样隔离 Pleora 与
  NI/SpecSensor 运行时；一个进程崩溃不会直接破坏另一相机的地址空间。
- 两台 HSI 使用内部连续 line timing，不连接 Arduino 触发。
- RGB 与 thermal 使用 UNO R4 的同一个定时循环：D11 接 RGB，D12 接 thermal。
  两个引脚由一次 Port 4 masked write 同时改变，软件不引入连续两次 `digitalWrite` 的偏差。
- 当前经过实测的 Arduino 固件固定为 2 Hz、1 ms 高电平。ROS 启动时会核对配置，
  若不是 115200 baud、2 Hz、1 ms 就拒绝启动，避免配置文件与实际脉冲不一致。
- PPS 是可选项。本四相机测试不需要 PPS；每个触发仍保存 UNO `micros()` tick、脉冲序号、
  ROS 接收时间及相机自己的 timestamp/counter，但 UTC 质量明确标记为 `UNSYNCED`。

## 启动顺序

1. 创建唯一数据集目录并保存本次配置、可执行文件 SHA-256 和所有控制日志。
2. 启动五个惰性 ROS 节点；此时还没有节点访问硬件。
3. 对 FX10e、SWIR、RGB、thermal 和 timing 逐一执行 `prepare` 与 `arm`。
4. timing 节点打开 UNO 串口，等待开发板复位，发送 `STOP`，确认 D11/D12 为 LOW，
   再用 `STATUS` 核对引脚、频率、脉宽和 watchdog。
5. 串行初始化 FX10e 与 SWIR，两台内部 shutter 保持关闭。
6. 提示用户盖住两台 HSI 镜头；采集各自独立的暗场文件，并分别等待节点写出
   `dark_complete` 诊断证据，不使用固定秒数猜测完成状态。
7. 提示用户取下镜头盖并确认场景准备好。
8. RGB 与 thermal 先进入外触发等待状态。
9. FX10e 与 SWIR 开始内部连续 line-scan sample 采集。
10. 最后才向 UNO 发送 `START`。第一对 RGB/thermal 脉冲在一秒后产生。

因此，RGB/thermal 不会在相机尚未准备好时收到触发。两台 HSI 的启动服务是顺序调用的，
所以其最初几秒的开始时刻不会完全相同；此后每条线都连续采集并保留自己的相机/主机时间证据。
最终跨传感器 UTC 对齐将在接入 GPS/PPS 后完成。

## 停止顺序和故障安全

用户按 Enter、Ctrl+C、达到设定时长或流程遇到异常时，脚本依次执行：

1. `/timing/disarm_keep_config`：先发送 `STOP` 并等待 UNO 返回 `STOPPED`，D11/D12 LOW；
2. 停止 RGB 与 thermal，完成 raw segment checkpoint 并释放 Spinnaker handle；
3. 停止 FX10e 与 SWIR，关闭 shutter、落盘并释放 SpecSensor handle；
4. `/timing/stop`：刷新 timing event log 并释放 COM 口；
5. 结束 ROS launch 进程，并对 HSI 数据执行只读 quick verification。

UNO 固件还具有 3 秒 host watchdog。ROS timing 节点每 800 ms 发送一次 `KEEPALIVE`；
如果主机进程崩溃或串口通信中断，UNO 会自行将两个输出拉低。
如果正常 disarm 在 8 秒内没有得到 `STOPPED` 回答，脚本只终止 UNO ROS 节点并等待
4 秒（超过 firmware watchdog）后才开始关闭相机，同时将本次测试标记为失败。所有已经
prepare/arm 的设备都会尝试 stop，包括“设备端可能已经 start、但 ROS 回答丢失”的情况。

采集期间脚本持续检查五个子进程，每分钟调用 RGB、thermal 与 timing 的 ROS `status`
服务；任一节点退出或报告 fault，流程进入触发优先的安全停止。结束后除 HSI quick CRC
验证外，还会确认 RGB/thermal `.ppbseg` 和 UNO timing evidence 存在且非空。

## 测试前必须完成

1. 四台相机和 UNO 均正确接线、上电；A6701 已完成预热。
2. Arduino 中仍为：
   `firmware/arduino_uno_r4_rgb_thermal_trigger/arduino_uno_r4_rgb_thermal_trigger.ino`。
3. 在设备管理器确认 UNO 当前 COM 口，将
   `src/ppbng_bringup/config/ppbng_config.yaml` 中的：

   ```yaml
   timing:
     port: "COM__TO_BE_CONFIRMED__"
   ```

   改为实际端口，例如 `"COM9"`。不要猜测端口，也不要使用被其他程序占用的串口。
4. 关闭 SpinView、eBUS Player、HyperFusion、Arduino Serial Monitor，以及任何可能占用相机或串口的程序。
5. 确认 RGB 与 thermal 的触发线没有互换：D11 → RGB Line0，D12 → A6701 SYNC IN；
   三者必须有正确的共同参考地。

## 运行命令

在普通 PowerShell 中执行：

```powershell
cd C:\Users\cairlab\Desktop\Projects\PPB_NG_2026\ppbng_ros2_ws
.\tools\run_four_camera_test.cmd four_camera_test_01 30
```

参数分别为数据集名称和采集分钟数。程序会两次明确等待 Enter：第一次确认 HSI 镜头已盖好，
第二次确认暗场完成、镜头盖已取下且正式场景准备好。服务调用期间键盘输入会被丢弃，终端每
5 秒打印一次仍在等待的提示；不要在初始化过程中反复按 Ctrl+C。

数据保存在 `data/<名称_时间>/`：

- `segments/`：四相机 raw 数据、HSI ENVI index、RGB/thermal association sidecar、UNO timing events；
- `control/ppbng_config.yaml`：本次实际使用的配置快照；
- `control/build_identity.txt`：关键可执行文件与配置的 SHA-256；
- `control/service_*.log`：每个 ROS service 的完整请求结果；
- `control/four_camera_launch.*.log`：五个节点的运行日志。

## 当前验证边界

代码已完成编译、ROS package 测试、PowerShell/launch 语法检查和“未显式授权即拒绝”的离线测试。
尚未在四台相机同时连接时运行，因此不能把四相机联合链路标记为硬件验收通过。第一次实测建议
先运行 2–5 分钟，检查各流的帧数、segment、association sidecar 和退出日志，再进行 30 分钟及
两小时长测。

## 上传到现有 GitHub 仓库

当前 `ppbng_ros2_ws` 还不是 Git 仓库。`.gitignore` 已排除 `build/`、`install/`、`log/`、
`data/`、`hardware_test_data/`、缓存和 secrets，因此 raw 数据和本机构建产物不会被提交。

如果远程 GitHub 仓库是空仓库，可在本目录执行：

```powershell
git init -b main
git add .
git status
git commit -m "Add PPB-NG ROS 2 acquisition workspace"
git remote add origin https://github.com/OWNER/REPOSITORY.git
git push -u origin main
```

执行 `git add .` 后必须先阅读 `git status`，确认没有数据集、密钥、厂商 SDK/DLL 或标定文件。

如果远程仓库已经有 README、历史代码或分支，不要直接强制 push。应先连接并取得远端历史：

```powershell
git init -b main
git remote add origin https://github.com/OWNER/REPOSITORY.git
git fetch origin
git branch -r
```

然后根据远程默认分支和现有目录结构决定是在 `origin/main` 上创建新分支，还是把本 workspace
放入仓库的一个子目录。推荐首次上传使用新分支（例如 `codex/ppbng-ros2-integration`）并通过
Pull Request 合并；不要使用 `--force`。如果提供准确的 GitHub 仓库 URL 和期望的目录位置，
可以在下一步先只读检查远端分支，再给出不会覆盖历史的精确命令。
