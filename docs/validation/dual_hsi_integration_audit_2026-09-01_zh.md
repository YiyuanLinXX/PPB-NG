# 双 HSI 集成审计（2026-09-01）

## 结论

FX10e 与 SWIR 的 ROS 节点采用两个独立进程。这样每台相机具有独立的
SpecSensor library lifetime、设备 handle、回调队列、ENVI writer、恢复状态和
ROS service namespace；一台相机的进程崩溃不会直接破坏另一进程的内存或 SDK
全局状态。SpecSensor 头文件定义了 `siHOSTThreadNotSafe`，因此当前每个节点使用
单线程 executor、仅在 SDK callback 中复制并入队数据，所有控制 API 都留在节点
executor 线程执行。不要把两个节点改为任意的多线程 component container，除非先
完成厂商线程模型验证。

这是一项架构和离线验证结论，不等于双相机并行硬件验证。双进程能否在该版本
SpecSensor、Pleora 和 NI 驱动上长时间并行稳定，必须由真实的并行采集测试确认。

## 已核查的生产工作流

1. `prepare` 只绑定已经存在的数据集子目录，不打开 SDK。
2. `arm` 校验设备身份、几何、缓存上限、输出目录和配置，但仍不打开硬件。
3. `start` 分别打开设备、应用并读回配置、关闭快门。
4. 用户确认遮盖镜头后，两个 `begin_dark` 并发启动；各节点只在全部暗场行已经
   持久化且 SDK acquisition 已停止后发布 `dark_complete`。
5. production manager 必须收到两台相机的完成状态才允许 sample confirmation。
6. `start_sample` 打开快门并开始内部连续 line timing；RGB/thermal 随后由 timing
   controller 的共享外部脉冲启动。
7. stop/fault 顺序先 disarm snapshot trigger，再停止全部相机，最后停止后台数据源
   并关闭 context writer。

## 时间与同步边界

FX10e/SWIR 没有逐行外部触发，因此每行保存 SDK frame number 和在 SDK callback
入口采集的 Windows steady-clock monotonic timestamp。`frame_context_node` 使用这个
同一主机单调时钟，把 HSI 行与 GNSS、RSM400 数据做最近邻/插值关联。它不能把这种
关联声明成硬件同步或 PPS locked：HSI `SampleStamp` 必须保持 `UNSYNCED`，原始证据
仍完整保存，供后处理和误差评估使用。

## 本轮加固

- production launch 与单相机 launch 都从唯一 `ppbng_config.yaml` 读取 SDK 初始化
  超时、ENVI segment 上限及 flush 周期。
- production 中两个 HSI 进程都显式注入同一 SpecSensor `bin/x64` DLL 路径，不依赖
  交互式终端的 PATH。
- production 在创建任何硬件进程前拒绝以下配置：HSI section/camera 被禁用、任何
  HSI 使用 External、对应 Arduino HSI channel 被启用、FX10e channel 为空或 `ui`、
  初始化超时或 flush 周期无效。
- 两台相机分别保留有界 callback queue 和恢复状态。短暂 transport fault 只在本节点
  内做有界恢复；storage/integrity fault 或恢复耗尽发布 fault，由 production manager
  执行全局有序停止。
- FX10e 无界面初始化先前实测约 9 秒，而旧的 production batch timeout 只有 5 秒，
  会产生“仍在正常初始化却被 manager 判定失联”的竞态。统一配置已把 batch timeout
  调为 30 秒，launch 并强制它必须大于 SDK initialization timeout。

## 尚需真实硬件关闭的交付门槛

- 双 HSI 同时初始化无 eBUS GUI、设备身份与配置读回完全匹配。
- 双暗场完成，并核对暗场行数、快门状态、ENVI/CSV/索引一致性。
- 双 sample 连续采集，先做短测，再做至少 2 小时；验证无 callback overflow、frame
  number gap、丢行、进程残留或磁盘吞吐不足。
- 分别模拟 FX10e 网络中断和 SWIR/NI transport 中断，确认另一进程不崩溃、故障证据
  落盘、恢复/全局停止符合策略。
- 接入 GNSS 与 RSM400 后，抽样核对每条 HSI sample 的 host monotonic context 关联；
  状态必须诚实保留为软件时间关联，而非硬件锁定。
- 最后与 RGB/thermal 同时运行完整生产 workflow，并验证 Arduino 只触发 snapshot
  相机，HSI timing outputs 保持禁用。

## 离线结果

`ppbng_hsi` 与 `ppbng_bringup` 的所选离线测试全部通过，0 failure；两 package
均已构建并安装成功。并行开发期间全 workspace 的测试总数会继续变化，因此以
最终整库回归报告为准。
本审计未打开相机、frame grabber 或修改任何网络/系统设置。
