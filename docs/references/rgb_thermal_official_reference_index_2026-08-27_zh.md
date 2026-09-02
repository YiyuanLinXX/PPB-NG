# RGB + Thermal 官方参考资料索引

整理日期：2026-08-27

本索引保存本阶段使用的官方资料入口和与代码设计直接相关的结论。网页内容可能随厂商版本变化；后续修改相机配置前应重新核对相应型号、SDK 版本和页面修订日期。

## FLIR Blackfly S BFS-U3-123S6C

1. [BFS-U3-123S6C 型号规格](https://softwareservices.flir.com/BFS-U3-123S6/latest/Model/spec.html)
   - 4096 × 3000，Sony IMX253，全局快门；
   - 列出 BayerRG8、BayerRG10p、BayerRG12p、BayerRG16 等格式；
   - 各格式的满分辨率最大帧率均高于本项目当前 2 Hz 需求。

2. [自动曝光、自动增益和自动白平衡控制](https://softwareservices.flir.com/BFS-U3-123S6/latest/Model/public/AutoAlgorithmControl.html)
   - ExposureAuto/GainAuto 可使用 Once 或 Continuous；
   - 可设置自动算法上下限、优先级、测光方式、ROI 和 damping；
   - Gain priority 以低增益/低噪声为目标，Exposure Time priority 可用于限制运动场景曝光；
   - 白平衡提供 Outdoor profile。

3. [Chunk Data Control](https://softwareservices.flir.com/BFS-U3-123S6/latest/Model/public/ChunkDataControl.html)
   - 支持 FrameID、Timestamp、ExposureTime、Gain、BlackLevel、PixelFormat、CRC 等每帧 Chunk；
   - Chunk 必须在开始采集前配置；
   - 这是后续实现每帧曝光可追溯性的主要依据。

4. [BFS-U3-123S6C 输入输出控制](https://softwareservices.flir.com/BFS-U3-123S6/latest/40-Installation/InputOutputControl.htm)
   - 用于核对 GPIO、电气输入和触发连接要求；
   - 实际接线仍应以相机随附型号手册、接口定义和仪器测量为准。

## FLIR A6701 / 热成像定量基础

1. [FLIR 热成像测量基础资料](https://support.flir.com/DSDownload/Assets/T810442-en-US_A4.pdf)
   - 用于理解发射率、反射、环境和大气对温度测量的影响。

2. [FLIR Object Parameters 说明](https://docs.flir.com/T810605/en-US/latest/s10.html)
   - 物体温度修正涉及 emissivity、reflected temperature、distance、humidity、atmosphere 和 external optics；
   - 原始辐射数据和这些参数应分别保存，便于后处理复算。

## Arduino UNO R4 WiFi

1. [Arduino UNO R4 WiFi 官方数据表](https://docs.arduino.cc/resources/datasheets/ABX00087-datasheet.pdf)
   - 用于核对 RA4M1 I/O 电气特性和板级接口；
   - 本项目的 D11/D12 双路触发由同一端口掩码操作更新，但相机输入保护、地线和电平兼容仍应按完整硬件链路验证。

## 本地厂商资料

项目中已有的 A6701、Spinnaker 和其他设备 PDF/手册仍应作为离线权威资料保存。若网页说明与相机随附手册、SDK 4.4 节点实际读回不一致，应记录差异，并优先依据明确适用于当前型号与固件版本的资料和只读实机读回。
