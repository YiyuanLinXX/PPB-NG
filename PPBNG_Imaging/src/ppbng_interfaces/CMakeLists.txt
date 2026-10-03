cmake_minimum_required(VERSION 3.16)
project(ppbng_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AcquisitionStatus.msg"
  "msg/DeviceStatus.msg"
  "msg/FaultEvent.msg"
  "msg/FrameMetadata.msg"
  "msg/FrameContext.msg"
  "msg/GnssObservation.msg"
  "msg/PpsAnchor.msg"
  "msg/RsmTelemetry.msg"
  "msg/SampleStamp.msg"
  "msg/TimeQuality.msg"
  "msg/ThermalNucState.msg"
  "msg/TriggerEvent.msg"
  "srv/AbortAcquisition.srv"
  "srv/ConfirmDarkReady.srv"
  "srv/ConfirmSampleReady.srv"
  "srv/PrepareDevice.srv"
  "srv/ResetRsmFaults.srv"
  "srv/SetRsmTarget.srv"
  "srv/StartAcquisition.srv"
  "srv/StopAcquisition.srv"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
