#!/usr/bin/env bash

# -----------------------------------------------------------------------------
# Run all ROS 2 nodes for RGB + Thermal imaging, GPS sync, and Arduino trigger
# -----------------------------------------------------------------------------

# 1. Workspace and environment
WORKSPACE=~/PPB_NG
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

# 2. Ask user for output subfolder name
read -p "Enter output folder name (e.g., Oblock_20250709_1600): " USER_TAG
OUTPUT_DIR="/media/Data/cairlab/${USER_TAG}"
GPS_LOG="$OUTPUT_DIR/gps_log.csv"

mkdir -p "$OUTPUT_DIR/RGB"
mkdir -p "$OUTPUT_DIR/thermal"
echo "[INFO] Data will be saved to: $OUTPUT_DIR"

# 3. Common parameters
# GPS receiver
GPS_PORT="/dev/serial/by-id/usb-Emlid_ReachRS3_8243559D61EFB119-if02"
GPS_BAUD=115200

# Arduino
ARDUINO_PORT="/dev/serial/by-id/usb-Arduino_UNO_WiFi_R4_CMSIS-DAP_F412FA75EC94-if01"
ARDUINO_BAUD=9600

# RGB Camera
EXPOSURE_TIME=250.0  # in microseconds
GAIN=5.0
WB_RED=1.34
WB_BLUE=2.98
RGB_SERIAL="22209867"

# Thermal camera
THERMAL_DEVICE_ID="00111C0408CD"
CALIBRATION_TAG="17mm, Empty, 10C - 90C"

# 4. Register trap to kill all background ROS 2 nodes gracefully
PIDS=()
cleanup() {
  echo "[INFO] Caught interrupt. Shutting down all ROS 2 nodes..."
  for pid in "${PIDS[@]}"; do
    kill "$pid" 2>/dev/null
  done
  wait
  echo "[INFO] All nodes shut down. Exit."
  exit 0
}
trap cleanup SIGINT SIGTERM

# 5. Launch gps_publisher
echo "[INFO] Launching gps_publisher_node..."
ros2 run ppbng_imaging gps_publisher_node \
  --ros-args \
    -p port:="$GPS_PORT" \
    -p baud:=$GPS_BAUD &
PIDS+=($!)
sleep 1

# 6. Launch gps_logger
echo "[INFO] Launching gps_logger_node..."
ros2 run ppbng_imaging gps_logger_node \
  --ros-args \
    -p log_file:="$GPS_LOG" &
PIDS+=($!)
sleep 1

# 7. Launch rgb_acquisition_node
echo "[INFO] Launching rgb_acquisition_node..."
ros2 run ppbng_imaging rgb_acquisition_node \
  --ros-args \
    -p output_dir:="$OUTPUT_DIR/RGB" \
    -p exposure_time:=$EXPOSURE_TIME \
    -p gain:=$GAIN \
    -p wb_red:=$WB_RED \
    -p wb_blue:=$WB_BLUE \
    -p rgb_serial:="\"$RGB_SERIAL\"" &
PIDS+=($!)
sleep 5

# 8. Launch thermal_acquisition_node
echo "[INFO] Launching thermal_acquisition_node..."
ros2 run ppbng_imaging thermal_acquisition_node \
  --ros-args \
    -p output_dir:="$OUTPUT_DIR" \
    -p calib_tag:="$CALIBRATION_TAG" \
    -p thermal_device_id:="$THERMAL_DEVICE_ID" &
PIDS+=($!)
sleep 5

# 9. Launch arduino_com_node to coordinate triggering
echo "[INFO] Launching arduino_com_node..."
ros2 run ppbng_imaging arduino_com_node \
  --ros-args \
    -p port:="$ARDUINO_PORT" \
    -p baud:=$ARDUINO_BAUD
PIDS+=($!)
sleep 1

# 10. Wait for all nodes
echo "[INFO] All nodes started. Press Ctrl+C to stop."
wait
