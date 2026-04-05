#!/bin/bash
source /opt/ros/noetic/setup.bash

# Start Lidar
source /opt/lidar_ws/devel/setup.bash
roslaunch ouster_ros sensor.launch sensor_hostname:=10.42.0.185 &
LIDAR_PID=$!

# Wait For Lidar to start
echo "[INFO] Waiting for /os_node to be available..."
while ! rosnode list | grep -q '/os_node'; do
    sleep 1
done

# Start Camera
source /opt/camera_ws/devel/setup.bash
roslaunch realsense2_camera rs_camera.launch &
CAMERA_PID=$!

# -------------------------------
# Wait for both to keep running
# -------------------------------
wait $LIDAR_PID $CAMERA_PID