#!/bin/bash
set -e

# Build workspace
cd /root/workspace
colcon build --symlink-install
source /root/workspace/install/setup.bash

# Launch simulation + controller in background
ros2 launch limo_control limo_controller_sim.launch.py &
SIM_PID=$!

# Wait for sim to start
sleep 4

# Publish 2D goal pose
X=25.0
Y=30.0
# Given THETA=30 deg
QZ=0.259  # sin(THETA/2)
QW=0.966  # cos(THETA/2)

ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'map'}, pose: {position: {x: $X, y: $Y, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: $QZ, w: $QW}}}" -1

ros2 run rqt_plot rqt_plot /l2_error /angular_error

# Optional: wait for sim/controller
wait $SIM_PID