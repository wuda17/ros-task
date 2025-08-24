#!/bin/bash

cd /root/workspace

colcon build --symlink-install

source /root/workspace/install/setup.bash

# Launch Limo Controller in Sim
ros2 launch limo_control limo_controller_sim.launch.py

# Publish a goal_pose command
X=1.0 # m
Y=2.0 # m 

# Given yaw angle THETA=0.5236 rad (~30 deg)
# Compute quaternion components for 2D rotation around Z:
#   HALF_THETA = THETA / 2 = 0.2618
#   q_z = sin(HALF_THETA) ~= 0.259
#   q_w = cos(HALF_THETA) ~= 0.966
# Final quaternion for 2D rotation: [x=0, y=0, z=0.259, w=0.966]
QZ=0.259  # sin(THETA/2)
QW=0.966  # cos(THETA/2)

ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: $X, y: $Y, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: $QZ, w: $QW}}}" -1