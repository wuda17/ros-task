#bash

# Launch Limo Controller in Sim
ros2 launch limo_control limo_controller_sim.launch.py

# Publish a goal_pose command
X=1.0 # m
Y=2.0 # m 
THETA=0.5236  # radians

# Compute quaternion components from a 2D yaw angle 
QZ=$(echo "scale=4; s($THETA/2)" | bc -l)
QW=$(echo "scale=4; c($THETA/2)" | bc -l)

ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: $X, y: $Y, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: $QZ, w: $QW}}}" -1