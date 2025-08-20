#!/bin/bash

# ROS2-Gazebo Bridge Setup Script
# This script establishes communication between your Gazebo simulation and ROS2

echo "=== Setting up ROS2-Gazebo Bridge ==="

# 1. Check what topics Gazebo is publishing
echo "Checking Gazebo topics..."
gz topic -l

echo ""
echo "=== Starting ROS2-Gazebo Bridges ==="

# 2. Bridge clock (essential for ROS2-Gazebo sync)
echo "Bridging simulation clock..."
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock &

# 3. Bridge joint states (if available)
echo "Bridging joint states..."
ros2 run ros_gz_bridge parameter_bridge /world/default/model/jetrover/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model &

# 4. Bridge world info
echo "Bridging world info..."
ros2 run ros_gz_bridge parameter_bridge /world/default/stats@ros_gz_interfaces/msg/WorldInfo[gz.msgs.WorldStatistics &

# 5. Bridge pose info
echo "Bridging pose info..."
ros2 run ros_gz_bridge parameter_bridge /world/default/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V &

# Wait a moment for bridges to establish
sleep 2

echo ""
echo "=== Checking ROS2 Topics ==="
ros2 topic list

echo ""
echo "=== Bridge setup complete! ==="
echo "You can now use: ros2 topic echo /joint_states"
