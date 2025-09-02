#!/bin/bash
set -e

# Source ROS2 workspace
source ~/ros2_ws/install/setup.bash

# Start camera viewer (live feed)
python3 ~/ros2_ws/src/jetrover_description/scripts/camera_viewer.py &

# Start grid detector (GUI for cell detection)
python3 ~/ros2_ws/src/jetrover_description/scripts/grid_detector.py &



wait
