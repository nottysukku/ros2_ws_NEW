# Step-by-Step Guide: Camera-Based Grid Detection System

## 1. Launch Gazebo Simulation
```bash
cd ~/ros2_ws/src/jetrover_description/models/jetrover
gz sim tictactoe_NEW3.6.sdf
```

## 2. Start ROS2 Bridge for Camera
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=/home/sukritchopra/ros2_ws/src/jetrover_ros2_control/config/bridge_config.yaml
```

## 3. Install Python Dependencies
```bash
pip install opencv-python numpy rclpy sensor_msgs cv_bridge pillow
sudo apt install python3-tk
```

## 4. Launch Camera Detection System
```bash
cd ~/ros2_ws/src/jetrover_ros2_control/scripts
chmod +x launch_camera_system.sh
./launch_camera_system.sh
cd /home/sukritchopra/ros2_ws/src/jetrover_description/scripts
python3 terminal_output_gui.py
```

## 5. What You Should See
- **Camera Viewer Window**: Live overhead camera feed
- **Grid Detector Window**: GUI showing detected cell (e.g., "Cell filled: (0, 1)")

## 6. Troubleshooting
- If no camera feed: Check Gazebo camera sensor and bridge config
- If no GUI: Ensure python3-tk is installed
- If grid detection is off: Adjust grid mapping logic in grid_detector.py

## 7. Customization
- Edit grid boundaries and detection thresholds in grid_detector.py for your board layout
- For advanced overlays, modify camera_viewer.py

---
**You are now ready to run real-time camera-based grid detection in ROS2!**
