# ROS2 Commands for Gazebo Simulation Control & Diagnostics

## 1. Node Information Commands

### List all running nodes
```bash
ros2 node list
```

### Get detailed info about a specific node
```bash
ros2 node info /node_name
```

### List nodes with their types
```bash
ros2 node list --verbose
```

---

## 2. Topic Commands

### List all active topics
```bash
ros2 topic list
```

### List topics with their message types
```bash
ros2 topic list -t
```

### Echo/monitor topic data in real-time
```bash
# Monitor joint states
ros2 topic echo /joint_states

# Monitor clock (simulation time)
ros2 topic echo /clock

# Monitor robot pose/transforms
ros2 topic echo /tf
ros2 topic echo /tf_static
```

### Get topic information
```bash
ros2 topic info /joint_states
ros2 topic info /clock
```

### Publish to a topic (manual control)
```bash
# Example: Publish joint positions (if you have joint position topics)
ros2 topic pub /joint_command sensor_msgs/msg/JointState "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''},
  name: ['j1', 'j2', 'j3', 'j4', 'j5'],
  position: [0.0, 0.5, -0.5, 0.0, 0.0],
  velocity: [],
  effort: []
}" --once
```

---

## 3. Service Commands

### List all available services
```bash
ros2 service list
```

### List services with their types
```bash
ros2 service list -t
```

### Call a service
```bash
# Gazebo-specific services
ros2 service call /world/default/create gazebo_msgs/srv/SpawnEntity

# Get world properties
ros2 service call /world/default/info gazebo_msgs/srv/GetWorldInfo

# Pause/unpause simulation
ros2 service call /world/default/control gazebo_msgs/srv/WorldControl "{pause: true}"
ros2 service call /world/default/control gazebo_msgs/srv/WorldControl "{pause: false}"

# Reset simulation
ros2 service call /world/default/control gazebo_msgs/srv/WorldControl "{reset: {all: true}}"
```

---

## 4. Action Commands

### List all action servers
```bash
ros2 action list
```

### List actions with their types
```bash
ros2 action list -t
```

### Send action goals
```bash
# Example: Control gripper (if gripper action server is running)
ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{
  command: {
    position: 0.15,
    max_effort: 5.0
  }
}" --feedback

# Example: Joint trajectory control (if arm controller is running)
ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: ['j1', 'j2', 'j3', 'j4', 'j5'],
    points: [{
      positions: [0.0, 0.5, -0.5, 0.0, 0.0],
      time_from_start: {sec: 2, nanosec: 0}
    }]
  }
}" --feedback
```

---

## 5. Parameter Commands

### List all parameters
```bash
ros2 param list
```

### Get parameter value
```bash
ros2 param get /node_name parameter_name
```

### Set parameter value
```bash
ros2 param set /node_name parameter_name value
```

### Dump all parameters for a node
```bash
ros2 param dump /node_name
```

---

## 6. Gazebo-Specific Commands

### Bridge Gazebo topics to ROS2
```bash
# Start ROS-Gazebo bridge (if not already running)
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock

# Bridge joint states
ros2 run ros_gz_bridge parameter_bridge /world/default/model/jetrover/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model
```

### Get Gazebo world information
```bash
# List all models in the world
gz model --list

# Get model info
gz model --model-name jetrover --info

# Get joint info
gz joint --model jetrover --list

# Move a model
gz model --model-name jetrover --pose "0 0 1 0 0 0"
```

---

## 7. Transform (TF) Commands

### View TF tree
```bash
ros2 run tf2_tools view_frames
# This creates a PDF file showing the transform tree
```

### Echo transform between frames
```bash
ros2 run tf2_ros tf2_echo base_link gripper_base_link
ros2 run tf2_ros tf2_echo world base_link
```

### List all frames
```bash
ros2 run tf2_ros tf2_monitor
```

---

## 8. Real-time Monitoring Commands

### Monitor multiple topics simultaneously
```bash
# Monitor joint states and simulation time
ros2 topic echo /joint_states --field name,position &
ros2 topic echo /clock --field clock.sec &
```

### Check system performance
```bash
# Check CPU usage of nodes
ros2 node list | xargs -I {} sh -c 'echo "=== {} ==="; ros2 node info {}'

# Monitor message frequency
ros2 topic hz /joint_states
ros2 topic hz /clock
```

---

## 9. Direct Joint Control (if gz-ros2-control is running)

### Check if controllers are loaded
```bash
ros2 control list_controllers
```

### Load and start controllers
```bash
# Load controller
ros2 control load_controller arm_controller

# Start controller
ros2 control set_controller_state arm_controller active

# List hardware interfaces
ros2 control list_hardware_interfaces
```

### Direct joint control commands
```bash
# Set joint positions (if position interface is available)
ros2 topic pub /j1_position_controller/commands std_msgs/msg/Float64 "{data: 0.5}" --once
ros2 topic pub /j2_position_controller/commands std_msgs/msg/Float64 "{data: -0.3}" --once
```

---

## 10. Debugging Commands

### Check for missing dependencies
```bash
# Check if all required packages are installed
ros2 pkg list | grep moveit
ros2 pkg list | grep control
ros2 pkg list | grep gz
```

### Verify communication
```bash
# Check if Gazebo is publishing joint states
ros2 topic echo /joint_states --once

# Check simulation time
ros2 topic echo /clock --once

# Verify robot model is loaded
ros2 service call /world/default/scene/info gazebo_msgs/srv/GetSceneInfo
```

### Log information
```bash
# Set log level for debugging
ros2 run rclcpp_components component_container --ros-args --log-level DEBUG

# View log output
ros2 run rqt_console rqt_console
```

---

## 11. Interactive Tools

### RQT Tools
```bash
# Robot steering GUI
rqt

# Joint state publisher GUI
ros2 run joint_state_publisher_gui joint_state_publisher_gui

# Joint trajectory controller GUI  
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller

# Service caller GUI
ros2 run rqt_service_caller rqt_service_caller

# Topic monitor GUI
ros2 run rqt_topic rqt_topic

# Graph visualization
ros2 run rqt_graph rqt_graph
```

---

## 12. Quick Diagnostic Script

Create a quick diagnostic script:
```bash
#!/bin/bash
echo "=== ROS2 Gazebo Diagnostics ==="
echo
echo "Active Nodes:"
ros2 node list
echo
echo "Active Topics:"
ros2 topic list
echo
echo "Available Services:"  
ros2 service list | head -10
echo
echo "Joint States (latest):"
timeout 2s ros2 topic echo /joint_states --once
echo
echo "Simulation Time:"
timeout 2s ros2 topic echo /clock --once
echo
echo "Controllers Status:"
ros2 control list_controllers 2>/dev/null || echo "No controllers found"
```

Save this as `diagnostic.sh`, make it executable with `chmod +x diagnostic.sh`, and run with `./diagnostic.sh`.

---

## Usage Examples for Your Current Simulation:

```bash
# 1. Check what's running
ros2 node list
ros2 topic list

# 2. Monitor your robot's joint states
ros2 topic echo /joint_states

# 3. Check if Gazebo simulation is running
ros2 topic echo /clock

# 4. Try to control joints (if controllers are loaded)
ros2 control list_controllers

# 5. View system graph
ros2 run rqt_graph rqt_graph
```

These commands will help you inspect and control your running Gazebo simulation in real-time!
