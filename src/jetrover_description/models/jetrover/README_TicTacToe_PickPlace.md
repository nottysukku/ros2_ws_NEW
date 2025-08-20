# TicTacToe Robot - Pick and Place Tutorial for Beginners

## Overview
This tutorial will guide you through setting up ROS2 controls and implementing pick-and-place operations to move X and O pieces from the floor to the tic-tac-toe grid using your JetRover robot arm.

## Prerequisites
- **Ubuntu 24.04** with **ROS2 Jazzy Jalisco**
- **Gazebo Harmonic 8.9.0** (gz-sim)
- Your SDF environment is ready with working joints (j1, j2, j3, j4, j5, rfj, lfj)
- Basic understanding of terminal commands

**⚠️ Important Version Notes:**
- This tutorial is updated for ROS2 Jazzy and Gazebo Harmonic
- Package names and some commands differ from older ROS2/Gazebo versions
- Gazebo is now called `gz` instead of `gazebo`

---

## 1. Required ROS2 Packages Installation

### Install Essential Packages
```bash
# Navigate to your workspace
cd /home/sukritchopra/ros2_ws

# Install MoveIt2 and related packages for ROS2 Jazzy
sudo apt update
sudo apt install -y \
    ros-jazzy-moveit \
    ros-jazzy-moveit-visual-tools \
    ros-jazzy-moveit-servo \
    ros-jazzy-moveit-planners-ompl \
    ros-jazzy-moveit-simple-controller-manager \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-joint-trajectory-controller \
    ros-jazzy-position-controllers \
    ros-jazzy-effort-controllers \
    ros-jazzy-gripper-controllers \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-rqt-joint-trajectory-controller

# Install additional tools for Jazzy
sudo apt install -y \
    ros-jazzy-tf2-tools \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-control-msgs \
    ros-jazzy-moveit-msgs \
    ros-jazzy-trajectory-msgs

# Install Gazebo Harmonic integration (if not already installed)
sudo apt install -y \
    gz-harmonic \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-interfaces
```

### Build the workspace
```bash
cd /home/sukritchopra/ros2_ws
colcon build --symlink-install
source install/setup.bash

# Also source ROS2 Jazzy environment
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source /home/sukritchopra/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 2. Robot Configuration Files Setup

### 2.1 Create URDF Configuration
Create a URDF file that matches your SDF joints:

```bash
# Create URDF directory
mkdir -p src/jetrover_description/urdf
```

You'll need to create a URDF file (`jetrover.urdf.xacro`) that defines:
- All 7 joints: j1, j2, j3, j4, j5, left_finger_joint, right_finger_joint
- Joint limits and types
- Link geometry and inertias

### 2.2 Create MoveIt Configuration Package
```bash
cd src/
ros2 pkg create --build-type ament_cmake jetrover_moveit_config
```

This package will contain:
- `config/jetrover.srdf` - Semantic robot description
- `config/joint_limits.yaml` - Joint velocity/acceleration limits
- `config/kinematics.yaml` - Solver configuration
- `config/controllers.yaml` - ROS2 controller configuration

---

## 3. ROS2 Control Setup

### 3.1 Controller Configuration
Create `config/controllers.yaml`:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    use_sim_time: true  # Important for Gazebo Harmonic
    
    # Joint State Broadcaster
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    
    # Arm Controller (Updated for Jazzy)
    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    
    # Gripper Controller (Updated for Jazzy)
    gripper_controller:
      type: gripper_controllers/GripperActionController

arm_controller:
  ros__parameters:
    joints:
      - j1
      - j2
      - j3
      - j4
      - j5
    interface_name: position
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    
gripper_controller:
  ros__parameters:
    joint: left_finger_joint
    action_monitor_rate: 20.0
    goal_tolerance: 0.01
    max_effort: 5.0
    allow_stalling: true
    stall_velocity_threshold: 0.001
    stall_timeout: 1.0
```

### 3.2 Launch File for Controllers
Create `launch/controllers.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the controllers config file
    controllers_config = os.path.join(
        get_package_share_directory('jetrover_moveit_config'),
        'config',
        'controllers.yaml'
    )
    
    return LaunchDescription([
        # Controller Manager Node (Updated for Jazzy)
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[controllers_config],
            output='screen',
            arguments=['--ros-args', '--log-level', 'info']
        ),
        
        # Delay loading controllers to ensure controller manager starts
        TimerAction(
            period=2.0,
            actions=[
                # Load Controllers (Updated commands for Jazzy)
                ExecuteProcess(
                    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
                    output='screen'
                ),
            ]
        ),
        
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'arm_controller'],
                    output='screen'
                ),
            ]
        ),
        
        TimerAction(
            period=4.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'gripper_controller'],
                    output='screen'
                ),
            ]
        ),
    ])
```

---

## 4. Pick and Place Implementation

### 4.1 Create Pick and Place Node
Create `src/jetrover_pickplace/jetrover_pickplace/pick_place_node.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import moveit_py
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header
import time

class TicTacToePickPlace(Node):
    def __init__(self):
        super().__init__('tictactoe_pick_place')
        
        # Initialize MoveIt
        self.robot = moveit_py.MoveItPy(node_name="moveit_py")
        self.arm_group = "arm"  # Define in SRDF
        self.gripper_group = "gripper"  # Define in SRDF
        
        self.get_logger().info("TicTacToe Pick and Place Node Started")
    
    def pick_piece(self, piece_position):
        """Pick up a piece from the floor"""
        # 1. Move to pre-grasp position
        pre_grasp_pose = Pose()
        pre_grasp_pose.position.x = piece_position.x
        pre_grasp_pose.position.y = piece_position.y  
        pre_grasp_pose.position.z = piece_position.z + 0.1  # 10cm above piece
        pre_grasp_pose.orientation.w = 1.0
        
        self.move_to_pose(pre_grasp_pose)
        
        # 2. Open gripper
        self.open_gripper()
        
        # 3. Move to grasp position
        grasp_pose = pre_grasp_pose
        grasp_pose.position.z = piece_position.z + 0.02  # Just above piece
        self.move_to_pose(grasp_pose)
        
        # 4. Close gripper
        self.close_gripper()
        
        # 5. Lift piece
        lift_pose = grasp_pose
        lift_pose.position.z = piece_position.z + 0.15
        self.move_to_pose(lift_pose)
    
    def place_piece(self, grid_position):
        """Place piece on tic-tac-toe grid"""
        # 1. Move to pre-place position
        pre_place_pose = Pose()
        pre_place_pose.position.x = grid_position.x
        pre_place_pose.position.y = grid_position.y
        pre_place_pose.position.z = grid_position.z + 0.1
        pre_place_pose.orientation.w = 1.0
        
        self.move_to_pose(pre_place_pose)
        
        # 2. Move to place position
        place_pose = pre_place_pose
        place_pose.position.z = grid_position.z + 0.02
        self.move_to_pose(place_pose)
        
        # 3. Open gripper (release piece)
        self.open_gripper()
        
        # 4. Move away
        place_pose.position.z = grid_position.z + 0.1
        self.move_to_pose(place_pose)
    
    def move_to_pose(self, target_pose):
        """Move arm to target pose using MoveIt"""
        # Plan and execute motion
        self.robot.set_pose_target(target_pose, self.arm_group)
        success = self.robot.go(wait=True)
        return success
    
    def open_gripper(self):
        """Open gripper fingers"""
        # Set gripper joint positions for open state
        self.robot.set_joint_value_target("left_finger_joint", 0.15)  # Open position
        self.robot.set_joint_value_target("right_finger_joint", 0.15)
        self.robot.go(wait=True)
    
    def close_gripper(self):
        """Close gripper fingers"""
        # Set gripper joint positions for closed state
        self.robot.set_joint_value_target("left_finger_joint", 0.0)   # Closed position
        self.robot.set_joint_value_target("right_finger_joint", 0.0)
        self.robot.go(wait=True)

def main():
    rclpy.init()
    node = TicTacToePickPlace()
    
    # Example: Pick piece at (0.1, -0.16, 0.43) and place at grid position (0.05, 0.05, 0.42)
    piece_pos = Point(x=0.1, y=-0.16, z=0.43)
    grid_pos = Point(x=0.05, y=0.05, z=0.42)
    
    node.pick_piece(piece_pos)
    time.sleep(2)
    node.place_piece(grid_pos)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 5. Step-by-Step Execution

### Step 1: Launch Gazebo Harmonic with your robot
```bash
cd /home/sukritchopra/ros2_ws/src/jetrover_description/models/jetrover
# Note: Using 'gz sim' for Gazebo Harmonic (not 'gazebo')
gz sim tictactoe_NEW3.2.sdf

# Alternative: Launch with ROS2 bridge for better integration
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="tictactoe_NEW3.2.sdf"
```

### Step 2: Start ROS2 Controllers (in new terminal)
```bash
cd /home/sukritchopra/ros2_ws
source install/setup.bash
ros2 launch jetrover_moveit_config controllers.launch.py
```

### Step 3: Launch MoveIt (in new terminal)
```bash
source install/setup.bash
ros2 launch jetrover_moveit_config move_group.launch.py
```

### Step 4: Run Pick and Place Node (in new terminal)
```bash
source install/setup.bash
ros2 run jetrover_pickplace pick_place_node
```

---

## 6. Manual Control Commands

### Joint Control Commands
```bash
# Move individual joints
ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
trajectory: {
  joint_names: ['j1', 'j2', 'j3', 'j4', 'j5'],
  points: [{
    positions: [0.0, 0.5, -0.5, 0.0, 0.0],
    time_from_start: {sec: 2}
  }]
}}"

# Control gripper
ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{
command: {
  position: 0.15,  # Open gripper (0.0 = closed, 0.15 = open)
  max_effort: 5.0
}}"
```

### Monitor Joint States
```bash
# View current joint positions
ros2 topic echo /joint_states

# View transform tree
ros2 run tf2_tools view_frames
```

### Using RQT for GUI Control
```bash
# Joint trajectory controller GUI
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller

# General RQT with robot steering
rqt
```

---

## 7. Coordinate System Reference

Based on your SDF file, here are approximate coordinates for game pieces:

### X Pieces (White spheres)
- x_piece_1: (0.107, -0.010, 0.504)
- x_piece_2: (0.116, 0.069, 0.488)
- x_piece_3: (0.108, 0.102, 0.508)
- x_piece_4: (0.120, -0.051, 0.501)
- x_piece_5: (0.110, 0.028, 0.490)

### O Pieces (Black spheres)
- o_piece_1: (0.049, -0.160, 0.434)
- o_piece_2: (-0.032, -0.161, 0.440)
- o_piece_3: (-0.110, -0.157, 0.433)
- o_piece_4: (0.010, -0.161, 0.434)
- o_piece_5: (-0.070, -0.162, 0.439)

### Tic-Tac-Toe Grid Positions (9 squares)
```
Grid Layout:
[1,1] [1,2] [1,3]
[2,1] [2,2] [2,3]  
[3,1] [3,2] [3,3]
```
Approximate coordinates (you may need to adjust):
- Position [1,1]: (-0.05, -0.05, 0.42)
- Position [1,2]: (0.00, -0.05, 0.42)  
- Position [1,3]: (0.05, -0.05, 0.42)
- Position [2,1]: (-0.05, 0.00, 0.42)
- Position [2,2]: (0.00, 0.00, 0.42)
- Position [2,3]: (0.05, 0.00, 0.42)
- Position [3,1]: (-0.05, 0.05, 0.42)
- Position [3,2]: (0.00, 0.05, 0.42)
- Position [3,3]: (0.05, 0.05, 0.42)

---

## 8. Troubleshooting

### Common Issues

1. **Controllers not starting**
   ```bash
   # Check controller status
   ros2 control list_controllers
   
   # Restart controller manager
   ros2 service call /controller_manager/reload_controller_libraries controller_manager_msgs/srv/ReloadControllerLibraries
   ```

2. **MoveIt planning fails**
   ```bash
   # Check joint states
   ros2 topic echo /joint_states
   
   # Check planning scene
   ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene
   ```

3. **Joint limits exceeded**
   - Check `joint_limits.yaml` configuration
   - Verify SDF joint limits match URDF limits

4. **TF tree issues**
   ```bash
   # View TF tree
   ros2 run tf2_tools view_frames
   
   # Check for missing transforms
   ros2 run tf2_ros tf2_echo base_link end_effector
   ```

---

## 9. Next Steps

1. **Calibrate coordinates** - Test and adjust piece and grid positions
2. **Add vision** - Implement camera-based piece detection
3. **Game logic** - Create tic-tac-toe game state management
4. **Error handling** - Add collision detection and error recovery
5. **User interface** - Create a simple GUI for game interaction

---

## 10. Jazzy-Specific Compatibility Notes

### Important Differences from Older ROS2 Versions:

1. **Package Names**: All packages now use `ros-jazzy-*` prefix instead of `ros-humble-*`

2. **Gazebo Integration**: 
   - Gazebo Harmonic uses `gz sim` command instead of `gazebo`
   - Better integration with `ros_gz_sim` package
   - Updated bridge interfaces

3. **Controller Manager**:
   - Updated controller loading syntax: `--set-state active` flag
   - Improved timing with `TimerAction` in launch files
   - Better parameter handling

4. **MoveIt Updates**:
   - MoveIt2 for Jazzy has improved planning algorithms
   - Updated OMPL integration
   - Better collision detection

5. **Python API Changes**:
   - Some MoveIt Python API methods may have different signatures
   - Updated action interfaces for controllers

### Migration from Humble to Jazzy:
If you have existing ROS2 Humble code, main changes needed:
- Update package names in `package.xml` and CMakeLists.txt
- Update launch file syntax for controller loading
- Verify MoveIt Python API compatibility
- Update Gazebo launch commands

---

## 11. Useful Commands Cheat Sheet (Updated for Jazzy)

```bash
# Build workspace
colcon build --symlink-install

# Source environment (Jazzy)
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# List available controllers (Updated command)
ros2 control list_controllers

# Load and start controller in one command (Jazzy)
ros2 control load_controller --set-state active arm_controller

# Check joint states
ros2 topic echo /joint_states

# Send joint trajectory goal (same as before)
ros2 action send_goal /arm_controller/follow_joint_trajectory [goal_definition]

# Control gripper (Updated for gripper_controllers)
ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand

# Launch MoveIt planning (same structure)
ros2 launch jetrover_moveit_config move_group.launch.py

# View robot model
ros2 launch jetrover_description display.launch.py

# Gazebo Harmonic launch (Updated)
gz sim your_world.sdf

# ROS-Gazebo bridge launch (New for Harmonic)
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="your_world.sdf"
```

---

This tutorial provides a complete framework for implementing pick-and-place operations with your tic-tac-toe robot. Start with the basic setup and gradually add more sophisticated features as you become comfortable with the system.