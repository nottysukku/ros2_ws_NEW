# TicTacToe Pick & Place Robot - Gazebo to Real-World Transition

## 🎯 Project Overview
This project demonstrates a 6-DOF robot arm performing tic-tac-toe ball pickup and placement tasks, transitioning from Gazebo simulation to real-world implementation using MoveIt2.

## 📁 Current Project Structure
```
models/jetrover/
├── meshes_1/                    # STL files for arm visualization
├── tictactoe_NEW3.4.sdf        # Gazebo simulation environment
├── move1.sh                    # Shell script for joint control
└── [STL files for arm links]   # 3D models for each joint
```

## 🚀 Roadmap: From Simulation to Real-World Robot Control

### Phase 1: Foundation Setup (Beginner Level)

#### Step 1.1: Install ROS2 and MoveIt2
```bash
# Install ROS2 Jazzy (if not already installed)
sudo apt update && sudo apt install ros-jazzy-desktop

# Install MoveIt2 and dependencies
sudo apt install ros-jazzy-moveit ros-jazzy-moveit-planners ros-jazzy-moveit-plugins
sudo apt install ros-jazzy-joint-state-publisher-gui ros-jazzy-robot-state-publisher
sudo apt install ros-jazzy-rviz2 ros-jazzy-tf2-tools
```

#### Step 1.2: Create Robot Description Package
```bash
# Navigate to your workspace
cd ~/ros2_ws/src

# Create URDF package for your robot
ros2 pkg create --build-type ament_cmake jetrover_description_urdf
cd jetrover_description_urdf

# Create necessary directories
mkdir -p urdf meshes config launch rviz
```

#### Step 1.3: Convert SDF to URDF
**Why needed**: MoveIt2 works with URDF, not SDF files.
```bash
# Install conversion tools
sudo apt install ros-jazzy-gazebo-ros-pkgs

# Manual conversion steps (detailed below)
```

### Phase 2: URDF Creation and Validation

#### Step 2.1: Create Base URDF from Your SDF
Create `jetrover_description_urdf/urdf/jetrover.urdf.xacro`:
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="jetrover">
  
  <!-- Import your existing STL meshes -->
  <xacro:property name="mesh_path" value="package://jetrover_description_urdf/meshes"/>
  
  <!-- Base Link (from your base_footprint) -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.055" length="0.065"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0.4 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.055" length="0.065"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.0015625" ixy="0" ixz="0" iyy="0.0015625" iyz="0" izz="0.00125"/>
    </inertial>
  </link>
  
  <!-- Joint 1 -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.051" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-2.09" upper="2.09" effort="25" velocity="2"/>
  </joint>
  
  <!-- Link 1 (using your STL) -->
  <link name="link1">
    <visual>
      <geometry>
        <mesh filename="${mesh_path}/arm/link1.STL" scale="0.7 0.7 0.7"/>
      </geometry>
      <material name="blue">
        <color rgba="0.016 0.663 0.918 1"/>
      </material>
    </visual>
    <!-- Add collision and inertial properties -->
  </link>
  
  <!-- Continue for all 6 joints... -->
  
</robot>
```

#### Step 2.2: Validate URDF
```bash
# Check URDF syntax
check_urdf ~/ros2_ws/src/jetrover_description_urdf/urdf/jetrover.urdf.xacro

# Visualize in RViz
ros2 launch robot_state_publisher view_robot.launch.py model:=jetrover.urdf.xacro
```

### Phase 3: MoveIt2 Configuration

#### Step 3.1: Generate MoveIt2 Configuration Package
```bash
# Install MoveIt Setup Assistant
sudo apt install ros-jazzy-moveit-setup-assistant

# Launch Setup Assistant
ros2 launch moveit_setup_assistant setup_assistant.launch.py

# Follow GUI steps:
# 1. Load your URDF file
# 2. Generate Self-Collision Matrix
# 3. Add Virtual Joints (base_link to world)
# 4. Add Planning Groups (arm: joint1-joint5, gripper: finger joints)
# 5. Add Robot Poses (home position, tic-tac-toe ready)
# 6. Add End-Effectors (gripper)
# 7. Generate Configuration Files
```

#### Step 3.2: Configure Planning Groups
**Arm Group**: joint1, joint2, joint3, joint4, joint5
**Gripper Group**: left_finger_joint, right_finger_joint
**End-Effector**: gripper attached to link5

#### Step 3.3: Test MoveIt2 Configuration
```bash
# Build your workspace
cd ~/ros2_ws && colcon build

# Launch MoveIt2 demo
ros2 launch jetrover_moveit_config demo.launch.py

# Test planning in RViz - you should see your robot model
```

### Phase 4: Cartesian Motion Planning

#### Step 4.1: Create Cartesian Control Node
Create `jetrover_cartesian_control/src/cartesian_controller.cpp`:
```cpp
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class CartesianController {
public:
  CartesianController() : move_group("arm") {
    move_group.setPlanningTime(10.0);
  }
  
  bool moveToXYZ(double x, double y, double z) {
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;  
    target_pose.position.z = z;
    target_pose.orientation.w = 1.0;
    
    move_group.setPoseTarget(target_pose);
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
    if (success) {
      move_group.execute(plan);
    }
    
    return success;
  }
  
private:
  moveit::planning_interface::MoveGroupInterface move_group;
};
```

#### Step 4.2: Create Python Interface (Easier for Beginners)
Create `jetrover_cartesian_control/scripts/cartesian_control.py`:
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import Pose

class CartesianController(Node):
    def __init__(self):
        super().__init__('cartesian_controller')
        
        # Initialize MoveIt components
        self.arm_group = MoveGroupCommander("arm")
        self.gripper_group = MoveGroupCommander("gripper")
        
        # Set planning parameters
        self.arm_group.set_planning_time(10.0)
        self.arm_group.set_num_planning_attempts(5)
        
    def move_to_xyz(self, x, y, z, roll=0, pitch=0, yaw=0):
        """Move end-effector to XYZ coordinates"""
        pose_goal = Pose()
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        
        # Convert roll/pitch/yaw to quaternion
        from tf_transformations import quaternion_from_euler
        q = quaternion_from_euler(roll, pitch, yaw)
        pose_goal.orientation.x = q[0]
        pose_goal.orientation.y = q[1] 
        pose_goal.orientation.z = q[2]
        pose_goal.orientation.w = q[3]
        
        self.arm_group.set_pose_target(pose_goal)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        
        return success
        
    def open_gripper(self):
        """Open gripper"""
        self.gripper_group.set_named_target("open")
        return self.gripper_group.go(wait=True)
        
    def close_gripper(self):
        """Close gripper"""
        self.gripper_group.set_named_target("close") 
        return self.gripper_group.go(wait=True)

# Tic-Tac-Toe specific positions (converted from your joint values)
class TicTacToeController(CartesianController):
    def __init__(self):
        super().__init__()
        
        # Convert your joint positions to XYZ coordinates
        self.white_ball_pos = self.joint_to_xyz(-0.35, 0.64, -0.08, 0.71, -1.55)
        self.board_center = self.joint_to_xyz(0.10, 0.13, 0.88, 0.11, 0.0)
        
    def joint_to_xyz(self, j1, j2, j3, j4, j5):
        """Convert joint angles to XYZ using forward kinematics"""
        # Use MoveIt's forward kinematics service
        # This requires your robot's kinematic model
        pass
        
    def pickup_white_ball(self):
        """Execute white ball pickup sequence"""
        # Open gripper
        self.open_gripper()
        
        # Move to white ball position
        success = self.move_to_xyz(*self.white_ball_pos)
        if not success:
            return False
            
        # Close gripper to grab ball
        self.close_gripper()
        return True
        
    def place_on_board(self):
        """Place ball on tic-tac-toe board center"""
        # Move to board center
        success = self.move_to_xyz(*self.board_center)
        if not success:
            return False
            
        # Open gripper to release ball
        self.open_gripper()
        return True
```

### Phase 5: Hardware Integration

#### Step 5.1: Hardware Driver Development
```bash
# Create hardware interface package
ros2 pkg create --build-type ament_cmake jetrover_hardware

# Implement ros2_control hardware interface
# This connects MoveIt2 to your actual servo motors
```

#### Step 5.2: Real Robot Configuration
Create `jetrover_description_urdf/config/jetrover_controllers.yaml`:
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
      
    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController
      
    gripper_controller:
      type: position_controllers/GripperActionController

arm_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2  
      - joint3
      - joint4
      - joint5
    interface_name: position
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
```

### Phase 6: Testing and Calibration

#### Step 6.1: Simulation Testing
```bash
# Launch Gazebo simulation with MoveIt2
ros2 launch jetrover_moveit_config demo_gazebo.launch.py

# Test cartesian control
ros2 run jetrover_cartesian_control cartesian_control.py
```

#### Step 6.2: Hardware Calibration
```python
# Create calibration script
def calibrate_positions():
    """Convert your known good joint positions to XYZ coordinates"""
    
    # Your known positions from move1.sh
    white_ball_joints = [-0.35, 0.64, -0.08, 0.71, -1.55]
    board_center_joints = [0.10, 0.13, 0.88, 0.11, 0.0]
    
    # Use MoveIt's forward kinematics to get XYZ
    white_ball_xyz = forward_kinematics(white_ball_joints)
    board_center_xyz = forward_kinematics(board_center_joints)
    
    print(f"White ball XYZ: {white_ball_xyz}")
    print(f"Board center XYZ: {board_center_xyz}")
```

### Phase 7: Advanced Features

#### Step 7.1: Trajectory Visualization
```python
# Add trajectory visualization in RViz
def visualize_trajectory(start_xyz, end_xyz):
    """Show planned trajectory in RViz"""
    # Create trajectory markers
    # Display in RViz for validation before execution
```

#### Step 7.2: Collision Avoidance
```python
# Add collision objects (table, tic-tac-toe board)
def add_collision_objects():
    planning_scene = PlanningSceneInterface()
    
    # Add table
    table_pose = Pose()
    table_pose.position.z = 0.375
    planning_scene.add_box("table", table_pose, (0.8, 0.6, 0.05))
    
    # Add tic-tac-toe board
    board_pose = Pose()
    board_pose.position.z = 0.45
    planning_scene.add_box("board", board_pose, (0.1, 0.1, 0.01))
```

## 🛠️ Implementation Timeline

### Week 1: Foundation
- [ ] Install MoveIt2 and dependencies
- [ ] Convert SDF to URDF
- [ ] Validate robot model in RViz

### Week 2: MoveIt2 Setup
- [ ] Configure MoveIt2 using Setup Assistant
- [ ] Test motion planning in simulation
- [ ] Create basic Cartesian control interface

### Week 3: Software Integration
- [ ] Develop Python/C++ control nodes
- [ ] Convert joint positions to XYZ coordinates
- [ ] Implement tic-tac-toe specific functions

### Week 4: Hardware Integration
- [ ] Develop hardware interface
- [ ] Connect to real servo motors
- [ ] Calibrate robot workspace

### Week 5: Testing & Refinement
- [ ] Test full pick-and-place sequence
- [ ] Add collision avoidance
- [ ] Fine-tune trajectory planning

## 📚 Learning Resources

### Essential Documentation
- [MoveIt2 Tutorials](https://moveit.picknik.ai/main/doc/tutorials/tutorials.html)
- [ROS2 Control](https://control.ros.org/master/index.html)
- [URDF Tutorials](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/URDF-Main.html)

### Key Concepts to Master
1. **Forward/Inverse Kinematics**: Converting between joint angles and XYZ positions
2. **Trajectory Planning**: Creating smooth, collision-free paths
3. **ROS2 Control**: Hardware abstraction layer
4. **Transform Trees**: Coordinate frame management

## 🎯 Success Metrics
- [ ] Robot moves smoothly to white ball position using XYZ coordinates
- [ ] Successful ball pickup and placement on tic-tac-toe board
- [ ] Trajectory visualization working in RViz
- [ ] Collision avoidance preventing table/obstacle collisions
- [ ] Real-world robot matches simulation behavior

## 🔧 Troubleshooting Guide

### Common Issues
1. **URDF Loading Errors**: Check mesh file paths and scaling
2. **Planning Failures**: Increase planning time, check joint limits
3. **Hardware Connection**: Verify serial/USB connections and permissions
4. **Calibration Issues**: Use joint_state_publisher_gui for manual verification

This roadmap provides a structured approach to transition your successful Gazebo simulation to real-world robot control using industry-standard tools and practices.
