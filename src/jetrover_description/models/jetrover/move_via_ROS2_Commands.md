# Moving 6-DOF Arm with ROS2 Commands

## 🎯 Overview
This guide shows how to transition from shell script control (`move1.sh`) to proper ROS2 command-based control for your 6-DOF robot arm in both Gazebo simulation and real-world hardware.

## 📋 Current vs Target Architecture

### Current Setup (Shell Scripts)
```bash
# Your current approach
./move1.sh white  # Uses gz topic commands directly
gz topic -t "/model/jetrover/joint/joint1/0/cmd_pos" -m gz.msgs.Double -p "data: -0.59"
```

### Target Setup (ROS2 Commands)
```bash
# Professional ROS2 approach
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory [...]
ros2 service call /move_group/plan_kinematic_path moveit_msgs/srv/GetMotionPlan [...]
```

---

## 🚀 Method 1: Direct ROS2 Topic Publishing (Beginner)

### Step 1.1: Install ROS2-Gazebo Bridge
```bash
# Install Gazebo-ROS2 bridge packages
sudo apt install ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim

# Create bridge configuration
mkdir -p ~/ros2_ws/src/jetrover_ros2_control/config
```

### Step 1.2: Create Topic Bridge Configuration
Create `~/ros2_ws/src/jetrover_ros2_control/config/bridge_config.yaml`:
```yaml
# Bridge Gazebo topics to ROS2
- ros_topic_name: "/joint_states"
  gz_topic_name: "/world/WORKINGPROTO/model/jetrover/joint_state"
  ros_type_name: "sensor_msgs/msg/JointState"
  gz_type_name: "gz.msgs.Model"

- ros_topic_name: "/joint1_cmd"
  gz_topic_name: "/model/jetrover/joint/joint1/0/cmd_pos"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "gz.msgs.Double"

- ros_topic_name: "/joint2_cmd"
  gz_topic_name: "/model/jetrover/joint/joint2/0/cmd_pos"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "gz.msgs.Double"

- ros_topic_name: "/joint3_cmd"
  gz_topic_name: "/model/jetrover/joint/joint3/0/cmd_pos"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "gz.msgs.Double"

- ros_topic_name: "/joint4_cmd"
  gz_topic_name: "/model/jetrover/joint/joint4/0/cmd_pos"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "gz.msgs.Double"

- ros_topic_name: "/joint5_cmd"
  gz_topic_name: "/model/jetrover/joint/joint5/0/cmd_pos"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "gz.msgs.Double"

- ros_topic_name: "/left_finger_cmd"
  gz_topic_name: "/model/jetrover/joint/left_finger_joint/0/cmd_pos"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "gz.msgs.Double"

- ros_topic_name: "/right_finger_cmd"
  gz_topic_name: "/model/jetrover/joint/right_finger_joint/0/cmd_pos"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "gz.msgs.Double"
```

### Step 1.3: Launch with ROS2 Bridge
```bash
# Terminal 1: Launch Gazebo with your SDF
gz sim tictactoe_NEW3.4.sdf

# Terminal 2: Start ROS2-Gazebo bridge
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=~/ros2_ws/src/jetrover_ros2_control/config/bridge_config.yaml

# Terminal 3: Test ROS2 commands
ros2 topic pub /joint1_cmd std_msgs/msg/Float64 "data: -0.59" --once
ros2 topic pub /joint2_cmd std_msgs/msg/Float64 "data: 0.17" --once
```

### Step 1.4: Create ROS2 Control Script
Create `~/ros2_ws/src/jetrover_ros2_control/scripts/ros2_arm_control.py`:
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time

class JetRoverController(Node):
    def __init__(self):
        super().__init__('jetrover_controller')
        
        # Create publishers for each joint
        self.joint_pubs = {
            'joint1': self.create_publisher(Float64, '/joint1_cmd', 10),
            'joint2': self.create_publisher(Float64, '/joint2_cmd', 10),
            'joint3': self.create_publisher(Float64, '/joint3_cmd', 10),
            'joint4': self.create_publisher(Float64, '/joint4_cmd', 10),
            'joint5': self.create_publisher(Float64, '/joint5_cmd', 10),
            'left_finger': self.create_publisher(Float64, '/left_finger_cmd', 10),
            'right_finger': self.create_publisher(Float64, '/right_finger_cmd', 10),
        }
        
        self.get_logger().info("JetRover Controller initialized")
        
    def move_joint(self, joint_name, position):
        """Move a single joint to specified position"""
        msg = Float64()
        msg.data = float(position)
        self.joint_pubs[joint_name].publish(msg)
        self.get_logger().info(f"Moving {joint_name} to {position}")
        
    def move_to_position(self, positions, duration=3.0):
        """Move multiple joints simultaneously"""
        for joint_name, position in positions.items():
            self.move_joint(joint_name, position)
        
        # Wait for movement to complete
        time.sleep(duration)
        
    def white_ball_position(self):
        """Move to white ball position: j1=-0.35, j2=0.64, j3=-0.08, j4=0.71, j5=-1.55"""
        positions = {
            'joint1': -0.35,
            'joint2': 0.64,
            'joint3': -0.08,
            'joint4': 0.71,
            'joint5': -1.55,
            'left_finger': 0.15,  # Open gripper
            'right_finger': 0.15
        }
        self.get_logger().info("Moving to white ball position")
        self.move_to_position(positions, 4.0)
        
    def board_center_position(self):
        """Move to board center: j1=0.10, j2=0.13, j3=0.88, j4=0.11, j5=0"""
        positions = {
            'joint1': 0.10,
            'joint2': 0.13,
            'joint3': 0.88,
            'joint4': 0.11,
            'joint5': 0.0,
            'left_finger': 0.15,  # Open gripper
            'right_finger': 0.15
        }
        self.get_logger().info("Moving to board center position")
        self.move_to_position(positions, 3.0)
        
    def close_gripper(self):
        """Close gripper to grab object"""
        positions = {
            'left_finger': 0.0,
            'right_finger': 0.0
        }
        self.get_logger().info("Closing gripper")
        self.move_to_position(positions, 1.0)
        
    def open_gripper(self):
        """Open gripper to release object"""
        positions = {
            'left_finger': 0.15,
            'right_finger': 0.15
        }
        self.get_logger().info("Opening gripper")
        self.move_to_position(positions, 1.0)
        
    def home_position(self):
        """Return to home position"""
        positions = {
            'joint1': 0.0,
            'joint2': 0.0,
            'joint3': 0.0,
            'joint4': 0.0,
            'joint5': 0.0,
            'left_finger': 0.0,
            'right_finger': 0.0
        }
        self.get_logger().info("Moving to home position")
        self.move_to_position(positions, 4.0)
        
    def execute_pickup_sequence(self):
        """Execute complete white ball pickup sequence"""
        self.get_logger().info("🎯 Starting white ball pickup sequence")
        
        # 1. Move to white ball position
        self.white_ball_position()
        
        # 2. Close gripper to grab ball
        self.close_gripper()
        
        # 3. Move to board center
        self.board_center_position()
        
        # 4. Open gripper to release ball
        self.open_gripper()
        
        # 5. Return home
        self.home_position()
        
        self.get_logger().info("✅ Pickup sequence completed!")

def main(args=None):
    rclpy.init(args=args)
    controller = JetRoverController()
    
    try:
        # Execute the pickup sequence
        controller.execute_pickup_sequence()
        
        # Keep node alive for manual control if needed
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        controller.get_logger().info("Shutting down JetRover Controller")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 1.5: Make Script Executable and Run
```bash
# Make script executable
chmod +x ~/ros2_ws/src/jetrover_ros2_control/scripts/ros2_arm_control.py

# Run the ROS2 control script
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 run jetrover_ros2_control ros2_arm_control.py
```

---

## 🎯 Method 2: ROS2 Control with Joint Trajectory Controller (Intermediate)

### Step 2.1: Add ROS2 Control to Your SDF
Modify your `tictactoe_NEW3.4.sdf` to include ROS2 control plugins:

```xml
<!-- Add this inside your jetrover model, after the existing plugins -->
<plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
  <parameters>$(find jetrover_ros2_control)/config/jetrover_controllers.yaml</parameters>
  <ros>
    <remapping>/joint_states:=/jetrover/joint_states</remapping>
  </ros>
</plugin>
```

### Step 2.2: Create ROS2 Controllers Configuration
Create `~/ros2_ws/src/jetrover_ros2_control/config/jetrover_controllers.yaml`:
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
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    allow_partial_joints_goal: false
    open_loop_control: true
    allow_integration_in_goal_trajectories: true
    
gripper_controller:
  ros__parameters:
    joint: left_finger_joint
    action_monitor_rate: 20.0
    goal_tolerance: 0.01
    max_effort: 5.0
```

### Step 2.3: Create Launch File
Create `~/ros2_ws/src/jetrover_ros2_control/launch/jetrover_gazebo.launch.py`:
```python
#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Start Gazebo with your SDF world
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gz', 'sim', '-r', 'tictactoe_NEW3.4.sdf'],
        cwd='/home/sukritchopra/ros2_ws/src/jetrover_description/models/jetrover',
        output='screen'
    )
    
    # Load and start the controllers
    load_controllers = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "arm_controller", "gripper_controller"],
        output="screen",
    )
    
    return LaunchDescription([
        start_gazebo_cmd,
        load_controllers
    ])
```

### Step 2.4: Use Joint Trajectory Commands
```bash
# Install joint trajectory controller
sudo apt install ros-jazzy-joint-trajectory-controller

# Launch your robot with controllers
ros2 launch jetrover_ros2_control jetrover_gazebo.launch.py

# Send trajectory commands
ros2 topic pub /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
joint_names: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
points:
- positions: [-0.35, 0.64, -0.08, 0.71, -1.55]
  velocities: [0.0, 0.0, 0.0, 0.0, 0.0]
  accelerations: [0.0, 0.0, 0.0, 0.0, 0.0]
  time_from_start:
    sec: 3
    nanosec: 0
" --once
```

---

## 🤖 Method 3: MoveIt2 Integration (Advanced)

### Step 3.1: Convert SDF to URDF
```bash
# Create URDF from your SDF (manual conversion needed)
# Extract joint and link information from tictactoe_NEW3.4.sdf
# Create proper URDF with kinematic chain
```

### Step 3.2: Generate MoveIt2 Config
```bash
# Install MoveIt2 Setup Assistant
sudo apt install ros-jazzy-moveit-setup-assistant

# Launch Setup Assistant
ros2 launch moveit_setup_assistant setup_assistant.launch.py

# Follow GUI to configure:
# 1. Load URDF
# 2. Generate collision matrix
# 3. Add planning groups
# 4. Configure controllers
```

### Step 3.3: MoveIt2 Python Control
Create `~/ros2_ws/src/jetrover_ros2_control/scripts/moveit2_control.py`:
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import moveit_commander
from geometry_msgs.msg import Pose
from moveit_commander.conversions import pose_to_list

class MoveIt2Controller(Node):
    def __init__(self):
        super().__init__('moveit2_controller')
        
        # Initialize MoveIt2 components
        moveit_commander.roscpp_initialize([])
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper")
        
        # Set planning parameters
        self.arm_group.set_planning_time(10.0)
        self.arm_group.set_num_planning_attempts(10)
        
    def move_to_joint_positions(self, joint_positions):
        """Move arm to specific joint positions"""
        self.arm_group.go(joint_positions, wait=True)
        self.arm_group.stop()
        
    def move_to_pose(self, x, y, z, roll=0, pitch=0, yaw=0):
        """Move end-effector to XYZ coordinates"""
        pose_goal = Pose()
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        
        # Convert RPY to quaternion
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
        
    def white_ball_sequence(self):
        """Execute white ball pickup using joint positions"""
        # Your precise joint positions
        white_ball_joints = [-0.35, 0.64, -0.08, 0.71, -1.55]
        board_center_joints = [0.10, 0.13, 0.88, 0.11, 0.0]
        home_joints = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        self.get_logger().info("🎯 MoveIt2: Moving to white ball")
        self.move_to_joint_positions(white_ball_joints)
        
        self.get_logger().info("📍 MoveIt2: Moving to board center")
        self.move_to_joint_positions(board_center_joints)
        
        self.get_logger().info("🏠 MoveIt2: Returning home")
        self.move_to_joint_positions(home_joints)
        
def main():
    rclpy.init()
    controller = MoveIt2Controller()
    
    try:
        controller.white_ball_sequence()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        moveit_commander.roscpp_shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 🔧 Real-World Hardware Integration

---

## 🤖 ESP32 + XArm 1S Integration Guide

### Overview: ESP32-XArm 1S + WSL Setup
Your XArm 1S with ESP32 board can be controlled via ROS2 commands from WSL Ubuntu. Here's the complete integration pathway:

```
[WSL Ubuntu ROS2] ↔ [USB/Serial] ↔ [ESP32 Board] ↔ [XArm 1S Servos]
```

### Step 1: WSL USB Device Access Setup

#### 1.1 Install USB/IP Support in Windows
```powershell
# Run in Windows PowerShell as Administrator
winget install --interactive --exact dorssel.usbipd-win
```

#### 1.2 Connect ESP32 and Share to WSL
```powershell
# In Windows PowerShell as Administrator
# Find your ESP32 device
usbipd list

# Share the ESP32 device (replace 1-1 with your device bus ID)
usbipd bind --busid 1-1

# Attach to WSL
usbipd attach --wsl --busid 1-1
```

#### 1.3 Verify ESP32 Connection in WSL
```bash
# In WSL terminal
lsusb | grep -i esp
ls /dev/ttyUSB* || ls /dev/ttyACM*

# Should show something like: /dev/ttyUSB0 or /dev/ttyACM0
```

### Step 2: ESP32 Firmware Development

#### 2.1 Create ESP32 ROS2 Bridge Firmware
Create `~/ros2_ws/src/xarm_esp32_bridge/firmware/xarm_ros2_bridge.ino`:
```cpp
#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float64.h>
#include <ESP32Servo.h>

// Servo objects for 6-DOF arm
Servo joint1, joint2, joint3, joint4, joint5, gripper;

// Servo pins (adjust for your XArm 1S wiring)
const int SERVO_PINS[] = {2, 4, 5, 18, 19, 21}; // GPIO pins
const int NUM_JOINTS = 6;

// ROS2 variables
rcl_subscription_t joint_subscribers[NUM_JOINTS];
std_msgs__msg__Float64 joint_msgs[NUM_JOINTS];
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Joint position storage (in radians)
float joint_positions[NUM_JOINTS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

// Servo angle limits (adjust for your XArm 1S)
const float SERVO_MIN_ANGLE[] = {0, 0, 0, 0, 0, 0};      // degrees
const float SERVO_MAX_ANGLE[] = {180, 180, 180, 180, 180, 180}; // degrees

void setup() {
  Serial.begin(115200);
  
  // Initialize servos
  joint1.attach(SERVO_PINS[0]);
  joint2.attach(SERVO_PINS[1]);
  joint3.attach(SERVO_PINS[2]);
  joint4.attach(SERVO_PINS[3]);
  joint5.attach(SERVO_PINS[4]);
  gripper.attach(SERVO_PINS[5]);
  
  // Move to home position
  moveToHomePosition();
  
  // Setup micro-ROS
  set_microros_transports();
  
  allocator = rcl_get_default_allocator();
  
  // Initialize support
  rclc_support_init(&support, 0, NULL, &allocator);
  
  // Create node
  rclc_node_init_default(&node, "xarm_esp32_controller", "", &support);
  
  // Create joint subscribers
  const char* joint_topics[] = {
    "/xarm/joint1_cmd",
    "/xarm/joint2_cmd", 
    "/xarm/joint3_cmd",
    "/xarm/joint4_cmd",
    "/xarm/joint5_cmd",
    "/xarm/gripper_cmd"
  };
  
  for(int i = 0; i < NUM_JOINTS; i++) {
    rclc_subscription_init_default(
      &joint_subscribers[i],
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
      joint_topics[i]
    );
  }
  
  // Create executor
  rclc_executor_init(&executor, &support.context, NUM_JOINTS, &allocator);
  
  // Add subscriptions to executor
  for(int i = 0; i < NUM_JOINTS; i++) {
    rclc_executor_add_subscription(&executor, &joint_subscribers[i], &joint_msgs[i], 
                                   &joint_callback, ON_NEW_DATA);
  }
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  delay(10);
}

void joint_callback(const void * msgin) {
  // Determine which joint this callback is for
  for(int i = 0; i < NUM_JOINTS; i++) {
    if(msgin == &joint_msgs[i]) {
      float position_rad = joint_msgs[i].data;
      moveJoint(i, position_rad);
      break;
    }
  }
}

void moveJoint(int joint_index, float position_radians) {
  // Convert radians to servo degrees
  float degrees = position_radians * 180.0 / PI;
  
  // Apply servo limits
  degrees = constrain(degrees, SERVO_MIN_ANGLE[joint_index], SERVO_MAX_ANGLE[joint_index]);
  
  // Move the appropriate servo
  switch(joint_index) {
    case 0: joint1.write(degrees); break;
    case 1: joint2.write(degrees); break;
    case 2: joint3.write(degrees); break;
    case 3: joint4.write(degrees); break;
    case 4: joint5.write(degrees); break;
    case 5: gripper.write(degrees); break;
  }
  
  joint_positions[joint_index] = position_radians;
  
  Serial.print("Joint ");
  Serial.print(joint_index);
  Serial.print(" moved to ");
  Serial.print(degrees);
  Serial.println(" degrees");
}

void moveToHomePosition() {
  // Move all joints to center position
  joint1.write(90);
  joint2.write(90);
  joint3.write(90);
  joint4.write(90);
  joint5.write(90);
  gripper.write(0); // Gripper closed
  delay(2000);
}
```

#### 2.2 Install Required Libraries
In Arduino IDE:
1. Install `micro_ros_arduino` library
2. Install `ESP32Servo` library
3. Select ESP32 board type

#### 2.3 Upload Firmware to ESP32
```bash
# Flash the firmware using Arduino IDE or platformio
# Make sure ESP32 is connected and recognized in WSL
```

### Step 3: ROS2 Hardware Interface Package

#### 3.1 Create XArm Hardware Controller Package
```bash
# Create new ROS2 package
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python xarm_esp32_control --dependencies rclpy std_msgs sensor_msgs

# Install micro-ROS agent
sudo apt install ros-jazzy-micro-ros-agent
```

#### 3.2 Create Hardware Interface Node
Create `~/ros2_ws/src/xarm_esp32_control/xarm_esp32_control/xarm_hardware_interface.py`:
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import threading
import time

class XArmHardwareInterface(Node):
    def __init__(self):
        super().__init__('xarm_hardware_interface')
        
        # Joint names matching your simulation
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'gripper']
        
        # Publishers to ESP32
        self.joint_cmd_pubs = {}
        for i, joint in enumerate(self.joint_names):
            topic = f'/xarm/{joint}_cmd'
            self.joint_cmd_pubs[joint] = self.create_publisher(Float64, topic, 10)
        
        # Subscribers for ROS2 commands (matching simulation topics)
        self.create_subscription(Float64, '/joint1_cmd', lambda msg: self.forward_command('joint1', msg), 10)
        self.create_subscription(Float64, '/joint2_cmd', lambda msg: self.forward_command('joint2', msg), 10)
        self.create_subscription(Float64, '/joint3_cmd', lambda msg: self.forward_command('joint3', msg), 10)
        self.create_subscription(Float64, '/joint4_cmd', lambda msg: self.forward_command('joint4', msg), 10)
        self.create_subscription(Float64, '/joint5_cmd', lambda msg: self.forward_command('joint5', msg), 10)
        self.create_subscription(Float64, '/gripper_cmd', lambda msg: self.forward_command('gripper', msg), 10)
        
        # Joint state publisher (feedback)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Current joint positions
        self.current_positions = [0.0] * len(self.joint_names)
        
        # Start joint state publishing thread
        self.state_thread = threading.Thread(target=self.publish_joint_states)
        self.state_thread.daemon = True
        self.state_thread.start()
        
        self.get_logger().info("XArm Hardware Interface started - Ready to control real robot!")
        
    def forward_command(self, joint_name, msg):
        """Forward ROS2 command to ESP32"""
        if joint_name in self.joint_cmd_pubs:
            self.joint_cmd_pubs[joint_name].publish(msg)
            
            # Update current position (in real system, this would come from encoders)
            joint_idx = self.joint_names.index(joint_name)
            self.current_positions[joint_idx] = msg.data
            
            self.get_logger().info(f"Forwarded command to {joint_name}: {msg.data:.3f} rad")
            
    def publish_joint_states(self):
        """Publish current joint states"""
        while rclpy.ok():
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = self.joint_names
            joint_state.position = self.current_positions
            joint_state.velocity = [0.0] * len(self.joint_names)
            joint_state.effort = [0.0] * len(self.joint_names)
            
            self.joint_state_pub.publish(joint_state)
            time.sleep(0.1)  # 10 Hz

def main(args=None):
    rclpy.init(args=args)
    interface = XArmHardwareInterface()
    
    try:
        rclpy.spin(interface)
    except KeyboardInterrupt:
        interface.get_logger().info("Shutting down XArm Hardware Interface")
    finally:
        interface.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 4: Launch Configuration for Real Hardware

#### 4.1 Create Hardware Launch File
Create `~/ros2_ws/src/xarm_esp32_control/launch/xarm_real_hardware.launch.py`:
```python
#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    
    # Start micro-ROS agent (connects to ESP32)
    micro_ros_agent = ExecuteProcess(
        cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev/ttyUSB0'],
        output='screen'
    )
    
    # Start hardware interface node
    hardware_interface = Node(
        package='xarm_esp32_control',
        executable='xarm_hardware_interface',
        name='xarm_hardware_interface',
        output='screen'
    )
    
    return LaunchDescription([
        micro_ros_agent,
        hardware_interface
    ])
```

#### 4.2 Setup Package Configuration
Create `~/ros2_ws/src/xarm_esp32_control/setup.py`:
```python
from setuptools import setup
import os
from glob import glob

package_name = 'xarm_esp32_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='XArm ESP32 Hardware Control Interface',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'xarm_hardware_interface = xarm_esp32_control.xarm_hardware_interface:main',
        ],
    },
)
```

### Step 5: Complete Integration Workflow

#### 5.1 Build and Setup
```bash
# Build your ROS2 workspace
cd ~/ros2_ws
colcon build --packages-select xarm_esp32_control
source install/setup.bash

# Give permissions to serial port
sudo chmod 666 /dev/ttyUSB0  # or /dev/ttyACM0
```

#### 5.2 Launch Real Hardware Control
```bash
# Terminal 1: Launch hardware interface
ros2 launch xarm_esp32_control xarm_real_hardware.launch.py

# Terminal 2: Use SAME commands as simulation!
ros2 run jetrover_ros2_control ros2_arm_control.py

# Or individual joint commands:
ros2 topic pub /joint1_cmd std_msgs/msg/Float64 "data: -0.35" --once
ros2 topic pub /joint2_cmd std_msgs/msg/Float64 "data: 0.64" --once
```

#### 5.3 Monitor Joint States
```bash
# View current joint positions
ros2 topic echo /joint_states

# Monitor ESP32 communication
ros2 topic list | grep xarm
```

### Step 6: Troubleshooting WSL + ESP32

#### Common Issues & Solutions:

**1. ESP32 Not Detected in WSL:**
```bash
# Re-attach USB device
usbipd detach --busid 1-1
usbipd attach --wsl --busid 1-1
```

**2. Serial Permission Issues:**
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER
# Logout and login again
```

**3. micro-ROS Connection Issues:**
```bash
# Check if micro-ROS agent is running
ps aux | grep micro_ros_agent

# Restart with different baud rate
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

### Step 7: Advanced Features

#### 7.1 Add Joint Position Feedback
Modify ESP32 code to read servo positions and publish joint states:
```cpp
// Add to ESP32 firmware
rcl_publisher_t joint_state_publisher;
sensor_msgs__msg__JointState joint_state_msg;

// Publish real joint positions back to ROS2
void publishJointStates() {
  // Read actual servo positions and publish
  joint_state_msg.header.stamp.sec = millis() / 1000;
  // ... populate joint positions
  rcl_publish(&joint_state_publisher, &joint_state_msg, NULL);
}
```

#### 7.2 Add Safety Limits
```python
# Add to hardware interface
def apply_joint_limits(self, joint_name, position):
    """Apply safety limits to joint commands"""
    limits = {
        'joint1': (-3.14, 3.14),
        'joint2': (-1.57, 1.57),
        # ... other joints
    }
    
    if joint_name in limits:
        min_pos, max_pos = limits[joint_name]
        return max(min_pos, min(max_pos, position))
    return position
```

---

## 🎯 **Your Complete Workflow:**

### Simulation Testing:
```bash
gz sim tictactoe_NEW3.4.sdf  # Test in simulation first
```

### Hardware Control:
```bash
ros2 launch xarm_esp32_control xarm_real_hardware.launch.py  # Control real robot
```

### Same Commands Work for Both!
```bash
ros2 run jetrover_ros2_control ros2_arm_control.py  # Your pickup sequence
```

This setup gives you seamless transition from Gazebo simulation to real XArm 1S hardware control using identical ROS2 commands! 🤖✨

### Real-World Launch Configuration
```bash
# For real hardware, modify your launch file
# Replace Gazebo launch with hardware interface

# Launch robot with real hardware drivers
ros2 launch jetrover_ros2_control jetrover_real.launch.py

# Same ROS2 commands work for both simulation and real hardware!
ros2 topic pub /arm_controller/joint_trajectory [...]
```

---

## 🎮 Command Comparison Summary

### Shell Script (Old Method)
```bash
# Manual gz topic commands
gz topic -t "/model/jetrover/joint/joint1/0/cmd_pos" -m gz.msgs.Double -p "data: -0.59"
```

### ROS2 Direct Topics (Method 1)
```bash
# ROS2 topic publishing
ros2 topic pub /joint1_cmd std_msgs/msg/Float64 "data: -0.59" --once
```

### ROS2 Trajectory Control (Method 2)
```bash
# Joint trajectory controller
ros2 topic pub /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory [...]
```

### MoveIt2 Commands (Method 3)
```bash
# High-level motion planning
ros2 run jetrover_ros2_control moveit2_control.py
```

---

## 🚀 Quick Start Commands

### For Gazebo Simulation
```bash
# Terminal 1: Launch Gazebo
gz sim tictactoe_NEW3.4.sdf

# Terminal 2: Start ROS2 bridge
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=bridge_config.yaml

# Terminal 3: Control robot
ros2 run jetrover_ros2_control ros2_arm_control.py
```

### For Real Hardware
```bash
# Terminal 1: Launch hardware drivers
ros2 launch jetrover_ros2_control jetrover_real.launch.py

# Terminal 2: Control robot (same commands!)
ros2 run jetrover_ros2_control ros2_arm_control.py
```

---

## ✅ Benefits of ROS2 Control

1. **Standardization**: Industry-standard robot control
2. **Flexibility**: Same code works in simulation and real hardware
3. **Integration**: Easy to integrate with other ROS2 packages
4. **Debugging**: Better logging, visualization, and debugging tools
5. **Scalability**: Easy to add new features like collision avoidance
6. **Community**: Large ecosystem of compatible tools and libraries

This approach transforms your shell script control into professional-grade robot control system! 🤖✨
