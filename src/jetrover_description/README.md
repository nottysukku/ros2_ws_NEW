# JetRover Description Package - Enhanced with Joint Controllers

This package contains the robot description and Gazebo simulation files for the JetRover robot with enhanced joint control capabilities.

## Recent Updates

### Joint Parameters
All joints now have realistic effort and velocity limits, plus PID control parameters:

#### Arm Joints (joint1-joint5)
- **joint1** (base rotation): 15.0 Nm max effort, 2.0 rad/s max velocity
- **joint2** (shoulder): 12.0 Nm max effort, 1.8 rad/s max velocity  
- **joint3** (elbow): 10.0 Nm max effort, 1.5 rad/s max velocity
- **joint4** (wrist pitch): 8.0 Nm max effort, 1.2 rad/s max velocity
- **joint5** (wrist roll): 5.0 Nm max effort, 1.0 rad/s max velocity

#### Gripper Joints
- **r_joint** (master gripper joint): 3.0 Nm max effort, 0.8 rad/s max velocity
- All other gripper joints are mimic joints that follow r_joint automatically

### Joint Position Controllers
Each joint has a dedicated position controller with tuned PID gains:

#### PID Parameters by Joint
- **joint1**: P=150.0, I=5.0, D=15.0
- **joint2**: P=120.0, I=4.0, D=12.0
- **joint3**: P=100.0, I=3.0, D=10.0
- **joint4**: P=80.0, I=2.5, D=8.0
- **joint5**: P=60.0, I=2.0, D=6.0
- **gripper**: P=40.0, I=1.5, D=4.0

### Control Topics
Each joint can be controlled by publishing Float64 messages to:

- `/joint1/cmd_pos` - Joint 1 position command (radians)
- `/joint2/cmd_pos` - Joint 2 position command (radians)
- `/joint3/cmd_pos` - Joint 3 position command (radians)
- `/joint4/cmd_pos` - Joint 4 position command (radians)
- `/joint5/cmd_pos` - Joint 5 position command (radians)
- `/gripper/cmd_pos` - Gripper position command (radians)

## Usage

### Building the Package
```bash
cd ~/ros2_ws
colcon build --packages-select jetrover_description
source install/setup.bash
```

### Running the Simulation
```bash
# Launch Gazebo with the robot
ros2 launch jetrover_description simulation_with_controllers.launch.py
```

### Manual Joint Control
```bash
# Interactive manual control
ros2 run jetrover_description manual_control

# Or run the Python script directly
python3 src/jetrover_description/scripts/manual_joint_control.py
```

Available commands in manual mode:
- `j1 <angle>` - Move joint1 to angle (radians)
- `j2 <angle>` - Move joint2 to angle (radians)
- `j3 <angle>` - Move joint3 to angle (radians)
- `j4 <angle>` - Move joint4 to angle (radians)
- `j5 <angle>` - Move joint5 to angle (radians)
- `grip <angle>` - Move gripper to angle (radians)
- `open` - Open gripper fully
- `close` - Close gripper fully
- `home` - Move all joints to home position (0.0)
- `limits` - Show joint limits
- `pos` - Show current positions
- `help` - Show help message
- `quit` - Exit

### Automated Demo
```bash
# Run demonstration sequence
ros2 run jetrover_description joint_demo

# Or run the Python script directly
python3 src/jetrover_description/scripts/joint_controller_demo.py
```

The demo performs an automated sequence:
1. Move to home position
2. Open gripper
3. Move to pickup position
4. Close gripper (simulate pickup)
5. Lift object
6. Rotate to place position
7. Lower to place
8. Open gripper (release)
9. Return home

### Example Commands
```bash
# Open gripper
ros2 topic pub /gripper/cmd_pos std_msgs/msg/Float64 "data: -1.0"

# Close gripper  
ros2 topic pub /gripper/cmd_pos std_msgs/msg/Float64 "data: 1.0"

# Move joint1 to 45 degrees (π/4 radians)
ros2 topic pub /joint1/cmd_pos std_msgs/msg/Float64 "data: 0.785"

# Move joint2 to -30 degrees
ros2 topic pub /joint2/cmd_pos std_msgs/msg/Float64 "data: -0.524"
```

## Gripper Control System

The gripper uses a mimic joint system where:

- **r_joint** is the master joint that you control directly
- **l_joint**, **l_in_joint**, **r_in_joint** follow with -1 multiplier (opposite direction)
- **l_out_joint**, **r_out_joint** follow with +1 multiplier (same direction)

This creates a coordinated opening/closing motion:
- Positive r_joint values → gripper closes
- Negative r_joint values → gripper opens

## Joint Limits

All joints have safety limits defined:

### Arm Joints
- **joint1-joint5**: ±2.09 radians (±119.7°)

### Gripper Joints  
- **r_joint** (and all mimic joints): ±1.57 radians (±90°)

## Files Structure

```
jetrover_description/
├── launch/
│   ├── simulation_with_controllers.launch.py  # Launch simulation
│   └── ...
├── scripts/
│   ├── joint_controller_demo.py              # Automated demo
│   ├── manual_joint_control.py               # Manual control interface
├── models/jetrover/
│   ├── jetrover_world1.sdf                   # Updated SDF with controllers
├── meshes/                                   # Robot mesh files
├── urdf/                                     # URDF files
└── README.md                                 # This file
```

## Troubleshooting

### Joints not responding
1. Check if Gazebo is running and robot is loaded
2. Verify topics exist: `ros2 topic list | grep cmd_pos`
3. Check joint state: `ros2 topic echo /joint_states`

### Gripper not moving properly
1. Only control r_joint - other gripper joints are mimic joints
2. Use negative values to open, positive to close
3. Check joint limits: -1.57 to +1.57 radians

### Performance issues
1. Reduce PID gains if joints are oscillating
2. Check effort limits match your robot's capabilities
3. Adjust damping and friction values in SDF for smoother motion

## Technical Details

The SDF file includes:
- Realistic joint dynamics with damping and friction
- Joint position controllers with tuned PID parameters
- Joint state publisher for monitoring
- Mimic joint system for coordinated gripper control
- Physics constraints and collision detection
