# JetRover Description Package - Setup and Visualization Guide

This guide will help you set up and visualize the JetRover robot model in RViz using ROS2 Jazzy on Ubuntu 24.04.

## Prerequisites

- Ubuntu 24.04
- ROS2 Jazzy Jalisco installed
- Basic ROS2 workspace setup

## Installation and Setup

### 1. Install Required Dependencies

```bash
# Update package list
sudo apt update

# Install ROS2 visualization tools
sudo apt install ros-jazzy-rviz2 ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher-gui

# Install xacro for processing URDF files
sudo apt install ros-jazzy-xacro
```

### 2. Set Up Your Workspace

Navigate to your ROS2 workspace:

```bash
cd ~/ros2_ws
```

### 3. Build the Package

```bash
# Build the jetrover_description package
colcon build --packages-select jetrover_description

# Source the workspace
source install/setup.bash
```

### 4. Fix the Launch File (if needed)

The launch file needs to be corrected for proper path handling. Update your `display.launch.py`:

```python
# filepath: /home/your_username/ros2_ws/src/jetrover_description/launch/display.launch.py
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    jetrover_description_package_path = get_package_share_directory('jetrover_description')
    
    # Define paths
    urdf_path = os.path.join(jetrover_description_package_path, 'urdf/jetrover.xacro')
    rviz_config_file = os.path.join(jetrover_description_package_path, 'rviz/view.rviz')

    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    frame_prefix = LaunchConfiguration('frame_prefix', default='')

    # Robot description
    robot_description = Command(['xacro ', urdf_path, 
                               ' LIDAR_TYPE:=ouster', 
                               ' MACHINE_TYPE:=jetrover'])

    # Nodes
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
            'frame_prefix': frame_prefix
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('frame_prefix', default_value=''),
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
    ])
```

### 5. Rebuild After Changes

```bash
# Rebuild the package
colcon build --packages-select jetrover_description

# Source the workspace again
source install/setup.bash
```

## Running the Visualization

### Method 1: Using Launch File (Recommended)

```bash
# Navigate to workspace
cd ~/ros2_ws

# Source the workspace
source install/setup.bash

# Set required environment variables
export LIDAR_TYPE=A1
export MACHINE_TYPE=JetRover_Mecanum
export need_compile=True

# Launch the visualization
ros2 launch jetrover_description display.launch.py
```

### Method 2: Manual Launch (Alternative)

If the launch file doesn't work, you can run the components manually:

```bash
# Terminal 1: Source workspace and start robot state publisher
cd ~/ros2_ws
source install/setup.bash
export LIDAR_TYPE=ouster
export MACHINE_TYPE=jetrover
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro /home/your_username/ros2_ws/src/jetrover_description/urdf/jetrover.xacro LIDAR_TYPE:=ouster MACHINE_TYPE:=jetrover)"

# Terminal 2: Start joint state publisher GUI
cd ~/ros2_ws
source install/setup.bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui

# Terminal 3: Start RViz
cd ~/ros2_ws
source install/setup.bash
rviz2
```

### Method 3: Viewing Just the Arm

To visualize only the arm component:

```bash
cd ~/ros2_ws
source install/setup.bash

# View the arm URDF directly
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro /home/your_username/ros2_ws/src/jetrover_description/urdf/arm.urdf.xacro)"

# In another terminal
ros2 run joint_state_publisher_gui joint_state_publisher_gui

# In another terminal
rviz2
```

## Configuring RViz

Once RViz opens:

1. **Add Robot Model Display:**
   - Click "Add" button
   - Select "RobotModel"
   - Set "Robot Description" topic to `/robot_description`

2. **Add TF Display:**
   - Click "Add" button
   - Select "TF"
   - This shows coordinate frames

3. **Set Fixed Frame:**
   - In "Global Options", set "Fixed Frame" to your base link (e.g., `base_link` or `link1`)

4. **Use Joint State Publisher:**
   - The GUI window allows you to move joints interactively
   - Slide the joint controls to see the robot move

## Troubleshooting

### Common Issues:

1. **Package not found:**
   ```bash
   # Make sure workspace is sourced
   source install/setup.bash
   ```

2. **Xacro file not found:**
   ```bash
   # Check if the file exists
   ls src/jetrover_description/urdf/
   ```

3. **Missing dependencies:**
   ```bash
   # Install missing packages
   sudo apt install ros-jazzy-joint-state-publisher-gui ros-jazzy-xacro
   ```

4. **Environment variables not set:**
   ```bash
   export LIDAR_TYPE=A1
   export MACHINE_TYPE=JetRover_Mecanum
   ```

### Verification Commands:

```bash
# Check if package is built correctly
ros2 pkg list | grep jetrover

# Check available launch files
ros2 pkg executables jetrover_description

# Test xacro conversion manually
xacro src/jetrover_description/urdf/jetrover.xacro LIDAR_TYPE:=ouster MACHINE_TYPE:=JetRover_Mecanum
```

## Package Structure

```
jetrover_description/
├── urdf/
│   ├── jetrover.xacro          # Main robot description
│   ├── arm.urdf.xacro          # Arm component
│   └── ...                     # Other URDF files
├── launch/
│   └── display.launch.py       # Launch file for visualization
├── rviz/
│   └── view.rviz              # RViz configuration
├── meshes/                     # 3D mesh files
└── package.xml                 # Package configuration
```

## Environment Variables

The jetrover.xacro file expects these environment variables:
- `LIDAR_TYPE`: Set to `A1` (default)
- `MACHINE_TYPE`: Set to `JetRover_Mecanum` (default)

These can be set in your `~/.bashrc` for persistence:

```bash
echo "export LIDAR_TYPE=A1" >> ~/.bashrc
echo "export MACHINE_TYPE=JetRover_Mecanum" >> ~/.bashrc
source ~/.bashrc
```

## Gazebo Setup

This section guides you through setting up and simulating the JetRover in Gazebo Harmonic.

### 1. Install Gazebo Harmonic and Required Dependencies

```bash
# Install Gazebo Harmonic
sudo apt update
sudo apt install ros-jazzy-gz-sim ros-jazzy-gz-gazebo

# Install Gazebo ROS packages for joint and arm control
sudo apt install ros-jazzy-ros-gz-control ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-interfaces

# Install joint controllers and related plugins
sudo apt install ros-jazzy-joint-state-controller ros-jazzy-joint-trajectory-controller ros-jazzy-effort-controllers ros-jazzy-position-controllers

# (Optional) Install additional Gazebo plugins for sensors and arms
sudo apt install ros-jazzy-ros-gz-sensors ros-jazzy-ros-gz-actuators
```

### 2. Launching JetRover in Gazebo

Navigate to the JetRover Gazebo model directory:

```bash
cd ~/ros2_ws/src/jetrover_description/models/jetrover
```

#### To launch the full JetRover world (with gravity and sleazy arm, no joints):

```bash
gz sim jetrover_world.sdf
or 
cd /home/your_user_name/ros2_ws/src/jetrover_description/models/jetrover
```



#### To launch Full TicTacToe Setup:

```bash
cd ~/ros2_ws/src/jetrover_description/models/jetrover
gz WORKINGPROTO_CorrectDim1.sdf
```

> **Note:**  
> - Make sure all dependencies are installed before launching Gazebo.  
> - For joint and arm control, ensure your URDF/SDF files include the appropriate controllers and plugins.


```