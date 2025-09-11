# 6DOF Robot Arm TicTacToe Project - Complete Setup Guide

This comprehensive guide covers the complete 6DOF robot arm TicTacToe project, including Gazebo simulation, ROS2 control, web interface, and motion planning with MoveIt2.

## ⚠️ **IMPORTANT: Path Configuration Warning**

**🚨 BEFORE STARTING: You MUST update all file paths in the project to match your Linux username!**

The project contains hardcoded paths with username `sukritchopra`. You need to replace these with your actual username in:

### 📝 Files to Update:
- **SDF Files**: `~/ros2_ws/src/jetrover_description/models/jetrover/*.sdf`
- **Python Scripts**: `~/ros2_ws/src/jetrover_ros2_NODEJS/ros2-joint-controller-api/*.py`
- **Launch Files**: `~/ros2_ws/src/jetrover_description/launch/*.py`
- **Shell Scripts**: Any `.sh` files in the project

### 🔍 Paths to Find & Replace:
```bash
# Find all files with hardcoded paths
cd ~/ros2_ws
grep -r "/home/sukritchopra" . --include="*.sdf" --include="*.py" --include="*.sh" --include="*.launch.py"

# Replace with your username (example: replace 'sukritchopra' with 'yourusername')
find . -type f \( -name "*.sdf" -o -name "*.py" -o -name "*.sh" -o -name "*.launch.py" \) -exec sed -i 's/sukritchopra/yourusername/g' {} +
```

### 🛠️ Quick Fix Command:
```bash
# Replace 'YOUR_USERNAME' with your actual Linux username
cd ~/ros2_ws
find . -type f \( -name "*.sdf" -o -name "*.py" -o -name "*.sh" -o -name "*.launch.py" \) -exec sed -i 's/sukritchopra/YOUR_USERNAME/g' {} +
```

**⚡ Alternative**: Use environment variables in paths like `$HOME` instead of hardcoded `/home/username/`

---

## 🎯 Project Overview

This project creates a complete robotic TicTacToe game with:
- **6DOF Robot Arm**: 5 joints + gripper for precise manipulation
- **Gazebo Simulation**: Complete 3D physics simulation environment
- **Web Interface**: Remote control via web browser
- **Motion Planning**: MoveIt2 integration for collision-free path planning
- **Real-time Control**: ROS2 bridges connecting simulation to web commands

**🌐 Live Demo**: [https://ros2-joint-controller-api.onrender.com/](https://ros2-joint-controller-api.onrender.com/)

## 📋 Prerequisites

- Ubuntu 24.04 (WSL or native)
- ROS2 Jazzy Jalisco installed
- Node.js and npm
- Python 3.12+
- Internet connection for ngrok tunneling

## 🚀 Quick Start (For Experienced Users)

```bash
# 1. Build the workspace
cd ~/ros2_ws && colcon build && source install/setup.bash

# 2. Launch Gazebo with TicTacToe world
cd ~/ros2_ws/src/jetrover_description/models/jetrover
gz sim tictactoe_NEW3.7.sdf

# 3. Start ROS2 bridges (new terminal)
cd ~/ros2_ws && source install/setup.bash
python3 src/jetrover_ros2_NODEJS/ros2-joint-controller-api/gazebo_bridge.py

# 4. Start web server (new terminal)  
cd ~/ros2_ws/src/jetrover_ros2_NODEJS/ros2-joint-controller-api
npm start

# 5. Expose to internet (new terminal)
ngrok http 3000
```

## 📁 Repository Structure & Mesh Files

### 🎨 Mesh Files Location
All STL mesh files for the robot arm are located in:
```
~/ros2_ws/src/jetrover_description/meshes/
├── arm/                    # Robot arm components
│   ├── link1.STL          # Base link mesh
│   ├── link2.STL          # Shoulder link mesh  
│   ├── link3.STL          # Elbow link mesh
│   ├── link4.STL          # Wrist link mesh
│   ├── link5.STL          # End effector base mesh
│   ├── left_finger.STL    # Gripper left finger
│   └── right_finger.STL   # Gripper right finger
├── common/                 # Shared components
├── mecanum/               # Mecanum wheel meshes (if used)
└── tank/                  # Tank drive meshes (if used)
```

### 📦 Key Project Components
```
~/ros2_ws/
├── src/
│   ├── jetrover_description/          # Robot URDF/SDF models & meshes
│   │   ├── models/jetrover/           # Gazebo SDF world files
│   │   │   ├── tictactoe_NEW3.7.sdf  # Main TicTacToe world
│   │   │   └── jetrover_world.sdf    # Basic robot world
│   │   ├── meshes/                   # STL mesh files (see above)
│   │   ├── urdf/                     # URDF/XACRO robot descriptions
│   │   └── launch/                   # ROS2 launch files
│   │
│   ├── jetrover_ros2_NODEJS/         # Web interface & bridges
│   │   └── ros2-joint-controller-api/
│   │       ├── server.js             # Node.js web server
│   │       ├── gazebo_bridge.py      # ROS2-Gazebo bridge
│   │       ├── tictactoe_gtk_gui.py  # Desktop GUI version
│   │       └── public/               # Web frontend files
│   │
│   └── jetrover_overall/             # Additional components
│
└── ws_moveit2/                       # MoveIt2 workspace (optional)
    └── src/jetrover_moveit_config/   # Motion planning setup
```

## 🛠 Complete Setup From Scratch

### Step 1: Install All Dependencies

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install ROS2 Jazzy (if not already installed)
# Follow: https://docs.ros.org/en/jazzy/Installation.html

# Install ROS2 visualization and control tools
sudo apt install -y \
    ros-jazzy-rviz2 \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-xacro \
    ros-jazzy-gz-sim \
    ros-jazzy-gz-gazebo \
    ros-jazzy-ros-gz-control \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-interfaces \
    ros-jazzy-joint-state-controller \
    ros-jazzy-joint-trajectory-controller \
    ros-jazzy-effort-controllers \
    ros-jazzy-position-controllers

# Install Node.js and npm
curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
sudo apt install -y nodejs

# Install Python dependencies
pip3 install rclpy geometry_msgs sensor_msgs std_msgs flask flask-cors requests

# Install ngrok for web tunneling
wget https://bin.equinox.io/c/bNyj1mQVY4c/ngrok-v3-stable-linux-amd64.tgz
tar -xzf ngrok-v3-stable-linux-amd64.tgz
sudo mv ngrok /usr/local/bin/
```

### Step 2: Build the ROS2 Workspace

```bash
# Navigate to workspace
cd ~/ros2_ws

# Build all packages
colcon build

# Source the workspace
source install/setup.bash

# Add to bashrc for automatic sourcing
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### Step 3: Install Web Server Dependencies

```bash
# Navigate to web server directory
cd ~/ros2_ws/src/jetrover_ros2_NODEJS/ros2-joint-controller-api

# Install Node.js dependencies
npm install express cors ws

# Verify installation
npm list
```

## 🎮 Running the Complete TicTacToe System

### 🔥 Method 1: Full System Launch (Recommended)

#### Terminal 1: Launch Gazebo Simulation
```bash
cd ~/ros2_ws
source install/setup.bash

# Navigate to Gazebo models directory
cd ~/ros2_ws/src/jetrover_description/models/jetrover

# Launch the TicTacToe world with 6DOF arm
gz sim tictactoe_NEW3.7.sdf
```

#### Terminal 2: Start ROS2-Gazebo Bridge
```bash
cd ~/ros2_ws
source install/setup.bash

# Start the bridge that connects ROS2 commands to Gazebo
python3 src/jetrover_ros2_NODEJS/ros2-joint-controller-api/gazebo_bridge.py
```

#### Terminal 3: Launch Web Server
```bash
cd ~/ros2_ws/src/jetrover_ros2_NODEJS/ros2-joint-controller-api

# Start the Node.js web server
npm start
```

#### Terminal 4: Expose Web Server to Internet
```bash
# Sign up for free ngrok account at https://ngrok.com
# Get your authtoken and authenticate
ngrok config add-authtoken YOUR_AUTH_TOKEN

# Create public tunnel to your local server
ngrok http 3000
```

The ngrok terminal will show your public URL (e.g., `https://abc123.ngrok.io`) - share this with anyone to control your robot!

### 🎯 Alternative Launch Methods

#### Desktop GUI Version (Local Control)
```bash
cd ~/ros2_ws
source install/setup.bash

# Launch desktop TicTacToe GUI
python3 src/jetrover_ros2_NODEJS/ros2-joint-controller-api/tictactoe_gtk_gui.py
```

#### Basic Robot Visualization (RViz)
```bash
cd ~/ros2_ws
source install/setup.bash

# Launch robot in RViz for debugging
ros2 launch jetrover_description display.launch.py
```

#### Simple Gazebo World (No TicTacToe)
```bash
cd ~/ros2_ws/src/jetrover_description/models/jetrover

# Launch basic robot world
gz sim jetrover_world.sdf
```

## 🤖 Motion Planning with MoveIt2 (Advanced)

For collision-free motion planning, you can optionally set up MoveIt2 integration:

### Setup MoveIt2 Workspace
```bash
# Create separate MoveIt workspace
mkdir -p ~/ws_moveit2/src
cd ~/ws_moveit2

# Clone MoveIt2 dependencies (if needed)
# Build MoveIt configuration
colcon build
source install/setup.bash

# Launch integrated MoveIt + Gazebo + TicTacToe
ros2 launch jetrover_moveit_config tictactoe_world_moveit.launch.py

# In new terminal: Start MoveIt-Gazebo bridge
cd ~/ws_moveit2 && source install/setup.bash
python3 src/jetrover_moveit_config/scripts/gazebo_moveit_bridge.py
```

## 🌐 Web Interface Usage

### Accessing the Interface
1. **Local**: `http://localhost:3000`
2. **Public**: Use the ngrok URL from Terminal 4
3. **Live Demo**: [https://ros2-joint-controller-api.onrender.com/](https://ros2-joint-controller-api.onrender.com/)

### Game Controls
- **Grid Buttons**: Click any tile (1-9) to place X/O
- **Joint Sliders**: Manual control of each robot joint
- **Reset Game**: Start a new TicTacToe match
- **Manual Mode**: Direct joint control
- **Auto Mode**: AI-controlled robot movements

### Robot Joint Controls
- **Joint 1**: Base rotation (-π to π rad)
- **Joint 2**: Shoulder elevation (-π/2 to π/2 rad)  
- **Joint 3**: Elbow bend (-π to π rad)
- **Joint 4**: Wrist rotation (-π to π rad)
- **Joint 5**: End effector rotation (-π to π rad)
- **Gripper**: Open/close gripper fingers (0 to 0.02 m)

## 🔧 Technical Architecture

### System Components
1. **Gazebo Simulation**: Physics engine with 6DOF robot arm
2. **ROS2 Bridge**: `gazebo_bridge.py` - connects ROS2 topics to Gazebo joint control
3. **Web Server**: Node.js Express server with WebSocket support
4. **Frontend**: HTML/CSS/JavaScript TicTacToe interface
5. **Game Logic**: Python TicTacToe game with robot movement integration

### Communication Flow
```
Web Browser → Node.js Server → ROS2 Topics → Gazebo Bridge → Gazebo Joints
```

### ROS2 Topics Used
- `/model/jetrover/joint/joint1/0/cmd_pos` - Joint 1 position control
- `/model/jetrover/joint/joint2/0/cmd_pos` - Joint 2 position control  
- `/model/jetrover/joint/joint3/0/cmd_pos` - Joint 3 position control
- `/model/jetrover/joint/joint4/0/cmd_pos` - Joint 4 position control
- `/model/jetrover/joint/joint5/0/cmd_pos` - Joint 5 position control
- `/model/jetrover/joint/left_finger_joint/0/cmd_pos` - Left gripper finger
- `/model/jetrover/joint/right_finger_joint/0/cmd_pos` - Right gripper finger

## 🎯 Game Features

### TicTacToe Grid Layout
```
[1] [2] [3]
[4] [5] [6]
[7] [8] [9]
```

### Robot Behavior
- **Place X/O**: Robot moves to grid position and places piece
- **Game Logic**: Automatic win/draw detection
- **Reset**: Robot returns to home position
- **Manual Control**: Direct joint manipulation via sliders

## 🛠 Troubleshooting

### Common Issues & Solutions

#### 1. Gazebo Won't Start
```bash
# Check if Gazebo processes are running
ps aux | grep gz

# Kill existing processes
killall gz

# Restart Gazebo
cd ~/ros2_ws/src/jetrover_description/models/jetrover
gz sim tictactoe_NEW3.7.sdf
```

#### 2. ROS2 Bridge Not Working
```bash
# Check ROS2 installation
ros2 topic list

# Verify bridge is publishing
ros2 topic echo /model/jetrover/joint/joint1/0/cmd_pos

# Restart bridge
python3 src/jetrover_ros2_NODEJS/ros2-joint-controller-api/gazebo_bridge.py
```

#### 3. Web Server Issues
```bash
# Check if port 3000 is available
netstat -an | grep 3000

# Kill process using port 3000
sudo lsof -t -i:3000 | xargs kill -9

# Restart server
cd ~/ros2_ws/src/jetrover_ros2_NODEJS/ros2-joint-controller-api
npm start
```

#### 4. Mesh Files Not Loading
```bash
# Verify mesh files exist
ls ~/ros2_ws/src/jetrover_description/meshes/arm/

# Check SDF file paths
grep -r "meshes" ~/ros2_ws/src/jetrover_description/models/jetrover/tictactoe_NEW3.7.sdf

# Rebuild package
cd ~/ros2_ws && colcon build --packages-select jetrover_description
```

#### 5. Joint Controllers Not Responding
```bash
# Check Gazebo topics
gz topic -l | grep cmd_pos

# Test manual joint command
gz topic -t /model/jetrover/joint/joint1/0/cmd_pos -m gz.msgs.Double -p 'data: 0.5'

# Verify controller parameters in SDF file
```

### Debugging Commands

#### Check System Status
```bash
# ROS2 nodes
ros2 node list

# ROS2 topics
ros2 topic list

# Gazebo topics  
gz topic -l

# Process status
ps aux | grep -E "(gz|node|python)"
```

#### Manual Testing
```bash
# Test joint movement
gz topic -t /model/jetrover/joint/joint1/0/cmd_pos -m gz.msgs.Double -p 'data: 1.0'

# Test ROS2 publishing
ros2 topic pub /test_joint std_msgs/Float64 "data: 0.5"

# Monitor joint states
gz topic -e -t /model/jetrover/joint_state
```

## 📚 Additional Resources

### Learning Materials
- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [Gazebo Documentation](https://gazebosim.org/docs)
- [MoveIt2 Tutorials](https://moveit.picknik.ai/jazzy/index.html)

### Related Projects
- **Original Repository**: This project
- **MoveIt2 Integration**: `~/ws_moveit2/` (optional advanced setup)
- **Desktop GUI**: `tictactoe_gtk_gui.py` for local control

### Development
```bash
# Run tests
cd ~/ros2_ws && colcon test

# Build specific package
colcon build --packages-select jetrover_description

# Clean build
rm -rf build/ install/ log/ && colcon build
```

## 🏆 Project Achievements

✅ **6DOF Robot Arm Simulation** - Complete physics simulation  
✅ **Web-based Control** - Remote robot control via browser  
✅ **Real-time Communication** - WebSocket integration  
✅ **TicTacToe Game Logic** - Complete game implementation  
✅ **Joint Control System** - Precise robot joint manipulation  
✅ **Public Web Access** - ngrok tunnel for remote access  
✅ **Motion Planning** - MoveIt2 integration (optional)  
✅ **Desktop GUI** - GTK-based local interface  
✅ **Live Demo** - Deployed web version  

**🚀 Live Demo**: [https://ros2-joint-controller-api.onrender.com/](https://ros2-joint-controller-api.onrender.com/)

---

*This project demonstrates the integration of ROS2, Gazebo, web technologies, and robotics for creating an interactive robotic game system. Perfect for learning robot control, web development, and ROS2 integration.*