# README4: Complete ngrok Setup for ROS2 Robot Arm Control

## Overview
This guide shows how to use ngrok to expose your WSL2/Linux ROS2 bridge to your cloud backend, enabling web-based robot arm control for tic-tac-toe gameplay.

---

## Prerequisites
- ngrok installed (✅ already done)
- ROS2 bridge running in WSL2/Linux
- Cloud backend (Render, Vercel, etc.)

---

## Step-by-Step Setup

### 1. Sign Up for ngrok Account
1. Go to https://dashboard.ngrok.com/signup
2. Create a free account
3. Go to https://dashboard.ngrok.com/get-started/your-authtoken
4. Copy your authtoken - 328IDXIqUzvTtchS3i9YNVdBa7C_2hEG7KqPyiegR2uqxhdqk

### 2. Authenticate ngrok in WSL2/Linux
```bash
ngrok config add-authtoken YOUR_AUTHTOKEN_HERE
```
Replace `YOUR_AUTHTOKEN_HERE` with your actual token.

### 3. Start Your Services (4 Terminals Required)

**Terminal 1: Start Gazebo**
```bash
cd ~/ros2_ws/src/jetrover_description/models/jetrover
source ~/ros2_ws/install/setup.bash
gz sim tictactoe_NEW3.6.sdf
```

**Terminal 2: Start ROS2-Gazebo Bridge**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=/home/sukritchopra/ros2_ws/src/jetrover_ros2_control/bridge_config.yaml
```

**Terminal 3: Start ROS2 Python Bridge**
```bash
cd ~/ros2_ws/src/jetrover_ros2_NODEJS
python3 wsl2_ros_bridge.py
```
- Wait for "✅ ROS2 node initialized" and "🌐 Starting Flask web server..."

**Terminal 4: Start ngrok Tunnel**
```bash
ngrok http 8080
```

### 4. Get Your ngrok URL
From Terminal 4, copy the HTTPS URL from the "Forwarding" line:
```
Forwarding    https://abc123-def456.ngrok-free.app -> http://localhost:8080
```

### 5. Test the Tunnel
```bash
curl https://abc123-def456.ngrok-free.app/health
```
You should get a JSON response like:
```json
{"status": "healthy", "ros2_initialized": true, "server": "WSL2 ROS2 Bridge"}
```

### 6. Update Your Cloud Backend
**Option A: Set Environment Variable on Cloud Platform**
- On Render/Vercel dashboard, set:
```
WSL2_TUNNEL_URL=https://abc123-def456.ngrok-free.app
```

**Option B: Update server.js Default**
- Change the default URL in your deployed `server.js`:
```js
const WSL2_TUNNEL_URL = process.env.WSL2_TUNNEL_URL || 'https://abc123-def456.ngrok-free.app';
```

### 7. Test Complete System
1. Visit your website (e.g., https://ros2-joint-controller-api.onrender.com)
2. Try moving joints or executing poses
3. Watch your robot arm move in Gazebo!

### 8. Play Tic-Tac-Toe with Robot Arm
```bash
cd ~/ros2_ws/src/jetrover_ros2_NODEJS/ros2-joint-controller-api
python3 tictactoe_logic.py
```
- The robot arm will automatically move when the computer plays
- Arm goes to "waiting" position during your turn
- Arm executes pickup → place sequence for computer moves

---

## Robot Arm Movement Sequence

### Human Turn:
- Arm moves to **waiting position** (up and out of the way)

### Computer Turn:
1. **Open gripper** - opens fingers to prepare for pickup
2. **Pickup position** - moves to game piece location
3. **Close gripper** - grabs the game piece
4. **Near home** - intermediate safe position
5. **Target tile position** - moves to chosen tile
6. **Open gripper** - releases the game piece
7. **Waiting position** - returns to up position

### Tile Positions (1-9):
- **Tile 1:** j1=0.24, j2=0.61, j3=0.05, j4=0.30, j5=-0.80
- **Tile 2:** j1=0.05, j2=0.40, j3=0.34, j4=0.41, j5=-0.50
- **Tile 3:** j1=-0.20, j2=0.40, j3=0.34, j4=0.41, j5=-0.50
- **Tile 4:** j1=0.23, j2=0.36, j3=0.31, j4=0.16, j5=-0.26
- **Tile 5:** j1=0.02, j2=0.36, j3=0.31, j4=0.11, j5=-0.26
- **Tile 6:** j1=-0.20, j2=0.36, j3=0.31, j4=0.11, j5=-0.26
- **Tile 7:** j1=0.23, j2=0.36, j3=0.31, j4=-0.22, j5=-0.26
- **Tile 8:** j1=-0.02, j2=0.35, j3=0.31, j4=-0.26, j5=-0.26
- **Tile 9:** j1=-0.23, j2=0.35, j3=0.31, j4=-0.26, j5=-0.26

### Gripper Control:
- **Open gripper:** left_finger=0.15, right_finger=0.15
- **Close gripper:** left_finger=0.0, right_finger=0.0

### Manual Control Commands:
```bash
# Joint control
ros2 topic pub /joint1_cmd std_msgs/msg/Float64 "data: 0.0" --once
ros2 topic pub /joint2_cmd std_msgs/msg/Float64 "data: 0.78" --once
ros2 topic pub /joint3_cmd std_msgs/msg/Float64 "data: 0.17" --once
ros2 topic pub /joint4_cmd std_msgs/msg/Float64 "data: 0.76" --once
ros2 topic pub /joint5_cmd std_msgs/msg/Float64 "data: 0.04" --once

# Gripper control
ros2 topic pub /left_finger_cmd std_msgs/msg/Float64 "data: 0.15" --once
ros2 topic pub /right_finger_cmd std_msgs/msg/Float64 "data: 0.15" --once
```

---

## Troubleshooting

### ngrok Issues
- **"Please sign up":** Need to authenticate with your authtoken
- **"tunnel not found":** Check if ngrok is still running
- **URL changed:** ngrok free gives new URLs on restart

### API Connection Issues
- **Connection refused:** Ensure ROS2 bridge is running on port 8080
- **500 errors:** Check cloud backend logs for WSL2_TUNNEL_URL setting
- **Timeout:** ngrok free can be slower than direct connections

### Robot Arm Issues
- **No movement:** Check ROS2-Gazebo bridge is running
- **Wrong positions:** Verify joint names match your robot model
- **Slow response:** Allow more time between movements

---

## Important Notes

### ngrok Free Limitations
- **8-hour sessions:** Tunnel expires after 8 hours
- **Random URLs:** New URL each time you restart ngrok
- **1 tunnel limit:** Only one active tunnel on free plan

### Production Tips
- **ngrok Pro:** Get static domains and longer sessions
- **Monitor usage:** Check ngrok dashboard for stats
- **Add delays:** Allow time for robot movements between commands

---

## Environment Variables
Set these for production:
```bash
# On your cloud platform
WSL2_TUNNEL_URL=https://your-ngrok-url.ngrok-free.app

# In WSL2/Linux (optional)
ROBOT_API_URL=https://ros2-joint-controller-api.onrender.com
```

---

## Quick Commands

**Start everything:**
```bash
# Terminal 1: Gazebo
gz sim tictactoe_NEW3.6.sdf

# Terminal 2: ROS2 Bridge  
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=/path/to/bridge_config.yaml

# Terminal 3: Python Bridge
python3 wsl2_ros_bridge.py

# Terminal 4: ngrok
ngrok http 8080
```

**Test connection:**
```bash
curl https://your-ngrok-url.ngrok-free.app/health
```

**Play tic-tac-toe:**
```bash
python3 tictactoe_logic.py
```

---

## Success Indicators
- ✅ ngrok shows "Session Status: online"
- ✅ curl to ngrok URL returns JSON health status
- ✅ Cloud backend logs show successful WSL2 connection
- ✅ Robot arm moves in Gazebo when computer plays tic-tac-toe
- ✅ Arm returns to waiting position for human turns

---

Your system is now ready for web-based robot arm control! 🤖🎮
