# ROS2 Joint Controller API (Windows + WSL2)

## Overview
This project lets you control a ROS2 robot arm (in Gazebo or real hardware) from a web API running on Windows, with ROS2 and Gazebo running in WSL2. It uses a Node.js server (Windows) and a Python ROS2 bridge (WSL2).

---

## Architecture
```
[Web Browser] → [Node.js API (Windows)] → [WSL2 ROS2 Bridge] → [ROS2 Topics] → [Gazebo/Robot]
```
- Node.js server runs on Windows (`localhost:3000`)
- Python bridge server runs in WSL2 (`WSL2_IP:8080`)
- ROS2 and Gazebo run in WSL2

---

## Step-by-Step Setup

### 1. Start ROS2 and Gazebo in WSL2
```bash
source ~/ros2_ws/install/setup.bash
gz sim ~/ros2_ws/src/jetrover_description/models/jetrover/tictactoe_NEW3.6.sdf
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=/home/sukritchopra/ros2_ws/src/jetrover_ros2_control/bridge_config.yaml
```

### 2. Start the WSL2 ROS2 Bridge Server
```bash
cd ~/ros2_ws/src/jetrover_ros2_NODEJS
python3 wsl2_ros_bridge.py
```
- Find your WSL2 IP: `hostname -I`

### 3. Start Node.js API Server on Windows
```bash
cd path\to\ros2-joint-controller-api
npm install
set WSL2_IP=<your_wsl2_ip>
npm start
``` 
- The server runs at `http://localhost:3000`

### 4. Test the API
```bash
node test_api.js
```
Or use curl/Postman:
```bash
curl -X POST http://localhost:3000/api/joint/joint2/move -H "Content-Type: application/json" -d "{\"position\": 1.5708}"
```

---

## API Endpoints
- `GET /api/health` — Check connection status
- `GET /api/joints` — List available joints
- `POST /api/joint/:jointName/move` — Move a single joint
- `POST /api/joints/move` — Move multiple joints
- `GET /api/poses` — List predefined poses
- `POST /api/pose/:poseName` — Move to a predefined pose

---

## Troubleshooting
- Make sure Windows firewall allows connections to WSL2 IP and port 8080
- Ensure WSL2 IP is correct (`hostname -I` in WSL2)
- Both servers (Node.js and Python bridge) must be running
- Use `test_api.js` for automated testing

---

## Customization
- Add more joints or poses in `server.js` and `wsl2_ros_bridge.py`
- Build a web UI to send API requests

---

## License
MIT
