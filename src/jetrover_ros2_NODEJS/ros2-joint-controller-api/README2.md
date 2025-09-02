# Platform-Agnostic ROS2 Joint Control: API Integration Guide

## Goal
Enable your ROS2 robot (Gazebo or hardware) to fetch and execute joint commands from any website backend, regardless of OS (WSL2, native Linux, Windows, cloud VM, etc.).

---

## Key Concepts
- **Platform Agnostic:** Your ROS2 control code should work on any Linux/WSL2/Windows machine, and with any backend IP/domain.
- **API Endpoint Discovery:** Dynamically determine the backend API address (not hardcoded to localhost or a specific IP).
- **Network Accessibility:** Ensure the backend is reachable from the ROS2 host (firewall, routing, etc.).
- **Security:** Use HTTPS and authentication for production.

---

## Step-by-Step Workflow

### 1. Host Your Website Backend
- Deploy your backend (Node.js, Flask, Django, etc.) on a public server, cloud VM, or local network.
- Example: `https://myrobotapi.example.com` or `http://192.168.1.100:3000`
- Make sure the backend is accessible from the ROS2 host (test with `curl`).

### 2. Discover Backend API Address
- **Best Practice:** Pass the backend API URL as an environment variable or config file to your ROS2 code.
- Example:
  ```bash
  export ROBOT_API_URL="https://myrobotapi.example.com"
  # Or in Python:
  import os
  api_url = os.getenv('ROBOT_API_URL', 'http://localhost:3000')
  ```
- For Docker/VM: Use container environment variables or config mounts.
- For cloud: Use secrets/config services.

### 3. Fetch API from ROS2 (Python Example)
```python
import os
import requests

API_URL = os.getenv('ROBOT_API_URL', 'http://localhost:3000')

# Example: Get joint commands
response = requests.get(f"{API_URL}/api/joints")
if response.ok:
    joints = response.json()['joints']
    print(joints)
else:
    print("API unreachable!", response.status_code)
```
- Use the same pattern for POST requests to move joints.

### 4. Dynamic IP Handling
- **Local Network:** Use the Linux host's IP (from `hostname -I` or `ip addr`) for backend to reach ROS2, and vice versa.
- **Cloud/Remote:** Use public DNS or IP.
- **No Hardcoding:** Never hardcode `localhost` or a static IP in your backend or ROS2 code.
- **Configurable:** Always allow the API URL to be set at runtime.

### 5. Cross-Platform Tips
- **WSL2:** Use the WSL2 IP for communication from Windows to WSL2, but use the Linux IP for native Linux.
- **Docker:** Use container networking (service names, bridge IPs).
- **Firewall:** Open required ports (e.g., 3000, 8080) on both backend and ROS2 host.
- **Testing:** Use `curl`, `ping`, and browser to verify connectivity.

### 6. Example: Universal Startup Script
```bash
# Set API URL dynamically
export ROBOT_API_URL="http://192.168.1.100:3000"
python3 my_ros2_control_script.py
```
- Or pass as a command-line argument:
  ```bash
  python3 my_ros2_control_script.py --api-url "https://myrobotapi.example.com"
  ```

### 7. Backend Code Suggestions
- Accept requests from any IP (use `0.0.0.0` as host in Express/Flask).
- Log incoming requests and IPs for debugging.
- Document the API endpoints clearly.

---

## Troubleshooting
- **API Not Reachable:** Check IP, firewall, and port. Try `curl` from ROS2 host.
- **Wrong IP:** Use `hostname -I` on Linux/WSL2 to get correct IP.
- **CORS Issues:** Allow CORS in backend for web clients.
- **Authentication:** Add API keys or OAuth for production.

---

## Summary
- Always make the backend API URL configurable.
- Use environment variables, config files, or command-line args.
- Test connectivity from ROS2 host to backend.
- Avoid hardcoding IPs or localhost.
- Works for WSL2, native Linux, Windows, Docker, cloud, etc.

---

## Further Reading
- [ROS2 Networking Guide](https://docs.ros.org/en/rolling/Guides/NetworkSetup.html)
- [Express.js Deployment](https://expressjs.com/en/advanced/best-practice-performance.html)
- [Python requests library](https://docs.python-requests.org/en/latest/)
- [Linux Networking Basics](https://wiki.archlinux.org/title/Network_configuration)
