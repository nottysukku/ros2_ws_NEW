# Integrating xarmapi.py & armapi_cli.py with Web API for ROS2 Control

## Goal
Enable your STM32 MPU (running Linux) to send POST requests from `xarmapi.py` and `armapi_cli.py` to a website backend, so joint commands can be executed in your ROS2 Gazebo simulation on a Linux laptop.

---

## Architecture Overview
```
[STM32 MPU (Linux)] → [Website Backend API] → [ROS2 Gazebo (Linux Laptop)]
```
- STM32 runs Python scripts (`xarmapi.py`, `armapi_cli.py`)
- Scripts send POST requests to backend API
- Backend forwards commands to ROS2 Gazebo

---

## Step 1: Add HTTP POST Support to xarmapi.py

1. **Import requests library:**
   ```python
   import requests
   import os
   ```
2. **Set API URL (configurable):**
   ```python
   API_URL = os.getenv('ROBOT_API_URL', 'http://your-backend-ip:3000')
   ```
3. **Add a method to send joint commands:**
   ```python
   def send_joint_command(joint_name, position):
       payload = {'joint_name': joint_name, 'position': position}
       response = requests.post(f"{API_URL}/api/joint/{joint_name}/move", json=payload)
       return response.ok, response.json() if response.ok else response.text
   ```
4. **Call this method after local servo move (optional):**
   ```python
   # After moving servos locally
   ok, resp = send_joint_command('joint1', 1.57)
   print('API response:', resp)
   ```

---

## Step 2: Update armapi_cli.py for Web API Integration

1. **Import requests and set API URL:**
   ```python
   import requests
   import os
   API_URL = os.getenv('ROBOT_API_URL', 'http://your-backend-ip:3000')
   ```
2. **Add a function to send multiple joint moves:**
   ```python
   def send_multi_joint_command(servo_moves):
       joints = {f'joint{servo_id}': pos for servo_id, pos in servo_moves}
       payload = {'joints': joints}
       response = requests.post(f"{API_URL}/api/joints/move", json=payload)
       return response.ok, response.json() if response.ok else response.text
   ```
3. **Call this function after local move:**
   ```python
   if servo_moves:
       controller.move_servos(servo_moves, 300)
       ok, resp = send_multi_joint_command(servo_moves)
       print('API response:', resp)
   ```

---

## Step 3: Testing from STM32 MPU
- Ensure Python `requests` is installed: `pip3 install requests`
- Set the API URL:
  ```bash
  export ROBOT_API_URL="http://your-backend-ip:3000"
  python3 armapi_cli.py
  ```
- After entering servo positions, the script will send POST requests to the backend.
- Check backend logs and ROS2 Gazebo for joint movement.

---

## Step 4: Backend API Requirements
- Must accept POST requests at `/api/joint/:jointName/move` and `/api/joints/move`
- Should forward commands to ROS2 Gazebo
- Allow CORS if accessed from browser
- Document expected payloads (see above)

---

## Step 5: Security & Reliability
- Use HTTPS for production
- Add authentication (API keys, tokens)
- Handle network errors gracefully in Python scripts
- Log all API responses for debugging

---

## Example: Minimal POST Request in Python
```python
import requests
API_URL = 'http://your-backend-ip:3000'
payload = {'joint_name': 'joint1', 'position': 1.57}
response = requests.post(f"{API_URL}/api/joint/joint1/move", json=payload)
print(response.status_code, response.text)
```

---

## Summary
- Update `xarmapi.py` and `armapi_cli.py` to send POST requests after local moves
- Make API URL configurable via environment variable
- Test end-to-end from STM32 to backend to ROS2 Gazebo
- Use Python `requests` for HTTP calls
- Works for any Linux device, including STM32 MPU

---

## Further Reading
- [Python requests library](https://docs.python-requests.org/en/latest/)
- [STM32 MPU Linux documentation](https://wiki.st.com/stm32mpu/index.php/Linux)
- [REST API design](https://restfulapi.net/)
