import requests
import os
# Add function to fetch API calls from cloud backend
def fetch_data_from_cloud_api():
    API_URL = os.getenv('API_URL', 'https://ros2-joint-controller-api.onrender.com')
    try:
        response = requests.get(f'{API_URL}/api/joints')
        joints = response.json()['joints']
        print('Received joints data from cloud API:', joints)
        # Further processing can be done here if needed.
    except requests.exceptions.RequestException as e:
        print(f"Error fetching data from cloud API: {e}")
#!/usr/bin/env python3
"""
WSL2 ROS2 Bridge Server
Receives HTTP requests from Windows and publishes to ROS2 topics
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import json
import threading
import time
from flask import Flask, request, jsonify
from flask_cors import CORS

class ROS2BridgeNode(Node):
    def __init__(self):
        super().__init__('ros2_bridge_node')
        # Dictionary to store joint publishers
        self.joint_publishers = {}

        # Joint configuration - update this to match your robot
        self.joint_config = {
            'joint1': {
                'topic': '/joint1_cmd',
                'type': 'std_msgs/Float64'
            },
            'joint2': {
                'topic': '/joint2_cmd',
                'type': 'std_msgs/Float64'
            },
            'joint3': {
                'topic': '/joint3_cmd',
                'type': 'std_msgs/Float64'
            },
            'joint4': {
                'topic': '/joint4_cmd',
                'type': 'std_msgs/Float64'
            },
            'joint5': {
                'topic': '/joint5_cmd',
                'type': 'std_msgs/Float64'
            }
        }

        # Create publishers for each joint
        for joint_name, config in self.joint_config.items():
            topic_name = config['topic']
            self.joint_publishers[joint_name] = self.create_publisher(
                Float64,
                topic_name,
                10
            )
            self.get_logger().info(f'Created publisher for {joint_name} on topic {topic_name}')

        self.get_logger().info('ROS2 Bridge Node initialized successfully')
    
    def move_joint(self, joint_name, position):
        """Move a single joint to specified position"""
        if joint_name not in self.joint_publishers:
            self.get_logger().error(f'Joint {joint_name} not found')
            return False
        try:
            msg = Float64()
            msg.data = float(position)
            self.joint_publishers[joint_name].publish(msg)
            self.get_logger().info(f'Published {position} to {joint_name}')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to move joint {joint_name}: {str(e)}')
            return False
    
    def move_multiple_joints(self, joints_data):
        """Move multiple joints simultaneously"""
        results = []
        
        for joint_data in joints_data:
            joint_name = joint_data.get('joint_name')
            position = joint_data.get('position')
            
            if not joint_name or position is None:
                results.append({
                    'joint': joint_name,
                    'success': False,
                    'error': 'Missing joint_name or position'
                })
                continue
            
            success = self.move_joint(joint_name, position)
            results.append({
                'joint': joint_name,
                'position': position,
                'success': success
            })
        
        return results
    
    def get_joint_list(self):
        """Get list of available joints"""
        return [
            {
                'name': joint_name,
                'topic': config['topic'],
                'type': config['type']
            }
            for joint_name, config in self.joint_config.items()
        ]

# Global ROS2 node instance
ros_node = None

# Flask web server
app = Flask(__name__)
CORS(app)

@app.route('/health', methods=['GET'])
def health_check():
    """Health check endpoint"""
    return jsonify({
        'status': 'healthy',
        'timestamp': time.time(),
        'ros2_initialized': ros_node is not None,
        'available_joints': len(ros_node.joint_config) if ros_node else 0,
        'server': 'WSL2 ROS2 Bridge'
    })

@app.route('/joints', methods=['GET'])
def get_joints():
    """Get available joints"""
    if not ros_node:
        return jsonify({'error': 'ROS2 not initialized'}), 500
    
    return jsonify({
        'joints': ros_node.get_joint_list()
    })

@app.route('/joint/move', methods=['POST'])
def move_single_joint():
    """Move a single joint"""
    if not ros_node:
        return jsonify({'error': 'ROS2 not initialized'}), 500
    
    data = request.get_json()
    if not data:
        return jsonify({'error': 'No JSON data provided'}), 400
    
    joint_name = data.get('joint_name')
    position = data.get('position')
    
    if not joint_name or position is None:
        return jsonify({
            'error': 'joint_name and position are required'
        }), 400
    
    try:
        success = ros_node.move_joint(joint_name, position)
        
        if success:
            return jsonify({
                'success': True,
                'joint': joint_name,
                'position': position,
                'timestamp': time.time()
            })
        else:
            return jsonify({
                'success': False,
                'joint': joint_name,
                'error': f'Failed to move joint {joint_name}'
            }), 500
            
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500

@app.route('/joints/move', methods=['POST'])
def move_multiple_joints():
    """Move multiple joints"""
    if not ros_node:
        return jsonify({'error': 'ROS2 not initialized'}), 500
    
    data = request.get_json()
    if not data:
        return jsonify({'error': 'No JSON data provided'}), 400
    
    joints_data = data.get('joints')
    if not joints_data:
        return jsonify({'error': 'joints data is required'}), 400
    
    try:
        results = ros_node.move_multiple_joints(joints_data)
        
        return jsonify({
            'success': True,
            'results': results,
            'timestamp': time.time()
        })
        
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500

@app.route('/topics/list', methods=['GET'])
def list_topics():
    """List all available ROS2 topics (for debugging)"""
    if not ros_node:
        return jsonify({'error': 'ROS2 not initialized'}), 500
    
    try:
        # Get topic list
        topic_names_and_types = ros_node.get_topic_names_and_types()
        
        topics = []
        for topic_name, topic_types in topic_names_and_types:
            topics.append({
                'name': topic_name,
                'types': topic_types
            })
        
        return jsonify({
            'topics': topics,
            'count': len(topics)
        })
        
    except Exception as e:
        return jsonify({
            'error': f'Failed to get topics: {str(e)}'
        }), 500

@app.route('/demo/sine_wave', methods=['POST'])
def demo_sine_wave():
    """Demo: Move joint in sine wave pattern"""
    if not ros_node:
        return jsonify({'error': 'ROS2 not initialized'}), 500
    
    data = request.get_json() or {}
    joint_name = data.get('joint_name', 'joint2')
    amplitude = data.get('amplitude', 1.5)
    frequency = data.get('frequency', 0.5)
    duration = data.get('duration', 10)
    
    def sine_wave_demo():
        import math
        start_time = time.time()
        
        while time.time() - start_time < duration:
            t = time.time() - start_time
            position = amplitude * math.sin(2 * math.pi * frequency * t)
            ros_node.move_joint(joint_name, position)
            time.sleep(0.1)  # 10 Hz
        
        # Return to home
        ros_node.move_joint(joint_name, 0.0)
    
    # Run demo in background thread
    demo_thread = threading.Thread(target=sine_wave_demo)
    demo_thread.daemon = True
    demo_thread.start()
    
    return jsonify({
        'success': True,
        'message': f'Started sine wave demo on {joint_name}',
        'parameters': {
            'joint': joint_name,
            'amplitude': amplitude,
            'frequency': frequency,
            'duration': duration
        }
    })

def ros_spinner():
    """Spin ROS2 node in separate thread"""
    global ros_node
    
    try:
        rclpy.init()
        ros_node = ROS2BridgeNode()
        rclpy.spin(ros_node)
    except Exception as e:
        print(f"ROS2 spinner error: {e}")
    finally:
        if ros_node:
            ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    print("🚀 Starting WSL2 ROS2 Bridge Server...")
    
    # Start ROS2 node in background thread
    ros_thread = threading.Thread(target=ros_spinner)
    ros_thread.daemon = True
    ros_thread.start()

    # Wait for ROS2 node to initialize (max 10 seconds)
    for _ in range(100):
        if ros_node is not None:
            break
        time.sleep(0.1)
    if ros_node is None:
        print("❌ Failed to initialize ROS2 node")
        exit(1)

    print("✅ ROS2 node initialized")
    print("🌐 Starting Flask web server...")

    # Get WSL2 IP address
    import socket
    try:
        # Get WSL2 IP
        hostname = socket.gethostname()
        wsl2_ip = socket.gethostbyname(hostname)
        print(f"🔗 WSL2 IP Address: {wsl2_ip}")
        print(f"🌐 Server will be available at: http://{wsl2_ip}:8080")
    except:
        wsl2_ip = '0.0.0.0'
        print("⚠️  Could not determine WSL2 IP, binding to all interfaces")

    print("\n📋 Available endpoints:")
    print(f"  GET  http://{wsl2_ip}:8080/health")
    print(f"  GET  http://{wsl2_ip}:8080/joints")
    print(f"  POST http://{wsl2_ip}:8080/joint/move")
    print(f"  POST http://{wsl2_ip}:8080/joints/move")
    print(f"  POST http://{wsl2_ip}:8080/demo/sine_wave")
    print(f"  GET  http://{wsl2_ip}:8080/topics/list")

    print(f"\n💡 Update your Windows server with: set WSL2_IP={wsl2_ip}")

    # Start Flask server
    try:
        app.run(host='0.0.0.0', port=8080, debug=False, threaded=True)
    except KeyboardInterrupt:
        print("\n👋 Shutting down WSL2 ROS2 Bridge Server...")
    except Exception as e:
        print(f"❌ Server error: {e}")
    finally:
        if ros_node:
            ros_node.destroy_node()
        rclpy.shutdown()