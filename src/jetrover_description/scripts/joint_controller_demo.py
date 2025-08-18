#!/usr/bin/env python3

"""
JetRover Arm and Gripper Controller Demo

This script demonstrates how to control the JetRover arm joints and gripper
using the position controllers defined in the SDF file.

Usage:
    ros2 run jetrover_description joint_controller_demo.py

Topics published to:
    /joint1/cmd_pos - Joint 1 position command (base rotation)
    /joint2/cmd_pos - Joint 2 position command (shoulder)  
    /joint3/cmd_pos - Joint 3 position command (elbow)
    /joint4/cmd_pos - Joint 4 position command (wrist pitch)
    /joint5/cmd_pos - Joint 5 position command (wrist roll)
    /gripper/cmd_pos - Gripper position command (controls r_joint, others follow via mimic)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math
import time
from threading import Thread


class JetRoverController(Node):
    def __init__(self):
        super().__init__('jetrover_controller')
        
        # Create publishers for each joint
        self.joint1_pub = self.create_publisher(Float64, '/joint1/cmd_pos', 10)
        self.joint2_pub = self.create_publisher(Float64, '/joint2/cmd_pos', 10)
        self.joint3_pub = self.create_publisher(Float64, '/joint3/cmd_pos', 10)
        self.joint4_pub = self.create_publisher(Float64, '/joint4/cmd_pos', 10)
        self.joint5_pub = self.create_publisher(Float64, '/joint5/cmd_pos', 10)
        self.gripper_pub = self.create_publisher(Float64, '/gripper/cmd_pos', 10)
        
        # Joint limits (from SDF file)
        self.joint_limits = {
            'joint1': (-2.09, 2.09),
            'joint2': (-2.09, 2.09),
            'joint3': (-2.09, 2.09),
            'joint4': (-2.09, 2.09),
            'joint5': (-2.09, 2.09),
            'gripper': (-1.57, 1.57)  # r_joint limits
        }
        
        self.get_logger().info('JetRover Controller initialized')
        
    def publish_joint_position(self, joint_name, position):
        """Publish a position command to a specific joint"""
        # Check joint limits
        if joint_name in self.joint_limits:
            min_pos, max_pos = self.joint_limits[joint_name]
            position = max(min_pos, min(max_pos, position))
        
        msg = Float64()
        msg.data = position
        
        if joint_name == 'joint1':
            self.joint1_pub.publish(msg)
        elif joint_name == 'joint2':
            self.joint2_pub.publish(msg)
        elif joint_name == 'joint3':
            self.joint3_pub.publish(msg)
        elif joint_name == 'joint4':
            self.joint4_pub.publish(msg)
        elif joint_name == 'joint5':
            self.joint5_pub.publish(msg)
        elif joint_name == 'gripper':
            self.gripper_pub.publish(msg)
            
        self.get_logger().info(f'Published {joint_name}: {position:.2f} rad')
    
    def move_to_home_position(self):
        """Move all joints to home position (0.0 radians)"""
        self.get_logger().info('Moving to home position...')
        joints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        for joint in joints:
            self.publish_joint_position(joint, 0.0)
            time.sleep(0.1)
    
    def open_gripper(self):
        """Open the gripper by setting r_joint to negative value"""
        self.get_logger().info('Opening gripper...')
        self.publish_joint_position('gripper', -1.0)  # Open position
    
    def close_gripper(self):
        """Close the gripper by setting r_joint to positive value"""
        self.get_logger().info('Closing gripper...')
        self.publish_joint_position('gripper', 1.0)   # Closed position
    
    def demo_sequence(self):
        """Run a demonstration sequence"""
        self.get_logger().info('Starting demonstration sequence...')
        
        # Wait for connections
        time.sleep(2.0)
        
        # 1. Move to home position
        self.move_to_home_position()
        time.sleep(3.0)
        
        # 2. Open gripper
        self.open_gripper()
        time.sleep(2.0)
        
        # 3. Move arm to pickup position
        self.get_logger().info('Moving to pickup position...')
        self.publish_joint_position('joint1', 0.5)    # Rotate base
        time.sleep(1.0)
        self.publish_joint_position('joint2', -0.5)   # Lower shoulder
        time.sleep(1.0)
        self.publish_joint_position('joint3', 1.0)    # Bend elbow
        time.sleep(1.0)
        self.publish_joint_position('joint4', -0.5)   # Adjust wrist
        time.sleep(2.0)
        
        # 4. Close gripper (simulate pickup)
        self.close_gripper()
        time.sleep(2.0)
        
        # 5. Lift object
        self.get_logger().info('Lifting object...')
        self.publish_joint_position('joint2', 0.2)    # Raise shoulder
        time.sleep(2.0)
        
        # 6. Rotate and place
        self.get_logger().info('Rotating to place position...')
        self.publish_joint_position('joint1', -0.5)   # Rotate base other way
        time.sleep(2.0)
        
        # 7. Lower to place
        self.get_logger().info('Placing object...')
        self.publish_joint_position('joint2', -0.3)   # Lower shoulder
        time.sleep(2.0)
        
        # 8. Open gripper (release)
        self.open_gripper()
        time.sleep(2.0)
        
        # 9. Return to home
        self.get_logger().info('Returning to home position...')
        self.move_to_home_position()
        time.sleep(3.0)
        
        self.get_logger().info('Demonstration complete!')


def main(args=None):
    rclpy.init(args=args)
    
    controller = JetRoverController()
    
    # Run the demo in a separate thread
    demo_thread = Thread(target=controller.demo_sequence)
    demo_thread.daemon = True
    demo_thread.start()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
