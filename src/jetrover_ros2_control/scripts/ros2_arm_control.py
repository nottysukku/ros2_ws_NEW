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
        
        self.get_logger().info("🤖 JetRover Controller initialized - Ready for ROS2 commands!")
        
    def move_joint(self, joint_name, position):
        """Move a single joint to specified position"""
        msg = Float64()
        msg.data = float(position)
        self.joint_pubs[joint_name].publish(msg)
        self.get_logger().info(f"Moving {joint_name} to {position:.3f}")
        
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
        self.get_logger().info("🎯 Moving to white ball position")
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
        self.get_logger().info("📍 Moving to board center position")
        self.move_to_position(positions, 3.0)
        
    def close_gripper(self):
        """Close gripper to grab object"""
        positions = {
            'left_finger': 0.0,
            'right_finger': 0.0
        }
        self.get_logger().info("✋ Closing gripper")
        self.move_to_position(positions, 1.0)
        
    def open_gripper(self):
        """Open gripper to release object"""
        positions = {
            'left_finger': 0.15,
            'right_finger': 0.15
        }
        self.get_logger().info("✋ Opening gripper")
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
        self.get_logger().info("🏠 Moving to home position")
        self.move_to_position(positions, 4.0)
        
    def execute_pickup_sequence(self):
        """Execute complete white ball pickup sequence"""
        self.get_logger().info("🚀 Starting white ball pickup sequence")
        
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
        self.get_logger().info("🔄 Sequence complete - node ready for manual commands")
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        controller.get_logger().info("👋 Shutting down JetRover Controller")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
