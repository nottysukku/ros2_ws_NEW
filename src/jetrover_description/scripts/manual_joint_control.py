#!/usr/bin/env python3

"""
Manual JetRover Joint Controller

This script allows manual control of individual joints via command line input.
Useful for testing and debugging joint movements.

Usage:
    ros2 run jetrover_description manual_joint_control.py

Commands:
    j1 <angle>    - Move joint1 to angle (radians)
    j2 <angle>    - Move joint2 to angle (radians)  
    j3 <angle>    - Move joint3 to angle (radians)
    j4 <angle>    - Move joint4 to angle (radians)
    j5 <angle>    - Move joint5 to angle (radians)
    grip <angle>  - Move gripper to angle (radians)
    open          - Open gripper fully
    close         - Close gripper fully
    home          - Move all joints to home position (0.0)
    limits        - Show joint limits
    help          - Show this help message
    quit          - Exit program
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import threading
import sys


class ManualJointController(Node):
    def __init__(self):
        super().__init__('manual_joint_controller')
        
        # Create publishers for each joint
        self.publishers = {
            'joint1': self.create_publisher(Float64, '/joint1/cmd_pos', 10),
            'joint2': self.create_publisher(Float64, '/joint2/cmd_pos', 10),
            'joint3': self.create_publisher(Float64, '/joint3/cmd_pos', 10),
            'joint4': self.create_publisher(Float64, '/joint4/cmd_pos', 10),
            'joint5': self.create_publisher(Float64, '/joint5/cmd_pos', 10),
            'gripper': self.create_publisher(Float64, '/gripper/cmd_pos', 10),
        }
        
        # Joint limits (from SDF file)
        self.joint_limits = {
            'joint1': (-2.09, 2.09),
            'joint2': (-2.09, 2.09),
            'joint3': (-2.09, 2.09),
            'joint4': (-2.09, 2.09),
            'joint5': (-2.09, 2.09),
            'gripper': (-1.57, 1.57)
        }
        
        # Current positions
        self.current_positions = {joint: 0.0 for joint in self.publishers.keys()}
        
        self.get_logger().info('Manual Joint Controller initialized')
        self.print_help()
        
    def publish_joint_position(self, joint_name, position):
        """Publish a position command to a specific joint"""
        if joint_name not in self.publishers:
            self.get_logger().error(f'Unknown joint: {joint_name}')
            return False
            
        # Check joint limits
        min_pos, max_pos = self.joint_limits[joint_name]
        if position < min_pos or position > max_pos:
            self.get_logger().warning(
                f'Position {position:.2f} for {joint_name} exceeds limits [{min_pos:.2f}, {max_pos:.2f}]'
            )
            position = max(min_pos, min(max_pos, position))
        
        msg = Float64()
        msg.data = position
        self.publishers[joint_name].publish(msg)
        
        self.current_positions[joint_name] = position
        self.get_logger().info(f'{joint_name}: {position:.3f} rad ({position * 180 / 3.14159:.1f}°)')
        return True
    
    def move_to_home(self):
        """Move all joints to home position"""
        self.get_logger().info('Moving all joints to home position...')
        for joint_name in self.publishers.keys():
            self.publish_joint_position(joint_name, 0.0)
    
    def open_gripper(self):
        """Open gripper fully"""
        self.publish_joint_position('gripper', -1.4)  # Nearly fully open
    
    def close_gripper(self):
        """Close gripper fully"""
        self.publish_joint_position('gripper', 1.4)   # Nearly fully closed
        
    def print_limits(self):
        """Print joint limits"""
        print("\n=== Joint Limits ===")
        for joint, (min_pos, max_pos) in self.joint_limits.items():
            print(f"{joint:8}: [{min_pos:6.2f}, {max_pos:6.2f}] rad "
                  f"([{min_pos*180/3.14159:6.1f}°, {max_pos*180/3.14159:6.1f}°])")
        print()
    
    def print_current_positions(self):
        """Print current joint positions"""
        print("\n=== Current Positions ===")
        for joint, pos in self.current_positions.items():
            print(f"{joint:8}: {pos:7.3f} rad ({pos*180/3.14159:6.1f}°)")
        print()
    
    def print_help(self):
        """Print help message"""
        print("\n" + "="*50)
        print("Manual JetRover Joint Controller")
        print("="*50)
        print("Commands:")
        print("  j1 <angle>    - Move joint1 to angle (radians)")
        print("  j2 <angle>    - Move joint2 to angle (radians)")
        print("  j3 <angle>    - Move joint3 to angle (radians)")
        print("  j4 <angle>    - Move joint4 to angle (radians)")
        print("  j5 <angle>    - Move joint5 to angle (radians)")
        print("  grip <angle>  - Move gripper to angle (radians)")
        print("  open          - Open gripper fully")
        print("  close         - Close gripper fully")
        print("  home          - Move all joints to home position")
        print("  limits        - Show joint limits")
        print("  pos           - Show current positions")
        print("  help          - Show this help message")
        print("  quit/exit     - Exit program")
        print("\nNote: Angles in radians. Use 'limits' to see valid ranges.")
        print("="*50)


def command_loop(controller):
    """Command input loop running in separate thread"""
    while rclpy.ok():
        try:
            cmd_input = input("\njetrover> ").strip().lower()
            
            if not cmd_input:
                continue
                
            parts = cmd_input.split()
            cmd = parts[0]
            
            if cmd in ['quit', 'exit', 'q']:
                controller.get_logger().info("Shutting down...")
                rclpy.shutdown()
                break
            elif cmd == 'help' or cmd == 'h':
                controller.print_help()
            elif cmd == 'limits':
                controller.print_limits()
            elif cmd == 'pos':
                controller.print_current_positions()
            elif cmd == 'home':
                controller.move_to_home()
            elif cmd == 'open':
                controller.open_gripper()
            elif cmd == 'close':
                controller.close_gripper()
            elif cmd in ['j1', 'j2', 'j3', 'j4', 'j5', 'grip']:
                if len(parts) != 2:
                    print(f"Usage: {cmd} <angle_in_radians>")
                    continue
                try:
                    angle = float(parts[1])
                    joint_map = {
                        'j1': 'joint1',
                        'j2': 'joint2', 
                        'j3': 'joint3',
                        'j4': 'joint4',
                        'j5': 'joint5',
                        'grip': 'gripper'
                    }
                    controller.publish_joint_position(joint_map[cmd], angle)
                except ValueError:
                    print("Invalid angle. Please enter a number.")
            else:
                print(f"Unknown command: {cmd}. Type 'help' for available commands.")
                
        except (EOFError, KeyboardInterrupt):
            controller.get_logger().info("Shutting down...")
            rclpy.shutdown()
            break
        except Exception as e:
            controller.get_logger().error(f"Error processing command: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    controller = ManualJointController()
    
    # Start command input in separate thread
    cmd_thread = threading.Thread(target=command_loop, args=(controller,))
    cmd_thread.daemon = True
    cmd_thread.start()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
