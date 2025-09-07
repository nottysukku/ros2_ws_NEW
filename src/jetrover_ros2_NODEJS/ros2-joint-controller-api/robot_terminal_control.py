#!/usr/bin/env python3
"""
Terminal-based Robot Arm Control Interface
Simple command-line interface for controlling robot arm positions
"""

import requests
import os
import time
import sys
from typing import Dict

class RobotTerminalControl:
    def __init__(self):
        # API configuration
        self.api_url = os.getenv('ROBOT_API_URL', 'https://ros2-joint-controller-api.onrender.com')
        
        # Predefined robot poses (updated for calm and composed movement)
                # Enhanced poses for seamless pickup and placement with crane-like movement
        self.poses = {
            'home': {'joint1': 0.0, 'joint2': 0.0, 'joint3': 0.0, 'joint4': 0.0, 'joint5': 0.0, 'left_finger': 0.0, 'right_finger': 0.0},
            'crane_very_high': {'joint1': 0.59, 'joint2': -0.50, 'joint3': 1.20, 'joint4': 0.30, 'joint5': 0.05, 'left_finger': 0.0, 'right_finger': 0.0},
            'crane_high': {'joint1': 0.59, 'joint2': 0.10, 'joint3': 0.50, 'joint4': 0.70, 'joint5': 0.06, 'left_finger': 0.15, 'right_finger': 0.15},
            'pickup_approach': {'joint1': 0.59, 'joint2': 0.50, 'joint3': -0.20, 'joint4': 0.95, 'joint5': 0.06, 'left_finger': 0.15, 'right_finger': 0.15},
            'pickup': {'joint1': 0.59, 'joint2': 0.94, 'joint3': -0.96, 'joint4': 1.22, 'joint5': 0.06, 'left_finger': 0.15, 'right_finger': 0.15},
            'carry_high': {'joint1': 0.20, 'joint2': -0.30, 'joint3': 0.80, 'joint4': 0.60, 'joint5': 0.05, 'left_finger': 0.0, 'right_finger': 0.0},
            'transit': {'joint1': 0.10, 'joint2': -0.50, 'joint3': 1.20, 'joint4': 0.40, 'joint5': 0.00, 'left_finger': 0.0, 'right_finger': 0.0},
            'pre_place': {'joint1': 0.12, 'joint2': -0.30, 'joint3': 1.40, 'joint4': 0.20, 'joint5': -0.10, 'left_finger': 0.0, 'right_finger': 0.0},
            'waiting': {'joint1': 0.0, 'joint2': -0.80, 'joint3': 1.40, 'joint4': 0.20, 'joint5': 0.0, 'left_finger': 0.0, 'right_finger': 0.0},
            'safe_retreat': {'joint1': 0.0, 'joint2': -0.60, 'joint3': 1.00, 'joint4': 0.50, 'joint5': 0.0, 'left_finger': 0.0, 'right_finger': 0.0},
            'grip_open': {'left_finger': 0.20, 'right_finger': 0.20},
            'grip_close': {'left_finger': 0.0, 'right_finger': 0.0},
            
            # Test positions using updated tile values
            'tile1': {'joint1': 0.34, 'joint2': -0.42, 'joint3': 1.52, 'joint4': 0.35, 'joint5': -0.13, 'left_finger': 0.0, 'right_finger': 0.0},
            'tile5': {'joint1': 0.12, 'joint2': 0.0, 'joint3': 2.03, 'joint4': -1.36, 'joint5': 1.44, 'left_finger': 0.0, 'right_finger': 0.0},
            'tile9': {'joint1': -0.15, 'joint2': 0.92, 'joint3': 0.65, 'joint4': -1.74, 'joint5': 1.76, 'left_finger': 0.0, 'right_finger': 0.0}
        }
        
        print("🚀 Robot Terminal Control Interface Starting...")
        self.check_connection()
    
    def check_connection(self):
        """Check API connection"""
        try:
            print("🔗 Checking robot connection...")
            response = requests.get(f'{self.api_url}/api/health', timeout=5)
            if response.ok:
                data = response.json()
                print(f"✅ Connected to robot API: {data.get('status', 'unknown')}")
                if data.get('wsl2_connected'):
                    print(f"✅ WSL2 bridge connected")
                else:
                    print(f"⚠️  WSL2 bridge status: {data.get('error', 'unknown')}")
            else:
                print(f"❌ API connection failed: {response.status_code}")
        except Exception as e:
            print(f"❌ Connection error: {e}")
            print("⚠️  Continuing anyway - robot commands may fail")
    
    def move_robot_arm(self, joint_positions: Dict, description: str = "Moving arm", joint_delay: float = 0.6) -> bool:
        """Send joint commands to robot arm via API - calm and composed sequential movement"""
        try:
            print(f"🤖 {description}...")
            
            # Define joint order for sequential movement (arm joints first, then gripper)
            joint_order = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'left_finger', 'right_finger']
            
            # Move each joint individually with calm delays for smooth operation
            for joint_name in joint_order:
                if joint_name in joint_positions:
                    position = joint_positions[joint_name]
                    print(f"   📍 Moving {joint_name} to {position}")
                    
                    # Send individual joint command
                    response = requests.post(
                        f'{self.api_url}/api/joint/{joint_name}/move', 
                        json={'position': position}, 
                        timeout=10
                    )
                    
                    if response.ok:
                        print(f"   ✅ {joint_name} moved successfully")
                    else:
                        print(f"   ❌ {joint_name} move failed: {response.status_code}")
                        return False
                    
                    # Calm, composed timing between joint movements
                    if joint_name in ['joint1', 'joint2', 'joint3']:
                        time.sleep(joint_delay + 0.2)  # Extra time for major joints
                    else:
                        time.sleep(joint_delay)
            
            print(f"✅ {description} completed smoothly")
            return True
            
        except requests.exceptions.RequestException as e:
            print(f"❌ Robot API error: {e}")
            return False
    
    def move_single_joint(self, joint_name: str, position: float, description: str = None) -> bool:
        """Move a single joint - useful for fine control"""
        if description is None:
            description = f"Moving {joint_name} to {position}"
        
        try:
            print(f"🤖 {description}...")
            response = requests.post(
                f'{self.api_url}/api/joint/{joint_name}/move', 
                json={'position': position}, 
                timeout=10
            )
            
            if response.ok:
                print(f"✅ {joint_name} moved to {position}")
                return True
            else:
                print(f"❌ {joint_name} move failed: {response.status_code}")
                return False
                
        except requests.exceptions.RequestException as e:
            print(f"❌ Robot API error: {e}")
            return False
    
    def move_to_pickup_position(self, description: str = "Moving to pickup position") -> bool:
        """Special pickup maneuver: open gripper after joint2, then continue with remaining joints"""
        try:
            print(f"🤖 {description}...")
            pickup_pose = self.poses['pickup']
            
            # Step 1: Move joints 1 and 2 first
            initial_joints = ['joint1', 'joint2']
            for joint_name in initial_joints:
                if joint_name in pickup_pose:
                    position = pickup_pose[joint_name]
                    print(f"   📍 Moving {joint_name} to {position}")
                    
                    response = requests.post(
                        f'{self.api_url}/api/joint/{joint_name}/move', 
                        json={'position': position}, 
                        timeout=10
                    )
                    
                    if response.ok:
                        print(f"   ✅ {joint_name} moved successfully")
                    else:
                        print(f"   ❌ {joint_name} move failed: {response.status_code}")
                        return False
                    
                    time.sleep(0.3)
            
            # Step 2: Open gripper after joint2 (critical for pickup safety)
            print("   🔓 Opening gripper after joint2 positioning...")
            gripper_joints = ['left_finger', 'right_finger']
            for joint_name in gripper_joints:
                if joint_name in pickup_pose:
                    position = pickup_pose[joint_name]
                    print(f"   📍 Moving {joint_name} to {position}")
                    
                    response = requests.post(
                        f'{self.api_url}/api/joint/{joint_name}/move', 
                        json={'position': position}, 
                        timeout=10
                    )
                    
                    if response.ok:
                        print(f"   ✅ {joint_name} moved successfully")
                    else:
                        print(f"   ❌ {joint_name} move failed: {response.status_code}")
                        return False
                    
                    time.sleep(0.2)
            
            # Step 3: Move joint5 after gripper opens
            if 'joint5' in pickup_pose:
                position = pickup_pose['joint5']
                print(f"   📍 Moving joint5 to {position}")
                
                response = requests.post(
                    f'{self.api_url}/api/joint/joint5/move', 
                    json={'position': position}, 
                    timeout=10
                )
                
                if response.ok:
                    print(f"   ✅ joint5 moved successfully")
                else:
                    print(f"   ❌ joint5 move failed: {response.status_code}")
                    return False
                
                time.sleep(0.3)
            
            # Step 4: Now move remaining joints 3 and 4
            final_joints = ['joint3', 'joint4']
            for joint_name in final_joints:
                if joint_name in pickup_pose:
                    position = pickup_pose[joint_name]
                    print(f"   📍 Moving {joint_name} to {position}")
                    
                    response = requests.post(
                        f'{self.api_url}/api/joint/{joint_name}/move', 
                        json={'position': position}, 
                        timeout=10
                    )
                    
                    if response.ok:
                        print(f"   ✅ {joint_name} moved successfully")
                    else:
                        print(f"   ❌ {joint_name} move failed: {response.status_code}")
                        return False
                    
                    time.sleep(0.3)
            
            print(f"✅ {description} completed with gripper opened after joint2, joint5 moved, then joints 3,4")
            return True
            
        except requests.exceptions.RequestException as e:
            print(f"❌ Robot API error: {e}")
            return False
    
    def safe_pickup_sequence(self) -> bool:
        """Execute ultra-safe pickup sequence: up first, open gripper from top, very slow descent"""
        try:
            print("🏗️ Executing ultra-safe crane pickup sequence...")
            print("📌 Step 1: Moving to high safe position first...")
            
            # Step 1: First move to a very high safe position above everything
            high_safe_pose = {
                'joint1': 0.59,    # Same rotation as pickup
                'joint2': -0.50,   # Much higher up to clear all balls
                'joint3': 0.80,    # High position
                'joint4': 0.20,    # Safe angle
                'joint5': 0.0,     # Neutral wrist
                'left_finger': 0.0, 'right_finger': 0.0  # Closed initially
            }
            
            success = self.move_robot_arm(high_safe_pose, "Moving to high safe position", 1.0)
            if not success:
                print("❌ Failed to reach high safe position")
                return False
            time.sleep(2.0)  # Long pause to ensure stability
            
            print("📌 Step 2: Opening gripper at safe height...")
            # Step 2: Open gripper while safely above everything
            success = self.move_robot_arm(self.poses['grip_open'], "Opening gripper at safe height", 0.5)
            if not success:
                print("❌ Failed to open gripper")
                return False
            time.sleep(1.5)
            
            print("📌 Step 3: Moving to crane high position...")
            # Step 3: Move to crane high position (still high but closer to pickup area)
            crane_high_with_open_gripper = self.poses['crane_high'].copy()
            crane_high_with_open_gripper.update(self.poses['grip_open'])  # Keep gripper open
            
            success = self.move_robot_arm(crane_high_with_open_gripper, "Moving to crane high position", 1.0)
            if not success:
                print("❌ Failed to reach crane high position")
                return False
            time.sleep(2.0)
            
            print("📌 Step 4: Very slow descent to approach position...")
            # Step 4: Very slow descent to approach position
            approach_with_open_gripper = self.poses['pickup_approach'].copy()
            approach_with_open_gripper.update(self.poses['grip_open'])  # Keep gripper open
            
            success = self.move_robot_arm(approach_with_open_gripper, "Very slow descent to approach", 1.5)
            if not success:
                print("❌ Failed to reach approach position")
                return False
            time.sleep(2.5)
            
            print("📌 Step 5: Ultra-careful final descent to pickup...")
            # Step 5: Ultra-careful final descent - move one joint at a time very slowly
            pickup_pose = self.poses['pickup']
            
            # Joint 2 first (main vertical movement) - VERY SLOW
            if 'joint2' in pickup_pose:
                position = pickup_pose['joint2']
                print(f"   📍 Ultra-slow joint2 descent to {position}")
                
                response = requests.post(
                    f'{self.api_url}/api/joint/joint2/move', 
                    json={'position': position}, 
                    timeout=10
                )
                
                if response.ok:
                    print(f"   ✅ joint2 descent successful")
                else:
                    print(f"   ❌ joint2 descent failed: {response.status_code}")
                    return False
                
                time.sleep(2.0)  # Very slow descent
            
            # Joint 3 second (elbow adjustment) - VERY SLOW  
            if 'joint3' in pickup_pose:
                position = pickup_pose['joint3']
                print(f"   📍 Ultra-slow joint3 positioning to {position}")
                
                response = requests.post(
                    f'{self.api_url}/api/joint/joint3/move', 
                    json={'position': position}, 
                    timeout=10
                )
                
                if response.ok:
                    print(f"   ✅ joint3 positioned successfully")
                else:
                    print(f"   ❌ joint3 positioning failed: {response.status_code}")
                    return False
                
                time.sleep(2.0)  # Very slow movement
            
            # Joint 4 third (wrist positioning) - CAREFUL
            if 'joint4' in pickup_pose:
                position = pickup_pose['joint4']
                print(f"   📍 Careful joint4 positioning to {position}")
                
                response = requests.post(
                    f'{self.api_url}/api/joint/joint4/move', 
                    json={'position': position}, 
                    timeout=10
                )
                
                if response.ok:
                    print(f"   ✅ joint4 positioned successfully")
                else:
                    print(f"   ❌ joint4 positioning failed: {response.status_code}")
                    return False
                
                time.sleep(1.5)
            
            # Joint 5 last (final wrist adjustment) - GENTLE
            if 'joint5' in pickup_pose:
                position = pickup_pose['joint5']
                print(f"   📍 Gentle joint5 adjustment to {position}")
                
                response = requests.post(
                    f'{self.api_url}/api/joint/joint5/move', 
                    json={'position': position}, 
                    timeout=10
                )
                
                if response.ok:
                    print(f"   ✅ joint5 adjusted successfully")
                else:
                    print(f"   ❌ joint5 adjustment failed: {response.status_code}")
                    return False
                
                time.sleep(1.0)
            
            print("✅ Ultra-safe pickup sequence completed - robot positioned for pickup without hitting balls")
            print("🎯 Robot is now in pickup position with gripper open and ready to grab")
            return True
            
        except Exception as e:
            print(f"❌ Ultra-safe pickup sequence error: {e}")
            return False
    
    def execute_pose(self, pose_name: str) -> bool:
        """Execute a specific pose"""
        if pose_name in self.poses:
            return self.move_robot_arm(self.poses[pose_name], f"Moving to {pose_name}")
        else:
            print(f"❌ Unknown pose: {pose_name}")
            return False
    
    def show_menu(self):
        """Display the control menu"""
        print("\n" + "="*50)
        print("🤖 ROBOT ARM TERMINAL CONTROL")
        print("="*50)
        print("1. 🏠 Home Position (All joints to 0)")
        print("2. 🏗️ Safe Pickup Sequence (Crane-like, avoids collisions)")
        print("3. ⏸️  Waiting Position (Arm up, out of way)")
        print("4. ✋ Open Gripper")
        print("5. 🤏 Close Gripper")
        print("6. 🎛️  Manual Joint Control")
        print("7. 🔄 Show Robot Status")
        print("8. 📋 Show Joint Positions")
        print("9. 🎯 Quick Demo Sequence (SAFE)")
        print("0. ❌ Exit")
        print("="*50)
        print("💡 Note: All movements are sequential for robot safety")
    
    def show_status(self):
        """Show current robot status"""
        try:
            response = requests.get(f'{self.api_url}/api/health', timeout=5)
            if response.ok:
                data = response.json()
                print("\n📊 ROBOT STATUS:")
                print(f"   Status: {data.get('status', 'unknown')}")
                print(f"   WSL2 Connected: {data.get('wsl2_connected', False)}")
                print(f"   Timestamp: {data.get('timestamp', 'unknown')}")
                if 'wsl2_status' in data:
                    wsl2 = data['wsl2_status']
                    print(f"   ROS2 Initialized: {wsl2.get('ros2_initialized', False)}")
                    print(f"   Available Joints: {wsl2.get('available_joints', 0)}")
            else:
                print(f"❌ Failed to get status: {response.status_code}")
        except Exception as e:
            print(f"❌ Status check error: {e}")
    
    def show_joint_positions(self):
        """Show current pose configurations"""
        print("\n📋 AVAILABLE POSES:")
        for pose_name, joints in self.poses.items():
            print(f"\n🎯 {pose_name.upper()}:")
            for joint, value in joints.items():
                print(f"   {joint}: {value}")
    
    def demo_sequence(self):
        """Execute a demonstration sequence with safe, slow movements"""
        print("\n🎬 Starting Demo Sequence...")
        print("⚠️  Using slow, safe movements to protect robot")
        
        # Home -> Safe Pickup with gripper pre-open -> Close gripper -> Waiting -> Open gripper -> Home
        print("\n🤖 Moving to home position...")
        if not self.execute_pose('home'):
            print("❌ Demo sequence interrupted due to error")
            return
        time.sleep(2)
        
        # Use special pickup method that opens gripper before joint4
        if not self.move_to_pickup_position("Moving to pickup position"):
            print("❌ Demo sequence interrupted due to error")
            return
        time.sleep(2)
        
        print("\n🤖 Closing gripper to grab object...")
        if not self.move_robot_arm(self.poses['grip_close'], "Closing gripper", 0.2):
            print("❌ Demo sequence interrupted due to error")
            return
        time.sleep(2)
        
        print("\n🤖 Moving to waiting position...")
        if not self.execute_pose('waiting'):
            print("❌ Demo sequence interrupted due to error")
            return
        time.sleep(2)
        
        print("\n🤖 Opening gripper to release object...")
        if not self.move_robot_arm(self.poses['grip_open'], "Opening gripper", 0.2):
            print("❌ Demo sequence interrupted due to error")
            return
        time.sleep(2)
        
        print("\n🤖 Returning to home position...")
        if not self.execute_pose('home'):
            print("❌ Demo sequence interrupted due to error")
            return
        
        print("\n✅ Demo sequence completed safely with improved pickup!")
    
    def manual_joint_control(self):
        """Manual control of individual joints"""
        print("\n🎛️  MANUAL JOINT CONTROL")
        print("Available joints: joint1, joint2, joint3, joint4, joint5, left_finger, right_finger")
        print("Type 'quit' to return to main menu")
        
        while True:
            try:
                joint_input = input("\n➤ Enter joint name: ").strip().lower()
                
                if joint_input == 'quit':
                    break
                
                if joint_input not in ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'left_finger', 'right_finger']:
                    print("❌ Invalid joint name!")
                    continue
                
                position_input = input(f"➤ Enter position for {joint_input} (radians, or 'cancel'): ").strip()
                
                if position_input.lower() == 'cancel':
                    continue
                
                try:
                    position = float(position_input)
                except ValueError:
                    print("❌ Invalid position! Please enter a number.")
                    continue
                
                # Safety limits
                joint_limits = {
                    'joint1': (-3.14, 3.14),
                    'joint2': (-1.57, 1.57), 
                    'joint3': (-1.57, 1.57),
                    'joint4': (-1.57, 1.57),
                    'joint5': (-3.14, 3.14),
                    'left_finger': (0.0, 0.25),
                    'right_finger': (0.0, 0.25)
                }
                
                if joint_input in joint_limits:
                    min_pos, max_pos = joint_limits[joint_input]
                    if position < min_pos or position > max_pos:
                        print(f"⚠️  Position {position} is outside safe limits ({min_pos} to {max_pos})")
                        confirm = input("Continue anyway? (y/N): ").strip().lower()
                        if confirm != 'y':
                            continue
                
                # Move the joint
                success = self.move_single_joint(joint_input, position)
                if success:
                    print(f"✅ {joint_input} successfully moved to {position}")
                
            except KeyboardInterrupt:
                print("\n⚠️  Returning to main menu...")
                break
            except Exception as e:
                print(f"❌ Error: {e}")
                continue
    
    def run(self):
        """Main control loop"""
        while True:
            try:
                self.show_menu()
                choice = input("\n➤ Enter your choice (0-9): ").strip()
                
                if choice == '0':
                    print("\n👋 Shutting down Robot Terminal Control...")
                    # Return to home before exiting
                    print("🏠 Returning robot to home position...")
                    self.execute_pose('home')
                    break
                
                elif choice == '1':
                    self.execute_pose('home')
                
                elif choice == '2':
                    self.safe_pickup_sequence()
                
                elif choice == '3':
                    self.execute_pose('waiting')
                
                elif choice == '4':
                    self.execute_pose('grip_open')
                
                elif choice == '5':
                    self.execute_pose('grip_close')
                
                elif choice == '6':
                    self.manual_joint_control()
                
                elif choice == '7':
                    self.show_status()
                
                elif choice == '8':
                    self.show_joint_positions()
                
                elif choice == '9':
                    self.demo_sequence()
                
                else:
                    print("❌ Invalid choice! Please enter 0-9.")
                
                # Pause before showing menu again
                input("\nPress Enter to continue...")
                
            except KeyboardInterrupt:
                print("\n\n⚠️  Interrupted by user")
                print("🏠 Returning robot to home position...")
                self.execute_pose('home')
                break
            except Exception as e:
                print(f"\n❌ Unexpected error: {e}")
                continue

def main():
    """Main entry point"""
    print("🚀 Starting Robot Terminal Control Interface...")
    
    # Check for required dependencies
    try:
        import requests
    except ImportError:
        print("❌ Missing required dependency: requests")
        print("Install with: pip install requests")
        return
    
    # Create and run the controller
    try:
        controller = RobotTerminalControl()
        controller.run()
    except KeyboardInterrupt:
        print("\n👋 Goodbye!")
    except Exception as e:
        print(f"❌ Fatal error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
