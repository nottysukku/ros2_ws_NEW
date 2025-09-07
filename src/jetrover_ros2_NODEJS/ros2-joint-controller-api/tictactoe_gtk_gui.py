#!/usr/bin/env python3
"""
Beautiful GTK-based Tic-Tac-Toe Game with Robot Arm Integration
Features:
- Mouse click interface
- Beautiful modern UI with animations
- Robot arm control integration
- Sound effects and visual feedback
- Real-time status updates
"""

import gi
gi.require_version('Gtk', '3.0')
gi.require_version('Gdk', '3.0')
from gi.repository import Gtk, Gdk, GLib, Pango
import random
import os
import requests
import time
import threading
import subprocess
import json
from typing import Optional, Dict, List

class TicTacToeGUI:
    def __init__(self):
        # Game state
        self.board = [' ' for _ in range(9)]
        self.human = 'X'
        self.computer = 'O'
        self.game_over = False
        self.current_player = self.human
        
        # API configuration
        self.api_url = os.getenv('ROBOT_API_URL', 'https://ros2-joint-controller-api.onrender.com')
        
    
        
        # Robot arm positions (updated for calm and composed movement)
        self.tile_positions = {
            1: {'joint1': 0.34, 'joint2': -0.42, 'joint3': 1.52, 'joint4': 0.35, 'joint5': -0.13, 'left_finger': 0.0, 'right_finger': 0.0},
            2: {'joint1': 0.16, 'joint2': -0.42, 'joint3': 1.52, 'joint4': 0.35, 'joint5': -0.14, 'left_finger': 0.0, 'right_finger': 0.0},
            3: {'joint1': -0.12, 'joint2': -0.42, 'joint3': 1.52, 'joint4': 0.35, 'joint5': -0.13, 'left_finger': 0.0, 'right_finger': 0.0},
            4: {'joint1': 0.37, 'joint2': 0.0, 'joint3': 2.03, 'joint4': -1.37, 'joint5': 1.45, 'left_finger': 0.0, 'right_finger': 0.0},
            5: {'joint1': 0.12, 'joint2': 0.0, 'joint3': 2.03, 'joint4': -1.36, 'joint5': 1.44, 'left_finger': 0.0, 'right_finger': 0.0},
            6: {'joint1': -0.12, 'joint2': -0.42, 'joint3': 1.52, 'joint4': -0.69, 'joint5': -0.13, 'left_finger': 0.0, 'right_finger': 0.0},
            7: {'joint1': 0.40, 'joint2': 0.92, 'joint3': 0.65, 'joint4': -1.72, 'joint5': 1.44, 'left_finger': 0.0, 'right_finger': 0.0},
            8: {'joint1': 0.12, 'joint2': 0.92, 'joint3': 0.65, 'joint4': -1.72, 'joint5': 1.48, 'left_finger': 0.0, 'right_finger': 0.0},
            9: {'joint1': -0.15, 'joint2': 0.92, 'joint3': 0.65, 'joint4': -1.74, 'joint5': 1.76, 'left_finger': 0.0, 'right_finger': 0.0}
        }
        
        # Gazebo world coordinates for each tile (spawn positions for white balls - human player only)
        self.tile_world_positions = {
            1: {'x': -0.01, 'y': 0.10, 'z': 0.47},
            2: {'x': 0.04, 'y': 0.10, 'z': 0.47},
            3: {'x': 0.09, 'y': 0.10, 'z': 0.47},
            4: {'x': -0.01, 'y': 0.05, 'z': 0.47},
            5: {'x': 0.04, 'y': 0.06, 'z': 0.47},
            6: {'x': 0.09, 'y': 0.06, 'z': 0.47},
            7: {'x': -0.01, 'y': 0.00, 'z': 0.47},
            8: {'x': 0.04, 'y': 0.00, 'z': 0.47},
            9: {'x': 0.09, 'y': 0.00, 'z': 0.47}
        }
        
        # Available game pieces in the world
        self.available_x_pieces = ['x_piece_1', 'x_piece_2', 'x_piece_4', 'x_piece_5']
        self.available_o_pieces = ['o_piece_1', 'o_piece_2', 'o_piece_4', 'o_piece_5']
        self.used_x_pieces = []
        self.used_o_pieces = []
        
        self.poses = {
            'home': {'joint1': 0.0, 'joint2': 0.0, 'joint3': 0.0, 'joint4': 0.0, 'joint5': 0.0, 'left_finger': 0.0, 'right_finger': 0.0},
            'crane_high': {'joint1': 0.59, 'joint2': 0.20, 'joint3': 0.40, 'joint4': 0.60, 'joint5': 0.05, 'left_finger': 0.15, 'right_finger': 0.15},
            'pickup_approach': {'joint1': 0.59, 'joint2': 0.60, 'joint3': 0.20, 'joint4': 0.80, 'joint5': 0.05, 'left_finger': 0.15, 'right_finger': 0.15},
            'pickup': {'joint1': 0.59, 'joint2': 0.85, 'joint3': -0.75, 'joint4': 1.10, 'joint5': 0.05, 'left_finger': 0.15, 'right_finger': 0.15},
            'carry_high': {'joint1': 0.20, 'joint2': -0.30, 'joint3': 0.80, 'joint4': 0.60, 'joint5': 0.05, 'left_finger': 0.0, 'right_finger': 0.0},
            'transit': {'joint1': 0.10, 'joint2': -0.50, 'joint3': 1.20, 'joint4': 0.40, 'joint5': 0.00, 'left_finger': 0.0, 'right_finger': 0.0},
            'pre_place': {'joint1': 0.12, 'joint2': -0.30, 'joint3': 1.40, 'joint4': 0.20, 'joint5': -0.10, 'left_finger': 0.0, 'right_finger': 0.0},
            'waiting': {'joint1': 0.0, 'joint2': -0.80, 'joint3': 1.40, 'joint4': 0.20, 'joint5': 0.0, 'left_finger': 0.0, 'right_finger': 0.0},
            'safe_retreat': {'joint1': 0.0, 'joint2': -0.60, 'joint3': 1.00, 'joint4': 0.50, 'joint5': 0.0, 'left_finger': 0.0, 'right_finger': 0.0},
            'grip_open': {'left_finger': 0.20, 'right_finger': 0.20},
            'grip_close': {'left_finger': 0.0, 'right_finger': 0.0}
        }
        
        # Initialize GUI
        self.create_gui()
        
        # Reset pieces to original positions at startup
        self.reset_pieces_to_original_positions()
        
        # Initialize robot arm
        self.initialize_robot()
    
    def create_gui(self):
        """Create the main GUI interface"""
        # Main window
        self.window = Gtk.Window(title="🤖 Robot Arm Tic-Tac-Toe")
        self.window.set_default_size(600, 700)
        self.window.set_resizable(False)
        self.window.connect("destroy", Gtk.main_quit)
        
        # Set window icon and styling
        self.window.set_position(Gtk.WindowPosition.CENTER)
        
        # Main container
        main_vbox = Gtk.VBox(spacing=20)
        main_vbox.set_margin_top(20)
        main_vbox.set_margin_bottom(20)
        main_vbox.set_margin_start(20)
        main_vbox.set_margin_end(20)
        self.window.add(main_vbox)
        
        # Title
        title_label = Gtk.Label()
        title_label.set_markup('<span font="20" weight="bold" color="#2E3440">🤖 Robot Arm Tic-Tac-Toe</span>')
        title_label.set_margin_bottom(10)
        main_vbox.pack_start(title_label, False, False, 0)
        
        # Status bar
        self.status_label = Gtk.Label()
        self.status_label.set_markup('<span font="12" color="#5E81AC">Welcome! Click to start playing</span>')
        self.status_label.set_margin_bottom(10)
        main_vbox.pack_start(self.status_label, False, False, 0)
        
        # Game board container
        board_frame = Gtk.Frame()
        board_frame.set_shadow_type(Gtk.ShadowType.ETCHED_IN)
        board_frame.set_margin_bottom(20)
        main_vbox.pack_start(board_frame, True, True, 0)
        
        # Game board grid
        self.board_grid = Gtk.Grid()
        self.board_grid.set_row_homogeneous(True)
        self.board_grid.set_column_homogeneous(True)
        self.board_grid.set_row_spacing(3)
        self.board_grid.set_column_spacing(3)
        self.board_grid.set_margin_top(10)
        self.board_grid.set_margin_bottom(10)
        self.board_grid.set_margin_start(10)
        self.board_grid.set_margin_end(10)
        board_frame.add(self.board_grid)
        
        # Create board buttons
        self.buttons = []
        for i in range(9):
            button = Gtk.Button()
            button.set_size_request(120, 120)
            button.connect("clicked", self.on_button_clicked, i)
            
            # Create label for button content
            label = Gtk.Label()
            label.set_markup('<span font="36" color="#4C566A">·</span>')
            button.add(label)
            
            # Style the button
            button.set_name(f"tile-button-{i}")
            self.style_button(button, i)
            
            row = i // 3
            col = i % 3
            self.board_grid.attach(button, col, row, 1, 1)
            self.buttons.append(button)
        
        # Robot status section
        robot_frame = Gtk.Frame(label="🤖 Robot Status")
        robot_frame.set_margin_bottom(10)
        main_vbox.pack_start(robot_frame, False, False, 0)
        
        robot_vbox = Gtk.VBox(spacing=5)
        robot_vbox.set_margin_top(10)
        robot_vbox.set_margin_bottom(10)
        robot_vbox.set_margin_start(10)
        robot_vbox.set_margin_end(10)
        robot_frame.add(robot_vbox)
        
        self.robot_status_label = Gtk.Label()
        self.robot_status_label.set_markup('<span color="#D08770">🔗 Initializing robot connection...</span>')
        robot_vbox.pack_start(self.robot_status_label, False, False, 0)
        
        self.robot_action_label = Gtk.Label()
        self.robot_action_label.set_markup('<span color="#88C0D0">🏠 Robot at home position</span>')
        robot_vbox.pack_start(self.robot_action_label, False, False, 0)
        
        # Control buttons
        button_hbox = Gtk.HBox(spacing=10)
        main_vbox.pack_start(button_hbox, False, False, 0)
        
        self.new_game_button = Gtk.Button(label="🆕 New Game")
        self.new_game_button.connect("clicked", self.new_game)
        self.new_game_button.set_size_request(120, 40)
        button_hbox.pack_start(self.new_game_button, True, True, 0)
        
        self.reset_arm_button = Gtk.Button(label="🏠 Reset Arm")
        self.reset_arm_button.connect("clicked", self.reset_arm_position)
        self.reset_arm_button.set_size_request(120, 40)
        button_hbox.pack_start(self.reset_arm_button, True, True, 0)
        
        self.quit_button = Gtk.Button(label="❌ Quit")
        self.quit_button.connect("clicked", self.on_quit)
        self.quit_button.set_size_request(120, 40)
        button_hbox.pack_start(self.quit_button, True, True, 0)
        
        # Score tracking
        score_frame = Gtk.Frame(label="📊 Score")
        main_vbox.pack_start(score_frame, False, False, 0)
        
        score_hbox = Gtk.HBox(spacing=20)
        score_hbox.set_margin_top(10)
        score_hbox.set_margin_bottom(10)
        score_hbox.set_margin_start(10)
        score_hbox.set_margin_end(10)
        score_frame.add(score_hbox)
        
        self.human_score = 0
        self.computer_score = 0
        
        self.human_score_label = Gtk.Label()
        self.human_score_label.set_markup(f'<span font="14" color="#A3BE8C">👤 You: {self.human_score}</span>')
        score_hbox.pack_start(self.human_score_label, True, True, 0)
        
        self.computer_score_label = Gtk.Label()
        self.computer_score_label.set_markup(f'<span font="14" color="#BF616A">🤖 Robot: {self.computer_score}</span>')
        score_hbox.pack_start(self.computer_score_label, True, True, 0)
        
        # Apply CSS styling
        self.apply_css_styles()
    
    def style_button(self, button, index):
        """Apply custom styling to board buttons"""
        css_provider = Gtk.CssProvider()
        css = """
        button {
            border-radius: 10px;
            border: 2px solid #4C566A;
            background: #ECEFF4;
            font-size: 48px;
            font-weight: bold;
        }
        button:hover {
            background: #D8DEE9;
            border-color: #5E81AC;
        }
        button:active {
            background: #D8DEE9;
        }
        """
        css_provider.load_from_data(css.encode())
        button.get_style_context().add_provider(css_provider, Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION)
    
    def apply_css_styles(self):
        """Apply global CSS styles"""
        css_provider = Gtk.CssProvider()
        css = """
        window {
            background: #ECEFF4;
        }
        frame {
            border-radius: 8px;
            border: 1px solid #D8DEE9;
            background: rgba(255, 255, 255, 0.8);
        }
        """
        css_provider.load_from_data(css.encode())
        Gtk.StyleContext.add_provider_for_screen(
            Gdk.Screen.get_default(),
            css_provider,
            Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION
        )
    
    def update_status(self, message: str, color: str = "#2E3440"):
        """Update the status message with color"""
        GLib.idle_add(self._update_status_safe, message, color)
    
    def _update_status_safe(self, message: str, color: str):
        """Thread-safe status update"""
        self.status_label.set_markup(f'<span font="12" color="{color}">{message}</span>')
        return False
    
    def update_robot_status(self, message: str, color: str = "#D08770"):
        """Update robot status with color"""
        GLib.idle_add(self._update_robot_status_safe, message, color)
    
    def _update_robot_status_safe(self, message: str, color: str):
        """Thread-safe robot status update"""
        self.robot_status_label.set_markup(f'<span color="{color}">{message}</span>')
        return False
    
    def update_robot_action(self, message: str, color: str = "#88C0D0"):
        """Update robot action with color"""
        GLib.idle_add(self._update_robot_action_safe, message, color)
    
    def _update_robot_action_safe(self, message: str, color: str):
        """Thread-safe robot action update"""
        self.robot_action_label.set_markup(f'<span color="{color}">{message}</span>')
        return False
    
    def initialize_robot(self):
        """Initialize robot arm in background thread"""
        def init_robot_thread():
            self.update_robot_status("🔗 Connecting to robot...", "#D08770")
            try:
                response = requests.get(f'{self.api_url}/health', timeout=5)
                if response.ok:
                    self.update_robot_status("✅ Robot connected successfully", "#A3BE8C")
                    
                    # Move to home position
                    self.update_robot_action("🏠 Moving to home position...", "#88C0D0")
                    success = self.move_robot_arm(self.poses['home'], "Initializing")
                    if success:
                        self.update_robot_action("🏠 Robot at home position", "#A3BE8C")
                    else:
                        self.update_robot_action("⚠️ Robot movement failed", "#BF616A")
                else:
                    self.update_robot_status("❌ Robot API connection failed", "#BF616A")
            except Exception as e:
                self.update_robot_status(f"❌ Robot connection error: {str(e)[:30]}...", "#BF616A")
        
        # Run in background thread to avoid blocking GUI
        threading.Thread(target=init_robot_thread, daemon=True).start()
    
    def move_robot_arm(self, joint_positions: Dict, description: str = "Moving arm") -> bool:
        """Send joint commands to robot arm via API - calm and composed sequential movement"""
        try:
            self.update_robot_action(f"🤖 {description}...", "#D08770")
            
            # Define joint order for sequential movement
            joint_order = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'left_finger', 'right_finger']
            
            # Move each joint individually with calm delays for smooth operation
            for joint_name in joint_order:
                if joint_name in joint_positions:
                    position = joint_positions[joint_name]
                    
                    # Send individual joint command
                    response = requests.post(
                        f'{self.api_url}/api/joint/{joint_name}/move', 
                        json={'position': position}, 
                        timeout=10
                    )
                    
                    if not response.ok:
                        self.update_robot_action(f"❌ {joint_name} move failed", "#BF616A")
                        return False
                    
                    # Calm, composed timing between joint movements
                    if joint_name in ['joint1', 'joint2', 'joint3']:
                        time.sleep(0.8)  # Slower for major joints
                    elif joint_name in ['joint4', 'joint5']:
                        time.sleep(0.6)  # Medium speed for positioning
                    else:
                        time.sleep(0.4)  # Faster for gripper
            
            self.update_robot_action(f"✅ {description} completed smoothly", "#A3BE8C")
            return True
            
        except requests.exceptions.RequestException as e:
            self.update_robot_action(f"❌ Robot API error", "#BF616A")
            return False
    
    def open_gripper(self, description: str = "Opening gripper") -> bool:
        """Open the gripper fingers"""
        return self.move_robot_arm(self.poses['grip_open'], description)
    
    def close_gripper(self, description: str = "Closing gripper") -> bool:
        """Close the gripper fingers"""
        return self.move_robot_arm(self.poses['grip_close'], description)
    
    def move_to_pickup_position(self, description: str = "Moving to pickup position") -> bool:
        """Crane-like pickup sequence: move to high position above pickup → open gripper → descend → position precisely"""
        try:
            self.update_robot_action(f"🤖 {description}...", "#D08770")
            
            # Step 1: Move to crane high position (directly above pickup area with gripper closed)
            crane_high_pose = self.poses['crane_high']
            success = self.move_robot_arm(crane_high_pose, "Moving to crane high position")
            if not success:
                self.update_robot_action("❌ Failed to reach crane high position", "#BF616A")
                return False
            time.sleep(1.0)  # Stable positioning above
            
            # Step 2: Open gripper while at high position (crane preparation)
            success = self.open_gripper("Opening gripper at crane position")
            if not success:
                self.update_robot_action("❌ Failed to open gripper", "#BF616A")
                return False
            time.sleep(0.8)  # Gripper stabilization
            
            # Step 3: Descend to intermediate approach position (crane lowering)
            approach_pose = self.poses['pickup_approach']
            success = self.move_robot_arm(approach_pose, "Descending to approach position")
            if not success:
                self.update_robot_action("❌ Failed to reach approach position", "#BF616A")
                return False
            time.sleep(1.0)  # Careful descent
            
            # Step 4: Final precise descent to pickup position (crane landing)
            pickup_pose = self.poses['pickup']
            
            # Move joints 2 and 3 first for the final descent (crane lowering motion)
            descent_joints = ['joint2', 'joint3']
            for joint_name in descent_joints:
                if joint_name in pickup_pose:
                    position = pickup_pose[joint_name]
                    
                    response = requests.post(
                        f'{self.api_url}/api/joint/{joint_name}/move', 
                        json={'position': position}, 
                        timeout=10
                    )
                    
                    if not response.ok:
                        self.update_robot_action(f"❌ {joint_name} descent failed", "#BF616A")
                        return False
                    
                    time.sleep(0.8)  # Slow, controlled descent like a crane
            
            # Step 5: Fine-tune joint4 for final positioning
            if 'joint4' in pickup_pose:
                position = pickup_pose['joint4']
                
                response = requests.post(
                    f'{self.api_url}/api/joint/joint4/move', 
                    json={'position': position}, 
                    timeout=10
                )
                
                if not response.ok:
                    self.update_robot_action("❌ joint4 positioning failed", "#BF616A")
                    return False
                
                time.sleep(0.6)  # Precise final positioning
            
            # Step 6: Final wrist adjustment (joint5) for optimal grip angle
            if 'joint5' in pickup_pose:
                position = pickup_pose['joint5']
                
                response = requests.post(
                    f'{self.api_url}/api/joint/joint5/move', 
                    json={'position': position}, 
                    timeout=10
                )
                
                if not response.ok:
                    self.update_robot_action("❌ joint5 positioning failed", "#BF616A")
                    return False
                
                time.sleep(0.5)  # Final wrist adjustment
            
            self.update_robot_action(f"✅ {description} completed - crane positioned for pickup", "#A3BE8C")
            return True
            
        except requests.exceptions.RequestException as e:
            self.update_robot_action(f"❌ Robot API error", "#BF616A")
            return False
    
    def teleport_game_piece(self, tile_number: int, piece_type: str = "x_piece"):
        """Teleport an existing game piece to specified tile position"""
        def teleport_thread():
            try:
                # Select available piece
                if piece_type == "x_piece":
                    if not self.available_x_pieces:
                        print("❌ No more X pieces available!")
                        return False
                    piece_name = self.available_x_pieces.pop(0)
                    self.used_x_pieces.append(piece_name)
                    piece_symbol = "X"
                    color_desc = "white"
                else:
                    if not self.available_o_pieces:
                        print("❌ No more O pieces available!")
                        return False
                    piece_name = self.available_o_pieces.pop(0)
                    self.used_o_pieces.append(piece_name)
                    piece_symbol = "O" 
                    color_desc = "black"
                
                # Get tile world coordinates
                pos = self.tile_world_positions[tile_number]
                
                # Teleport the piece using gz service
                pose_cmd = f"{pos['x']} {pos['y']} {pos['z']} 0 0 0"
                cmd = f'gz service -s /world/WORKINGPROTO/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 1000 --req \'name: "{piece_name}" position {{x: {pos["x"]} y: {pos["y"]} z: {pos["z"]}}} orientation {{x: 0 y: 0 z: 0 w: 1}}\''
                
                result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
                
                if result.returncode == 0:
                    print(f"✅ Teleported {color_desc} {piece_symbol} piece ({piece_name}) to tile {tile_number}")
                    return True
                else:
                    print(f"❌ Failed to teleport {piece_symbol} piece: {result.stderr}")
                    # Return piece to available list if teleport failed
                    if piece_type == "x_piece":
                        self.used_x_pieces.remove(piece_name)
                        self.available_x_pieces.insert(0, piece_name)
                    else:
                        self.used_o_pieces.remove(piece_name)
                        self.available_o_pieces.insert(0, piece_name)
                    return False
                    
            except Exception as e:
                print(f"❌ Gazebo teleport error: {e}")
                return False
        
        # Run in background thread
        threading.Thread(target=teleport_thread, daemon=True).start()
    
    def teleport_human_piece(self, tile_number: int):
        """Teleport white ball (X piece) for human player"""
        self.teleport_game_piece(tile_number, "x_piece")
        self.update_status(f"⚪ White piece teleported to tile {tile_number}!", "#A3BE8C")
    
    def teleport_computer_piece(self, tile_number: int):
        """Teleport black ball (O piece) for computer player"""
        self.teleport_game_piece(tile_number, "o_piece")
        self.update_robot_action(f"⚫ Black piece moved to tile {tile_number}!", "#BF616A")
    
    def robot_sequence_to_tile(self, tile_number: int):
        """Simplified robot sequence: move directly to tile position → teleport black ball"""
        def robot_sequence_thread():
            try:
                self.update_robot_action(f"🤖 Computer moving to tile {tile_number}...", "#D08770")
                
                # Step 1: Move directly to target tile position (top of tile)
                target_position = self.tile_positions[tile_number]
                success = self.move_robot_arm(target_position, f"Moving directly to tile {tile_number}")
                if not success:
                    self.update_robot_action(f"❌ Failed to reach tile {tile_number}", "#BF616A")
                    return
                time.sleep(1.0)  # Brief pause for positioning
                
                # Step 2: Immediately teleport black ball to tile
                self.update_robot_action(f"✅ Computer positioned at tile {tile_number}", "#A3BE8C")
                self.teleport_computer_piece(tile_number)
                    
            except Exception as e:
                self.update_robot_action(f"❌ Robot sequence error: {str(e)[:30]}...", "#BF616A")
        
        # Run in background thread
        threading.Thread(target=robot_sequence_thread, daemon=True).start()
    
    def human_turn_start(self):
        """Move arm to waiting position for human turn"""
        def human_turn_thread():
            self.update_robot_action("👤 Your turn - moving arm to waiting position...", "#88C0D0")
            success = self.move_robot_arm(self.poses['waiting'], "Moving arm up for human turn")
            if success:
                self.update_robot_action("👤 Your turn - arm in waiting position", "#A3BE8C")
            else:
                self.update_robot_action("⚠️ Arm movement failed", "#BF616A")
        
        threading.Thread(target=human_turn_thread, daemon=True).start()
    
    def on_button_clicked(self, button, position):
        """Handle button click events"""
        if self.game_over or self.board[position] != ' ' or self.current_player != self.human:
            return
        
        # Make human move
        self.make_move(position, self.human)
        self.update_button_display(position, self.human)
        self.update_status(f"👤 You placed X on tile {position + 1}", "#A3BE8C")
        
        # Spawn white ball immediately when human clicks
        self.teleport_human_piece(position + 1)
        
        # Check for winner after human move
        winner = self.check_winner()
        if winner:
            self.end_game(winner)
            return
        
        if self.is_board_full():
            self.end_game(None)  # Tie
            return
        
        # Switch to computer turn
        self.current_player = self.computer
        self.update_status("🤖 Computer is thinking...", "#D08770")
        self.disable_buttons()
        
        # Execute computer move after a brief delay
        GLib.timeout_add(1000, self.execute_computer_move)
    
    def execute_computer_move(self):
        """Execute computer move and robot sequence"""
        def computer_move_thread():
            # Calculate best move
            best_move = self.get_best_move()
            if best_move is not None:
                # Make the move
                self.make_move(best_move, self.computer)
                
                # Update UI in main thread
                GLib.idle_add(self.update_button_display, best_move, self.computer)
                GLib.idle_add(self.update_status, f"🤖 Computer chose tile {best_move + 1}", "#5E81AC")
                
                # Execute robot sequence for computer move
                self.robot_sequence_to_tile(best_move + 1)
                
                # Check for winner after computer move
                winner = self.check_winner()
                if winner:
                    GLib.idle_add(self.end_game, winner)
                    return
                
                if self.is_board_full():
                    GLib.idle_add(self.end_game, None)
                    return
                
                # Back to human turn after robot finishes moving
                GLib.timeout_add(8500, self.prepare_human_turn)  # Extended time for enhanced sequence (pickup + carry + transit + pre-place + place + retreat + waiting)
        
        # Run computer move in background thread
        threading.Thread(target=computer_move_thread, daemon=True).start()
        return False  # Don't repeat timeout
    
    def prepare_human_turn(self):
        """Prepare for human turn after computer move"""
        self.current_player = self.human
        self.enable_buttons()
        self.update_status("👤 Your turn! Click a tile", "#A3BE8C")
        
        # Move arm to waiting position for human turn
        self.human_turn_start()
        return False  # Don't repeat timeout
    
    def update_button_display(self, position: int, player: str):
        """Update button appearance after move"""
        button = self.buttons[position]
        label = button.get_child()
        
        if player == self.human:
            label.set_markup('<span font="36" color="#A3BE8C" weight="bold">✗</span>')
            # Add green background for human moves
            self.add_button_color(button, "#A3BE8C")
        else:
            label.set_markup('<span font="36" color="#BF616A" weight="bold">⭕</span>')
            # Add red background for computer moves
            self.add_button_color(button, "#BF616A")
        
        button.set_sensitive(False)
    
    def add_button_color(self, button, color):
        """Add background color to button"""
        css_provider = Gtk.CssProvider()
        css = f"""
        button {{
            background: rgba(163, 190, 140, 0.3);
            border-color: {color};
            border-width: 3px;
        }}
        """
        css_provider.load_from_data(css.encode())
        button.get_style_context().add_provider(css_provider, Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION)
    
    def disable_buttons(self):
        """Disable all board buttons"""
        for button in self.buttons:
            if self.board[self.buttons.index(button)] == ' ':
                button.set_sensitive(False)
    
    def enable_buttons(self):
        """Enable empty board buttons"""
        for i, button in enumerate(self.buttons):
            if self.board[i] == ' ':
                button.set_sensitive(True)
    
    def make_move(self, position: int, player: str) -> bool:
        """Make a move on the board"""
        if self.is_valid_move(position):
            self.board[position] = player
            return True
        return False
    
    def is_valid_move(self, position: int) -> bool:
        """Check if the move is valid"""
        return 0 <= position <= 8 and self.board[position] == ' '
    
    def check_winner(self) -> Optional[str]:
        """Check if there's a winner"""
        win_patterns = [
            [0, 1, 2], [3, 4, 5], [6, 7, 8],  # Rows
            [0, 3, 6], [1, 4, 7], [2, 5, 8],  # Columns
            [0, 4, 8], [2, 4, 6]              # Diagonals
        ]
        
        for pattern in win_patterns:
            if (self.board[pattern[0]] == self.board[pattern[1]] == 
                self.board[pattern[2]] != ' '):
                return self.board[pattern[0]]
        return None
    
    def is_board_full(self) -> bool:
        """Check if the board is full"""
        return ' ' not in self.board
    
    def get_empty_positions(self) -> List[int]:
        """Get list of empty positions"""
        return [i for i, spot in enumerate(self.board) if spot == ' ']
    
    def minimax(self, is_maximizing: bool, depth: int = 0) -> int:
        """Minimax algorithm for AI decision making"""
        winner = self.check_winner()
        
        if winner == self.computer:
            return 10 - depth
        elif winner == self.human:
            return depth - 10
        elif self.is_board_full():
            return 0
        
        if is_maximizing:
            best_score = float('-inf')
            for pos in self.get_empty_positions():
                self.board[pos] = self.computer
                score = self.minimax(False, depth + 1)
                self.board[pos] = ' '
                best_score = max(score, best_score)
            return best_score
        else:
            best_score = float('inf')
            for pos in self.get_empty_positions():
                self.board[pos] = self.human
                score = self.minimax(True, depth + 1)
                self.board[pos] = ' '
                best_score = min(score, best_score)
            return best_score
    
    def get_best_move(self) -> Optional[int]:
        """Get the best move for computer using minimax"""
        best_score = float('-inf')
        best_move = None
        
        for pos in self.get_empty_positions():
            self.board[pos] = self.computer
            score = self.minimax(False)
            self.board[pos] = ' '
            
            if score > best_score:
                best_score = score
                best_move = pos
        
        return best_move
    
    def end_game(self, winner: Optional[str]):
        """Handle game end"""
        self.game_over = True
        self.disable_buttons()
        
        if winner == self.human:
            self.human_score += 1
            self.update_status("🎉 Congratulations! You won!", "#A3BE8C")
            self.update_robot_action("🎉 Victory celebration!", "#A3BE8C")
            # Victory celebration
            threading.Thread(target=lambda: self.move_robot_arm(self.poses['waiting'], "Victory celebration"), daemon=True).start()
        elif winner == self.computer:
            self.computer_score += 1
            self.update_status("💻 Computer wins! Better luck next time!", "#BF616A")
            self.update_robot_action("🤖 Computer celebration!", "#BF616A")
            # Computer celebration
            threading.Thread(target=lambda: self.move_robot_arm(self.poses['home'], "Computer celebration"), daemon=True).start()
        else:
            self.update_status("🤝 It's a tie! Good game!", "#5E81AC")
            self.update_robot_action("🤝 Tie game - returning home", "#88C0D0")
            threading.Thread(target=lambda: self.move_robot_arm(self.poses['home'], "Game ended"), daemon=True).start()
        
        # Update scores
        self.human_score_label.set_markup(f'<span font="14" color="#A3BE8C">👤 You: {self.human_score}</span>')
        self.computer_score_label.set_markup(f'<span font="14" color="#BF616A">🤖 Robot: {self.computer_score}</span>')
        
        # Show play again dialog after a delay
        GLib.timeout_add(3000, self.show_play_again_dialog)
    
    def show_play_again_dialog(self):
        """Show play again dialog"""
        dialog = Gtk.MessageDialog(
            transient_for=self.window,
            modal=True,
            message_type=Gtk.MessageType.QUESTION,
            buttons=Gtk.ButtonsType.YES_NO,
            text="🎮 Play Again?"
        )
        dialog.format_secondary_text("Would you like to start a new game?")
        
        response = dialog.run()
        dialog.destroy()
        
        if response == Gtk.ResponseType.YES:
            self.new_game(None)
        
        return False  # Don't repeat timeout
    
    def reset_pieces_to_original_positions(self):
        """Reset all used pieces back to their original positions"""
        def reset_pieces_thread():
            try:
                # Original positions from the SDF file
                original_positions = {
                    'x_piece_1': {'x': 0.197454, 'y': -0.026346, 'z': 0.483760},
                    'x_piece_2': {'x': 0.175725, 'y': -0.053232, 'z': 0.478285},
                    'x_piece_4': {'x': 0.219260, 'y': 0.000635, 'z': 0.488400},
                    'x_piece_5': {'x': 0.153513, 'y': -0.079791, 'z': 0.473160},
                    'o_piece_1': {'x': -0.071335, 'y': 0.05269, 'z': 0.494283},
                    'o_piece_2': {'x': -0.077689, 'y': 0.128588, 'z': 0.493151},
                    'o_piece_4': {'x': -0.076757, 'y': 0.086961, 'z': 0.489135},
                    'o_piece_5': {'x': -0.072095, 'y': 0.150449, 'z': 0.497005}
                }
                
                # Reset all used pieces
                all_used_pieces = self.used_x_pieces + self.used_o_pieces
                
                for piece_name in all_used_pieces:
                    if piece_name in original_positions:
                        pos = original_positions[piece_name]
                        cmd = f'gz service -s /world/WORKINGPROTO/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 1000 --req \'name: "{piece_name}" position {{x: {pos["x"]} y: {pos["y"]} z: {pos["z"]}}} orientation {{x: 0 y: 0 z: 0 w: 1}}\''
                        
                        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
                        if result.returncode == 0:
                            print(f"✅ Reset {piece_name} to original position")
                        else:
                            print(f"⚠️ Failed to reset {piece_name}: {result.stderr}")
                        
                        time.sleep(0.1)  # Small delay between resets
                
                # Reset piece availability
                self.available_x_pieces = ['x_piece_1', 'x_piece_2', 'x_piece_4', 'x_piece_5']
                self.available_o_pieces = ['o_piece_1', 'o_piece_2', 'o_piece_4', 'o_piece_5']
                self.used_x_pieces = []
                self.used_o_pieces = []
                
                print("✅ All pieces reset to original positions")
                
            except Exception as e:
                print(f"❌ Piece reset error: {e}")
        
        # Run in background thread
        threading.Thread(target=reset_pieces_thread, daemon=True).start()
    
    def new_game(self, widget):
        """Start a new game"""
        # Reset pieces to original positions first
        self.reset_pieces_to_original_positions()
        
        self.board = [' ' for _ in range(9)]
        self.game_over = False
        self.current_player = self.human
        
        # Reset buttons
        for i, button in enumerate(self.buttons):
            label = button.get_child()
            label.set_markup('<span font="36" color="#4C566A">·</span>')
            button.set_sensitive(True)
            self.style_button(button, i)
        
        self.update_status("👤 New game started! Click a tile to begin", "#A3BE8C")
        
        # Move arm to waiting position for human start
        self.human_turn_start()
    
    def reset_arm_position(self, widget):
        """Reset robot arm to home position"""
        def reset_thread():
            self.update_robot_action("🏠 Resetting arm to home position...", "#88C0D0")
            success = self.move_robot_arm(self.poses['home'], "Resetting to home")
            if success:
                self.update_robot_action("🏠 Robot at home position", "#A3BE8C")
            else:
                self.update_robot_action("⚠️ Reset failed", "#BF616A")
        
        threading.Thread(target=reset_thread, daemon=True).start()
    
    def on_quit(self, widget):
        """Handle quit button"""
        # Return arm to home before quitting
        def quit_sequence():
            self.update_robot_action("🏠 Returning arm to home before exit...", "#88C0D0")
            self.move_robot_arm(self.poses['home'], "Final return to home")
            GLib.idle_add(Gtk.main_quit)
        
        threading.Thread(target=quit_sequence, daemon=True).start()
    
    def run(self):
        """Start the GUI application"""
        self.window.show_all()
        
        # Start with human turn
        self.update_status("👤 Your turn! Click any tile to start", "#A3BE8C")
        
        # Start GTK main loop
        Gtk.main()

def main():
    """Main entry point"""
    print("🚀 Starting Robot Arm Tic-Tac-Toe GUI...")
    
    # Check for required dependencies
    try:
        import gi
        gi.require_version('Gtk', '3.0')
        from gi.repository import Gtk
    except ImportError:
        print("❌ GTK3 not found. Please install python3-gi:")
        print("sudo apt update && sudo apt install python3-gi python3-gi-cairo gir1.2-gtk-3.0")
        return
    
    # Create and run the game
    game = TicTacToeGUI()
    
    try:
        game.run()
    except KeyboardInterrupt:
        print("\n👋 Goodbye!")
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    main()
