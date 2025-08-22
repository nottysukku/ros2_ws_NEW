#!/bin/bash

# Enhanced trajectory script for SCALED-DOWN arm (0.7x scale)
# Adjusted joint values for black ball pickup with scaled arm dimensions
# Fixes positioning errors + compensates for 30% smaller arm components

echo "Starting SCALED robot arm movement test (0.7x scale) for tictactoe_NEW3.3.sdf..."

# Function to send joint command
send_joint_command() {
    local joint_name=$1
    local position=$2
    local topic="/model/jetrover/joint/${joint_name}/0/cmd_pos"
    
    echo "→ Moving $joint_name to $position"
    gz topic -t "$topic" -m gz.msgs.Double -p "data: $position"
}

# Function to move gripper
move_gripper() {
    local grip_pos=$1
    
    echo "Moving gripper - Opening to: $grip_pos"
    gz topic -t "/model/jetrover/joint/left_finger_joint/0/cmd_pos" -m gz.msgs.Double -p "data: $grip_pos"
    gz topic -t "/model/jetrover/joint/right_finger_joint/0/cmd_pos" -m gz.msgs.Double -p "data: $grip_pos"
    sleep 0.5
}

# Function to move all joints with synchronized timing
move_all_joints_smooth() {
    local j1=$1 j2=$2 j3=$3 j4=$4 j5=$5 j6=$6 j7=$7
    local duration=${8:-3}
    
    echo "🎯 Target: J1=$j1, J2=$j2, J3=$j3, J4=$j4, J5=$j5, Fingers=$j6"
    
    # Send all commands simultaneously for synchronized movement
    send_joint_command "joint1" "$j1" &
    send_joint_command "joint2" "$j2" &
    send_joint_command "joint3" "$j3" & 
    send_joint_command "joint4" "$j4" &
    send_joint_command "joint5" "$j5" &
    send_joint_command "left_finger_joint" "$j6" &
    send_joint_command "right_finger_joint" "$j7" &
    
    wait  # Wait for all commands to be sent
    
    echo "⏱️  Executing movement (${duration}s)..."
    sleep "$duration"
    echo "✅ Movement complete"
    echo ""
}

# Enhanced trajectory with adjusted values for scaled arm and positioning corrections
execute_scaled_trajectory() {
    echo "🚀 EXECUTING SCALED ARM TRAJECTORY (0.7x scale with corrections)"
    echo "=============================================================="
    echo "🔧 SCALING COMPENSATION: All joint angles increased ~40% for reach"
    echo "🎯 POSITION CORRECTIONS: Ball +3cm closer, Board +7cm forward"
    echo ""
    
    # Adjusted waypoints for scaled arm (0.7x) with position corrections
    # Formula: Increase joint angles by ~40% to compensate for 30% smaller arm
    
    # Original approach: (0, -0.23, 1.24, 0.17, -0.30, 0.15, 0.15)
    # Scaled compensation: More aggressive angles for same reach
    local approach=(0.05 -0.35 1.65 0.25 -0.42 0.10 0.10)
    
    # Original pickup: (-0.03, -0.11, 1.28, 0.12, -0.04, 0.00, 0.00)  
    # Scaled + position correction: Closer to ball + more extension
    local pickup=(0.02 -0.18 1.70 0.18 -0.08 0.00 0.00)
    
    # Lift position - slight adjustment from pickup
    local lift=(0.02 -0.15 1.70 0.18 -0.08 0.00 0.00)
    
    # Original place: (0.04, -0.71, 1.94, 0.73, 0.27, 0.15, 0.15)
    # Scaled + Y-axis correction: Much more extension needed
    local place=(0.08 -1.00 2.50 1.00 0.40 0.10 0.10)
    
    # Safe retract position
    local retract=(0.08 -0.90 2.50 1.00 0.40 0.10 0.10)
    
    # Home position
    local home=(0.0 0.0 0.0 0.0 0.0 0.0 0.0)
    
    echo "📍 WAYPOINT 1: HOME → APPROACH POSITION"
    echo "Moving from home to above black ball"
    echo "SCALING: J2 -0.23→-0.35, J3 1.24→1.65 (+40% extension)"
    move_all_joints_smooth ${approach[@]} 4
    
    echo "📍 WAYPOINT 2: APPROACH → PICKUP POSITION"
    echo "Descending to pickup position and closing gripper"
    echo "CORRECTION: J1 -0.03→0.02, J2 -0.11→-0.18 (better ball contact)"
    move_all_joints_smooth ${pickup[@]} 3
    
    echo "📍 WAYPOINT 3: PICKUP → LIFT POSITION"
    echo "Lifting ball slightly to clear surface"
    move_all_joints_smooth ${lift[@]} 2
    
    echo "📍 WAYPOINT 4: LIFT → TRANSPORT POSITION"
    echo "Moving ball to tic-tac-toe board center"
    echo "MAJOR CORRECTION: J2 -0.71→-1.00, J3 1.94→2.50 (7cm Y-axis + scaling)"
    move_all_joints_smooth ${place[@]} 5
    
    echo "📍 WAYPOINT 5: PLACE → RETRACT POSITION" 
    echo "Retracting to safe position after ball placement"
    move_all_joints_smooth ${retract[@]} 2
    
    echo "📍 WAYPOINT 6: RETRACT → HOME POSITION"
    echo "Returning to home position"
    move_all_joints_smooth ${home[@]} 4
    
    echo "🎉 BLACK BALL TRAJECTORY COMPLETE! Starting WHITE BALL sequence..."
    echo ""
    
    # Wait at home position
    echo "📍 WAYPOINT 7: HOME → WHITE BALL APPROACH"
    echo "Moving from home to white ball position"
    echo "WHITE BALL: J1=-0.59, J2=0.17, J3=1.09, J4=0.30, J5=-1.64"
    local white_approach=(-0.59 0.17 1.09 0.30 -1.64 0.10 0.10)
    move_all_joints_smooth ${white_approach[@]} 4
    
    echo "📍 WAYPOINT 8: WHITE BALL PICKUP"
    echo "Descending to pickup white ball and closing gripper"
    local white_pickup=(-0.59 0.20 1.12 0.28 -1.64 0.00 0.00)
    move_all_joints_smooth ${white_pickup[@]} 3
    
    echo "📍 WAYPOINT 9: WHITE BALL LIFT"
    echo "Lifting white ball to clear surface"
    local white_lift=(-0.59 0.17 1.12 0.28 -1.64 0.00 0.00)
    move_all_joints_smooth ${white_lift[@]} 2
    
    echo "📍 WAYPOINT 10: WHITE BALL → TIC-TAC-TOE CENTER"
    echo "Moving white ball to tic-tac-toe board center"
    echo "Using scaled center placement position"
    local white_place=(0.08 -1.00 2.50 1.00 0.40 0.10 0.10)
    move_all_joints_smooth ${white_place[@]} 5
    
    echo "📍 WAYPOINT 11: WHITE BALL RETRACT"
    echo "Retracting after white ball placement"
    local white_retract=(0.08 -0.90 2.50 1.00 0.40 0.10 0.10)
    move_all_joints_smooth ${white_retract[@]} 2
    
    echo "� WAYPOINT 12: FINAL HOME POSITION"
    echo "Returning to final home position"
    local final_home=(0.0 0.0 0.0 0.0 0.0 0.0 0.0)
    move_all_joints_smooth ${final_home[@]} 4
    
    echo "🎉 COMPLETE TWO-BALL TRAJECTORY FINISHED!"
    echo ""
    echo "📊 FULL TRAJECTORY SUMMARY:"
    echo "• BLACK BALL: Pickup → Board Center → Home"
    echo "• WHITE BALL: Pickup → Board Center → Home"
    echo "• Arm components: Scaled to 70% of original size"
    echo "• Joint angles: Increased ~40% for same reach capability"
    echo "• Ball pickup: Corrected +3cm for better alignment"
    echo "• Board placement: Corrected +7cm in Y-axis direction"
    echo "• Gripper: Scaled from 0.15 to 0.10 opening"
    echo "• Total waypoints: 12 (6 for each ball sequence)"
}

# Alternative conservative trajectory for testing
execute_conservative_trajectory() {
    echo "🐌 EXECUTING CONSERVATIVE SCALED TRAJECTORY"
    echo "===========================================" 
    echo "Using smaller adjustments for safety testing"
    echo ""
    
    # More conservative scaling adjustments
    local approach=(0.03 -0.28 1.45 0.20 -0.35 0.12 0.12)
    local pickup=(0.01 -0.14 1.50 0.15 -0.06 0.00 0.00)
    local lift=(0.01 -0.12 1.50 0.15 -0.06 0.00 0.00)
    local place=(0.06 -0.85 2.15 0.85 0.32 0.12 0.12)
    local retract=(0.06 -0.75 2.15 0.85 0.32 0.12 0.12)
    local home=(0.0 0.0 0.0 0.0 0.0 0.0 0.0)
    
    echo "📍 Conservative approach..."
    move_all_joints_smooth ${approach[@]} 3
    move_all_joints_smooth ${pickup[@]} 3
    move_all_joints_smooth ${lift[@]} 2
    move_all_joints_smooth ${place[@]} 4
    move_all_joints_smooth ${retract[@]} 2
    move_all_joints_smooth ${home[@]} 3
    
    echo "✅ Black ball conservative trajectory complete!"
    echo ""
    echo "🎯 Starting WHITE BALL conservative sequence..."
    
    # Conservative white ball sequence
    local white_approach=(-0.59 0.15 1.20 0.25 -1.50 0.12 0.12)
    local white_pickup=(-0.59 0.18 1.23 0.23 -1.50 0.00 0.00)
    local white_lift=(-0.59 0.15 1.23 0.23 -1.50 0.00 0.00)
    local white_place=(0.06 -0.85 2.15 0.85 0.32 0.12 0.12)
    local white_retract=(0.06 -0.75 2.15 0.85 0.32 0.12 0.12)
    local final_home=(0.0 0.0 0.0 0.0 0.0 0.0 0.0)
    
    echo "📍 White ball approach..."
    move_all_joints_smooth ${white_approach[@]} 3
    move_all_joints_smooth ${white_pickup[@]} 3
    move_all_joints_smooth ${white_lift[@]} 2
    move_all_joints_smooth ${white_place[@]} 4
    move_all_joints_smooth ${white_retract[@]} 2
    move_all_joints_smooth ${final_home[@]} 3
    
    echo "✅ Complete conservative TWO-BALL trajectory finished!"
}

# Function to move to black ball position (scaled and corrected values)
move_to_black_ball_scaled() {
    echo "Moving to black ball position with SCALED and CORRECTED joint values..."
    echo "Original: J1=0.02, J2=-0.11, J3=1.05, J4=0.37, J5=-0.41"
    echo "Scaled:   J1=0.02, J2=-0.18, J3=1.70, J4=0.45, J5=-0.55 (+40% extension)"
    send_joint_command "joint1" "0.02"   # Base rotation
    send_joint_command "joint2" "-0.18"  # Shoulder (increased for scaled reach)
    send_joint_command "joint3" "1.70"   # Elbow (significantly increased)
    send_joint_command "joint4" "0.45"   # Wrist pitch (increased)
    send_joint_command "joint5" "-0.55"  # Wrist roll (increased)
    sleep 2
}

# Function to move to white ball position and then to tic-tac-toe center
move_to_white_ball_scaled() {
    echo "🎯 WHITE BALL TEST SEQUENCE"
    echo "=========================="
    echo ""
    
    echo "STEP 1: Moving to white ball position with PROVIDED joint values..."
    echo "User values: J1=-0.59, J2=0.17, J3=1.09, J4=0.30, J5=-1.64"
    send_joint_command "joint1" "-0.59"  # Base rotation
    send_joint_command "joint2" "0.17"   # Shoulder 
    send_joint_command "joint3" "1.09"   # Elbow
    send_joint_command "joint4" "0.30"   # Wrist pitch
    send_joint_command "joint5" "-1.64"  # Wrist roll
    sleep 3
    echo "✅ White ball position reached"
    echo ""
    
    echo "STEP 2: Moving to tic-tac-toe board center..."
    echo "Center values: J1=0.02, J2=-0.24, J3=1.28, J4=0.65, J5=0.27"
    send_joint_command "joint1" "0.02"   # Base rotation
    send_joint_command "joint2" "-0.24"  # Shoulder
    send_joint_command "joint3" "1.28"   # Elbow
    send_joint_command "joint4" "0.65"   # Wrist pitch
    send_joint_command "joint5" "0.27"   # Wrist roll
    sleep 3
    echo "✅ Tic-tac-toe board center reached"
    echo ""
    
    echo "STEP 3: Returning to home position..."
    send_joint_command "joint1" "0.0"
    send_joint_command "joint2" "0.0"
    send_joint_command "joint3" "0.0"
    send_joint_command "joint4" "0.0"
    send_joint_command "joint5" "0.0"
    sleep 3
    echo "✅ Home position reached"
    echo ""
    
    echo "🎉 WHITE BALL TEST SEQUENCE COMPLETE!"
    echo "Sequence: White Ball → Tic-Tac-Toe Center → Home"
}

# Main execution menu
echo ""
echo "SCALED ARM TRAJECTORY OPTIONS:"
echo "=============================="
echo "1. Full scaled TWO-BALL trajectory (black → white)"
echo "2. Conservative scaled TWO-BALL trajectory (safer testing)"  
echo "3. Test black ball position only"
echo "4. Test white ball position only"
echo ""
echo "Usage:"
echo "  ./move1.sh              - Run full TWO-BALL trajectory"
echo "  ./move1.sh conservative - Run conservative TWO-BALL trajectory"
echo "  ./move1.sh black        - Test black ball position only"
echo "  ./move1.sh white        - Test white ball position only"
echo ""

# Parse command line argument
if [[ "$1" == "conservative" ]]; then
    execute_conservative_trajectory
elif [[ "$1" == "black" ]]; then
    move_to_black_ball_scaled
elif [[ "$1" == "white" ]]; then
    move_to_white_ball_scaled  
elif [[ "$1" == "test" ]]; then
    # Legacy support for 'test' - defaults to black ball
    move_to_black_ball_scaled
else
    execute_scaled_trajectory
fi

echo ""
echo "🔧 SCALING INFO:"
echo "This script is optimized for 0.7x scaled arm components"
echo "Joint angles increased ~40% to compensate for smaller arm"
echo "Position corrections applied for better ball pickup/placement"
echo "Use 'conservative' option if movements seem too aggressive"
echo ""

# Check for command line arguments
if [ "$1" == "blackball" ]; then
    echo "=== ADJUSTED Quick Black Ball Test ==="
    move_to_black_ball
    exit 0
fi

if [ "$1" == "pickup" ]; then
    echo "=== ADJUSTED Quick Black Ball Pickup Test ==="
    move_gripper "0.15"  # Open gripper
    move_to_black_ball
    move_gripper "0.0"   # Close gripper
    echo "ADJUSTED black ball pickup complete!"
    exit 0
fi

if [ "$1" == "trajectory" ]; then
    echo "=== ADJUSTED Full Trajectory (Version 1) ==="
    execute_ball_pickup_trajectory
    exit 0
fi

if [ "$1" == "v2" ]; then
    echo "=== ADJUSTED Full Trajectory (Version 2 - More Aggressive) ==="
    execute_ball_pickup_trajectory_v2
    exit 0
fi

if [ "$1" == "test" ]; then
    echo "=== POSITION COMPARISON TEST ==="
    echo ""
    echo "Testing ORIGINAL positions vs ADJUSTED positions..."
    echo ""
    
    echo "1. Testing ORIGINAL approach position..."
    move_all_joints 0 -0.23 1.24 0.17 -0.30 0.15 0.15 3
    echo "Press Enter to continue to ADJUSTED approach..."
    read
    
    echo "2. Testing ADJUSTED approach position..."
    move_all_joints 0.02 -0.20 1.20 0.15 -0.28 0.15 0.15 3
    echo "Press Enter to continue..."
    read
    
    echo "3. Testing ORIGINAL board position..."
    move_all_joints 0.04 -0.71 1.94 0.73 0.27 0.15 0.15 3
    echo "Press Enter to continue to ADJUSTED board..."
    read
    
    echo "4. Testing ADJUSTED board position..."
    move_all_joints 0.08 -0.64 1.87 0.68 0.22 0.15 0.15 3
    echo ""
    
    echo "✅ Position comparison test complete!"
    echo "Compare the positions to see which is more accurate."
    exit 0
fi

# Default behavior - show usage
echo ""
echo "🎯 ADJUSTED TRAJECTORY SCRIPT - USAGE:"
echo "======================================"
echo ""
echo "Available commands:"
echo "1. ./move1.sh trajectory    - Execute adjusted trajectory (recommended)"
echo "2. ./move1.sh v2            - Execute more aggressive adjustments"
echo "3. ./move1.sh test          - Compare original vs adjusted positions"
echo "4. ./move1.sh blackball     - Test adjusted ball position only"
echo "5. ./move1.sh pickup        - Test adjusted pickup sequence only"
echo ""
echo "🔧 ADJUSTMENTS MADE:"
echo "• Ball pickup: 2-3cm positioning correction"
echo "• Board placement: 6-7cm Y-axis correction (forward)"
echo "• Smoother intermediate waypoints"
echo "• Fine-tuned joint angles for better accuracy"
echo ""
echo "💡 RECOMMENDATIONS:"
echo "1. Start with: ./move1.sh trajectory"
echo "2. If still missing, try: ./move1.sh v2"  
echo "3. Use test mode to compare positions visually"
echo ""
echo "Original vs Adjusted comparison:"
echo "Ball approach: J2 -0.23→-0.20, J3 1.24→1.20 (closer)"
echo "Ball pickup:   J1 -0.03→-0.01, J2 -0.11→-0.08 (better contact)"
echo "Board place:   J1 0.04→0.08, J2 -0.71→-0.64 (Y-axis forward 6-7cm)"
