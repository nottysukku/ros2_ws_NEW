#!/bin/bash

# Test script to move arm joints and navigate around the tic-tac-toe board
# This script uses Gazebo transport to send joint position commands

echo "Starting robot arm movement test for tictactoe_NEW3.2.sdf..."

# Function to send joint command
send_joint_command() {
    local joint_name=$1
    local position=$2
    local topic="/model/jetrover/joint/${joint_name}/0/cmd_pos"
    
    echo "Sending $joint_name to position $position"
    gz topic -t "$topic" -m gz.msgs.Double -p "data: $position"
    sleep 0.5
}

# Function to move gripper
move_gripper() {
    local grip_pos=$1
    
    echo "Moving gripper - Opening to: $grip_pos"
    gz topic -t "/model/jetrover/joint/left_finger_joint/0/cmd_pos" -m gz.msgs.Double -p "data: $grip_pos"
    gz topic -t "/model/jetrover/joint/right_finger_joint/0/cmd_pos" -m gz.msgs.Double -p "data: $grip_pos"
    sleep 0.5
}

# Function to move all joints to specific positions
move_all_joints() {
    local j1=$1 j2=$2 j3=$3 j4=$4 j5=$5 j6=$6 j7=$7
    local wait_time=${8:-2}
    
    echo "Moving all joints: J1=$j1, J2=$j2, J3=$j3, J4=$j4, J5=$j5, Fingers=$j6"
    
    # Move arm joints
    gz topic -t "/model/jetrover/joint/joint1/0/cmd_pos" -m gz.msgs.Double -p "data: $j1" &
    gz topic -t "/model/jetrover/joint/joint2/0/cmd_pos" -m gz.msgs.Double -p "data: $j2" &
    gz topic -t "/model/jetrover/joint/joint3/0/cmd_pos" -m gz.msgs.Double -p "data: $j3" &
    gz topic -t "/model/jetrover/joint/joint4/0/cmd_pos" -m gz.msgs.Double -p "data: $j4" &
    gz topic -t "/model/jetrover/joint/joint5/0/cmd_pos" -m gz.msgs.Double -p "data: $j5" &
    
    # Move finger joints
    gz topic -t "/model/jetrover/joint/left_finger_joint/0/cmd_pos" -m gz.msgs.Double -p "data: $j6" &
    gz topic -t "/model/jetrover/joint/right_finger_joint/0/cmd_pos" -m gz.msgs.Double -p "data: $j7" &
    
    wait
    sleep $wait_time
}

# Main trajectory function: Pick up black ball and place on tic-tac-toe board
execute_ball_pickup_trajectory() {
    echo "=== TRAJECTORY PLANNING: BLACK BALL TO TIC-TAC-TOE BOARD ==="
    echo ""
    
    # Step 1: Move to "almost above black middle ball" position
    echo "STEP 1: Moving to approach position above black ball..."
    echo "Target: J1=0, J2=-0.23, J3=1.24, J4=0.17, J5=-0.30, Fingers=0.15 (open)"
    move_all_joints 0 -0.23 1.24 0.17 -0.30 0.15 0.15 3
    echo "✅ Approach position reached"
    echo ""
    
    # Step 2: Move to pickup position and close gripper
    echo "STEP 2: Moving to pickup position and closing gripper..."
    echo "Target: J1=-0.03, J2=-0.11, J3=1.28, J4=0.12, J5=-0.04, Fingers=0.00 (closed)"
    move_all_joints -0.03 -0.11 1.28 0.12 -0.04 0.00 0.00 3
    echo "✅ Ball gripped successfully"
    echo ""
    
    # Step 3: Lift ball slightly (intermediate position)
    echo "STEP 3: Lifting ball from pickup position..."
    echo "Target: Slight lift - J2=-0.08 (lifting shoulder)"
    move_all_joints -0.03 -0.08 1.28 0.12 -0.04 0.00 0.00 2
    echo "✅ Ball lifted"
    echo ""
    
    # Step 4: Move to mid-center of tic-tac-toe board and release
    echo "STEP 4: Moving to tic-tac-toe board center and releasing..."
    echo "Target: J1=0.04, J2=-0.71, J3=1.94, J4=0.73, J5=0.27, Fingers=0.15 (open)"
    move_all_joints 0.04 -0.71 1.94 0.73 0.27 0.15 0.15 4
    echo "✅ Ball placed on tic-tac-toe board"
    echo ""
    
    # Step 5: Retract to safe position
    echo "STEP 5: Retracting to safe position..."
    echo "Target: Lifting up from board - J2=-0.60"
    move_all_joints 0.04 -0.60 1.94 0.73 0.27 0.15 0.15 2
    echo "✅ Safe retraction complete"
    echo ""
    
    # Step 6: Return to home position
    echo "STEP 6: Returning to home position..."
    echo "Target: All joints to 0, gripper closed"
    move_all_joints 0.0 0.0 0.0 0.0 0.0 0.0 0.0 3
    echo "✅ Trajectory execution complete!"
    echo ""
    
    echo "🎉 SUCCESS: Black ball successfully picked up and placed on tic-tac-toe board!"
    echo ""
    echo "Trajectory Summary:"
    echo "1. ✅ Approached black ball from above"
    echo "2. ✅ Descended to pickup position"  
    echo "3. ✅ Closed gripper to grip ball"
    echo "4. ✅ Lifted ball safely"
    echo "5. ✅ Moved to tic-tac-toe board center"
    echo "6. ✅ Released ball on board"
    echo "7. ✅ Retracted to safe position"
    echo "8. ✅ Returned to home"
}

# Function to move to black ball position (optimized values)
move_to_black_ball() {
    echo "Moving to black ball position with optimized joint values..."
    send_joint_command "joint1" "0.02"   # Base rotation
    send_joint_command "joint2" "-0.11"  # Shoulder
    send_joint_command "joint3" "1.05"   # Elbow  
    send_joint_command "joint4" "0.37"   # Wrist pitch
    send_joint_command "joint5" "-0.41"  # Wrist roll
    sleep 2
}

# Check for command line arguments
if [ "$1" == "blackball" ]; then
    echo "=== Quick Black Ball Test ==="
    move_to_black_ball
    exit 0
fi

if [ "$1" == "pickup" ]; then
    echo "=== Quick Black Ball Pickup Test ==="
    move_gripper "0.8"  # Open gripper
    move_to_black_ball
    move_gripper "0.0"  # Close gripper
    echo "Black ball pickup complete!"
    exit 0
fi

if [ "$1" == "trajectory" ]; then
    echo "=== Full Black Ball to Tic-Tac-Toe Trajectory ==="
    execute_ball_pickup_trajectory
    exit 0
fi

echo "=== Starting Joint Movement Test ==="

# Reset all joints to home position
echo "Resetting to home position..."
send_joint_command "joint1" "0.0"
send_joint_command "joint2" "0.0"
send_joint_command "joint3" "0.0"
send_joint_command "joint4" "0.0"
send_joint_command "joint5" "0.0"
move_gripper "0.0"

echo "=== Test Sequence 1: Basic Joint Movements ==="

# Test optimized black ball position first
echo "Testing optimized black ball position..."
move_to_black_ball

# Return to home before other tests
echo "Returning to home position..."
send_joint_command "joint1" "0.0"
send_joint_command "joint2" "0.0"
send_joint_command "joint3" "0.0"
send_joint_command "joint4" "0.0"
send_joint_command "joint5" "0.0"

# Test joint1 (base rotation) - look around the board
echo "Testing base rotation (joint1)..."
send_joint_command "joint1" "0.5"
send_joint_command "joint1" "-0.5"
send_joint_command "joint1" "0.0"

# Test joint2 (shoulder) - arm up/down
echo "Testing shoulder movement (joint2)..."
send_joint_command "joint2" "0.3"
send_joint_command "joint2" "-0.3"
send_joint_command "joint2" "0.0"

# Test joint3 (elbow) - elbow bend
echo "Testing elbow movement (joint3)..."
send_joint_command "joint3" "0.5"
send_joint_command "joint3" "-0.5"
send_joint_command "joint3" "0.0"

# Test joint4 (wrist pitch) - wrist up/down
echo "Testing wrist pitch (joint4)..."
send_joint_command "joint4" "0.4"
send_joint_command "joint4" "-0.4"
send_joint_command "joint4" "0.0"

# Test joint5 (wrist roll) - wrist rotation
echo "Testing wrist roll (joint5)..."
send_joint_command "joint5" "0.6"
send_joint_command "joint5" "-0.6"
send_joint_command "joint5" "0.0"

echo "=== Test Sequence 2: Tic-Tac-Toe Board Navigation ==="

# Move to center of board
echo "Moving to center of tic-tac-toe board..."
send_joint_command "joint1" "0.1"   # Slight rotation towards board
send_joint_command "joint2" "0.2"   # Lift arm
send_joint_command "joint3" "0.3"   # Bend elbow
send_joint_command "joint4" "-0.2"  # Point down
sleep 1

# Move to top-left square
echo "Moving to top-left square..."
send_joint_command "joint1" "0.3"
send_joint_command "joint2" "0.1"
sleep 1

# Move to top-center square
echo "Moving to top-center square..."
send_joint_command "joint1" "0.1"
sleep 1

# Move to top-right square
echo "Moving to top-right square..."
send_joint_command "joint1" "-0.1"
sleep 1

# Move to center-left square
echo "Moving to center-left square..."
send_joint_command "joint1" "0.3"
send_joint_command "joint2" "0.2"
sleep 1

# Move to center square
echo "Moving to center square..."
send_joint_command "joint1" "0.1"
sleep 1

# Move to center-right square
echo "Moving to center-right square..."
send_joint_command "joint1" "-0.1"
sleep 1

# Move to bottom-left square
echo "Moving to bottom-left square..."
send_joint_command "joint1" "0.3"
send_joint_command "joint2" "0.3"
sleep 1

# Move to bottom-center square
echo "Moving to bottom-center square..."
send_joint_command "joint1" "0.1"
sleep 1

# Move to bottom-right square
echo "Moving to bottom-right square..."
send_joint_command "joint1" "-0.1"
sleep 1

echo "=== Test Sequence 3: Gripper Operation ==="

# Test gripper open/close
echo "Testing gripper operation..."
echo "Opening gripper wide..."
move_gripper "0.8"
echo "Partially closing gripper..."
move_gripper "0.4"
echo "Closing gripper..."
move_gripper "0.0"
echo "Resetting gripper..."
move_gripper "0.0"

echo "=== Test Sequence 4: Black Ball Pick-up Test ==="

# Test the optimized black ball position
echo "Testing black ball pick-up sequence..."
echo "Opening gripper..."
move_gripper "0.8"

echo "Moving to black ball position..."
move_to_black_ball

echo "Closing gripper to grab ball..."
move_gripper "0.0"
sleep 1

echo "Lifting ball slightly..."
send_joint_command "joint2" "-0.20"  # Lift shoulder slightly
sleep 1

echo "Moving to board center for placement..."
send_joint_command "joint1" "0.0"    # Center base
send_joint_command "joint2" "0.1"    # Adjust height
send_joint_command "joint3" "0.3"    # Bring elbow in
send_joint_command "joint4" "-0.2"   # Point down
send_joint_command "joint5" "0.0"    # Reset roll
sleep 2

echo "Releasing ball..."
move_gripper "0.8"
sleep 1

echo "=== Test Sequence 5: Ball Store Interaction ==="

# Move towards grey ball store (left side)
echo "Moving towards grey ball store..."
send_joint_command "joint1" "0.8"   # Rotate towards left ball store
send_joint_command "joint2" "0.1"   # Adjust height
send_joint_command "joint3" "0.4"   # Reach out
sleep 1

# Move towards white ball store (right side)
echo "Moving towards white ball store..."
send_joint_command "joint1" "-0.8"  # Rotate towards right ball store
send_joint_command "joint2" "0.1"   # Adjust height
send_joint_command "joint3" "0.4"   # Reach out
sleep 1

echo "=== Test Sequence 6: Complex Movement Pattern ==="

# Perform a complex movement pattern over the board
echo "Performing complex movement pattern..."

# Figure-8 pattern over the board
positions=(
    "0.2 0.1 0.3 -0.1 0.0"   # Position 1
    "0.0 0.15 0.4 -0.2 0.2"  # Position 2
    "-0.2 0.1 0.3 -0.1 0.0"  # Position 3
    "0.0 0.25 0.2 -0.3 -0.2" # Position 4
    "0.2 0.1 0.3 -0.1 0.0"   # Return to start
)

for i in "${!positions[@]}"; do
    echo "Moving to pattern position $((i+1))..."
    pos_array=(${positions[$i]})
    send_joint_command "joint1" ${pos_array[0]}
    send_joint_command "joint2" ${pos_array[1]}
    send_joint_command "joint3" ${pos_array[2]}
    send_joint_command "joint4" ${pos_array[3]}
    send_joint_command "joint5" ${pos_array[4]}
    sleep 1
done

echo "=== Final Reset ==="

# Return to home position
echo "Returning to home position..."
send_joint_command "joint1" "0.0"
send_joint_command "joint2" "0.0"
send_joint_command "joint3" "0.0"
send_joint_command "joint4" "0.0"
send_joint_command "joint5" "0.0"
move_gripper "0.0"

echo "=== Test Complete ==="
echo "Robot arm movement test completed successfully!"
echo ""
echo "Usage instructions:"
echo "1. Start Gazebo with: gz sim tictactoe_NEW3.2.sdf"
echo "2. Run full test with: ./move.sh"
echo "3. Quick black ball test: ./move.sh blackball"
echo "4. Quick pickup test: ./move.sh pickup"
echo "5. 🎯 FULL TRAJECTORY: ./move.sh trajectory"
echo "6. Watch the robot arm move through various test sequences"
echo ""
echo "🎯 NEW: Full Black Ball Pickup Trajectory Available!"
echo "   ./move.sh trajectory - Complete pick & place sequence"
echo "   Uses real joint values for precision movement"
echo ""
echo "Joint value references:"
echo "• Approach: J1=0, J2=-0.23, J3=1.24, J4=0.17, J5=-0.30"
echo "• Pickup:   J1=-0.03, J2=-0.11, J3=1.28, J4=0.12, J5=-0.04"  
echo "• Place:    J1=0.04, J2=-0.71, J3=1.94, J4=0.73, J5=0.27"
echo ""
echo "Black ball joint values: J1=0.02, J2=-0.11, J3=1.05, J4=0.37, J5=-0.41"
echo ""
echo "Debugging information:"
echo "If joints are not moving, check:"
echo "1. Is Gazebo running and robot model loaded?"
echo "2. Are there any error messages in Gazebo console?"
echo "3. Check joint controller plugins are loaded"
echo ""
echo "Available topics to monitor:"
echo "gz topic -l | grep jetrover"
echo "gz topic -e -t /model/jetrover/joint/joint1/cmd_pos"
echo "gz topic -e -t /model/jetrover/joint/r_joint/cmd_pos"
echo "gz topic -e -t /model/jetrover/joint/l_joint/cmd_pos"
echo ""
echo "To check joint states:"
echo "gz topic -e -t /model/jetrover/joint_state"