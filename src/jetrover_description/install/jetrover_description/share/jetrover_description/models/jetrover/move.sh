#!/bin/bash

# Test script to move arm joints and navigate around the tic-tac-toe board
# This script uses Gazebo transport to send joint position commands

echo "Starting robot arm movement test for tictactoe_NEW2.sdf..."

# Function to send joint command
send_joint_command() {
    local joint_name=$1
    local position=$2
    local topic="/model/jetrover/joint/${joint_name}/cmd_pos"
    
    echo "Sending $joint_name to position $position"
    gz topic -t "$topic" -m gz.msgs.Double -p "data: $position"
    sleep 0.5
}

# Function to move gripper
move_gripper() {
    local r_pos=$1
    local l_pos=${2:-$(echo "$r_pos * -1" | bc -l)}
    
    echo "Moving gripper - R_joint: $r_pos, L_joint: $l_pos"
    gz topic -t "/model/jetrover/joint/r_joint/cmd_pos" -m gz.msgs.Double -p "data: $r_pos"
    gz topic -t "/model/jetrover/joint/l_joint/cmd_pos" -m gz.msgs.Double -p "data: $l_pos"
    sleep 0.3
}

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

echo "=== Test Sequence 4: Ball Store Interaction ==="

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

echo "=== Test Sequence 5: Complex Movement Pattern ==="

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
echo "1. Start Gazebo with: gz sim tictactoe_NEW2.sdf"
echo "2. Run this script with: ./move.sh"
echo "3. Watch the robot arm move through various test sequences"
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