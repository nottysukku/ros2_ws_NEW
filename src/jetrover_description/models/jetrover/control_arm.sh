#!/bin/bash

# JetRover Arm Control Script - Precise Pick and Place for O Pieces
echo "=== JetRover Arm Movement - O Piece Pick & Place ==="

# SDF Coordinate Analysis:
# Robot base: (-0.026, 0.218, 0.684)
# O piece 1: (0.048, -0.157, 0.432) - Relative: (0.074, -0.375, -0.252) - RIGHT SIDE
# O piece 2: (-0.032, -0.157, 0.432) - Relative: (-0.006, -0.375, -0.252) - CENTER  
# O piece 3: (-0.112, -0.156, 0.432) - Relative: (-0.086, -0.374, -0.252) - LEFT SIDE
# Board center: (-0.122, -0.071, 0.450) - Relative: (-0.096, -0.289, -0.234)

# Function to move to home/safe position
move_to_home() {
    echo "Moving to home position..."
    gz topic -t /model/jetrover/joint/joint1/0/cmd_pos -m gz.msgs.Double -p 'data: 0.0'
    gz topic -t /model/jetrover/joint/joint2/0/cmd_pos -m gz.msgs.Double -p 'data: 0.0'
    gz topic -t /model/jetrover/joint/joint3/0/cmd_pos -m gz.msgs.Double -p 'data: 0.0'
    gz topic -t /model/jetrover/joint/joint4/0/cmd_pos -m gz.msgs.Double -p 'data: 0.0'
    gz topic -t /model/jetrover/joint/joint5/0/cmd_pos -m gz.msgs.Double -p 'data: 0.0'
    sleep 2
}

# Function to open gripper
open_gripper() {
    echo "Opening gripper..."
    gz topic -t /model/jetrover/joint/left_finger_joint/0/cmd_pos -m gz.msgs.Double -p 'data: 0.08'
    gz topic -t /model/jetrover/joint/right_finger_joint/0/cmd_pos -m gz.msgs.Double -p 'data: 0.08'
    sleep 1.5
}

# Function to close gripper
close_gripper() {
    echo "Closing gripper..."
    gz topic -t /model/jetrover/joint/left_finger_joint/0/cmd_pos -m gz.msgs.Double -p 'data: 0.0'
    gz topic -t /model/jetrover/joint/right_finger_joint/0/cmd_pos -m gz.msgs.Double -p 'data: 0.0'
    sleep 1.5
}

# Function to move to pre-pick position for O piece 1 (rightmost)
move_to_o1_pick() {
    echo "Moving to O piece 1 (right side) at (0.048, -0.157, 0.432)..."
    gz topic -t /model/jetrover/joint/joint1/0/cmd_pos -m gz.msgs.Double -p 'data: -1.4'  # Rotate right
    sleep 2
    gz topic -t /model/jetrover/joint/joint2/0/cmd_pos -m gz.msgs.Double -p 'data: 1.3'   # Shoulder down
    sleep 2
    gz topic -t /model/jetrover/joint/joint3/0/cmd_pos -m gz.msgs.Double -p 'data: -1.9'  # Elbow extend
    sleep 2
    gz topic -t /model/jetrover/joint/joint4/0/cmd_pos -m gz.msgs.Double -p 'data: 0.6'   # Wrist down
    sleep 2
}

# Function to move to pre-pick position for O piece 2 (center)
move_to_o2_pick() {
    echo "Moving to O piece 2 (center) at (-0.032, -0.157, 0.432)..."
    gz topic -t /model/jetrover/joint/joint1/0/cmd_pos -m gz.msgs.Double -p 'data: -0.1'  # Slight rotation
    sleep 2
    gz topic -t /model/jetrover/joint/joint2/0/cmd_pos -m gz.msgs.Double -p 'data: 1.3'   # Shoulder down
    sleep 2
    gz topic -t /model/jetrover/joint/joint3/0/cmd_pos -m gz.msgs.Double -p 'data: -1.8'  # Elbow extend
    sleep 2
    gz topic -t /model/jetrover/joint/joint4/0/cmd_pos -m gz.msgs.Double -p 'data: 0.5'   # Wrist down
    sleep 2
}

# Function to move to pre-pick position for O piece 3 (leftmost)
move_to_o3_pick() {
    echo "Moving to O piece 3 (left side) at (-0.112, -0.156, 0.432)..."
    gz topic -t /model/jetrover/joint/joint1/0/cmd_pos -m gz.msgs.Double -p 'data: 0.8'   # Rotate left
    sleep 2
    gz topic -t /model/jetrover/joint/joint2/0/cmd_pos -m gz.msgs.Double -p 'data: 1.2'   # Shoulder down
    sleep 2
    gz topic -t /model/jetrover/joint/joint3/0/cmd_pos -m gz.msgs.Double -p 'data: -1.7'  # Elbow extend
    sleep 2
    gz topic -t /model/jetrover/joint/joint4/0/cmd_pos -m gz.msgs.Double -p 'data: 0.5'   # Wrist down
    sleep 2
}

# Function to move to pre-pick position (above O piece) - Default to O1
move_to_pre_pick() {
    echo "Moving to pre-pick position (above O piece 1 - default)..."
    move_to_o1_pick
}

# Function to move to pick position (lower to grab piece)
move_to_pick() {
    echo "Lowering to pick position..."
    # Lower the arm slightly to touch the O piece
    gz topic -t /model/jetrover/joint/joint2/0/cmd_pos -m gz.msgs.Double -p 'data: 1.4'  # Lower more
    gz topic -t /model/jetrover/joint/joint3/0/cmd_pos -m gz.msgs.Double -p 'data: -2.0' # Extend further
    sleep 3
}

# Function to lift piece after grasping
lift_piece() {
    echo "Lifting piece..."
    gz topic -t /model/jetrover/joint/joint2/0/cmd_pos -m gz.msgs.Double -p 'data: 1.0'  # Lift up
    gz topic -t /model/jetrover/joint/joint3/0/cmd_pos -m gz.msgs.Double -p 'data: -1.6' # Retract slightly
    sleep 3
}

# Function to move to pre-place position (above board center)
move_to_pre_place() {
    echo "Moving to pre-place position (above board center)..."
    # Move to board center (-0.122, -0.071, 0.450) relative to robot base (-0.026, 0.218, 0.684)
    # Relative position: (-0.096, -0.289, -0.234)
    gz topic -t /model/jetrover/joint/joint1/0/cmd_pos -m gz.msgs.Double -p 'data: -0.8' # Rotate toward board center
    sleep 2
    gz topic -t /model/jetrover/joint/joint2/0/cmd_pos -m gz.msgs.Double -p 'data: 0.8'  # Adjust height for board
    sleep 2
    gz topic -t /model/jetrover/joint/joint3/0/cmd_pos -m gz.msgs.Double -p 'data: -1.2' # Adjust reach
    sleep 2
    gz topic -t /model/jetrover/joint/joint4/0/cmd_pos -m gz.msgs.Double -p 'data: 0.4'  # Adjust wrist
    sleep 2
}

# Function to place piece on board
place_piece() {
    echo "Placing piece on board..."
    # Lower slightly to place on board surface
    gz topic -t /model/jetrover/joint/joint2/0/cmd_pos -m gz.msgs.Double -p 'data: 1.0'  # Lower to board
    sleep 2
}

# Complete pick and place sequence for O piece
pick_and_place_o_piece() {
    echo "=== Starting complete O piece pick and place sequence ==="
    move_to_home
    open_gripper
    move_to_pre_pick
    move_to_pick
    close_gripper
    lift_piece
    move_to_pre_place
    place_piece
    open_gripper
    sleep 1
    move_to_home
    echo "=== O piece pick and place complete! ==="
}

# Calibration sequence - test positions step by step
calibrate_positions() {
    echo "=== Position Calibration Sequence ==="
    echo "Testing home position..."
    move_to_home
    sleep 3
    
    echo "Testing base rotation for O piece..."
    gz topic -t /model/jetrover/joint/joint1/0/cmd_pos -m gz.msgs.Double -p 'data: -1.0'
    sleep 3
    
    echo "Testing shoulder movement..."
    gz topic -t /model/jetrover/joint/joint2/0/cmd_pos -m gz.msgs.Double -p 'data: 1.0'
    sleep 3
    
    echo "Testing elbow extension..."
    gz topic -t /model/jetrover/joint/joint3/0/cmd_pos -m gz.msgs.Double -p 'data: -1.5'
    sleep 3
    
    echo "Testing wrist adjustment..."
    gz topic -t /model/jetrover/joint/joint4/0/cmd_pos -m gz.msgs.Double -p 'data: 0.5'
    sleep 3
    
    echo "Returning to home..."
    move_to_home
    echo "=== Calibration complete! ==="
}

# Test individual joint movement
test_joint() {
    echo "Testing joint $1 with value $2..."
    gz topic -t /model/jetrover/joint/joint$1/0/cmd_pos -m gz.msgs.Double -p "data: $2"
    sleep 2
}

# Main execution
case "$1" in
    "home")
        move_to_home
        ;;
    "open")
        open_gripper
        ;;
    "close")
        close_gripper
        ;;
    "prepick")
        move_to_pre_pick
        ;;
    "o1")
        move_to_o1_pick
        ;;
    "o2")
        move_to_o2_pick
        ;;
    "o3")
        move_to_o3_pick
        ;;
    "pick")
        move_to_pick
        ;;
    "lift")
        lift_piece
        ;;
    "preplace")
        move_to_pre_place
        ;;
    "place")
        place_piece
        ;;
    "full")
        pick_and_place_o_piece
        ;;
    "calibrate")
        calibrate_positions
        ;;
    "test")
        if [ "$2" ] && [ "$3" ]; then
            test_joint $2 $3
        else
            echo "Usage for test: $0 test <joint_number> <position_value>"
            echo "Example: $0 test 1 0.5"
        fi
        ;;
    *)
        echo "=== JetRover O Piece Pick & Place Control ==="
        echo "Usage: $0 {command}"
        echo ""
        echo "Individual commands:"
        echo "  home     - Move to safe home position"
        echo "  open     - Open gripper"
        echo "  close    - Close gripper"
        echo "  prepick  - Move above O piece 1 (default)"
        echo "  o1       - Move to O piece 1 (right side)"
        echo "  o2       - Move to O piece 2 (center)"  
        echo "  o3       - Move to O piece 3 (left side)"
        echo "  pick     - Lower to pick position"
        echo "  lift     - Lift piece after grasping"
        echo "  preplace - Move above board center"
        echo "  place    - Lower to place on board"
        echo ""
        echo "Complete sequences:"
        echo "  full     - Complete pick and place of O piece"
        echo "  calibrate - Step-by-step position calibration"
        echo ""
        echo "Testing:"
        echo "  test <joint> <value> - Test individual joint"
        echo "         Example: $0 test 1 0.5"
        echo ""
        echo "Target: O piece at (0.048, -0.157, 0.432)"
        echo "Destination: Board center at (-0.122, -0.071, 0.450)"
        ;;
esac
