#!/bin/bash

echo "=== BLACK BALL TRAJECTORY TEST ==="

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

echo "STEP 1: Moving to approach position above black ball..."
echo "Target: J1=0, J2=-0.23, J3=1.24, J4=0.17, J5=-0.30, Fingers=0.15 (open)"
move_all_joints 0 -0.23 1.24 0.17 -0.30 0.15 0.15 3
echo "✅ Approach position reached - Press Enter to continue..."
read

echo "STEP 2: Moving to pickup position and closing gripper..."
echo "Target: J1=-0.03, J2=-0.11, J3=1.28, J4=0.12, J5=-0.04, Fingers=0.00 (closed)"
move_all_joints -0.03 -0.11 1.28 0.12 -0.04 0.00 0.00 3
echo "✅ Ball gripped - Press Enter to continue..."
read

echo "STEP 3: Lifting ball from pickup position..."
echo "Target: Slight lift - J2=-0.08 (lifting shoulder)"
move_all_joints -0.03 -0.08 1.28 0.12 -0.04 0.00 0.00 2
echo "✅ Ball lifted - Press Enter to continue..."
read

echo "STEP 4: Moving to tic-tac-toe board center and releasing..."
echo "Target: J1=0.04, J2=-0.71, J3=1.94, J4=0.73, J5=0.27, Fingers=0.15 (open)"
move_all_joints 0.04 -0.71 1.94 0.73 0.27 0.15 0.15 4
echo "✅ Ball placed on tic-tac-toe board - Press Enter to continue..."
read

echo "STEP 5: Retracting to safe position..."
echo "Target: Lifting up from board - J2=-0.60"
move_all_joints 0.04 -0.60 1.94 0.73 0.27 0.15 0.15 2
echo "✅ Safe retraction complete - Press Enter to continue..."
read

echo "STEP 6: Returning to home position..."
echo "Target: All joints to 0, gripper closed"
move_all_joints 0.0 0.0 0.0 0.0 0.0 0.0 0.0 3
echo "✅ Trajectory execution complete!"

echo ""
echo "🎉 SUCCESS: Black ball trajectory test completed!"
