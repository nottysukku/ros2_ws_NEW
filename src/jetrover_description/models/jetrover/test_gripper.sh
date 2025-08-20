#!/bin/bash

# Gripper Test Script - Enhanced Control
echo "=== Enhanced Gripper Test ==="

echo "Testing gripper with various positions..."

# Test full range of motion
echo "1. Full open (0.12m):"
gz topic -t /model/jetrover/joint/left_finger_joint/0/cmd_pos -m gz.msgs.Double -p 'data: 0.12'
gz topic -t /model/jetrover/joint/right_finger_joint/0/cmd_pos -m gz.msgs.Double -p 'data: 0.12'
sleep 3

echo "2. Partially open (0.08m):"
gz topic -t /model/jetrover/joint/left_finger_joint/0/cmd_pos -m gz.msgs.Double -p 'data: 0.08'
gz topic -t /model/jetrover/joint/right_finger_joint/0/cmd_pos -m gz.msgs.Double -p 'data: 0.08'
sleep 3

echo "3. Small opening (0.04m):"
gz topic -t /model/jetrover/joint/left_finger_joint/0/cmd_pos -m gz.msgs.Double -p 'data: 0.04'
gz topic -t /model/jetrover/joint/right_finger_joint/0/cmd_pos -m gz.msgs.Double -p 'data: 0.04'
sleep 3

echo "4. Light grip (0.02m):"
gz topic -t /model/jetrover/joint/left_finger_joint/0/cmd_pos -m gz.msgs.Double -p 'data: 0.02'
gz topic -t /model/jetrover/joint/right_finger_joint/0/cmd_pos -m gz.msgs.Double -p 'data: 0.02'
sleep 3

echo "5. Tight grip (0.01m):"
gz topic -t /model/jetrover/joint/left_finger_joint/0/cmd_pos -m gz.msgs.Double -p 'data: 0.01'
gz topic -t /model/jetrover/joint/right_finger_joint/0/cmd_pos -m gz.msgs.Double -p 'data: 0.01'
sleep 3

echo "6. Maximum grip (0.0m):"
gz topic -t /model/jetrover/joint/left_finger_joint/0/cmd_pos -m gz.msgs.Double -p 'data: 0.0'
gz topic -t /model/jetrover/joint/right_finger_joint/0/cmd_pos -m gz.msgs.Double -p 'data: 0.0'
sleep 3

echo "7. Back to full open:"
gz topic -t /model/jetrover/joint/left_finger_joint/0/cmd_pos -m gz.msgs.Double -p 'data: 0.12'
gz topic -t /model/jetrover/joint/right_finger_joint/0/cmd_pos -m gz.msgs.Double -p 'data: 0.12'
sleep 2

echo "=== Gripper test complete! ==="
echo "If gripper moved through all positions, the fix was successful!"
