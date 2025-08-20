#!/bin/bash

echo "=== SIMPLE GRIPPER TEST ==="
echo "Testing basic open/close functionality..."

echo "Step 1: Opening gripper to 6cm..."
gz topic -t /model/jetrover/joint/left_finger_joint/0/cmd_pos -m gz.msgs.Double -p 'data: 0.06'
gz topic -t /model/jetrover/joint/right_finger_joint/0/cmd_pos -m gz.msgs.Double -p 'data: 0.06'
sleep 2

echo "Step 2: Closing gripper to 2cm..."
gz topic -t /model/jetrover/joint/left_finger_joint/0/cmd_pos -m gz.msgs.Double -p 'data: 0.02'
gz topic -t /model/jetrover/joint/right_finger_joint/0/cmd_pos -m gz.msgs.Double -p 'data: 0.02'
sleep 2

echo "Step 3: Fully closing gripper to 0.5cm..."
gz topic -t /model/jetrover/joint/left_finger_joint/0/cmd_pos -m gz.msgs.Double -p 'data: 0.005'
gz topic -t /model/jetrover/joint/right_finger_joint/0/cmd_pos -m gz.msgs.Double -p 'data: 0.005'
sleep 2

echo "Step 4: Opening gripper to 4cm..."
gz topic -t /model/jetrover/joint/left_finger_joint/0/cmd_pos -m gz.msgs.Double -p 'data: 0.04'
gz topic -t /model/jetrover/joint/right_finger_joint/0/cmd_pos -m gz.msgs.Double -p 'data: 0.04'
sleep 2

echo "Step 5: Fully opening gripper to 7cm..."
gz topic -t /model/jetrover/joint/left_finger_joint/0/cmd_pos -m gz.msgs.Double -p 'data: 0.07'
gz topic -t /model/jetrover/joint/right_finger_joint/0/cmd_pos -m gz.msgs.Double -p 'data: 0.07'
sleep 2

echo "=== TEST COMPLETE ==="
echo "If you saw the fingers moving, the gripper is working!"
