# JetRover Robot Control - Working Commands ✅

## **SUCCESS! Direct Gazebo Commands Work Perfectly**

### **Individual Joint Control**
```bash
# Base rotation (joint1): -2.0 to 2.0 radians
gz topic -t /model/jetrover/joint/joint1/0/cmd_pos -m gz.msgs.Double -p 'data: 0.5'

# Shoulder (joint2): -1.5 to 1.5 radians  
gz topic -t /model/jetrover/joint/joint2/0/cmd_pos -m gz.msgs.Double -p 'data: 0.8'

# Elbow (joint3): -2.0 to 0.5 radians
gz topic -t /model/jetrover/joint/joint3/0/cmd_pos -m gz.msgs.Double -p 'data: -1.2'

# Wrist pitch (joint4): -1.5 to 1.5 radians
gz topic -t /model/jetrover/joint/joint4/0/cmd_pos -m gz.msgs.Double -p 'data: 0.0'

# End effector (joint5): -3.14 to 3.14 radians
gz topic -t /model/jetrover/joint/joint5/0/cmd_pos -m gz.msgs.Double -p 'data: 0.0'
```

### **Gripper Control**
```bash
# Open gripper (0.15m max opening)
gz topic -t /model/jetrover/joint/left_finger_joint/0/cmd_pos -m gz.msgs.Double -p 'data: 0.1'
gz topic -t /model/jetrover/joint/right_finger_joint/0/cmd_pos -m gz.msgs.Double -p 'data: 0.1'

# Close gripper
gz topic -t /model/jetrover/joint/left_finger_joint/0/cmd_pos -m gz.msgs.Double -p 'data: 0.0'
gz topic -t /model/jetrover/joint/right_finger_joint/0/cmd_pos -m gz.msgs.Double -p 'data: 0.0'
```

### **Automated Script Commands**
```bash
# Use the control script for complex movements
./control_arm.sh home     # Move to safe home position
./control_arm.sh open     # Open gripper
./control_arm.sh close    # Close gripper
./control_arm.sh pick     # Move to pick position and grab
./control_arm.sh place    # Move to place position and release
./control_arm.sh test     # Run full pick-and-place sequence
```

## **Coordinate System - Game Piece Locations**
Based on tictactoe_NEW3.2.sdf file positions:

### **O Pieces (Left side)**
```bash
# O1: (-0.4, -0.1, 0.54)
# O2: (-0.4, -0.1, 0.60) 
# O3: (-0.4, -0.1, 0.66)
```

### **X Pieces (Right side)**
```bash
# X1: (0.4, -0.1, 0.54)
# X2: (0.4, -0.1, 0.60)
# X3: (0.4, -0.1, 0.66)
```

### **Game Grid Centers**
```bash
# Grid positions (3x3):
# Row 1: (-0.1, 0.1, 0.51), (0.0, 0.1, 0.51), (0.1, 0.1, 0.51)
# Row 2: (-0.1, 0.0, 0.51), (0.0, 0.0, 0.51), (0.1, 0.0, 0.51)  
# Row 3: (-0.1, -0.1, 0.51), (0.0, -0.1, 0.51), (0.1, -0.1, 0.51)
```

## **Quick Start Guide**
1. **Start Gazebo**: `gz sim tictactoe_NEW3.2.sdf`
2. **Test gripper**: `./control_arm.sh open`
3. **Test movement**: `./control_arm.sh test`
4. **Manual control**: Use individual `gz topic` commands above

## **Problem Solved!** ✅
- **Issue**: ROS2 Jazzy rejects topic names with `/0/` (numbers)
- **Solution**: Direct Gazebo commands bypass ROS2 validation
- **Result**: Full robot arm control working perfectly!

**Status: ROBOT ARM FULLY OPERATIONAL** 🤖✅
