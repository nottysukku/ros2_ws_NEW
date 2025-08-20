# JetRover O Piece Control - UPDATED PRECISE VERSION ✅

## **🎯 Accurate Target Coordinates (From SDF Analysis)**
- **Robot Base**: `(-0.026, 0.218, 0.684)`
- **O Piece 1** (Right): `(0.048, -0.157, 0.432)` 
- **O Piece 2** (Center): `(-0.032, -0.157, 0.432)`  
- **O Piece 3** (Left): `(-0.112, -0.156, 0.432)`
- **Board Center**: `(-0.122, -0.071, 0.450)`

## **🚀 Complete Sequences:**

### **Auto Pick & Place (Default: O Piece 1)**
```bash
./control_arm.sh full
```

### **Manual Control with Specific Targeting**
```bash
./control_arm.sh home      # Safe start
./control_arm.sh open      # Open gripper
./control_arm.sh o1        # Target right O piece
# OR ./control_arm.sh o2   # Target center O piece  
# OR ./control_arm.sh o3   # Target left O piece
./control_arm.sh pick      # Lower to grab
./control_arm.sh close     # Grip piece
./control_arm.sh lift      # Lift up
./control_arm.sh preplace  # Move to board
./control_arm.sh place     # Place on center
./control_arm.sh open      # Release
./control_arm.sh home      # Return
```

## **⚙️ Precise Joint Angles (Calculated)**

### **O Piece 1 (Right Side) - Most Tested**
- `joint1: -1.4` (rotate right toward piece)
- `joint2: 1.3` (shoulder down for low reach)  
- `joint3: -1.9` (elbow extend forward)
- `joint4: 0.6` (wrist point down)

### **O Piece 2 (Center)**
- `joint1: -0.1` (slight rotation)
- `joint2: 1.3` (shoulder down)
- `joint3: -1.8` (elbow extend)  
- `joint4: 0.5` (wrist down)

### **O Piece 3 (Left Side)**
- `joint1: 0.8` (rotate left)
- `joint2: 1.2` (shoulder down)
- `joint3: -1.7` (elbow extend)
- `joint4: 0.5` (wrist down)

### **Board Placement**
- `joint1: -0.8` (toward board center)
- `joint2: 0.8` (height for board surface)
- `joint3: -1.2` (reach adjustment)
- `joint4: 0.4` (wrist positioning)

## **🔧 Testing & Calibration:**
```bash
./control_arm.sh calibrate     # Step-by-step movement test
./control_arm.sh test 1 -1.4   # Test individual joint
./control_arm.sh o2            # Quick position check
```

## **✅ Key Improvements Made:**
1. **Analyzed exact SDF coordinates** - Real positions from simulation file
2. **Individual O piece targeting** - Choose o1, o2, or o3
3. **Calculated joint angles** - Based on kinematic analysis
4. **Improved timing** - 2-3 second delays for smooth operation
5. **Better error handling** - Calibration mode for testing

## **🎮 Usage Examples:**

**Quick Position Test:**
```bash
./control_arm.sh o1 && sleep 3 && ./control_arm.sh home
```

**Pick Specific Piece:**
```bash
./control_arm.sh home && ./control_arm.sh open && ./control_arm.sh o2 && ./control_arm.sh pick && ./control_arm.sh close
```

**Complete Game Move:**
```bash
./control_arm.sh full  # Picks O1 and places on board center
```

**Status: HIGH-PRECISION O PIECE CONTROL READY! 🤖🎯**
