# Black O Piece Pick & Place - Quick Commands

## **Precise O Piece Coordinates:**
- **Target**: Black O piece at `(0.048, -0.157, 0.432)`
- **Destination**: Board center at `(-0.122, -0.071, 0.450)`
- **Robot Base**: `(-0.026, 0.218, 0.684)`

## **Step-by-Step Commands:**

### **1. Complete Sequence (Recommended)**
```bash
./control_arm.sh full
```
**Executes full pick-and-place automatically!**

### **2. Manual Step-by-Step**
```bash
./control_arm.sh home      # Start in safe position
./control_arm.sh open      # Open gripper 
./control_arm.sh prepick   # Move above O piece
./control_arm.sh pick      # Lower to grab piece
./control_arm.sh close     # Close gripper around piece
./control_arm.sh lift      # Lift piece up
./control_arm.sh preplace  # Move above board center
./control_arm.sh place     # Lower to place on board
./control_arm.sh open      # Release piece
./control_arm.sh home      # Return to safe position
```

### **3. Individual Joint Testing**
```bash
./control_arm.sh test 1 0.3    # Test joint1 (base rotation)
./control_arm.sh test 2 -0.8   # Test joint2 (shoulder)  
./control_arm.sh test 3 1.4    # Test joint3 (elbow)
./control_arm.sh test 4 -0.6   # Test joint4 (wrist)
./control_arm.sh test 5 0.0    # Test joint5 (end effector)
```

## **Key Joint Positions for O Piece:**

### **Pre-Pick Position (Above Piece)**
- Joint1: `0.3` (rotate toward piece)
- Joint2: `-0.8` (lower shoulder)
- Joint3: `1.4` (extend elbow)
- Joint4: `-0.6` (adjust wrist pitch)
- Joint5: `0.0` (straight)

### **Pre-Place Position (Above Board)**
- Joint1: `-0.1` (rotate toward board)
- Joint2: `-0.6` (adjust height)
- Joint3: `1.0` (adjust reach)
- Joint4: `-0.4` (adjust wrist)
- Joint5: `0.0` (straight)

## **Safety Notes:**
- Always start with `./control_arm.sh home`
- Watch the robot during operation
- Use individual commands for fine-tuning positions
- Gripper opens to `0.08m` (8cm) for piece clearance

## **Usage Examples:**
```bash
# Just pick up the piece (no placing)
./control_arm.sh home && ./control_arm.sh open && ./control_arm.sh prepick && ./control_arm.sh pick && ./control_arm.sh close && ./control_arm.sh lift

# Just place (if already holding piece)
./control_arm.sh preplace && ./control_arm.sh place && ./control_arm.sh open && ./control_arm.sh home

# Quick gripper test
./control_arm.sh open && sleep 2 && ./control_arm.sh close
```

**Status: Ready for black O piece manipulation! 🤖✅**
