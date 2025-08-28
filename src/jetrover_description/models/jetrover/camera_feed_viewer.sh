#!/bin/bash

echo "📷 OVERHEAD CAMERA FEED VIEWER"
echo "================================"
echo ""

# Method 1: Gazebo GUI Instructions
echo "🎯 METHOD 1 - INSIDE GAZEBO (EASIEST):"
echo "   1. In Gazebo window → Menu: Window → Camera"
echo "   2. OR press Ctrl+Shift+C"
echo "   3. Select 'workspace_camera' from dropdown"
echo "   4. Live video feed will appear in panel!"
echo ""

# Method 2: Check for camera topics
echo "🎯 METHOD 2 - CHECK CAMERA TOPICS:"
echo "   Checking for camera image topics..."
sleep 2

# Try to find camera topics
CAMERA_TOPICS=$(gz topic -l | grep -i camera)
IMAGE_TOPICS=$(gz topic -l | grep -i image)

if [ ! -z "$CAMERA_TOPICS" ] || [ ! -z "$IMAGE_TOPICS" ]; then
    echo "✅ Found camera-related topics:"
    gz topic -l | grep -E "(camera|image)"
    echo ""
    echo "📺 To view camera feed in terminal:"
    echo "   gz topic -e -t /camera/image"
    echo ""
    echo "💾 To save camera images:"
    echo "   Images auto-saved to: /tmp/camera_save_tutorial/"
else
    echo "❌ No camera topics found yet"
    echo ""
fi

echo "🎮 GAZEBO CAMERA CONTROLS:"
echo "   • Window → Camera (opens camera panel)"
echo "   • Ctrl+Shift+C (camera panel shortcut)"
echo "   • Right-click → Follow → overhead_camera"
echo "   • F11 (cycle camera views)"
echo ""

echo "📍 YOUR CAMERA DETAILS:"
echo "   • Name: workspace_camera"
echo "   • Position: Above tic-tac-toe board"
echo "   • Direction: Pointing straight down"
echo "   • Resolution: 1920x1080 @ 30fps"
echo "   • FOV: 60 degrees"

# Check if Gazebo is running
if pgrep -x "gz" > /dev/null; then
    echo ""
    echo "✅ Gazebo is running - Camera should be available!"
    echo "   Try: Window → Camera → workspace_camera"
else
    echo ""
    echo "❌ Gazebo not detected. Start with:"
    echo "   gz sim tictactoe_NEW3.5.sdf"
fi
