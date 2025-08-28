#!/bin/bash

echo "📷 GAZEBO IMAGE DISPLAY - CAMERA POV VIEWER"
echo "============================================="
echo ""
echo "🎯 METHOD 1: IMAGE DISPLAY PLUGIN (BEST)"
echo "   In your Gazebo window:"
echo "   1. Top menu → 'Window' → 'Image Display'"
echo "   2. OR click the 'Plugins' dropdown → 'Image Display'"
echo "   3. A new window will open showing camera feed!"
echo "   4. In the Image Display window, select topic: /camera/image"
echo ""

echo "🎯 METHOD 2: CAMERA WIDGET"
echo "   1. Right-click in Gazebo 3D view"
echo "   2. Select 'Camera Info' or 'Camera'"
echo "   3. Choose 'workspace_camera' from list"
echo "   4. Live camera view appears in panel"
echo ""

echo "🎯 METHOD 3: DEVELOPER VIEW"
echo "   1. Press F11 to cycle through available cameras"
echo "   2. Your main viewport switches to camera POV"
echo "   3. Press F11 again to cycle back"
echo ""

echo "📺 VERIFYING CAMERA TOPIC:"
if pgrep -x "gz" > /dev/null; then
    echo "✅ Gazebo is running"
    sleep 2
    CAMERA_TOPIC=$(gz topic -l | grep -i "camera\|image")
    if [ ! -z "$CAMERA_TOPIC" ]; then
        echo "✅ Camera topics found:"
        echo "$CAMERA_TOPIC"
    else
        echo "❌ No camera topics yet - try Image Display GUI method"
    fi
else
    echo "❌ Gazebo not running"
fi

echo ""
echo "🎮 STEP-BY-STEP INSTRUCTIONS:"
echo "   1. Make sure Gazebo is open with your simulation"
echo "   2. Look for 'Window' in the top menu bar"
echo "   3. Click 'Window' → 'Image Display'"
echo "   4. New popup window = LIVE CAMERA FEED!"
echo "   5. Select camera topic if multiple options"
echo ""
echo "💡 TIP: If Image Display not in Window menu:"
echo "   • Try 'Plugins' menu → 'Image Display'"
echo "   • Or right-click → 'Camera Info'"
echo ""
echo "📍 YOUR CAMERA DETAILS:"
echo "   • Name: workspace_camera" 
echo "   • Topic: /camera/image"
echo "   • View: Top-down of tic-tac-toe board"
echo "   • Resolution: 1920x1080"
