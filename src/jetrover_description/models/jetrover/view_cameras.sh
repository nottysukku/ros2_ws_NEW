#!/bin/bash

echo "🎯 CAMERA FEED VIEWER for Gazebo Simulation"
echo "============================================="
echo ""
echo "📷 Method 1 - IN GAZEBO GUI:"
echo "   1. Right-click in 3D viewport"
echo "   2. Select 'Camera' → Choose camera sensor"
echo "   3. Or press Ctrl+Alt+C to cycle cameras"
echo ""
echo "📷 Method 2 - Available Cameras:"
echo "   - workspace_camera (overhead view)"
echo "   - side_view_camera (side view)" 
echo ""
echo "📷 Method 3 - Check Camera Status:"
echo "   Running gz topic check..."

# Check if cameras are publishing
sleep 2
gz topic -l | grep -i camera

if [ $? -eq 0 ]; then
    echo "✅ Camera topics found!"
    echo "Use: gz topic -e -t /camera_topic_name"
else
    echo "❌ No camera topics - Use Gazebo GUI method"
    echo ""
    echo "🔧 SOLUTION: In Gazebo window:"
    echo "   1. Press F11 for camera views"
    echo "   2. Or Window → Camera Inspector"
    echo "   3. Select workspace_camera or side_view_camera"
fi

echo ""
echo "🎮 CAMERA CONTROLS in Gazebo:"
echo "   Mouse drag = Rotate view"
echo "   Scroll = Zoom in/out"
echo "   WASD = Move camera position"
echo "   F11 = Toggle camera modes"
