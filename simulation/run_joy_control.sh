#!/bin/bash

# Make sure ROS2 is sourced
source /opt/ros/humble/setup.bash

# Fix any Windows line ending issues
echo "Fixing line ending issues in Python files..."
apt-get update && apt-get install -y dos2unix || true
find /root/aerostack2_ws/src/drone_joy_control -name "*.py" -exec dos2unix {} \; || true
chmod +x /root/aerostack2_ws/src/drone_joy_control/drone_joy_control/*.py || true
chmod +x /root/aerostack2_ws/src/drone_joy_control/launch/*.py || true

# Build the joystick controller package
cd /root/aerostack2_ws
source /root/aerostack2_ws/install/setup.bash

echo "Building drone_joy_control package..."
colcon build --packages-select drone_joy_control

# Source the workspace again to include new package
source /root/aerostack2_ws/install/setup.bash

# Run the joystick controller
echo "Starting joystick controller for drone0..."
ros2 launch drone_joy_control joy_control.launch.py drone_id:=drone0 