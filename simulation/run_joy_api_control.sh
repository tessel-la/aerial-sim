#!/bin/bash

# Fix any windows line ending issues
if [ -f /usr/bin/dos2unix ]; then
    find /root/aerostack2_ws/src/drone_joy_control -type f -name "*.py" -exec dos2unix {} \;
    find /root/aerostack2_ws/src/drone_joy_control -type f -name "*.sh" -exec dos2unix {} \;
    dos2unix "$0"
fi

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source /root/aerostack2_ws/install/setup.bash

# Set drone id - use first argument or default to "drone0"
# Note: Don't use ${0} as it's the script name, which causes an error
DRONE_ID=${1:-drone0}
cd /root/aerostack2_ws/ && colcon build --packages-select drone_joy_control
source /root/aerostack2_ws/install/setup.bash


# Launch the joy controller with API
ros2 launch drone_joy_control joy_api_control.launch.py drone_id:=$DRONE_ID 