FROM aerostack2/humble:1.1.2 

# Set tmux mouse on
RUN echo "set -g mouse on" > ~/.tmux.conf 

# Clone demo project
WORKDIR /root/

# Update package lists and install basic dependencies
RUN apt-get update && apt-get install --fix-missing -y \
    git \
    python3-pip \
    libxcb-xinerama0 \
    libxcb-cursor0 \
    udev \
    usbutils

# Install ROS2 packages with error handling
RUN apt-get update && apt-get install -y \
    ros-humble-image-transport \
    ros-humble-image-geometry \
    || (apt-get update && apt-get install -y ros-humble-image-transport ros-humble-image-geometry)

# Try to install cyclonedds packages, but don't fail if not available
RUN apt-get update && apt-get install -y ros-humble-rmw-cyclonedds-cpp || echo "CycloneDDS packages not available, continuing without them"

RUN pip3 install --upgrade pip

WORKDIR /root/aerostack2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash"

# add source to bashrc
RUN echo "source /root/aerostack2_ws/install/setup.bash" >> ~/.bashrc

WORKDIR /root/simulator