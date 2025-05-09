FROM aerostack2/humble:1.1.2 

# Set tmux mouse on
RUN echo "set -g mouse on" > ~/.tmux.conf 

# Clone demo project
WORKDIR /root/

RUN apt-get update && apt-get install -y git python3-pip libxcb-xinerama0 libxcb-cursor0 ros-humble-image-transport ros-humble-image-geometry ros-humble-rmw-cyclonedds-cpp
RUN pip3 install --upgrade pip
RUN apt install udev usbutils -y

WORKDIR /root/aerostack2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash; colcon build --symlink-install --packages-select lidar_to_image_depth"

# add source to bashrc
RUN echo "source /root/aerostack2_ws/install/setup.bash" >> ~/.bashrc

WORKDIR /root/simulator