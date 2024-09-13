FROM osrf/ros:noetic-desktop-full

# Update
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update -y && apt-get upgrade -y

RUN apt-get install -y python3-catkin-tools \
    python3-osrf-pycommon \
    python3-wstool \
    ros-noetic-gazebo-ros \
    ros-noetic-ros-controllers \
    ros-noetic-ackermann-msgs \
    ros-noetic-geometry2 \
    ros-noetic-hector-gazebo \
    ros-noetic-hector-models \
    ros-noetic-jsk-rviz-plugins \
    ros-noetic-ros-control \
    ros-noetic-velodyne-simulator \
    git \
    software-properties-common

# Dependencies for Voxblox
RUN apt-get install -y \
    ros-noetic-cmake-modules \
    protobuf-compiler \
    rsync \
    autoconf

# Install Voxblox
# RUN mkdir -p ~/voxblox_ws/src && \
#     cd ~/voxblox_ws  && \
#     catkin init && \
#     catkin config --extend /opt/ros/noetic && \
#     catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release && \
#     catkin config --merge-devel && \
#     cd ~/voxblox_ws/src/ && \
#     git clone https://github.com/ethz-asl/voxblox.git && \
#     wstool init . ./voxblox/voxblox_https.rosinstall && \
#     wstool update && \
#     catkin build voxblox_ros && \
#     source ../devel/setup.zsh

# Installing zsh
RUN sudo apt-get install -y zsh
# Installing Oh-My-Zsh
# Install Oh my zsh
RUN sh -c "$(wget -O- https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" -- \
    -p git \
    -p docker-compose \
    -p docker \
    -p https://github.com/zsh-users/zsh-autosuggestions \
    -p https://github.com/zsh-users/zsh-completions \
    -p https://github.com/zsh-users/zsh-syntax-highlighting 

# Installing required Python libraries
RUN apt-get install -y python3-pip
# Install do-mpc
RUN pip3 install do-mpc[full]
# Install TOPP-RA for trajectory planning
RUN pip3 install toppra

# Configuring Access to Displays
RUN mkdir -m 0700 /tmp/runtime-root && \
    apt-get install -y \
    dbus-x11 \
    libgl1-mesa-dri \
    libgl1-mesa-glx
ENV XDG_RUNTIME_DIR=/tmp/runtime-root
ENV DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus
ENV LIBGL_ALWAYS_SOFTWARE=1

WORKDIR /
CMD ["/bin/zsh"]