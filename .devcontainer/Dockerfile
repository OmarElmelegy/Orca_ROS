FROM osrf/ros:noetic-desktop-full

# Install basic tools
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-opencv \
    python3-numpy \
    python3-yaml \
    libi2c-dev \
    i2c-tools \
    vim \
    nano \
    wget \
    curl \
    lsb-release \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# Install ROS packages
RUN apt-get update && apt-get install -y \
    ros-noetic-mavros \
    ros-noetic-mavros-extras \
    ros-noetic-mavros-msgs \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-camera-info-manager \
    ros-noetic-tf2 \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-rviz \
    ros-noetic-rqt \
    ros-noetic-rqt-common-plugins \
    ros-noetic-usb-cam \
    ros-noetic-realsense2-camera \
    ros-noetic-realsense2-description \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-executive-smach \
    ros-noetic-smach-ros \
    ros-noetic-smach-viewer \
    && rm -rf /var/lib/apt/lists/*

# Install MAVLink
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    chmod +x install_geographiclib_datasets.sh && \
    ./install_geographiclib_datasets.sh && \
    rm install_geographiclib_datasets.sh

# Install Python packages
RUN pip3 install --upgrade pip && \
    pip3 install \
    pymavlink \
    pyserial \
    numpy \
    opencv-python \
    matplotlib \
    autopep8 \
    pylint

# Install RealSense SDK
RUN apt-get update && apt-get install -y \
    software-properties-common \
    && apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
    && add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u \
    && apt-get install -y \
    librealsense2-dev \
    librealsense2-utils \
    && rm -rf /var/lib/apt/lists/*

# Create a workspace directory
WORKDIR /workspaces/Orca_ws

# Set up environment
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Create entrypoint script
RUN echo '#!/bin/bash' > /entrypoint.sh && \
    echo 'set -e' >> /entrypoint.sh && \
    echo 'source /opt/ros/noetic/setup.bash' >> /entrypoint.sh && \
    echo 'if [ -f "/workspaces/Orca_ws/devel/setup.bash" ]; then' >> /entrypoint.sh && \
    echo '    source /workspaces/Orca_ws/devel/setup.bash' >> /entrypoint.sh && \
    echo 'fi' >> /entrypoint.sh && \
    echo 'exec "$@"' >> /entrypoint.sh && \
    chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
