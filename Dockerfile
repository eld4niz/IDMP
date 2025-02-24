# Use an official ROS Noetic base image
FROM ros:noetic-ros-core-focal

# Set environment variables for ROS
ENV ROS_DISTRO=noetic
ENV DEBIAN_FRONTEND=noninteractive

# Update and install dependencies
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    build-essential \
    cmake \
    g++ \
    libeigen3-dev \
    ros-noetic-sensor-filters \
    ros-noetic-point-cloud2-filters \
    ros-noetic-robot-body-filter \
    ros-noetic-rviz \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-tf2-eigen \
    ros-noetic-image-transport \
    ros-noetic-camera-info-manager \
    ros-noetic-image-geometry \
    ros-noetic-depth-image-proc \
    git \
    x11-apps \
    libopencv-dev

# Optional: Install OpenMP for multithreading
RUN apt-get install -y libomp-dev

# Install Python dependencies
RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install --ignore-installed numpy opencv-python open3d rosnumpy

# Create the catkin workspace
WORKDIR /root/catkin_ws/src

# Clone the IDMP package
RUN git clone https://github.com/UTS-RI/IDMP.git

# Navigate to the workspace root and build the workspace
WORKDIR /root/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Source the workspace in the container's environment
RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Default command
CMD ["/bin/bash"]

ENV DISPLAY=:0
