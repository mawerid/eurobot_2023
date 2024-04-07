# Use ROS2 humble from osrf/ros image
FROM ros:humble-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install necessary packages for building OpenCV
RUN apt-get update && apt-get install -y \
    git build-essential cmake \
    libcanberra-gtk-module libcanberra-gtk3-module fuse3 libfuse2 libqt5svg5-dev \
    python3-pip python3-opencv python3-tk python3-pyqt5.qtwebengine python3-rosdep

# Install OpenCV
RUN pip3 install opencv-contrib-python

# Source ROS2 setup.bash
RUN /bin/bash -c "source /opt/ros/humble/setup.bash"

# Create and initialize workspace
RUN mkdir -p /workspace/src
WORKDIR /workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    apt update && apt upgrade -y && apt install ros-humble-micro-ros-msgs -y"

RUN /bin/bash -c "git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup"

RUN /bin/bash -c "rosdep install --from-paths src --ignore-src --rosdistro humble -y"
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

RUN sudo apt-get install -y ros-humble-image-common ros-humble-vision-opencv ros-humble-vision-rviz2 xauth


RUN echo "source /workspace/install/setup.sh" >> ~/.bashrc

# Remove display warnings
RUN mkdir /tmp/runtime-root
ENV XDG_RUNTIME_DIR "/tmp/runtime-root"
RUN chmod -R 0700 /tmp/runtime-root
ENV NO_AT_BRIDGE 1

# Set entrypoint to bash
CMD ["/bin/bash"]
