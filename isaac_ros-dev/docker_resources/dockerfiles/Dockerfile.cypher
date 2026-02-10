ARG BASE_IMAGE
FROM ${BASE_IMAGE}

# Install dependencies
RUN apt-get update && apt-get install -y \
    nano \
    v4l-utils \
    libwebsocketpp-dev \
    libgstreamer1.0-dev \
    python3-colcon-clean \
    gir1.2-gstreamer-1.0 \
    libgstreamer-plugins-base1.0-dev \
    ros-humble-camera-ros \
    ros-humble-magic-enum \
    ros-humble-librealsense2 \
    ros-humble-foxglove-msgs \
    ros-humble-apriltag-msgs \
    ros-humble-tf-transformations \
    ros-humble-isaac-ros-image-proc \
    ros-humble-isaac-ros-h264-decoder \
    ros-humble-ament-cmake-clang-format \
    ros-humble-apriltag-msgs

# Python dependencies
# pip3 install -U jetson-stats
RUN pip3 install --ignore-installed transforms3d
