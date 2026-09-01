FROM ros:jazzy-ros-base

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    can-utils \
    iproute2 \
    python3-colcon-common-extensions \
    python3-jsonschema \
    python3-pytest \
    python3-yaml \
    ros-jazzy-ament-cmake-gtest \
    ros-jazzy-ament-cmake-pytest \
    ros-jazzy-hardware-interface \
    ros-jazzy-pluginlib \
    ros-jazzy-rclcpp \
    ros-jazzy-rclcpp-lifecycle \
    libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
