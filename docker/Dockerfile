# Build-and-test image for machines without a native ROS 2 Jazzy install.
# Not needed to build or run the project on Ubuntu 24.04 (see docs/setup.md).
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-gz \
        ros-jazzy-xacro \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-robot-localization \
        ros-jazzy-teleop-twist-keyboard \
        ros-jazzy-tf2-tools \
        ros-jazzy-tf2-ros-py \
        ros-jazzy-rviz2 \
        ros-jazzy-joint-state-publisher-gui \
        ros-jazzy-ament-cmake-pytest \
        ros-jazzy-ament-lint-auto \
        ros-jazzy-ament-cmake-flake8 \
        ros-jazzy-ament-cmake-pep257 \
        ros-jazzy-ament-cmake-xmllint \
        ros-jazzy-ament-flake8 \
        ros-jazzy-ament-pep257 \
        ros-jazzy-ament-cmake-gtest \
        ros-jazzy-ament-cmake-cpplint \
        ros-jazzy-ament-cmake-lint-cmake \
        ros-jazzy-nav2-core \
        ros-jazzy-nav2-util \
        ros-jazzy-nav2-msgs \
        ros-jazzy-nav2-costmap-2d \
        ros-jazzy-nav2-planner \
        ros-jazzy-nav2-map-server \
        ros-jazzy-nav2-amcl \
        ros-jazzy-nav2-lifecycle-manager \
        ros-jazzy-nav2-navfn-planner \
        ros-jazzy-nav2-smac-planner \
        ros-jazzy-nav2-theta-star-planner \
        ros-jazzy-nav2-rviz-plugins \
        liburdfdom-tools \
        python3-numpy \
        python3-pil \
        python3-pytest \
        python3-yaml \
        libegl1 \
        libgl1-mesa-dri \
        mesa-utils \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep update --rosdistro jazzy

# Software rendering for headless Gazebo sensors.
ENV LIBGL_ALWAYS_SOFTWARE=1
WORKDIR /ws
