# Build-and-test image for machines without a native ROS 2 install.
# Not needed to build or run the project natively (see docs/setup.md).
# ROS_DISTRO selects the setup: jazzy (Ubuntu 24.04, Gazebo Harmonic, the default) or lyrical (Ubuntu 26.04,
# Gazebo Jetty).
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base
ARG ROS_DISTRO

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-ros-gz \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-robot-state-publisher \
        ros-${ROS_DISTRO}-robot-localization \
        ros-${ROS_DISTRO}-teleop-twist-keyboard \
        ros-${ROS_DISTRO}-tf2-tools \
        ros-${ROS_DISTRO}-tf2-ros-py \
        ros-${ROS_DISTRO}-rviz2 \
        ros-${ROS_DISTRO}-joint-state-publisher-gui \
        ros-${ROS_DISTRO}-ament-cmake-pytest \
        ros-${ROS_DISTRO}-ament-lint-auto \
        ros-${ROS_DISTRO}-ament-cmake-flake8 \
        ros-${ROS_DISTRO}-ament-cmake-pep257 \
        ros-${ROS_DISTRO}-ament-cmake-xmllint \
        ros-${ROS_DISTRO}-ament-flake8 \
        ros-${ROS_DISTRO}-ament-pep257 \
        ros-${ROS_DISTRO}-ament-cmake-gtest \
        ros-${ROS_DISTRO}-ament-cmake-cpplint \
        ros-${ROS_DISTRO}-ament-cmake-lint-cmake \
        ros-${ROS_DISTRO}-nav2-core \
        ros-${ROS_DISTRO}-nav2-util \
        ros-${ROS_DISTRO}-nav2-msgs \
        ros-${ROS_DISTRO}-nav2-costmap-2d \
        ros-${ROS_DISTRO}-nav2-planner \
        ros-${ROS_DISTRO}-nav2-map-server \
        ros-${ROS_DISTRO}-nav2-amcl \
        ros-${ROS_DISTRO}-nav2-lifecycle-manager \
        ros-${ROS_DISTRO}-nav2-navfn-planner \
        ros-${ROS_DISTRO}-nav2-smac-planner \
        ros-${ROS_DISTRO}-nav2-theta-star-planner \
        ros-${ROS_DISTRO}-nav2-rviz-plugins \
        ros-${ROS_DISTRO}-nav2-controller \
        ros-${ROS_DISTRO}-dwb-core \
        ros-${ROS_DISTRO}-dwb-critics \
        ros-${ROS_DISTRO}-dwb-plugins \
        ros-${ROS_DISTRO}-nav2-regulated-pure-pursuit-controller \
        ros-${ROS_DISTRO}-nav2-bt-navigator \
        ros-${ROS_DISTRO}-nav2-behavior-tree \
        ros-${ROS_DISTRO}-nav2-behaviors \
        ros-${ROS_DISTRO}-nav2-simple-commander \
        liburdfdom-tools \
        python3-numpy \
        python3-pil \
        python3-pytest \
        python3-yaml \
        python3-venv \
        libegl1 \
        libgl1-mesa-dri \
        mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# PyTorch for the RAFT flow backends, in a virtual environment that still sees the system (ROS) packages.
# CPU wheels only. pip must not replace the numpy the ROS binaries are built against.
RUN python3 -m venv --system-site-packages /opt/venv \
    && /opt/venv/bin/pip install --no-cache-dir torch torchvision --index-url https://download.pytorch.org/whl/cpu
ENV PATH=/opt/venv/bin:$PATH

# Fetch the RAFT weights now, so that runs need no network.
RUN python3 -c "from torchvision.models.optical_flow import raft_small, raft_large, Raft_Small_Weights, Raft_Large_Weights; raft_small(weights=Raft_Small_Weights.DEFAULT); raft_large(weights=Raft_Large_Weights.DEFAULT)"

RUN rosdep update --rosdistro ${ROS_DISTRO}

# Software rendering for headless Gazebo sensors.
ENV LIBGL_ALWAYS_SOFTWARE=1
WORKDIR /ws
