# Setup

## Supported setups

| Ubuntu | ROS 2 | Gazebo | Nav2 | status |
|---|---|---|---|---|
| 24.04 | Jazzy | Harmonic | 1.3 | primary: all results in the docs come from it |
| 26.04 | Lyrical | Jetty | 1.5 | supported: builds, tests and runs the same launch files |

The same code builds on both; CI builds and tests both. Below, `<distro>` is `jazzy` or `lyrical`.

## Install ROS 2 and Gazebo

Follow the official install guide for your setup
([Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) on Ubuntu 24.04,
[Lyrical](https://docs.ros.org/en/lyrical/Installation/Ubuntu-Install-Debs.html) on Ubuntu 26.04), then install the
packages this project uses. Gazebo comes with `ros-<distro>-ros-gz` (Harmonic on Jazzy, Jetty on Lyrical):

```bash
# Jazzy on Ubuntu 24.04
sudo apt install ros-jazzy-desktop ros-jazzy-ros-gz python3-colcon-common-extensions python3-rosdep
# Lyrical on Ubuntu 26.04
sudo apt install ros-lyrical-desktop ros-lyrical-ros-gz python3-colcon-common-extensions python3-rosdep
```

## Get and build the workspace

```bash
mkdir -p ~/ros_ws && cd ~/ros_ws
git clone https://github.com/harsh-vsharma/rapid-nav.git
cd rapid-nav
sudo rosdep init   # only once per machine; skip if it says it is already initialised
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

Every new terminal needs `source /opt/ros/<distro>/setup.bash` and `source install/setup.bash`.

## Shared networks

ROS 2 nodes find each other on the local network. On a shared network (a lab, a campus Wi-Fi), two people running
the simulation at the same time see each other's topics. Give each machine its own domain, in every terminal (or in
`~/.bashrc`):

```bash
export ROS_DOMAIN_ID=42   # any number from 1 to 101, different per person
```

`export ROS_LOCALHOST_ONLY=1` (Jazzy) or `export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` (Jazzy and Lyrical) keeps
the traffic on the machine itself.

## Updating

Run `git pull`, then `rosdep install --from-paths src --ignore-src -y` and `colcon build --symlink-install` again.

## System check

If something does not build or run, paste the output of these commands when you ask for help:

```bash
lsb_release -ds; printenv ROS_DISTRO; gz sim --versions; nproc; free -h | head -2
nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv 2>/dev/null || echo "no NVIDIA GPU"
python3 -c "import torch; print('torch', torch.__version__, 'cuda', torch.cuda.is_available())" 2>/dev/null || echo "no torch"
```

## Python packages for perception

PyTorch setup is described with the perception package.
