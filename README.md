# ROS2_CONTROL DRIVE

This is a simple setup of a ROS2 workspace with Gazebo and ros2_control in order to implement a wheel/tank drive system for a rover that I am working on.

## Dependancies

Before getting started, the basic dependancies are ROS2. I am using the Humble Hawksbill release, thus, in my case, my `$ROS_DISTRO` is `humble`.

### ros2_control

The installtion instructions below are for an installation from a binary build, if you wish to persue a semi-binary build or a source build then please refer to the offcial repo [https://github.com/ros-controls/ros2_control](https://github.com/ros-controls/ros2_control).

##### 1. Add the ROS2 apt repository to your system's list of package sources

```bash
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
```

Adding the ROS2 apt repo (which contains pre-built binaries for ROS2 packages) to your sources list will allow you to download and install ROS2 packages using the apt package manager.

##### 2. Update package index and install ros2_control

```bash
sudo apt update && sudo apt install ros-$ROS_DISTRO-ros2-control
```

Remember to source ros2 when using ros2_control, this can be added to your `~/.bashrc` file for convince:

```bash
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
```

### gazebo_ros_pkgs

The installtion instructions below are for an installation from a binary build, if you wish to presue a source build, please refer to the offical repo (make sure you are on the `ros2` branch) [https://github.com/ros-simulation/gazebo_ros_pkgs/tree/ros2](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/ros2)

##### Install from debian packages (Ubuntu)

```bash
sudo apt install ros-$ROS_DISTRO-gazebo-ros-pkgs
```
