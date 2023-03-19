# ROS2_CONTROL DRIVE

This is a simple setup of a ROS2 workspace with Gazebo and ros2_control in order to implement a wheel/tank drive system for a rover that I am working on.

## Dependancies

Before getting started, the basic dependancies are ros 2 (in my case I am using the Humble Hawksbill release)

### ros2_control

This is the simple Binary Build, if you wish to persue a semi-binary build or a source build then please refer to the offcial repo [https://github.com/ros-controls/ros2_control](https://github.com/ros-controls/ros2_control)

#### Add the ROS2 apt repository to your system's list of package sources

```bash
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
```

#### Update package index and install ros2_control

```bash
sudo apt update && sudo apt install ros-$ROS_DISTRO-ros2-control
```

In my case, my `$ROS_DISTRO` is humble.

Remember to source ros2 when using ros2_control, this can be added to your `~/.bashrc` file for convince:

```bash
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
```
