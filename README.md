# ROS2_CONTROL DRIVE

This is a simple setup of a ROS2 workspace with Gazebo and ros2_control in order to implement a wheel/tank drive system for a rover that I am working on.

## Dependancies

Before getting started, the basic dependancies are ROS2. I am using the Humble Hawksbill release, thus, in my case, my `$ROS_DISTRO` is `humble`.

### ros2_control

`ros2_control` is a ROS2 package that provides a plugin-based architecture for hardware and controller management. It defines interfaces for hardware interfaces and controllers, which are used to implement robot hardware and control algorithms.

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

### ros2_controllers

`ros2_controllers` is a package that provides a set of "ready-to-use" controllers for `ros2_control`. These include position controllers, velocity controllers, and effort controllers, which can be used to control robot joints, wheels, and/or other mechanisms. it has pre-built integration with various hardware interfaces supported by `ros2_control`.

The installtion instructions below are for an installation from a binary build, if you wish to persue a semi-binary build or a source build then please refer to the offcial repo [https://github.com/ros-controls/ros2_control](https://github.com/ros-controls/ros2_control).

##### 1. Add the ROS2 apt repository to your system's list of package sources (same as ros2_control step)

```bash
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
```

Adding the ROS2 apt repo (which contains pre-built binaries for ROS2 packages) to your sources list will allow you to download and install ROS2 packages using the apt package manager.

##### 2. Update package index and install ros2_control

```bash
sudo apt update && sudo apt install ros-$ROS_DISTRO-ros2-controllers
```

### Gazebo Fortress Installation
`Gazebo Fortress` (formerly Ignition) allows for simulation and enchanced development of ROS-based systems. [For more installation information](https://gazebosim.org/docs/fortress/install_ubuntu)

Install the necessary dependencies first:
```bash
sudo apt-get update
sudo apt-get install lsb-release wget gnupg
```

Now add the Gazebo Fortress apt repository to your system's list of package sources and install the `ignition-fortress` package:
```bash
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install ignition-fortress
```

### gazebo_ros_pkgs

`gazebo_ros_pkgs` is a set of ROS2 packages that provide integration between Gazebo and ROS2. This package includes plugins that allow Gazebo to communicate with ROS2 nodes, allowing it to publish and subscribe to ROS2 topics, and send and receive ROS2 services. `gazebo_ros_pkgs` also provides ROS2 interfaces for simulating robot sensors and actuators in Gazebo, such as cameras, lasers, and joints. It can be used to simulate robots and test control algorithms in a virtual environment before deploying them on real hardware.

The installtion instructions below are for an installation from a binary build, if you wish to presue a source build, please refer to the offical repo (make sure you are on the `ros2` branch) [https://github.com/ros-simulation/gazebo_ros_pkgs/tree/ros2](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/ros2)

##### Install from debian packages (Ubuntu)

```bash
sudo apt install ros-$ROS_DISTRO-gazebo-ros-pkgs
```


### Other Dependancies

Remember to source ros2 before you can actually use ros2, the source can be added to your `~/.bashrc` file for convince:

```bash
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
```

You will also need the gazebo_ros2_control package which can be installed as such:
```bash
sudo apt-get install ros-<ros2_distro>-gazebo-ros2-control
```