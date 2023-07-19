# launches file to set up the ros2_control node and the controller spawner in the current "launch" directory

from launch import LaunchDescription
from launch_ros.actions import Node

'''
$ tree
.
├── CMakeLists.txt
├── config
│   └── rover_control.yaml
├── include
│   └── rover_control
├── launch
│   ├── ps4_launch.py
│   ├── rover_control_launch.py
│   ├── rover_gazebo_launch.py
│   └── state_pub_launch.py
├── package.xml
├── src
│   └── ps4_controller_node.py
└── urdf
    ├── driverover.urdf.xacro
    └── rover.urdf.xacro

6 directories, 10 files
'''


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rover_control',
            executable='ps4_controller_node',
            output='screen'
        )


    ])
