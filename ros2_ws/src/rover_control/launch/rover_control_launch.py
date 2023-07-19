# launches file to set up the ros2_control node and the controller spawner in the current "launch" directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rover_control',
            executable='ps4_controller_node',
            output='screen'
        )
    ])


'''
   cd ~/ros2_ws/src
   ros2 pkg create ps4_control --build-type ament_python

'''
