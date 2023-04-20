# launch file for to spawn in the rover in Gazebo and loads the controllers in the "launch" directory.

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():
    return LaunchDescription([
        # starts Gazebo with an empty world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/gazebo.launch.py']),
            launch_arguments={'world': 'worlds/empty.world.yaml'}.items(),
        ),
        
        # spawns the rover model
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'rover', '-x', '0', '-y', '0', '-z', '0.1', '-file', "$(find rover_description)/urdf/rover.xacro"],
            output='screen'
        ),
        
        # loads and start the controllers
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/rover_control_launch.py']),
        ),
    ])
