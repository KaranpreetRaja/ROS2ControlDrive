import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

# ros2 launch rover_control state_pub_launch.py
# ros2 launch gazebo_ros gazebo.launch.py world:=src/robot_testing/worlds/world6.world
# ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity rover_brother


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'rover_control'
    file_subpath = 'urdf/driverover.urdf.xacro'

    # Use xacro to process the file
    xacro_file = os.path.join(
        get_package_share_directory(pkg_name), file_subpath)
    robot_desc = xacro.process_file(xacro_file).toxml()

    # if xml, uncomment
    # with open(xacro_file, 'r') as infp:
    #     robot_desc = infp.read()

    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True,
                     'use_ros2_control': True}]  # add other parameters here if required
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'extra_gazebo_args': '--ros-args'}.items()
    )
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'rover_brother'],
        output='screen')

    drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["drive_controller"],
    )

    # Run the node
    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        drive_spawner
    ])
