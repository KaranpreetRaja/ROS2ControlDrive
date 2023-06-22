from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    gazebo_node = Node(
        package="gazebo_ros",
        executable="gazebo",
        arguments=["-s", "libgazebo_ros_factory.so"],
        parameters=[
            {"world_name": "$(find robot_testing)/worlds/world6.world"}]
    )

    state_pub_node = Node(
        package="rover_control",
        executable="state_pub_launch.py",
    )

    spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "rover_bro"],
    )

    ld.add_action(gazebo_node)
    ld.add_action(state_pub_node)
    ld.add_action(spawn_entity_node)

    return ld
