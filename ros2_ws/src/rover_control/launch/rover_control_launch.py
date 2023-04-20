# launches file to set up the ros2_control node and the controller spawner in the current "launch" directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{
                'controller_manager': {
                    'update_rate': 100.0
                },
                'hardware': {
                    'components': [{
                        'name': 'rover_simulated_hardware',
                        'type': 'system',
                        'class': 'ros2_control_demo_hardware/PositionActuatorHardwareSim',
                        'parameters': {
                            'joints': ['front_left_wheel_joint', 'front_right_wheel_joint', 'rear_left_wheel_joint', 'rear_right_wheel_joint']
                        }
                    }]
                }
            }],
            output='screen'
        ),
        Node(
            package='rover_control',
            executable='controller_spawner',
            arguments=['joint_state_controller', 'diff_drive_controller'],
            output='screen'
        )
    ])
