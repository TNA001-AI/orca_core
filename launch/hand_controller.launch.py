from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='orca_core',
            executable='hand_controller',
            name='hand_controller',
            output='screen',
            emulate_tty=True,
        ),
    ]) 