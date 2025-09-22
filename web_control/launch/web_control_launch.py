from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='web_control',
            executable='web_gui',
            name='web_gui',
            output='screen'
        ),
        Node(
            package='web_control',
            executable='cmd_val_for_ard',
            name='cmd_val_for_ard',
            output='screen'
        )
    ])
