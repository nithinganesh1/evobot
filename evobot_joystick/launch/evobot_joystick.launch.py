from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    evobot_node = Node(
        package='evobot_joystick',
        executable='joy_diff_drive',
        name='joy_diff_drive',
        output='screen'
    )

    return LaunchDescription([
        joy_node,
        evobot_node
    ])
