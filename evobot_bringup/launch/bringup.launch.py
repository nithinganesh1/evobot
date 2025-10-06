from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Path to the RPLIDAR launch file
    rplidar_launch_file = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'launch',
        'rplidar_a1_launch.py'
    )

    return LaunchDescription([
        # Web GUI Node
        Node(
            package='web_control',
            executable='web_gui',
            name='web_gui',
            output='screen'
        ),

        # Command Value Node
        Node(
            package='web_control',
            executable='cmd_val_for_ard',
            name='cmd_val_for_ard',
            output='screen'
        ),

        # RPLIDAR A1 launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch_file),
        )
    ])

# Needed import
from ament_index_python.packages import get_package_share_directory
