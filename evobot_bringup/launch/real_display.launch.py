from launch_ros.actions import Node
from launch import LaunchDescription
import xacro
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    share_dir = get_package_share_directory('evobot_description')

    # Load URDF
    xacro_file = os.path.join(share_dir, 'urdf', 'evobot.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # RViz config (for real robot visualization)
    rviz_config_file = os.path.join(share_dir, 'config', 'gazebo.rviz')

    # Robot state publisher (publishes TF from real robot’s joint states)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_urdf}, {'use_sim_time': use_sim_time}]
    )

    # RViz node — visualizes lidar, TF, odometry, etc.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )


    return LaunchDescription([
        declare_use_sim_time,
        robot_state_publisher_node,
        rviz_node,
    ])
