import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('aeris10_driver')
    params_default = os.path.join(pkg, 'config', 'aeris10_driver.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=params_default,
                              description='Path to aeris10_driver.yaml'),
        DeclareLaunchArgument('sim_mode', default_value='false',
                              description='true = synthetic target, no USB'),
        DeclareLaunchArgument('drone_ns', default_value='',
                              description='Drone namespace prefix'),

        Node(
            package='aeris10_driver',
            executable='driver_node',
            name='aeris10_driver',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'sim_mode':  LaunchConfiguration('sim_mode'),
                    'drone_ns':  LaunchConfiguration('drone_ns'),
                },
            ],
            output='screen',
        ),
    ])
