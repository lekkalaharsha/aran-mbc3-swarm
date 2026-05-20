"""
Launch radar_fusion pipeline.

Single drone:  ros2 launch radar_fusion radar_fusion.launch.py
Swarm (5):     ros2 launch radar_fusion radar_fusion.launch.py mode:=swarm

Optional override:
  params_file:=<absolute path to yaml>
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace


DRONES = ['drone_L', 'drone_S1', 'drone_S2', 'drone_S3', 'drone_S4']

_DEFAULT_YAML = os.path.join(
    get_package_share_directory('radar_fusion'),
    'config', 'radar_fusion.yaml',
)


def _is_swarm(mode):
    """Substitution that evaluates True when mode == 'swarm'."""
    return PythonExpression(["'", mode, "' == 'swarm'"])


def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='single',
        description='"single" — one detection node, no namespace  |  "swarm" — 5 namespaced nodes + fusion',
    )
    params_arg = DeclareLaunchArgument(
        'params_file', default_value=_DEFAULT_YAML,
        description='Absolute path to parameter YAML',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Set true when running alongside Gazebo',
    )

    mode         = LaunchConfiguration('mode')
    params_file  = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    is_swarm     = _is_swarm(mode)

    # ── Single-drone detection (no namespace, mode=single) ───────────────────
    # B4: don't pass drone_ns when empty — ROS2 CLI rejects empty string param
    single_det = Node(
        condition=UnlessCondition(is_swarm),
        package='radar_fusion',
        executable='detection_node',
        name='radar_detection',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    # ── Swarm: 5 namespaced detection nodes (mode=swarm) ────────────────────
    swarm_dets = [
        GroupAction(
            condition=IfCondition(is_swarm),
            actions=[
                PushRosNamespace(drone),
                Node(
                    package='radar_fusion',
                    executable='detection_node',
                    name='radar_detection',
                    parameters=[params_file, {'drone_ns': drone, 'use_sim_time': use_sim_time}],
                    output='screen',
                ),
            ],
        )
        for drone in DRONES
    ]

    # ── Fusion node (swarm only) ─────────────────────────────────────────────
    fusion = Node(
        condition=IfCondition(is_swarm),
        package='radar_fusion',
        executable='fusion_node',
        name='radar_fusion',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        mode_arg,
        params_arg,
        use_sim_time_arg,
        single_det,
        *swarm_dets,
        fusion,
    ])
