"""Launch ARiADNE (rl_planner) + our adapter against an already running sim + slam_toolbox + Nav2.

Usage:
  ros2 launch ariadne_adapter ariadne.launch.py params_file:=<repo>/configs/methods/ariadne.yaml

Differences from the authors' rl_planner.launch.py (see docs/baseline_provenance.md):
  - no octomap_server: the map comes from slam_toolbox through the adapter
  - no LD_PRELOAD of /usr/lib/x86_64-linux-gnu/libstdc++.so.6 (that path does not exist on the
    aarch64 Jetson; re-add it on the laptop only if rl_planner fails to import torch)
  - no RViz
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            description='configs/methods/ariadne.yaml in the ExploreBench repo'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        Node(
            package='ariadne_adapter',
            executable='adapter_node',
            name='ariadne_adapter',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='rl_planner',
            executable='rl_planner',
            name='rl_planner',
            output='screen',
            emulate_tty=True,
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
    ])
