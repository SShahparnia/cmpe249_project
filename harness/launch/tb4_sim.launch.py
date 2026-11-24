"""TurtleBot 4 Ignition sim with an optional Gazebo GUI.

Same as turtlebot4_ignition_bringup/turtlebot4_ignition.launch.py (Humble), plus
`gui:=false` to run the Gazebo server only. The GUI renders the 3D scene in
software on machines without a GPU (the Mac container), which costs several cores.

    ros2 launch harness/launch/tb4_sim.launch.py gui:=false slam:=true nav2:=true rviz:=true

slam, nav2, localization, rviz, x, y, z, yaw pass through to turtlebot4_spawn.launch.py.
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false'],
                          description='Start the Gazebo GUI (false = server only)'),
    DeclareLaunchArgument('world', default_value='warehouse', description='Ignition world'),
    DeclareLaunchArgument('model', default_value='standard', choices=['standard', 'lite']),
    DeclareLaunchArgument('namespace', default_value=''),
    DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
] + [DeclareLaunchArgument(p, default_value='0.0') for p in ('x', 'y', 'z', 'yaw')]


def generate_launch_description():
    share = get_package_share_directory
    tb4_ign = share('turtlebot4_ignition_bringup')

    resource_path = SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', [
        os.path.join(tb4_ign, 'worlds'), ':',
        os.path.join(share('irobot_create_ignition_bringup'), 'worlds'), ':',
        str(Path(share('turtlebot4_description')).parent.resolve()), ':',
        str(Path(share('irobot_create_description')).parent.resolve())])
    gui_plugin_path = SetEnvironmentVariable('IGN_GUI_PLUGIN_PATH', [
        os.path.join(share('turtlebot4_ignition_gui_plugins'), 'lib'), ':',
        os.path.join(share('irobot_create_ignition_plugins'), 'lib')])

    ign_gazebo = PathJoinSubstitution([share('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py'])
    world = [LaunchConfiguration('world'), '.sdf', ' -r', ' -v 2']
    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ign_gazebo]),
        condition=IfCondition(LaunchConfiguration('gui')),
        launch_arguments=[('ign_args', world + [
            ' --gui-config ',
            PathJoinSubstitution([tb4_ign, 'gui', LaunchConfiguration('model'), 'gui.config'])])])
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ign_gazebo]),
        condition=UnlessCondition(LaunchConfiguration('gui')),
        launch_arguments=[('ign_args', world + [' -s'])])

    clock_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
                        output='screen',
                        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'])

    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([tb4_ign, 'launch', 'turtlebot4_spawn.launch.py'])]),
        launch_arguments=[(k, LaunchConfiguration(k))
                          for k in ('namespace', 'rviz', 'x', 'y', 'z', 'yaw')])

    ld = LaunchDescription(ARGUMENTS)
    for a in (resource_path, gui_plugin_path, gazebo_gui, gazebo_server, clock_bridge, robot_spawn):
        ld.add_action(a)
    return ld
