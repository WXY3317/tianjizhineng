"""Reopen GUI windows while reusing an existing SDK simulation backend."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    model=Path(get_package_share_directory('marvin_mesh_sim'))
    core=Path(get_package_share_directory('marvin_sim'))
    params={'sdk_root':LaunchConfiguration('sdk_root').perform(context),
            'description_root':str(model),'sim_ip':LaunchConfiguration('sim_ip').perform(context)}
    return [
        Node(package='rviz2',executable='rviz2',arguments=['-d',str(core/'rviz/simulation.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz')),output='screen'),
        Node(package='marvin_mesh_sim',executable='panel',parameters=[params],
             condition=IfCondition(LaunchConfiguration('panel')),output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sdk_root'),DeclareLaunchArgument('sim_ip',default_value=''),
        DeclareLaunchArgument('bridge',default_value='true'),
        DeclareLaunchArgument('rviz',default_value='true'),DeclareLaunchArgument('panel',default_value='true'),
        OpaqueFunction(function=start)])
