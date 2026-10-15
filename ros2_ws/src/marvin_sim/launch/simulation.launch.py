"""SDK UDP controller + real SDK bridge + URDF visualization."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction, RegisterEventHandler, EmitEvent
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def start(context):
    description = get_package_share_directory('marvin_description')
    simulation = get_package_share_directory('marvin_sim')
    sdk = LaunchConfiguration('sdk_root').perform(context)
    params = {'sdk_root': sdk, 'description_root': description,
              'sim_ip': LaunchConfiguration('sim_ip').perform(context)}
    robot = xacro.process_file(str(Path(description)/'urdf/marvin.urdf.xacro'),
                               mappings={'ros2_control': 'false', 'use_fake_hardware': 'true'}).toxml()
    server = Node(package='marvin_sim', executable='controller', output='screen', parameters=[params])
    bridge = Node(package='marvin_sim', executable='controller', arguments=['--bridge'],
                  output='screen', parameters=[params], condition=IfCondition(LaunchConfiguration('bridge')))
    return [
        RegisterEventHandler(OnProcessExit(target_action=server, on_exit=[
            EmitEvent(event=Shutdown(reason='Virtual controller exited'))])),
        RegisterEventHandler(OnProcessExit(target_action=bridge, on_exit=[
            EmitEvent(event=Shutdown(reason='SDK bridge exited'))])),
        server,
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot}], output='screen'),
        TimerAction(period=1., actions=[bridge]),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(Path(simulation)/'rviz/simulation.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
        TimerAction(period=2., actions=[Node(
            package='marvin_sim', executable='panel', parameters=[params],
            condition=IfCondition(LaunchConfiguration('panel')), output='screen')]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sdk_root', description='Absolute path to tianji_armSDK'),
        DeclareLaunchArgument('sim_ip', default_value='', description='Assigned local IPv4, auto if empty'),
        DeclareLaunchArgument('bridge', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('panel', default_value='true'),
        OpaqueFunction(function=start),
    ])
