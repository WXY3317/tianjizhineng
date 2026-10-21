# 启动整机仿真控制器、SDK 桥接、模型发布器和可选界面；后端退出时联动关闭本次启动。
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,TimerAction,RegisterEventHandler,EmitEvent
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    model=Path(get_package_share_directory('marvin_mesh_sim'))
    params={'sdk_root':LaunchConfiguration('sdk_root').perform(context),
            'description_root':str(model),'sim_ip':LaunchConfiguration('sim_ip').perform(context),
            'gento_sdk_root':LaunchConfiguration('gento_sdk_root').perform(context),
            'sg100_sdk_root':LaunchConfiguration('sg100_sdk_root').perform(context)}
    server=Node(package='marvin_mesh_sim',executable='controller',parameters=[params],output='screen')
    bridge=Node(package='marvin_mesh_sim',executable='controller',arguments=['--bridge'],
                parameters=[params],output='screen',condition=IfCondition(LaunchConfiguration('bridge')))
    actions=[server,
        Node(package='robot_state_publisher',executable='robot_state_publisher',
             parameters=[{'robot_description':(model/'urdf/marvin_mesh_hands.urdf').read_text()}],output='screen'),
        TimerAction(period=1.,actions=[bridge]),
        Node(package='rviz2',executable='rviz2',arguments=['-d',str(model/'rviz/whole_robot.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz')),output='screen'),
        TimerAction(period=2.,actions=[Node(package='marvin_mesh_sim',executable='panel',
             parameters=[params],condition=IfCondition(LaunchConfiguration('panel')),output='screen')])]
    for process in (server,bridge):
        actions.insert(0,RegisterEventHandler(OnProcessExit(target_action=process,on_exit=[
            EmitEvent(event=Shutdown(reason='Mesh simulator backend exited'))])))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sdk_root'),DeclareLaunchArgument('sim_ip',default_value=''),
        DeclareLaunchArgument('gento_sdk_root',default_value=''),
        DeclareLaunchArgument('sg100_sdk_root',default_value=''),
        DeclareLaunchArgument('bridge',default_value='true'),
        DeclareLaunchArgument('rviz',default_value='true'),DeclareLaunchArgument('panel',default_value='true'),
        OpaqueFunction(function=start)])
