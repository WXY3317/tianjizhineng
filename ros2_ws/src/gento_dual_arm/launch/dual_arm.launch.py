# 声明双臂控制参数并启动 Gento ROS 桥接节点。
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {
        'sdk_root': ('', str),
        'ip': ('', str),
        'execute': ('false', bool),
        'speed': ('5.0', float), 
        'acc': ('5.0', float),
        'plan_speed': ('0.05', float), 
        'plan_acc': ('0.05', float),
        'max_delta': ('5.0', float), 
        'timeout': ('60.0', float), 
        'tolerance': ('0.2', float),
        'publish_rate': ('20.0', float), 
        'command_max_age': ('1.0', float),
        'sdk_length_to_m': ('0.001', float),
        'left_frame': ('left_arm_sdk_base', str), 
        'right_frame': ('right_arm_sdk_base', str),
    }
    declarations = [DeclareLaunchArgument(k, default_value=v) for k, (v, _) in defaults.items()]
    params = {k: ParameterValue(LaunchConfiguration(k), value_type=t) for k, (_, t) in defaults.items()}
    return LaunchDescription(declarations + [Node(package='gento_dual_arm', executable='bridge',
                                                 name='gento_dual_arm', output='screen', parameters=[params])])
