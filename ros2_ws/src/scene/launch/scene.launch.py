from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mqtt_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('mqtt_port', default_value='1883'),
        DeclareLaunchArgument('mqtt_topic', default_value='/vision_targe'),
        Node(package='scene', executable='vision_targets', name='scene_vision_targets',
             output='screen', parameters=[{
                 'mqtt_host': LaunchConfiguration('mqtt_host'),
                 'mqtt_port': ParameterValue(LaunchConfiguration('mqtt_port'), value_type=int),
                 'mqtt_topic': LaunchConfiguration('mqtt_topic'),
             }]),
    ])
