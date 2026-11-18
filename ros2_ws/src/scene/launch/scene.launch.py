from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {
        # position1 完成后到发送 position2 的间隔，单位秒。
        'interval': ('1.0', float),
        # 抓取后回退、左臂 pre、躯干回零，最后移动到这组左臂关节角；单位弧度。
        'left_post_grasp_joints': ('[2.845654625621635, 0.4907167724907257, '
                                   '-2.826141844584338, -1.5708835932574963, '
                                   '-2.0122948143793824, 0.49986229777117597, '
                                   '-1.035084966187757]', str),
        'right_handover_joints': ('[0.2175552912610932, -0.4690223298884362, '
                                  '-0.19600047499896323, -1.5263951506241609, '
                                  '-1.1898433109620943, -0.5670400206804377, '
                                  '-1.070166084152843]', str),
        'back_left_post_grasp_joints': ('[2.840, 0.40, -2.850, -1.57, -2.01, 0.50, -1.03]', str),
        'back_right_post_grasp_joints': ('[0.220, -0.4, -0.20, -1.520, -1.190, -0.560, -1.1]', str),
        'handover_y_offset_m': ('0.065', float),
    }

    return LaunchDescription([DeclareLaunchArgument(k, default_value=v)
                              for k, (v, _) in defaults.items()] + [
        DeclareLaunchArgument('arm_positions_file',
                              default_value=PathJoinSubstitution([FindPackageShare('scene'), 'config', 'arm_positions.json'])),
        DeclareLaunchArgument('mqtt_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('mqtt_port', default_value='1883'),
        DeclareLaunchArgument('mqtt_topic', default_value='vs/workpiece/6d_pose'),
        DeclareLaunchArgument('dry_run', default_value='false'),
        # 法兰沿自身局部 +Z 到夹具尖端的距离，单位米；换 10 cm 夹具时改为 '0.10'。
        # 若尖端位于法兰局部 -Z 方向，使用负值。启动命令传同名参数可临时覆盖。
        DeclareLaunchArgument('tool_tip_offset_z_m', default_value='0.20'),
        DeclareLaunchArgument('allow_tool_z_rotation', default_value='true'),
        DeclareLaunchArgument('last_rotation_file', default_value='~/.ros/scene_last_rotation.json'),
        DeclareLaunchArgument('tool_z_angle_file', default_value='~/.ros/scene_tool_z_angle.json'),
        Node(package='scene', executable='vision_targets', name='scene_vision_targets',
             output='screen', parameters=[{
                 'arm_positions_file': LaunchConfiguration('arm_positions_file'),
                 'mqtt_host': LaunchConfiguration('mqtt_host'),
                 'mqtt_port': ParameterValue(LaunchConfiguration('mqtt_port'), value_type=int),
                 'mqtt_topic': LaunchConfiguration('mqtt_topic'),
                 'dry_run': ParameterValue(LaunchConfiguration('dry_run'), value_type=bool),
                 'tool_tip_offset_z_m': ParameterValue(LaunchConfiguration('tool_tip_offset_z_m'), value_type=float),
                 'allow_tool_z_rotation': ParameterValue(LaunchConfiguration('allow_tool_z_rotation'), value_type=bool),
                 'last_rotation_file': LaunchConfiguration('last_rotation_file'),
                 'tool_z_angle_file': LaunchConfiguration('tool_z_angle_file'),
                 **{k: ParameterValue(LaunchConfiguration(k), value_type=t)
                    for k, (_, t) in defaults.items()},
             }]),
    ])
