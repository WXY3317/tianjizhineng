from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    calibration = get_package_share_directory('gripper') + '/config/calibration.csv'
    defaults = {
        'port': '/dev/ttyUSB0', 'baud_rate': '115200',
        'left_id': '1', 'right_id': '2', 'feedback_rate_hz': '10.0',
        'left_calibration_path': calibration,
        'right_calibration_path': get_package_share_directory('gripper') + '/config/right_calibration.csv',
        'left_use_existing_zero': 'false', 'right_use_existing_zero': 'false',
        'left_current_threshold_a': '0.5', 'right_current_threshold_a': '0.5',
        'left_speed_mps': '0.005', 'right_speed_mps': '0.005',
    }
    parameters = {'dual_gripper': True}
    for name, kind in [('port', str), ('baud_rate', int), ('feedback_rate_hz', float)]:
        parameters[name] = ParameterValue(LaunchConfiguration(name), value_type=kind)
    for side in ('left', 'right'):
        for suffix, key, kind in [('id', 'motor_id', int), ('calibration_path', 'calibration_path', str),
                                  ('use_existing_zero', 'use_existing_zero', bool),
                                  ('current_threshold_a', 'current_threshold_a', float),
                                  ('speed_mps', 'speed_mps', float)]:
            parameters[f'{side}.{key}'] = ParameterValue(
                LaunchConfiguration(f'{side}_{suffix}'), value_type=kind)
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()],
        Node(package='gripper', executable='gripper_node', output='screen', parameters=[parameters]),
    ])
