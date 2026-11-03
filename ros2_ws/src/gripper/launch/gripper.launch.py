from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def gripper_config(side, motor_id='', calibration_path=''):
    if side not in ('left', 'right'):
        raise ValueError("side must be 'left' or 'right'")
    share = get_package_share_directory('gripper')
    selected_id = 1 if side == 'left' else 2
    selected_table = 'calibration.csv' if side == 'left' else 'right_calibration.csv'
    return (int(motor_id) if motor_id else selected_id,
            calibration_path or f'{share}/config/{selected_table}')


def make_node(context):
    side = LaunchConfiguration('side').perform(context)
    motor_id, calibration = gripper_config(
        side,
        LaunchConfiguration('motor_id').perform(context),
        LaunchConfiguration('calibration_path').perform(context))
    parameters = {
        'port': ParameterValue(LaunchConfiguration('port'), value_type=str),
        'motor_id': motor_id,
        'calibration_path': calibration,
        'baud_rate': ParameterValue(LaunchConfiguration('baud_rate'), value_type=int),
        'use_existing_zero': ParameterValue(LaunchConfiguration('use_existing_zero'), value_type=bool),
        'current_threshold_a': ParameterValue(LaunchConfiguration('current_threshold_a'), value_type=float),
        'feedback_rate_hz': ParameterValue(LaunchConfiguration('feedback_rate_hz'), value_type=float),
        'speed_mps': ParameterValue(LaunchConfiguration('speed_mps'), value_type=float),
    }
    return [Node(package='gripper', executable='gripper_node',
                 namespace=LaunchConfiguration('namespace'), output='screen',
                 parameters=[parameters])]


def generate_launch_description():
    defaults = {
        'side': 'left', 'namespace': 'gripper', 'port': '/dev/gripper_rs485',
        'baud_rate': '115200', 'use_existing_zero': 'true',
        'current_threshold_a': '0.5', 'feedback_rate_hz': '10.0',
        'speed_mps': '0.002',
        # Optional overrides kept for existing launch scripts.
        'motor_id': '', 'calibration_path': '',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()],
        OpaqueFunction(function=make_node),
    ])
