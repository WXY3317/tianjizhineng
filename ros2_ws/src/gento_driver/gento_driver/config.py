DEFAULTS = {
    'sdk_root': '', 'ip': '', 'enable_control': False,
    'speed': 5.0, 'acc': 5.0,
    'max_delta': 5.0, 'timeout': 60.0, 'tolerance': 0.2,
    'publish_rate': 20.0, 'command_max_age': 1.0,
    'sdk_length_to_m': 0.001,
    'trajectory_log_dir': '~/.ros/gento_trajectories',
    'cartesian_execution': 'trajectory', 'cartesian_path_tolerance': 0.005,
    'cartesian_speed': 0.01, 'cartesian_angular_speed': 0.1,
    'cartesian_joint_speed': 5.0, 'cartesian_joint_acc': 10.0,
    'left_frame': 'left_arm_sdk_base', 'right_frame': 'right_arm_sdk_base',
    'body_frame': 'body_base',
    'body_position_tolerance_mm': 1.0, 'body_orientation_tolerance_deg': 0.2,
}
