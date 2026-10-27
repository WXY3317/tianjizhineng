# 声明双臂控制参数并启动 Gento ROS 桥接节点。
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# 修改此处设置真机控制器地址；也可通过 ip:=... 临时覆盖。
CONTROLLER_IP = '6.6.7.190'


def default_sdk_root():
    configured = os.environ.get('GENTO_SDK_ROOT')
    if configured:
        return str(Path(configured).expanduser())
    for parent in Path(__file__).resolve().parents:
        candidate = parent / 'tianji-robot-SDK'
        if (candidate / 'PYTHON_SDK/GentoRobot.py').is_file():
            return str(candidate)
    return ''  # 独立部署时使用 sdk_root:=... 或 GENTO_SDK_ROOT。


def generate_launch_description():
    # 各项为（默认值，参数类型）；可在启动时用 参数名:=值 覆盖。
    # 例如：./run_real_robot.sh cartesian_speed:=0.02
    # 节点将这些参数声明为只读，修改后需重启节点；所有数值参数必须为有限正数。
    defaults = {
        # SDK 根目录，包含 PYTHON_SDK/GentoRobot.py；优先读取 GENTO_SDK_ROOT，否则向上查找。
        'sdk_root': (default_sdk_root(), str),
        # 机器人控制器 IPv4 地址，不是部署用 ARM 主机的 SSH 地址。
        'ip': (CONTROLLER_IP, str),
        # POSITION 模式速度百分比：5.0 表示 5%，当前允许 (0, 10]；不是末端 m/s。
        'speed': ('8.0', float),
        # POSITION 模式加速度百分比：5.0 表示 5%，当前允许 (0, 10]。
        'acc': ('6.0', float),
        # 关节目标的 SDK 点到点规划速度比例，0.05=5%，允许 [0.01, 0.1]；不用于末端直线插补。
        'plan_speed': ('0.05', float),
        # 关节目标的 SDK 点到点规划加速度比例，0.05=5%，允许 [0.01, 0.1]。
        'plan_acc': ('0.05', float),
        # 每个关节相对本次起点的最大角度变化，单位度；同时检查目标和轨迹点，不是相邻点步长。
        'max_delta': ('90.0', float),
        # 运动执行超时，单位秒；不含离线规划时间。直线轨迹规划也须为到位确认预留时间。
        'timeout': ('60.0', float),
        # 关节到位容差，单位度；满足容差并持续 0.5 秒才完成，手臂还需结束轨迹下发/播放。
        'tolerance': ('0.2', float),
        # 关节/TCP 反馈及运动完成检查频率，单位 Hz，最大 100；不改变直线指令的 50 Hz 目标频率。
        'publish_rate': ('20.0', float),
        # 目标消息允许的最大年龄，单位秒；目标必须带新鲜时间戳，忙碌期间的目标不排队。
        'command_max_age': ('1.0', float),
        # SDK 长度到米的换算系数：0.001 表示 SDK 使用 mm；影响位姿输入、反馈和距离检查。
        'sdk_length_to_m': ('0.001', float),
        # 每次直线运动的规划、反馈和故障诊断 JSON 目录；空字符串关闭文件记录。
        'trajectory_log_dir': ('~/.ros/gento_trajectories', str),
        # trajectory：完整上传点集由控制器播放；stream：旧的主机逐点位置下发方式。
        'cartesian_execution': ('trajectory', str),
        # 实际反馈 TCP 到规划线段的最大距离，单位 m；默认 5 mm，超出时软件急停。
        'cartesian_path_tolerance': ('0.005', float),
        # 末端直线规划的线速度上限，单位 m/s；0.05=5 cm/s，含平滑加减速，实际可能因重规划/调度而降低。
        'cartesian_speed': ('0.08', float),
        # 末端姿态 SLERP 的角速度上限，单位 rad/s；0.1 约为 5.73°/s，姿态保持时不产生旋转。
        'cartesian_angular_speed': ('0.2', float),
        # 直线 IK 关节速度配置上限，单位度/s；实际还受各关节 VelMax × 控制器比例 × 90% 约束。
        'cartesian_joint_speed': ('20.0', float),
        # 直线 IK 关节加速度配置上限，单位度/s²；实际还受各关节 AccMax × 控制器比例 × 90% 约束。
        'cartesian_joint_acc': ('25.0', float),
        # 左臂位姿反馈和目标使用的 SDK 基坐标系名称；仅校验名称，不执行 TF 变换，也不改变实际坐标轴。
        'left_frame': ('left_arm_sdk_base', str),
        # 右臂 SDK 基坐标系名称，必须与 left_frame 不同；左右臂位姿目标可独立发送。
        'right_frame': ('right_arm_sdk_base', str),
    }
    declarations = [DeclareLaunchArgument(k, default_value=v) for k, (v, _) in defaults.items()]
    params = {k: ParameterValue(LaunchConfiguration(k), value_type=t) for k, (_, t) in defaults.items()}
    return LaunchDescription(declarations + [Node(package='gento_dual_arm', executable='bridge',
                                                 name='gento_dual_arm', output='screen', parameters=[params])])
