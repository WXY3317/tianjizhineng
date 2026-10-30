# Gento 真机工作空间

ROS 2 Humble 工作空间，提供双臂、躯干控制及关节和 TCP 反馈。三个业务节点独立运行，driver 独占唯一 SDK 连接。

六轴躯干正逆解在 `gento_driver.body_kinematics` 中实现，使用躯干固定基座和 `BD_FL` 末端，支持控制器限位、J2 ±50°约束及接近当前关节角的选解。调用方式和离线示例见 [躯干运动学说明](src/gento_driver/BODY_KINEMATICS.md)。

## 编译、部署与启动

需要 ROS 2 Humble、Python 3.10、colcon，以及与运行主机架构、固件匹配的 SDK。以下命令在 `ros2_ws` 目录执行。

```bash
# 部署前先在远端按 Ctrl+C 退出机器人节点
# 自动交叉编译 ARM64 SDK、同步并在远端编译四个包，不启动机器人
./deploy_arm.sh --host tianji

# 启动二选一；--build 表示先编译，后续启动可省略
./run_feedback_only.sh --build  # driver + feedback，开放执行接口，control 单独启动
./run_real_robot.sh --build    # driver + feedback + control，允许控制

# 临时覆盖参数
./run_real_robot.sh speed:=5.0 acc:=5.0 cartesian_angular_speed:=0.1
```

无需图形界面。`run_feedback_only.sh` 与 `run_real_robot.sh` 不能同时运行，也不要同时使用其他厂商客户端连接机器人。两个脚本都会执行恢复和 POSITION 初始化，等待 `ready` 后发送目标，启动不会自动回零。`run_feedback_only.sh` 仅表示不启动 control，并非只读模式。

需要分两个终端启动时：

```bash
# 终端一：启动允许控制的 driver + feedback，不启动 control
./run_feedback_only.sh --build
# 终端二：只启动 control，复用 driver 的 SDK 连接
./run_control_only.sh
```

`run_control_only.sh --build` 可先编译控制包及其依赖。不要重复启动 control。
两个终端须使用相同的 ROS_DOMAIN_ID 和可互通的 DDS 配置。只有 driver 建立 SDK 连接，control 通过 ROS 接口使用它。

| SDK 构建脚本 | 用途 |
| --- | --- |
| `build_gento_sdk.sh` | 用本机编译器构建 SDK，输出到 `../tianji-robot-SDK/PYTHON_SDK/libGentoSDKPY.so` |
| `cross_build_gento_sdk.sh` | 默认交叉编译 ARM64 SDK，输出到 `artifacts/gento-sdk/arm64/`，不覆盖本机库 |
| `deploy_arm.sh` | 自动调用交叉编译脚本，再同步并在远端编译 ROS 包；`--dry-run` 预览操作 |

## 反馈与目标命令

在运行主机的新终端先加载环境：

```bash
source /opt/ros/humble/setup.bash
source install_real/setup.bash

ros2 topic echo --once /gento_feedback/left/tcp_pose
ros2 topic echo --once /gento_feedback/right/tcp_pose
ros2 topic echo --once /gento_feedback/body/tcp_pose
ros2 topic echo --once /gento_feedback/full_joint_states
ros2 topic echo /gento_control/status
```

控制目标统一使用 `/gento_control/` 前缀，左右臂可单独发送：

| 话题后缀 | 消息与单位 |
| --- | --- |
| `left/target_pose`、`right/target_pose` | `geometry_msgs/msg/PoseStamped`，位置 m、姿态单位四元数 |
| `body/target_pose` | `geometry_msgs/msg/PoseStamped`，躯干末端 `BD_FL` 的绝对位姿，位置 m、姿态单位四元数，参考系 `body_base` |
| `left/target_joints`、`right/target_joints` | `sensor_msgs/msg/JointState`，完整的 7 个关节名称与位置，rad |
| `body/target_joints` | `sensor_msgs/msg/JointState`，完整的 6 个关节名称与位置，rad |

目标是绝对值，必须带新鲜时间戳。JointState 不接受 velocity/effort。任务不排队，等待 `completed` 后再发下一条。以下为话题发布格式示例，数值不代表机器人当前位置，执行前应替换为现场确认过的目标；不要连续执行全部示例。

```bash
# 左臂末端位姿：位置 m，姿态四元数
ros2 topic pub --once /gento_control/left/target_pose geometry_msgs/msg/PoseStamped \
"{header: {stamp: now, frame_id: left_arm_sdk_base},
  pose: {
    position: {x: 0.25, y: 0.25, z: 0.21},
    orientation: {x: -0.5523719707682738, y: 0.3977202702536071, z: -0.30106956732332846, w: 0.6678779141210726}
  }}"

# 右臂末端位姿
ros2 topic pub --once /gento_control/right/target_pose geometry_msgs/msg/PoseStamped \
"{header: {stamp: now, frame_id: right_arm_sdk_base},
  pose: {
    position: {x: 0.31, y: -0.31, z: 0.21},
    orientation: {x: 0.631002025966013, y: 0.3239363696324344, z: 0.14142256111728738, w: 0.6905804304084884}
  }}"

# 左臂关节位置：rad，与 name 顺序对应
ros2 topic pub --once /gento_control/left/target_joints sensor_msgs/msg/JointState \
"{header: auto,
  name: [left_joint_1, left_joint_2, left_joint_3, left_joint_4, left_joint_5, left_joint_6, left_joint_7],
  position: [2.47, -1.45, -1.62, -2.49, -0.02, 0.60, 0.075]}"

# 躯干关节位置：rad；此示例为全部零位的绝对目标，不是相对微动
ros2 topic pub --once /gento_control/body/target_joints sensor_msgs/msg/JointState \
"{header: auto,
  name: [body_joint_1, body_joint_2, body_joint_3, body_joint_4, body_joint_5, body_joint_6],
  position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

双臂位姿分别使用 `left_arm_sdk_base`、`right_arm_sdk_base`，躯干位姿使用 `body_base`，不自动做 TF 变换。双臂 TCP 来自 SDK 正解；躯干 `body/tcp_pose` 来自当前六轴反馈角的本地正解，末端为 `BD_FL`，不是外部测量。旧 `/gento_dual_arm/...` 话题已移除。

躯干目标通过 `/gento_control/body/target_pose` 发送，消息格式与手臂相同，`frame_id` 设为 `body_base`。以当前反馈位姿为起点修改目标，必须重新填写时间戳。逆解约束完整位置与朝向，并在 `max_delta` 和有效限位内优先接近当前关节角；J2 额外限制为 ±50°。求解失败不下发关节命令。躯干复用控制器的关节位置执行通道，只保证末端目标，不保证移动路径为直线，详见 [躯干话题用法](src/gento_driver/BODY_KINEMATICS.md#ros-话题反馈和控制)。

## 停止与清错

```bash
# 唯一软件停止话题
ros2 topic pub --once /gento_driver/stop std_msgs/msg/Empty '{}'

# 排除故障原因后恢复；等待 ready，再发送新目标
ros2 topic pub --once /gento_control/reset_error std_msgs/msg/Empty '{}'
```

停止会锁定故障，不续跑旧轨迹，control 同时取消规划。全局恢复会清错、确认 POSITION 并更换会话。`/gento_control/{left,right,body}/reset_error` 只处理对应对象，恢复整体运动仍使用全局恢复命令。正常退出用 Ctrl+C，节点会处理停止、模式退出和 SDK 断开。

## 参数与运动行为

默认值和详细注释见 [dual_arm.launch.py](src/gento_driver/launch/dual_arm.launch.py)。启动时用 `参数名:=值` 覆盖；运行中参数只读，修改后需重启。

| 参数 | 含义 |
| --- | --- |
| `ip` | 机器人控制器地址，与部署用 SSH 地址不同 |
| `speed`、`acc` | 规划速度、加速度的控制器百分比，范围 `(0,10]` |
| `cartesian_speed`、`cartesian_angular_speed` | 末端线速度 m/s、姿态角速度 rad/s 上限 |
| `cartesian_joint_speed`、`cartesian_joint_acc` | 关节速度 °/s、加速度 °/s² 上限 |
| `max_delta` | 每关节相对本次起点的最大位移，°，不是相邻点步长 |
| `tolerance`、`timeout` | 最终关节到位容差 °、执行超时 s |
| `body_position_tolerance_mm`、`body_orientation_tolerance_deg` | 躯干末端到位容差，默认 1 mm / 0.2°；与关节容差同时满足并持续 0.5 秒才完成 |
| `cartesian_path_tolerance` | 实际 TCP 到起终点线段的最大距离，默认 0.005 m |
| `publish_rate` | 反馈发布频率 Hz，不改变轨迹的 50 Hz 采样周期 |
| `sdk_length_to_m` | SDK 长度换算为米，默认 0.001（mm → m），不能用于调速 |
| `trajectory_log_dir` | 轨迹记录目录，默认 `~/.ros/gento_trajectories`，空字符串关闭记录 |

双臂位姿任务先做位置直线插值和姿态 SLERP，再以上一点关节解为参考逐点求 IK。完整路径通过限位、连续性、速度和加速度检查后，才上传关节轨迹执行，必要时自动延长时间。终点可达不代表保持指定姿态的整条直线路径可达。

轨迹播放期间，移动臂的 SDK 全局比例临时设为 100%，避免控制器重复限速扭曲轨迹；规划和驱动校验仍使用原低速约束，完成或停止后恢复比例。反馈中看到 100% 不等于以额定速度运动。

## 功能包与通信

| 包 | 职责 |
| --- | --- |
| `gento_driver` | 唯一 `robot.link(ip)`；SDK 读写、点级 FK/IK、独立轨迹校验与执行保护 |
| `gento_control` | 接收目标、路径和时间规划、Action 执行管理与心跳 |
| `gento_feedback` | 汇总关节/TCP/错误反馈，诊断并记录轨迹 JSON |
| `gento_interfaces` | 查询服务和轨迹 Action 定义，不运行节点 |

control 通过 `/gento_driver/query` 查询、计算 IK/FK 和恢复，通过 `/gento_driver/execute` 提交轨迹，通过 `/gento_driver/heartbeat` 维持心跳。driver 发布 `/gento_driver/raw_state` 和 `/gento_driver/status`；feedback 整理为 `/gento_feedback/...`，结合 `/gento_control/plan` 和 `/gento_control/status` 保存记录。control、feedback 不直接连接 SDK。

driver 独立检查起点、限位、速度、模式、断链、超时、心跳和 TCP 直线偏差，异常会停止并锁定故障。诊断 JSON 包含规划、实际反馈和结束原因，可用于排查问题。软件保护依赖 ROS、SDK 和通信，不能替代实体急停；SDK 阻塞会延迟停止处理，当前没有碰撞检测，跨机器运行需同步时钟。

修改代码后可运行回归测试（先构建并加载环境）：

```bash
python3 -m pytest src/gento_driver/test src/gento_control/test src/gento_feedback/test -q
```
