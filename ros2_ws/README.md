# Gento 真机工作空间

ROS 2 Humble 工作空间，提供双臂、躯干、夹爪控制，关节与法兰位姿反馈，
以及 MQTT 视觉目标场景。driver 独占唯一 SDK 连接；control、feedback、scene 不直接连接 SDK。

六轴躯干正逆解在 `gento_driver.body_kinematics` 中实现，使用躯干固定基座和第 6 关节 `BD_J6` 末端，支持控制器限位、J2 ±50°约束及接近当前关节角的选解。调用方式和离线示例见 [躯干运动学说明](src/gento_driver/BODY_KINEMATICS.md)。

## 编译、部署与启动

需要 ROS 2 Humble、Python 3.10、colcon，以及与运行主机架构、固件匹配的 SDK。以下命令在 `ros2_ws` 目录执行。

```bash
# 部署前先在远端按 Ctrl+C 退出机器人节点
# 自动交叉编译 ARM64 SDK、同步并在远端编译 driver 及其依赖，不启动机器人
./deploy_arm.sh --host tianji

# 启动二选一；--build 表示先编译，后续启动可省略
./run_feedback_only.sh --build  # driver + feedback，开放执行接口，control 单独启动
./run_real_robot.sh --build    # driver + feedback + control，并构建 scene；允许控制

# 临时覆盖参数
./run_real_robot.sh speed:=5.0 acc:=5.0 cartesian_angular_speed:=0.1

# 场景节点单独启动，自动加载工作区环境；首次构建可加 --build
./run_scene.sh
# 或随真机程序一起启动场景（不再单独运行 run_scene.sh）
./run_real_robot.sh start_scene:=true
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
`run_scene.sh` 只启动视觉目标节点，不启动真机驱动；须确保 control 与 feedback 已运行。
其默认 MQTT Broker 为本机 `127.0.0.1:1883`，也可传入
`mqtt_host:=... mqtt_port:=... mqtt_topic:=...`。默认主题为 `/vision_target`。

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
ros2 topic echo --once /gento_feedback/left/tcp_pose_body_base
ros2 topic echo --once /gento_feedback/left/tcp_pose_bd_j6
ros2 topic echo --once /gento_feedback/full_joint_states
ros2 topic echo /gento_control/status
```

控制目标统一使用 `/gento_control/` 前缀，左右臂可单独发送：

| 话题后缀 | 消息与单位 |
| --- | --- |
| `left/target_pose`、`right/target_pose` | `geometry_msgs/msg/PoseStamped`，位置 m、姿态单位四元数 |
| `body/target_pose` | `geometry_msgs/msg/PoseStamped`，躯干 `BD_J6` 位姿目标，逆解后执行，参考系 `body_base` |
| `body/preview_pose` | `geometry_msgs/msg/PoseStamped`，仅预览躯干逆解，不执行 |
| `body/ik_result` | `std_msgs/msg/String`，逆解结果 JSON，含六轴角度和弧度 |
| `left/target_joints`、`right/target_joints` | `sensor_msgs/msg/JointState`，完整的 7 个关节名称与位置，rad |
| `body/target_joints` | `sensor_msgs/msg/JointState`，完整的 6 个关节名称与位置，rad |

目标是绝对值，必须带新鲜时间戳。JointState 不接受 velocity/effort。任务不排队，等待 `completed` 后再发下一条。以下为话题发布格式示例，数值不代表机器人当前位置，执行前应替换为现场确认过的目标；不要连续执行全部示例。

```bash
# 左臂末端位姿：位置 m，姿态四元数
ros2 topic pub --once /gento_control/left/target_pose geometry_msgs/msg/PoseStamped \
"{header: {stamp: now, frame_id: body_base},
  pose: {
    position: {x: 0.25, y: 0.25, z: 0.21},
    orientation: {x: -0.5523719707682738, y: 0.3977202702536071, z: -0.30106956732332846, w: 0.6678779141210726}
  }}"

# 右臂末端位姿
ros2 topic pub --once /gento_control/right/target_pose geometry_msgs/msg/PoseStamped \
"{header: {stamp: now, frame_id: body_base},
  pose: {
    position: {x: 0.31, y: -0.31, z: 0.21},
    orientation: {x: 0.631002025966013, y: 0.3239363696324344, z: 0.14142256111728738, w: 0.6905804304084884}
  }}"

# 左臂关节位置：rad，与 name 顺序对应
ros2 topic pub --once /gento_control/left/target_joints sensor_msgs/msg/JointState \
"{header: auto,
  name: [left_joint_1, left_joint_2, left_joint_3, left_joint_4, left_joint_5, left_joint_6, left_joint_7],
  position: [2.47, -1.45, -1.62, -2.49, -0.02, 0.60, 0.075]}"

```

双臂位姿目标标准坐标系为固定在躯干第 1 轴中心的 `body_base`；程序使用当前躯干角度转换到 SDK 臂基座后逆解，不依赖设备 TF。`/gento_feedback/left|right/tcp_pose_body_base` 给出统一坐标反馈；`/gento_feedback/left|right/tcp_pose_bd_j6` 给出相对当前躯干末端 `BD_J6` 的位姿，仅用于反馈，不作为逆解目标。旧 `left|right/tcp_pose` 仍为各自 SDK 基座坐标。安装变换包含控制器 DH 中 SDK 基座到第 1 轴中心的 158.6 mm 偏移；双臂零位法兰相对 `BD_J6` 的 Y 约为 ±0.8296 m。躯干 `body/tcp_pose` 来自当前六轴反馈角的本地正解，末端为 `BD_J6`，不是外部测量。旧 `/gento_dual_arm/...` 话题已移除。

`BD_J6` 双臂反馈同时通过独立 MQTT 节点发往设备原有配置的 `127.0.0.1:1883`，主题为
`gento/feedback/left/tcp_pose_bd_j6` 和 `gento/feedback/right/tcp_pose_bd_j6`。
启动参数 `start_mqtt_feedback:=false` 可关闭；tianji 的 Mosquitto 默认在
`0.0.0.0:1883` 监听所有 IPv4 接口，本机通过 `127.0.0.1:1883`、
局域网设备通过 `192.168.2.121:1883` 发布与订阅；服务启动无需等待局域网 IP。

### MQTT 视觉目标场景

`scene` 默认订阅 MQTT `/vision_target`。消息使用米和 `xyzw` 单位四元数，
`position1`、`position2` 是 `BD_J6` 下依次执行的目标，二者共用 `orientation`。
每个目标的 Y > 0 选择左臂，Y < 0 选择右臂；全零 `position2` 为占位并跳过。
节点使用实时 `/gento_feedback/body/tcp_pose` 将两点转换到 `body_base`，
第一步收到 control 的 `completed` 后才发送第二步。MQTT retained 消息、
过期躯干反馈和 Y = 0 的非占位目标会被拒绝。查看 `/scene/status` 了解处理结果。
完整消息格式见 [scene 功能包说明](src/scene/README.md)。

躯干位姿通过 `/gento_control/body/target_pose` 发送，`frame_id` 设为 `body_base`。该话题会逆解并**执行运动**；只预览逆解时使用 `/gento_control/body/preview_pose`，结果发布到 `/gento_control/body/ik_result` 并在 control 日志打印。以当前反馈位姿为起点修改目标，必须重新填写时间戳。逆解约束完整位置与朝向，并在 `max_delta` 和有效限位内优先接近当前关节角；J2 额外限制为 ±50°。关节角执行仍使用 `/gento_control/body/target_joints`。详见 [躯干话题用法](src/gento_driver/BODY_KINEMATICS.md#ros-话题反馈和控制)。

`target_pose` 现在从当前 `BD_J6` 位姿到目标做位置直线插补和姿态 SLERP，沿同一逆解分支求出关节锚点，再生成 50 Hz 关节样本。下发前逐点检查正解位姿、关节限位、速度和加速度；driver 按时间顺序发送关节目标并监控实际末端到线段的偏差。`target_joints` 仍是单点关节目标，不走此直线规划。轨迹点由控制器跟踪，实机路径误差受 `cartesian_path_tolerance` 监控，不能把模型直线等同于物理上的零误差直线。

躯干位姿逆解的 `body_base` 高度范围默认 **0.55–0.76 m**（包含端点），预览和执行均检查目标以及最终逆解结果；超出范围会拒绝。可在 `src/gento_driver/launch/dual_arm.launch.py` 修改 `body_z_min_m`、`body_z_max_m`，或启动时覆盖：`./run_real_robot.sh body_z_min_m:=0.55 body_z_max_m:=0.76`。零位正解高度为 0.765 m，因此零位不能作为这两个位姿话题的逆解目标。直线路径允许从当前范围外高度向范围内移动，但不能越过当前高度继续远离范围。关节角执行话题不经过此位姿逆解限制。

## 躯干控制命令

以下命令在运行主机的 `ros2_ws` 目录执行。先在终端一启动驱动、反馈和控制节点：

```bash
./run_real_robot.sh  # 代码更新后需重新编译时加 --build
```

在终端二加载环境，先读取当前躯干末端位姿和六轴关节角：

```bash
source /opt/ros/humble/setup.bash
source install_real/setup.bash
ros2 topic echo --once /gento_feedback/body/tcp_pose
ros2 topic echo --once /gento_feedback/body/joint_states
```

**按末端位姿执行运动**：将下面 `X/Y/Z` 替换为目标位置（m），`QX/QY/QZ/QW` 替换为目标单位四元数。以刚读取的当前位姿为基础修改；若保持朝向，直接沿用当前反馈的四元数。下面的命令会让躯干运动。

```bash
ros2 topic pub --once /gento_control/body/target_pose geometry_msgs/msg/PoseStamped \
"{header: {stamp: now, frame_id: body_base},
  pose: {
    position: {x: X, y: Y, z: Z},
    orientation: {x: QX, y: QY, z: QZ, w: QW}
  }}"
```

只想预览逆解时，把上述命令的话题改为 `/gento_control/body/preview_pose`。在另一终端读取预览结果：

```bash
ros2 topic echo --once /gento_control/body/ik_result
```

结果 JSON 中 `joints_deg` 是六轴角度（度），`joints_rad` 是弧度，`reference_joints_deg` 是求解时的当前反馈角。结果话题保留最近一条消息，读取时应核对 `target_stamp` 是否与本次目标一致。手动发送 `target_joints` 时使用 `joints_rad`，先自行核对零位、限位和实机姿态。`preview_pose` 不会调用执行 Action。

参考系 `body_base` 固定在第 1 轴中心，轴方向与配置中的 AGV 平行；末端是第 6 关节坐标系 `BD_J6`，不包含其后的 391.5 mm 安装偏移。目标和反馈均表示 `T_B_BD_J6`；当前配置全零位的位置为 `[0, 0, 765]` mm（ROS 消息为 `[0, 0, 0.765]` m）。消息是绝对目标位姿，不是位移增量。该话题同时约束位置和朝向，不自动做 TF 转换。

旧 `BD_FL` 目标不能直接复用，需按 `T_B_J6 = T_B_FL @ inverse(T_J6_FL)` 转换。当前安装变换等价于沿旧末端自身 Z 方向减去 391.5 mm（ROS 为 0.3915 m），朝向不变；不能一律从 `body_base` 的 Z 坐标相减。

**按六轴关节角控制**：将 `J1` 至 `J6` 替换为目标绝对角度，单位 **rad**，不能直接填写度数。

```bash
ros2 topic pub --once /gento_control/body/target_joints sensor_msgs/msg/JointState \
"{header: {stamp: now},
  name: [body_joint_1, body_joint_2, body_joint_3, body_joint_4, body_joint_5, body_joint_6],
  position: [J1, J2, J3, J4, J5, J6]}"
```

预览求解及执行均受有效关节限位和 `max_delta` 约束。第 2 轴额外限制为 ±50°，即约 ±0.872665 rad。目标不排队；在另一终端观察状态，预览等待 `previewed`，执行等待 `completed` 后再发送下一条新目标：

```bash
ros2 topic echo /gento_control/status
```

需要软件停止时执行以下命令；这会停止双臂与躯干并锁定故障，恢复步骤见下一节：

```bash
ros2 topic pub --once /gento_driver/stop std_msgs/msg/Empty '{}'
```

## 停止与清错

```bash
# 唯一软件停止话题
ros2 topic pub --once /gento_driver/stop std_msgs/msg/Empty '{}'

# 排除故障原因后恢复；等待 ready，再发送新目标
ros2 topic pub --once /gento_control/reset_error std_msgs/msg/Empty '{}'
```

停止会锁定故障，不续跑旧轨迹，control 同时取消规划。全局恢复会清错、确认 POSITION 并更换会话。`/gento_control/{left,right,body}/reset_error` 只处理对应对象，恢复整体运动仍使用全局恢复命令。正常退出用 Ctrl+C，节点会处理停止、模式退出和 SDK 断开。

## 夹爪控制命令

远端夹爪包已安装在同一 `install_real` 工作空间。每个终端先加载环境：

```bash
cd /home/nvidia/wxy/tianjizhineng/ros2_ws
source /opt/ros/humble/setup.bash
source install_real/setup.bash
```

**单夹爪启动**（在终端一执行）：

```bash
ros2 launch gripper gripper.launch.py side:=left
# 或只启动右夹爪：ros2 launch gripper gripper.launch.py side:=right
```

单夹爪 launch 默认使用 `/dev/gripper_rs485`、已有零位和 `0.002 m/s`（2 mm/s）速度。可在启动时覆盖单项参数，例如左夹爪使用其他串口：

```bash
ros2 launch gripper gripper.launch.py side:=left namespace:=left_gripper \
  port:=/dev/ttyUSB4 speed_mps:=0.005
```

单夹爪 launch 默认使用已有零位（`use_existing_zero=true`），不自动使能或运动，因此无需在命令中重复填写。若不使用已有零位，添加 `use_existing_zero:=false`。速度参数启动后只读，修改需重启节点。

在其他已加载环境的终端分别查看状态和应答：

```bash
ros2 topic echo /gripper/state
ros2 topic echo /gripper/ack
```

**控制指令**：逐条发送并检查 ACK 和状态，不要整段连续执行。移动前应满足 `valid=true`、`error_code=0`、`homed=true`，并先成功使能。

```bash
# 1. 使能
ros2 topic pub --once /gripper/command gripper/msg/GripperCommand \
  '{action: 2}'

# 2. 移动到 60 mm 开度，速度使用 launch 配置（默认 5 mm/s）
ros2 topic pub --once /gripper/command gripper/msg/GripperCommand \
  '{action: 0, position_m: 0.060}'

# 3. 改为 30 mm 开度，速度使用同一 launch 配置
ros2 topic pub --once /gripper/command gripper/msg/GripperCommand \
  '{action: 0, position_m: 0.030}'

# 4. 失能
ros2 topic pub --once /gripper/command gripper/msg/GripperCommand \
  '{action: 3}'

# 清错：不自动使能，也不自动恢复旧目标
ros2 topic pub --once /gripper/command gripper/msg/GripperCommand \
  '{action: 4}'
```

开度 `position_m` 单位 **m**，范围 `0～0.098`；数值越大越张开。速度在 launch 的 `speed_mps` 中配置，单位 **m/s**，默认 `0.005`（5 mm/s），须为有限正数；控制消息中不再包含速度。MOVE 的 ACK 只代表指令已接受，完成需查看 `/gripper/state` 的 `at_target`；触发电流保持时查看 `current_limit_reached`、`holding_position`。

| action | 动作 |
| --- | --- |
| `0` | MOVE，移动到指定开度 |
| `1` | ZERO，按已有标定流程标零，会主动张开，完成后失能 |
| `2` | ENABLE，使能 |
| `3` | DISABLE，失能 |
| `4` | CLEAR，清错 |

仅在适用的已有标定流程下执行标零；它不是未知零点时的通用恢复命令：

```bash
ros2 topic pub --once /gripper/command gripper/msg/GripperCommand \
  '{action: 1}'
```

`/gento_driver/stop` 和 `/gento_control/reset_error` 只处理机器人双臂、躯干，不控制这个独立夹爪节点。夹爪使用自己的 DISABLE / CLEAR 指令。

**双夹爪共用一路串口**：不要同时运行单夹爪节点。设备地址须已设置为左 1、右 2；左侧默认使用 `config/calibration.csv`，右侧默认使用 `config/right_calibration.csv`：

```bash
ros2 launch gripper dual_gripper.launch.py port:=/dev/ttyUSB0 \
  left_id:=1 right_id:=2
```

两侧各自确认零位有效后，可在上述启动命令后添加 `left_use_existing_zero:=true right_use_existing_zero:=true`。控制消息格式与单夹爪相同，仅替换话题：

双夹爪速度分别由 `dual_gripper.launch.py` 的 `left_speed_mps`、`right_speed_mps` 配置，均默认 `0.005 m/s`。例如启动时覆盖：`left_speed_mps:=0.005 right_speed_mps:=0.010`。

| 夹爪 | 命令话题 | 状态 / ACK |
| --- | --- | --- |
| 左 | `/left_gripper/command` | `/left_gripper/state`、`/left_gripper/ack` |
| 右 | `/right_gripper/command` | `/right_gripper/state`、`/right_gripper/ack` |

```bash
# 左、右分别使能；每条都需要确认对应 ACK
ros2 topic pub --once /left_gripper/command gripper/msg/GripperCommand '{action: 2}'
ros2 topic pub --once /right_gripper/command gripper/msg/GripperCommand '{action: 2}'

# 左、右分别设置 60 mm 开度，速度使用各自 launch 配置
ros2 topic pub --once /left_gripper/command gripper/msg/GripperCommand '{action: 0, position_m: 0.060}'
ros2 topic pub --once /right_gripper/command gripper/msg/GripperCommand '{action: 0, position_m: 0.060}'
```

标定、电流保持和双夹爪总线细节见 [gripper 功能包说明](src/gripper/README.md)。

## 参数与运动行为

默认值和详细注释见 [dual_arm.launch.py](src/gento_driver/launch/dual_arm.launch.py)。启动时用 `参数名:=值` 覆盖；运行中参数只读，修改后需重启。

| 参数 | 含义 |
| --- | --- |
| `ip` | 机器人控制器地址，与部署用 SSH 地址不同 |
| `speed`、`acc` | 规划速度、加速度的控制器百分比，范围 `(0,10]` |
| `cartesian_speed`、`cartesian_angular_speed` | 末端线速度 m/s、姿态角速度 rad/s 上限 |
| `cartesian_joint_speed`、`cartesian_joint_acc` | 关节速度 °/s、加速度 °/s² 上限 |
| `max_delta` | 每关节相对本次起点的最大位移，°，不是相邻点步长 |
| `body_z_min_m`、`body_z_max_m` | 躯干 `BD_J6` 位姿逆解目标与结果的高度范围，`body_base` 中，默认 0.55–0.76 m |
| `tolerance`、`timeout` | 最终关节到位容差 °、执行超时 s |
| `body_position_tolerance_mm`、`body_orientation_tolerance_deg` | 躯干末端到位容差，默认 1 mm / 0.2°；与关节容差同时满足并持续 0.5 秒才完成 |
| `cartesian_path_tolerance` | 实际 TCP 到起终点线段的最大距离，默认 0.005 m |
| `publish_rate` | 反馈发布频率 Hz，不改变轨迹的 50 Hz 采样周期 |
| `sdk_length_to_m` | SDK 长度换算为米，默认 0.001（mm → m），不能用于调速 |
| `trajectory_log_dir` | 轨迹记录目录，默认 `~/.ros/gento_trajectories`，空字符串关闭记录 |
| `start_scene`、`vision_mqtt_topic` | 随真机启动 scene（默认关闭）及其 MQTT 主题（默认 `/vision_target`） |

双臂位姿任务先做位置直线插值和姿态 SLERP，再以上一点关节解为参考逐点求 IK。完整路径通过限位、连续性、速度和加速度检查后，才上传关节轨迹执行，必要时自动延长时间。终点可达不代表保持指定姿态的整条直线路径可达。

轨迹播放期间，移动臂的 SDK 全局比例临时设为 100%，避免控制器重复限速扭曲轨迹；规划和驱动校验仍使用原低速约束，完成或停止后恢复比例。反馈中看到 100% 不等于以额定速度运动。

## 功能包与通信

| 包 | 职责 |
| --- | --- |
| `gento_driver` | 唯一 `robot.link(ip)`；SDK 读写、点级 FK/IK、独立轨迹校验与执行保护 |
| `gento_control` | 接收目标、路径和时间规划、Action 执行管理与心跳 |
| `gento_feedback` | 汇总关节/TCP/错误反馈，诊断并记录轨迹 JSON |
| `gento_interfaces` | 查询服务和轨迹 Action 定义，不运行节点 |
| `scene` | MQTT 视觉目标解析、`BD_J6` 到 `body_base` 转换及双臂目标顺序发布 |
| `gripper` | RS485 左右夹爪状态与控制 |

control 通过 `/gento_driver/query` 查询、计算 IK/FK 和恢复，通过 `/gento_driver/execute` 提交轨迹，通过 `/gento_driver/heartbeat` 维持心跳。driver 发布 `/gento_driver/raw_state` 和 `/gento_driver/status`；feedback 整理为 `/gento_feedback/...`，结合 `/gento_control/plan` 和 `/gento_control/status` 保存记录。control、feedback 不直接连接 SDK。

driver 独立检查起点、限位、速度、模式、断链、超时、心跳和 TCP 直线偏差，异常会停止并锁定故障。诊断 JSON 包含规划、实际反馈和结束原因，可用于排查问题。软件保护依赖 ROS、SDK 和通信，不能替代实体急停；SDK 阻塞会延迟停止处理，当前没有碰撞检测，跨机器运行需同步时钟。

修改代码后可运行回归测试（先构建并加载环境）：

```bash
python3 -m pytest src/gento_driver/test src/gento_control/test src/gento_feedback/test src/scene/test -q
```
