# ROS 2 话题说明

## 1. 关节反馈

| 话题名称 | 消息类型 | 内容 |
| --- | --- | --- |
| `/gento_feedback/joint_states` | `sensor_msgs/msg/JointState` | 双臂 14 个关节，左臂在前、右臂在后，位置单位 rad |
| `/gento_feedback/body/joint_states` | `sensor_msgs/msg/JointState` | 躯干 6 个关节，位置单位 rad |
| `/gento_feedback/head/joint_states` | `sensor_msgs/msg/JointState` | 头部关节，数量由 SDK 反馈决定，位置单位 rad |
| `/gento_feedback/full_joint_states` | `sensor_msgs/msg/JointState` | 全部关节，顺序：头部 → 躯干 → 左臂 → 右臂，位置单位 rad |

消息样例（以躯干反馈为例，其他关节反馈使用相同结构）：

```yaml
header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: ''
name: [body_joint_1, body_joint_2, body_joint_3, body_joint_4, body_joint_5, body_joint_6]
position: [0.1, 0.2, 0.3, 0.0, 0.0, 0.0]
velocity: []
effort: []
```

关节名称：左臂 `left_joint_1` 至 `left_joint_7`，右臂 `right_joint_1` 至 `right_joint_7`，躯干 `body_joint_1` 至 `body_joint_6`，头部 `head_joint_1` 至 `head_joint_N`。`position` 与 `name` 一一对应。

## 2. TCP 位姿反馈

| 话题名称 | 消息类型 | 内容 |
| --- | --- | --- |
| `/gento_feedback/left/tcp_pose` | `geometry_msgs/msg/PoseStamped` | 左臂 TCP 位姿，位置 m、姿态单位四元数，默认坐标系 `left_arm_sdk_base` |
| `/gento_feedback/right/tcp_pose` | `geometry_msgs/msg/PoseStamped` | 右臂 TCP 位姿，位置 m、姿态单位四元数，默认坐标系 `right_arm_sdk_base` |
| `/gento_feedback/left/tcp_pose_body_base` | `geometry_msgs/msg/PoseStamped` | 左臂 TCP 在固定 `body_base` 中的位姿，位置 m |
| `/gento_feedback/right/tcp_pose_body_base` | `geometry_msgs/msg/PoseStamped` | 右臂 TCP 在固定 `body_base` 中的位姿，位置 m |
| `/gento_feedback/left/tcp_pose_bd_j6` | `geometry_msgs/msg/PoseStamped` | 左臂 TCP 相对当前 `BD_J6` 的位姿，位置 m、姿态单位四元数，仅反馈 |
| `/gento_feedback/right/tcp_pose_bd_j6` | `geometry_msgs/msg/PoseStamped` | 右臂 TCP 相对当前 `BD_J6` 的位姿，位置 m、姿态单位四元数，仅反馈 |
| `/gento_feedback/body/tcp_pose` | `geometry_msgs/msg/PoseStamped` | 当前躯干六轴反馈角正解得到的 `BD_J6` 位姿，位置 m、姿态单位四元数，默认坐标系 `body_base` |

同一组 `BD_J6` 位姿还由独立的 MQTT 节点发布到设备配置的 Broker `127.0.0.1:1883`：
`gento/feedback/left/tcp_pose_bd_j6` 和 `gento/feedback/right/tcp_pose_bd_j6`。
消息是 JSON，包含 `frame_id`、`stamp`（`sec`/`nanosec`）、`position`（米）和
`orientation`（四元数 `x/y/z/w`）。MQTT 仅反馈，QoS 0，不保留旧位姿；
Broker 暂不可用时节点自动重连。启动参数 `start_mqtt_feedback:=false` 可关闭，
`mqtt_host`、`mqtt_port`、`mqtt_topic_prefix` 可覆盖连接和主题前缀。

消息样例（左臂；右臂替换 `frame_id`）：

```yaml
header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: left_arm_sdk_base
pose:
  position: {x: 0.25, y: 0.25, z: 0.21}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
```

## 3. 错误与状态反馈

| 话题名称 | 消息类型 | 内容 |
| --- | --- | --- |
| `/gento_feedback/joint_errors` | `std_msgs/msg/String` | `data` 为 JSON 字符串，包含反馈可用性、对象状态、错误码及故障对象列表 |
| `/gento_feedback/status` | `std_msgs/msg/String` | `data` 为 JSON 字符串，转发 driver、control 的状态事件，字段随事件变化 |

错误反馈消息样例：

```yaml
data: '{"available": true, "objects": [{"object_id": 0, "state": 1, "error_code": 0}, {"object_id": 1, "state": 1, "error_code": 0}, {"object_id": 3, "state": 1, "error_code": 0}, {"object_id": 2, "state": 1, "error_code": 0}], "stamp": {"sec": 0, "nanosec": 0}, "fault_objects": []}'
```

对象 ID：`0` 左臂、`1` 右臂、`2` 头部、`3` 躯干。反馈超时时的消息样例：

```yaml
data: '{"available": false, "reason": "Driver feedback timeout"}'
```

状态消息样例（`task_id`、`reason`、`terminal` 等字段不保证每条事件都有）：

```yaml
data: '{"state": "completed", "session_id": "session-001", "task_id": "task-001", "reason": "Completed", "terminal": true}'
```

## 4. TCP 位姿目标

| 话题名称 | 消息类型 | 内容 |
| --- | --- | --- |
| `/gento_control/left/target_pose` | `geometry_msgs/msg/PoseStamped` | 左臂绝对目标位姿，位置 m、姿态单位四元数，标准坐标系 `body_base` |
| `/gento_control/right/target_pose` | `geometry_msgs/msg/PoseStamped` | 右臂绝对目标位姿，位置 m、姿态单位四元数，标准坐标系 `body_base` |
| `/gento_control/body/target_pose` | `geometry_msgs/msg/PoseStamped` | 躯干 `BD_J6` 位姿目标，逆解后执行，默认坐标系 `body_base` |
| `/gento_control/body/preview_pose` | `geometry_msgs/msg/PoseStamped` | 躯干逆解预览，不执行 |
| `/gento_control/body/ik_result` | `std_msgs/msg/String` | 逆解结果 JSON，含 `joints_deg` 和 `joints_rad` |

双臂目标使用 `frame_id: body_base`。`body_base` 固定于躯干第 1 轴中心，轴方向与 AGV 平行；它不随第 1 关节旋转。
控制节点使用当前躯干反馈角和 `luna_body.Cord` 的 `BD_J6 -> BD_FL -> LA_J1/RA_J1` 安装链，
将目标转换到各自 SDK 基座后执行现有逆解。躯干在双臂执行期间若偏离起点超过 0.2°，驱动停止执行。
真机执行前必须核准 Cord 的 `LA_J1/RA_J1` 与 SDK 逆解基座的原点和轴向一致。
旧 SDK 基坐标系目标仍可接收，用于兼容和核对；旧 TCP 反馈话题保持 SDK 坐标，新增的 `*_body_base` 话题用于统一坐标反馈。

发布样例（左臂；数值仅说明格式，执行前应确认目标及路径）：

```bash
ros2 topic pub --once /gento_control/left/target_pose geometry_msgs/msg/PoseStamped \
"{header: {stamp: now, frame_id: body_base},
  pose: {
    position: {x: 0.25, y: 0.25, z: 0.21},
    orientation: {x: -0.5523719707682738, y: 0.3977202702536071, z: -0.30106956732332846, w: 0.6678779141210726}
  }}"
```

右臂替换话题为 `/gento_control/right/target_pose`，仍使用 `body_base`。目标必须带新鲜时间戳；程序内用当前躯干角度转换，无需设备发布 TF。

躯干逆解执行使用 `/gento_control/body/target_pose`，`frame_id: body_base`；只预览时使用 `/gento_control/body/preview_pose`，结果在 `/gento_control/body/ik_result`。`body_base` 固定在第 1 轴中心、轴方向与配置 AGV 平行；末端为第 6 关节坐标系 `BD_J6`，不包含其后的 391.5 mm 安装偏移。目标和反馈均表示 `T_B_BD_J6`；当前配置全零位的位置为 `[0, 0, 765]` mm（ROS 消息为 `[0, 0, 0.765]` m）。先读取 `/gento_feedback/body/tcp_pose`，以实际当前位置和朝向填写目标并刷新时间戳，不要把示例中的手臂位置直接用于躯干。

位姿逆解目标和最终结果的 `body_base` 高度默认限制在 0.55–0.76 m（含端点），可通过 launch 参数 `body_z_min_m`、`body_z_max_m` 调整。零位 0.765 m 高于默认上限。位姿直线路径可从当前范围外向范围内移动，但不能越过当前高度继续远离范围；直接发送关节角不受此逆解限制。

旧 `BD_FL` 目标不能直接复用，需按 `T_B_J6 = T_B_FL @ inverse(T_J6_FL)` 转换。当前安装变换等价于沿旧末端自身 Z 方向减去 391.5 mm（ROS 为 0.3915 m），朝向不变；不能一律从 `body_base` 的 Z 坐标相减。

躯干先求完整位姿逆解，在控制器限位与额外 J1 ±11°、J2 ±50°、J5 ±30°、J6 ±170°内选择接近当前角度的解；J3/J4 采用控制器限位，并受 `max_delta` 限制。`target_pose` 从当前位姿做位置直线和姿态插补，生成并核验 50 Hz 关节目标后逐点发送；实际路径偏差由 `cartesian_path_tolerance` 监控。`target_joints` 仍为单点关节命令。无解、错误坐标系、过期消息或非法四元数会被拒绝。反馈中 `body/tcp_pose` 是关节反馈正解，不是目标位姿回显。

## 5. 停止与清错

| 话题名称 | 消息类型 | 内容 |
| --- | --- | --- |
| `/gento_driver/stop` | `std_msgs/msg/Empty` | 软件停止并锁定故障，不续跑旧轨迹 |
| `/gento_control/reset_error` | `std_msgs/msg/Empty` | 全局清错与恢复 |
| `/gento_control/left/reset_error` | `std_msgs/msg/Empty` | 左臂清错 |
| `/gento_control/right/reset_error` | `std_msgs/msg/Empty` | 右臂清错 |
| `/gento_control/body/reset_error` | `std_msgs/msg/Empty` | 躯干清错 |

发布样例：

```bash
# 软件停止
ros2 topic pub --once /gento_driver/stop std_msgs/msg/Empty '{}'

# 全局清错与恢复
ros2 topic pub --once /gento_control/reset_error std_msgs/msg/Empty '{}'

# 左臂清错；右臂、躯干分别将 left 替换为 right、body
ros2 topic pub --once /gento_control/left/reset_error std_msgs/msg/Empty '{}'
```

## 6. MQTT 视觉目标场景

独立 `scene` 包默认订阅 MQTT `/vision_targe`，从
`/gento_feedback/body/tcp_pose` 取得实时 `T_body_base_BD_J6`，再向
`/gento_control/left/target_pose` 或 `/gento_control/right/target_pose`
发布 `body_base` 下的目标。`position1`、`position2` 按顺序执行；
每个目标在 `BD_J6` 下的 Y 为正选择左臂，为负选择右臂。两点共用
`orientation`；全零 `position2` 占位会跳过。状态见 `/scene/status`。
可用 `./run_real_robot.sh start_scene:=true` 随真机程序启动，或单独
`ros2 launch scene scene.launch.py`；默认不启动场景节点。
MQTT JSON 格式见 `ros2_ws/src/scene/README.md`。
