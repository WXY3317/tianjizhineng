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
| `/gento_feedback/body/tcp_pose` | `geometry_msgs/msg/PoseStamped` | 当前躯干六轴反馈角正解得到的 `BD_FL` 位姿，位置 m、姿态单位四元数，默认坐标系 `body_base` |

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
| `/gento_control/left/target_pose` | `geometry_msgs/msg/PoseStamped` | 左臂绝对目标位姿，位置 m、姿态单位四元数，默认坐标系 `left_arm_sdk_base` |
| `/gento_control/right/target_pose` | `geometry_msgs/msg/PoseStamped` | 右臂绝对目标位姿，位置 m、姿态单位四元数，默认坐标系 `right_arm_sdk_base` |
| `/gento_control/body/target_pose` | `geometry_msgs/msg/PoseStamped` | 躯干末端 `BD_FL` 的绝对目标位姿，位置 m、姿态单位四元数，默认坐标系 `body_base` |

发布样例（左臂；数值仅说明格式，执行前应确认目标及路径）：

```bash
ros2 topic pub --once /gento_control/left/target_pose geometry_msgs/msg/PoseStamped \
"{header: {stamp: now, frame_id: left_arm_sdk_base},
  pose: {
    position: {x: 0.25, y: 0.25, z: 0.21},
    orientation: {x: -0.5523719707682738, y: 0.3977202702536071, z: -0.30106956732332846, w: 0.6678779141210726}
  }}"
```

右臂替换话题为 `/gento_control/right/target_pose`，默认坐标系替换为 `right_arm_sdk_base`。目标必须带新鲜时间戳，不自动做 TF 转换。

躯干目标使用 `/gento_control/body/target_pose`，`frame_id: body_base`。`body_base` 固定在第 1 轴中心、轴方向与配置 AGV 平行；末端为 `BD_FL`，包含第 6 轴之后的安装偏移。先读取 `/gento_feedback/body/tcp_pose`，以实际当前位置和朝向填写目标并刷新时间戳，不要把示例中的手臂位置直接用于躯干。

躯干先求完整位姿逆解，在控制器限位与额外 J1 ±11°、J2 ±50°、J5 ±30°、J6 ±170°内选择接近当前角度的解；J3/J4 采用控制器限位，并受 `max_delta` 限制。再复用现有单点关节位置执行通道，不保证躯干末端沿直线运动。无解、错误坐标系、过期消息或非法四元数会被拒绝。反馈中 `body/tcp_pose` 是关节反馈正解，不是目标位姿回显。

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
