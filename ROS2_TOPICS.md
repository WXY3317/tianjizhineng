# 项目控制接口

本文只列对外使用的控制入口。以下示例会向机器人发送目标；请按现场情况填写目标值。位置单位为米，关节角和旋转增量单位为弧度。

目标命令中的 `stamp: now` 会在发布时生成新时间戳。控制任务忙碌时，新目标不会排队。

## 左右臂

| 控制话题 | 作用 |
| --- | --- |
| `/gento_control/left/target_joints` | 左臂七轴绝对关节目标 |
| `/gento_control/right/target_joints` | 右臂七轴绝对关节目标 |
| `/gento_control/left/target_pose` | 左臂法兰绝对位姿，末端按直线轨迹运动 |
| `/gento_control/right/target_pose` | 右臂法兰绝对位姿，末端按直线轨迹运动 |
| `/gento_control/left/target_pose_ptp` | 左臂法兰绝对位姿，按关节空间轨迹运动 |
| `/gento_control/right/target_pose_ptp` | 右臂法兰绝对位姿，按关节空间轨迹运动 |
| `/gento_control/left/rotate_tcp` | 保持左臂当前 TCP 位置，按固定 `body_base` 轴旋转姿态 |
| `/gento_control/right/rotate_tcp` | 保持右臂当前 TCP 位置，按固定 `body_base` 轴旋转姿态 |

七轴目标按关节名对应，单位为弧度。分别发送到左右臂的样例：

```bash
ros2 topic pub --once /gento_control/left/target_joints sensor_msgs/msg/JointState \
  '{header: {stamp: now}, name: [left_joint_1, left_joint_2, left_joint_3, left_joint_4, left_joint_5, left_joint_6, left_joint_7], position: [1.91314266, 1.31929438, -1.61547676, -1.34913951, -1.97765003, 0.11096803, -0.39805724]}'

ros2 topic pub --once /gento_control/right/target_joints sensor_msgs/msg/JointState \
  '{header: {stamp: now}, name: [right_joint_1, right_joint_2, right_joint_3, right_joint_4, right_joint_5, right_joint_6, right_joint_7], position: [0.25, -0.45, 0.30, -1.00, 0.0, 0.0, 0.0]}'
```

法兰位姿目标使用 `body_base` 坐标，位置为米，姿态为单位四元数 `x/y/z/w`。下例分别使用左臂直线轨迹和右臂关节空间轨迹。将话题后缀在 `target_pose` 与 `target_pose_ptp` 之间替换，即可切换轨迹方式；关节空间轨迹不保证法兰路径为直线。

```bash
ros2 topic pub --once /gento_control/left/target_pose geometry_msgs/msg/PoseStamped \
  '{header: {stamp: now, frame_id: body_base}, pose: {position: {x: 0.5599476948, y: 0.2499921409, z: 0.6499149393}, orientation: {x: 0.7070993883, y: 0.0000101756, z: -0.0000073963, w: 0.7071141739}}}'

ros2 topic pub --once /gento_control/right/target_pose_ptp geometry_msgs/msg/PoseStamped \
  '{header: {stamp: now, frame_id: body_base}, pose: {position: {x: 0.5600184529, y: -0.2699976378, z: 0.6497690436}, orientation: {x: -0.0000354015, y: 0.7078026320, z: 0.7064102419, w: 0.0000559751}}}'
```

`rotate_tcp` 的 `vector.x/y/z` 是绕 `body_base` 固定 X/Y/Z 轴的相对增量；同时指定多轴时按 X、Y、Z 顺序应用，即 `Rz × Ry × Rx`。例如左臂绕固定 Z 轴旋转 10°，或右臂绕固定 X 轴旋转 5°：

```bash
ros2 topic pub --once /gento_control/left/rotate_tcp geometry_msgs/msg/Vector3Stamped \
  '{header: {stamp: now, frame_id: body_base}, vector: {x: 0.0, y: 0.0, z: 0.1745329252}}'

ros2 topic pub --once /gento_control/right/rotate_tcp geometry_msgs/msg/Vector3Stamped \
  '{header: {stamp: now, frame_id: body_base}, vector: {x: 0.0872664626, y: 0.0, z: 0.0}}'
```

## 躯干与头部

| 控制话题 | 作用 |
| --- | --- |
| `/gento_control/body/target_joints` | 躯干六轴绝对关节目标 |
| `/gento_control/body/target_pose` | 躯干第六轴 `BD_J6` 的绝对位姿目标，执行运动 |
| `/gento_control/body/preview_pose` | 对躯干位姿目标求解预览，不执行运动 |
| `/gento_control/head/target_joints` | 头部两轴绝对关节目标 |

躯干关节和头部关节目标的单位为弧度。头部控制需启用 `enable_head_control:=true`。

```bash
ros2 topic pub --once /gento_control/body/target_joints sensor_msgs/msg/JointState \
  '{header: {stamp: now}, name: [body_joint_1, body_joint_2, body_joint_3, body_joint_4, body_joint_5, body_joint_6], position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'

ros2 topic pub --once /gento_control/head/target_joints sensor_msgs/msg/JointState \
  '{header: {stamp: now}, name: [head_joint_1, head_joint_2], position: [0.0, 0.0]}'
```

躯干位姿目标使用 `body_base` 坐标和完整的位置、姿态。下面先给出不执行运动的预览样例；确认目标后，将话题改为 `/gento_control/body/target_pose` 执行。默认目标高度范围为 0.55～0.76 m，目标仍需满足关节与路径约束。

```bash
ros2 topic pub --once /gento_control/body/preview_pose geometry_msgs/msg/PoseStamped \
  '{header: {stamp: now, frame_id: body_base}, pose: {position: {x: 0.0011537186, y: 0.0, z: 0.6644140365}, orientation: {x: 0.0000079485, y: 0.4554128364, z: 0.0000155383, w: 0.8902803761}}}'
```

## 场景控制

| 控制入口 | 作用 |
| --- | --- |
| `/scene/arm_position` | `0`：左右臂依次到初始关节位置；`1`：依次到准备位置 |
| `/scene/restore_tool_z` | 在当前法兰位置撤销上一次由 scene 执行的夹具局部 Z 轴旋转 |
| `/scene/attach_workpiece`（`std_srvs/srv/Trigger`） | 第 5 步人工确认左夹爪抓牢后调用；启动左臂回退、准备位、躯干回零和左臂交接位运动 |
| `/scene/start_handover`（`std_srvs/srv/Trigger`） | 第 10 步人工确认交接区安全后调用；启动右臂交接流程 |
| `/scene/detach_workpiece`（`std_srvs/srv/Trigger`） | 工件实际释放后结束位姿跟踪 |
| MQTT `vs/workpiece/6d_pose` | 提交工件位姿；`position1` 使用关节空间轨迹，`position2` 使用法兰直线轨迹 |
| `/scene/status`（`std_msgs/msg/String`） | JSON 状态；含 `awaiting_attach`、`awaiting_handover`、`handover_waiting`、`post_grasp_completed` 和 `rejected` |
| `/scene/workpiece_pose`（`geometry_msgs/msg/PoseStamped`） | 根据当前持有臂的法兰反馈推算的工件位姿，坐标系为 `body_base` |

`/scene/arm_position` 使用 launch 中的 `left_arm_joint_init`、`right_arm_joint_init`、`left_arm_joint_pre`、`right_arm_joint_pre` 七轴预设，单位弧度。先移动左臂，完成后再移动右臂；任一步失败即停止。需以 `dry_run:=false` 启动 scene。

```bash
ros2 topic pub --once /scene/arm_position std_msgs/msg/Int8 '{data: 0}'  # 到初始位置
ros2 topic pub --once /scene/arm_position std_msgs/msg/Int8 '{data: 1}'  # 到准备位置

ros2 service call /scene/restore_tool_z std_srvs/srv/Trigger '{}'
# 分别在终端中观察状态与工件位姿：
ros2 topic echo /scene/status
ros2 topic echo /scene/workpiece_pose
# 看到 /scene/status 的 awaiting_attach，且确认左夹爪已抓牢后：
ros2 service call /scene/attach_workpiece std_srvs/srv/Trigger '{}'
# 看到 awaiting_handover，且确认交接区域安全后：
ros2 service call /scene/start_handover std_srvs/srv/Trigger '{}'
# 工件实际释放后：
ros2 service call /scene/detach_workpiece std_srvs/srv/Trigger '{}'
```

scene 默认接收下面的 MQTT 工件目标。`position1` 是预抓取位置，通过 `/gento_control/{left,right}/target_pose_ptp` 执行；完成后等待 `interval` 秒（默认 1.0 秒），`position2` 作为工件实际位置通过 `/gento_control/{left,right}/target_pose` 执行。第二步继承第一步选定的法兰姿态和绕轴角度，只改变位置；第二步不可达时拒绝，不再另选姿态。Y 正值选左臂，Y 负值选右臂；两个非零目标必须选择同一只手臂。位置单位为米，姿态为 `body_base` 下的单位四元数。`position2` 可省略或填全零占位。工件原点作为夹具尖端目标，法兰到尖端的局部 Z 偏移默认 0.20 m。实际执行需以 `dry_run:=false` 启动 scene。

左臂到 `position2` 后，scene 自动发送左夹爪 27 mm 命令并等待夹爪反馈到位；`awaiting_attach` 后才由操作者调用附着服务。左臂依次退回 `position1`、到 `left_arm_joint_pre`、躯干六轴回零、到 `left_post_grasp_joints`。`awaiting_handover` 后才由操作者启动右臂。右臂到 `right_handover_joints` 并收到完成状态后等待 `interval`，再保持姿态沿 `body_base` 的 +Y 方向直线移动 `handover_y_offset_m`（默认 0.07 m）；右夹爪到 27 mm、左夹爪到 0 mm 后右臂退回预备交接位。每段运动等待控制节点报告完成，夹爪动作等待状态到位；夹爪到位本身不证明工件已可靠持有。

附着期间，scene 根据法兰反馈持续发布工件估计位姿。交接时由左右法兰反馈求右法兰到工件的固定平移与旋转，随后改用右臂跟踪。当前尚未实现右臂放置流程。放置目标若由视觉给出，须用工件目标位姿乘以右法兰到工件固定变换的逆，得到右法兰目标；不能只旋转一个角度。

```bash
mosquitto_pub -h 127.0.0.1 -t vs/workpiece/6d_pose -m \
  '{"position1":{"x":0.55,"y":0.25,"z":0.65},"position2":{"x":0.55,"y":0.25,"z":0.62},"orientation":{"x":0,"y":0,"z":0,"w":1}}'
```

## 夹爪

| 控制话题 | 作用 |
| --- | --- |
| `/gripper/command` | 单夹爪启动方式下的控制入口 |
| `/left_gripper/command` | 双夹爪启动方式下的左夹爪控制入口 |
| `/right_gripper/command` | 双夹爪启动方式下的右夹爪控制入口 |

动作编号：`0` 移动到指定开度，`1` 标零，`2` 使能，`3` 失能，`4` 清错，`5` 设置电流触发保持阈值。移动开度 `position_m` 单位为米，范围 0～0.098 m；动作 `5` 的 `current_threshold_a` 单位为安培。标零会驱动夹爪到机械全开端，应先确认空载、张开路径畅通。

```bash
ros2 topic pub --once /left_gripper/command gripper/msg/GripperCommand '{action: 1}'
ros2 topic pub --once /left_gripper/command gripper/msg/GripperCommand '{action: 2}'
ros2 topic pub --once /left_gripper/command gripper/msg/GripperCommand '{action: 0, position_m: 0.060}'
ros2 topic pub --once /left_gripper/command gripper/msg/GripperCommand '{action: 5, current_threshold_a: 0.8}'
ros2 topic pub --once /left_gripper/command gripper/msg/GripperCommand '{action: 3}'

ros2 topic pub --once /right_gripper/command gripper/msg/GripperCommand '{action: 4}'
```

单夹爪模式下将样例话题换为 `/gripper/command`；双夹爪模式分别使用左右话题。

## 停止与清错

| 控制话题 | 作用 |
| --- | --- |
| `/gento_driver/stop` | 软件停止当前任务并锁定故障 |
| `/gento_control/reset_error` | 全局清错与恢复 |
| `/gento_control/left/reset_error` | 左臂清错 |
| `/gento_control/right/reset_error` | 右臂清错 |
| `/gento_control/body/reset_error` | 躯干清错 |
| `/gento_control/head/reset_error` | 头部清错 |

```bash
ros2 topic pub --once /gento_driver/stop std_msgs/msg/Empty '{}'
ros2 topic pub --once /gento_control/reset_error std_msgs/msg/Empty '{}'
ros2 topic pub --once /gento_control/left/reset_error std_msgs/msg/Empty '{}'
ros2 topic pub --once /gento_control/right/reset_error std_msgs/msg/Empty '{}'
ros2 topic pub --once /gento_control/body/reset_error std_msgs/msg/Empty '{}'
ros2 topic pub --once /gento_control/head/reset_error std_msgs/msg/Empty '{}'
```
