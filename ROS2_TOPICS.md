# 项目控制接口

本文只列对外使用的控制入口。以下示例会向机器人发送目标；请按现场情况填写目标值。位置单位为米，关节角和旋转增量单位为弧度。

目标命令中的 `stamp: now` 会在发布时生成新时间戳。控制任务忙碌时，新目标不会排队。

## 左右臂

| 控制话题 | 作用 |
| --- | --- |
| `/gento_control/left/target_joints` | 左臂七轴绝对关节目标 |
| `/gento_control/right/target_joints` | 右臂七轴绝对关节目标 |
| `/gento_control/dual/target_joints` | 左右臂共十四轴绝对关节目标，同一任务同步执行 |
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

左右夹具尖端在 `body_base` 下的估计位姿持续发布到
`/gento_feedback/left/tool_tip_pose_body_base` 和
`/gento_feedback/right/tool_tip_pose_body_base`（`geometry_msgs/msg/PoseStamped`）。
它们使用法兰反馈与局部 `+Z` 方向的夹具偏移计算；默认偏移为 0.20 m，
可通过启动参数 `scene_tool_tip_offset_z_m` 配置。反馈来自模型和关节角，
并非夹具尖端的独立传感器测量。

```bash
ros2 topic echo /gento_feedback/left/tool_tip_pose_body_base --once
ros2 topic echo /gento_feedback/right/tool_tip_pose_body_base --once
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
| `/gento_control/body/target_pose_ptp` | 躯干位姿目标，只求终点逆解并执行关节位置运动，末端路径不保证直线 |
| `/gento_control/head/target_joints` | 头部两轴绝对关节目标 |

躯干关节和头部关节目标的单位为弧度。头部控制需启用 `enable_head_control:=true`。

```bash
ros2 topic pub --once /gento_control/body/target_joints sensor_msgs/msg/JointState \
  '{header: {stamp: now}, name: [body_joint_1, body_joint_2, body_joint_3, body_joint_4, body_joint_5, body_joint_6], position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'

ros2 topic pub --once /gento_control/head/target_joints sensor_msgs/msg/JointState \
  '{header: {stamp: now}, name: [head_joint_1, head_joint_2], position: [0.0, 0.0]}'
```

躯干位姿目标使用 `body_base` 坐标和完整的位置、姿态。下面先给出不执行运动的预览样例；确认目标后，将话题改为 `/gento_control/body/target_pose` 执行末端直线运动，或改为 `/gento_control/body/target_pose_ptp` 执行终点逆解后的关节位置运动。默认目标高度范围为 0.55～0.76 m，目标仍需满足关节限位。

```bash
ros2 topic pub --once /gento_control/body/preview_pose geometry_msgs/msg/PoseStamped \
  '{header: {stamp: now, frame_id: body_base}, pose: {position: {x: 0.0011537186, y: 0.0, z: 0.6644140365}, orientation: {x: 0.0000079485, y: 0.4554128364, z: 0.0000155383, w: 0.8902803761}}}'
```

## 场景控制

| 控制入口 | 作用 |
| --- | --- |
| `/scene/start`（`std_msgs/msg/String`） | `start`：同时向两夹爪发送 0 mm，双方到位后双臂到 `pre`、躯干到 `body_photo`；`end`：躯干六轴回零、双臂同步到 `home`，再向两夹爪同时发送 0 mm 并分别等待到位 |
| `/scene/arm_position`（`std_msgs/msg/Int8`） | 单独执行配置编号：`0=init`、`1=pre`、`2=body_photo`、`3=body_home`、`9=home` |
| `/scene/restore_tool_z`（`std_srvs/srv/Trigger`） | 在当前法兰位置撤销 scene 上一次工具局部 Z 轴旋转 |
| `/scene/attach_workpiece` / `/scene/detach_workpiece`（`std_srvs/srv/Trigger`） | 手动附着 / 解除工件跟踪；正常自动抓取不需要手动附着 |
| `/scene/status`（`std_msgs/msg/String`） | JSON：`state` 为 `running`、`success` 或 `failed`；`event` 保留 `arm_position_completed`、`start_completed`、`post_grasp_completed`、`rejected` 等步骤事件。`success` 仅在两次抓取及全部收尾完成后发布，失败原因在 `reason` |
| `/scene/workpiece_pose`（`geometry_msgs/msg/PoseStamped`） | 根据持有臂法兰反馈推算工件位姿，坐标系 `body_base` |

预设位置来自 `ros2_ws/src/scene/config/arm_positions.json`。`data: 3` 向躯干发送六轴绝对关节角 `[0, 0, 0, 0, 0, 0]`，不是末端 XYZ 回零。`data: 2` 把 `body_photo` 的配置位姿发到 `/gento_control/body/target_pose`，执行末端直线轨迹；若当前 J6 偏离 0 超过 0.01 rad，先依据最新反馈将 J4 增加 0.15 rad，并等待完成。自动抓取中的 `body_photo_restore` 仍按配置的 `body_pose_ptp` 执行。修改配置后重启 scene；随真机启动时可用 `scene_arm_positions_file:=绝对路径` 覆盖配置。

```bash
ros2 topic pub --once /scene/start std_msgs/msg/String '{data: start}'
ros2 topic pub --once /scene/start std_msgs/msg/String '{data: end}'
ros2 topic pub --once /scene/arm_position std_msgs/msg/Int8 '{data: 1}'  # 双臂到 pre
ros2 topic pub --once /scene/arm_position std_msgs/msg/Int8 '{data: 2}'  # 躯干沿直线到 body_photo
ros2 topic pub --once /scene/arm_position std_msgs/msg/Int8 '{data: 3}'  # 躯干六轴回零
ros2 topic echo /scene/status
```

运行时 scene 必须设置 `dry_run:=false`，且上一任务已完成。正常抓取前无需调用 `/scene/attach_workpiece`：有效的 `position2` 到位后，scene 自动依次使能两个夹爪、让抓取侧夹爪张开到 27 mm；到位或触发电流限制保持后，自动附着工件并回退。若 `position2` 缺失或是全零占位，不会自动附着和进入完整回退。夹爪电流限制保持只满足程序继续条件，不能证明实际夹牢。

随 `run_all_first.sh` / `run_all_second.sh` 启动时，scene 默认订阅 MQTT `/vision_target`；单独运行 `run_scene.sh` 时默认订阅 `vs/workpiece/6d_pose`。消息须含 `label`、`position1`、`orientation`，自动抓取还须提供有效 `position2`。只处理 `无遮挡正面` 和 `无遮挡背面`，其他标签与 retained 消息被忽略。位置单位为米，姿态是 `body_base` 下的 `xyzw` 单位四元数；位置是夹具尖端目标，法兰到尖端默认沿局部 +Z 相距 0.20 m。第一次按 `position1.Y≥0` 选左臂，否则选右臂；第二次按第一件所在手臂和流程分支选臂，详见 [抓取流程 2.0](抓取流程2.0.txt) 与 [流程图](抓取流程2.0.mmd)。

```bash
mosquitto_pub -h 127.0.0.1 -t /vision_target -m \
  '{"label":"无遮挡正面","position1":{"x":0.65,"y":0.15,"z":0.50},"position2":{"x":0.65,"y":0.15,"z":0.45},"orientation":{"x":0,"y":0,"z":0,"w":1}}'
```

上述 MQTT 数值仅示意消息格式，不能直接作为现场抓取目标。抓取过程和异常处理见 [scene 功能包说明](ros2_ws/src/scene/README.md)。

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
