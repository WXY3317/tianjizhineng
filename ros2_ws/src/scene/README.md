# scene: MQTT 视觉目标到双臂位姿命令

启动真机驱动后，在另一个终端运行：

```bash
cd ~/wxy/tianjizhineng/ros2_ws
./run_scene.sh
```

首次构建或源码更新后使用 `./run_scene.sh --build`。脚本自动加载 ROS 和工作区环境。

默认 `dry_run:=false`，收到有效目标后会执行运动。预览时使用 `./run_scene.sh dry_run:=true`；预览只在 `/scene/status` 的 `previewed` 事件中发布换算后的法兰位置和姿态，不发送运动命令。

MQTT 视觉目标是 `body_base` 下的工件位姿；工件原点作为夹具尖端目标，工件局部 Z 轴作为夹具局部 Z 轴方向，绕该轴的角度允许调整。控制器逆解目标表示第 7 轴法兰。默认夹具尖端相对法兰沿法兰局部 +Z 方向偏移 0.20 m，坐标轴平行。scene 直接按目标姿态减去这段偏移，得到发给手臂的法兰目标。请先在 `dry_run:=true` 核对法兰目标方向；若实际沿局部 -Z，使用 `tool_tip_offset_z_m:=-0.20`。随真机启动时对应参数为 `scene_tool_tip_offset_z_m`。

也可以在真机启动命令中传入 `start_scene:=true` 一起启动。此选项默认关闭；
收到新 MQTT 目标后才会向双臂控制话题发布命令。

默认订阅本机 Broker 的 MQTT 主题 **`vs/workpiece/6d_pose`**。
如需其他 Broker，可向脚本传入 `mqtt_host:=... mqtt_port:=... mqtt_topic:=...`。

消息以 UTF-8 发送。`label` 是字符串；支持标准 JSON，也兼容未加引号的顶层字段名：

```json
{
  "label": "无遮挡正面",
  "position1": {"x": 0.12, "y": -0.08, "z": 0.39},
  "position2": {"x": 0.12, "y": -0.08, "z": 0.35},
  "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
}
```

当前执行 `无遮挡正面` 和 `无遮挡背面`。正面和背面均按 `position1` 的 Y 值选臂：Y ≥ 0.05 m 用左臂，Y ≤ -0.05 m 用右臂。正面抓取后回退 `position1`、该臂 `pre` 姿态，再到对应的 `front_left_post_grasp_joints` 或 `front_right_post_grasp_joints`；不启动左右臂交接。其他四种标签会拒绝。背面右臂交接的两组关节角默认未配置，右臂背面目标会在运动前拒绝；测量后用 `right_post_grasp_joints` 与 `left_handover_joints` 设置各七个弧度值。

位置单位为米，姿态为 `body_base` 下的 `xyzw` 单位四元数，两个目标共用姿态。
按 `position1`、`position2` 顺序执行。
两点分别表示同一只手臂的预抓取位置和工件实际位置；两点选出的手臂不同时会拒绝。
`position2` 可以省略；恰好为 `(0,0,0)` 时视为占位并跳过。
两种模式下，非占位目标 Y = 0 会被拒绝。MQTT retained 消息被忽略。

节点直接使用 MQTT 给出的 `body_base` 工件位姿，换算为法兰位姿后依次发布：
`position1` 使用 `/gento_control/{left,right}/target_pose_ptp`，按关节空间轨迹到目标；
`position2` 使用 `/gento_control/{left,right}/target_pose`，按法兰直线轨迹到目标。
若 `position1` 通过绕工具局部 Z 轴旋转才到达，`position2` 直接继承该次选定的法兰目标姿态和旋转角；
第二步只改变位置，姿态保持相同。若此姿态下 `position2` 无法执行，则拒绝第二步，不再搜索其他角度。
第一步收到控制节点的 `completed` 后等待 `interval` 秒再发送第二步（默认 1.0 秒，另有最多约 50 ms 的定时器调度间隔）。
单独启动可用 `interval:=秒数` 覆盖；随真机一起启动可用 `scene_interval:=秒数` 覆盖。
执行期间收到的新 MQTT 消息会被忽略。`position2` 为全零占位或省略时不会发送第二步。
需要控制节点；绕轴候选先尝试原始姿态，再按绝对角度从小到大尝试。
可在 `/scene/status` 查看发送、完成及拒绝原因。

到达 `position2` 后，scene 向所选手臂夹爪发送 27 mm 命令，等待夹爪到位或触发电流阈值并保持当前位置。电流保持可能发生在 27 mm 之前，不能单凭反馈确认抓牢；收到 `awaiting_attach` 状态后，操作者确认实物已夹牢，再手动调用下面的附着服务。背面模式会依次回退到 `position1`、抓取臂 `pre`、躯干六轴回零、该臂交接点，再交给另一只手臂。左抓取使用 `left_post_grasp_joints` 和 `right_handover_joints`；右抓取使用 `right_post_grasp_joints` 和 `left_handover_joints`，接收臂直线靠近 7 cm、抓取、原抓取臂松开后回退。正面模式会依次回退到 `position1`、所选手臂 `pre`、该臂的正面终点关节角，不进行交接。每段运动均等待完成反馈才继续。

scene 将所选绕轴角和法兰到夹具尖端的偏移作为固定变换，
每次收到该手臂的新鲜法兰位姿，就在 `/scene/workpiece_pose` 发布工件在 `body_base` 下的估计位姿。
四段运动期间也持续更新，工件姿态按法兰反馈乘以先前局部 Z 轴旋转的逆旋转求得。`header.stamp` 沿用法兰位姿的时间戳。服务成功只表示退回命令已发送；左臂到交接点查看 `handover_started`，全部完成查看 `post_grasp_completed`，失败查看 `rejected`。释放工件时调用分离服务，停止发布。必须由操作者或抓取流程在确认夹牢后调用附着服务；
仅到达 `position2` 不表示已夹牢。scene 重启后需重新建立附着状态。

```bash
ros2 service call /scene/attach_workpiece std_srvs/srv/Trigger '{}'
ros2 topic echo /scene/status
ros2 topic echo /scene/workpiece_pose
ros2 service call /scene/detach_workpiece std_srvs/srv/Trigger '{}'
```

左臂到达交接点后，右臂自动先到 `right_handover_joints`；控制节点确认到位后等待 `interval`（默认 1.0 秒），再保持姿态沿 `body_base` 的 +Y 直线运动 7 cm。随后右夹爪到 27 mm 或触发电流保持，切换工件附着到右臂，左夹爪到 0 mm，右臂先退回预备交接关节位，再到 `right_arm_joint_pre` 准备姿态。右夹爪到位时根据双臂当前法兰反馈计算右法兰到工件的完整固定变换，并用右臂反馈持续发布工件位姿。`right_handover_joints` 和 `handover_y_offset_m` 可在单独启动时覆盖，随真机启动时加 `scene_` 前缀。夹爪反馈错误或 15 秒未到位时流程停止，需检查现场并重启 scene。

该位姿由法兰位姿和固定夹具几何推算，不是工件的独立视觉测量；前提是工件夹取后相对夹具不滑动。
附着期间 scene 不接受新的 MQTT 工件目标，释放后可处理下一件。

左右臂关节预设写在 `config/arm_positions.json`，单位弧度。每项包含 Int8 编号和 `name`。双臂关节姿态填写 `left`、`right` 两组七轴弧度数组；
躯干 TCP 位姿填写 `body_pose.position`（米，body_base 坐标系）和
`body_pose.orientation`（单位四元数 x/y/z/w），执行末端直线轨迹；填写 `body_pose_ptp` 的相同字段时，只求终点逆解，再由控制器执行关节位置运动，末端不保证直线。躯干关节预设填写 `body_joints` 六轴弧度数组。默认 `0=init`、`1=pre`、`2=body_photo`（关节空间运动）、`3=body_home`、
`9=home`（下班回收）。
可添加其他编号（-128～127）的姿态；`1=pre` 供抓取流程使用，应保留。
修改配置后重新启动 scene；也可用 `arm_positions_file:=/绝对路径/arm_positions.json`
指定另一份文件。通过 `/scene/arm_position`（`std_msgs/msg/Int8`）发送编号；
双臂姿态先发送左臂，确认完成后再发送右臂；躯干位姿直接发送到
`/gento_control/body/target_pose`，`body_pose_ptp` 发到 `/gento_control/body/target_pose_ptp`；躯干关节预设发送到 `/gento_control/body/target_joints`。任一步失败即停止。
此功能仅在 `dry_run:=false` 且 scene 和控制节点空闲时执行。例如：

```bash
ros2 topic pub --once /scene/arm_position std_msgs/msg/Int8 '{data: 0}'
ros2 topic pub --once /scene/arm_position std_msgs/msg/Int8 '{data: 1}'
ros2 topic pub --once /scene/arm_position std_msgs/msg/Int8 '{data: 2}'
ros2 topic pub --once /scene/arm_position std_msgs/msg/Int8 '{data: 3}'
ros2 topic pub --once /scene/arm_position std_msgs/msg/Int8 '{data: 9}'
```

随 `run_real_robot.sh` 启动时，使用 `scene_arm_positions_file:=/绝对路径/arm_positions.json` 覆盖配置。

通过 `/scene/status` 观察 `arm_position_sent`、`arm_position_step_completed` 和
`arm_position_completed`。启动或更新代码后需要重新构建并重启 scene 节点。

默认 `allow_tool_z_rotation:=true`：如果控制节点因逆解、关节限位或轨迹逆解问题拒绝 `position1` 目标，
scene 保持夹具尖端目标位置和局部 Z 轴方向不变，按 `0°、+15°、-15°、+30°、-30°…` 搜索绕轴角度。每次尝试仍由控制节点完整校验；一旦开始运动，
不再自动切换目标。`position2` 不重新搜索姿态。所有候选被拒绝时，scene 报告拒绝，不能保证任意位置都可达。
`/scene/status` 的 `sent` 事件包含当前尝试的 `tool_z_rotation_deg` 和 `attempt`。
控制节点确认 `completed` 后，`step_completed` 事件包含 `rotation_matrix`（本次绕夹具局部 Z 轴增加的 3×3 旋转矩阵 B；未换角度时为单位矩阵）和 `flange_rotation_matrix`（最终法兰姿态在 `body_base` 下的旋转矩阵）。相同数据也写入 `~/.ros/scene_last_rotation.json`，包括位置、四元数和旋转角度；可用 `last_rotation_file:=PATH` 指定其他路径。预览和被拒绝的姿态不会覆盖该文件。

若成功目标曾绕局部 Z 轴旋转非零角度，scene 还会将增量矩阵 B 和角度保存在 `~/.ros/scene_tool_z_angle.json`（可用 `tool_z_angle_file:=PATH` 改路径）。手臂随后可移动到其他位置 C；需要在 C 处反向转回时，调用 `ros2 service call /scene/restore_tool_z std_srvs/srv/Trigger '{}'`。若新鲜关节反馈显示第 6、7 轴都在 0° 附近（各不超过 0.5°），scene 只调整第 5 轴：将 `-B` 分成每段不超过 90° 的关节目标，逐段等待完成，并在发送前核对第 5 轴及其他关节反馈。目标第 5 轴超出模型限位时在运动前拒绝。每段完成后更新文件里的剩余角度；若运动中断导致实际关节角不确定，标记记录并禁止自动重试。其他第 6、7 轴姿态使用原来的笛卡尔回转，目标仍为 `R_C · B⁻¹`，保持 C 的法兰目标位置。服务返回 `success: true` 只代表首段已发送；最终结果看 `/scene/status` 的 `restore_completed` 或 `rejected`。成功后清除待回转矩阵，避免重复调用。`dry_run:=true`、scene 忙或反馈过期时拒绝调用。
若工件要求夹具固定绕轴角度，启动时设置 `allow_tool_z_rotation:=false`；
随真机一起启动时对应 `scene_allow_tool_z_rotation:=false`。

此包发布控制目标和推算的工件位姿，不直接调用 SDK；目标的直线路径逆解和执行由 `gento_control` 负责。
运行 scene 时不要同时从其他来源向同一个控制节点发送目标，以免完成状态混淆。
