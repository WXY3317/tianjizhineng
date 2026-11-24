# Agent 交接说明

本文件供接手本项目的 agent 使用。工作目录是本文件所在的项目根目录。**每轮对话结束前都要更新本文件的“最近交接记录”**：记录用户本轮目标、实际改动、验证结果、是否部署到 tianji、未完成事项。即使本轮只回答问题，也至少追加一条简短记录；只写已核实的事实，不把计划写成已完成。保留最近的重要记录，旧记录可合并归档，避免文件无限增长。更新时不要覆盖其他 agent 或用户刚写的内容。

## 项目与入口

- ROS 2 Humble 工作区在 `ros2_ws/`；主要包为 `gento_driver`（SDK 桥接）、`gento_control`（运动控制与逆解）、`gento_feedback`、`scene`（MQTT 抓取流程）、`gripper`。
- 项目总览见 `README.md`、`ros2_ws/README.md`；话题见 `ROS2_TOPICS.md`；场景细节见 `ros2_ws/src/scene/README.md`。
- **以代码为实际行为依据**：场景状态机在 `ros2_ws/src/scene/scene/node.py`，参数在 `ros2_ws/src/scene/launch/scene.launch.py`，预设关节角和位姿在 `ros2_ws/src/scene/config/arm_positions.json`。`抓取流程2.0.txt` 与 `抓取流程2.0.mmd` 是当前流程说明，修改流程时同步更新。`抓取流程.txt` 是较早版本，可能过时。
- `ros2_ws/run_all_first.sh` 用于夹爪重新标零后启动；`ros2_ws/run_all_second.sh` 沿用同一上电周期的零位。二者共用 `ros2_ws/run_all_common.sh`。`/scene/start` 的 `start` 同时向两夹爪发 0 mm，双方到位后双臂到 `pre`、躯干到 `body_photo`。
- `/scene/start` 的 `end` 命令要求 scene 空闲且无附着工件：躯干六轴到 `body_home`，双臂同步到 `home`，然后向两个夹爪同时发出 0 mm 目标并等待双方反馈。

## 当前抓取流程要点

- `run_all_*` 启动时 scene 默认订阅 MQTT `/vision_target`；单独 `run_scene.sh` 默认订阅 `vs/workpiece/6d_pose`。只处理非 retained 的 `无遮挡正面`、`无遮挡背面`；自动抓取须有有效 `position1`、`position2`。
- 第一次按 `position1.Y≥0` 选左臂，否则选右臂。`-0.10≤Y≤0.10 m` 时先转躯干 J6：左 `-0.5`、右 `+0.5 rad`。每次抓取按 `position1.X`：`X≤0.60 m` 躯干沿 body_base X 后移 `0.25 m`；`X≥0.75 m` 前移 `0.25 m`。
- `position1` 到 `position2` 后使能两夹爪、张开抓取侧到 27 mm 或满足电流保持条件、自动附着并回退。回退后抓取臂 J1 绝对目标左 `+1.5`／右 `-1.5 rad`，躯干 J4 从最新反馈角 `+0.15 rad`，再到 `pre`；第一次必须回 `body_photo`。
- 首件正面：第二件由另一臂抓，不交接。首件背面：收到第二条目标后先交接首件，第二件仍由第一次抓取臂抓。第二次转 J6 的阈值和目标见 `抓取流程2.0.txt`；不要凭记忆改符号或边界。
- 两次抓取后躯干六轴回零、双臂到 `home`、左右夹爪依次到 0 mm。`/scene/status` 为 `std_msgs/msg/String`，其 `data` 是 JSON：`state` 为 `running`／`success`／`failed`，`event` 为具体步骤；只有最后一只夹爪闭合确认后才发 `success`、`event=post_grasp_completed`，错误用 `failed`、`event=rejected` 和 `reason`。状态表示程序流程，不证明实物抓牢。

## 验证与部署

- 场景测试：`PYTHONPATH=ros2_ws/src/scene python3 -m pytest -q ros2_ws/src/scene/test`。
- 本机构建 scene：先加载 `/opt/ros/humble/setup.bash` 和 `ros2_ws/install/setup.bash`，再用 `colcon build --base-paths ros2_ws/src --packages-select scene`；若使用新的独立 install 目录，需先构建 gripper 等依赖。
- 用户明确要求部署时，从本机执行 `cd ros2_ws && ./deploy_arm.sh`。脚本先检查远端架构与运行进程、交叉编译 ARM64 SDK、rsync 项目，再在远端构建。默认 SSH 别名 `tianji`，目标路径 `/home/nvidia/wxy/tianjizhineng`。脚本**不会自动停止机器人节点，也不会启动机器人**；运行节点会令部署预检失败。远端可用 `ssh tianji` 验证。不要把本地构建等同于远端已部署。
- 涉及实机运动或部署时，先看当前设备状态和用户授权。不要为了验证文档而发送运动命令。

## 工作区注意事项

- 本工作区当前存在多处尚未提交的用户/前序 agent 改动，包括场景代码、文档、启动脚本；不要 `git reset`、`git clean` 或覆盖不相关改动。`gripper_RS485_so/tmp/` 是未跟踪目录，未经确认不要清理。
- 用户通常用中文交流。变更后说明改了什么、怎样验证、是否部署；没有实测就明确写未实测。

## 最近交接记录

### 2026-11-24：推送当前项目改动到 GitHub

- 用户要求先推送到 GitHub。已确认本地分支 `feature/leg-j1-arm-targets` 与 `origin` 起初同进度；提交并推送当前项目代码、文档和启动脚本改动，提交号 `5511354`。
- `git diff --cached --check` 与三个 `run_all_*` 脚本的 `bash -n` 检查通过；本轮未新增实机测试或部署到 tianji。未跟踪的 `gripper_RS485_so/tmp/` 是临时资料，未纳入提交，也未清理。

### 2026-11-24：分析护栏碰撞风险

- 用户在 `优化.md` 提出夹具尖端在 `body_base` 下 Z<0.87 m 时需保持 |Y|≤0.25 m，并询问当前抓取流程风险。本轮只读检查 `scene` 状态机和 `arm_positions.json`，将潜在风险逐段写入 `优化.md`：pre/home 关节动作、抓后 J1、躯干 J6/X/J4/恢复动作、背面交接、视觉目标下降与回退。
- 代码尚无按该包络对两只夹具尖端完整轨迹的校验；静态分析不能确认哪一段实机必然碰栏。本轮未修改运动代码、未构建、未部署到 tianji、未发送运动命令。后续若实现保护，应先明确高度与边界余量及处理违规目标的策略。

### 2026-11-24：同步夹具末端位姿反馈到 tianji

- 用户要求同步到 tianji。部署前 SSH 确认远端为 `aarch64`，未发现正在运行的 ROS 机器人节点；执行 `ros2_ws/deploy_arm.sh` 成功，远端 6 个包构建完成。
- 远端核对 `arm_positions.json`：`pre` 双臂新关节角已到位，`body_photo` 高度为 0.70 m。新增的左右夹具末端 `body_base` 位姿反馈源码已随项目同步；本轮没有启动机器人或实机订阅话题验证。

### 2026-11-24：发布夹具末端在 body_base 下的位姿

- 用户要求持续发布左右夹具末端位姿，并说明自己已修改 `arm_positions.json` 的 `pre` 和 `body_photo`。本轮保留用户配置：`pre` 左右臂 J2 为 1.57 rad、J3 为 ∓1.57 rad，`body_photo` 高度为 0.70 m；同步更新了原先写死 0.60 m 的配置测试断言。
- `gento_feedback` 新增 `/gento_feedback/left/tool_tip_pose_body_base` 和 `/gento_feedback/right/tool_tip_pose_body_base`，类型为 `geometry_msgs/msg/PoseStamped`。基于对应法兰在 `body_base` 中的位姿，沿法兰局部 +Z 加 `tool_tip_offset_z_m` 计算；`dual_arm.launch.py` 将 scene 的同名偏移参数传给反馈节点。更新话题文档。
- 本地反馈与配置测试共 19 项通过；本地 `gento_feedback`、`gento_driver` 构建通过。完整 scene 测试中另有 31 项因现有测试假时钟返回非 ROS `Time` 对象失败，尚未修复。本轮未部署到 tianji，未启动机器人或发送运动命令；新话题尚未在实机确认。

### 2026-11-24：接手阅读项目文件与交接文档

- 用户要求读取项目文件并查看 `AGENTS.md`。本轮已阅读 `AGENTS.md`、`README.md`、`ROS2_TOPICS.md`、`抓取流程2.0.txt`、`优化.md`、`ros2_ws/README.md`、`ros2_ws/src/scene/README.md` 及 `ros2_ws/src/scene/config/arm_positions.json`，并核对了工作区目录结构与 `git status`。
- 当前分支 `feature/leg-j1-arm-targets` 存在未提交改动与未跟踪文件；本轮未修改业务代码、未运行测试、未部署到 `tianji`、未发送运动命令，等待用户下一步任务。

### 2026-11-24：核对夹具尖端在 body_base 下的位姿反馈

- 用户询问是否发布夹具末端位姿。只读检查 `gento_feedback/node.py` 与 tianji 当前 ROS 话题：有 `/gento_feedback/left/tcp_pose_body_base`、`/gento_feedback/right/tcp_pose_body_base`，它们是手臂第 7 轴法兰/TCP 的 `body_base` 位姿；没有持续发布的左右夹具尖端专用话题。
- `/scene/workpiece_pose` 话题存在，但 scene 仅在工件已附着且收到新鲜持有臂法兰反馈时发布工件估计位姿，交接时还使用相对变换，不能等同于始终可用的夹具尖端反馈。默认夹具尖端可由法兰位姿加法兰局部 `+Z` 方向 0.20 m 计算，姿态在轴平行假设下相同；偏移由 `scene_tool_tip_offset_z_m` 配置。
- 本轮未改代码、未部署、未发送运动命令。

### 2026-11-24：将双臂关节命令拆为左右单臂

- 用户要求把上一轮的 14 轴同步关节目标改为左右臂分别发送。已核对单臂话题为 `/gento_control/left/target_joints` 与 `/gento_control/right/target_joints`，各用 `sensor_msgs/msg/JointState`、七个对应关节名和七个绝对弧度值，时间戳为 `now`；角度沿用用户上一轮反馈。
- 本轮只给出两条命令，未向设备发送、未部署。两条独立命令不保证同步启动。

### 2026-11-24：根据双臂反馈生成同步关节目标命令

- 用户提供 `/gento_feedback/joint_states` 的左右共 14 轴反馈，左臂约 `[1.9000003303060669, 0.8000065692366409, -2.000007696445342, -1.000003848222671, 0, 0, 0]`，右臂约 `[-1.9000003303060669, 0.8000065692366409, 2.000007696445342, -1.000003848222671, 0, 0, 0]`；询问同步发送命令。已核对控制入口为 `/gento_control/dual/target_joints`，消息类型 `sensor_msgs/msg/JointState`，14 个有序关节名和 14 个绝对弧度值，时间戳使用 `now`。
- 本轮只生成命令，未发送到设备、未部署。此前控制器曾报 Arm1 仍在运动及 Arm0 IDLE 超时；用户执行前应确认现场与控制器状态正常。

### 2026-11-24：根据躯干反馈生成 Z +10 cm 命令

- 用户提供 `/gento_feedback/body/tcp_pose`：位置 `(-0.05000427268604594, 0, 0.6000001293996642) m`、四元数 `(0, 0.38940529205642094, 0, 0.9210665114520523)`，询问 Z +10 cm 话题命令。目标 Z 计算为 `0.7000001293996642 m`，X、Y、四元数不变；应使用带 `stamp: now` 的 `/gento_control/body/target_pose` `geometry_msgs/msg/PoseStamped`，坐标系 `body_base`。
- 本轮只计算并给出命令，未发送到机器人、未改控制代码、未部署。发送前需确认反馈仍代表当前姿态，且控制节点和现场状态正常。

### 2026-11-24：检查并清理故障启动残留进程

- 用户授权停止上轮 `run_all_second.sh` 启动的远端进程。通过 SSH 核对原日志 PID `2296252`、`2296253`、`2296295`、`2296297`、`2296299`、`2296301`、`2296303`、`2296305`，均已不存在；再次按对应的 launch、`run_all_second.sh` 和 install 可执行路径搜索，未找到残留。因此没有实际发送 kill 信号，避免影响其他进程。
- 进程退出不代表控制器已确认 IDLE；上轮 `SDK=-10` 的状态仍需现场确认。未连接 SDK、未发送运动命令、未部署。

### 2026-11-24：`run_all_second.sh` 启动时桥接失败

- 用户贴出 tianji 启动日志：`FX_L1_State_SwitchToPositionMode: Arm1 ... it is moving`，主异常为 `arm1 POSITION failed: SDK=-6`。本地 SDK `FXErrorCode.h` 将 `-6` 定义为操作前置条件不满足；C SDK 日志明确当时 Arm1 被控制器判定仍在运动。`Bridge.__init__` 的 `engine.connect()->reset_errors()` 尝试切 POSITION 失败后走 `engine.close()`，其 Arm0 切 IDLE 再报 `SDK=-10`（操作超时），属于清理阶段的次生错误，安全 IDLE 状态未确认。
- 只读 SSH 检查：该次 `ros2 launch`、夹爪、feedback、scene、control 进程仍在，bridge 进程已退出。未连接机器人 SDK、未发送停止或运动指令。应由现场确认机器人停止，若仍在运动使用实体急停；再停止残留 launch、检查控制器状态，确认安全后才重新启动。不能从当前日志确定先前运动来自旧任务还是外部控制。
- 本轮只读诊断并答复，未改控制代码、未部署。

### 2026-11-24：解释抓取臂回退到 position1 后的流程

- 用户询问回退到 `position1` 后的当前流程。本轮对照 `ros2_ws/src/scene/scene/node.py` 的 `attach_workpiece`、`send_post_grasp_next` 和完成反馈分支确认：先按最新反馈仅改抓取臂 J1 为左 `+1.5`／右 `-1.5 rad`，完成后仅改躯干 J4 为当前值 `+0.15 rad`；首件正面双臂同步到 `pre`、首件背面抓取臂单独到 `pre`，随后首件均恢复 `body_photo` 并等待第二条合法 MQTT。首件背面收到第二条后才交接首件；第二次抓取回退后双臂同步到 `pre`，按本次是否动过躯干恢复 `body_photo`，然后躯干六轴回零、双臂同步到 `home`、左右夹爪依次闭合到 0 mm，最后报告 `post_grasp_completed`。
- 本轮只读核对并答复，未改控制代码、未部署、未发送运动命令。

### 2026-11-24：`start` 两夹爪并行关闭及 Ctrl+C 说明

- 用户更新 `优化.md`：`/scene/start` 的 `start` 也要两夹爪同时关闭，并询问发送 `start/end` 时 Ctrl+C 的效果。已将 `start` 改为同阶段向左右夹爪都发送 0 mm，并分别等待反馈后再执行 `pre`、`body_photo`。`end` 继续使用同一并行夹爪机制。RS485 总线共用，物理起动时刻不能保证完全一致。
- 代码检查确认：在 `ros2 topic pub --once` 的终端按 Ctrl+C 只终止发布进程；消息到达 scene 后，流程继续。在 `run_all_first.sh` / `run_all_second.sh` 的终端按 Ctrl+C 会给两个 launch 发 SIGINT；driver 关闭时尝试停止活动轨迹，夹爪节点析构时尝试禁用电机。不能把 Ctrl+C 当作已确认的安全停止；另有 `/gento_driver/stop` 和实体急停。说明已补入 README 和 scene README。
- scene 125 项测试通过，本地构建和 `git diff --check` 通过。沿用用户前一轮对 `优化.md` 的部署要求运行 `deploy_arm.sh`；远端无运行机器人节点，SDK 校验通过，远端 6 包构建成功。未发送实机运动命令，`start/end` 的实机行为尚未验证。

### 2026-11-24：新增 `/scene/start` 的 `end` 命令并部署

- 用户在 `优化.md` 要求 `ros2 topic pub --once /scene/start std_msgs/msg/String '{data: end}'`：躯干六轴回零、双臂同步到 `arm_positions.json` 的 `9=home`、两夹爪关闭到 0 mm。已在 scene 状态机实现顺序等待躯干和双臂完成，随后向两夹爪同时发布命令并分别确认到位；失败或 15 秒超时停止。仍要求 scene 空闲且没有附着工件。共用 RS485 总线，因此命令无需等待另一侧但无法保证两电机物理起动时间完全一致。
- 已更新根目录及工作区 README、scene README、`ROS2_TOPICS.md` 和测试。scene 全部 125 项测试通过，本地 scene 构建通过；`git diff --check`、Python 语法检查通过。
- 已运行 `ros2_ws/deploy_arm.sh`：远端无运行机器人节点，ARM64 SDK 校验通过，远端 6 个 ROS 包构建成功。没有启动机器人或发送运动命令。结束流程尚未在实机执行验证。

### 2026-11-24：排查躯干位姿直线运动的抖动

- 用户询问躯干发位姿后运动中抖动原因。本轮只读检查：`/gento_control/body/target_pose` 在 `body_cartesian.py` 规划 50 Hz 末端直线位置与姿态 SLERP，经逐点 IK 和关节三次样条；`gento_driver/node.py` 由 Python 20 ms 定时器逐点调用 SDK `runtime_set_joint_pos_cmd(3, ...)`。`/target_pose_ptp` 与自动抓取的 `body_photo_restore` 仍是关节空间运动，不保证末端直线。
- 只读查看远端近期轨迹记录：最近一条躯干直线任务为 861 点、规划 17.2 s、记录 300 个反馈样本、未丢样，终态 completed，最大 TCP 到线段偏差约 3.32 mm。没有对应的抖动发生时刻或逐点命令时间戳，不能据此确定根因；调度/SDK 命令间隔、跟踪误差、IK 与机械因素都需结合具体任务分析。
- 本轮未修改控制代码、未部署、未发送运动命令。后续若要修复，请先定位抖动对应的轨迹 JSON 和时刻，分析关节反馈与计划、命令发送抖动，再决定是否修改执行器。

### 2026-11-24：同步到 tianji

- 用户要求同步到 tianji。确认远端为 `aarch64` 且没有运行中的机器人节点后，执行 `ros2_ws/deploy_arm.sh`：ARM64 SDK 交叉编译并在远端通过 SHA256、架构和动态加载检查；项目文件同步完成；远端 `gento_interfaces`、`gento_feedback`、`gripper`、`gento_control`、`scene`、`gento_driver` 六个包均构建成功，launch 参数检查通过。
- 本次包含上一轮 `/scene/status` 状态字段、文档及本交接文件；没有启动控制节点或发送运动命令。部署后的实机抓取流程尚未测试。

### 2026-11-24：新增本交接文件

- 用户要求写 agent 交接文档，并要求每轮对话更新。已新增根目录 `AGENTS.md`，建立更新规则、代码入口、流程要点、验证和部署说明。
- 当轮只写文档，未向 tianji 部署，也未发送实机运动命令。更早一轮新增 `/scene/status` 的 `running/success/failed` 与 `event` 字段；scene 测试 123 项和本地构建通过。上述改动已在随后一次部署中同步到远端。
- 后续 agent 接手时先看 `git status --short`，核对本文件与当前代码；完成每轮任务前更新这里，并在答复中注明部署状态。
