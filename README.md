# 天机机器人 ROS 2 常用命令

## 部署到 tianji（在本机运行）

部署前停止 tianji 上运行的机器人和夹爪节点。

```bash
cd ros2_ws
./deploy_arm.sh
```

## 启动（在 tianji 上运行）

一条命令启动夹爪、真机控制和视觉场景；按夹爪是否需要重新标零选择脚本。

```bash
cd /home/nvidia/wxy/tianjizhineng/ros2_ws

# 首次启动或断电后：夹爪重新标零并使能
./run_all_first.sh

# 同一上电周期内已标零：沿用零位并使能
./run_all_second.sh

# 只预览视觉目标，不执行场景运动
./run_all_second.sh scene_dry_run:=true
```

启动后，可在另一个已加载工作区环境的终端发送：

```bash
# 同时命令左右夹爪闭合到 0 mm，确认双方到位后双臂到 pre、躯干到 body_photo
ros2 topic pub --once /scene/start std_msgs/msg/String '{data: start}'
# 躯干六轴回零、双臂同步到 home，再同时命令两夹爪闭合到 0 mm
ros2 topic pub --once /scene/start std_msgs/msg/String '{data: end}'
ros2 topic echo /scene/status  # state: running / success / failed；event: 具体步骤
```

按 Ctrl+C 会退出该启动脚本管理的两个 launch；部署前先确认它们已退出。
在发送 `ros2 topic pub --once /scene/start ...` 的终端按 Ctrl+C，只会中断该发布命令；消息已送达后，scene 仍会继续执行。若要中断正在执行的运动，在另一终端发送下面的停止话题，或使用实体急停。

## 停止运动

```bash
ros2 topic pub --once /gento_driver/stop std_msgs/msg/Empty '{}'
```

详细说明：[工作空间](ros2_ws/README.md) · [夹爪](ros2_ws/src/gripper/README.md) · [场景](ros2_ws/src/scene/README.md) · [ROS 2 话题](ROS2_TOPICS.md)。
