# 天机机器人 ROS 2 常用命令

## 部署到 tianji（在本机运行）

部署前停止 tianji 上运行的机器人和夹爪节点。

```bash
cd ros2_ws
./deploy_arm.sh
```

## 启动（在 tianji 上运行）

一条命令启动夹爪、真机控制和视觉场景；按夹爪是否需要重新标零选择模式。

```bash
cd /home/nvidia/wxy/tianjizhineng/ros2_ws

# 首次启动或断电后：夹爪重新标零并使能
./run_all.sh first

# 同一上电周期内已标零：沿用零位并使能
./run_all.sh second

# 只预览视觉目标，不执行场景运动
./run_all.sh second scene_dry_run:=true
```


## 停止运动

```bash
ros2 topic pub --once /gento_driver/stop std_msgs/msg/Empty '{}'
```

详细说明：[工作空间](ros2_ws/README.md) · [夹爪](ros2_ws/src/gripper/README.md) · [场景](ros2_ws/src/scene/README.md) · [ROS 2 话题](ROS2_TOPICS.md)。
