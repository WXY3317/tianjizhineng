# 天机机器人 ROS 2 常用命令

## 部署到 tianji（在本机运行）

部署前停止 tianji 上运行的机器人和夹爪节点。

```bash
cd ros2_ws
./deploy_arm.sh
```

## 启动（在 tianji 上运行）

每个运行中的节点使用单独终端；两个夹爪脚本按情况选一个。

```bash
cd /home/nvidia/wxy/tianjizhineng/ros2_ws

# 首次启动或断电后：左右夹爪自动标零
./run_gripper_frist.sh

# 同一上电周期内已标零：沿用零位重启夹爪节点
./run_gripper_second.sh

# 真机驱动与控制
./run_real_robot.sh

# 视觉场景：默认实际执行，允许工具 Z 轴旋转
./run_scene.sh

# 视觉场景：只预览，不发送运动命令
./run_scene.sh dry_run:=true
```


## 停止运动

```bash
ros2 topic pub --once /gento_driver/stop std_msgs/msg/Empty '{}'
```

详细说明：[工作空间](ros2_ws/README.md) · [夹爪](ros2_ws/src/gripper/README.md) · [场景](ros2_ws/src/scene/README.md) · [ROS 2 话题](ROS2_TOPICS.md)。
