# 天机机器人 ROS 2 真机控制

通过 ROS 2 Humble 连接实体控制器，读取双臂、躯干和头部反馈，控制双臂及六轴躯干。
driver 独占 SDK 连接，control 规划轨迹，feedback 发布位姿和状态；独立的 `scene`
包可将 MQTT 视觉目标转换为双臂运动目标。

| 目录 | 内容 |
|---|---|
| `ros2_ws/src/gento_driver` | 唯一 SDK、原始读写与独立执行保护 |
| `ros2_ws/src/gento_control` | 运动任务、路径规划与执行管理 |
| `ros2_ws/src/gento_feedback` | 反馈汇总、诊断和轨迹记录 |
| `ros2_ws/src/gento_interfaces` | ROS 查询服务和轨迹 Action 定义 |
| `ros2_ws/src/scene` | MQTT `/vision_target` 视觉目标转换和顺序执行 |
| `ros2_ws/src/gripper` | 左右夹爪的 RS485 ROS 接口 |
| `ros2_ws/README.md` | 启动部署、常用命令与功能包说明 |
| `tianji-robot-SDK` | Gento 原厂 SDK 与示例 |

功能包职责、启动方式及新命令见 [工作空间说明](ros2_ws/README.md)。

## 启动

一并启动真机控制使用 `ros2_ws/run_real_robot.sh`；分开启动可用
`run_feedback_only.sh` 加 `run_control_only.sh`。场景节点可单独使用
`run_scene.sh`，或在真机启动时传入 `start_scene:=true`。场景默认关闭。
具体命令见 [工作空间说明](ros2_ws/README.md)。

真机控制模式启动时会对处于 ERROR 的对象清错、确认 IDLE，再切换到 POSITION。
收到合法目标后执行运动；启动本身不下发目标或回零。

需要 ROS 2 Humble、Python 3.10、colcon、`paho-mqtt`，以及与固件和主机架构匹配的 SDK 动态库。
下面的 IP 是机器人控制器地址，不是 tianji 主机的 SSH 地址：

```bash
cd ros2_ws
# 首次构建并连接真机，等待目标命令
./run_real_robot.sh --build ip:=6.6.7.190
# 后续连接；收到合法目标后直接执行
./run_real_robot.sh ip:=6.6.7.190

# 另一终端：接收 MQTT 视觉目标并发布双臂目标
./run_scene.sh
```

双臂位姿目标使用固定在躯干第 1 轴中心的 `body_base`；`BD_J6` 是随躯干运动的
第 6 轴坐标系。左、右法兰相对这两个坐标系的反馈分别在
`/gento_feedback/{left,right}/tcp_pose_body_base` 和 `.../tcp_pose_bd_j6`。
控制器 DH 中臂 SDK 基座到第 1 轴的 158.6 mm 偏移已计入安装变换。
话题格式和坐标变换见 [ROS 2 话题说明](ROS2_TOPICS.md)。

MQTT Broker 在 tianji 监听 `0.0.0.0:1883`，本机可通过 `127.0.0.1:1883`、
局域网设备可通过 `192.168.2.121:1883` 连接；启动不依赖固定局域网 IP 已就绪。
场景订阅 **`/vision_target`**；目标点在 `BD_J6` 下 Y 为正由左臂执行，为负由右臂执行，
`position1` 完成后再执行 `position2`。消息格式见 [scene 说明](ros2_ws/src/scene/README.md)。

当前控制接口没有碰撞检测；运行前应核对实机目标与路径。

## 获取代码

```bash
git clone git@github.com:WXY3317/tianjizhineng.git
cd tianjizhineng
git switch feature/leg-j1-arm-targets
```

Gento SDK 保留其随附的许可证与说明。


软件停止：

```bash
ros2 topic pub --once /gento_driver/stop std_msgs/msg/Empty '{}'
```

## 状态恢复与退出流程

启动与显式 reset_error 采用原厂示例的状态分支：ERROR 才调用 SDK reset_error；正常 IDLE 不复位；其他模式先切 IDLE并确认，不额外复位。随后完成模型、限位与反馈检查并设置 POSITION。复位失败仍保持故障锁，不绕过已有错误。

正常 Ctrl+C 退出时先停止节点定时回调，逐个检查双臂和躯干：正常非 IDLE 对象切回 IDLE并确认；已有 ERROR 保留、不自动清错。之后 cleanup 销毁运动学资源，unlink 断开通信。退出中的任何 IDLE 切换失败都会尝试受控对象软急停，仍释放 SDK 资源并记录错误；不会因失败而继续运动。运动中退出维持软急停流程，不自动解除 ERROR。链路断开时不能确认 IDLE，会明确报告；重复清理不会重复操作控制器。

这解决正常退出时对象仍留在 POSITION 的软件遗漏，不代表已经证明或解决控制电源欠压、PDO 异常等现场故障。
