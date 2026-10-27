# 天机双臂机器人真机控制（real 分支）

通过 ROS 2 话题连接实体控制器，读取双臂关节反馈和 TCP 位姿，并控制躯干 6 个关节，
调用 Gento SDK 完成 FK/IK、关节轨迹规划及执行。

| 目录 | 内容 |
|---|---|
| `ros2_ws/src/gento_dual_arm` | 双臂真机 ROS 2 控制功能包 |
| `ros2_ws/src/gento_dual_arm/REAL_TEST.md` | 包内单臂、双臂命令行测试说明 |
| `tianji-robot-SDK` | Gento 原厂 SDK 与示例 |

## 启动

ROS 节点每次连接成功后自动清除双臂与躯干错误、确认 IDLE，并切换到 POSITION。收到合法目标后执行运动；启动不下发目标或回零。

需要 ROS 2 Humble、Python 3.10、colcon，以及与真机固件和主机架构匹配的 SDK 动态库。
将 IP 替换为实际控制器地址：

```bash
cd ros2_ws
# 首次构建并连接真机，等待目标命令
./run_real_robot.sh --build ip:=6.6.7.190
# 后续连接；收到合法目标后直接执行
./run_real_robot.sh ip:=6.6.7.190
```

话题格式、坐标系、目标发布和停止示例见 [双臂真机使用说明](ros2_ws/REAL_ROBOT.md)，
完整参数见 [工作空间说明](ros2_ws/README.md)。

本分支已移除仿真功能包及启动脚本；仿真代码保留在其他分支。
当前接口没有碰撞检测，尚未进行硬件联调。

## 获取代码

```bash
git clone --branch real git@github.com:WXY3317/tianjizhineng.git
cd tianjizhineng
```

Gento SDK 保留其随附的许可证与说明。


急停：
ros2 topic pub --once /gento_dual_arm/stop std_msgs/msg/Empty '{}'

## 状态恢复与退出流程

启动与显式 reset_error 采用原厂示例的状态分支：ERROR 才调用 SDK reset_error；正常 IDLE 不复位；其他模式先切 IDLE并确认，不额外复位。随后完成模型、限位与反馈检查并设置 POSITION。复位失败仍保持故障锁，不绕过已有错误。

正常 Ctrl+C 退出时先停止节点定时回调，逐个检查双臂和躯干：正常非 IDLE 对象切回 IDLE并确认；已有 ERROR 保留、不自动清错。之后 cleanup 销毁运动学资源，unlink 断开通信。退出中的任何 IDLE 切换失败都会尝试受控对象软急停，仍释放 SDK 资源并记录错误；不会因失败而继续运动。运动中退出维持软急停流程，不自动解除 ERROR。链路断开时不能确认 IDLE，会明确报告；重复清理不会重复操作控制器。

这解决正常退出时对象仍留在 POSITION 的软件遗漏，不代表已经证明或解决控制电源欠压、PDO 异常等现场故障。
