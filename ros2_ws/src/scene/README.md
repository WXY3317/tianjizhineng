# scene: MQTT 视觉目标到双臂位姿命令

启动真机驱动后，在另一个终端运行：

```bash
cd ~/wxy/tianjizhineng/ros2_ws
./run_scene.sh
```

首次构建或源码更新后使用 `./run_scene.sh --build`。脚本自动加载 ROS 和工作区环境。

也可以在真机启动命令中传入 `start_scene:=true` 一起启动。此选项默认关闭；
收到新 MQTT 目标后才会向双臂控制话题发布命令。

默认订阅本机 Broker 的 MQTT 主题 **`/vision_targe`**（保留原拼写及开头的 `/`）。
如需其他 Broker，可向脚本传入 `mqtt_host:=... mqtt_port:=... mqtt_topic:=...`。

消息以 UTF-8 发送。支持标准 JSON，也兼容示意图里未加引号的三个顶层字段名：

```json
{
  "position1": {"x": 0.12, "y": -0.08, "z": 0.39},
  "position2": {"x": 0.10, "y": 0.15, "z": 0.35},
  "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
}
```

位置单位为米，姿态为 `BD_J6` 下的 `xyzw` 单位四元数，两个目标共用姿态。
按 `position1`、`position2` 顺序执行；**目标在 `BD_J6` 中的 Y > 0 选左臂，Y < 0 选右臂**。
`position2` 可以省略；恰好为 `(0,0,0)` 时视为占位并跳过。
其他 Y = 0 的目标会被拒绝。MQTT retained 消息被忽略。

节点使用 `/gento_feedback/body/tcp_pose` 中实时 `T_body_base_BD_J6`，
将两个目标都转换为固定 `body_base` 下的位姿，逐个发布至
`/gento_control/{left,right}/target_pose`。前一步收到控制节点的 `completed`
状态后才发送下一步；执行期间收到的新 MQTT 消息会被忽略。
需要控制节点和新鲜的躯干位姿反馈（默认 0.5 秒内）。
可在 `/scene/status` 查看发送、完成及拒绝原因。

此包只发布目标，不直接调用 SDK；目标的直线路径逆解和执行由 `gento_control` 负责。
运行 scene 时不要同时从其他来源向同一个控制节点发送目标，以免完成状态混淆。
