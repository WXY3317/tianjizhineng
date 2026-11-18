# RS485 夹爪底层工具

本目录保留独立的串口点动和端点标定工具 `gripper_calibrate`，以及可选的供应商运动示例。日常双夹爪 ROS 话题控制位于 [`ros2_ws/src/gripper/`](../ros2_ws/src/gripper/README.md)，其测试已放在该包的 `test/`；此处不再编译 ROS 包的旧命令行 `example/`。

## 构建

依赖 C++17、CMake >= 3.16 和 Eigen3。按目标处理器自动选择 `lib/x86` 或 `lib/arm`。

```sh
cd gripper_RS485
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target gripper_calibrate -j2
./build/gripper_calibrate --help
```

供应商运动示例默认关闭；使用 `-DGRIPPER_BUILD_EXAMPLES=ON` 可构建 `position_loop_demo`、`velocity_loop_demo`、`torque_loop_demo` 和 `zero_demo`。独立文本命令控制示例仍在 [`gripper_RS485_so/demo/`](../gripper_RS485_so/demo/README.md)，不由本目录的 CMake 构建。使用任一串口工具前先停止占用该串口的 ROS 节点。

## ROS2 话题功能包

`tianji` 上的工作区统一使用 `ros2_ws/build/`、`install/`、`log/`。部署与启动命令见 [工作区说明](../ros2_ws/README.md)。

## 右夹爪 ID 2：点动与端点标定

新增的 `build/gripper_calibrate` 是独立串口工具，操作前停止占用该串口的 ROS 节点。它固定 115200 波特率、电机方向 `+1`。`open` 发负角速度，`close` 发正角速度，速度默认 0.5 rad/s、范围 0.05～0.5 rad/s；每次最多 5 秒，结束时发送停止与失能。点动电流阈值默认 0.5 A，可用第三个点动参数指定 0～5 A，`0` 关闭此软件电流阈值；驱动报错或通信失败仍尝试停止并失能。该阈值不是硬件限流。操作时夹爪应空载、行程无遮挡，观察实物运动；不要连续顶住机械限位。命令均不自动清错。

在目标机器上构建（或用 ARM64 工具链交叉编译），以下命令在 `gripper_RS485/` 目录执行：

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target gripper_calibrate -j2
PORT=/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0
./build/gripper_calibrate "$PORT" 2 status
./build/gripper_calibrate "$PORT" 2 open 1000
```

逐次点动，靠近全开时缩短到 `open 100` 或 `open 200`。确认**机械全开**后停止运动，测量接触面间距，执行 `status`，记下 `angle_rad` 作为 `OPEN_BEFORE_ZERO`。同一位置再读一次，检查角度稳定。确认位置未变后用记录值设零；命令要求当前角度与记录值相差不超过 0.03 rad，并在写入后检查反馈接近 0 rad：

```sh
./build/gripper_calibrate "$PORT" 2 set-open-zero OPEN_BEFORE_ZERO
./build/gripper_calibrate "$PORT" 2 status
```

**保留 `OPEN_BEFORE_ZERO` 的原始数字。**零点建立后，逐次执行 `close 1000`，接近闭合时改用 `close 100` 或 `close 200`。在两侧接触面刚接触时停止，不要持续挤压。执行 `status`，记录 `angle_rad` 作为 `CLOSED_AFTER_ZERO`；用卡尺确认闭合开度约为 0 mm。此时右夹爪的零位后闭合行程应为正值。若为负值，停止，不要套用左夹爪模型。

为避免混用设零前后坐标，生成脚本要求**两个角度都在设零前的同一坐标系**。因此将闭合端读数还原为 `CLOSED_BEFORE_ZERO = OPEN_BEFORE_ZERO + CLOSED_AFTER_ZERO`，再生成右夹爪表。以下示例中的角度和宽度须替换成现场记录，`--model` 必须指向当前机器架构的动态库：

```sh
python3 generate_calibration.py \
  --model ./lib/arm/libgrip_pawl_lib.so \
  --open-width-mm 98 \
  --open-angle-rad OPEN_BEFORE_ZERO \
  --closed-angle-rad CLOSED_BEFORE_ZERO \
  --output right_calibration.csv
```

表中间 511 点来自原机构模型，不是实测值；用卡尺核对几个中间开度后再用于位置控制。ROS2 节点可通过参数 `right.calibration_path` 指向此文件；不要让 ID 2 使用默认的左夹爪表。设零或端点记录前不要用旧表发送位置控制命令。`set-open-zero` 会修改驱动器零位，仅在确认全开位置时执行。



angle_rad=-49.512298584 current_a=0 speed_rad_s=0
nvidia@vision:~/wxy/tianjizhineng/gripper_RS485$ ./build/gripper_calibrate "$PORT" 2 set-open-zero -49.5127754211
Bus SocketCan registered
SerialBus Rs485 registered
ProtocolFactory register: kaiser
ProtocolFactory register: hdriver
ProtocolFactory register: lens
ModbusProtocolFactory register: smodule_modbus
open_zero_set previous_angle_rad=-49.512298584 angle_rad=0
nvidia@vision:~/wxy/tianjizhineng/gripper_RS485$ ./build/gripper_calibrate "$PORT" 2 status
Bus SocketCan registered
SerialBus Rs485 registered
ProtocolFactory register: kaiser
ProtocolFactory register: hdriver
ProtocolFactory register: lens
ModbusProtocolFactory register: smodule_modbus
angle_rad=0 current_a=0 speed_rad_s=0
nvidia@vision:~/wxy/tianjizhineng/gripper_RS485$

右夹爪 ID 2 的 `right_calibration.csv` 已按目标/实测开度 15→14、20→18、30→27 mm 修正；原始仅端点归一化的表保存在 `right_calibration_initial.csv`。用 `refine_calibration.py` 可从原表和新的测量点重新生成，不要对已修正表重复应用同一组修正。中间位置仍需独立实测复核。

右表第二次修正使用目标/实测开度 10→10、20→20、30→30、40→38、50→47、60→56、70→65、80→74、90→82 mm；上一版本保存在 `right_calibration_v2.csv`。90～93 mm 段因实测 90→82 与全开 93→93 差异较大，必须复测，不要把该段反馈当作独立测量。

右表第三次修正仅调整 80～93 mm，使用最新复测 84→83、86→84、90→85 mm，80 mm 以下映射保持不变；上一版本保存在 `right_calibration_v3.csv`。90 mm 目标对应电机角度约 0.212 rad，接近全开机械端，复测时应监测电流并避免连续顶限位。

2026-11-02 右夹爪曾确认全开实际宽度为 85 mm，设零前角度约 -51.4629478455 rad，已在该位置重新设零。当时原 93 mm 的 `right_calibration.csv` 暂不可用于位置控制；需在此次零位下实测完全闭合角度后，用 `--open-width-mm 85 --open-angle-rad 0 --closed-angle-rad 实测闭合角度` 生成新表。

## Tianji 串口固定名称

`udev/99-gripper-rs485.rules` 将当前 CH340 RS485 适配器固定为 `/dev/gripper_rs485`。该适配器没有唯一 USB 序列号，规则同时匹配 VID/PID `1a86:7523` 和当前物理 USB 路径 `platform-14180000.pcie-pci-0000:01:00.0-usb-0:2.1:1.0`。更换 USB 插口或集线器后必须重新核对 `udevadm info --query=property --name=/dev/ttyUSB0` 中的 `ID_PATH`。Modbus ID 1/2 在 USB 适配器后面，udev 无法据此区分夹爪；两只夹爪共用这个串口时仍分别用电机 ID 选设备。

在 Tianji 上安装（需要 sudo）：

```sh
cd ~/wxy/tianjizhineng/gripper_RS485
sudo install -m 0644 udev/99-gripper-rs485.rules /etc/udev/rules.d/99-gripper-rs485.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty --action=add
udevadm settle
readlink -f /dev/gripper_rs485
```

最后一条应解析到当前 RS485 适配器，例如 `/dev/ttyUSB0`。后续可用 `port:=/dev/gripper_rs485` 和 `PORT=/dev/gripper_rs485`。这个规则只创建稳定设备名，不影响电机零位或标定表。

2026-11-02 右夹爪 ID 2 重新确认全开 85 mm 并在全开处设零，完全闭合角度 56.6882591248 rad。当时 `right_calibration.csv` 已按 85 mm/0 mm 端点重新生成，中间开度仍是机构模型估算；先前 93 mm 表保存在 `right_calibration_93mm.csv`，不可用于当前零位/机构状态。

2026-11-06 右夹爪 ID 2 再次确认全开 86 mm，并在此处设零；全闭角度复读为 55.2002983093 rad。当前 `right_calibration.csv` 已重建为 86 mm/0 mm 两端标定，旧 85 mm 表保存在 `right_calibration_85mm.csv`。中间开度仍由机构模型估算，需要重新实测。该右侧驱动器曾在断电后于同一全开位置出现约五圈角度偏移，重新上电后须核对实际开度与编码器角度，不能只凭 CSV 存在就认为零位有效。

2026-11-06 右夹爪 ID 2 重新执行 `zero_demo 2` 后，现场确认机械全开实际宽度 **98 mm**。当时将 **63.5037384033 rad** 写入表作为 0 mm 端点；用户现确认这个全闭角度并未独立验证，因此它只是暂用估计，不能据此认定右侧 0 mm 指令安全或准确。此前 86 mm 表保存在 `right_calibration_86mm.csv`。驱动器断电后须重新确认零点。

随后右侧实测发现：原端点表在电机角度约 **26.60555 rad** 时反馈约 **60 mm**，接触面实测约 **45 mm**。第一次修正用该测量点执行 `refine_calibration.py --point 60:45`，原端点表保存在 `right_calibration_98mm_endpoints.csv`。

第一次修正后，电机角度 **19.7244053 rad** 时反馈 60 mm，现场实测 **56 mm**。第二次修正从端点原表重新生成，使用原端点模型宽度 **60.0065013→45 mm**、**70.7547302→56 mm** 两个实测锚点；一次修正表保存在 `right_calibration_98mm_one_anchor.csv`。

第二次修正后，右侧请求 60 mm，静止电机角度约 **18.009 rad**，现场实测 **59 mm 多**；本次精度核对约在 1 mm 内。其他中间位置尚未实测确认。

随后实测右侧在第二次修正表下的四个点：目标/实测 **20→13、30→25、50→50、70→66 mm**，静止电机角度分别约 **46.655、38.732、23.528、13.588 rad**。第三次右表从端点原表按这些点与先前的 45、56 mm 测量一次性重建，保持 0/98 mm 端点；第二次修正表保存在 `right_calibration_98mm_two_anchors.csv`。测量值取整至毫米，其他开度及往返重复性仍需复核。

2026-11-07 设备重新上电并执行 `zero_demo 2` 后，右侧机械全开再次确认 **98 mm**，但第三次表在本次零位下目标/实测为 **5→12、10→22、15→31、20→37、25→42、30→47、35→52、40→57、45→62、50→66、55→71、60→75、65→80、70→83、75→85、80→87 mm**。原表在电机角度 **29.69662 rad** 时反馈 40 mm、实测约 57 mm。当前 `right_calibration.csv` 用这些点从第三次表修正生成，第三次表保存在 `right_calibration_98mm_six_anchors.csv`。之后用户确认本次上电右侧在 61.209 rad 时物理全闭为 0 mm；现已将表内旧的 63.5037384033 rad 估计端点修正为 61.2092854977 rad，并保留已测的中间角度映射。0～12 mm 之间仍需进一步核对。下次上电后的适用性尚未验证。

此表部署后，本次上电的独立下发验证：**50 mm 指令实测 50 mm**（静止角度 33.94968 rad），**20 mm 指令实测 20 mm**（静止角度 51.61164 rad）；两次均在到位后失能。其余目标尚未用新表独立复测。
