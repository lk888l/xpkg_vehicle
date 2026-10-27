# MARK1_DIFF · ROS 2 Jazzy · SocketCAN

针对已接入 `can0` 的 MARK1_DIFF 重写的 ROS 2 工作空间。仅继承旧工程的 CAN 命令与字段定义，结合实测修正协议；不依赖 `xpkg_comm`、`xpkg_msgs`、ROS 1、串口桥、JSON 控制或旧里程学。

- 工作目录：`/home/kk/kk_data/ros2_project/vehicle_socketcan`
- Git 分支：`refactor/socketcan-jazzy`，由 `main@f90f687` 创建 worktree。
- 旧工程仍在 `../xpkg_vehicle` 的 `main` 分支。
- 实机结果：[验收报告](docs/validation.md)、[原始统计](artifacts/hardware_acceptance_2026-10-27.json)。低速精度尚未通过验收。

## 构建与启动

宿主机现有 `ros2_jazzy` 容器采用 host 网络，父目录映射到容器 `/workspace`。在本工程目录运行：

```bash
./tools/dev.sh build
./tools/dev.sh test
./tools/dev.sh run
```

或在容器中：

```bash
source /opt/ros/jazzy/setup.bash
cd /workspace/vehicle_socketcan
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
ros2 launch vehicle_driver vehicle.launch.py can_interface:=can0
```

启动后收到目标设备心跳，查询版本、使能一次；确认使能心跳后发送 CAN 模式及反馈配置一次。没有速度输入时不发送非零运动指令。退出时发送零速并默认失能。显式失能、切换模式后不会被后台循环改回；不自动复位、不自动清里程。

CAN 接口需由宿主机配置；本节点不修改位率、网卡状态或内核发送队列。当前 `can0` 已为 **500000 bit/s**，无需重复配置。新机器可按实际接口配置：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

自定义配置：

```bash
ros2 launch vehicle_driver vehicle.launch.py config:=/absolute/path/vehicle.yaml
# 取消启动时的自动使能/模式设置，使用服务自行操作：
ros2 launch vehicle_driver vehicle.launch.py auto_initialize:=false
# 节点也可以直接运行，或加载到 rclcpp_components 容器：
ros2 run vehicle_driver vehicle_node --ros-args -p can_interface:=can0
ros2 component load /ComponentManager vehicle_driver vehicle_driver::VehicleNode
```

组件加载示例使用默认 `can0`。同一设备应只有一个控制节点；原来的 `xpkg_comm`/底盘节点不要与本节点并行发送命令。

## ROS 接口

默认节点 `/vehicle`；所有名称均可 namespace/remap。速度使用 SI 单位。

| 接口 | 类型 | 说明 / QoS |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | 输入，reliable / volatile / depth 1，只使用 `linear.x`、`angular.z` |
| `/odom` | `nav_msgs/msg/Odometry` | reliable / depth 10，默认 B2 速度积分 |
| `/tf` | TF | 默认 `odom → base_link`，可关闭 |
| `/vehicle/velocity` | `geometry_msgs/msg/TwistStamped` | B2 原始速度换算，无平滑或死区 |
| `/vehicle/state` | `vehicle_interfaces/msg/State` | reliable / transient-local / depth 1，含在线状态、独立心跳/状态时间戳及有效位 |
| `/vehicle/version` | `vehicle_interfaces/msg/Version` | reliable / transient-local / depth 1，两段 ASCII |
| `/vehicle/wheel_mileage` | `vehicle_interfaces/msg/WheelMileage` | B3/B4，保留 int32 毫米累计值；`secondary` 区分副轮 |
| `/vehicle/driver_motion` | `vehicle_interfaces/msg/DriverMotion` | B7，channel 0–3、driver_id 1–4、RPM、脉冲 |
| `/vehicle/driver_status` | `vehicle_interfaces/msg/DriverStatus` | B8，电压、电流、温度及原始故障位 |
| `/vehicle/remote` | `vehicle_interfaces/msg/Remote` | B5，四个开关、六个**有符号**摇杆字段 |
| `/vehicle/sensors` | `vehicle_interfaces/msg/Sensors` | B6，保留原始值；当前实机未观察到 |
| `/vehicle/faults` | `vehicle_interfaces/msg/Faults` | BA，reliable / depth 10；布局尚未用硬件故障验证 |
| `/vehicle/acknowledgements` | `vehicle_interfaces/msg/Acknowledgement` | A3/A4/A5，reliable / depth 10 |
| `/vehicle/can_rx` | `vehicle_interfaces/msg/CanFrame` | 筛选后接收帧及总线错误帧，best-effort / depth 128；不含本 socket 自发帧 |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | reliable，2 Hz；帧计数、错误、丢帧、反馈年龄、命令状态 |
| `/vehicle/command` | `vehicle_interfaces/srv/Command` | 非阻塞设备命令事务 |
| `/vehicle/reset_odometry` | `std_srvs/srv/Trigger` | 只重置 ROS 位姿，不清设备里程 |

表中未单列 QoS 的遥测采用 `SensorDataQoS`（best-effort / volatile / depth 5）。订阅者需选择兼容 QoS。状态与版本的持久缓存便于后启动的订阅者读取，不能替代时间戳新鲜度判断。

发布速度：

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/TwistStamped \
  '{twist: {linear: {x: 0.1}, angular: {z: 0.0}}}'
```

零时间戳表示采用本地接收时间；非零时间戳会拒绝超过 `command_timeout` 的旧命令及超过 100 ms 的未来命令。需要兼容 `Twist` 的上层时，把 `use_stamped_cmd_vel` 设为 `false`，此时**同一 `/cmd_vel` 话题的类型改为 `geometry_msgs/msg/Twist`**。

命令最新值以 50 Hz 发送，输入停止后默认 300 ms 发出一次零速并结束重发；设备本身另有实测约 500 ms 的断命令保护。NaN/Inf 输入会清空旧指令并停速。线速度/角速度默认限幅为 1 m/s、2 rad/s，可在 YAML 中修改。差速底盘不执行横移，其他轴忽略并限频提示。

## 命令服务

```bash
# 版本：command=2
ros2 service call /vehicle/command vehicle_interfaces/srv/Command '{command: 2}'
# 使能/失能：command=3
ros2 service call /vehicle/command vehicle_interfaces/srv/Command '{command: 3, enabled: true}'
ros2 service call /vehicle/command vehicle_interfaces/srv/Command '{command: 3, enabled: false}'
# 模式：0 待机 / 1 遥控 / 2 CAN / 3 自由；beep 为蜂鸣器
ros2 service call /vehicle/command vehicle_interfaces/srv/Command '{command: 17, mode: 2, beep: false}'
# 清故障 / 清设备里程与驱动器脉冲
ros2 service call /vehicle/command vehicle_interfaces/srv/Command '{command: 4}'
ros2 service call /vehicle/command vehicle_interfaces/srv/Command '{command: 5}'
# B5、B6、B7、B8 开关分别是 command=21、22、23、24
ros2 service call /vehicle/command vehicle_interfaces/srv/Command '{command: 23, enabled: true}'
# 仅重置本地里程计
ros2 service call /vehicle/reset_odometry std_srvs/srv/Trigger '{}'
```

响应将三件事分开：

- `sent`：写入本地 SocketCAN 成功，不等于已上总线或执行成功。
- `acknowledged`：收到了匹配的 A2/A3/A4/A5。A3/A4/A5 的载荷为零。
- `state_confirmed`：查询版本已解析；使能有新的 B0 匹配；模式有新的 B1 mode/beep 匹配。**不表示刹车、特殊功能、清故障或清零的所有物理效果已确认。**

模式服务的 `brake`、`special` 字段可发送，但没有可靠的对应状态反馈。反馈开关没有 ACK，返回仅表示发送成功，需要观察话题变化。CAN ACK 没有事务编号，同一时刻只等待一个服务事务；忙时返回明确说明，不自动排队或重发。服务等待默认 1.2 秒，不阻塞 CAN、速度下发或超时停止。

复位 `command=1` 仅显式发送一次，返回不宣称成功；已知此前发送后曾失联，所以不进入初始化或验收脚本。恢复时先用 `ip -details -statistics link show can0` 检查，再按需要重建接口并重新查询版本。详细证据见[协议说明](docs/protocol.md)。

## 里程计与配置

参数集中于 [vehicle.yaml](src/vehicle_driver/config/vehicle.yaml)，启动后只读，改参数需重新启动。不会接受参数变更却继续使用旧值。

默认 `odom_source: velocity`：B2 的 vx/wz 使用真实单调时钟接收间隔和 SE(2) 圆弧积分；没有测量时不外推，超过 250 ms 间隔重建基线。该里程计仍受设备低速反馈抖动影响。它不是独立定位真值。

需要基于 B3 累计里程计算时，设置 `odom_source: wheel_mileage` 和经过校准的 `effective_track_width`（米）。四轮滑移转向的有效轮距不应直接当成物理轮距。当前默认 0 表示未校准，因此不替用户猜测。B3 路径处理 int32 回绕、断连、A5 清零、明显计数跳变。A5 后只重建计数基线，保留 ROS 位姿连续；重置位姿另用服务。

协方差参数是初始工程估计，需要依据实际标定/融合需求调整；未观测的自由度给大方差。接入已有定位融合节点时，可将 `publish_tf` 关闭以避免重复发布同一 TF。

## 测试与开发

```bash
# 容器内：自动创建独立 vcan_veh_test，退出时删除由脚本创建的接口
bash tools/test.sh
# ASan/UBSan：纯 C++ 库无需 ROS 运行时
bash tools/sanitizers.sh
# 实机验收，包含低速运动；启动并结束本脚本自己的驱动实例
python3 tools/hardware_acceptance.py --interface can0 --motion \
  --output artifacts/my_acceptance.json
```

`tools/test.sh` 需要容器具备 NET_ADMIN 和宿主机 vcan 支持；当前 `ros2_jazzy` 已具备。普通 `colcon test` 未设置 `VEHICLE_TEST_CAN` 时会明确跳过虚拟总线集成测试。注入测试会校验内核设备类型，拒绝在物理 `can0` 上执行。GitLab CI 在具有相同权限的 runner 上运行构建、全部测试及 sanitizer，不连接硬件。

`tools/dev.sh build` 以宿主用户 UID/GID 构建；容器 root 运行测试产生的日志可能为 root 所有。可通过 `VEHICLE_CONTAINER`、`VEHICLE_CONTAINER_PATH` 覆盖容器名和挂载路径。

目录及设计依据见[架构说明](docs/architecture.md)。协议与运行证据明确区分已实测字段、继承但未验证的字段、未知字段。
