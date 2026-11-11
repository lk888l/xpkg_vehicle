# MARK1_DIFF · ROS1 Noetic · SocketCAN

本目录仅作 ROS1 Noetic 实机部署源码存档，主工程优先使用上级目录的 ROS2 版本。2026-11-11 从 reComputer Mini J3011 同步，保留底盘 CAN 协议、IMU Classic CAN / CAN-FD 协议、全部遥测字段、里程计、TF、方向标定、定角/定距运动和 PyQt5 控制台。`COLCON_IGNORE` 隔离默认 ROS2 构建；核心源码、头文件和测试均为独立文件，不再通过软链接复用 ROS2 文件。同步范围、校验清单与验证结果见[存档同步记录](docs/archive-sync.md)。

已部署到 `orin@172.18.7.207:/home/orin/vehicle_noetic_ws`。默认底盘接口 `can0`，IMU 使用共享 `can0` 上的 Classic CAN 四 PDO 协议、节点号 31；独立 CAN-FD IMU 可显式选择接口和协议。实际验证情况见 [Orin 验证记录](docs/validation.md)。

## 使能前选择有无 IMU

统一入口是 `use_imu:=true|false`，驱动启动时固定，改变模式需先失能、结束该 launch，再以新参数启动。参数默认 `true`，保留已有有 IMU 行为。没有运行时模式切换服务，`rosparam set` 不会修改已运行驱动的策略。

```bash
cd /home/orin/vehicle_noetic_ws
# 当前仅连接 can0 底盘、未连接 IMU，默认允许实际控制：
bash tools/run.sh gui use_imu:=false
# 无界面运行，仍由 /vehicle/command 显式使能和设置模式：
bash tools/run.sh driver use_imu:=false
# 接入 IMU 后选择：
bash tools/run.sh gui use_imu:=true
# 只观察、不发送底盘命令时显式选择：
bash tools/run.sh gui use_imu:=false receive_only:=true
```

GUI 启动后点击“使能”，选择“CAN 控制”并点击“应用模式与蜂鸣器”，再按住方向按钮或 W/A/S/D 控制。默认 `receive_only=false` 已开放命令发送，不需要额外参数；`auto_initialize=false` 表示使能和模式仍由操作者明确下达。已有进程必须退出重启才能使用新默认值，`start_driver:=false` 只附着已有驱动，不会更改其配置。

| 行为 | `use_imu:=true` | `use_imu:=false` |
|---|---|---|
| IMU 节点 / 航向订阅 | 默认启动并订阅 | 不启动、不订阅，忽略其他节点发布的 IMU |
| 自动定角、定距、键盘保向 | 新鲜底盘里程计与 IMU 航向 | 新鲜底盘里程计航向 |
| IMU 标定 / 姿态置零 | 保留 | 不发起自动标定；标定服务拒绝，界面两个按钮禁用 |
| 反馈超时 | 自动运动停止，IMU 恢复后不自动续跑 | 自动运动停止，底盘反馈恢复后不自动续跑 |
| 普通手动控制 | 保留 | 保留 |

无 IMU 时默认依据 B2 实测速度与角速度积分，支持相同的定角、定距 API，但轮滑、反馈偏差会积累成位置和航向误差，不代表与 IMU 相同的实车精度。`odom_source=wheel_mileage` 仍需校准 `effective_track_width`。

2026-11-09 实机测试发现：发送全零速度时，B2 仍报告较大速度波动，因此本次没有执行实车定角/定距目标，结束已确认失能。应先排查底盘实际轮动 / 编码器反馈，再验收运动精度；详细数值和原始记录见 [可选 IMU 验证结果](docs/optional-imu.md#2026-11-09-orin-验证结果)。

`start_imu` 只决定是否由本 launch 启动传感器进程：已有外部 IMU 时用 `use_imu:=true start_imu:=false`；单独设置 `start_imu:=false` **不代表**无 IMU 模式。`use_imu:=false` 始终禁止此 launch 启动 IMU。GUI 读取驱动发布的 latched `/vehicle/imu_enabled`，附着已有驱动时不会用 GUI 参数覆盖驱动配置。直接 `rosrun vehicle_driver vehicle_node` 时对应私有参数为 `_use_imu_heading:=false`；launch 中以 `use_imu` 覆盖 YAML 的该字段。

无 IMU 实机只读检查：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 tools/hardware_smoke.py --use-imu false --can-interface can0 \
  --output artifacts/no_imu_hardware.json
```

设计和验证说明见 [可选 IMU 架构](docs/optional-imu.md)。

## 启动方式

所有启动入口、YAML 和直接运行的底盘节点都默认 `receive_only=false`，可发送使能、模式与速度命令。需要只接收观察时设置 `receive_only:=true`；该模式禁止底盘全部应用层 CAN 发送，包括退出时的零速/失能帧，服务明确返回拒绝原因。IMU 的寻址 NMT Start 与姿态置零保持可用，不驱动底盘。

在 Orin 上：

```bash
cd /home/orin/vehicle_noetic_ws
bash tools/build.sh
bash tools/run.sh driver                 # 底盘控制 + IMU
bash tools/run.sh imu                    # 单独测试 IMU
bash tools/run.sh gui                    # 在图形桌面中打开可实际控制的完整控制台
```

需要 install 空间时运行 `bash tools/build.sh install`，随后加载 `install/setup.bash` 使用标准 `roslaunch` 命令；该构建与入口也已在 Orin 验证。

等价的 ROS1 命令：

```bash
source /opt/ros/noetic/setup.bash
source /home/orin/vehicle_noetic_ws/devel/setup.bash
roslaunch vehicle_driver vehicle.launch
roslaunch vehicle_driver imu.launch              # 共享 can0，Classic CAN
# 独立旧版 CAN-FD IMU：
# roslaunch vehicle_driver imu.launch imu_can_interface:=can1 imu_protocol:=legacy_fd
roslaunch vehicle_gui console.launch start_driver:=false
rostopic echo /imu/status
rostopic hz /imu/data
rosservice call /imu/zero_orientation '{}'
```

`start_driver:=false` 时 GUI 的 `start_imu` 默认也为 false，可独立覆盖。关闭 GUI 会结束该 launch 管理的节点。单独运行 GUI 不会终止外部已有驱动。

CAN 网络配置由独立脚本显式执行，节点本身不改变接口和位率：

```bash
sudo bash tools/configure_can.sh shared can0          # 底盘与 IMU 共用 500 kbit/s Classic CAN
# 仅独立旧版 CAN-FD IMU 需要：
# sudo bash tools/configure_can.sh legacy-imu-fd can1  # 1 Mbit/s 仲裁、4 Mbit/s 数据、FD
python3 tools/can_probe.py can0 --seconds 5     # 完全只接收，不查询、不使能
# python3 tools/can_probe.py can1 --seconds 5   # 使用独立接口时
```

加载 Noetic 与 `devel/setup.bash` 后，可运行 `python3 tools/hardware_smoke.py --output artifacts/hardware.json`，自动启动隔离 ROS master 与只接收节点，记录实际 IMU 状态、姿态频率、置零结果及底盘启动至退出的发送计数。脚本不提供实车运动选项；`imu_orientation_valid=false` 表示尚未验证实物 IMU，不能当作通过。

默认底盘与 Classic CAN IMU 共享 `can0` 的 500 kbit/s 网络。只有旧版 CAN-FD IMU 使用独立总线，不能把共享底盘网络改成 CAN-FD 位率。CAN 接口配置为运行时状态，重启后需由系统网络配置或上述脚本恢复。Seeed Mini 的专用网络自启动安装入口是 `tools/install_shared_can_service.sh`，只配置收发器、引脚和 CAN 网络，不启动 ROS 或使能底盘；本次同步未执行该脚本。

## 对照原工程

| 功能 | Noetic 实现 |
|---|---|
| CAN 编解码、身份过滤、故障和丢帧统计、重连 | 原协议与算法；SocketCAN 使用 Ubuntu 20.04 兼容的 `can_dlc` 字段 |
| 最新速度、50 Hz 下发、300 ms 超时、NaN/Inf 和旧时间戳拒绝 | 保留原 `CommandBuffer` 和校验逻辑 |
| 版本、使能、模式、反馈开关、清故障、清设备计数、显式复位 | 保留全部服务字段和发送/应答/状态确认的区分 |
| B2/B3 里程计、计数回绕、SE(2) 积分、IMU 航向回退、TF | 复用原 `Odometry`；默认 `odom → base_link` |
| HPM IMU、安装变换、量纲、计数新鲜度、姿态置零 | 独立 `imu_node`；姿态置零不发 IMU 复位或底盘命令 |
| 双向 yaw 方向标定 | 复用原 `YawCalibration`，等待/停止/失败行为保留 |
| 定角、路径航向、定距、滑移前馈和目标续期 | 复用原 `MotionController`，保留全部服务参数与状态 |
| 控制台、键盘保向、曲线、四路驱动反馈、原始帧、CSV 路径 | 原 PyQt5 界面与模型，通信层改为 `rospy`，兼容 Python 3.8 |
| 状态、版本、标定与运动结果缓存 | ROS1 latched publisher，对应原 transient-local depth 1 |
| ROS2 组件加载 | 使用 ROS1 独立进程；不需要 DDS 或 ROS2 运行环境 |

2026-11-10 起默认允许底盘发送，`auto_initialize=false`、`auto_imu_calibration=false` 保留。需要启动后自动使能并设置 CAN 模式时可显式设置 `auto_initialize:=true`；IMU 双向旋转标定是另一个独立选项 `auto_imu_calibration:=true`，普通实际控制不要求启用它。

## ROS1 接口

默认节点 `/vehicle`、`/imu`；支持 `namespace` 和标准 ROS1 remap。速度与几何量沿用 SI 单位。所有消息均通过 TCPROS 传输；没有 ROS2 QoS 参数。

| 话题或服务 | ROS1 类型 | 含义 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/TwistStamped` | 输入，队列 1；`use_stamped_cmd_vel=false` 时改为 `geometry_msgs/Twist` |
| `/odom`、`/tf` | `nav_msgs/Odometry`、TF | 里程计和可关闭的坐标变换 |
| `/vehicle/velocity` | `geometry_msgs/TwistStamped` | B2 原始速度 |
| `/vehicle/state`、`/vehicle/version` | `vehicle_interfaces/State`、`Version` | latched；State 的两个时间字段改为 ROS1 原生 `time` |
| `/vehicle/wheel_mileage` | `vehicle_interfaces/WheelMileage` | 主/副轮 int32 毫米累计值 |
| `/vehicle/driver_motion`、`/vehicle/driver_status` | `vehicle_interfaces/DriverMotion`、`DriverStatus` | 四路转速、脉冲、电气和温度反馈 |
| `/vehicle/remote`、`/vehicle/sensors`、`/vehicle/faults` | `vehicle_interfaces/Remote`、`Sensors`、`Faults` | 保留原始字段和原有未验证范围 |
| `/vehicle/acknowledgements`、`/vehicle/can_rx` | `vehicle_interfaces/Acknowledgement`、`CanFrame` | 设备 ACK 和筛选后的接收帧 |
| `/vehicle/heading` | `vehicle_interfaces/Heading` | 与里程计同步的航向和来源 |
| `/vehicle/imu_enabled` | `std_msgs/Bool` | latched，实际生效的固定 IMU 运行模式；false 表示底盘航向策略 |
| `/vehicle/imu_calibration`、`/vehicle/motion_status` | `vehicle_interfaces/ImuCalibration`、`MotionStatus` | latched，约 10 Hz 及状态切换时发布 |
| `/vehicle/motion_keepalive` | `std_msgs/UInt64` | 自动运动目标 ID 续期 |
| `/imu/data`、`/imu/raw` | `sensor_msgs/Imu` | 姿态及原始惯性量，缺测协方差首项为 -1 |
| `/imu/magnetic_field`、`/imu/temperature` | `sensor_msgs/MagneticField`、`Temperature` | 磁场、温度 |
| `/imu/status` | `vehicle_interfaces/ImuStatus` | 10 Hz 心跳、有效性、计数、错误和数据年龄 |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 2 Hz；包含 `receive_only`、`tx_frames` 和阻止发送的计数 |
| `/vehicle/command` | `vehicle_interfaces/Command` | 全部原底盘命令；只接收模式明确拒绝 |
| `/vehicle/execute_motion` | `vehicle_interfaces/ExecuteMotion` | 定角/定距或取消；结果见 motion_status |
| `/vehicle/calibrate_imu` | `std_srvs/Trigger` | 会转动底盘的方向标定，只接收模式拒绝 |
| `/vehicle/reset_odometry` | `std_srvs/Trigger` | 只重置 ROS 位姿，只接收模式不发零速帧 |
| `/imu/zero_orientation` | `std_srvs/Trigger` | 软件姿态参考置零，要求新鲜有效姿态 |

参数集中在 [vehicle.yaml](src/vehicle_driver/config/vehicle.yaml)，名称、单位和算法初值保持一致。ROS1 参数服务器上的值修改后不会动态应用，需重启节点。launch 对 `receive_only`、`auto_initialize`、`auto_imu_calibration` 的默认覆盖优先于 YAML；改变运行模式时使用显式 launch 参数。

ROS1 底盘命令服务使用独立回调队列等待应答，等待期间释放会话互斥锁，主队列继续接收 CAN、处理速度和执行看门狗。并发命令事务返回忙，不排队重发。时间限制和积分间隔沿用 `steady_clock`，定时器使用 wall time，ROS 时间只用于时间戳。

## 构建、部署和测试

系统依赖包括 Noetic 的 `roscpp`、`rospy`、标准消息、`tf2_ros`、`message_generation`，以及 `python3-pyqt5`、`python3-pyqtgraph`、`python3-numpy`、`python3-pytest` 和 gtest。Orin 已完成安装。安装系统依赖时，ARM64 机器应使用 Ubuntu Ports 源。

本存档可脱离原 ROS2 工程独立构建。移到另一台机器时使用下列导出流程，导出的工作区包含所有实际文件：

```bash
# 在本工程 ros1 目录中
bash tools/export_source.sh /tmp/vehicle_noetic_source.tgz
scp /tmp/vehicle_noetic_source.tgz orin@172.18.7.207:/tmp/
# 在 Orin 上
mkdir -p ~/vehicle_noetic_ws
tar -xzf /tmp/vehicle_noetic_source.tgz -C ~/vehicle_noetic_ws
cd ~/vehicle_noetic_ws
bash tools/build.sh
```

完整测试包含原 C++ 核心用例、ROS1 驱动/IMU 集成、PyQt 键盘与服务交互、路径 CSV、运动续期、三个默认控制启动入口及显式只接收模式。只在隔离 `vcan_noetic` 上模拟运动：

```bash
# 创建/删除 vcan、测试网络断开恢复需要 NET_ADMIN；Orin 可使用 sudo。
sudo bash tools/test.sh
```

测试脚本启动独立的 `127.0.0.1:11329` ROS master，验证接口内核类型为 `vcan`，结束时清理自己创建的接口与 master。可用 `VEHICLE_TEST_CAN`、`VEHICLE_TEST_MASTER_URI` 覆盖；指定物理 CAN 会被拒绝。报告写入 `build/test_results` 和 `test_results/pytest.xml`。单纯运行 `pytest` 而未设置虚拟接口时，集成测试明确跳过。

只读硬件检查使用 `tools/hardware_smoke.py`；显式实机控制验证使用 `tools/hardware_bench.py`，历史结果见[可选 IMU 验证结果](docs/optional-imu.md)。本次存档同步仅在隔离虚拟 CAN 上复测，没有运行实机控制验证。
