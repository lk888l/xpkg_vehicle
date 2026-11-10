# IMU 航向、转向与路径记录

底盘继续使用 `can0`；HPM IMU 使用独立的 `can1` SocketCAN FD。协议依据本机 `/home/kk/kk_data/cpp_project/hpm-imu/docs/can.md` 及固件实现，默认 CANopen Node-ID 为 31。IMU 节点与底盘节点分别持有 socket，IMU 缺失不会阻止底盘启动或普通手动控制。

## 启动和配置

```bash
./tools/dev.sh build
./tools/dev.sh gui
# 已有底盘和 IMU 节点时，只打开 GUI：
./tools/dev.sh gui start_driver:=false
# 仅启动底盘，不启动 IMU：
./tools/dev.sh run start_imu:=false
```

驱动与 GUI 的 launch 默认同时启动 `/vehicle`、`/imu`。GUI 的 `start_imu` 默认跟随 `start_driver`，可以独立覆盖；`imu_can_interface:=can1` 可修改接口。没有 IMU 时底盘反馈仍用于里程计，方向按钮、W/A/S/D 和方向键仍可使用。

默认启用启动自动方向标定：底盘和 IMU 就绪后会自行原地正反转。若需关闭，在加载的 `vehicle.yaml` 中设置 `auto_imu_calibration: false`，此时使用手动配置的 `imu_yaw_sign`。`auto_initialize` 只控制启动使能/模式配置；若设备本来已使能且处于 CAN 模式，单独将它设为 false 不会取消方向标定。

宿主接口配置为仲裁段 **1 Mbit/s**、数据段 **4 Mbit/s**、FD 开启、两段采样点 80%。2026-11-06 已将本机 can1 从 1M/1M 修正为 1M/4M；这是运行时配置，系统重启后由宿主网络配置负责恢复。新机器可执行：

```bash
sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000 sample-point 0.8 \
  dbitrate 4000000 dsample-point 0.8 fd on
sudo ip link set can1 up
```

节点本身不修改网卡。收到指定节点的 boot-up 或 PRE-OP 心跳后，发送寻址 NMT Start `000#011F`，不广播、不复位、不写 PDO 映射或持久配置。若节点仍处于 PRE-OP，最多每秒再发一次 Start；已 OPERATIONAL 时只接收。`imu_auto_start: false` 可交给外部 CANopen 管理器启动。

参数位于 [vehicle.yaml](../src/vehicle_driver/config/vehicle.yaml)，启动后只读：

| 参数 | 默认 | 含义 |
|---|---|---|
| `imu_can_interface` | `can1` | IMU SocketCAN FD 接口 |
| `imu_node_id` | `31` | 1～127，固件保留的 100、101 除外 |
| `imu_auto_start` | `true` | 看到 boot-up / PRE-OP 后启动指定节点 |
| `use_imu_heading` | `true` | 底盘里程计使用有效的 IMU 航向增量 |
| `auto_imu_calibration` | `true` | 每次底盘驱动启动自动执行一次 yaw 方向标定 |
| `imu_yaw_sign` | `1` | 手动方向初值，限 +1 / -1；关闭自动标定时直接使用 |
| `imu_calibration_speed` | `0.30` | 原地转速，rad/s，受 max_angular_speed 限制 |
| `imu_calibration_angle` | `0.1745329252` | 正转目标约 10°；反转按实测正转角差回位，rad |
| `imu_calibration_min_angle` | `0.0523598776` | 每方向至少观测约 3° 才判断方向，rad |
| `imu_calibration_settle_time` | `0.30` | 开始前、两方向之间和最后的零速等待时间，秒 |
| `imu_calibration_phase_timeout` | `3.0` | 每个转动阶段最长时间，秒 |
| `imu_mount_rpy` | `[0.0, 0.0, 0.0]` | 传感器轴到 `base_link` 轴的 roll/pitch/yaw，单位 rad |
| `imu_orientation_variance` | `0.01` | 姿态对角协方差，rad²，工程初值 |
| `imu_gyro_variance` | `0.0004` | 角速度对角协方差，(rad/s)² |
| `imu_accel_variance` | `0.04` | 加速度对角协方差，(m/s²)² |

安装方向按本次约定先取 **X 向前、Y 向左、Z 向上**。后续标定修改 `imu_mount_rpy`；它是传感器向量旋转到车体坐标的旋转，而不是直接给世界航向加偏移。四元数按 `world_from_sensor × inverse(base_from_sensor)` 变换。仅处理安装旋转，未做加速度杆臂补偿。

## 启动自动方向标定

标定在 `/vehicle` 内运行，启动 GUI 或只启动驱动均会默认请求一次。驱动等待已使能的 CAN 模式、正常且新鲜的底盘状态，以及有效 IMU yaw；不新增确认、解锁或人工使能步骤。既有自动初始化仍负责使能和切换 CAN 模式。

流程：零速等待 0.3 秒 → `vx=0, wz=+0.30 rad/s` 原地正转约 10° → 零速等待 0.3 秒 → `wz=-0.30 rad/s` 按正转实测角差反转 → 零速等待并检查结果。每段转动最多 3 秒；超时后若变化不足 3° 则失败，若已经取得足够角差则结束该段继续验证。回位是近似回位，不承诺物理角度完全归零。

测量使用**安装变换之后、尚未乘方向系数的原始 yaw**，连续角差处理 ±180° 跨界。两段响应必须足够大且符号相反：正转时原始 yaw 增加，系数为 +1；正转时原始 yaw 减少，系数为 -1。以底盘 `angular.z` 命令正负为参考，仅判断 yaw 方向，不标定比例、陀螺零偏、安装倾角或磁北航向。

成功后将系数应用于底盘里程计的 IMU yaw，转向控制和路径随之使用修正后的航向。方向系数不会改写 `/imu/data` 的姿态、陀螺和加速度。重新建立融合基线时保留已有位姿，不清轮里程、不复位 IMU、不清路径。系数保存在本次驱动进程中，每次重新启动再次自动测定；只读参数 `imu_yaw_sign` 保持配置初值，当前实际系数读取状态话题。

标定期间使用底盘里程计。自动标定尚未成功时也使用底盘里程计和普通键盘速度控制，不用未确认的方向做航向闭环；若本进程已有成功结果，之后重标定失败会保留原系数。关闭自动标定时直接使用人工配置系数。

任意 `/cmd_vel`（包括零速、GUI 停止或失焦产生的零速）取消等待中或正在运行的标定并交还控制；失能、切换模式、清计数、重置里程计、CAN 断连及 IMU 失效也会终止活动标定。数据中断超过 250 ms 或双向结果不一致时停止。失败或取消后不自动重试、不自动恢复旧速度；普通方向键仍可使用。

`/vehicle/imu_calibration` 使用 `vehicle_interfaces/ImuCalibration`，reliable / transient-local / depth 1，10 Hz 及阶段切换时发布。状态依次为 DISABLED=0、WAITING=1、SETTLING=2、POSITIVE=3、PAUSE=4、NEGATIVE=5、VERIFYING=6、SUCCEEDED=7、FAILED=8、CANCELED=9。同时报告实际系数、是否有成功结果、正反转的原始角差和自动角速度。GUI 左侧显示阶段与系数，速度曲线包含自动标定指令，CSV 记录 `imu_yaw_sign` 和 `imu_calibration_state`。

需要开始或重新测定时，点击 GUI 左侧 IMU 状态下方的“开始 IMU 标定／重新标定 IMU”。按钮会结束当前手动或定角运动并直接请求标定；等待或运行期间禁用重复请求，完成、失败或取消后可再次点击。停止按钮、空格、Esc 和新的方向键输入随时接管；释放标定前已按住的方向键不会打断新标定。也可在停止人工速度输入后调用：

```bash
ros2 service call /vehicle/calibrate_imu std_srvs/srv/Trigger '{}'
ros2 topic echo /vehicle/imu_calibration --qos-durability transient_local
```

Trigger 响应只确认请求受理，完成或失败由状态话题报告。请求不会绕过正在发生的人工输入，也不会自动重新使能或切换用户指定的设备模式。已有硬件验收脚本显式关闭此启动运动，保持各自既定的测量流程。

## IMU 角度置零

点击 GUI 左侧标定按钮下方的 **“IMU 角度置零”**，将当前姿态设为 roll、pitch、yaw 的零点。按钮会结束当前手动输入、直行保向或定角转向；缺少有效姿态、数据过期或正在等待其他服务响应时禁用。结果区显示“置零完成”或具体失败原因，后续角度显示为相对此次零点的姿态。

也可以独立调用 IMU 节点的服务，命名空间随节点命名空间变化：

```bash
ros2 service call /imu/zero_orientation std_srvs/srv/Trigger '{}'
```

`success=true` 表示置零已经完成。服务要求最新有效姿态的总年龄小于 250 ms（含固件报告年龄）；无效、超时或重复计数停更时拒绝请求，保留原零点。服务本身只更新姿态参考，GUI 负责结束当前控制输入。

当前固件的 SFLP 四元数对象只读，没有 CAN 置零命令。本功能与固件自带调试 GUI 的“共同归零”采用同样的主机参考方式：先按安装旋转得到 `q_base`，记录当前 `q_zero = q_base`，随后 `/imu/data` 发布 `inverse(q_zero) × q_base`。三轴同时归零，连续点击以每次的实际姿态替换参考；陀螺、加速度、磁场和固件计数继续使用原测量值。这不是陀螺零偏标定，也不产生持久配置或 IMU 复位。

切换参考时先通过 reliable `/imu/data` 发布失效标记，再发布零点后的姿态，保留该样本的原时间戳。底盘据此重建 IMU 航向基线，保持已有 X/Y、路径航向和轨迹连续；需要把路径坐标也清零时使用“里程计清零”。活动中的方向标定会在失效标记处结束。其他使用姿态增量的订阅者也应处理失效标记。

零点仅保存在当前 IMU 节点内存中；节点重启、CAN 连接重建、非运行态心跳或检测到四元数计数重启后清除。单纯数据过期或暂时无效会保留零点，恢复有效数据后继续使用。

## 数据协议与 ROS 接口

仅接收指定 Node-ID 的标准帧和 CAN 错误帧。心跳为 Classic CAN `0x700 + node`；TPDO1 为 `0x180 + node`，TPDO2 为 `0x280 + node`，默认均为 32 字节 FD 帧，29 字节有效载荷、3 字节零填充，固件事件周期 8 ms。修改过 PDO 映射时需先恢复固件默认映射，接收端不会猜测新布局。

TPDO1 包含加速度、角速度、磁场、温度、样本计数及故障；TPDO2 包含 SFLP 四元数 w/x/y/z、重力、陀螺零偏、四元数计数、样本年龄及有效位。全部按小端解码，将 mg、0.1°/s、mGauss 分别转换为 m/s²、rad/s、T。四元数检查有效位、设备年龄小于 250 ms 和单位长度，再归一化。

| Topic | 类型 | QoS | 内容 |
|---|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | reliable / volatile / 10 | 安装旋转后相对于当前零点的片上姿态，附带最近一次有效加速度、角速度；默认零点为单位四元数 |
| `/imu/raw` | `sensor_msgs/Imu` | best-effort / volatile / 5 | TPDO1 加速度和角速度，姿态 covariance[0]=-1 |
| `/imu/magnetic_field` | `sensor_msgs/MagneticField` | best-effort / volatile / 5 | 有效磁场，单位 T |
| `/imu/temperature` | `sensor_msgs/Temperature` | best-effort / volatile / 5 | 有效温度，单位 °C |
| `/imu/status` | `vehicle_interfaces/ImuStatus` | reliable / volatile / 10 | 10 Hz 在线状态、样本有效性/年龄/计数、NMT、故障及接收统计 |
| `/vehicle/heading` | `vehicle_interfaces/Heading` | reliable / volatile / 10 | 与 `/odom` 同次发布的航向、角速度、IMU 是否参与、IMU 年龄 |
| `/vehicle/imu_calibration` | `vehicle_interfaces/ImuCalibration` | reliable / transient-local / 1 | 自动标定阶段、实测角差和方向系数 |

所有 IMU 向量和姿态按 `base_frame` 表达，默认 `base_link`。`/vehicle/heading` 则使用 `odom_frame`。`imu/data` 时间戳为用户态接收时间减去固件报告的四元数年龄；它不包含芯片融合延迟补偿。TPDO1 与 TPDO2 独立采样，组合消息中的惯性量不承诺与四元数同步，精确分析应保留 `/imu/raw` 的独立时间戳。未提供的数据将对应 covariance[0] 设为 -1。

重复计数不会刷新姿态时间或重复发布有效四元数。无效姿态、MCU 重启和连接中断会发布失效标记；可靠、有序的 `imu/data` 传输使底盘先清除旧航向基线，再接受新姿态。磁场故障不使独立的六轴 SFLP 姿态失效。

`ImuStatus` 的 `online` 来自最近 1.5 秒心跳，`measurement_valid`、`orientation_valid` 分别依据各自样本判断，不能只用在线状态推断数据有效。`Heading.imu_active` 表示这次里程计使用的航向来源，`imu_age=-1` 表示没有航向样本。

## 里程计与键盘配合

线位移仍来自 B2 速度或 B3 轮里程，转向增量在 IMU 有效时改用相邻四元数航向差，并处理 ±π 跨界；位置继续使用 SE(2) 圆弧积分。因此原地转向改变航向，直行沿新航向延伸，组合 W+A / W+D 会记录弧线。不对加速度二次积分，不把六轴相对 yaw 当作磁北绝对航向。

首个有效 IMU 样本会与当前里程计对齐。超过 250 ms 未得到新姿态时自动用底盘角速度/轮差积分；恢复后重新对齐，保留位置和航向连续。里程计清零同时重新建立本地参考，设备计数清零只重建积分基线。

控制台提供：

- **相对转角**：输入 -360°～360°，正值向左、负值向右；包括完整一圈的定向转动。
- **路径航向**：以当前里程计零点为 0°，按最短角差转到指定航向，和路径图使用同一坐标系。
- **执行角度转向**：点击后由驱动原地转动，旋转速度框作为 CAN 角速度指令上限。随误差减速，默认航向误差不超过 1°、测量线速度低于 0.03 m/s、角速度低于 0.05 rad/s 且连续保持 200 ms 后完成。这是软件完成条件，实车精度需标定验证。
- **定距离前进 / 执行前进**：输入 0.01～100 m，锁定执行瞬间的航向，用 IMU 纠偏。距离按里程计位移在起始航向上的投影计算，不用指令速度乘时间；侧向位移不计作前进距离。默认距离误差 10 mm，距离和航向均到位、测量速度满足上述条件并保持 200 ms 后结束。超调时允许低速回调。
- **展开转向与行驶参数**：设置轮距、轴距、滑移前馈系数、航向和角速度反馈增益、角度/距离误差、距离增益和执行超时。上方速度框同时作为自动运动的速度上限。自动任务提交时快照参数，编辑不会改变正在运行的任务；键盘保向使用界面当前参数。GUI 参数保存在本次窗口内，重开恢复界面初值；无 GUI 的服务默认使用 YAML。
- **直行保向**：默认开启。按 W/S 时锁定开始直行的航向并持续纠偏；按 A/D 时自由转弯，松开 A/D 而继续按 W/S 时锁定新的航向。倒车也使用相同的航向误差方向。
- 方向键或方向按钮可直接接管定角转向；空格、Esc、停止按钮和原有窗口失焦/输入续期逻辑均可结束。IMU 过期时结束当前定角转向，普通键盘速度控制继续可用；不增加解锁、使能确认或额外启动门槛。

定角/定距控制由驱动中的 [MotionController](../src/vehicle_core/src/motion_controller.cpp) 执行；GUI 使用 `/vehicle/execute_motion` 提交目标，订阅 `/vehicle/motion_status`。CAN 仍是已有的 `0x12` 车体 vx/wz 命令，不新增轮级协议。键盘保向仍在 GUI 计算并发布 `/cmd_vel`；其他速度发布者不受该开关影响。橙线显示当前航向、绿虚线显示目标航向，侧边显示剩余角度或距离、前馈倍率和结束原因。

## 四个固定动力轮的前馈与闭环

本车是四轮滑移差速底盘，默认左右轮距 **B=0.535 m**、前后轴距 **L=0.420 m**。不使用阿克曼转角公式。理想左右轮线速度为 `v_left = vx - B*wz/2`、`v_right = vx + B*wz/2`；轴距不进入理想两侧差速关系，实际四轮转弯会产生侧向滑移阻力。

实现使用可校准的对称线性滑移阻力近似：`G = 1 + k*(L/B)^2`。该形式可由四个接地点的纵向/横向线性阻力矩平衡得到，`k` 表示横向与纵向阻力比的工程调节量；它不等同于已实测轮胎摩擦系数，也不能由尺寸唯一确定。默认 `k=0.5` 是待实车调试的初值，得到 `G≈1.308`；`k=0` 恢复理想差速前馈。滑移模型需依赖地面与车辆参数的背景可参考 [ICRA 2019 摩擦运动学研究](https://amrl.cs.utexas.edu/papers/icra2019_skid_steer_kinematics.pdf)；本工程采用的是上述简化可调模型，并未复现该论文求解器。

控制步骤为 `w_ref = clamp(K_heading * heading_error)`，再计算 `w_cmd = clamp(G*w_ref + K_rate*(w_ref-w_measured))`。航向误差和测量角速度来自使用 IMU 航向增量的里程计；最终角速度受 GUI 上限及驱动 `max_angular_speed` 双重限制。进入角度误差范围后请求零角速度，等待实际停稳。直行请求 `vx = clamp(K_distance * remaining)`，并随航向偏差减小线速度。

底盘固件接收车体角速度，并在内部映射左右轮。这里的前馈是在该接口上进行补偿；实际倍率还包含固件轮距设置和内部控制效果，需在实车上校准。模型中的 `B*G` 不自动写入里程计的 `effective_track_width`，两者用途不同。B2/B3 仍受轮滑与低速反馈抖动影响，10 mm / 1° 是软件到位阈值，并非实测精度承诺。

驱动 YAML 的 `motion_track_width`、`motion_wheelbase` 使用米，`motion_angle_tolerance` 使用 rad；其他 `motion_*` 参数名称与界面对应。默认航向增益 1.8、角速度反馈增益 0.2、距离增益 0.8、线速度 0.10 m/s、角速度 0.20 rad/s、超时 120 s。配置在启动时读取。

## 定角 / 定距 ROS 命令

无需打开 GUI，即可使用默认参数：

```bash
# 相对左转 90°；右转用负角度
ros2 service call /vehicle/execute_motion vehicle_interfaces/srv/ExecuteMotion \
  '{mode: 1, target: 1.5707963267948966}'
# 转到路径坐标系 0°（最短方向）
ros2 service call /vehicle/execute_motion vehicle_interfaces/srv/ExecuteMotion \
  '{mode: 2, target: 0.0}'
# 按起始航向前进 1 m；负数表示倒车
ros2 service call /vehicle/execute_motion vehicle_interfaces/srv/ExecuteMotion \
  '{mode: 3, target: 1.0}'
# 取消；或发布任意新的 cmd_vel 接管
ros2 service call /vehicle/execute_motion vehicle_interfaces/srv/ExecuteMotion '{mode: 0}'
ros2 topic echo /vehicle/motion_status --qos-durability transient_local
```

服务 [`ExecuteMotion.srv`](../src/vehicle_interfaces/srv/ExecuteMotion.srv) 的 `accepted` 只表示受理，`goal_id` 用于匹配 [`MotionStatus.msg`](../src/vehicle_interfaces/msg/MotionStatus.msg)。状态为 IDLE=0、RUNNING=1、SUCCEEDED=2、CANCELED=3、FAILED=4；目标/已完成/剩余的单位是 rad 或 m，路径航向请求会转成相对最短角差。状态以 10 Hz 和状态切换时发布，reliable / transient-local / 1。

`use_custom_config=false` 使用驱动 YAML；设为 true 时，必须填齐服务中的全部参数，角度单位 rad、距离单位 m。NaN/Inf、非法尺寸、增益、速度、容差和超时被拒绝。角度目标支持 ±10 圈，距离支持 ±100 m；GUI 提供 ±360°、前进 0.01～100 m。启动要求底盘已使能、正常 CAN 模式、有新鲜 IMU/里程计，且未在标定。接口不自行使能或切换模式。新合法目标替换当前目标，非法目标不替换它。

任意 `/cmd_vel`（含零速或被拒绝的旧时间戳）、取消命令、失能、模式改变、里程计清零、设备清计数、IMU 标定、CAN/反馈中断或执行超时会结束任务并请求零速，不自动恢复。GUI 的失焦、空格、Esc、停止和关闭沿用该停止路径。

CLI 默认 `client_timeout=0`，任务在驱动自主执行到完成、取消或超时。GUI 使用 0.35 s 续期超时：界面每 50 ms 续期本地意图，工作线程只在意图小于 200 ms 时通过 `/vehicle/motion_keepalive`（`std_msgs/UInt64`）发送当前 `goal_id`。旧目标 ID 无效。界面冻结时工作线程发零速；整个 GUI 进程退出或失去调度时驱动独立检测续期超时。恢复界面不会恢复旧任务。

## 路径记录

“导出当前路径”导出显示缓冲，最多 60000 点、约 10 Hz；“开始记录路径”选择 CSV 后，将每个收到的里程计样本流式写入文件，不受显示缓冲上限影响，每秒 flush，结束记录或退出 GUI 时关闭文件。清空显示轨迹不会删除正在记录的文件；里程计清零后 `segment` 加一，供离线区分坐标参考。文件写入错误在记录栏显示。

CSV 保存：接收单调时间、ROS 时间戳、segment、X/Y/yaw、测量速度、IMU 是否参与及年龄、IMU 发布的相对 yaw（含角度置零参考）、目标航向、控制模式、指令速度、yaw 方向系数和标定状态。角度单位为 rad；`frame`/`child` 保存坐标系。跨话题使用最近已收到的样本，另存 IMU/heading 接收时间及 heading 的 ROS 时间戳，便于识别不同步的数据，不能把它们当作硬件同步采样。

## 验证

`./tools/dev.sh test` 覆盖固件字节布局、单位/符号、安装旋转、±π 跨界、计数回绕/重复/重启、里程计回退与恢复、真实 CAN-FD vcan + DDS、Qt 定角转向、键盘弧线/直行保向和 CSV 连续记录。`tools/sanitizers.sh` 用 ASan/UBSan 检查纯 C++ 库。测试注入仅使用独立 vcan。

2026-11-06 角度置零回归：四个包构建通过，79 个实际用例通过（colcon 汇总 81 项，包含两个 CTest 外层入口），0 错误、0 失败、0 跳过。新增覆盖三轴与安装旋转后的置零、重复置零、原样本时间戳、无效/过期数据拒绝、设备重启清除参考、里程计连续性，以及真实 GUI 按钮调用、停止手动/定角输入和服务不可用提示；使用 `vcan_imu_zero`，未进行物理 IMU 置零验证。

2026-11-06 本机 can1 五秒检查未收到节点 31 的心跳或 PDO（RX=0），见 [接口检查记录](../artifacts/imu_probe_2026-11-06.json)。因此软件链路经过虚拟总线验证，尚未完成 IMU 实板和实车转向精度验证。可只启动 IMU，检查供电、CAN_H/CAN_L、终端电阻、固件是否运行以及实际 Node-ID：

```bash
ros2 run vehicle_driver imu_node --ros-args -p imu_can_interface:=can1 -p imu_node_id:=31
ros2 topic echo /imu/status
ros2 topic echo /imu/data --once
```
