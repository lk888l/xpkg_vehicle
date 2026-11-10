# 可选 IMU：启动时选择航向策略

## 决策

两种配置共用协议解析、里程计和 `MotionController`。`HeadingPolicy` 是值类型的轻量策略模式：只封装所需反馈来源、就绪条件和拒绝原因；驱动在构造时注入配置，保存为 `const`，整个进程生命周期不变。当前只有两种稳定策略，无需虚函数继承层次、插件系统或运行时状态机。

- `vehicle.launch` 是唯一驱动组合入口，GUI 通过 include 复用，避免两套参数分叉。`use_imu` 同时决定 IMU 进程是否启动及底盘是否使用 IMU 航向，非法布尔值在启动节点前被拒绝。
- `Odometry` 提供统一位姿；无 IMU 时已有的 B2 积分或经校准的 B3 轮里程计直接提供航向。
- `MotionController` 只消费位姿和反馈就绪条件，不区分传感器类型。
- `/vehicle/imu_enabled`（`std_msgs/Bool`，latched）公布实际生效的固定策略。GUI 即使后连接也能获知；不通过猜测“有没有收到 IMU”选择模式。
- `/diagnostics` 新增 `imu_enabled`、`motion_feedback_ready`。`Heading.imu_active` 仍仅表示实际使用新鲜 IMU，不能用它代替所选运行模式。

有 IMU 配置在 IMU 丢失后，里程计保留原有底盘积分回退，自动运动仍停止并要求重新下达目标。无 IMU 配置从不订阅 IMU，也不因 IMU 恢复而切换航向来源。两者都要求使能、CAN 模式、正常工作状态和新鲜底盘里程计；保留反馈超时、命令超时和 GUI 目标续期保护。

## 操作契约

先失能并退出本驱动，使用 `use_imu:=false` 或 `use_imu:=true` 重新启动，再使能。没有运行时修改接口。ROS1 参数服务器只是启动输入，启动后的 `rosparam set` 不生效，GUI 持续以驱动发布的生效值为准。

2026-11-10 起统一默认 `receive_only=false`，允许实际控制；只观察时显式指定 `receive_only:=true`。`auto_initialize=false`、`auto_imu_calibration=false` 保留，使能和模式由操作者明确下达。无 IMU 模式即使残留 `auto_imu_calibration=true` 也不会请求方向标定。`start_imu=false` 用于外部 IMU 驱动，只有 `use_imu=false` 才选择无 IMU 策略。

无 IMU 的定角和定距精度取决于底盘速度 / 轮里程计精度及轮滑；应按实际地面测量后再调整容差。底盘失能时当前设备只发 B0 心跳，尚无 B1/B2 时界面能正确显示“无 IMU 模式”，但不会凭空产生可用航向或接受自动运动。

## 验证

`sudo bash tools/test.sh` 在隔离 vcan 与独立 ROS master 上运行：

- 两个 launch 入口、两种模式、是否自行启动 IMU 的组合及 GUI 单独附着。
- 无 IMU 的相对转角、绝对航向、定距完成；外部 IMU 被忽略，修改参数不切换策略。
- 底盘里程计过期、失能、模式改变、故障和目标续期超时均停止；恢复后不自行续跑。
- GUI 显示模式、禁用 IMU 专属操作、通过底盘航向执行定角/定距/键盘保向。
- 原有 IMU CAN-FD / Classic、标定、失效行为、只接收零发送和 GUI 回归测试。

`tools/hardware_smoke.py --use-imu false` 在实机执行只读启动/退出检查，不要求存在 IMU 服务。具体测试结果和实机证据见 `artifacts/optional_imu_*`。

## 2026-11-09 Orin 验证结果

- 原生 ARM64 / Noetic 编译及 `install` 更新通过。
- 完整回归：34 个独立 C++ gtest、90 个 Python/ROS/GUI 用例通过，零失败、零跳过；catkin 汇总显示 68 是同一核心结果被重复计数，以 gtest XML 的 34 为准。PyQtGraph 有既有的 Python 3.8 弃用警告。
- 无 IMU 只读实机检查：can0 底盘在线且失能，GUI 正常，`imu_enabled=false`，没有 IMU 节点；启动、关闭 GUI、退出全过程 CAN TX 增量为 0。
- 静态协议检查：真实设备 `001v / 001v` 的查询、使能、CAN 模式设置和最终失能均获得协议 / 状态确认；IMU 禁用时 `motion_feedback_ready=1`，说明无 IMU 路径能接收并使用实际底盘反馈。
- 实车自动运动未通过验收：持续发送全零速度的采样期间，B2 报告的速度峰值为 `|vx|=0.524 m/s`、`|wz|=1.264 rad/s`。监测到的 161 个本机速度帧全部为零。测试因反馈超过静态阈值而中止，没有提交定角或定距目标，随后确认底盘失能。软件记录不能区分真实轮动与底盘反馈错误，需要检查底盘驱动器 / 编码器及反馈后再做实车运动精度验证；不对反馈增加掩盖问题的死区。
- 有 IMU 路径通过虚拟 CAN 的 Classic / FD、标定及失效回归；本次未连接实物 IMU，未宣称通过实物 IMU 验证。

原始记录：`artifacts/optional_imu_hardware_readonly.json`、`artifacts/optional_imu_hardware_stationary.json`、`artifacts/optional_imu_console.png`。部署前备份：`artifacts/optional_imu_backup_20261109_205617.tgz`。

## 2026-11-10 默认实际控制修复

问题来自原先三个层次均默认 `receive_only=true`：GUI / 驱动 launch、YAML、C++ 节点。它会明确拒绝使能、模式与速度命令，与本次用户期望的默认实际控制不符。现统一改为 `false`，GUI 和直接运行节点均无需附加该参数。`receive_only:=true` 仍能显式选择只接收，自动使能和 IMU 自动标定继续为独立的显式选项。

远端地址更新为 `orin@172.18.7.207`。Orin 原生编译、devel / install 更新完成；34 个 C++ 核心测试、103 个 Python/ROS/GUI 测试通过，零失败、零跳过。新增用例实际启动三个入口，在不传 `receive_only` 和 `auto_initialize` 参数时验证初始失能、显式使能、CAN 模式、非零速度和停止；原只接收测试改为显式传入 `true`，继续校验全过程无底盘发送。

用户确认底盘架空后，使用 `tools/hardware_bench.py --use-imu false --motion` 测试实际 can0。未传 `receive_only`，诊断确认生效值为 0；前进 / 后退给定 ±0.05 m/s，左右转向给定 ±0.15 rad/s，每段 1.2 秒，均观察到四路轮速响应并确认停转。独立 CAN 抓包中的速度元组只有 `(±50,0,0)`、`(0,0,±150)` 和零速，单位 mm/s、mrad/s；没有异常放大后的发送命令。停止发布的看门狗测试在约 306 ms 后发出零速。结束时确认失能，四路轮速均为 0 RPM，驱动收发错误计数为 0。

这次验证控制链路和停止行为，不代表速度精度通过：B2 反馈峰值仍达 |vx|=1.387 m/s、|wz|=3.928 rad/s，明显超过上述给定值；没有独立测量来判定是真实轮动、控制器或反馈量纲问题。地面定速、定距和定角精度仍需后续验收。

原始证据：`artifacts/default_control_bench_2026-11-10.json`、同名 `.frames.json`、`test_results/pytest.xml`。变更前备份：`artifacts/default_control_backup_20261110_121149.tgz`。
