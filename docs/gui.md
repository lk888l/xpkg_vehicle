# PyQt5 底盘控制台

使用 **PyQt5 + rclpy + PyQtGraph**，通过已有 ROS 2 消息和服务控制底盘。GUI 不直接打开 CAN；`vehicle_driver` 独占设备通信。当前工作树位于 `/home/kk/kk_data/ros2_project/vehicle_socketcan`，ROS 环境为 `ros2_jazzy`。

## 启动

当前机器已经完成依赖准备和构建。在宿主机图形桌面的终端执行：

```bash
cd /home/kk/kk_data/ros2_project/vehicle_socketcan
./tools/dev.sh gui
```

同时启动驱动和 GUI，默认使用 `can0`。驱动收到设备心跳后会使能一次并设置 CAN 模式；没有操作方向按钮时不发送非零速度。关闭 GUI 时先发零速，再关闭本次启动的驱动，驱动默认发送失能。

已有驱动在运行时，只启动 GUI：

```bash
./tools/dev.sh gui start_driver:=false
```

此模式关闭 GUI 会发送零速，外部驱动继续运行，设备使能状态由外部驱动或显式操作决定。两种方式都不应同时运行多个主动控制 GUI。调试底盘时也要避免其他 `/cmd_vel` 发布者争用同一驱动。

其他启动参数：

```bash
# 观察设备，不在启动时自动使能或设置模式
./tools/dev.sh gui auto_initialize:=false
# 同时为 GUI 与驱动设置命名空间
./tools/dev.sh gui namespace:=robot1
# 连接使用 Twist 的已有驱动（两端类型必须一致）
./tools/dev.sh gui start_driver:=false use_stamped_cmd_vel:=false
# 界面启动检查，4 秒后自动关闭
./tools/dev.sh gui auto_initialize:=false quit_after:=4
```

`config:=/容器内路径/vehicle.yaml`、`can_interface:=can0` 也可覆盖。连接已有驱动时，GUI 的 namespace 和 `use_stamped_cmd_vel` 必须与其一致。

容器内手动启动：

```bash
source /opt/ros/jazzy/setup.bash
cd /workspace/vehicle_socketcan
source install/setup.bash
source tools/gui_env.sh
ros2 launch vehicle_gui console.launch.py
# 或只运行 GUI，并覆盖驱动名称
ros2 run vehicle_gui vehicle_gui --ros-args -p driver_name:=vehicle
```

宿主机启动脚本会临时传递当前 X11/Xwayland 的认证 cookie，结束后删除；不修改桌面的全局访问权限。容器须能访问宿主显示服务，当前 host 网络容器已经验证可用。若 `DISPLAY` 为空，请从图形桌面的终端运行。

## 操作与显示

逐项 CAN 映射见 [GUI 功能与 CAN 帧对应关系](gui-can.md)：包含按钮对应的 ID/DLC/载荷、运动与服务发送时序、启动配置、反馈来源、本地操作和退出安排。

- 左侧设置直行速度、旋转速度；按住鼠标方向按钮，或按住 W/A/S/D、方向键运动。松开停止，空格或 Esc 停止，窗口失焦停止。编辑数值时键盘按键只用于编辑；点击窗口空白处可恢复方向快捷键。支持同时按住直行与旋转键。
- 使能、失能、模式和蜂鸣器通过异步 ROS 服务执行，不弹出额外确认对话框。输入区中的模式、蜂鸣器是待发送的设置；顶部设备卡和诊断页显示实际反馈。
- “运动监测”显示实际线速度/角速度、电压、最近 30 秒指令与反馈曲线、四路 RPM/脉冲/电压/电流/温度/原始故障位，以及本地里程计轨迹。温度列依次为驱动器温度、马达温度。电压没有转换成未经标定的电量百分比。
- “诊断与 CAN”提供版本查询、清故障、清运动计数、重置本地位姿；显示诊断统计、整车接收帧和应答记录。报文来自驱动筛选后的 `can_rx`，不包含 0x05 类别的底层驱动器报文。
- “清运动计数”作用于设备里程及驱动脉冲；“重置本地位姿”只作用于 ROS 里程计。GUI 不提供已知有失联现象的设备复位按钮。
- CSV 导出保存有界缓冲中的原始指令和速度反馈，含相对 GUI 启动时间、数据来源和 SI 单位；导出前会停止当前手动运动。每种曲线最多保留 3000 个样本，CSV 不是无限期日志。

状态显示区分 **发送 / 应答 / 状态**。A3/A4/A5 的 ACK 只能确认接收，不能独立证明物理执行效果。清故障和清计数的结果会明确提示观察反馈；使能和模式操作需等新 B0/B1 匹配才显示状态确认。刹车、特殊功能始终标为未知，B1 D5 只显示原始值；未收到 BA 不解释成“无故障”。

曲线不滤波或增加死区，保留设备低速抖动。过期遥测明确标记，长时间无反馈时曲线留出断点。`odom` 来自设备反馈积分，不是外部定位真值；架空时轨迹仅表达反馈计算结果。

## 依赖：只补一个本地包

本机容器已有 PyQt5 **5.15.10**、Qt **5.15.13**、rclpy、NumPy、pytest。只补充 PyQtGraph **0.13.4** 到工作树 `.gui_deps/`，没有安装 Qt6、SciPy、Qt Creator 或 pytest-qt。中文字体复用宿主机已有 Noto Sans CJK 文件，无需下载字体包。

新建相同环境或清理工作树后的准备步骤：

```bash
./tools/dev.sh gui-setup
./tools/dev.sh build
./tools/dev.sh gui
```

`gui-setup` 先检查现有依赖；只有无法导入 PyQtGraph 时才执行：

```bash
python3 -m pip install --no-deps --target "$PWD/.gui_deps" pyqtgraph==0.13.4
```

使用项目内目录避免改动系统 Python，`--no-deps` 复用已有 PyQt5/NumPy。在当前 Ubuntu 软件源中，`apt install python3-pyqtgraph` 会额外拉入 Qt6 等包，因此本工程采用上述安装方式。其他机器若缺少 PyQt5/rclpy/NumPy，需要先提供这些基础依赖，脚本不会隐式安装整套桌面工具。CI 通过 rosdep 准备基础依赖，并跳过 `python3-pyqtgraph` 后单独安装。

`.gui_deps` 不纳入 Git。宿主启动脚本和测试脚本会自动加载 `tools/gui_env.sh`；自行在容器中运行时需显式 source。首次安装需要包源网络，之后可离线启动。

## 结构与并发

`vehicle_gui` 是独立 `ament_python` 包：

```text
window.py       Qt 控件、绘图、按键与鼠标事件
viewmodel.py    操作入口、忙状态和服务结果
model.py        有界遥测快照、事件记录、短时运动意图
ros_worker.py   独立 QThread 内的 rclpy Context / Executor / Node
main.py        进程入口、字体、退出和截图
```

采用 View / ViewModel / ROS 适配器分层。ROS 回调只写线程安全快照，主线程以 20 Hz 刷新控件；跨线程通知使用 Qt signal。服务使用 future，最多等待 3 秒，不在主线程等待网络。所有缓冲均有上限。

方向输入以 50 ms 周期续期，后台最多约 20 Hz 发布 `/cmd_vel`。主线程停止续期超过 200 ms 后，后台发送一次零速并结束重发；UI 恢复时会丢弃旧的按住状态，需重新按下方向键。整个 GUI 进程失去调度时，驱动仍有默认 300 ms 输入超时；设备端另有此前实测约 500 ms 断命令保护。这些机制不以 GUI 定时器作为唯一停止条件，也不宣称是硬实时系统。

## 验证与证据

在当前 Jazzy 容器中：

- 四个包构建通过。
- 原有 17 个 C++ 核心测试、7 个驱动集成测试通过；新增 5 个模型测试、6 个真实 Qt + DDS + vcan 集成测试通过。
- `colcon test-result` 合计 37 项、0 错误、0 失败、0 跳过：35 个实际用例，另 2 个 CTest 外层入口。
- GUI 测试覆盖按钮到 CAN、松开停速、界面冻结后指令过期且恢复不重放、键盘/失焦/空格、ACK 与实际状态分离、持久版本缓存与断连、退出线程回收。不需要 pytest-qt。
- 物理 can0 联调通过 11 项检查：版本 3.0/4.0、四路驱动、速度/里程/CAN、曲线、实际按钮发布 0.05 m/s 和 0.10 rad/s、松开停速、失能 ACK 与心跳确认、线程结束。运动按钮分别短按住 1.2 秒，最终使能状态为 false。此测试验证 GUI 链路，没有重新认定低速精度合格。

[运动联调 JSON](../artifacts/gui_hardware_2026-10-27.json)、[运动界面截图](../artifacts/gui_hardware_2026-10-27_console.png)、[诊断界面截图](../artifacts/gui_hardware_2026-10-27_diagnostics.png)。该报告的最后一次诊断采样跨越失能过程：B2 已停止而旧诊断仍按使能检查，曾记录反馈过期 WARN；同份报告的最终 B0 和服务响应均已确认失能，收发/解码/丢帧错误计数为零。

最终界面无运动复验的 8 项检查也通过：[JSON](../artifacts/gui_final_smoke_2026-10-27.json)、[独立 CAN 抓包](../artifacts/gui_final_smoke_2026-10-27.candump.gz)。失能后等待诊断刷新，结果为 `Online, disabled`，`command_active=0`，收发错误和丢帧计数均为零。宿主机 `./tools/dev.sh gui auto_initialize:=false quit_after:=4` 已验证实际 Xwayland 窗口启动和两个进程正常退出。

### 启动瞬间的查询超时

无运动复验发现两次很早的版本查询没有收到 A2：[第一次记录](../artifacts/gui_version_timeout_2026-10-27.json)、[带抓包复现记录](../artifacts/gui_startup_query_timeout_2026-10-27.json)及[抓包](../artifacts/gui_startup_query_timeout_2026-10-27.candump.gz)。后者的启动自动查询先正常收到 A2；手动查询距最后一条启动反馈配置约 11 ms，能在独立 CAN socket 观察到请求，但总线上没有对应 A2。GUI 显示超时，没有将已缓存版本冒充本次应答。

[失能时的连续查询实验](../artifacts/gui_version_query_timing_2026-10-27.json)中，8 次查询全部应答，最短查询间隔约 150 ms；不能据此认定存在通用的 1 秒查询节流。验收脚本增加启动配置后的 300 ms 稳定时间后，单次查询已成功，未重试。该等待仅用于验收脚本，不在 GUI 中增加操作门槛。现有证据提示启动配置附近的设备响应存在时序限制，尚未确定固件内部原因；若人工操作遇到超时，等待初始化稳定后可重新查询。

软件回归：

```bash
./tools/dev.sh test
```

物理 GUI 验收脚本（需要可用图形显示，或 `QT_QPA_PLATFORM=offscreen`）：

```bash
# 容器内，已 source ROS、工作空间和 gui_env.sh
python3 tools/gui_acceptance.py --interface can0 --motion \
  --output artifacts/my_gui_acceptance.json
```

脚本启动自己的驱动、使用实际 Qt 按钮并保存窗口截图与 JSON，退出时停止自己的实例。省略 `--motion` 可只检查通信与显示。自动化软件测试只向独立 vcan 注入帧，不向 can0 注入模拟数据。
