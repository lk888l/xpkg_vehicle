# ROS1 存档同步：2026-11-11

本工程以 ROS2 为主，`ros1/` 仅保存 reComputer Mini J3011 上实际部署的 ROS1 Noetic 工程。ROS2 源码、配置、工具和默认构建入口保持原样；工程根目录只更新 README 的存档说明。

## 来源与范围

- 设备：`orin@172.18.7.207`。
- 工作区：`/home/orin/vehicle_noetic_ws`，实际指向 `/home/orin/robotics/dependencies/vehicle_noetic_ws`。
- 同步四个包的源码、消息与服务、配置、launch、测试、工具、文档、许可证，以及已有的 JSON / XML / PNG / patch 验证证据，共 119 个远程文件。
- 排除 `build/`、`devel/`、`install/`、`test_results/`、日志、Python 缓存、`.catkin_workspace` 和旧备份压缩包。

远程工作区没有 Git 元数据，使用逐文件 SHA-256 和权限记录标识快照：[来源清单](../artifacts/source_snapshot_2026-11-11.json)。其中 118 个文件保持远程原样；`README.md` 仅修订存档定位、独立文件说明、旧地址和与当前 CAN 配置不符的操作示例。新增的同步记录和本次复测报告属于本地存档元数据。

原有 11 处软链接已全部替换为远程实际文件或目录，包含核心算法、头文件、核心测试、部分 GUI 测试和许可证。存档可脱离上级 ROS2 工程构建；保留 `COLCON_IGNORE`，默认 ROS2 构建会跳过本目录。

## 归档版本的行为

| 项目 | 当前 ROS1 默认值与行为 |
|---|---|
| 底盘接口 | `can0`，500 kbit/s Classic CAN |
| IMU | 同一 `can0`，`imu_protocol=classic`，节点号 31，支持四 PDO；独立旧版 CAN-FD 使用 `imu_protocol=legacy_fd` |
| 有无 IMU | 启动时选择 `use_imu:=true|false`，默认 true；无 IMU 时使用底盘里程计航向 |
| 底盘发送 | `receive_only=false`；只接收需显式设置 `receive_only:=true` |
| 初始化与标定 | `auto_initialize=false`、`auto_imu_calibration=false`，使能和模式由操作者明确下达 |
| 控制台 | 显示实际生效的 IMU 策略，无 IMU 时禁用 IMU 专属操作 |

详见 [ROS1 使用说明](../README.md)、[可选 IMU 架构及既有实机结果](optional-imu.md)。`validation.md` 和旧日期的 artifacts 保留原测试时的地址、默认值和结果，不能用来描述当前默认值。

## 复测方式

从本地独立存档通过 `tools/export_source.sh` 导出，再放到设备上的新临时目录中构建；没有覆盖部署工作区。测试使用单独的 `vcan_ar1111` 和 `http://127.0.0.1:11349` ROS master，GUI 使用 offscreen 模式。未运行实机控制、网络配置或 systemd 安装脚本。

在临时工作区以 root 调用存档原有入口（虚拟接口操作需要 NET_ADMIN）：

```bash
bash tools/build.sh
VEHICLE_TEST_CAN=vcan_ar1111 \
VEHICLE_TEST_MASTER_URI=http://127.0.0.1:11349 \
bash tools/test.sh
```

本地另检查 Python 3.8 语法、shell 语法、launch / package XML、可执行权限、快照哈希、存档无软链接，以及 ROS2 文件保持不变。

## 本次复测结果

在设备原生 Ubuntu 20.04.6 / ARM64 / ROS1 Noetic / GCC 9.4 / Python 3.8.10 环境中，独立存档 catkin 构建成功，137 项测试通过，零失败、零错误、零跳过。

| 范围 | 通过数量 | 原始结果 |
|---|---:|---|
| C++ 核心协议、IMU、里程计、标定、运动与通信 | 34 | [gtest XML](../artifacts/archive_sync_core_2026-11-11.xml) |
| Python / ROS1 / GUI、共享 CAN、可选 IMU、默认控制和只接收保护 | 103 | [pytest XML](../artifacts/archive_sync_pytest_2026-11-11.xml) |

catkin 汇总中的 68 是同一核心测试结果被重复计数，以上采用原始 gtest XML 的 34 项。pytest 耗时约 188 秒，包含系统旧版 PyQtGraph 的 10007 条数值 / Qt 枚举隐式转换弃用警告。

本地语法与结构检查通过：22 个 Python 入口 / 模块、7 个 shell 脚本、7 个 launch / package XML；76 个源码、工具和测试文件与远程逐字节一致。存档外的 123 个受跟踪文件及其权限保持原样，仅另行更新根 README 的存档说明。

测试创建的 ROS master、虚拟 CAN 和临时工作区均已清理。复测结束后重新读取全部 119 个远程文件，确认部署工作区内容未变。机器汇总与完整检查结果见[本次复测记录](../artifacts/archive_sync_validation_2026-11-11.json)。本次未重新验证实车运动精度或实物 IMU。
