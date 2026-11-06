# Dex-planning

面向机器人规划算法工程师的 W1 规划与控制源码集合，按路径规划、轨迹时间参数化、运动学、碰撞检查、实时运动规划、动力学与柔顺控制整理。

本仓库保存实际可取得的源码和接口快照，并提供算法阅读入口。客户端来自 `dexe_robot_client 0.8.2`，服务端来自 `robot_server_lib 0.5.1`；具体提交与逐文件 SHA-256 见 [来源清单](docs/source_manifest.json)。

## 从这里开始

| 方向 | 主要源码入口 | 内容 |
| --- | --- | --- |
| 几何路径 | [blend_path_segment.py](client/src/dexe_robot_client/w1/path_planner/blend_path_segment.py)、[blend_path.py](client/src/dexe_robot_client/w1/path_planner/blend_path.py) | 五次 Bézier，Rn、SO(3)、R3×SO(3) 路径和平滑过渡 |
| 笛卡尔到关节空间 | [cartesian_to_joint_planner.py](client/src/dexe_robot_client/w1/path_planner/cartesian_to_joint_planner.py) | 单臂与全身 IK 路径转换 |
| 轨迹时间参数化 | [toppra_trajectory_planner.py](client/src/dexe_robot_client/w1/trajectory_planner/toppra_trajectory_planner.py) | 样条路径、速度/加速度约束、TOPPRA 求解与采样 |
| 全身 IK | [whole_body_ik.py](client/src/dexe_robot_client/w1/kinematics/whole_body_ik.py) | Pinocchio、CasADi，优化目标和约束、IPOPT/SQP 配置 |
| 碰撞与双臂协调 | [collision](client/src/dexe_robot_client/w1/collision)、[trajectory_coordination.py](client/src/dexe_robot_client/w1/trajectory_planner/trajectory_coordination.py) | Coal 碰撞检查，双臂路径进度协调空间中的搜索 |
| 实时运动规划 | [motion_planner](server/robot_server_lib/motion_planner)、[planner_manager.cpp](server/robot_server_lib/src/planner_manager.cpp) | MoveJoint、MoveLine、MoveTraj、流式控制与 Ruckig |
| 服务端运动学 | [kinematics](server/robot_server_lib/kinematics) | SRS/FEP SDK 适配、数值运动学、躯干运动学 |
| 动力学与柔顺控制 | [dynamics](server/robot_server_lib/dynamics)、[torque_controller](server/robot_server_lib/torque_controller) | 逆动力学、工具补偿、导纳、阻抗、PD、力矩前馈 |
| 辨识与信号处理 | [dyn_params_ident](server/script/dyn_params_ident)、[utils](server/robot_server_lib/utils) | 激励轨迹、符号动力学生成、Butterworth 滤波、EI 输入整形 |
| DexeRP 可读 Python 源码 | [sdk_python_excerpt](sdk_python_excerpt) | 单臂数值 IK、躯干解析运动学、TOPPRA 封装 |

完整阅读路线、职责和函数入口见 [算法工程师代码地图](docs/ALGORITHM_MAP.md)。

## 源码结构

```text
client/                  客户端算法及其 Python 通信/模型适配依赖
  src/dexe_robot_client/w1/
  src/examples/          规划、运动学、碰撞示例
server/
  robot_server_lib/      C++ 规划、运动学、动力学、控制、测试与模型
  lib/dexe_rp/dexe_rp/   DexeRP 0.4.4 C++ 头文件
  dexe_interfaces/       运动请求与流式命令接口
  script/dyn_params_ident/
sdk_python_excerpt/      从 DexeRP 0.4.2 安装包取得的 Python 源码片段
docs/                    算法地图、依赖说明、来源与核验记录
tools/verify_export.py   无需机器人或第三方库的源码导出检查
```

## DexeRP 的完整性边界

**DexeRP 的完整 C++ 实现尚未取得。** 已有二进制中的编译路径指向 `dexe_robot_planning/cpp/dexe_rp/`，包括 `SRSKinematics.cpp`、`FEPKinematics.cpp`、`DoubleSProfile.cpp`、`DoubleSTrajectory.cpp`。本仓库提供可取得的 C++ 头文件，未把声明当成实现，也未附带 `.so`、wheel 或 deb 安装包。

`sdk_python_excerpt` 的 Python 源码版本为 **0.4.2**；客户端要求 **dexe_rp >= 0.4.4**，服务端随附头文件为 **0.4.4**。这些快照用于阅读和追踪算法，0.4.2 片段不能替代完整的 0.4.4 SDK。

这是源码提取仓库。运行完整客户端、构建 ROS 2 服务端，需要补齐匹配的 SDK、第三方库、完整模型资源和运行配置，详见 [依赖与复现边界](docs/DEPENDENCIES.md)。

## 本地核验

```bash
python3 tools/verify_export.py
```

检查来源文件 SHA-256、Python 语法、XML/URDF 格式、文档链接及提交内容中的凭据模式。脚本只读文件，不导入机器人包、不连接机器人、不执行运动示例。当前导出核验结果见 [validation.json](docs/validation.json)。

源码提取过程保留原文件内容及其版权/许可声明；新增内容为目录索引、说明和核验工具。未执行整机编译或算法运行测试。
