# 规划算法工程师代码地图

## 1. 主链路

```mermaid
flowchart LR
  A[稀疏目标或末端位姿] --> B[几何路径与平滑]
  B --> C[IK 转关节路径]
  C --> D[碰撞检查与路径协调]
  D --> E[轨迹时间参数化]
  E --> F[运动请求与服务端 PlannerManager]
  F --> G[周期关节命令]
  G --> H[动力学补偿与控制]
```

图中表示模块职责关系；实际调用会随请求类型和配置选择分支。`W1Manager` 的 `USE_TOPPRA = False` 使部分路径走 DexeRP DoubleS；全身 IK 后的密集路径仍可能使用 TOPPRA，应按具体调用追踪。

## 2. 优先阅读的算法

下表中的客户端路径相对于 `client/src/dexe_robot_client/w1/`。

| 模块 | 文件与入口 | 重点理解与可开发内容 |
| --- | --- | --- |
| 五次 Bézier 与旋转路径 | `path_planner/blend_path_segment.py`：`calc_5th_bezier_Rn_control_points`、`calc_5th_bezier_SO3_control_points` | 位置和旋转的表达、切向连续性、曲线导数、权重与尺度 |
| 分段路径和平滑 | `path_planner/blend_path.py`：`PathRnBezier.calc_path`、`PathR3xSO3Bezier.calc_path` | 稀疏点与密集路径的区别、转角过渡、采样精度 |
| 笛卡尔路径 | `path_planner/cartesian_planner.py`：`CartesianPlanner` | 平移/姿态插值和路径表达 |
| 笛卡尔转关节 | `path_planner/cartesian_to_joint_planner.py`：`plan_single_arm`、`plan_whole_body` | 参考关节种子、IK 连续性、失败处理、全身求解分支 |
| 全身优化 IK | `kinematics/whole_body_ik.py`：`WholeBodyIK.solve_ik`、`IKProblemSpec` | 末端误差、姿态与平滑目标、关节约束、可配置问题构建、求解耗时 |
| IK 管理 | `kinematics/whole_body_ik_manager.py`：`WholeBodyIKSolverManager.solve_ik` | 构型适配、求解器创建和复用 |
| 模型与末端挂载 | `kinematics/pinocchio_kinematics.py` | URDF/关节索引映射、FK、末端安装变换 |
| TOPPRA | `trajectory_planner/toppra_trajectory_planner.py`：`ToppraTrajectoryPlanner.plan` | 样条、逐段速度限制、加速度限制、可行性、采样终点 |
| 碰撞 | `collision/collision_checker_wrapper.py`、`coal_collision_wrapper.py` | 碰撞模型、碰撞对、配置与运动有效性；核心几何计算由 Coal 提供 |
| 双臂协调空间搜索 | `trajectory_planner/trajectory_coordination.py`：`CoordinationSpaceRRT`、`DualArmCoordination` | 用左右臂既定路径的进度组成二维空间，碰撞有效性与单向进度检查 |

双臂协调空间搜索调整的是两条已给定路径的执行配合；它与在完整关节空间重新搜索任意绕障路径的任务范围不同。几何 Bézier 平滑本身也不保证避障。

## 3. 服务端实时规划与控制

下列路径相对于 `server/robot_server_lib/`。

| 文件或目录 | 关键入口与职责 |
| --- | --- |
| `include/planner_manager.h`、`src/planner_manager.cpp` | `PlannerManager`：任务调度、规划器选择、停止与状态转换 |
| `motion_planner/move_planner_base.*` | `MotionPlannerBase`：`ResetForNewTrajectory`、`ComputeCommand`、`ComputeStopCommand` 接口 |
| `motion_planner/move_joint_planner.*` | 关节目标，Ruckig 速度/加速度/jerk 约束，完成条件 |
| `motion_planner/move_line_planner.*` | 末端直线运动与运动学转换 |
| `motion_planner/move_traj_planner.*` | 已规划轨迹的执行与采样 |
| `motion_planner/joint_streaming_planner.*` | 流式关节指令、在线平滑与超时 |
| `motion_planner/ee_pose_streaming_planner.*` | 流式末端位姿指令 |
| `motion_planner/ee_velocity_streaming_planner.*` | 流式末端速度指令 |
| `kinematics/kinematics_srs.*`、`kinematics_fep.*` | 对 DexeRP 求解器的适配；完整 SRS/FEP 实现在 SDK 中 |
| `kinematics/kinematics_numerical.*`、`kinematics_torso.*` | 数值运动学和躯干运动学源码 |
| `dynamics/inverse_dynamics.*` | 逆动力学接口与数值计算 |
| `dynamics/inverse_dynamics_symbol*` | 符号动力学调度，以及 W1 V02/V03 生成的回归与质量项函数 |
| `dynamics/tool_compensation.*` | 工具动力学补偿 |
| `dynamics/joint_admittance_controller.*` | 关节导纳 |
| `dynamics/cartesian_impedance_controller.*` | 笛卡尔阻抗 |
| `dynamics/joint_pd_controller.*`、`torque_controller/` | PD 与逆动力学力矩控制组织 |
| `utils/butterworth_filter.*`、`ei_input_shaper.*` | 滤波、振动抑制及其已有测试 |

`server/dexe_interfaces/robot_server_interfaces` 给出 `MoveRobot.action`、`StreamingCommand.msg` 等边界数据结构。`src/robot_server_base.cpp` 保留服务端上下文，便于追踪规划请求和反馈。

## 4. DexeRP 中能读到什么

- **可读 Python 算法**：`sdk_python_excerpt/dexe_rp/kinematics/single_arm_025f_num_kine.py` 的 `set_problem`/`get_nearest_ik`，`torso_kinematics.py` 的 `get_fk`/`get_ik`。
- **可读封装**：`sdk_python_excerpt/dexe_rp/trajectory/toppra_planner.py` 配置外部 TOPPRA；`w1_kinematics.py` 选择 SRS/FEP 或单臂数值 IK。
- **可读 C++ 接口及部分内联实现**：`server/lib/dexe_rp/dexe_rp` 中的运动学、路径、轨迹、碰撞、规划头文件。
- **尚缺的完整实现**：DexeRP 的 SRS/FEP、DoubleS 等 `.cpp`，以及未取得的 C++ SDK 其他实现。头文件中的 OMPL、Coal 等接口也不能等同于其对应 SDK 源码已经齐全。

## 5. 工程师开发顺序

1. 先画清目标位姿、路径点、关节位置/速度/加速度与控制命令之间的转换，明确单位、坐标系、关节顺序。
2. 修改几何路径和 IK 时，检查端点、连续性、限位、可达性和碰撞；比较 IK 成功率、关节跳变与求解耗时。
3. 修改时间参数化时，检查轨迹终点与边界状态，并分别统计速度、加速度和 jerk；TOPPRA 当前封装只设置速度和加速度约束。
4. 修改实时规划器时，检查停止、超时、任务完成条件和反馈状态，区分参考位置与测量位置。
5. 修改动力学或柔顺控制时，先检查重力方向、惯性参数、工具坐标、控制周期和限幅；复用 `server/robot_server_lib/test` 中相关测试组织离线验证。

本次只做源码整理与静态核验，未执行上述算法实验。模型与 SDK 准备齐全后，可从 `client/src/examples/planner`、`examples/kinematics` 和 `examples/collision` 阅读原项目的使用方式。
