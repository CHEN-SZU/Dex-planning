# 依赖与复现边界

## 来源版本

| 来源 | 版本 / 提交 | 本仓库收录内容 |
| --- | --- | --- |
| dexe_robot_client | 0.8.2 / `bc2168ee3259f2ce3f4e373a1012f3b2b8a53d48` | Python 包源码、相关示例、原构建元数据 |
| robot_server_lib | 0.5.1 / `140d57be1ab779bb1dd905ebc63f7c7f9c1bcade` | C++ 算法、管理器、测试、URDF 与碰撞网格、辨识脚本 |
| server 随附 DexeRP 头文件 | 0.4.4 | `server/lib/dexe_rp/dexe_rp` |
| 机器人已安装 DexeRP Python 包 | 0.4.2 | 6 个 Python 文件及原 METADATA；完整包未取得 |

逐文件来源、哈希和大小见 [source_manifest.json](source_manifest.json)。本次以当前本机已同步源码和先前取得的 SDK 片段为输入，没有重新修改或同步机器人上的代码。

## 客户端

保留的原始 [pyproject.toml](../client/pyproject.toml) 声明 Python >=3.10，以及 numpy、scipy、toppra、trimesh、pathfinding、dexe_rp 等依赖。根据源码导入，运行特定算法还需要：

| 功能 | 依赖 |
| --- | --- |
| 全身/单臂优化 IK | CasADi、含 CasADi 支持的 Pinocchio，所选优化器 |
| 碰撞检查 | Coal 及相应 Python 接口、完整碰撞模型 |
| 双臂协调空间 RRT | OMPL Python 接口；相关模块还导入 IPython |
| W1Manager 与 ROS 通信 | ROS 2 / rclpy、生成的消息与 action 包 |
| 模型加载与末端挂载 | 与机器人版本一致的 URDF、末端资源、索引配置及数据目录 |
| DoubleS、SRS/FEP、其他 SDK API | 与客户端匹配的完整 `dexe_rp >=0.4.4` 安装包 |

不要把 `sdk_python_excerpt` 作为可安装的 DexeRP 包，也不要把这些不同版本的片段组合成已验证的 SDK。原项目元数据仍可能没有穷尽所有可选模块的依赖。

## C++ 服务端

原 CMake 入口为 [robot_server_lib/CMakeLists.txt](../server/robot_server_lib/CMakeLists.txt)。构建依赖包括 ROS 2/ament、Eigen、Pinocchio、Ruckig、yaml-cpp、fmt、spdlog、nlohmann_json、magic_enum 和机器人接口消息等。

CMake 仍按原工程布局寻找架构匹配的 `libDexeRP.so`。本仓库只收录头文件，未提供该库，也未导出整机启动、部署或硬件驱动工程，因此不是完整的 colcon 工作区。

保留 URDF 和碰撞网格以展示模型接口关系；可视化纹理、DAE/GLB 等 visual 资源未导出，URDF 的 visual 引用需要从原项目资源补齐。

## 已执行的验证

`tools/verify_export.py` 使用 Python 标准库执行：

- 所有提取文件的 SHA-256 与来源清单逐项一致；
- 所有 `.py` 文件的语法解析；
- XML/URDF 格式解析与本仓库 Markdown 相对链接检查；
- 排除编译库、安装包、缓存、私钥和常见凭据格式。

检查结果记录在 [validation.json](validation.json)。这些检查证明导出内容完整且静态可解析，不构成算法正确性、数值性能、C++ 可构建性或机器人运行行为验证。
