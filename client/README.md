# 客户端规划算法

来源：`dexe_robot_client 0.8.2`，提交 `bc2168ee3259f2ce3f4e373a1012f3b2b8a53d48`。

核心算法位于 [src/dexe_robot_client/w1](src/dexe_robot_client/w1)。为保留 `W1Manager` 的调用上下文，Python 包中的通信、末端执行器及导航客户端适配层一并保留；这些适配层不代表导航算法内核。

`src/examples` 仅收录规划、运动学和碰撞相关示例。示例属于原项目使用代码，未在本次导出中执行。

原项目的 `pyproject.toml`、`MANIFEST.in` 保持原内容。运行还需要 ROS 2 Python 环境、接口消息、模型资源与匹配的 DexeRP SDK；新增依赖提示见 [依赖说明](../docs/DEPENDENCIES.md)。
