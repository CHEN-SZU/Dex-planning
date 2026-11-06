# DexeRP 0.4.2 Python 源码片段

这些文件来自 PC1 已安装的 `dexe_rp 0.4.2`，通过只读连接复制到本机，再纳入此仓库。原安装包的元数据保存在 [METADATA.upstream](METADATA.upstream)。

| 文件 | 作用 |
| --- | --- |
| [single_arm_025f_num_kine.py](dexe_rp/kinematics/single_arm_025f_num_kine.py) | Pinocchio 符号 FK，CasADi 优化建模，IPOPT 数值 IK |
| [torso_kinematics.py](dexe_rp/kinematics/torso_kinematics.py) | 躯干几何参数及解析 FK/IK |
| [w1_kinematics.py](dexe_rp/kinematics/w1_kinematics.py) | W1 不同构型求解器选择和连续 IK 组织 |
| [toppra_planner.py](dexe_rp/trajectory/toppra_planner.py) | TOPPRA 路径参数化封装，核心求解由外部 toppra 库提供 |

此目录不是完整的可安装包。原 `__init__.py` 仍引用未收录的 `PyDexeRP`、工具和可视化模块，以保留原始接口关系；没有重写为可独立安装的替代实现。

客户端使用的 SDK 要求为 `>=0.4.4`；不能把本目录加入运行时 `PYTHONPATH` 来替代该依赖。
