'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

from .version import __version__

# Import the compiled C++ module
try:
    from .PyDexeRP import *
    from .PyDexeRP import __version__ as _cpp_version
except ImportError as e:
    raise ImportError(
        "Failed to import the compiled C++ module. "
        "Make sure the project is properly built and installed.") from e

# Import Python utilities
from .utils import ArmType, RobotVersion, EndEffectorType, ChassisType, get_idx_mapping, read_config
from .kinematics.single_arm_025f_num_kine import SingleArm025fNumKinematics
from .kinematics.torso_kinematics import (ReducedTorsoQpos, ReducedTorsoPose,
                                          CommonParams, GeometryParams,
                                          ReducedTorsoKineParams,
                                          ReducedTorsoKineParamsRegistry,
                                          ReducedTorsoKinematics)
from .kinematics.w1_kinematics import DexforceW1Kinematics, calc_nearest_iks
from .trajectory.toppra_planner import ToppraPlanner

# Visualization: depends on (trimesh, scipy, loop_rate_limiters, viser)
try:
    from .vis.w1_vis import W1Vis
    from .vis.geometry import Material, GeometryType, Geometry, Box, Sphere, Cylinder, Mesh, PointCloud
    from .vis.utils import rotation_to_wxyz, wxyz_to_rotation, pos_wxyz_to_homogeneous
except ImportError:
    import warnings
    warnings.warn(
        "Visualization modules require optional dependencies.\n"
        "Install with: pip install dexe_rp-xxx.whl[vis]\n", RuntimeWarning)

# Ensure version consistency
if __version__ != _cpp_version:
    import warnings
    warnings.warn(f"Version mismatch: Python package version ({__version__}) "
                  f"differs from C++ module version ({_cpp_version})")
