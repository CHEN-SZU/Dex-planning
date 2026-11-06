'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

from dataclasses import dataclass, field
from typing import Optional
import numpy as np

from dexe_rp import DexeRPLogger, StatusType


@dataclass
class ReducedTorsoQpos:
    ankle: float
    knee: float
    buttock: float


@dataclass
class ReducedTorsoPose:
    x: float
    z: float
    y_theta: float


@dataclass(frozen=True)
class CommonParams:
    qpos_lb: np.ndarray
    qpos_ub: np.ndarray
    qpos_offsets: np.ndarray
    axis_dirs: np.ndarray
    pose_b_ob: ReducedTorsoPose
    pose_oe_e: ReducedTorsoPose


@dataclass(frozen=True)
class GeometryParams:
    d_01: float
    d_12: float
    d_23: float
    # # 默认关节轴朝向
    # Jw_home: np.ndarray = field(
    #     default_factory=lambda: np.array([0,0,0],
    #                                      [1,1,1],
    #                                      [0,0,0])
    # )


@dataclass(frozen=True)
class ReducedTorsoKineParams:
    brand: str
    model: str
    common: CommonParams
    geometry: GeometryParams


class ReducedTorsoKineParamsRegistry:
    _models: dict[str, dict[str, ReducedTorsoKineParams]] = {}

    @classmethod
    def register(cls, params: ReducedTorsoKineParams) -> bool:
        cls._models.setdefault(params.brand, {})[params.model] = params
        return True

    @classmethod
    def get(cls, brand: str, model: str) -> Optional[ReducedTorsoKineParams]:
        return cls._models.get(brand, {}).get(model)

    @classmethod
    def supported_robots(cls) -> list[tuple[str, str]]:
        return [(brand, model)
                for brand, model_map in cls._models.items()
                for model in model_map]


# auto init
ReducedTorsoKineParamsRegistry.register(
    ReducedTorsoKineParams(
        "DexForce", "W1V021",
        CommonParams(qpos_lb=np.deg2rad([-90, -155, -90]),
                     qpos_ub=np.deg2rad([90, 155, 110]),
                     qpos_offsets=np.array([0, 0, 0]),
                     axis_dirs=np.array([1, 1, 1]),
                     pose_b_ob=ReducedTorsoPose(0, 0.2875, 0),
                     pose_oe_e=ReducedTorsoPose(0, 0, 0)),
        GeometryParams(d_01=0.34, d_12=0.34, d_23=0.158)))

ReducedTorsoKineParamsRegistry.register(
    ReducedTorsoKineParams(
        "DexForce", "W1V022",
        CommonParams(qpos_lb=np.deg2rad([-90, -155, -90]),
                     qpos_ub=np.deg2rad([90, 155, 110]),
                     qpos_offsets=np.array([0, 0, 0]),
                     axis_dirs=np.array([1, 1, 1]),
                     pose_b_ob=ReducedTorsoPose(0, 0.2875, 0),
                     pose_oe_e=ReducedTorsoPose(0, 0, 0)),
        GeometryParams(d_01=0.34, d_12=0.34, d_23=0.158)))

ReducedTorsoKineParamsRegistry.register(
    ReducedTorsoKineParams(
        "DexForce", "W1V023",
        CommonParams(qpos_lb=np.deg2rad([-90, -155, -90]),
                     qpos_ub=np.deg2rad([90, 155, 110]),
                     qpos_offsets=np.array([0, 0, 0]),
                     axis_dirs=np.array([1, 1, 1]),
                     pose_b_ob=ReducedTorsoPose(0, 0.2875, 0),
                     pose_oe_e=ReducedTorsoPose(0, 0, 0)),
        GeometryParams(d_01=0.34, d_12=0.34, d_23=0.158)))

ReducedTorsoKineParamsRegistry.register(
    ReducedTorsoKineParams(
        "DexForce", "W1V023I",
        CommonParams(qpos_lb=np.deg2rad([-90, -155, -90]),
                     qpos_ub=np.deg2rad([90, 155, 110]),
                     qpos_offsets=np.array([0, 0, 0]),
                     axis_dirs=np.array([1, 1, 1]),
                     pose_b_ob=ReducedTorsoPose(0, 0.2875, 0),
                     pose_oe_e=ReducedTorsoPose(0, 0, 0)),
        GeometryParams(d_01=0.34, d_12=0.34, d_23=0.158)))

ReducedTorsoKineParamsRegistry.register(
    ReducedTorsoKineParams(
        "DexForce", "W1V024",
        CommonParams(qpos_lb=np.deg2rad([-90, -155, -90]),
                     qpos_ub=np.deg2rad([90, 155, 110]),
                     qpos_offsets=np.array([0, 0, 0]),
                     axis_dirs=np.array([1, 1, 1]),
                     pose_b_ob=ReducedTorsoPose(0, 0.2875, 0),
                     pose_oe_e=ReducedTorsoPose(0, 0, 0)),
        GeometryParams(d_01=0.34, d_12=0.34, d_23=0.158)))

ReducedTorsoKineParamsRegistry.register(
    ReducedTorsoKineParams(
        "DexForce", "W1V024I",
        CommonParams(qpos_lb=np.deg2rad([-90, -155, -90]),
                     qpos_ub=np.deg2rad([90, 155, 110]),
                     qpos_offsets=np.array([0, 0, 0]),
                     axis_dirs=np.array([1, 1, 1]),
                     pose_b_ob=ReducedTorsoPose(0, 0.2875, 0),
                     pose_oe_e=ReducedTorsoPose(0, 0, 0)),
        GeometryParams(d_01=0.34, d_12=0.34, d_23=0.158)))

ReducedTorsoKineParamsRegistry.register(
    ReducedTorsoKineParams(
        "DexForce", "W1V025",
        CommonParams(qpos_lb=np.deg2rad([-90, -155, -90]),
                     qpos_ub=np.deg2rad([90, 155, 110]),
                     qpos_offsets=np.array([0, 0, 0]),
                     axis_dirs=np.array([1, 1, 1]),
                     pose_b_ob=ReducedTorsoPose(0, 0.2875, 0),
                     pose_oe_e=ReducedTorsoPose(0, 0, 0)),
        GeometryParams(d_01=0.34, d_12=0.34, d_23=0.158)))

ReducedTorsoKineParamsRegistry.register(
    ReducedTorsoKineParams(
        "DexForce", "W1V026",
        CommonParams(qpos_lb=np.deg2rad([-90, -155, -90]),
                     qpos_ub=np.deg2rad([90, 155, 110]),
                     qpos_offsets=np.array([0, 0, 0]),
                     axis_dirs=np.array([1, 1, 1]),
                     pose_b_ob=ReducedTorsoPose(0, 0.2875, 0),
                     pose_oe_e=ReducedTorsoPose(0, 0, 0)),
        GeometryParams(d_01=0.34, d_12=0.34, d_23=0.158)))

ReducedTorsoKineParamsRegistry.register(
    ReducedTorsoKineParams(
        "DexForce", "W1V030",
        CommonParams(qpos_lb=np.deg2rad([-90, -155, -90]),
                     qpos_ub=np.deg2rad([90, 155, 110]),
                     qpos_offsets=np.array([0, 0, 0]),
                     axis_dirs=np.array([1, 1, 1]),
                     pose_b_ob=ReducedTorsoPose(0, 0.2875, 0),
                     pose_oe_e=ReducedTorsoPose(0, 0, 0)),
        GeometryParams(d_01=0.34, d_12=0.34, d_23=0.153)))

ReducedTorsoKineParamsRegistry.register(
    ReducedTorsoKineParams(
        "DexForce", "W1V031",
        CommonParams(qpos_lb=np.deg2rad([-90, -155, -90]),
                     qpos_ub=np.deg2rad([90, 155, 110]),
                     qpos_offsets=np.array([0, 0, 0]),
                     axis_dirs=np.array([1, 1, 1]),
                     pose_b_ob=ReducedTorsoPose(0, 0.2875, 0),
                     pose_oe_e=ReducedTorsoPose(0, 0, 0)),
        GeometryParams(d_01=0.34, d_12=0.34, d_23=0.153)))

ReducedTorsoKineParamsRegistry.register(
    ReducedTorsoKineParams(
        "DexForce", "W1V021F",
        CommonParams(qpos_lb=np.deg2rad([-90, -155, -90]),
                     qpos_ub=np.deg2rad([90, 155, 110]),
                     qpos_offsets=np.array([0, 0, 0]),
                     axis_dirs=np.array([1, 1, 1]),
                     pose_b_ob=ReducedTorsoPose(0, 0.2875, 0),
                     pose_oe_e=ReducedTorsoPose(0, 0, 0)),
        GeometryParams(d_01=0.34, d_12=0.34, d_23=0.158)))

ReducedTorsoKineParamsRegistry.register(
    ReducedTorsoKineParams(
        "DexForce", "W1V022F",
        CommonParams(qpos_lb=np.deg2rad([-90, -155, -90]),
                     qpos_ub=np.deg2rad([90, 155, 110]),
                     qpos_offsets=np.array([0, 0, 0]),
                     axis_dirs=np.array([1, 1, 1]),
                     pose_b_ob=ReducedTorsoPose(0, 0.2875, 0),
                     pose_oe_e=ReducedTorsoPose(0, 0, 0)),
        GeometryParams(d_01=0.34, d_12=0.34, d_23=0.158)))

ReducedTorsoKineParamsRegistry.register(
    ReducedTorsoKineParams(
        "DexForce", "W1V023F",
        CommonParams(qpos_lb=np.deg2rad([-90, -155, -90]),
                     qpos_ub=np.deg2rad([90, 155, 110]),
                     qpos_offsets=np.array([0, 0, 0]),
                     axis_dirs=np.array([1, 1, 1]),
                     pose_b_ob=ReducedTorsoPose(0, 0.2875, 0),
                     pose_oe_e=ReducedTorsoPose(0, 0, 0)),
        GeometryParams(d_01=0.34, d_12=0.34, d_23=0.158)))

ReducedTorsoKineParamsRegistry.register(
    ReducedTorsoKineParams(
        "DexForce", "W1V024F",
        CommonParams(qpos_lb=np.deg2rad([-90, -155, -90]),
                     qpos_ub=np.deg2rad([90, 155, 110]),
                     qpos_offsets=np.array([0, 0, 0]),
                     axis_dirs=np.array([1, 1, 1]),
                     pose_b_ob=ReducedTorsoPose(0, 0.2875, 0),
                     pose_oe_e=ReducedTorsoPose(0, 0, 0)),
        GeometryParams(d_01=0.34, d_12=0.34, d_23=0.158)))

ReducedTorsoKineParamsRegistry.register(
    ReducedTorsoKineParams(
        "DexForce", "W1V025F",
        CommonParams(qpos_lb=np.deg2rad([-90, -155, -90]),
                     qpos_ub=np.deg2rad([90, 155, 110]),
                     qpos_offsets=np.array([0, 0, 0]),
                     axis_dirs=np.array([1, 1, 1]),
                     pose_b_ob=ReducedTorsoPose(0, 0.2875, 0),
                     pose_oe_e=ReducedTorsoPose(0, 0, 0)),
        GeometryParams(d_01=0.34, d_12=0.34, d_23=0.158)))


# SE2 transform: a @ b
def compose(a: ReducedTorsoPose, b: ReducedTorsoPose) -> ReducedTorsoPose:
    s = np.sin(a.y_theta)
    c = np.cos(a.y_theta)
    x = a.x + c * b.x - s * b.z
    z = a.z + s * b.x + c * b.z
    y_theta = a.y_theta + b.y_theta
    return ReducedTorsoPose(x, z, y_theta)


# SE2 inverse
def inverse(pose: ReducedTorsoPose) -> ReducedTorsoPose:
    s = np.sin(pose.y_theta)
    c = np.cos(pose.y_theta)
    x = -c * pose.x - s * pose.z
    z = s * pose.x - c * pose.z
    y_theta = -pose.y_theta
    return ReducedTorsoPose(x, z, y_theta)


class ReducedTorsoKinematics:

    @dataclass
    class Config:
        # 1: knee-forward, -1: knee-backward
        knee: float

    def __init__(self, kine_params: ReducedTorsoKineParams):
        self.logger = DexeRPLogger.get_logger()
        if kine_params is None:
            self.logger.error("Kinematics parameters cannot be None.")
            raise ValueError("Kinematics parameters cannot be None.")
        self._params = kine_params
        # user defined bound
        self._qpos_lb = None
        self._qpos_ub = None
        # calculation model bound
        self._model_qpos_lb = None
        self._model_qpos_ub = None

    def get_config(self, qpos: ReducedTorsoQpos) -> Config:
        return self.Config(knee=np.sign(-qpos.knee))

    def get_fk(self, qpos: ReducedTorsoQpos) -> ReducedTorsoPose:
        l1 = self._params.geometry.d_01
        l2 = self._params.geometry.d_12
        l3 = self._params.geometry.d_23
        q1 = qpos.ankle
        q2 = qpos.knee
        q3 = qpos.buttock
        q12 = q1 + q2
        q123 = q1 + q2 + q3

        x = l1 * np.sin(q1) + l2 * np.sin(q12) + l3 * np.sin(q123)
        z = l1 * np.cos(q1) + l2 * np.cos(q12) + l3 * np.cos(q123)
        y_theta = q123
        pose_ob_oe = ReducedTorsoPose(x, z, y_theta)
        return compose(compose(self._params.common.pose_b_ob, pose_ob_oe),
                       self._params.common.pose_oe_e)

    def get_ik(self, pose: ReducedTorsoPose,
               cfg: Config) -> tuple[StatusType, Optional[ReducedTorsoQpos]]:
        l1 = self._params.geometry.d_01
        l2 = self._params.geometry.d_12
        l3 = self._params.geometry.d_23
        pose_ob_oe: ReducedTorsoPose = compose(
            inverse(self._params.common.pose_b_ob),
            compose(pose, inverse(self._params.common.pose_oe_e)))
        x = pose_ob_oe.x
        z = pose_ob_oe.z
        y_theta = pose_ob_oe.y_theta

        # Check validity
        if np.sqrt(x**2 + z**2) > (l1 + l2 + l3):
            self.logger.error(f"Target pose out of workspace.")
            return StatusType.Kine_OutOfWorkspace, None

        x_ = x - l3 * np.sin(y_theta)
        z_ = z - l3 * np.cos(y_theta)
        c2 = (l1**2 + l2**2 - (x_**2 + z_**2)) / (2 * l1 * l2)
        if cfg.knee > 0:
            q2 = np.arccos(c2) - np.pi
        else:
            q2 = np.pi - np.arccos(c2)
        q1 = np.arctan2(x_, z_) - np.arctan2(l2 * np.sin(q2),
                                             l1 + l2 * np.cos(q2))
        q3 = y_theta - q1 - q2

        if not self._check_qpos_limits(np.array([q1, q2, q3]), self._qpos_lb,
                                       self._qpos_ub):
            return StatusType.Kine_OutOfPosLimit, None

        return StatusType.Success, ReducedTorsoQpos(q1, q2, q3)

    def set_qpos_limits(self, qpos_lb: np.ndarray, qpos_ub: np.ndarray) -> bool:
        if qpos_lb.shape != qpos_ub.shape:
            self.logger.error(
                f"Dimension mismatch: {qpos_lb.shape} != {qpos_ub.shape}")
            return False

        if np.any(qpos_lb > qpos_ub):
            self.logger.error(
                f"Invalid qpos limits: lower_bound {qpos_lb} is greater than upper_bound {qpos_ub}"
            )
            return False

        self._qpos_lb = qpos_lb.copy()
        self._qpos_ub = qpos_ub.copy()
        return True

    def get_qpos_limits(self) -> tuple[np.ndarray, np.ndarray]:
        return self._qpos_lb, self._qpos_ub

    def _check_qpos_limits(self,
                           qpos: np.ndarray,
                           qpos_lb: np.ndarray,
                           qpos_ub: np.ndarray,
                           verbose: bool = True) -> bool:
        invalid_mask = (qpos < qpos_lb) | (qpos > qpos_ub)
        if not np.any(invalid_mask):
            return True
        if verbose:
            for i in np.where(invalid_mask)[0]:
                self.logger.error(
                    f"Joint {i + 1} out of pos limit: "
                    f"value={qpos[i]}, range=[{qpos_lb[i]}, {qpos_ub[i]}]")
        return False


# class ReducedTorsoKinematics(ReducedTorsoKinematicsBase):
#     def __init__(self, kine_params: ReducedTorsoKineParams):
#         super().__init__(kine_params)
