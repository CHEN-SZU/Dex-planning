'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

import casadi
import numpy as np
import pinocchio
import os
from dataclasses import dataclass
from pinocchio import casadi as pin_casadi
from dexe_rp import DexeRPLogger, StatusType


@dataclass
class TrackingFrame:
    left_arm: np.ndarray = np.array([[0.0, 0.0, -1.0, -0.078],
                                     [0.0, 1.0, 0.0, 0.0],
                                     [1.0, 0.0, 0.0, 0.0025],
                                     [0.0, 0.0, 0.0, 1.0]])
    right_arm: np.ndarray = np.array([[0.0, 0.0, -1.0, -0.078],
                                      [0.0, 1.0, 0.0, 0.0],
                                      [1.0, 0.0, 0.0, 0.0025],
                                      [0.0, 0.0, 0.0, 1.0]])


@dataclass
class WeightConfig:
    translation: float = 500.0
    rotation: float = 10.0
    regularization: float = 0.0
    smoothness: float = 5.0


class SingleArm025fNumKinematics:

    def __init__(self,
                 urdf_path: str,
                 name: str,
                 tracking_frame: TrackingFrame = TrackingFrame(),
                 weight_config: WeightConfig = WeightConfig()):
        self._logger = DexeRPLogger.get_logger()
        if not os.path.exists(urdf_path):
            self._logger.error(f"URDF file '{urdf_path}' not found")
            raise FileNotFoundError(f"URDF file '{urdf_path}' not found")
        if name not in ["left_arm", "right_arm"]:
            self._logger.error(
                f"Invalid name '{name}', expected 'left_arm' or 'right_arm'")
            raise ValueError(
                f"Invalid name '{name}', expected 'left_arm' or 'right_arm'")
        self.urdf_path = urdf_path
        self.urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
        self.name = name

        self.robot_ = pinocchio.RobotWrapper.BuildFromURDF(
            urdf_path, self.urdf_dir)
        self.tracking_frame = tracking_frame
        self.weight_config = weight_config

        self._add_tracking_frame()
        self._create_pin_casadi()
        self.set_problem()

        self.tcp = np.eye(4)

    def _add_tracking_frame(self):
        if self.name == "left_arm":
            frame_name = "tracking_frame_left_arm"
            parent_joint = "LEFT_J7"
            parent_T_frame = self.tracking_frame.left_arm
        elif self.name == "right_arm":
            frame_name = "tracking_frame_right_arm"
            parent_joint = "RIGHT_J7"
            parent_T_frame = self.tracking_frame.right_arm
        else:
            raise ValueError(
                f"Invalid name '{self.name}', expected 'left_arm' or 'right_arm'"
            )

        self.robot_.model.addFrame(
            pinocchio.Frame(
                f"{frame_name}", self.robot_.model.getJointId(parent_joint),
                pinocchio.SE3(parent_T_frame[:3, :3], parent_T_frame[:3, 3]),
                pinocchio.FrameType.OP_FRAME))
        self.robot_.data = self.robot_.model.createData()

        self.tracking_frame_id = self.robot_.model.getFrameId(frame_name)

    def _create_pin_casadi(self):
        self.cmodel = pin_casadi.Model(self.robot_.model)
        self.cdata = self.cmodel.createData()

        self.cq = casadi.SX.sym("q", self.robot_.model.nq, 1)
        self.cee_tf = casadi.SX.sym("ee_tf", 4, 4)
        pin_casadi.framesForwardKinematics(self.cmodel, self.cdata, self.cq)

    def set_problem(self):
        self.opti = casadi.Opti()
        self.var_q = self.opti.variable(self.robot_.model.nq)
        self.var_q_last = self.opti.parameter(self.robot_.model.nq)
        self.param_ee_tf = self.opti.parameter(4, 4)
        self.param_translation_threshold = self.opti.parameter()
        self.param_rotation_threshold = self.opti.parameter()

        solver_opts = {
            'ipopt': {
                'print_level': 0,
                'max_iter': 100,
                'tol': 1e-4
            },
            'print_time': False,
            'calc_lam_p': False,
        }
        self.opti.solver('ipopt', solver_opts)

        translation_error = [
            self.cdata.oMf[self.tracking_frame_id].translation -
            self.cee_tf[:3, 3]
        ]
        translation_err_func = casadi.Function(
            "translation_error", [self.cq, self.cee_tf],
            [casadi.vertcat(*translation_error)])
        self.translation_cost = casadi.sumsqr(
            translation_err_func(self.var_q, self.param_ee_tf))

        R_current = self.cdata.oMf[self.tracking_frame_id].rotation
        R_target = self.cee_tf[:3, :3]
        rotation_error = [
            R_current[:, 0] - R_target[:, 0],
            R_current[:, 1] - R_target[:, 1],
            R_current[:, 2] - R_target[:, 2],
        ]
        rotation_err_func = casadi.Function("rotation_error",
                                            [self.cq, self.cee_tf],
                                            [casadi.vertcat(*rotation_error)])
        self.rotation_cost = casadi.sumsqr(
            rotation_err_func(self.var_q, self.param_ee_tf))

        self.regularization_cost = casadi.sumsqr(self.var_q)
        self.smoothness_cost = casadi.sumsqr(self.var_q - self.var_q_last)

        lower = self.robot_.model.lowerPositionLimit
        upper = self.robot_.model.upperPositionLimit
        margin = 0.01 * (upper - lower)
        inner_lower = lower + margin
        inner_upper = upper - margin
        self.opti.subject_to(
            self.opti.bounded(inner_lower, self.var_q, inner_upper))

        if self.name == "left_arm":
            self.opti.subject_to(self.var_q[3] <= 0.0)
        elif self.name == "right_arm":
            self.opti.subject_to(self.var_q[3] >= 0.0)

        self.opti.subject_to(
            self.translation_cost <= self.param_translation_threshold)
        self.opti.subject_to(
            self.rotation_cost <= self.param_rotation_threshold)

        self.opti.minimize(
            self.weight_config.translation * self.translation_cost +
            self.weight_config.rotation * self.rotation_cost +
            self.weight_config.regularization * self.regularization_cost +
            self.weight_config.smoothness * self.smoothness_cost)

    def get_nearest_ik(self,
                       ee_target: np.ndarray,
                       ref_qpos: np.ndarray = None,
                       is_strict: bool = False):
        ee_target = ee_target @ np.linalg.inv(self.tcp)
        if ref_qpos is None:
            ref_qpos = np.zeros(self.robot_.model.nq)

        self.opti.set_initial(self.var_q, ref_qpos)
        self.opti.set_value(self.var_q_last, ref_qpos)
        self.opti.set_value(self.param_ee_tf, ee_target)
        if is_strict:
            self.opti.set_value(self.param_translation_threshold, 1e-6)
            self.opti.set_value(self.param_rotation_threshold, 5e-5)
        else:
            self.opti.set_value(self.param_translation_threshold, 1e3)
            self.opti.set_value(self.param_rotation_threshold, 1e3)

        try:
            sol = self.opti.solve()
            q_sol = sol.value(self.var_q)
            return StatusType.Success, q_sol
        except Exception as e:
            self._logger.error(f"IK solve failed: {e}")
            return StatusType.Kine_NumIKFailed, None

    def set_tcp(self, tcp: np.ndarray) -> bool:
        if tcp.shape != (4, 4):
            self._logger.error(
                f"Invalid TCP shape {tcp.shape}, expected (4, 4)")
            return StatusType.InputInvalid
        self.tcp = tcp
        return StatusType.Success

    def get_tcp(self) -> bool:
        return self.tcp

    def set_qpos_limits(self, lower_bound: np.ndarray,
                        upper_bound: np.ndarray) -> bool:
        if np.any(lower_bound > upper_bound):
            self._logger.error(
                f"Lower bound {lower_bound} should <= upper bound {upper_bound}"
            )
            return StatusType.InputInvalid
        if lower_bound.shape != (
                self.robot_.model.nq,) and lower_bound.shape != (
                    self.robot_.model.nq,
                    1) and lower_bound.shape != (1, self.robot_.model.nq):
            self._logger.error(
                f"Invalid lower bound shape {lower_bound.shape}, expected ({self.robot_.model.nq}, ) or ({self.robot_.model.nq}, 1)"
            )
            return StatusType.InputInvalid
        if upper_bound.shape != (
                self.robot_.model.nq,) and upper_bound.shape != (
                    self.robot_.model.nq,
                    1) and upper_bound.shape != (1, self.robot_.model.nq):
            self._logger.error(
                f"Invalid upper bound shape {upper_bound.shape}, expected ({self.robot_.model.nq}, ) or ({self.robot_.model.nq}, 1)"
            )
            return StatusType.InputInvalid
        self.robot_.model.lowerPositionLimit = lower_bound.reshape(
            self.robot_.model.nq,)
        self.robot_.model.upperPositionLimit = upper_bound.reshape(
            self.robot_.model.nq,)

        self.set_problem()

        return StatusType.Success

    def get_qpos_limits(self) -> tuple[np.ndarray, np.ndarray]:
        lower_bounds = np.asarray(self.robot_.model.lowerPositionLimit)
        upper_bounds = np.asarray(self.robot_.model.upperPositionLimit)
        return lower_bounds, upper_bounds

    def get_fk(self, qpos: np.ndarray) -> np.ndarray:
        if qpos.shape != (self.robot_.model.nq,) and qpos.shape != (
                self.robot_.model.nq,
                1) and qpos.shape != (1, self.robot_.model.nq):
            self._logger.error(
                f"Invalid qpos shape {qpos.shape}, expected ({self.robot_.model.nq}, ) or ({self.robot_.model.nq}, 1)"
            )
            return None
        qpos = qpos.reshape(self.robot_.model.nq,)
        pinocchio.forwardKinematics(self.robot_.model, self.robot_.data, qpos)
        pinocchio.updateFramePlacements(self.robot_.model, self.robot_.data)
        if self.name == "left_arm":
            ee_tf = self.robot_.data.oMf[self.robot_.model.getFrameId(
                "LEFT_EE")].homogeneous
            ee_tf = ee_tf @ self.tcp
        elif self.name == "right_arm":
            ee_tf = self.robot_.data.oMf[self.robot_.model.getFrameId(
                "RIGHT_EE")].homogeneous
            ee_tf = ee_tf @ self.tcp
        else:
            self._logger.error(
                f"Invalid name '{self.name}', expected 'left_arm' or 'right_arm'"
            )
            return None
        return ee_tf

    def calc_nearest_iks(
            self,
            start_qpos: np.ndarray,
            pose_list: list[np.ndarray],
            is_strict: bool = False) -> tuple[StatusType, list[np.ndarray]]:
        if len(pose_list) == 0:
            self._logger.error("Pose list is empty.")
            return StatusType.InputInvalid, None

        if start_qpos.shape != (self.robot_.model.nq,) and start_qpos.shape != (
                self.robot_.model.nq,
                1) and start_qpos.shape != (1, self.robot_.model.nq):
            self._logger.error(
                f"Invalid start_qpos shape {start_qpos.shape}, expected ({self.robot_.model.nq}, ) or ({self.robot_.model.nq}, 1)"
            )
            return StatusType.InputInvalid, None

        for i, pose in enumerate(pose_list):
            if pose.shape != (4, 4):
                self._logger.error(
                    f"Invalid pose_list[{i}] shape {pose.shape}, expected (4, 4)"
                )
                return StatusType.InputInvalid, None

        start_qpos = start_qpos.reshape(self.robot_.model.nq,)

        start_pose = self.get_fk(start_qpos)
        if start_pose is None:
            return StatusType.Kine_StartPoseNotMatch, None

        if np.max(np.abs(start_pose - pose_list[0])) > 1e-4:
            self._logger.error(
                "Start pose does not match the first target pose.")
            return StatusType.Kine_StartPoseNotMatch, None

        def is_continuous(a: np.ndarray, b: np.ndarray) -> bool:
            return np.max(np.abs(a.reshape(-1) -
                                 b.reshape(-1))) <= np.deg2rad(30.0)

        qpos_list = [start_qpos.copy()]
        qpos_seed = start_qpos.copy()

        for i in range(1, len(pose_list)):
            success, qpos = self.get_nearest_ik(
                pose_list[i],
                ref_qpos=qpos_seed,
                is_strict=is_strict,
            )

            if success != StatusType.Success:
                self._logger.error(f"IK failed at pose index {i}.")
                return StatusType.Kine_NumIKFailed, None

            qpos = qpos.reshape(self.robot_.model.nq,)

            if not is_continuous(qpos, qpos_seed):
                self._logger.error(f"IK not continuous at pose index {i}. "
                                   f"qpos: {qpos}, qpos_seed: {qpos_seed}")
                return StatusType.Kine_IKNotContinue, None

            qpos_list.append(qpos.copy())
            qpos_seed = qpos

        return StatusType.Success, qpos_list
