'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

from typing import Tuple
from pathlib import Path
import numpy as np
import pinocchio as pin

from dexe_rp import (DexeRPLogger, RobotVersion, get_idx_mapping, read_config,
                     SRSKinematics, SRSKineParamsRegistry, FEPKinematics,
                     FEPKineParamsRegistry, StatusType)
from .single_arm_025f_num_kine import SingleArm025fNumKinematics
from .torso_kinematics import ReducedTorsoKinematics, ReducedTorsoKineParamsRegistry


def calc_nearest_iks(kine: SingleArm025fNumKinematics,
                     start_qpos: np.ndarray,
                     pose_list: list[np.ndarray],
                     is_strict: bool = False) -> tuple[bool, list[np.ndarray]]:
    if len(pose_list) == 0:
        kine._logger.error("Pose list is empty.")
        return StatusType.InputInvalid, None

    if start_qpos.shape != (kine.robot_.model.nq,) and start_qpos.shape != (
            kine.robot_.model.nq,
            1) and start_qpos.shape != (1, kine.robot_.model.nq):
        kine._logger.error(
            f"Invalid start_qpos shape {start_qpos.shape}, expected ({kine.robot_.model.nq}, ) or ({kine.robot_.model.nq}, 1)"
        )
        return StatusType.InputInvalid, None

    for i, pose in enumerate(pose_list):
        if pose.shape != (4, 4):
            kine._logger.error(
                f"Invalid pose_list[{i}] shape {pose.shape}, expected (4, 4)")
            return StatusType.InputInvalid, None

    start_qpos = start_qpos.reshape(kine.robot_.model.nq,)

    start_pose = kine.get_fk(start_qpos)
    if start_pose is None:
        return StatusType.Kine_StartPoseNotMatch, None

    if np.max(np.abs(start_pose[:3, 3] - pose_list[0][:3, 3])) > 2e-3:
        kine._logger.error("Start pose does not match the first target pose.")
        return StatusType.Kine_StartPoseNotMatch, None

    def is_continuous(a: np.ndarray, b: np.ndarray) -> bool:
        return np.max(np.abs(a.reshape(-1) - b.reshape(-1))) <= np.deg2rad(30.0)

    qpos_list = [start_qpos.copy()]
    qpos_seed = start_qpos.copy()

    for i in range(1, len(pose_list)):
        success, qpos = kine.get_nearest_ik(
            pose_list[i],
            ref_qpos=qpos_seed,
            is_strict=is_strict,
        )

        if success != StatusType.Success:
            kine._logger.error(f"IK failed at pose index {i}.")
            return StatusType.Kine_NumIKFailed, None

        qpos = qpos.reshape(kine.robot_.model.nq,)

        if not is_continuous(qpos, qpos_seed):
            kine._logger.error(f"IK not continuous at pose index {i}. "
                               f"qpos: {qpos}, qpos_seed: {qpos_seed}")
            return StatusType.Kine_IKNotContinue, None

        qpos_list.append(qpos.copy())
        qpos_seed = qpos

    return StatusType.Success, qpos_list


class DexforceW1Kinematics:
    """All pose are w.r.t. global frame (geometric center of chassis)"""

    def __init__(self):
        self._logger = DexeRPLogger.get_logger()

        self._dof = 20

        self._tcp_map = {
            "head": np.eye(4),
            "left_arm": np.eye(4),
            "right_arm": np.eye(4)
        }

        config = read_config()
        if not config:
            raise RuntimeError("Failed to read config.yaml.")
        self.robot_version = RobotVersion.from_robot_and_version_safe(
            config["robot_type"], config["version"])
        urdf_path = config["file_path"]["robot_urdf"]
        ros2_to_urdf_map = config["ros2_to_urdf_map"]

        model_str = (self.robot_version.robot_family +
                     self.robot_version.version_value).upper()

        self.torso_kine = ReducedTorsoKinematics(
            ReducedTorsoKineParamsRegistry.get("DexForce", model_str))

        if self.robot_version.version_value == "v025f":
            left_urdf_path = Path.home(
            ) / ".cache/dexe_data/assembled/w1/left_arm.urdf"
            self.left_arm_kine = SingleArm025fNumKinematics(
                left_urdf_path, "left_arm")
            right_urdf_path = Path.home(
            ) / ".cache/dexe_data/assembled/w1/right_arm.urdf"
            self.right_arm_kine = SingleArm025fNumKinematics(
                right_urdf_path, "right_arm")
        else:
            Kinematics = None
            ParamsRegistry = None
            if self.robot_version.version_value.startswith("v02"):
                Kinematics = SRSKinematics
                ParamsRegistry = SRSKineParamsRegistry
            elif self.robot_version.version_value.startswith("v03"):
                Kinematics = FEPKinematics
                ParamsRegistry = FEPKineParamsRegistry
            else:
                raise ValueError(
                    f"Unsupported robot version {self.robot_version}")

            self.left_arm_kine = Kinematics(
                ParamsRegistry.get("DexForce", f"{model_str}Left"))
            self.right_arm_kine = Kinematics(
                ParamsRegistry.get("DexForce", f"{model_str}Right"))

        self.left_arm_kine.set_tcp(self._tcp_map["left_arm"])
        self.right_arm_kine.set_tcp(self._tcp_map["right_arm"])

        self._pin_model = pin.buildModelFromUrdf(str(urdf_path))
        self._pin_data = self._pin_model.createData()
        real_keys = tuple(ros2_to_urdf_map.values())
        urdf_keys = tuple(self._pin_model.names)[1:]
        _pin2real_idx = get_idx_mapping(urdf_keys, real_keys)
        self._min_qpos_list = self._pin_model.lowerPositionLimit[_pin2real_idx]
        self._max_qpos_list = self._pin_model.upperPositionLimit[_pin2real_idx]

        # User pos limits
        self._user_min_pos_list = None
        self._user_max_pos_list = None
        if not self.set_user_qpos_limits(self._min_qpos_list,
                                         self._max_qpos_list):
            raise RuntimeError("Failed to set user qpos limits.")

    def get_fk(self, name: str, qpos: np.ndarray) -> Tuple[bool, np.ndarray]:
        """Get forward kinematics of specific link

        Args:
            name (str): ["head", "head_flange"
                        "waist",
                        "left_arm_base", "left_arm", "left_arm_flange", 
                        "right_arm_base", "right_arm", "right_arm_flange"]
            qpos (np.ndarray): dimension should be 20

        Returns:
            Tuple[bool, np.ndarray]: 
        """
        # Check dimension
        if qpos.shape[0] != self._dof:
            self._logger.error(
                f"Dimension of qpos {qpos.shape[0]} != {self._dof}.")
            return False, None

        # Check pos limits
        if np.any(qpos < self._user_min_pos_list):
            idx = np.where(qpos < self._user_min_pos_list)[0]
            self._logger.error(
                f"Qpos below limits at joints {idx+1}, values={qpos[idx]}, min={self._user_min_pos_list[idx]}"
            )
            return False, None

        if np.any(qpos > self._user_max_pos_list):
            idx = np.where(qpos > self._user_max_pos_list)[0]
            self._logger.error(
                f"Qpos above limits at joints {idx+1}, values={qpos[idx]}, max={self._user_max_pos_list[idx]}"
            )
            return False, None

        if self.robot_version.version_value.startswith("v03"):
            link_name_map = {
                "head": "eyes",
                "head_flange": "head_pitch_j2_link",
                "waist": "waist_yaw_link",
                "left_arm_base": "left_arm_base",
                "left_arm": "left_ee",
                "left_arm_flange": "left_ee",
                "right_arm_base": "right_arm_base",
                "right_arm": "right_ee",
                "right_arm_flange": "right_ee"
            }
        else:
            link_name_map = {
                "head": "neck2",
                "head_flange": "neck2",
                "waist": "waist",
                "left_arm_base": "left_arm_base",
                "left_arm": "left_ee",
                "left_arm_flange": "left_ee",
                "right_arm_base": "right_arm_base",
                "right_arm": "right_ee",
                "right_arm_flange": "right_ee"
            }

        if name not in link_name_map:
            valid_names = ", ".join(link_name_map)
            self._logger.error(
                f"Invalid name {name}. Must be one of {[valid_names]}")
            return False, None

        pin.forwardKinematics(self._pin_model, self._pin_data, qpos)
        pin.updateFramePlacements(self._pin_model, self._pin_data)

        link_name = link_name_map[name]
        frame_id = self._pin_model.getFrameId(link_name)
        frame = self._pin_data.oMf[frame_id].homogeneous
        if name in self._tcp_map:
            return True, frame @ self._tcp_map[name]
        return True, frame

    def set_tcp(self, name: str, pose: np.ndarray) -> bool:
        """Set tcp of specific end link

        Args:
            name (str): ["head", "left_arm", "right_arm"]
            pose (np.ndarray): tcp

        Returns:
            bool: 
        """
        if name not in self._tcp_map.keys():
            self._logger.error(
                f"Invalid name {name}. Must be one of {self._tcp_map.keys()}")
            return False

        self._tcp_map[name] = pose
        if name.startswith("left"):
            self.left_arm_kine.set_tcp(pose)
        if name.startswith("right"):
            self.right_arm_kine.set_tcp(pose)
        return True

    def get_tcp(self, name: str) -> Tuple[bool, np.ndarray]:
        """Get tcp of specific end link

        Args:
            name (str): ["head", "left_arm", "right_arm"]

        Returns:
            Tuple[bool, np.ndarray]: 
        """
        tcp = self._tcp_map.get(name)
        if tcp is None:
            valid_names = ", ".join(self._tcp_map)
            self._logger.error(
                f"Invalid name {name}. Must be one of {valid_names}")
            return False, None
        return True, tcp

    def set_user_qpos_limits(self, lower_bound: np.ndarray,
                             upper_bound: np.ndarray) -> bool:
        """Set user qpos limits of specific component

        Args:
            lower_bound (np.ndarray): 
            upper_bound (np.ndarray): 

        Returns:
            bool: 
        """
        # Check dimension
        if lower_bound.shape[0] != self._dof:
            self._logger.error(
                f"Dimension of lower bound {lower_bound.shape[0]} != {self._dof}."
            )
            return False
        if upper_bound.shape[0] != self._dof:
            self._logger.error(
                f"Dimension of upper bound {upper_bound.shape[0]} != {self._dof}."
            )
            return False

        # Check validity
        if np.any(lower_bound > upper_bound):
            self._logger.error(
                f"Lower bound {lower_bound} should <= upper bound {upper_bound}"
            )
            return False

        self._user_min_pos_list = lower_bound
        self._user_max_pos_list = upper_bound

        self.torso_kine.set_qpos_limits(self._user_min_pos_list[0:3],
                                        self._user_max_pos_list[0:3])

        # NOTE: v03x not support change limits now！
        if self.robot_version.version_value.startswith("v02"):
            self.left_arm_kine.set_qpos_limits(self._user_min_pos_list[6:13],
                                               self._user_max_pos_list[6:13])
            self.right_arm_kine.set_qpos_limits(self._user_min_pos_list[13:],
                                                self._user_max_pos_list[13:])
        return True

    def get_user_qpos_limits(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get user qpos limits of specific component

        Returns:
            Tuple[np.ndarray, np.ndarray]: lower_bound, upper_bound
        """
        return self._user_min_pos_list, self._user_max_pos_list
