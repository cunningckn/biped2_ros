"""Pinocchio-based forward and inverse kinematics for biped2 legs."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, List, Tuple

import numpy as np
import pinocchio as pin


JOINT_NAMES: List[str] = [
    'LeftHipYaw',
    'LeftHipRoll',
    'LeftHipPitch',
    'LeftKneePitch',
    'LeftAnklePitch',
    'RightHipYaw',
    'RightHipRoll',
    'RightHipPitch',
    'RightKneePitch',
    'RightAnklePitch',
]

DEFAULT_JOINT_ANGLES: Dict[str, float] = {
    'RightHipYaw': 0.0,
    'RightHipRoll': 0.0,
    'RightHipPitch': 0.4,
    'RightKneePitch': -0.8,
    'RightAnklePitch': 0.4,
    'LeftHipYaw': 0.0,
    'LeftHipRoll': 0.0,
    'LeftHipPitch': 0.4,
    'LeftKneePitch': -0.8,
    'LeftAnklePitch': 0.4,
}

LEG_IK_JOINTS: Dict[str, List[str]] = {
    'left': ['LeftHipPitch', 'LeftKneePitch', 'LeftAnklePitch'],
    'right': ['RightHipPitch', 'RightKneePitch', 'RightAnklePitch'],
}

FOOT_FRAME_NAMES: Dict[str, str] = {
    'left': 'LeftAnklePitch_Link',
    'right': 'RightAnklePitch_Link',
}


@dataclass
class Biped2Kinematics:
    model: pin.Model
    data: pin.Data
    joint_name_to_q_idx: Dict[str, int]
    joint_name_to_v_idx: Dict[str, int]
    left_foot_frame_id: int
    right_foot_frame_id: int

    @classmethod
    def from_urdf(cls, urdf_path: str, package_dirs: List[str] | None = None) -> 'Biped2Kinematics':
        del package_dirs  # Pinocchio 3.x expects preprocessed absolute mesh paths.
        model = pin.buildModelFromUrdf(urdf_path)
        data = model.createData()

        joint_name_to_q_idx: Dict[str, int] = {}
        joint_name_to_v_idx: Dict[str, int] = {}
        for joint_name in JOINT_NAMES:
            joint_id = model.getJointId(joint_name)
            if joint_id == 0:
                raise ValueError(f'Joint {joint_name} not found in URDF model.')
            joint_name_to_q_idx[joint_name] = model.joints[joint_id].idx_q
            joint_name_to_v_idx[joint_name] = model.joints[joint_id].idx_v

        left_foot_frame_id = model.getFrameId(FOOT_FRAME_NAMES['left'])
        right_foot_frame_id = model.getFrameId(FOOT_FRAME_NAMES['right'])
        if left_foot_frame_id == len(model.frames) or right_foot_frame_id == len(model.frames):
            raise ValueError('Foot frame not found in URDF model.')

        return cls(
            model=model,
            data=data,
            joint_name_to_q_idx=joint_name_to_q_idx,
            joint_name_to_v_idx=joint_name_to_v_idx,
            left_foot_frame_id=left_foot_frame_id,
            right_foot_frame_id=right_foot_frame_id,
        )

    @property
    def nq(self) -> int:
        return self.model.nq

    def default_configuration(self) -> np.ndarray:
        q = pin.neutral(self.model)
        for joint_name, angle in DEFAULT_JOINT_ANGLES.items():
            q[self.joint_name_to_q_idx[joint_name]] = angle
        return q

    def configuration_from_joint_dict(self, joint_positions: Dict[str, float]) -> np.ndarray:
        q = self.default_configuration()
        for joint_name, angle in joint_positions.items():
            if joint_name in self.joint_name_to_q_idx:
                q[self.joint_name_to_q_idx[joint_name]] = angle
        return q

    def joint_dict_from_configuration(self, q: np.ndarray) -> Dict[str, float]:
        return {
            joint_name: float(q[idx])
            for joint_name, idx in self.joint_name_to_q_idx.items()
        }

    def _foot_frame_id(self, leg: str) -> int:
        if leg == 'left':
            return self.left_foot_frame_id
        if leg == 'right':
            return self.right_foot_frame_id
        raise ValueError(f'Unknown leg: {leg}')

    def forward_kinematics(self, q: np.ndarray) -> Tuple[pin.SE3, pin.SE3]:
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        left_pose = self.data.oMf[self.left_foot_frame_id]
        right_pose = self.data.oMf[self.right_foot_frame_id]
        return left_pose, right_pose

    def foot_pose(self, q: np.ndarray, leg: str) -> pin.SE3:
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self._foot_frame_id(leg)]

    def _clip_configuration(self, q: np.ndarray) -> np.ndarray:
        q_clipped = q.copy()
        for joint_id in range(1, self.model.njoints):
            if self.model.nqs[joint_id] != 1:
                continue
            idx_q = self.model.joints[joint_id].idx_q
            lower = self.model.lowerPositionLimit[idx_q]
            upper = self.model.upperPositionLimit[idx_q]
            if np.isfinite(lower) and np.isfinite(upper):
                q_clipped[idx_q] = np.clip(q_clipped[idx_q], lower, upper)
        return q_clipped

    def inverse_kinematics(
        self,
        leg: str,
        target_pose: pin.SE3,
        q_init: np.ndarray,
        active_joint_names: Iterable[str] | None = None,
        max_iter: int = 80,
        tol: float = 1e-4,
        damping: float = 1e-4,
    ) -> Tuple[np.ndarray, bool, float]:
        active_joint_names = list(active_joint_names or LEG_IK_JOINTS[leg])
        active_q_indices = [self.joint_name_to_q_idx[name] for name in active_joint_names]
        active_v_indices = [self.joint_name_to_v_idx[name] for name in active_joint_names]
        foot_frame_id = self._foot_frame_id(leg)

        q = q_init.copy()
        success = False
        final_error = float('inf')

        for _ in range(max_iter):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            current_pose = self.data.oMf[foot_frame_id]
            error = pin.log6(current_pose.inverse() * target_pose).vector
            final_error = float(np.linalg.norm(error))
            if final_error < tol:
                success = True
                break

            jacobian = pin.computeFrameJacobian(
                self.model,
                self.data,
                q,
                foot_frame_id,
                pin.LOCAL_WORLD_ALIGNED,
            )
            jacobian_reduced = jacobian[:, active_v_indices]
            hessian = jacobian_reduced @ jacobian_reduced.T + damping * np.eye(6)
            delta_q_reduced = jacobian_reduced.T @ np.linalg.solve(hessian, error)

            for idx_q, delta in zip(active_q_indices, delta_q_reduced):
                q[idx_q] += delta
            q = self._clip_configuration(q)

        return q, success, final_error
