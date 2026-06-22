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

STAND_FOOT_Z = 0.0
# Joint deltas at full lift (~12 cm foot clearance); scaled by lift_height / LIFT_REFERENCE_HEIGHT.
LIFT_REFERENCE_HEIGHT = 0.12
LIFT_JOINT_DELTAS: Dict[str, Dict[str, float]] = {
    'left': {
        'LeftHipPitch': -0.8,
        'LeftKneePitch': -0.5,
        'LeftAnklePitch': -0.5,
    },
    'right': {
        'RightHipPitch': 0.3086,
        'RightKneePitch': -0.773,
        'RightAnklePitch': 0.0,
    },
}

# Symmetric standing pose: both feet at the same height (foot_z ~= STAND_FOOT_Z).
DEFAULT_JOINT_ANGLES: Dict[str, float] = {
    'LeftHipYaw': 0.0,
    'LeftHipRoll': 0.0,
    'LeftHipPitch': 0.0155,
    'LeftKneePitch': -0.0268,
    'LeftAnklePitch': 0.0,
    'RightHipYaw': 0.0,
    'RightHipRoll': 0.0,
    'RightHipPitch': -0.0155,
    'RightKneePitch': 0.0268,
    'RightAnklePitch': 0.0,
}

LEG_IK_JOINTS: Dict[str, List[str]] = {
    'left': ['LeftHipPitch', 'LeftKneePitch', 'LeftAnklePitch'],
    'right': ['RightHipPitch', 'RightKneePitch', 'RightAnklePitch'],
}

SWING_MIRROR_JOINTS: List[Tuple[str, str]] = [
    ('RightHipPitch', 'LeftHipPitch'),
    ('RightKneePitch', 'LeftKneePitch'),
    ('RightAnklePitch', 'LeftAnklePitch'),
]

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

    def compute_symmetric_stand_configuration(self, foot_z: float = STAND_FOOT_Z) -> np.ndarray:
        q = self.default_configuration()
        flat_rotations = self.compute_flat_foot_rotations(q)
        left_pose, right_pose = self.forward_kinematics(q)
        foot_y = 0.5 * (abs(left_pose.translation[1]) + abs(right_pose.translation[1]))
        foot_x = 0.5 * (left_pose.translation[0] + right_pose.translation[0])

        for _ in range(30):
            for leg, y_sign in (('right', 1.0), ('left', -1.0)):
                target = pin.SE3(
                    flat_rotations[leg],
                    np.array([foot_x, y_sign * foot_y, foot_z]),
                )
                q, _, _ = self.inverse_kinematics(
                    leg,
                    target,
                    q,
                    position_only=False,
                )
        return q

    def compute_flat_foot_rotations(self, q: np.ndarray | None = None) -> Dict[str, np.ndarray]:
        q_ref = self.default_configuration() if q is None else q.copy()
        q_ref[self.joint_name_to_q_idx['LeftAnklePitch']] = 0.0
        q_ref[self.joint_name_to_q_idx['RightAnklePitch']] = 0.0
        return {
            'left': self.foot_pose(q_ref, 'left').rotation.copy(),
            'right': self.foot_pose(q_ref, 'right').rotation.copy(),
        }

    def flat_sole_score(self, leg: str, rotation: np.ndarray) -> float:
        world_up = np.array([0.0, 0.0, 1.0])
        if leg == 'left':
            return float((-rotation[:, 1]) @ world_up)
        return float(rotation[:, 1] @ world_up)

    def adjust_ankle_for_flat_sole(self, q: np.ndarray, leg: str) -> np.ndarray:
        ankle_name = 'LeftAnklePitch' if leg == 'left' else 'RightAnklePitch'
        idx_q = self.joint_name_to_q_idx[ankle_name]
        lower = self.model.lowerPositionLimit[idx_q]
        upper = self.model.upperPositionLimit[idx_q]

        best_q = q.copy()
        best_score = self.flat_sole_score(leg, self.foot_pose(q, leg).rotation)
        for ankle_angle in np.linspace(lower, upper, 81):
            candidate = q.copy()
            candidate[idx_q] = ankle_angle
            score = self.flat_sole_score(leg, self.foot_pose(candidate, leg).rotation)
            if score > best_score:
                best_score = score
                best_q = candidate
        return best_q

    def apply_lift_joint_deltas(self, q: np.ndarray, leg: str, scale: float) -> np.ndarray:
        q_lift = q.copy()
        for joint_name, delta in LIFT_JOINT_DELTAS[leg].items():
            q_lift[self.joint_name_to_q_idx[joint_name]] += delta * scale
        return self._clip_configuration(q_lift)

    def mirror_right_swing_to_left(self, stand_q: np.ndarray, right_swing_q: np.ndarray) -> np.ndarray:
        """Map a right-leg swing configuration to the left leg in the sagittal plane."""
        q_left = stand_q.copy()
        for right_name, left_name in SWING_MIRROR_JOINTS:
            right_idx = self.joint_name_to_q_idx[right_name]
            left_idx = self.joint_name_to_q_idx[left_name]
            delta = right_swing_q[right_idx] - stand_q[right_idx]
            q_left[left_idx] = stand_q[left_idx] - delta
        return self._clip_configuration(q_left)

    def lock_leg_yaw_roll(self, q: np.ndarray, leg: str, stand_q: np.ndarray) -> np.ndarray:
        q_locked = q.copy()
        prefix = 'Left' if leg == 'left' else 'Right'
        for joint_name in (f'{prefix}HipYaw', f'{prefix}HipRoll'):
            idx = self.joint_name_to_q_idx[joint_name]
            q_locked[idx] = stand_q[idx]
        return q_locked

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
        max_iter: int = 120,
        tol: float = 5e-4,
        damping: float = 1e-3,
        position_only: bool = False,
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

            jacobian = pin.computeFrameJacobian(
                self.model,
                self.data,
                q,
                foot_frame_id,
                pin.LOCAL_WORLD_ALIGNED,
            )
            jacobian_reduced = jacobian[:, active_v_indices]

            if position_only:
                error = target_pose.translation - current_pose.translation
                jacobian_reduced = jacobian_reduced[:3, :]
                hessian = jacobian_reduced @ jacobian_reduced.T + damping * np.eye(3)
            else:
                error = pin.log6(current_pose.inverse() * target_pose).vector
                hessian = jacobian_reduced @ jacobian_reduced.T + damping * np.eye(6)

            final_error = float(np.linalg.norm(error))
            if final_error < tol:
                success = True
                break

            delta_q_reduced = jacobian_reduced.T @ np.linalg.solve(hessian, error)

            for idx_q, delta in zip(active_q_indices, delta_q_reduced):
                q[idx_q] += delta
            q = self._clip_configuration(q)

        return q, success, final_error
