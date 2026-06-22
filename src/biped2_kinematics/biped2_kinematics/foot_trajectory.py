"""Foot-end trajectory generators for leg swing demos."""

from __future__ import annotations

import numpy as np
import pinocchio as pin


def cycloid_swing_offset(
    progress: float,
    step_length: float,
    lift_height: float,
) -> tuple[float, float]:
    """Cycloid-like swing displacement in the sagittal plane.

    Parameter ``progress`` in [0, 1] maps one closed swing cycle:
    - start/end at the stance foot position (dx=0, dz=0)
    - peak clearance ``lift_height`` at progress=0.5
    - peak forward excursion ``step_length`` at progress=0.5

    Uses a periodic cycloid profile (cos / sin^2) so the foot returns smoothly,
    mimicking human swing-phase foot paths.
    """
    t = float(np.clip(progress, 0.0, 1.0))
    theta = 2.0 * np.pi * t
    # Robot forward axis is -X in world frame; +X motion looked like a backward swing.
    dx = -0.5 * step_length * (1.0 - np.cos(theta))
    dz = lift_height * np.sin(0.5 * theta) ** 2
    return dx, dz


def cycloid_foot_target(
    flat_rotation: np.ndarray,
    stand_position: np.ndarray,
    progress: float,
    step_length: float,
    lift_height: float,
) -> pin.SE3:
    """Build a world-frame foot target pose along the cycloid swing path.

    ``flat_rotation`` keeps the sole parallel to the ground during swing.
    """
    dx, dz = cycloid_swing_offset(progress, step_length, lift_height)
    target_position = stand_position + np.array([dx, 0.0, dz])
    return pin.SE3(flat_rotation.copy(), target_position)
