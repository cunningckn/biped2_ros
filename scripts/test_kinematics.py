#!/usr/bin/env python3
"""Offline kinematics smoke test (no ROS runtime required)."""

from __future__ import annotations

import os
import sys

import numpy as np
import pinocchio as pin

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'biped2_kinematics'))

from biped2_kinematics.pinocchio_leg_ik import Biped2Kinematics  # noqa: E402


def main() -> int:
    urdf_path = '/tmp/biped2_test.urdf'
    if not os.path.exists(urdf_path):
        print('Missing /tmp/biped2_test.urdf. Run xacro first.')
        return 1

    share_dir = '/home/ws0/Workspace/ckn/biped2_ros/install/biped2_description/share/biped2_description'
    kin = Biped2Kinematics.from_urdf(urdf_path, package_dirs=[share_dir])
    q = kin.default_configuration()
    left_pose, right_pose = kin.forward_kinematics(q)
    print('Default FK left foot z:', left_pose.translation[2])
    print('Default FK right foot z:', right_pose.translation[2])

    target = right_pose.copy()
    target.translation = right_pose.translation + np.array([0.0, 0.0, 0.06])
    q_ik, success, error = kin.inverse_kinematics('right', target, q)
    _, right_after = kin.forward_kinematics(q_ik)
    print('Right leg IK success:', success, 'error:', error)
    print('Right foot z after IK:', right_after.translation[2])
    return 0 if success else 2


if __name__ == '__main__':
    raise SystemExit(main())
