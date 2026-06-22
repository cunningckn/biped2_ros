"""Alternating foot-lift demo with half-cycle offset between legs."""

from __future__ import annotations

import os

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from biped2_kinematics.foot_trajectory import cycloid_foot_target, cycloid_swing_offset
from biped2_kinematics.pinocchio_leg_ik import Biped2Kinematics, JOINT_NAMES, LEG_IK_JOINTS
from biped2_kinematics.urdf_utils import xacro_to_pinocchio_urdf

TRAJECTORY_FRAME_ID = 'world'
IK_ACCEPT_ERROR = 0.04
LEG_PHASE_OFFSET = 0.5


class FootLiftDemo(Node):
    def __init__(self) -> None:
        super().__init__('foot_lift_demo')
        self.declare_parameter('lift_height', 0.10)
        self.declare_parameter('step_length', 0.08)
        self.declare_parameter('cycle_duration', 2.0)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('trajectory_frame_id', 'world')
        self.declare_parameter('leg_phase_offset', LEG_PHASE_OFFSET)
        self.declare_parameter('stand_foot_z', 0.0)

        self.lift_height = self.get_parameter('lift_height').get_parameter_value().double_value
        self.step_length = self.get_parameter('step_length').get_parameter_value().double_value
        self.cycle_duration = self.get_parameter('cycle_duration').get_parameter_value().double_value
        self.control_rate = self.get_parameter('control_rate').get_parameter_value().double_value
        self.leg_phase_offset = (
            self.get_parameter('leg_phase_offset').get_parameter_value().double_value
        )
        self.stand_foot_z = self.get_parameter('stand_foot_z').get_parameter_value().double_value
        self.trajectory_frame_id = (
            self.get_parameter('trajectory_frame_id').get_parameter_value().string_value
            or TRAJECTORY_FRAME_ID
        )

        urdf_path = self._resolve_urdf_path()
        self.kinematics = Biped2Kinematics.from_urdf(urdf_path)
        self.stand_q = self.kinematics.compute_symmetric_stand_configuration(
            foot_z=self.stand_foot_z
        )
        self.q = self.stand_q.copy()
        self._cache_stand_foot_poses()
        self.joint_state_ready = False
        self.demo_start = self.get_clock().now()

        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10,
        )
        self.timer = self.create_timer(1.0 / self.control_rate, self.control_step)
        self.get_logger().info(
            'Foot lift demo started with half-cycle alternating legs. '
            f'lift_height={self.lift_height:.2f} m, step_length={self.step_length:.2f} m, '
            f'leg_phase_offset={self.leg_phase_offset:.2f}, '
            f'stand_foot_z={self.stand_foot_z:.2f} m, '
            f'frame_id={self.trajectory_frame_id!r}'
        )

    def _resolve_urdf_path(self) -> str:
        share_dir = get_package_share_directory('biped2_description')
        xacro_path = os.path.join(share_dir, 'urdf', 'biped2.urdf.xacro')
        return xacro_to_pinocchio_urdf(
            xacro_path,
            share_dir,
            os.path.join('/tmp', 'biped2_demo.urdf'),
        )

    def _cache_stand_foot_poses(self) -> None:
        left_pose, right_pose = self.kinematics.forward_kinematics(self.stand_q)
        self.stand_foot_position = {
            'left': np.array(left_pose.translation.copy()),
            'right': np.array(right_pose.translation.copy()),
        }
        self.flat_foot_rotation = self.kinematics.compute_flat_foot_rotations(self.stand_q)

    def joint_state_callback(self, msg: JointState) -> None:
        joint_positions = dict(zip(msg.name, msg.position))
        if all(name in joint_positions for name in JOINT_NAMES):
            self.joint_state_ready = True

    def _global_phase(self) -> float:
        elapsed = (self.get_clock().now() - self.demo_start).nanoseconds * 1e-9
        return (elapsed / self.cycle_duration) % 1.0

    def _leg_progress(self, leg: str, global_phase: float) -> float:
        if leg == 'right':
            return global_phase
        return (global_phase + self.leg_phase_offset) % 1.0

    def _compute_right_leg_q(self, progress: float) -> np.ndarray:
        target = cycloid_foot_target(
            self.flat_foot_rotation['right'],
            self.stand_foot_position['right'],
            progress,
            self.step_length,
            self.lift_height,
        )
        q_solution, success, error = self.kinematics.inverse_kinematics(
            leg='right',
            target_pose=target,
            q_init=self.stand_q,
            active_joint_names=LEG_IK_JOINTS['right'],
            position_only=True,
        )
        if not success and error >= IK_ACCEPT_ERROR:
            self.get_logger().warn(
                f'Cycloid IK for right leg did not fully converge '
                f'(progress={progress:.2f}, error={error:.4f}). Using best effort.'
            )
        q_solution = self.kinematics.lock_leg_yaw_roll(q_solution, 'right', self.stand_q)
        return self.kinematics.adjust_ankle_for_flat_sole(q_solution, 'right')

    def _compute_left_leg_q(self, progress: float) -> np.ndarray:
        # Mirror the right-leg cycloid solution to avoid left-leg HipYaw/Roll coupling.
        target = cycloid_foot_target(
            self.flat_foot_rotation['right'],
            self.stand_foot_position['right'],
            progress,
            self.step_length,
            self.lift_height,
        )
        q_right, success, error = self.kinematics.inverse_kinematics(
            leg='right',
            target_pose=target,
            q_init=self.stand_q,
            active_joint_names=LEG_IK_JOINTS['right'],
            position_only=True,
        )
        if not success and error >= IK_ACCEPT_ERROR:
            self.get_logger().warn(
                f'Right reference IK for left swing did not fully converge '
                f'(progress={progress:.2f}, error={error:.4f}). Using best effort mirror.'
            )
        q_right = self.kinematics.adjust_ankle_for_flat_sole(q_right, 'right')
        q_left = self.kinematics.mirror_right_swing_to_left(self.stand_q, q_right)
        q_left = self.kinematics.lock_leg_yaw_roll(q_left, 'left', self.stand_q)
        return self.kinematics.adjust_ankle_for_flat_sole(q_left, 'left')

    def _merge_leg_configurations(self, q_right: np.ndarray, q_left: np.ndarray) -> None:
        self.q = self.stand_q.copy()
        for joint_name in LEG_IK_JOINTS['right']:
            idx = self.kinematics.joint_name_to_q_idx[joint_name]
            self.q[idx] = q_right[idx]
        for joint_name in LEG_IK_JOINTS['left']:
            idx = self.kinematics.joint_name_to_q_idx[joint_name]
            self.q[idx] = q_left[idx]

    def _publish_trajectory(self) -> None:
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.trajectory_frame_id or TRAJECTORY_FRAME_ID
        msg.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = [float(self.q[self.kinematics.joint_name_to_q_idx[name]]) for name in JOINT_NAMES]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(1e8)
        msg.points = [point]
        self.traj_pub.publish(msg)

    def control_step(self) -> None:
        if not self.joint_state_ready:
            self.q = self.stand_q.copy()
            self._publish_trajectory()
            return

        global_phase = self._global_phase()
        progress_right = self._leg_progress('right', global_phase)
        progress_left = self._leg_progress('left', global_phase)

        q_right = self._compute_right_leg_q(progress_right)
        q_left = self._compute_left_leg_q(progress_left)
        self._merge_leg_configurations(q_right, q_left)
        self._publish_trajectory()

        tick = int((self.get_clock().now() - self.demo_start).nanoseconds * 1e-9)
        if tick > 0 and tick % max(int(self.control_rate), 1) == 0:
            right_dx, right_dz = cycloid_swing_offset(
                progress_right, self.step_length, self.lift_height
            )
            left_dx, left_dz = cycloid_swing_offset(
                progress_left, self.step_length, self.lift_height
            )
            self.get_logger().info(
                'Alternating swing '
                f'phase={global_phase:.2f}, '
                f'right(p={progress_right:.2f}, dz={right_dz:.3f}), '
                f'left(p={progress_left:.2f}, dz={left_dz:.3f})'
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FootLiftDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
