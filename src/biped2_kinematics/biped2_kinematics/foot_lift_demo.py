"""Alternating foot-lift demo using simplified Pinocchio inverse kinematics."""

from __future__ import annotations

import os
from enum import Enum, auto

import numpy as np
import pinocchio as pin
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from biped2_kinematics.pinocchio_leg_ik import (
    Biped2Kinematics,
    DEFAULT_JOINT_ANGLES,
    JOINT_NAMES,
    LEG_IK_JOINTS,
)
from biped2_kinematics.urdf_utils import xacro_to_pinocchio_urdf


class DemoState(Enum):
    STAND = auto()
    LIFT_RIGHT = auto()
    LIFT_LEFT = auto()


class FootLiftDemo(Node):
    SEQUENCE = [
        DemoState.STAND,
        DemoState.LIFT_RIGHT,
        DemoState.STAND,
        DemoState.LIFT_LEFT,
    ]

    def __init__(self) -> None:
        super().__init__('foot_lift_demo')
        self.declare_parameter('lift_height', 0.06)
        self.declare_parameter('cycle_duration', 2.0)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('trajectory_frame_id', 'base_link')

        self.lift_height = self.get_parameter('lift_height').get_parameter_value().double_value
        self.cycle_duration = self.get_parameter('cycle_duration').get_parameter_value().double_value
        self.control_rate = self.get_parameter('control_rate').get_parameter_value().double_value
        self.trajectory_frame_id = (
            self.get_parameter('trajectory_frame_id').get_parameter_value().string_value
        )

        urdf_path = self._resolve_urdf_path()
        self.kinematics = Biped2Kinematics.from_urdf(urdf_path)
        self.q = self.kinematics.default_configuration()
        self.joint_state_ready = False

        self.sequence_index = 0
        self.phase_start = self.get_clock().now()
        self.stand_left_pose = None
        self.stand_right_pose = None

        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10,
        )
        self.timer = self.create_timer(1.0 / self.control_rate, self.control_step)
        self.get_logger().info('Foot lift demo started.')

    def _resolve_urdf_path(self) -> str:
        share_dir = get_package_share_directory('biped2_description')
        xacro_path = os.path.join(share_dir, 'urdf', 'biped2.urdf.xacro')
        return xacro_to_pinocchio_urdf(
            xacro_path,
            share_dir,
            os.path.join('/tmp', 'biped2_demo.urdf'),
        )

    def joint_state_callback(self, msg: JointState) -> None:
        joint_positions = dict(zip(msg.name, msg.position))
        if all(name in joint_positions for name in JOINT_NAMES):
            self.q = self.kinematics.configuration_from_joint_dict(joint_positions)
            self.joint_state_ready = True

    def _capture_stand_poses(self) -> None:
        left_pose, right_pose = self.kinematics.forward_kinematics(self.q)
        self.stand_left_pose = left_pose.copy()
        self.stand_right_pose = right_pose.copy()

    def _phase_progress(self) -> float:
        elapsed = (self.get_clock().now() - self.phase_start).nanoseconds * 1e-9
        return np.clip(elapsed / self.cycle_duration, 0.0, 1.0)

    def _lift_offset(self) -> float:
        progress = self._phase_progress()
        return self.lift_height * np.sin(np.pi * progress)

    def _target_pose(self, stand_pose: pin.SE3, lift: float) -> pin.SE3:
        target = stand_pose.copy()
        target.translation = stand_pose.translation + np.array([0.0, 0.0, lift])
        return target

    def _solve_leg(self, leg: str, target_pose: pin.SE3) -> None:
        q_solution, success, error = self.kinematics.inverse_kinematics(
            leg=leg,
            target_pose=target_pose,
            q_init=self.q,
            active_joint_names=LEG_IK_JOINTS[leg],
        )
        if success or error < 0.05:
            self.q = q_solution
        else:
            self.get_logger().warn(
                f'IK for {leg} leg did not fully converge (error={error:.4f}). Keeping previous solution.'
            )

    def _apply_non_ik_defaults(self) -> None:
        for joint_name, angle in DEFAULT_JOINT_ANGLES.items():
            if joint_name not in LEG_IK_JOINTS['left'] and joint_name not in LEG_IK_JOINTS['right']:
                self.q[self.kinematics.joint_name_to_q_idx[joint_name]] = angle

    def _publish_trajectory(self) -> None:
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.trajectory_frame_id
        msg.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = [float(self.q[self.kinematics.joint_name_to_q_idx[name]]) for name in JOINT_NAMES]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(1e8)
        msg.points = [point]
        self.traj_pub.publish(msg)

    def control_step(self) -> None:
        if not self.joint_state_ready:
            return

        if self.stand_left_pose is None or self.stand_right_pose is None:
            self._capture_stand_poses()

        state = self.SEQUENCE[self.sequence_index]
        progress = self._phase_progress()
        lift = 0.0

        if state == DemoState.LIFT_RIGHT:
            lift = self._lift_offset()
            target = self._target_pose(self.stand_right_pose, lift)
            self._apply_non_ik_defaults()
            self._solve_leg('right', target)
        elif state == DemoState.LIFT_LEFT:
            lift = self._lift_offset()
            target = self._target_pose(self.stand_left_pose, lift)
            self._apply_non_ik_defaults()
            self._solve_leg('left', target)
        else:
            self._apply_non_ik_defaults()
            for joint_name in JOINT_NAMES:
                self.q[self.kinematics.joint_name_to_q_idx[joint_name]] = DEFAULT_JOINT_ANGLES[joint_name]

        self._publish_trajectory()

        if progress >= 1.0:
            self.sequence_index = (self.sequence_index + 1) % len(self.SEQUENCE)
            self.phase_start = self.get_clock().now()
            self._capture_stand_poses()
            next_state = self.SEQUENCE[self.sequence_index]
            self.get_logger().info(f'Phase complete. Next state={next_state.name}')
        elif int(progress * self.control_rate) % max(int(self.control_rate), 1) == 0:
            self.get_logger().info(f'State={state.name}, lift={lift:.3f} m')


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
