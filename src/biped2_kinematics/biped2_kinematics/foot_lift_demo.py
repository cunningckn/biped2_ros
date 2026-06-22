"""Alternating foot-lift demo using symmetric standing pose and joint-space swing."""

from __future__ import annotations

import os
from enum import Enum, auto

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from biped2_kinematics.pinocchio_leg_ik import (
    Biped2Kinematics,
    JOINT_NAMES,
    LIFT_REFERENCE_HEIGHT,
)
from biped2_kinematics.urdf_utils import xacro_to_pinocchio_urdf

TRAJECTORY_FRAME_ID = 'world'


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
        self.declare_parameter('lift_height', 0.10)
        self.declare_parameter('cycle_duration', 2.0)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('trajectory_frame_id', 'world')

        self.lift_height = self.get_parameter('lift_height').get_parameter_value().double_value
        self.cycle_duration = self.get_parameter('cycle_duration').get_parameter_value().double_value
        self.control_rate = self.get_parameter('control_rate').get_parameter_value().double_value
        self.trajectory_frame_id = (
            self.get_parameter('trajectory_frame_id').get_parameter_value().string_value
            or TRAJECTORY_FRAME_ID
        )

        urdf_path = self._resolve_urdf_path()
        self.kinematics = Biped2Kinematics.from_urdf(urdf_path)
        self.stand_q = self.kinematics.compute_symmetric_stand_configuration()
        self.q = self.stand_q.copy()
        self.joint_state_ready = False

        self.sequence_index = 0
        self.phase_start = self.get_clock().now()

        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10,
        )
        self.timer = self.create_timer(1.0 / self.control_rate, self.control_step)
        self.get_logger().info(
            f'Foot lift demo started. symmetric stand, lift_height={self.lift_height:.2f} m, '
            f'trajectory frame_id={self.trajectory_frame_id!r}'
        )

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
            self.joint_state_ready = True

    def _phase_progress(self) -> float:
        elapsed = (self.get_clock().now() - self.phase_start).nanoseconds * 1e-9
        return np.clip(elapsed / self.cycle_duration, 0.0, 1.0)

    def _lift_scale(self) -> float:
        progress = self._phase_progress()
        lift_fraction = np.sin(np.pi * progress)
        return (self.lift_height / LIFT_REFERENCE_HEIGHT) * lift_fraction

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

        state = self.SEQUENCE[self.sequence_index]
        progress = self._phase_progress()
        lift_scale = 0.0

        self.q = self.stand_q.copy()
        if state == DemoState.LIFT_RIGHT:
            lift_scale = self._lift_scale()
            self.q = self.kinematics.apply_lift_joint_deltas(self.q, 'right', lift_scale)
        elif state == DemoState.LIFT_LEFT:
            lift_scale = self._lift_scale()
            self.q = self.kinematics.apply_lift_joint_deltas(self.q, 'left', lift_scale)

        self._publish_trajectory()

        if progress >= 1.0:
            self.sequence_index = (self.sequence_index + 1) % len(self.SEQUENCE)
            self.phase_start = self.get_clock().now()
            next_state = self.SEQUENCE[self.sequence_index]
            self.get_logger().info(f'Phase complete. Next state={next_state.name}')
        elif int(progress * self.control_rate) % max(int(self.control_rate), 1) == 0:
            self.get_logger().info(f'State={state.name}, lift_scale={lift_scale:.3f}')


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
