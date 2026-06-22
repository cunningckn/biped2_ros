"""Publish biped2 foot poses computed with Pinocchio forward kinematics."""

from __future__ import annotations

import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState

from biped2_kinematics.pinocchio_leg_ik import Biped2Kinematics
from biped2_kinematics.urdf_utils import xacro_to_pinocchio_urdf


def se3_to_pose_stamped(pose, frame_id: str, stamp) -> PoseStamped:
    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.pose.position.x = float(pose.translation[0])
    msg.pose.position.y = float(pose.translation[1])
    msg.pose.position.z = float(pose.translation[2])
    quat = pinocchio_quat_from_rotation(pose.rotation)
    msg.pose.orientation.x = float(quat[0])
    msg.pose.orientation.y = float(quat[1])
    msg.pose.orientation.z = float(quat[2])
    msg.pose.orientation.w = float(quat[3])
    return msg


def pinocchio_quat_from_rotation(rotation) -> tuple[float, float, float, float]:
    import pinocchio as pin

    quat = pin.Quaternion(rotation)
    return quat.coeffs()[0], quat.coeffs()[1], quat.coeffs()[2], quat.coeffs()[3]


class FootFkPublisher(Node):
    def __init__(self) -> None:
        super().__init__('foot_fk_publisher')
        self.declare_parameter('frame_id', 'world')

        urdf_path = self._resolve_urdf_path()
        self.kinematics = Biped2Kinematics.from_urdf(urdf_path)
        self.q = self.kinematics.default_configuration()
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.left_pub = self.create_publisher(PoseStamped, '/biped2/left_foot_pose', 10)
        self.right_pub = self.create_publisher(PoseStamped, '/biped2/right_foot_pose', 10)
        self.timer = self.create_timer(0.05, self.publish_poses)
        self.get_logger().info(f'Foot FK publisher ready. URDF: {urdf_path}')

    def _resolve_urdf_path(self) -> str:
        share_dir = get_package_share_directory('biped2_description')
        xacro_path = os.path.join(share_dir, 'urdf', 'biped2.urdf.xacro')
        return xacro_to_pinocchio_urdf(
            xacro_path,
            share_dir,
            os.path.join('/tmp', 'biped2_fk.urdf'),
        )

    def joint_state_callback(self, msg: JointState) -> None:
        joint_positions = dict(zip(msg.name, msg.position))
        self.q = self.kinematics.configuration_from_joint_dict(joint_positions)

    def publish_poses(self) -> None:
        left_pose, right_pose = self.kinematics.forward_kinematics(self.q)
        stamp = self.get_clock().now().to_msg()
        self.left_pub.publish(se3_to_pose_stamped(left_pose, self.frame_id, stamp))
        self.right_pub.publish(se3_to_pose_stamped(right_pose, self.frame_id, stamp))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FootFkPublisher()
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
