#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from aruco_msgs.msg import MarkerArray
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

qos_profile = QoSProfile(depth=10)
qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

def quat_multiply(q1, q2):
    # q = [x, y, z, w]
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    return np.array([x, y, z, w])


def quat_rotate_vector(q, v):
    # Rotate vector v by quaternion q
    qx, qy, qz, qw = q
    vx, vy, vz = v

    # Quaternion * vector
    ix = qw * vx + qy * vz - qz * vy
    iy = qw * vy + qz * vx - qx * vz
    iz = qw * vz + qx * vy - qy * vx
    iw = -qx * vx - qy * vy - qz * vz

    # Result * quaternion conjugate
    rx = ix * qw + iw * (-qx) + iy * (-qz) - iz * (-qy)
    ry = iy * qw + iw * (-qy) + iz * (-qx) - ix * (-qz)
    rz = iz * qw + iw * (-qz) + ix * (-qy) - iy * (-qx)

    return np.array([rx, ry, rz])


class PoseInWorldNode(Node):
    def __init__(self):
        super().__init__("pose_in_world")

        self.camera_pose = None

        self.create_subscription(
            PoseStamped, "/vrpn_mocap/USB_cam/pose", self.camera_pose_cb, qos_profile
        )

        self.get_logger().info(
            "Subscribed to /vrpn_mocap/USB_cam/pose with PoseStamped"
        )

        self.create_subscription(
            MarkerArray, "/marker_publisher/markers", self.marker_callback, 10
        )

        self.marker_pub = self.create_publisher(PoseStamped, "marker_in_world_pose", 10)

    def camera_pose_cb(self, msg):
        self.camera_pose = msg

    def marker_callback(self, msg: MarkerArray):
        if not self.camera_pose:
            self.get_logger().warn("No camera pose received yet.")
            return

        for marker in msg.markers:
            # Camera pose in world
            cam_p = np.array(
                [
                    self.camera_pose.pose.position.x,
                    self.camera_pose.pose.position.y,
                    self.camera_pose.pose.position.z,
                ]
            )
            cam_q = np.array(
                [
                    self.camera_pose.pose.orientation.x,
                    self.camera_pose.pose.orientation.y,
                    self.camera_pose.pose.orientation.z,
                    self.camera_pose.pose.orientation.w,
                ]
            )

            # Marker pose in camera
            m_p = np.array(
                [
                    marker.pose.pose.position.x,
                    marker.pose.pose.position.y,
                    marker.pose.pose.position.z,
                ]
            )
            m_q = np.array(
                [
                    marker.pose.pose.orientation.x,
                    marker.pose.pose.orientation.y,
                    marker.pose.pose.orientation.z,
                    marker.pose.pose.orientation.w,
                ]
            )

            # Rotate marker position by camera orientation
            rotated_pos = quat_rotate_vector(cam_q, m_p)

            # Translate by camera position
            world_pos = cam_p + rotated_pos

            # Compose orientations: world_q = cam_q * marker_q
            world_q = quat_multiply(cam_q, m_q)

            # Publish pose
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "world"
            pose_msg.pose.position.x = float(world_pos[0])
            pose_msg.pose.position.y = float(world_pos[1])
            pose_msg.pose.position.z = float(world_pos[2])
            pose_msg.pose.orientation.x = float(world_q[0])
            pose_msg.pose.orientation.y = float(world_q[1])
            pose_msg.pose.orientation.z = float(world_q[2])
            pose_msg.pose.orientation.w = float(world_q[3])

            self.marker_pub.publish(pose_msg)
            # self.get_logger().info(f"Published marker {marker.id} pose in world frame.")


def main(args=None):
    rclpy.init(args=args)
    node = PoseInWorldNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
