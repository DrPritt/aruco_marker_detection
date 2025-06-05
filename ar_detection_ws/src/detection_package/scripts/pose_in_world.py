#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from aruco_msgs.msg import MarkerArray
import numpy as np
import tf2_ros
import tf_transformations
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from tf2_ros import TransformException

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

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Parameters
        self.declare_parameter("camera_frame", "camera")
        self.declare_parameter("world_frame", "world")

        self.camera_frame = (
            self.get_parameter("camera_frame").get_parameter_value().string_value
        )
        self.world_frame = (
            self.get_parameter("world_frame").get_parameter_value().string_value
        )

        # Subscribe to MarkerArray (marker poses in camera frame)
        self.create_subscription(
            MarkerArray, "/marker_publisher/markers", self.marker_callback, qos_profile
        )

        # Publisher for marker pose in world frame
        self.marker_pub = self.create_publisher(PoseStamped, "marker_in_world_pose", 10)

        self.get_logger().info(
            f"Listening to TF for '{self.world_frame} → {self.camera_frame}' and to '/marker_publisher/markers'."
        )

    def marker_callback(self, msg: MarkerArray):
        # For each detected marker in camera frame, compute pose in world frame
        for marker in msg.markers:
            try:
                # Lookup latest transform: world → camera
                tf_cam = self.tf_buffer.lookup_transform(
                    self.world_frame, self.camera_frame, rclpy.time.Time()
                )
            except TransformException as e:
                self.get_logger().warn(
                    f"Could not lookup transform {self.world_frame} → {self.camera_frame}: {e}"
                )
                return

            # Extract camera position & orientation from TF
            cam_trans = tf_cam.transform.translation
            cam_rot = tf_cam.transform.rotation
            cam_p = np.array([cam_trans.x, cam_trans.y, cam_trans.z])
            cam_q = np.array([cam_rot.x, cam_rot.y, cam_rot.z, cam_rot.w])

            # Marker pose in camera frame from MarkerArray
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

            # Rotate marker position into world frame
            rotated_pos = quat_rotate_vector(cam_q, m_p)
            world_pos = cam_p + rotated_pos

            # Compose orientations: world_q = cam_q * marker_q
            world_q = quat_multiply(cam_q, m_q)

            # Publish PoseStamped in world frame
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = self.world_frame
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
