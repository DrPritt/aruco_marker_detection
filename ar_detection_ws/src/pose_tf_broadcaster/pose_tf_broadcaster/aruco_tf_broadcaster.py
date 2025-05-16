#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from aruco_msgs.msg import MarkerArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class ArucoTfBroadcaster(Node):
    def __init__(self):
        super().__init__('aruco_tf_broadcaster')
        self.declare_parameter('aruco_topic', '/marker_publisher/markers')
        self.declare_parameter('camera_frame', 'optitrack_camera')
        self.declare_parameter('child_frame_prefix', 'marker_aruco_cam_')

        topic    = self.get_parameter('aruco_topic').value
        self.cam = self.get_parameter('camera_frame').value
        prefix   = self.get_parameter('child_frame_prefix').value

        self.broadcaster = TransformBroadcaster(self)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        self.create_subscription(
            MarkerArray,
            topic,
            lambda msg: self.on_markers(msg, prefix),
            qos)

        self.get_logger().info(f"Broadcasting ArUco from {topic} under {self.cam}")

    def on_markers(self, msg, prefix):
        for m in msg.markers:
            tf = TransformStamped()
            tf.header.stamp = m.header.stamp
            tf.header.frame_id = self.cam
            tf.child_frame_id = prefix + str(m.id)
            tf.transform.translation.x = m.pose.pose.position.x
            tf.transform.translation.y = m.pose.pose.position.y
            tf.transform.translation.z = m.pose.pose.position.z
            tf.transform.rotation = m.pose.pose.orientation
            self.broadcaster.sendTransform(tf)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoTfBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

