#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class OptiTrackTfBroadcaster(Node):
    def __init__(self):
        super().__init__('optitrack_tf_broadcaster')
        # parameters
        # self.declare_parameter("use_sim_time", True)
        self.declare_parameter('pose_topic', '/vrpn_mocap/USB_cam/pose')
        self.declare_parameter('parent_frame', 'world')
        self.declare_parameter('child_frame', 'optitrack_camera')

        topic       = self.get_parameter('pose_topic').value
        self.parent = self.get_parameter('parent_frame').value
        self.child  = self.get_parameter('child_frame').value

        self.broadcaster = TransformBroadcaster(self)

        # QoS matching VRPN’s best-effort publisher
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(
            PoseStamped,
            topic,
            self.pose_callback,
            qos
        )
        self.get_logger().info(
            f"Broadcasting {topic} → {self.parent} → {self.child} with BEST_EFFORT"
        )

    def pose_callback(self, msg: PoseStamped):
        tf = TransformStamped()
        tf.header.stamp    = msg.header.stamp
        tf.header.frame_id = self.parent
        tf.child_frame_id  = self.child
        tf.transform.translation.x = msg.pose.position.x
        tf.transform.translation.y = msg.pose.position.y
        tf.transform.translation.z = msg.pose.position.z
        tf.transform.rotation     = msg.pose.orientation
        self.broadcaster.sendTransform(tf)

def main(args=None):
    rclpy.init(args=args)
    node = OptiTrackTfBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
