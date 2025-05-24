#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import tf2_ros
import math


class LiveErrorPlotNode(Node):
    def __init__(self):
        super().__init__("live_error_plot")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub_x = self.create_publisher(Float64, "/error/x", 10)
        self.pub_y = self.create_publisher(Float64, "/error/y", 10)
        self.pub_z = self.create_publisher(Float64, "/error/z", 10)
        self.pub_dist = self.create_publisher(Float64, "/error/distance", 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            now = rclpy.time.Time()
            tf1 = self.tf_buffer.lookup_transform("world", "ar_marker_optitrack", now)
            tf2 = self.tf_buffer.lookup_transform("world", "ar_marker_camera77", now)

            dx = tf1.transform.translation.x - tf2.transform.translation.x
            dy = tf1.transform.translation.y - tf2.transform.translation.y
            dz = tf1.transform.translation.z - tf2.transform.translation.z
            dist = math.sqrt(dx**2 + dy**2 + dz**2)

            self.pub_x.publish(Float64(data=dx))
            self.pub_y.publish(Float64(data=dy))
            self.pub_z.publish(Float64(data=dz))
            self.pub_dist.publish(Float64(data=dist))

        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = LiveErrorPlotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
