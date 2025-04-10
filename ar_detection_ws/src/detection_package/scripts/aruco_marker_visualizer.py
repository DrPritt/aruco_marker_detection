#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from aruco_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker, MarkerArray as VizMarkerArray
from std_msgs.msg import ColorRGBA
import tf_transformations


class ArucoMarkerVisualizer(Node):
    def __init__(self):
        super().__init__('aruco_marker_visualizer')

        self.subscription = self.create_subscription(
            MarkerArray,
            '/marker_publisher/markers',
            self.marker_callback,
            10)

        self.publisher = self.create_publisher(
            VizMarkerArray,
            '/visualization_marker_array',
            10)

    def marker_callback(self, msg):
        markers_viz = VizMarkerArray()
        for marker in msg.markers:
            m = Marker()
            m.header = marker.header
            m.ns = 'aruco'
            m.id = marker.id
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose = marker.pose.pose
            m.scale.x = 0.08  # or marker.size if available
            m.scale.y = 0.08
            m.scale.z = 0.001
            m.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
            m.lifetime = rclpy.duration.Duration(seconds=0).to_msg()  # persistent
            markers_viz.markers.append(m)

        self.publisher.publish(markers_viz)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoMarkerVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

