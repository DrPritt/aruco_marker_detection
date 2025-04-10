#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from aruco_msgs.msg import MarkerArray
import csv
import os
from builtin_interfaces.msg import Time

class MarkerLogger(Node):
    def __init__(self):
        super().__init__('marker_logger')

        self.subscription = self.create_subscription(
            MarkerArray,
            '/marker_publisher/markers',
            self.listener_callback,
            10
        )

        self.csv_file_path = os.path.expanduser('~/marker_detections.csv')
        self.init_csv()

    def init_csv(self):
        if not os.path.exists(self.csv_file_path):
            with open(self.csv_file_path, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([
                    'timestamp_sec', 'marker_id',
                    'position_x', 'position_y', 'position_z',
                    'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w'
                ])

    def listener_callback(self, msg: MarkerArray):
        for marker in msg.markers:
            pose = marker.pose.pose
            timestamp = marker.header.stamp.sec + marker.header.stamp.nanosec * 1e-9

            row = [
                timestamp,
                marker.id,
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
            ]

            with open(self.csv_file_path, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(row)

            self.get_logger().info(f"Logged marker ID {marker.id} at time {timestamp:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = MarkerLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

