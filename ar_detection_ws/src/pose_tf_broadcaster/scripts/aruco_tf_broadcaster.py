import rclpy
from rclpy.node import Node
from aruco_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class ArucoTFBroadcaster(Node):
    def __init__(self):
        super().__init__('aruco_tf_broadcaster')
        self.aruco_topic = self.declare_parameter('aruco_topic').value
        self.camera_frame = self.declare_parameter('camera_frame').value
        self.child_frame = self.declare_parameter('child_frame').value

        self.br = TransformBroadcaster(self)
        self.sub = self.create_subscription(Marker, self.aruco_topic, self.callback, 10)

    def callback(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.camera_frame
        t.child_frame_id = self.child_frame
        t.transform.translation = msg.pose.pose.position
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoTFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()

