import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster

class PoseTFBroadcaster(Node):
    def __init__(self):
        super().__init__('pose_tf_broadcaster')
        self.pose_topic = self.declare_parameter('pose_topic').value
        self.child_frame = self.declare_parameter('child_frame').value

        self.br = TransformBroadcaster(self)
        self.sub = self.create_subscription(PoseStamped, self.pose_topic, self.callback, 10)

    def callback(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id  # usually 'world'
        t.child_frame_id = self.child_frame
        t.transform.translation = msg.pose.position
        t.transform.rotation = msg.pose.orientation
        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = PoseTFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()

