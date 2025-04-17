# tf_republisher_node.py (ROS2版)
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

class TFRepublisher(Node):
    def __init__(self):
        super().__init__('tf_republisher')
        self.tf_sub = self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        self.tf_static_sub = self.create_subscription(TFMessage, '/tf_static', self.tf_callback, 10)
        self.tf_pub = self.create_publisher(TFMessage, '/web_tf', 10)

    def tf_callback(self, msg):
        # 轉發給 /web_tf topic
        self.tf_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TFRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

