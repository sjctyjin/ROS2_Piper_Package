import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import random

class DynamicTFBroadcaster(Node):
    def __init__(self):
        super().__init__('dynamic_tf_broadcaster')

        # TF 廣播器
        self.tf_broadcaster = TransformBroadcaster(self)

        # 定時器：每 0.1 秒更新一次 TF
        self.timer = self.create_timer(0.1, self.broadcast_new_tf)

        # 假設的相對於 base_link 的初始位姿
        self.translation = [random.random() , random.random() , 0.3]  # x, y, z
        self.rotation = [0.0, 0.0, 0.0]  # Roll, Pitch, Yaw (radians)

    def broadcast_new_tf(self):
        # 構建 TransformStamped 消息
        t = TransformStamped()

        # 設置 Header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'  # 父座標系
        t.child_frame_id = 'object_point'  # 新增的子座標系

        # 設置平移
        t.transform.translation.x = self.translation[0]
        t.transform.translation.y = self.translation[1]
        t.transform.translation.z = self.translation[2]

        # 設置旋轉 (從 RPY 轉換為四元數)
        q = self.rpy_to_quaternion(self.rotation[0], self.rotation[1], self.rotation[2])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # 廣播 Transform
        self.tf_broadcaster.sendTransform(t)

        self.get_logger().info(f"Broadcasting TF: {t.child_frame_id} relative to {t.header.frame_id}")

    def rpy_to_quaternion(self, roll, pitch, yaw):
        """將 Roll, Pitch, Yaw (radians) 轉換為四元數"""
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return [qx, qy, qz, qw]


def main(args=None):
    rclpy.init(args=args)
    node = DynamicTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

