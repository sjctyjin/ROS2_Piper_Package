import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
import time


class PoseVisualizer(Node):
    def __init__(self):
        super().__init__('pose_visualizer')
        
        # Publisher 設定
        self.pose_publisher = self.create_publisher(PoseStamped, 'visualization_pose', 10)
        
        # 計時器 (1 秒發布一次 PoseStamped)
        self.timer = self.create_timer(1.0, self.publish_pose)
        
        # 假資料：物體在世界座標系的 Pose
        self.object_pose = {
            "position": [0.1, 0.2, 0.3],  # XYZ (單位：米)
            "orientation": [0.0, 0.0, 0.0, 1.0]  # 四元數 (Quaternion: x, y, z, w)
        }

    def publish_pose(self):
        """ 發布 PoseStamped 資料到 RViz """
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "base_link"  # 根據需要設定為相應的座標系

        # 設定位置 (Position)
        pose.pose.position.x = self.object_pose["position"][0]
        pose.pose.position.y = self.object_pose["position"][1]
        pose.pose.position.z = self.object_pose["position"][2]

        # 設定姿態 (Orientation)
        pose.pose.orientation.x = self.object_pose["orientation"][0]
        pose.pose.orientation.y = self.object_pose["orientation"][1]
        pose.pose.orientation.z = self.object_pose["orientation"][2]
        pose.pose.orientation.w = self.object_pose["orientation"][3]

        # 發布 PoseStamped 訊息
        self.pose_publisher.publish(pose)
        self.get_logger().info(f"Published pose: {pose}")


def main(args=None):
    rclpy.init(args=args)
    node = PoseVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

