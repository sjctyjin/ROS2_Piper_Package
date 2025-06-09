import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
import tf2_ros
import numpy as np
import torch
import cv2
from ultralytics import YOLO

class YoloD435TFBroadcaster(Node):
    def __init__(self):
        super().__init__('yolo_d435_tf_broadcaster')

        # 初始化 YOLOv8 模型
        self.model = YOLO('yolov8n.pt')  # 替換為你的 YOLOv8 模型路徑

        # 訂閱相機影像和相機資訊
        self.image_sub = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10)

        # TF 廣播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # CvBridge 用於 ROS 圖像格式轉換
        self.bridge = CvBridge()

        # 相機內參
        self.camera_intrinsics = None

    def camera_info_callback(self, msg):
        """接收相機內參"""
        self.camera_intrinsics = np.array([
            [msg.k[0], msg.k[1], msg.k[2]],
            [msg.k[3], msg.k[4], msg.k[5]],
            [msg.k[6], msg.k[7], msg.k[8]]
        ])

    def image_callback(self, msg):
        """處理影像訊息"""
        if self.camera_intrinsics is None:
            self.get_logger().warning("Camera intrinsics not received yet.")
            return

        # 將 ROS Image 轉換為 OpenCV 格式
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 使用 YOLOv8 偵測物體
        results = self.model(cv_image)
        detections = results[0].boxes.data.cpu().numpy()  # 偵測結果 [x1, y1, x2, y2, conf, class]

        # 處理每個偵測到的物體
        for detection in detections:
            x1, y1, x2, y2, conf, cls = detection
            pixel_x = int((x1 + x2) / 2)
            pixel_y = int((y1 + y2) / 2)

            # 獲取深度值 (假設深度資料從 /camera/aligned_depth_to_color/image_raw 獲得)
            depth = self.get_depth(pixel_x, pixel_y)

            if depth is None:
                continue

            # 計算相機座標
            point_camera = self.pixel_to_camera(pixel_x, pixel_y, depth)

            # 假設物體在相機坐標系下的姿態 (此處可替換為更精確的估計)
            rotation_quaternion = [0, 0, 0, 1]  # 單位四元數

            # 廣播到 TF
            self.broadcast_tf(point_camera, rotation_quaternion, 'object_frame')

    def pixel_to_camera(self, pixel_x, pixel_y, depth):
        """將像素座標轉換為相機座標"""
        uv = np.array([pixel_x, pixel_y, 1.0])  # 齊次像素座標
        xyz = depth * np.linalg.inv(self.camera_intrinsics).dot(uv)
        return xyz

    def broadcast_tf(self, translation, rotation, child_frame_id):
        """廣播物體的 TF"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_link'
        t.child_frame_id = child_frame_id
        t.transform.translation.x = translation[2]
        t.transform.translation.y = translation[0]*-1
        t.transform.translation.z = translation[1]*-1
        t.transform.rotation.x = float(rotation[0])
        t.transform.rotation.y = float(rotation[1])
        t.transform.rotation.z = float(rotation[2])
        t.transform.rotation.w = float(rotation[3])

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f"Broadcasting TF for {child_frame_id}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloD435TFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

