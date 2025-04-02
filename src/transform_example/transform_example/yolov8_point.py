import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
import tf2_ros
import numpy as np
import cv2
from ultralytics import YOLO


class CameraYoloProcessor(Node):
    def __init__(self):
        super().__init__('camera_yolo_processor')

        # YOLO 模型加載
        self.model = YOLO('pamu.pt')  # 替換為你的模型路徑

        # 訂閱影像和相機參數
        self.image_sub = self.create_subscription(Image, '/camera/camera/color/image_rect_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)

        self.bridge = CvBridge()
        self.camera_intrinsics = None
        self.depth_image = None
        # TF 廣播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def camera_info_callback(self, msg):
        """接收相機內參"""
        self.camera_intrinsics = np.array([
            [msg.k[0], msg.k[1], msg.k[2]],
            [msg.k[3], msg.k[4], msg.k[5]],
            [msg.k[6], msg.k[7], msg.k[8]]
        ])

    def depth_callback(self, msg):
        """接收深度影像"""
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def image_callback(self, msg):
        """處理彩色影像"""
        if self.camera_intrinsics is None or self.depth_image is None:
            self.get_logger().warning("等待相機參數和深度影像")
            return

        # 將 ROS Image 轉換為 OpenCV 格式
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        print("Test : ",cv_image.shape[:2])
        # 使用 YOLO 偵測物體
        results = self.model(cv_image)
        detections = results[0].boxes.data.cpu().numpy()  # 偵測結果 [x1, y1, x2, y2, conf, class]
 
        height, width = cv_image.shape[:2]  # 取得影像高度和寬度
        print(height,width)
        center_x, center_y = width // 2, height // 2  # 計算中心點

	    # 畫水平線
        cv2.line(cv_image, (0, center_y), (width, center_y), (255, 255, 255), 2)

	    # 畫垂直線
        cv2.line(cv_image, (center_x, 0), (center_x, height), (255, 255, 255), 2)
        #若無目標物 回歸原點

        for detection in detections:
            x1, y1, x2, y2, conf, cls = detection
            # if cls != 0:
            #     self.broadcast_tf([0.1, 0.0, 0.195], [0, 0, 0, 1], 'object_frame')
            #     continue
            pixel_x = int((x1 + x2) / 2)
            pixel_y = int((y1 + y2) / 2)

            # 獲取深度值
            depth = self.depth_image[pixel_y, pixel_x] / 1000.0  # 假設深度以毫米為單位，轉換為米
            if depth == 0:
                continue

            # 將像素座標轉換為相機座標
            uv = np.array([pixel_x, pixel_y, 1.0])
            xyz_camera = depth * np.linalg.inv(self.camera_intrinsics).dot(uv)
            # 在影像上標記
            cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.circle(cv_image, (pixel_x ,pixel_y), 10, (255, 0, 0), -4)
            cv2.putText(cv_image, f"XYZ: {xyz_camera*100}mm", ((pixel_x-100), pixel_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # 假設物體在相機坐標系下的姿態 (此處可替換為更精確的估計)
            rotation_quaternion = [0, 0, 0, 1]  # 單位四元數
            if cls == 0:
                # 廣播到 TF
                self.broadcast_tf(xyz_camera, rotation_quaternion, 'object_frame')

        # 顯示影像
        cv2.imshow("YOLO Detection", cv_image)
        cv2.waitKey(1)
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
    node = CameraYoloProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

