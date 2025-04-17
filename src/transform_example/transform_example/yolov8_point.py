import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
import tf2_ros
import numpy as np
import cv2
from ultralytics import YOLO
from tf_transformations import quaternion_multiply, quaternion_from_matrix, quaternion_matrix
from scipy.spatial.transform import Rotation as Rs


class CameraYoloProcessor(Node):
    def __init__(self):
        super().__init__('camera_yolo_processor')

        # YOLO 模型加載
        self.model = YOLO('pamu.pt')  # 替換為你的模型路徑

        # 訂閱影像和相機參數
        #self.image_sub = self.create_subscription(Image, '/camera/camera/color/image_rect_raw', self.image_callback, 10)
        #self.depth_sub = self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        #self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)
        
        self.image_sub = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10)
        self.color_frame_id = 'camera_color_optical_frame' 


        self.bridge = CvBridge()
        self.camera_intrinsics = None
        self.depth_image = None
        # TF 廣播器
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        # TF2 Buffer 和 Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        
        # 座標系名稱
        self.object_frame = 'object_frame'  # 物體座標系 (來自 YOLO 輸出的 TF)
        self.camera_frame = 'camera_color_optical_frame'   # 相機座標系
        self.link6_frame = 'link6'          # link6 座標系
        self.base_frame = 'base_link'       # 基座座標系

        # 啟動定時器，每 0.5 秒執行一次
        self.timer = self.create_timer(0.1, self.transform_object_to_base)
        self.get_logger().info('啟動定時器')
        

    def camera_info_callback(self, msg):
        """接收相機內參"""
        self.camera_intrinsics = np.array([
            [msg.k[0], msg.k[1], msg.k[2]],
            [msg.k[3], msg.k[4], msg.k[5]],
            [msg.k[6], msg.k[7], msg.k[8]]
        ])
        self.color_frame_id = msg.header.frame_id

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
            cv2.putText(cv_image, f"conf {conf} ,XYZ: {xyz_camera*100}mm", ((pixel_x-100), pixel_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

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
        t.header.frame_id = self.color_frame_id
        t.child_frame_id = child_frame_id
        t.transform.translation.x = translation[0] 
        t.transform.translation.y = translation[1] 
        t.transform.translation.z = (translation[2]-0.1)
        t.transform.rotation.x = float(rotation[0])
        t.transform.rotation.y = float(rotation[1])
        t.transform.rotation.z = float(rotation[2])
        t.transform.rotation.w = float(rotation[3])

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f"Broadcasting TF for {child_frame_id}")
    def transform_object_to_base(self):
        try:
            # 直接使用 TF2 的查詢功能獲取從相機到基座的變換
            self.get_logger().info('嘗試查詢從相機到基座的變換...')
            transform_base_to_camera = self.tf_buffer.lookup_transform(
                self.base_frame, self.camera_frame, rclpy.time.Time()
            )

            # 嘗試獲取物體相對於相機的變換
            self.get_logger().info('嘗試查詢物體相對於相機的變換...')
            transform_camera_to_object = self.tf_buffer.lookup_transform(
                self.camera_frame, self.object_frame, rclpy.time.Time()
            )
            # 1. 將相機到物體的變換轉換為矩陣
            T_camera_to_object = self.transform_to_matrix(transform_camera_to_object)

            # 2. 將基座到相機的變換轉換為矩陣
            T_base_to_camera = self.transform_to_matrix(transform_base_to_camera)

            # 3. 計算基座到物體的變換矩陣
            T_base_to_object = np.dot(T_base_to_camera, T_camera_to_object)

            # 4. 從變換矩陣提取位置和姿態
            position = T_base_to_object[:3, 3]
            rotation_matrix = T_base_to_object[:3, :3]

            # 從旋轉矩陣計算四元數
            quaternion = quaternion_from_matrix(T_base_to_object)
            
            # 補 Z 軸旋轉 90 度
            q_orig = Rs.from_quat(quaternion) 
            q_z90 = Rs.from_euler('z', 90, degrees=True)
            q_new = q_orig * q_z90
            quaternion_fixed = q_new.as_quat()
            

            # 5. 廣播物體相對於基座的TF
            self.broadcast_object_tf(position, quaternion_fixed)

            # 輸出結果
            self.get_logger().info(f"物體位置相對於基座: {position}")
            self.get_logger().info(f"物體姿態相對於基座(四元數): {quaternion}")          

        except Exception as e:
            self.get_logger().error(f"Failed to transform object point: {str(e)}")

    def transform_to_matrix(self, transform: TransformStamped):
        """ 將 TF 變換轉換為 4x4 齊次變換矩陣 """
        trans = transform.transform.translation
        rot = transform.transform.rotation

        # 旋轉矩陣 (四元數轉換)
        q = [rot.x, rot.y, rot.z, rot.w]
        R = self.quaternion_to_rotation_matrix(q)
      
        # 平移向量
        T = np.array([[R[0, 0], R[0, 1], R[0, 2], trans.x],
                      [R[1, 0], R[1, 1], R[1, 2], trans.y],
                      [R[2, 0], R[2, 1], R[2, 2], trans.z],
                      [0, 0, 0, 1]])
        return T

    def quaternion_to_rotation_matrix(self, q):
        """ 將四元數轉換為旋轉矩陣 """
        x, y, z, w = q
        R = np.array([
            [1 - 2 * y**2 - 2 * z**2, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
            [2 * x * y + 2 * z * w, 1 - 2 * x**2 - 2 * z**2, 2 * y * z - 2 * x * w],
            [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x**2 - 2 * y**2]
        ])
        return R
        
    def broadcast_object_tf(self, position,quaternion):
        """ 廣播物體的 TF 到 base_link """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame  # 基座座標系
        t.child_frame_id = 'object_in_base'  # 新的物體 TF 名稱

        # 平移部分
        t.transform.translation.x = position[0]
        t.transform.translation.y = position[1]
        t.transform.translation.z = position[2]

        # 假設旋轉為單位四元數 (物體沒有旋轉)
        # t.transform.rotation.x = 0.0
        # t.transform.rotation.y = 0.0
        # t.transform.rotation.z = 0.0
        # t.transform.rotation.w = 1.0
        # 旋轉部分（四元數）
        t.transform.rotation.x = float(quaternion[0])
        t.transform.rotation.y = float(quaternion[1])
        t.transform.rotation.z = float(quaternion[2])
        t.transform.rotation.w = float(quaternion[3])

        # 發布 TF
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f"發布了物體TF，位置: {position}，姿態: {quaternion}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraYoloProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

