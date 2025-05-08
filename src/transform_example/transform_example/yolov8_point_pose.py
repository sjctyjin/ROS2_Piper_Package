import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import TransformStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import tf2_ros
import numpy as np
import cv2
import open3d as o3d
import copy
from ultralytics import YOLO
from ultralytics import SAM  # Import the SAM model
import time
from tf_transformations import quaternion_multiply, quaternion_from_matrix, quaternion_matrix
from scipy.spatial.transform import Rotation as Rs

class GraspPoseDetector(Node):
    def __init__(self):
        super().__init__('grasp_pose_detector')
        # 初始化偵測時間
        self.last_detection_time = self.get_clock().now()  
        # 可容忍時間
        self.detection_timeout = rclpy.duration.Duration(seconds=1)  

        # 宣告參數
        self.declare_parameter('yolo_model', 'demo.pt')
        self.declare_parameter('sam_model', 'mobile_sam.pt')  # Add parameter for SAM model sam2.1_b
        self.declare_parameter('debug_visualization', True)
        self.detected_image_pub = self.create_publisher(Image, '/yolo/detect_img', 10)
        # 创建发布者
        self.publisher = self.create_publisher(
            TransformStamped, '/object_in_frame', 10)
        # 獲取參數
        yolo_model_path = self.get_parameter('yolo_model').get_parameter_value().string_value
        sam_model_path = self.get_parameter('sam_model').get_parameter_value().string_value
        self.debug_viz = self.get_parameter('debug_visualization').get_parameter_value().bool_value

        # YOLO 模型載入
        self.get_logger().info(f'正在載入 YOLOv8 模型: {yolo_model_path}...')
        self.model = YOLO(yolo_model_path)
        self.get_logger().info('YOLOv8 模型載入完成')

        # SAM 模型載入
        self.get_logger().info(f'正在載入 SAM 模型: {sam_model_path}...')
        try:
            # 載入 SAM 模型，設定為分割任務
            self.sam_model = SAM(sam_model_path)
            self.get_logger().info('SAM 模型載入完成')
        except Exception as e:
            self.get_logger().error(f'載入 SAM 模型失敗: {str(e)}')
            self.get_logger().warning('將繼續使用 YOLO 檢測結果，但不使用 SAM 分割')
            self.sam_model = None

        # 訂閱影像和相機參數
        self.image_sub = self.create_subscription(Image, '/camera/camera/color/image_rect_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)
        
        #self.image_sub = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        #self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        #self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10)
        # 新增遮罩發布者（用於調試和可視化）
        self.mask_pub = self.create_publisher(Image, 'segmentation_mask', 10)

        self.bridge = CvBridge()
        self.camera_intrinsics = None
        self.depth_image = None
        self.color_frame_id = 'camera_color_optical_frame'  # 相機坐標系名稱
        self.color_image = None  # 儲存最新的彩色影像

        # TF 廣播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 創建可視化標記發布者，用於 RViz2 顯示
        self.marker_pub = self.create_publisher(MarkerArray, 'grasp_visualization', 10)
        self.grasp_points_pub = self.create_publisher(MarkerArray, 'grasp_points', 10)

        # 添加結果影像發布者
        self.result_image_pub = self.create_publisher(Image, 'detection_result', 10)

        self.get_logger().info('夾取姿態偵測節點已初始化，等待影像數據...')
        
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
        #self.timer = self.create_timer(0.1, self.transform_object_to_base)
        # 创建定时器(給web使用)
        self.tf_timer = self.create_timer(0.1, self.publish_transform)
        self.get_logger().info('啟動定時器')

    def camera_info_callback(self, msg):
        """接收相機內參"""
        self.camera_intrinsics = np.array([
            [msg.k[0], msg.k[1], msg.k[2]],
            [msg.k[3], msg.k[4], msg.k[5]],
            [msg.k[6], msg.k[7], msg.k[8]]
        ])
        self.color_frame_id = msg.header.frame_id
        #self.get_logger().info(f'已獲取相機內參，相機坐標系: {self.color_frame_id}')

    def depth_callback(self, msg):
        """接收深度影像"""
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def image_callback(self, msg):
    
        def display():
            # 顯示影像（無檢測結果）
            cv2.imshow("Object Detection", cv_image)
            cv2.waitKey(1)
            #發布影像
            result_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            result_msg.header.stamp = msg.header.stamp
            result_msg.header.frame_id = self.color_frame_id  # 與相機一致
            self.detected_image_pub.publish(result_msg)
            
        """處理彩色影像"""
        if self.camera_intrinsics is None or self.depth_image is None:
            self.get_logger().warning("等待相機參數和深度影像")
            return

        # 將 ROS Image 轉換為 OpenCV 格式
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.color_image = cv_image.copy()  # 保存彩色影像的副本

        # 使用 YOLO 偵測物體
        results = self.model(cv_image)

        # 檢查是否有檢測到物體
        if len(results[0].boxes) == 0:
            self.get_logger().warning("無物體")
            
            if self.debug_viz:
                # 顯示影像（無檢測結果）
                cv2.imshow("Object Detection", cv_image)
                cv2.waitKey(1)
                #發布影像
                result_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
                result_msg.header.stamp = msg.header.stamp
                result_msg.header.frame_id = self.color_frame_id  # 與相機一致
                self.detected_image_pub.publish(result_msg)
            return

        # 獲取所有檢測結果
        detections = results[0].boxes.data.cpu().numpy()  # 偵測結果 [x1, y1, x2, y2, conf, class]

        # 取得影像高度和寬度
        height, width = cv_image.shape[:2]

        # 計算中心點
        center_x, center_y = width // 2, height // 2

        # 畫中心十字線（可選）
        if self.debug_viz:
            # 畫水平線
            cv2.line(cv_image, (0, center_y), (width, center_y), (255, 255, 255), 1)
            # 畫垂直線
            cv2.line(cv_image, (center_x, 0), (center_x, height), (255, 255, 255), 1)

        # 選擇置信度最高的檢測結果
        best_detection = None
        best_conf = 0

        for detection in detections:
            x1, y1, x2, y2, conf, cls = detection
            if conf > best_conf:
                best_conf = conf
                best_detection = detection
                                                 
        if best_conf < 0.85:
            self.get_logger().warning(f'信心不足 ： {best_conf}')
            display()
            return
            
        # 處理最佳檢測結果
        if best_detection is not None:
            x1, y1, x2, y2, conf, cls = best_detection
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            # 計算邊界框中心點
            center_pixel_x = int((x1 + x2) / 2)
            center_pixel_y = int((y1 + y2) / 2)

            # 獲取中心點的深度值
            center_depth = self.depth_image[center_pixel_y, center_pixel_x] / 1000.0  # 毫米轉米
            
            if center_depth > 0.4:
                self.get_logger().warning('超出距離')
                display()
                return


            # 使用SAM進行像素級分割 (如果可用)
            if self.sam_model is not None:
                mask = self.segment_with_sam(cv_image, x1, y1, x2, y2)
                if mask is None:
                    self.get_logger().warning('無法生成像素級分割遮罩')
                    display()
                    return
            else:
                # 如果SAM模型不可用，則使用邊界框代替
                self.get_logger().info('使用邊界框代替像素級分割')
                mask = np.zeros(cv_image.shape[:2], dtype=np.uint8)
                mask[y1:y2, x1:x2] = 255

            
            if center_depth == 0:
                self.get_logger().warning(f"物體中心點深度無效: ({center_pixel_x}, {center_pixel_y})")
                display()
                return

            # 使用遮罩提取點雲，而不是簡單的矩形區域
            roi_points = self.extract_masked_point_cloud(mask)

            # 檢查是否提取到足夠的點
            if len(roi_points) < 10:
                self.get_logger().warning(f"遮罩內有效點數不足: {len(roi_points)}")
                display()
                return

            # 將點列表轉換為 NumPy 數組
            roi_points_np = np.array(roi_points)

            # 創建 Open3D 點雲對象
            o3d_cloud = o3d.geometry.PointCloud()
            o3d_cloud.points = o3d.utility.Vector3dVector(roi_points_np)

            # 移除離群點（可選）
            o3d_cloud, _ = o3d_cloud.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

            # 生成抓取姿態
            #self.get_logger().info('計算最佳抓取姿態...')
            result = self.generate_grasp_pose(o3d_cloud)

            if result is not None:
                grasp_position, rotation_matrix, narrow_width, grasp_points = result

            if self.debug_viz:
                # 顯示基本的檢測結果
                detection_viz = cv_image.copy()
                cv2.rectangle(detection_viz, (x1, y1), (x2, y2), (0, 255, 0), 2)

                # 添加遮罩覆蓋
                mask_overlay = np.zeros_like(cv_image)
                mask_overlay[mask > 0] = [0, 0, 255]  # 紅色遮罩
                result_viz = cv2.addWeighted(detection_viz, 0.7, mask_overlay, 0.3, 0)

                # 顯示抓取信息
                cv2.putText(result_viz,
                            f"Grasp: [{grasp_position[0]:.2f}, {grasp_position[1]:.2f}, {grasp_position[2]:.2f}]m",
                            (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.putText(result_viz, f"Width: {narrow_width * 100:.1f}cm",
                            (x1, y1 - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                cv2.imshow("Object Detection", result_viz)
                cv2.waitKey(1)
                #發布影像
                result_msg = self.bridge.cv2_to_imgmsg(result_viz, encoding='bgr8')
                result_msg.header.stamp = msg.header.stamp
                result_msg.header.frame_id = self.color_frame_id  # 與相機一致
                self.detected_image_pub.publish(result_msg)

                # 發布 TF 變換
                self.publish_object_tf(grasp_position, rotation_matrix, msg.header.stamp)
                

                # 發布 RViz2 可視化標記
                #self.publish_rviz_markers(grasp_position, rotation_matrix, narrow_width, grasp_points, msg.header.stamp)

                #self.get_logger().info(f"抓取位置: {grasp_position}")
                #self.get_logger().info(f"夾爪開度: {narrow_width:.3f}m")

            else:
                self.get_logger().warning('無法生成抓取姿態')

        # 顯示影像
        # if self.debug_viz:
        #     original_with_bbox = cv_image.copy()
        #     # 只畫出邊界框和YOLO結果
        #     for detection in detections:
        #         x1, y1, x2, y2, conf, cls = detection
        #         x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
        #         cv2.rectangle(original_with_bbox, (x1, y1), (x2, y2), (0, 255, 0), 2)
        #         cv2.putText(original_with_bbox, f"Conf: {conf:.2f}", (x1, y1 - 5),
        #                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        #
        #     cv2.imshow("Object Detection", original_with_bbox)
        #     cv2.waitKey(1)

    def segment_with_sam(self, image, x1, y1, x2, y2):
        """
        使用SAM模型對YOLOv8檢測到的區域進行像素級分割

        Args:
            image: 輸入彩色影像
            x1, y1, x2, y2: YOLO 檢測的邊界框

        Returns:
            mask: 分割遮罩，與原始影像大小相同的二值遮罩
        """
        self.get_logger().info(f'使用SAM對檢測到的物體進行分割...')
        try:
            # 裁剪出檢測到的物體區域
            roi = image[y1:y2, x1:x2]
            sam_start = time.time()
            # 使用SAM對ROI進行分割
            results = self.sam_model(
                source=image,  # 只處理物體區域
                bboxes=[[x1, y1, x2, y2]],
                conf=0.25,  # 較低的置信度閾值以確保獲得完整遮罩
                save=False,  # 不保存結果
                verbose=False  # 不顯示詳細信息
            )

            # 初始化完整影像大小的遮罩
            full_mask = np.zeros(image.shape[:2], dtype=np.uint8)

            # 檢查是否有分割結果
            if hasattr(results[0], 'masks') and results[0].masks is not None and len(results[0].masks) > 0:
                # SAM 可能產生多個 mask，這裡僅取第一個做示範
                roi_mask = results[0].masks.data[0].cpu().numpy()

                # 將浮點數遮罩轉換為二值遮罩 (閾值 0.5)
                binary_mask = np.zeros(roi_mask.shape[:2], dtype=np.uint8)
                binary_mask[roi_mask > 0.5] = 255

                # SAM 的遮罩通常已對應整張影像大小，無需手動「放回去」
                full_mask = binary_mask

                #self.get_logger().info(f'SAM分割成功，遮罩中點數: {np.sum(full_mask > 0)}')
            else:
                self.get_logger().warning('SAM未生成有效遮罩，使用邊界框代替')
                full_mask[y1:y2, x1:x2] = 255

            # 簡單視覺化（如果需要）
            # if self.debug_viz:
            #     mask_msg = self.bridge.cv2_to_imgmsg(full_mask, encoding="mono8")
            #     mask_msg.header.stamp = self.get_clock().now().to_msg()
            #     mask_msg.header.frame_id = self.color_frame_id
            #     self.mask_pub.publish(mask_msg)
            sam_end = time.time()
            sam_time = (sam_end - sam_start) * 1000  # 毫秒
            self.get_logger().info(f'SAM處理時間 : {sam_time}')
            return full_mask

        except Exception as e:
            self.get_logger().error(f'SAM分割錯誤: {str(e)}')
            # 出錯時使用邊界框
            fallback_mask = np.zeros(image.shape[:2], dtype=np.uint8)
            fallback_mask[y1:y2, x1:x2] = 255
            return fallback_mask

    def extract_masked_point_cloud(self, mask):
        """
        從深度圖像中提取遮罩區域內的點雲

        Args:
            mask: 二值遮罩，指示要提取的像素

        Returns:
            roi_points: 遮罩區域內的3D點列表 [[x, y, z], ...]
        """
        roi_points = []

        # 找到所有遮罩區域內的像素坐標
        mask_indices = np.where(mask > 0)

        # 遍歷遮罩內的每個像素
        for i in range(len(mask_indices[0])):
            v = mask_indices[0][i]  # y 坐標
            u = mask_indices[1][i]  # x 坐標

            # 檢查坐標是否在深度圖像範圍內
            if v >= self.depth_image.shape[0] or u >= self.depth_image.shape[1]:
                continue

            # 獲取深度值
            depth = self.depth_image[v, u] / 1000.0  # 毫米轉米

            # 跳過無效深度
            if depth <= 0 or depth > 3.0:
                continue

            # 將像素坐標轉換為相機坐標
            uv = np.array([u, v, 1.0])
            xyz_camera = depth * np.linalg.inv(self.camera_intrinsics).dot(uv)

            # 添加到點雲
            roi_points.append([xyz_camera[0], xyz_camera[1], xyz_camera[2]])

        #self.get_logger().info(f'從遮罩中提取的點數量: {len(roi_points)}')
        return roi_points

    def generate_grasp_pose(self, point_cloud):
        """
        從點雲生成抓取姿態，確保Z軸方向保持穩定

        Args:
            point_cloud: Open3D點雲對象

        Returns:
            grasp_position: 抓取位置 (3D向量)
            rotation_matrix: 抓取方向 (3x3旋轉矩陣)
            narrow_width: 最窄方向的寬度
            grasp_points: 最窄方向上的兩個極點
        """
        # 獲取點雲數據
        points = np.asarray(point_cloud.points)
        if len(points) < 10:
            self.get_logger().warning('點雲中點數太少，無法生成可靠的抓取姿態')
            return None

        # 計算點雲的中心
        centroid = np.mean(points, axis=0)
        points_centered = points - centroid

        # 對點雲進行主成分分析
        cov_matrix = np.cov(points_centered, rowvar=False)
        eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)

        # 對特徵值/特徵向量進行排序（從小到大）
        idx = eigenvalues.argsort()
        eigenvalues = eigenvalues[idx]
        eigenvectors = eigenvectors[:, idx]

        # 提取主成分軸
        minor_axis = eigenvectors[:, 0]  # 最小主成分 - 物體最窄的方向
        middle_axis = eigenvectors[:, 1]  # 中間主成分
        major_axis = eigenvectors[:, 2]  # 最大主成分 - 物體最長的方向

        # 估計物體尺寸
        obj_size = np.max(np.abs(points_centered)) * 2

        # 物體形狀分析
        shape_elongation = eigenvalues[2] / eigenvalues[1]
        shape_flatness = eigenvalues[1] / eigenvalues[0]
        #self.get_logger().info(f"形狀分析 - 延展度: {shape_elongation:.2f}, 平坦度: {shape_flatness:.2f}")
        self.get_logger().info(f"物體尺寸估計: {obj_size:.3f} m")

        # 定義相機座標系中的Z軸方向（通常是[0,0,1]，朝前）
        camera_z = np.array([0, 0, 1])

        # 首先選擇最小主成分作為抓取方向的基準
        grasp_axis = minor_axis

        # 如果最小主成分不適合作為抓取方向（例如幾乎垂直於相機平面），則使用中間主成分
        if abs(np.dot(minor_axis, camera_z)) > 0.8:
            #self.get_logger().info("最小主成分接近垂直於相機平面，使用中間主成分作為抓取方向")
            grasp_axis = middle_axis

        # 確保grasp_axis是單位向量
        grasp_axis = grasp_axis / np.linalg.norm(grasp_axis)

        # 檢查grasp_axis是否與相機Z軸接近垂直，這是我們想要的
        # 如果不夠垂直，可以進行調整
        z_alignment = np.abs(np.dot(grasp_axis, camera_z))
        if z_alignment < 0.3:  # 允許一定的傾斜
            self.get_logger().info(f"抓取軸與相機Z軸足夠垂直: {z_alignment:.2f}")
        else:
            #self.get_logger().info(f"抓取軸與相機Z軸不夠垂直: {z_alignment:.2f}，進行調整")
            # 嘗試使用其他主成分
            if abs(np.dot(middle_axis, camera_z)) < z_alignment:
                grasp_axis = middle_axis
                #self.get_logger().info("改用中間主成分作為抓取方向")
            elif abs(np.dot(major_axis, camera_z)) < z_alignment:
                grasp_axis = major_axis
                #self.get_logger().info("改用最大主成分作為抓取方向")

        # 再次確保grasp_axis是單位向量
        grasp_axis = grasp_axis / np.linalg.norm(grasp_axis)

        # 使Z軸始終與相機Z軸方向相似（保持Z軸的穩定性）
        # 計算與相機Z軸最接近的方向作為Z軸
        z_axis = camera_z

        # 確保Z軸與抓取軸垂直
        # 首先，從Z軸中減去它在抓取軸上的投影
        z_proj_on_grasp = np.dot(z_axis, grasp_axis) * grasp_axis
        z_axis_adjusted = z_axis - z_proj_on_grasp

        # 如果調整後的Z軸太小，使用另一個方向
        if np.linalg.norm(z_axis_adjusted) < 0.1:
            # 選擇一個與grasp_axis垂直的任意方向
            if abs(grasp_axis[0]) < 0.9:
                z_axis_adjusted = np.cross(grasp_axis, [1, 0, 0])
            else:
                z_axis_adjusted = np.cross(grasp_axis, [0, 1, 0])

        # 標準化Z軸
        z_axis = z_axis_adjusted / np.linalg.norm(z_axis_adjusted)

        # 計算X軸為抓取方向
        y_axis = grasp_axis

        # 重新計算Y軸以確保正交性
        x_axis = np.cross(z_axis, y_axis)
        x_axis = x_axis / np.linalg.norm(x_axis)

        # 最後再檢查一次，確保X軸仍然是抓取方向
        y_axis = np.cross(x_axis, z_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)
        
        # 沿Z軸旋轉90度
        # 使用旋轉矩陣進行旋轉（順時針90度，即-90度）
        theta = -np.pi/2  # 負的90度（順時針）
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)

        # 旋轉矩陣（繞Z軸旋轉）
        rot_z = np.array([
            [cos_theta, -sin_theta, 0],
            [sin_theta, cos_theta, 0],
            [0, 0, 1]
        ])

        # 構建旋轉矩陣
        rotation_matrix = np.column_stack((x_axis, y_axis, z_axis))  @ rot_z

        # 尋找抓取軸上的兩個極點
        # 將點投影到x_axis上
        proj = np.dot(points, y_axis)
        min_idx = np.argmin(proj)
        max_idx = np.argmax(proj)

        # 獲取最窄方向上的兩個極點
        min_point = points[min_idx]
        max_point = points[max_idx]

        # 計算極點之間的精確距離
        narrow_width = np.linalg.norm(max_point - min_point)

        # 抓取點應該在兩個極點的中間
        midpoint = (min_point + max_point) / 2

        # 將中點作為抓取位置
        grasp_position = midpoint

        # 記錄主要軸向方向
        #self.get_logger().info(f"X軸（夾爪開合方向）: {x_axis}")
        #self.get_logger().info(f"Y軸（接近方向）: {y_axis}")
        #self.get_logger().info(f"Z軸（穩定方向）: {z_axis}")
        #self.get_logger().info(f"最窄部位的實際寬度: {narrow_width:.3f}m")

        return grasp_position, rotation_matrix, narrow_width, [min_point, max_point]

    def publish_object_tf(self, position, rotation_matrix, timestamp):
        """
        發布物體的 TF 變換

        Args:
            position: 物體位置
            rotation_matrix: 旋轉矩陣
            timestamp: 時間戳
        """
        # 創建一個 TransformStamped 消息
        t = TransformStamped()

        # 設置時間戳和坐標系
        t.header.stamp = timestamp
        t.header.frame_id = self.color_frame_id
        t.child_frame_id = "object_frame"

       

        # 從旋轉矩陣轉換為四元數
        trace = rotation_matrix[0, 0] + rotation_matrix[1, 1] + rotation_matrix[2, 2]

        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) * s
            y = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) * s
            z = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) * s
        else:
            if rotation_matrix[0, 0] > rotation_matrix[1, 1] and rotation_matrix[0, 0] > rotation_matrix[2, 2]:
                s = 2.0 * np.sqrt(1.0 + rotation_matrix[0, 0] - rotation_matrix[1, 1] - rotation_matrix[2, 2])
                w = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
                x = 0.25 * s
                y = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
                z = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
            elif rotation_matrix[1, 1] > rotation_matrix[2, 2]:
                s = 2.0 * np.sqrt(1.0 + rotation_matrix[1, 1] - rotation_matrix[0, 0] - rotation_matrix[2, 2])
                w = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s
                x = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
                y = 0.25 * s
                z = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + rotation_matrix[2, 2] - rotation_matrix[0, 0] - rotation_matrix[1, 1])
                w = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
                x = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
                y = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
                z = 0.25 * s
         # 設置位置
        t.transform.translation.x = float(position[0])
        t.transform.translation.y = float(position[1])
        t.transform.translation.z = float(position[2]) - 0.10

        # 設置四元數
        t.transform.rotation.x = float(x)
        t.transform.rotation.y = float(y)
        t.transform.rotation.z = float(z)
        t.transform.rotation.w = float(w)

        # 發布 TF
        self.tf_broadcaster.sendTransform(t) 
        #self.get_logger().info(f"已發布物體 TF: {t.child_frame_id}")
        self.last_detection_time = self.get_clock().now()
        # 直接call object_in_base變換
        self.transform_object_to_base() 

    def publish_rviz_markers(self, position, rotation_matrix, narrow_width, grasp_points, timestamp):
        """
        發布 RViz2 可視化標記，用於顯示夾爪和抓取點

        Args:
            position: 抓取位置
            rotation_matrix: 旋轉矩陣
            narrow_width: 最窄方向的寬度
            grasp_points: 最窄方向上的兩個極點
            timestamp: 時間戳
        """
        if grasp_points is None or len(grasp_points) != 2:
            self.get_logger().warning('需要兩個極點來創建 RViz 標記')
            return

        # 創建標記數組
        marker_array = MarkerArray()
        grasp_points_array = MarkerArray()

        # 1. 添加最窄方向線標記（兩個極點之間的線）
        line_marker = Marker()
        line_marker.header.frame_id = self.color_frame_id
        line_marker.header.stamp = timestamp
        line_marker.ns = "grasp_visualization"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.pose.orientation.w = 1.0
        line_marker.scale.x = 0.005  # 線寬
        line_marker.color.r = 0.0
        line_marker.color.g = 1.0
        line_marker.color.b = 0.0
        line_marker.color.a = 1.0

        # 添加線的點
        p1 = Point()
        p1.x, p1.y, p1.z = float(grasp_points[0][0]), float(grasp_points[0][1]), float(grasp_points[0][2])
        p2 = Point()
        p2.x, p2.y, p2.z = float(grasp_points[1][0]), float(grasp_points[1][1]), float(grasp_points[1][2])

        line_marker.points = [p1, p2]
        marker_array.markers.append(line_marker)

        # 2. 添加兩個抓取點標記（紅色球體）
        for i, point in enumerate(grasp_points):
            point_marker = Marker()
            point_marker.header.frame_id = self.color_frame_id
            point_marker.header.stamp = timestamp
            point_marker.ns = "grasp_points"
            point_marker.id = i
            point_marker.type = Marker.SPHERE
            point_marker.action = Marker.ADD
            point_marker.pose.position.x = float(point[0])
            point_marker.pose.position.y = float(point[1])
            point_marker.pose.position.z = float(point[2])
            point_marker.pose.orientation.w = 1.0
            point_marker.scale.x = narrow_width * 0.1
            point_marker.scale.y = narrow_width * 0.1
            point_marker.scale.z = narrow_width * 0.1
            point_marker.color.r = 1.0
            point_marker.color.g = 0.0
            point_marker.color.b = 0.0
            point_marker.color.a = 1.0

            grasp_points_array.markers.append(point_marker)

        # 3. 添加夾爪視覺模型標記
        # 夾爪基座
        base_marker = Marker()
        base_marker.header.frame_id = self.color_frame_id
        base_marker.header.stamp = timestamp
        base_marker.ns = "grasp_visualization"
        base_marker.id = 3
        base_marker.type = Marker.CUBE
        base_marker.action = Marker.ADD

        # 設置夾爪基座位置和方向
        base_marker.pose.position.x = float(position[0])
        base_marker.pose.position.y = float(position[1])
        base_marker.pose.position.z = float(position[2] + 0.15)  # 稍微抬高一點

        # 從旋轉矩陣轉換為四元數（與 TF 發布相同的計算方法）
        trace = rotation_matrix[0, 0] + rotation_matrix[1, 1] + rotation_matrix[2, 2]
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) * s
            y = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) * s
            z = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) * s
        else:
            if rotation_matrix[0, 0] > rotation_matrix[1, 1] and rotation_matrix[0, 0] > rotation_matrix[2, 2]:
                s = 2.0 * np.sqrt(1.0 + rotation_matrix[0, 0] - rotation_matrix[1, 1] - rotation_matrix[2, 2])
                w = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
                x = 0.25 * s
                y = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
                z = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
            elif rotation_matrix[1, 1] > rotation_matrix[2, 2]:
                s = 2.0 * np.sqrt(1.0 + rotation_matrix[1, 1] - rotation_matrix[0, 0] - rotation_matrix[2, 2])
                w = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s
                x = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
                y = 0.25 * s
                z = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + rotation_matrix[2, 2] - rotation_matrix[0, 0] - rotation_matrix[1, 1])
                w = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
                x = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
                y = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
                z = 0.25 * s

        base_marker.pose.orientation.x = float(x)
        base_marker.pose.orientation.y = float(y)
        base_marker.pose.orientation.z = float(z)
        base_marker.pose.orientation.w = float(w)

        base_marker.scale.x = 0.02
        base_marker.scale.y = 0.02
        base_marker.scale.z = 0.02
        base_marker.color.r = 0.8
        base_marker.color.g = 0.8
        base_marker.color.b = 0.8
        base_marker.color.a = 1.0

        marker_array.markers.append(base_marker)

        # 4. 添加坐標軸標記
        axes_marker = Marker()
        axes_marker.header.frame_id = "object_frame"
        axes_marker.header.stamp = timestamp
        axes_marker.ns = "grasp_visualization"
        axes_marker.id = 4
        axes_marker.type = Marker.ARROW
        axes_marker.action = Marker.ADD
        axes_marker.pose.orientation.w = 1.0
        axes_marker.scale.x = 0.01  # 軸桿直徑
        axes_marker.scale.y = 0.02  # 箭頭頭部直徑
        axes_marker.scale.z = 0.0  # 不使用

        # X軸（紅色）
        p1 = Point()
        p1.x, p1.y, p1.z = 0.0, 0.0, 0.0
        p2 = Point()
        p2.x, p2.y, p2.z = 0.05, 0.0, 0.0  # X軸方向
        axes_marker.points = [p1, p2]
        axes_marker.color.r = 1.0
        axes_marker.color.g = 0.0
        axes_marker.color.b = 0.0
        axes_marker.color.a = 1.0
        axes_marker.id = 4
        marker_array.markers.append(copy.deepcopy(axes_marker))

        # Y軸（綠色）
        p2.x, p2.y, p2.z = 0.0, 0.05, 0.0  # Y軸方向
        axes_marker.points = [p1, p2]
        axes_marker.color.r = 0.0
        axes_marker.color.g = 1.0
        axes_marker.color.b = 0.0
        axes_marker.id = 5
        marker_array.markers.append(copy.deepcopy(axes_marker))

        # Z軸（藍色）
        p2.x, p2.y, p2.z = 0.0, 0.0, 0.05  # Z軸方向
        axes_marker.points = [p1, p2]
        axes_marker.color.r = 0.0
        axes_marker.color.g = 0.0
        axes_marker.color.b = 1.0
        axes_marker.id = 6
        marker_array.markers.append(copy.deepcopy(axes_marker))

        # 發布標記數組
        self.marker_pub.publish(marker_array)
        self.grasp_points_pub.publish(grasp_points_array)

        #self.get_logger().info("已發布 RViz 可視化標記")
    def transform_object_to_base(self):
        try:
            now = self.get_clock().now()
            if now - self.last_detection_time > self.detection_timeout:
                self.get_logger().info("⏸️ 偵測超時，跳過 object_in_base 的發布")
                return  # 物體已不在畫面中，停止發布
            # 直接使用 TF2 的查詢功能獲取從相機到基座的變換
            #self.get_logger().info('嘗試查詢從相機到基座的變換...')
            transform_base_to_camera = self.tf_buffer.lookup_transform(
                self.base_frame, self.camera_frame, rclpy.time.Time()
            )

            # 嘗試獲取物體相對於相機的變換
            #self.get_logger().info('嘗試查詢物體相對於相機的變換...')
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
            quaternion = q_new.as_quat()
            
            # 5. 廣播物體相對於基座的TF
            self.broadcast_object_tf(position, quaternion)

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
    def publish_transform(self):
        try:
            # 查找TF
            transform = self.tf_buffer.lookup_transform(
                'base_link', 'object_in_base', rclpy.time.Time())
            now = self.get_clock().now()
            if now - self.last_detection_time > self.detection_timeout:
                self.get_logger().info("⏸️ 偵測超時，跳過 object_in_base 的發布")
                return  # 物體已不在畫面中，停止發布
            # 发布到话题
            self.publisher.publish(transform)
            self.get_logger().info(f"已發布話題 \n當前時間：{now}\n最後時間 ： {self.last_detection_time}\n延遲時間:{self.detection_timeout}")
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warning(f'无法查找变换: {e}')
def main(args=None):
    rclpy.init(args=args)

    node = GraspPoseDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 清理資源
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
