import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class RealsenseHsvTuner(Node):
    def __init__(self):
        super().__init__('realsense_hsv_tuner')

        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('h_low1', 0)
        self.declare_parameter('h_high1', 24)
        self.declare_parameter('s_low', 170)
        self.declare_parameter('v_low', 70)
        self.declare_parameter('min_area', 200)
        self.declare_parameter('max_area', 200000)
        self.declare_parameter('expand', 6)

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.bridge = CvBridge()

        self.window_name = 'HSV Tuner'
        self.mask_window_name = 'HSV Mask'
        self.result_window_name = 'HSV Result'

        self.mask_pub = self.create_publisher(Image, 'hsv_mask', 10)
        self.result_pub = self.create_publisher(Image, 'hsv_result', 10)
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self._create_windows()
        self.get_logger().info(f'訂閱影像 topic: {self.image_topic}')
        self.get_logger().info('按 s 可在終端輸出目前 HSV 參數，按 q 可關閉節點')

    def _create_windows(self):
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.namedWindow(self.mask_window_name, cv2.WINDOW_NORMAL)
        cv2.namedWindow(self.result_window_name, cv2.WINDOW_NORMAL)

        cv2.createTrackbar('H low', self.window_name, self._param_int('h_low1'), 179, self._noop)
        cv2.createTrackbar('H high', self.window_name, self._param_int('h_high1'), 179, self._noop)
        cv2.createTrackbar('S low', self.window_name, self._param_int('s_low'), 255, self._noop)
        cv2.createTrackbar('V low', self.window_name, self._param_int('v_low'), 255, self._noop)
        cv2.createTrackbar('Min area', self.window_name, self._param_int('min_area'), 500000, self._noop)
        cv2.createTrackbar('Max area', self.window_name, self._param_int('max_area'), 500000, self._noop)
        cv2.createTrackbar('Expand', self.window_name, self._param_int('expand'), 50, self._noop)

    def _param_int(self, name):
        return self.get_parameter(name).get_parameter_value().integer_value

    @staticmethod
    def _noop(_value):
        return

    def _read_ui_values(self):
        h_low = cv2.getTrackbarPos('H low', self.window_name)
        h_high = cv2.getTrackbarPos('H high', self.window_name)
        s_low = cv2.getTrackbarPos('S low', self.window_name)
        v_low = cv2.getTrackbarPos('V low', self.window_name)
        min_area = cv2.getTrackbarPos('Min area', self.window_name)
        max_area = cv2.getTrackbarPos('Max area', self.window_name)
        expand = cv2.getTrackbarPos('Expand', self.window_name)

        if h_high < h_low:
            h_high = h_low
            cv2.setTrackbarPos('H high', self.window_name, h_high)
        if max_area < min_area:
            max_area = min_area
            cv2.setTrackbarPos('Max area', self.window_name, max_area)

        return {
            'h_low1': h_low,
            'h_high1': h_high,
            's_low': s_low,
            'v_low': v_low,
            'min_area': min_area,
            'max_area': max_area,
            'expand': expand,
        }

    def _filter_mask(self, hsv_image, params):
        lower = np.array([params['h_low1'], params['s_low'], params['v_low']], dtype=np.uint8)
        upper = np.array([params['h_high1'], 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv_image, lower, upper)

        clean_kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, clean_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, clean_kernel)

        if params['expand'] > 0:
            kernel_size = params['expand'] * 2 + 1
            expand_kernel = np.ones((kernel_size, kernel_size), np.uint8)
            mask = cv2.dilate(mask, expand_kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        filtered_mask = np.zeros_like(mask)
        filtered_contours = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if params['min_area'] <= area <= params['max_area']:
                cv2.drawContours(filtered_mask, [contour], -1, 255, thickness=cv2.FILLED)
                filtered_contours.append(contour)

        return filtered_mask, filtered_contours

    @staticmethod
    def _draw_label(image, text):
        cv2.rectangle(image, (10, 10), (280, 42), (0, 0, 0), thickness=-1)
        cv2.putText(image, text, (18, 33), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        return image

    def _log_current_params(self, params):
        self.get_logger().info(
            '# HSV 參數設定（可調整）\n'
            f'self.h_low1 = {params["h_low1"]}\n'
            f'self.h_high1 = {params["h_high1"]}\n'
            f'self.s_low = {params["s_low"]}\n'
            f'self.v_low = {params["v_low"]}\n'
            f'self.min_area = {params["min_area"]}\n'
            f'self.max_area = {params["max_area"]}\n'
            f'self.expand = {params["expand"]}'
        )

    def image_callback(self, msg):
        try:
            bgr_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'影像轉換失敗: {exc}')
            return

        params = self._read_ui_values()
        hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        filtered_mask, contours = self._filter_mask(hsv_image, params)

        result_image = cv2.bitwise_and(bgr_image, bgr_image, mask=filtered_mask)
        contour_image = bgr_image.copy()
        if contours:
            cv2.drawContours(contour_image, contours, -1, (0, 255, 0), 2)

        mask_preview = cv2.cvtColor(filtered_mask, cv2.COLOR_GRAY2BGR)
        self._draw_label(contour_image, 'Original')
        self._draw_label(mask_preview, 'Mask')
        self._draw_label(result_image, 'Result')

        cv2.imshow(self.window_name, contour_image)
        cv2.imshow(self.mask_window_name, mask_preview)
        cv2.imshow(self.result_window_name, result_image)

        mask_msg = self.bridge.cv2_to_imgmsg(filtered_mask, encoding='mono8')
        mask_msg.header = msg.header
        self.mask_pub.publish(mask_msg)

        result_msg = self.bridge.cv2_to_imgmsg(result_image, encoding='bgr8')
        result_msg.header = msg.header
        self.result_pub.publish(result_msg)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            self._log_current_params(params)
        elif key == ord('q'):
            self.get_logger().info('收到 q，關閉 HSV 調參節點')
            rclpy.shutdown()

    def destroy_node(self):
        cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealsenseHsvTuner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
