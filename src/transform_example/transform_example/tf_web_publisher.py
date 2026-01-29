# tf_republisher_node.py (ROS2版)
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
import time

class TFRepublisher(Node):
    def __init__(self):
        super().__init__('tf_republisher')

        self.timer = self.create_timer(0.1, self.T1)
        self.timer2 = self.create_timer(0.1, self.T2)
        self.real_subscription = self.create_subscription(
            JointState,
            '/joint_states_single',
            self.joint_callback,
            10
        )
        self.test1 = 0
        self.test2 = 0
        self.pick_check = 0
        self.pre_pick_check = -1
        self.gripper_data_event = True
    def T1(self):
        # 轉發給 /web_tf topic
        #while self.gripper_data_event:
        
        self.get_logger().info('這是線程--1')
          #  self.get_logger().info(f'這是線程--1{self.pick_check}')
        timeout = 2.0  # 最多等待秒數
        start_time = self.get_clock().now()
        
        while self.gripper_data_event:
            rclpy.spin_once(self, timeout_sec=0.05)  # 允許 callback 執行
            now = self.get_clock().now()
            if (now - start_time).nanoseconds / 1e9 > timeout:
                self.get_logger().warn("等待 gripper_data_event 超時")
                break      
        time.sleep(1)
        
    def T2(self):
        # 轉發給 /web_tf topic
        self.get_logger().info('這是線程2')
        if self.pre_pick_check != self.pick_check:
                self.pre_pick_check = self.pick_check
                self.get_logger().info('這是線程')
                self.get_logger().info(f'這是線程--{self.pick_check}')
                self.gripper_data_event = False
        
    def joint_callback(self, msg: JointState):
        # 取得 gripper 的位置
        self.joint_state_received = True
        try:
            gripper_index = msg.name.index('gripper')  # 找到 gripper 在 name 中的索引
            gripper_position = msg.position[gripper_index]  # 取得對應位置
            self.get_logger().info(f'Gripper position: {gripper_position:.4f}')

            # 根據 gripper 開口程度判斷是否抓取成功（依據你的實際值調整閾值）
            if abs(gripper_position) > 0.02:
                #self.get_logger().info("✅ 夾取成功（Gripper 關閉）")
                self.pick_check = 1
            else:
                #self.get_logger().info("❌ 可能未成功夾取（Gripper 打開）")
                self.pick_check = 0

        except ValueError:
            self.get_logger().warn("找不到 'gripper' 關節名稱")
            

def main(args=None):
    rclpy.init(args=args)
    node = TFRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

