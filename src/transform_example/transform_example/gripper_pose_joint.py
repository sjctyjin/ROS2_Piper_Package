#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import time

class GripperServiceNode(Node):
    def __init__(self):
        super().__init__('gripper_service_node')
        
        self.gripper_joint_name = 'joint7'
        self.gripper_open_value = 0.3
        self.gripper_close_value = 0.0
        
        self.joint_states_pub = self.create_publisher(JointState, '/joint_custom_state', 10)
        self.current_joint_state = None
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # ✅ 建立兩個獨立的 Trigger 服務
        self.create_service(Trigger, 'open_gripper', self.open_gripper_callback)
        self.create_service(Trigger, 'close_gripper', self.close_gripper_callback)
        
        self.get_logger().info('等待接收当前关节状态...')
        self._wait_for_joint_state()
        self.get_logger().info('夹爪 Trigger 服务已启动')
    
    def joint_state_callback(self, msg):
        self.current_joint_state = msg
        
    def _wait_for_joint_state(self, timeout=5.0):
        start_time = time.time()
        while self.current_joint_state is None and time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.current_joint_state is None:
            self.get_logger().warn('等待关节状态超时!')

    def open_gripper_callback(self, request, response):
        success = self.set_gripper_position(self.gripper_open_value)
        response.success = success
        response.message = "夹爪已打开" if success else "打开夹爪失败"
        return response

    def close_gripper_callback(self, request, response):
        success = self.set_gripper_position(self.gripper_close_value)
        response.success = success
        response.message = "夹爪已关闭" if success else "关闭夹爪失败"
        return response

    def set_gripper_position(self, position):
        if self.current_joint_state is None:
            self.get_logger().error('没有当前关节状态，无法控制夹爪')
            return False
        
        try:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(self.current_joint_state.name)
            msg.position = list(self.current_joint_state.position)
            msg.velocity = [0.0] * len(msg.name)
            msg.effort = [0.0] * len(msg.name)

            if self.gripper_joint_name in msg.name:
                idx = msg.name.index(self.gripper_joint_name)
                original = msg.position[idx]
                msg.position[idx] = position
                self.get_logger().info(f'设置夹爪位置: {original:.4f} → {position:.4f}')
            else:
                self.get_logger().error(f'找不到关节 "{self.gripper_joint_name}"')
                return False

            for _ in range(5):
                self.joint_states_pub.publish(msg)
                time.sleep(0.05)

            return True
        except Exception as e:
            self.get_logger().error(f'设置夹爪位置时出错: {e}')
            return False

def main(args=None):
    rclpy.init(args=args)
    node = GripperServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

