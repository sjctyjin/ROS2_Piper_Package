#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

import time

class GripperServiceNode(Node):
    """提供夹爪控制服务的节点"""
    
    def __init__(self):
        super().__init__('gripper_service_node')
        
        # 夹爪参数
        self.gripper_joint_name = 'gripper'  # 夹爪关节名称
        self.gripper_open_value = 0.3       # 夹爪打开位置
        self.gripper_close_value = 0.0      # 夹爪关闭位置
        
        # 创建发布器
        self.joint_states_pub = self.create_publisher(JointState, '/joint_states_single', 10)
        
        # 订阅当前关节状态
        self.current_joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states_single',
            self.joint_state_callback,
            10
        )
        
        # 创建服务
        self.open_close_srv = self.create_service(
            SetBool, 
            'gripper/set_open', 
            self.open_close_callback
        )
        
        self.get_logger().info('等待接收当前关节状态...')
        self._wait_for_joint_state()
        self.get_logger().info('夹爪服务已启动!')
        self.get_logger().info('可用服务:')
        self.get_logger().info(' - gripper/set_open (std_srvs/SetBool) - 设置开关状态 (data:true=打开,false=关闭)')
        self.get_logger().info(' - gripper/set_position (example_interfaces/SetFloat64) - 设置具体位置')
        
    def joint_state_callback(self, msg):
        """更新当前关节状态"""
        self.current_joint_state = msg
        
    def _wait_for_joint_state(self, timeout=5.0):
        """等待接收关节状态"""
        start_time = time.time()
        rate = self.create_rate(10)  # 10Hz
        
        while self.current_joint_state is None:
            if time.time() - start_time > timeout:
                self.get_logger().warn('等待关节状态超时!')
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
            
        return True
        
    def open_close_callback(self, request, response):
        """处理开关服务调用"""
        if request.data:
            # 打开夹爪
            success = self.set_gripper_position(self.gripper_open_value)
            response.success = success
            response.message = "夹爪已打开" if success else "打开夹爪失败"
        else:
            # 关闭夹爪
            success = self.set_gripper_position(self.gripper_close_value)
            response.success = success
            response.message = "夹爪已关闭" if success else "关闭夹爪失败"
            
        return response
    
    def position_callback(self, request, response):
        """处理位置设置服务调用"""
        position = request.data
        success = self.set_gripper_position(position)
        response.success = success
        response.message = f"夹爪位置已设置为 {position}" if success else f"设置夹爪位置 {position} 失败"
        
        return response
        
    def set_gripper_position(self, position):
        """设置夹爪位置"""
        if self.current_joint_state is None:
            self.get_logger().error('没有当前关节状态，无法控制夹爪')
            return False
            
        try:
            # 创建新的关节状态消息
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(self.current_joint_state.name)
            msg.position = list(self.current_joint_state.position)
            msg.velocity = [0.0] * len(msg.name)
            msg.effort = [0.0] * len(msg.name)
            
            # 查找夹爪关节索引
            if self.gripper_joint_name in msg.name:
                gripper_index = msg.name.index(self.gripper_joint_name)
                # 更新夹爪位置
                original_position = msg.position[gripper_index]
                msg.position[gripper_index] = position
                self.get_logger().info(f'设置夹爪位置: {original_position} -> {position}')
            else:
                self.get_logger().error(f'关节 {self.gripper_joint_name} 不在当前状态中')
                return False
                
            # 发布消息 (多次发布以确保接收)
            for _ in range(5):
                self.joint_states_pub.publish(msg)
                time.sleep(0.05)
                
            return True
                
        except Exception as e:
            self.get_logger().error(f'设置夹爪位置时出错: {str(e)}')
            return False

def main(args=None):
    rclpy.init(args=args)
    node = GripperServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
