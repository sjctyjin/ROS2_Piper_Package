#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import copy

class ImprovedDualArmJointRelayNode(Node):
    def __init__(self):
        super().__init__('dual_arm_joint_relay_node')
        
        # 定義所有關節名稱類別
        self.chassis_joint_names = ['Z', 'Y', 'X', 'R', 'G']
        self.wheel_joint_names = ['front_right_wheel', 'front_left_wheel', 'rear_left_wheel', 'rear_right_wheel']
        self.arm1_joint_names = ['arm1_joint1', 'arm1_joint2', 'arm1_joint3', 'arm1_joint4', 
                                'arm1_joint5', 'arm1_joint6', 'arm1_joint7', 'arm1_joint8']
        self.arm2_joint_names = ['arm2_joint1', 'arm2_joint2', 'arm2_joint3', 'arm2_joint4', 
                                'arm2_joint5', 'arm2_joint6', 'arm2_joint7', 'arm2_joint8']
        # 🆕 加入第三隻手臂的關節名稱
        self.arm3_joint_names = ['dummy_joint1', 'dummy_joint2', 'dummy_joint3', 
                                'dummy_joint4', 'dummy_joint5','dummy_joint6']
        # 合併的關節名稱列表
        self.combined_joint_names = self.arm1_joint_names + self.arm2_joint_names + self.arm3_joint_names 
        
        # 完整的機器人關節名稱列表
        #self.all_joint_names = self.chassis_joint_names + self.wheel_joint_names + self.combined_joint_names
        self.all_joint_names = self.combined_joint_names
        # 初始化完整的關節狀態
        self.complete_joint_state = JointState()
        self.complete_joint_state.name = self.all_joint_names
        self.complete_joint_state.position = [0.0] * len(self.all_joint_names)
        self.complete_joint_state.velocity = [0.0] * len(self.all_joint_names)
        self.complete_joint_state.effort = [0.0] * len(self.all_joint_names)
        
        # 發布者: 為每個手臂創建獨立的發布者，以及合併的 joint_states 發布者
        self.arm1_publisher = self.create_publisher(JointState, '/arm1/joint_states', 10)
        self.arm2_publisher = self.create_publisher(JointState, '/arm2/joint_states', 10)     
           
        self.arm3_publisher = self.create_publisher(JointState, '/arm3/joint_states', 10)
        self.joint_states_publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.joint_states_publisher_cus = self.create_publisher(JointState, '/joint_custom_state', 10)
        
        # 訂閱者: 分別訂閱每個手臂的自定義關節命令
        self.subscription_arm1_custom = self.create_subscription(
            JointState,
            '/arm1/joint_custom_state',
            self.arm1_custom_joint_callback,
            10
        )
        
        self.subscription_arm2_custom = self.create_subscription(
            JointState,
            '/arm2/joint_custom_state',
            self.arm2_custom_joint_callback,
            10
        )
        # 🆕 第三隻手臂的訂閱者
        self.subscription_arm3_custom = self.create_subscription(
            JointState,
            '/arm3/joint_custom_state',
            self.arm3_custom_joint_callback,
            10
        )
        # 向後兼容: 訂閱原始的 joint_custom_state (用於一次控制所有關節)
        self.subscription_combined_custom = self.create_subscription(
            JointState,
            '/joint_custom_state',
            self.combined_custom_joint_callback,
            10
        )
        
        # 訂閱每個手臂的 joint_states 以合併
        #self.subscription_arm1_states = self.create_subscription(
        #    JointState,
        #    '/arm1/joint_states',
        #    self.arm1_states_callback,
        #    10
        #)
        
        #self.subscription_arm2_states = self.create_subscription(
        #    JointState,
        #    '/arm2/joint_states',
        #    self.arm2_states_callback,
        #    10
        #)
        
        # 跟踪最新的手臂狀態
        self.latest_arm1_state = None
        self.latest_arm2_state = None
        # 🆕 第三隻手臂的狀態
        self.latest_arm3_state = None
        #
        
        # 初始化的動作
        self.initial_position_sent = False
        self.init_timer = 0
        self.create_timer(0.5, self.publish_initial_joint_state)
        #初始化定時器
        self.timer_period = 0.02  # 50Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info('✅ 改進的雙臂關節轉發節點已啟動')
        
    def publish_initial_joint_state(self):
        """發布完整的初始關節狀態，包含底盤、輪子和雙臂"""
        if self.initial_position_sent:
            return
        
        # 創建完整的關節狀態消息
        complete_msg = JointState()
        complete_msg.header.stamp = self.get_clock().now().to_msg()
        complete_msg.name = []
        complete_msg.position = []
        complete_msg.velocity = []
        complete_msg.effort = []
        
        # 添加底盤關節
        chassis_positions = [0.0, 0.0, 0.0, 0.0, 0.0]  # Z, Y, X, R, G 的初始位置
        complete_msg.name.extend(self.chassis_joint_names)
        complete_msg.position.extend(chassis_positions)
        complete_msg.velocity.extend([0.0] * len(self.chassis_joint_names))
        complete_msg.effort.extend([0.0] * len(self.chassis_joint_names))
        
        # 添加輪子關節
        wheel_positions = [0.0, 0.0, 0.0, 0.0]  # 四個輪子的初始位置
        complete_msg.name.extend(self.wheel_joint_names)
        complete_msg.position.extend(wheel_positions)
        complete_msg.velocity.extend([0.0] * len(self.wheel_joint_names))
        complete_msg.effort.extend([0.0] * len(self.wheel_joint_names))
        
        # 添加左臂關節
        arm1_positions = [0.2, 0.40, -0.8, 0.0, 0.5, 0.0, -0.04, 0.04]  # 左臂初始位置
        complete_msg.name.extend(self.arm1_joint_names)
        complete_msg.position.extend(arm1_positions)
        complete_msg.velocity.extend([0.0] * len(self.arm1_joint_names))
        complete_msg.effort.extend([0.0] * len(self.arm1_joint_names))
        
        # 添加右臂關節
        arm2_positions = [-0.2, 0.40, -0.8, 0.0, 0.5, 0.0, -0.04, 0.04]  # 右臂初始位置
        complete_msg.name.extend(self.arm2_joint_names)
        complete_msg.position.extend(arm2_positions)
        complete_msg.velocity.extend([0.0] * len(self.arm2_joint_names))
        complete_msg.effort.extend([0.0] * len(self.arm2_joint_names))
        
        # 🆕 添加第三隻手臂關節
        arm3_positions = [0.0, 0.0, 0.0, 0.0, 0.0,0.0]  # 第三隻手臂初始位置 (5個關節)
        complete_msg.name.extend(self.arm3_joint_names)
        complete_msg.position.extend(arm3_positions)
        complete_msg.velocity.extend([0.0] * len(self.arm3_joint_names))
        complete_msg.effort.extend([0.0] * len(self.arm3_joint_names))
        
        # 發布完整的關節狀態到 /joint_states
        self.joint_states_publisher.publish(complete_msg)
        self.joint_states_publisher_cus.publish(complete_msg)
        
        # 同時更新內部儲存的完整狀態
        self.complete_joint_state.header.stamp = self.get_clock().now().to_msg()
        self.complete_joint_state.name = complete_msg.name
        self.complete_joint_state.position = complete_msg.position
        self.complete_joint_state.velocity = complete_msg.velocity
        self.complete_joint_state.effort = complete_msg.effort
        
        # 分別發布到兩個手臂的話題
        arm1_msg = JointState()
        arm1_msg.header.stamp = self.get_clock().now().to_msg()
        arm1_msg.name = self.arm1_joint_names
        arm1_msg.position = arm1_positions
        arm1_msg.velocity = [0.0] * len(self.arm1_joint_names)
        arm1_msg.effort = [0.0] * len(self.arm1_joint_names)
        self.arm1_publisher.publish(arm1_msg)
        
        arm2_msg = JointState()
        arm2_msg.header.stamp = self.get_clock().now().to_msg()
        arm2_msg.name = self.arm2_joint_names
        arm2_msg.position = arm2_positions
        arm2_msg.velocity = [0.0] * len(self.arm2_joint_names)
        arm2_msg.effort = [0.0] * len(self.arm2_joint_names)
        self.arm2_publisher.publish(arm2_msg)
        
        # 🆕 第三隻手臂的初始狀態
        arm3_msg = JointState()
        arm3_msg.header.stamp = self.get_clock().now().to_msg()
        arm3_msg.name = self.arm3_joint_names
        arm3_msg.position = arm3_positions
        arm3_msg.velocity = [0.0] * len(self.arm3_joint_names)
        arm3_msg.effort = [0.0] * len(self.arm3_joint_names)
        self.arm3_publisher.publish(arm3_msg)
        
        self.init_timer += 1
        self.get_logger().info(f"🚀 已發送{self.init_timer}次完整的初始關節狀態")
        
        if self.init_timer >= 4:
            self.initial_position_sent = True
            self.get_logger().info("✅ 初始化完成，不再發送初始狀態")
    
    def update_joint_state(self, msg: JointState):
        """更新完整的關節狀態，只更新消息中包含的關節"""
        for i, name in enumerate(msg.name):
            if name in self.complete_joint_state.name:
                idx = self.complete_joint_state.name.index(name)
                
                # 更新位置
                if i < len(msg.position):
                    self.complete_joint_state.position[idx] = msg.position[i]
                
                # 更新速度（如果有）
                if msg.velocity and i < len(msg.velocity):
                    self.complete_joint_state.velocity[idx] = msg.velocity[i]
                    
                # 更新力矩（如果有）
                if msg.effort and i < len(msg.effort):
                    self.complete_joint_state.effort[idx] = msg.effort[i]
    
    def arm1_custom_joint_callback(self, msg: JointState):
        """處理左臂的自定義關節命令"""
        # 創建一個新的消息，添加前綴（如果需要）
        arm1_msg = JointState()
        arm1_msg.header.stamp = self.get_clock().now().to_msg()
        
        # 添加前綴（如果需要）
        arm1_msg.name = []
        arm1_msg.position = []
        
        for i, name in enumerate(msg.name):
            # 如果沒有前綴，添加前綴
            if not name.startswith('arm1_'):
                prefixed_name = f'arm1_{name}'
            else:
                prefixed_name = name
                
            arm1_msg.name.append(prefixed_name)
            
            if i < len(msg.position):
                arm1_msg.position.append(msg.position[i])
        
        # 確保有速度和力矩字段
        if msg.velocity:
            arm1_msg.velocity = msg.velocity
        else:
            arm1_msg.velocity = [0.0] * len(arm1_msg.name)
            
        if msg.effort:
            arm1_msg.effort = msg.effort
        else:
            arm1_msg.effort = [0.0] * len(arm1_msg.name)
            
        self.latest_arm1_state = arm1_msg
        # 更新完整狀態
        self.update_joint_state(arm1_msg)
        
        # 發送到左臂
        #self.arm1_publisher.publish(arm1_msg)
        
        self.get_logger().debug(f"🔄 轉發左臂自定義關節命令: {arm1_msg.name}")
    
    def arm2_custom_joint_callback(self, msg: JointState):
        """處理右臂的自定義關節命令"""
        # 創建一個新的消息，添加前綴（如果需要）
        arm2_msg = JointState()
        arm2_msg.header.stamp = self.get_clock().now().to_msg()
        
        # 添加前綴（如果需要）
        arm2_msg.name = []
        arm2_msg.position = []
        
        for i, name in enumerate(msg.name):
            # 如果沒有前綴，添加前綴
            if not name.startswith('arm2_'):
                prefixed_name = f'arm2_{name}'
            else:
                prefixed_name = name
                
            arm2_msg.name.append(prefixed_name)
            
            if i < len(msg.position):
                arm2_msg.position.append(msg.position[i])
        
        # 確保有速度和力矩字段
        if msg.velocity:
            arm2_msg.velocity = msg.velocity
        else:
            arm2_msg.velocity = [0.0] * len(arm2_msg.name)
            
        if msg.effort:
            arm2_msg.effort = msg.effort
        else:
            arm2_msg.effort = [0.0] * len(arm2_msg.name)
        self.latest_arm2_state = arm2_msg
        # 更新完整狀態
        self.update_joint_state(arm2_msg)
        
        # 發送到右臂
        #self.arm2_publisher.publish(arm2_msg)
        
        self.get_logger().debug(f"🔄 轉發右臂自定義關節命令: {arm2_msg.name}")
    # 🆕 第三隻手臂的回調函數
    def arm3_custom_joint_callback(self, msg: JointState):
        """處理第三隻手臂的自定義關節命令"""
        arm3_msg = JointState()
        arm3_msg.header.stamp = self.get_clock().now().to_msg()
        arm3_msg.name = []
        arm3_msg.position = []
        
        for i, name in enumerate(msg.name):
            # 處理關節名稱，支援 'Revolute 1' 或 'arm3_Revolute_1' 格式
            if not name.startswith('arm3_'):
                # 將空格替換為底線並加上前綴
                clean_name = name.replace(' ', '_')
                prefixed_name = f'arm3_{clean_name}'
            else:
                prefixed_name = name
                
            arm3_msg.name.append(prefixed_name)
            
            if i < len(msg.position):
                arm3_msg.position.append(msg.position[i])
        
        if msg.velocity:
            arm3_msg.velocity = msg.velocity
        else:
            arm3_msg.velocity = [0.0] * len(arm3_msg.name)
            
        if msg.effort:
            arm3_msg.effort = msg.effort
        else:
            arm3_msg.effort = [0.0] * len(arm3_msg.name)
            
        self.latest_arm3_state = arm3_msg
        self.update_joint_state(arm3_msg)
        self.get_logger().debug(f"🔄 轉發第三隻手臂自定義關節命令: {arm3_msg.name}")
        
    def combined_custom_joint_callback(self, msg: JointState):
        """處理合併的自定義關節命令 (控制所有關節，包括底盤、輪子和雙臂)"""
        # 先更新完整的關節狀態，確保所有關節都被保留
        self.update_joint_state(msg)
        
        # 創建左臂、右臂的訊息
        arm1_msg = JointState()
        arm1_msg.header.stamp = self.get_clock().now().to_msg()
        arm1_msg.name = []
        arm1_msg.position = []
        arm1_msg.velocity = []
        arm1_msg.effort = []
        
        arm2_msg = JointState()
        arm2_msg.header.stamp = self.get_clock().now().to_msg()
        arm2_msg.name = []
        arm2_msg.position = []
        arm2_msg.velocity = []
        arm2_msg.effort = []
        # 🆕 第三隻手臂的訊息
        arm3_msg = JointState()
        arm3_msg.header.stamp = self.get_clock().now().to_msg()
        arm3_msg.name = []
        arm3_msg.position = []
        arm3_msg.velocity = []
        arm3_msg.effort = []
        # 分類關節
        for i, name in enumerate(msg.name):
            # 左臂關節
            if name.startswith('arm1_') or (name.startswith('joint') and not name.startswith('joint9') and len(name) <= 6):
                # 如果是無前綴的關節名，添加前綴
                if name.startswith('joint'):
                    arm_name = f'arm1_{name}'
                else:
                    arm_name = name
                    
                arm1_msg.name.append(arm_name)
                if i < len(msg.position):
                    arm1_msg.position.append(msg.position[i])
                if msg.velocity and i < len(msg.velocity):
                    arm1_msg.velocity.append(msg.velocity[i])
                if msg.effort and i < len(msg.effort):
                    arm1_msg.effort.append(msg.effort[i])
            
            # 右臂關節
            elif name.startswith('arm2_') or name.startswith('joint9'):
                # 如果是無前綴的關節名，添加前綴
                if name.startswith('joint'):
                    arm_name = f'arm2_{name}'
                else:
                    arm_name = name
                    
                arm2_msg.name.append(arm_name)
                if i < len(msg.position):
                    arm2_msg.position.append(msg.position[i])
                if msg.velocity and i < len(msg.velocity):
                    arm2_msg.velocity.append(msg.velocity[i])
                if msg.effort and i < len(msg.effort):
                    arm2_msg.effort.append(msg.effort[i])
            # 🆕 第三隻手臂關節
            elif name.startswith('arm3_') or name.startswith('Revolute'):
                if name.startswith('Revolute'):
                    clean_name = name.replace(' ', '_')
                    arm_name = f'arm3_{clean_name}'
                else:
                    arm_name = name
                    
                arm3_msg.name.append(arm_name)
                if i < len(msg.position):
                    arm3_msg.position.append(msg.position[i])
                if msg.velocity and i < len(msg.velocity):
                    arm3_msg.velocity.append(msg.velocity[i])
                if msg.effort and i < len(msg.effort):
                    arm3_msg.effort.append(msg.effort[i]) 
        
        # 補充缺失的字段
        def fill_missing_fields(msg):
            if msg.name and not msg.velocity:
                msg.velocity = [0.0] * len(msg.name)
            if msg.name and not msg.effort:
                msg.effort = [0.0] * len(msg.name)
            return msg
        
        arm1_msg = fill_missing_fields(arm1_msg)
        arm2_msg = fill_missing_fields(arm2_msg)
        arm3_msg = fill_missing_fields(arm3_msg)
        
        # 發布到各自的 joint_states 話題
        if arm1_msg.name:
        #    self.arm1_publisher.publish(arm1_msg)
            self.latest_arm1_state = arm1_msg
        #    self.get_logger().debug(f"🔄 發送到左臂: {len(arm1_msg.name)}個關節")
        
        if arm2_msg.name:
            self.arm2_publisher.publish(arm2_msg)
            self.latest_arm2_state = arm2_msg
        #    self.get_logger().debug(f"🔄 發送到右臂: {len(arm2_msg.name)}個關節")
        
        # 🆕 第三隻手臂的發布
        if arm3_msg.name:
            self.latest_arm3_state = arm3_msg
            
        # 發布完整狀態到 /joint_states
        self.joint_states_publisher.publish(self.complete_joint_state)
    
    def timer_callback(self):
        """定時發布完整的機器人關節狀態，包括底盤、輪子和雙臂"""
        # 確保有狀態可以發布
        if not hasattr(self, 'complete_joint_state') or len(self.complete_joint_state.name) == 0:
            return
        
        # 更新時間戳
        self.complete_joint_state.header.stamp = self.get_clock().now().to_msg()
        
        # 發布完整的關節狀態
        self.joint_states_publisher.publish(self.complete_joint_state)
        
        if self.latest_arm1_state:        
            self.arm1_publisher.publish(self.latest_arm1_state)
        
        if self.latest_arm2_state:        
            self.arm2_publisher.publish(self.latest_arm2_state)
            
        # 🆕 第三隻手臂的定時發布
        if self.latest_arm3_state:
            self.arm3_publisher.publish(self.latest_arm3_state)
            
        
        # 可選的調試日誌
        if self.get_parameter('log_level').get_parameter_value().string_value.upper() == 'DEBUG':
            arm1_joints = [j for j in self.complete_joint_state.name if j.startswith('arm1_')]
            arm2_joints = [j for j in self.complete_joint_state.name if j.startswith('arm2_')]
            # 🆕
            arm3_joints = [j for j in self.complete_joint_state.name if j.startswith('arm3_')]  

            chassis_joints = [j for j in self.complete_joint_state.name if j in self.chassis_joint_names]
            wheel_joints = [j for j in self.complete_joint_state.name if j in self.wheel_joint_names]
            
            self.get_logger().debug(
                f"📊 定時發布完整狀態: 總計{len(self.complete_joint_state.name)}個關節 "
                f"(底盤:{len(chassis_joints)}, 輪子:{len(wheel_joints)}, "
                f"左臂:{len(arm1_joints)}, 右臂:{len(arm2_joints)})"#, 第三臂:{len(arm3_joints)})"  # 🆕
            )


def main(args=None):
    rclpy.init(args=args)
    
    # 創建節點時添加默認參數
    node = ImprovedDualArmJointRelayNode()
    node.declare_parameter('log_level', 'info')
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
