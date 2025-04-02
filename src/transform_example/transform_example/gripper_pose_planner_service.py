import rclpy
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from sensor_msgs.msg import JointState
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
from rclpy.executors import MultiThreadedExecutor
import time

class GripperTest(Node):
    def __init__(self):
        super().__init__('gripper_test')
        
        # 建立 MoveGroup Action Client
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')
        
        # 設定夾爪規劃組和關節名稱 (根據您的配置修改)
        self.gripper_planning_group = 'gripper'
        self.gripper_joint_name = 'joint7'
        
        # 夾爪目標位置
        self.open_position = 0.8  # 開啟
        self.close_position = 0.002    # 關閉
        
        # 訂閱關節狀態（可選，用來追蹤當前狀態）
        self.joint_states = {}
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )
        
        # 創建服務：開啟夾爪和關閉夾爪
        self.open_service = self.create_service(
            Trigger,
            'open_gripper',
            self.open_gripper_callback
        )
        self.close_service = self.create_service(
            Trigger,
            'close_gripper',
            self.close_gripper_callback
        )
        
        self.get_logger().info('Gripper test node initialized.')
        self.get_logger().info('Available services: /open_gripper, /close_gripper')
    
    def joint_state_callback(self, msg):
        """更新 joint_states"""
        for i, name in enumerate(msg.name):
            self.joint_states[name] = msg.position[i]
    
    def open_gripper_callback(self, request, response):
        self.get_logger().info('Received open gripper request')
        success = self.move_gripper(self.open_position)
        #response.success = success
        response.message = "Open gripper succeeded" if success else "Open gripper failed"
        return response
    
    def close_gripper_callback(self, request, response):
        self.get_logger().info('Received close gripper request')
        success = self.move_gripper(self.close_position)
        #response.success = success
        response.message = "Close gripper succeeded" if success else "Close gripper failed"
        return response
    
    def move_gripper(self, target_position):
        try:
            self.get_logger().info(f'Moving gripper to position: {target_position}')
            
            # 等待 MoveIt 伺服器（增加等待時間以提高成功率）
            self.get_logger().info('Waiting for MoveIt action server...')
            if not self.move_group_client.wait_for_server(timeout_sec=10.0):
                self.get_logger().error('MoveIt action server not available')
                return False
            
            # 建立 MoveGroup 請求
            goal_msg = MoveGroup.Goal()
            goal_msg.request.group_name = self.gripper_planning_group
            goal_msg.request.workspace_parameters.header.frame_id = 'base_link'
            goal_msg.request.allowed_planning_time = 5.0
            goal_msg.request.num_planning_attempts = 5
            
            # 設定起始狀態為當前狀態
            goal_msg.request.start_state.is_diff = True
            
            # 設定目標關節約束：針對 gripper_joint_name 進行限制
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = self.gripper_joint_name
            joint_constraint.position = target_position
            joint_constraint.tolerance_above = 0.0005
            joint_constraint.tolerance_below = 0.0005
            joint_constraint.weight = 1.0
            
            constraints = Constraints()
            constraints.joint_constraints.append(joint_constraint)
            goal_msg.request.goal_constraints.append(constraints)
            
            self.get_logger().info(f'Sending gripper move request (target: {target_position})...')
            future = self.move_group_client.send_goal_async(goal_msg)
            time.sleep(3)
            #rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            self.get_logger().info('五秒時間到')
            return True
            
          
            
        except Exception as e:
            self.get_logger().error(f'Exception during gripper move: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            return False

def main(args=None):
    rclpy.init(args=args)
    node = GripperTest()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        node.get_logger().info('Gripper test node running. Waiting for service calls...')
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

