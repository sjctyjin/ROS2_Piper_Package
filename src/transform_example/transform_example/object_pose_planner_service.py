import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import RobotState, JointConstraint
from sensor_msgs.msg import JointState
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
import tf2_ros
import numpy as np
import time

class MoveItTFPlanner(Node):
    def __init__(self):
        super().__init__('moveit_tf_planner')

        # MoveGroup Action Client
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')

        # 設定規劃組的名稱
        self.arm_planning_group = 'arm'
        self.gripper_planning_group = 'gripper'  # 假設夾爪的規劃組名稱為 'gripper'
        
        # 設定夾爪關節名稱和位置值
        self.gripper_joint_name = 'joint7'  # 夾爪關節名稱
        self.gripper_open_value = 0.3    # 夾爪打開位置值
        self.gripper_close_value = 0.0     # 夾爪關閉位置值

        # TF2 Buffer 和 Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 創建觸發服務
        self.trigger_service = self.create_service(
            Trigger,
            'trigger_plan',
            self.trigger_callback
        )

        # 獲取機器人當前狀態的訂閱者
        self.robot_state = None
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        self.get_logger().info('MoveIt TF Planner 已初始化，等待觸發服務呼叫...')

    def joint_state_callback(self, msg):
        """更新機器人關節狀態"""
        self.robot_state = msg

    def trigger_callback(self, request, response):
        """當收到觸發請求時執行規劃"""
        self.get_logger().info('收到觸發請求，開始規劃...')

        try:
            # 1. 首先讓夾爪打開
            self.get_logger().info('步驟 1: 打開夾爪...')
            gripper_success = self.move_gripper(self.gripper_open_value)
            
            if not gripper_success:
                response.success = False
                response.message = "打開夾爪失敗"
                return response
                
            # 2. 處理目標姿態並執行手臂運動規劃
            self.get_logger().info('步驟 2: 移動手臂到目標位置...')
            arm_success = self.process_target_pose()
            
            if not arm_success:
                response.success = False
                response.message = "手臂移動規劃失敗"
                return response
                
            # 3. 關閉夾爪抓取物體
            self.get_logger().info('步驟 3: 關閉夾爪抓取物體...')
            gripper_close_success = self.move_gripper(self.gripper_close_value)
            
            if not gripper_close_success:
                response.success = False
                response.message = "關閉夾爪失敗"
                return response

            response.success = True
            response.message = "完整運動序列已成功執行"

        except Exception as e:
            self.get_logger().error(f"規劃過程中出現錯誤: {str(e)}")
            response.success = False
            response.message = f"錯誤: {str(e)}"

        return response

    def move_gripper(self, target_value):
        """移動夾爪到指定位置"""
        try:
            self.get_logger().info(f'移動夾爪到位置: {target_value}')
            
            # 創建 MoveGroup 請求
            goal_msg = MoveGroup.Goal()
            goal_msg.request.group_name = self.gripper_planning_group
            goal_msg.request.workspace_parameters.header.frame_id = 'base_link'
            goal_msg.request.allowed_planning_time = 2.0
            goal_msg.request.num_planning_attempts = 5
            
            # 設定目標關節位置約束
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = self.gripper_joint_name
            joint_constraint.position = target_value
            joint_constraint.tolerance_above = 0.005
            joint_constraint.tolerance_below = 0.005
            joint_constraint.weight = 1.0
            
            # 添加關節約束
            from moveit_msgs.msg import Constraints
            constraints = Constraints()
            constraints.joint_constraints.append(joint_constraint)
            goal_msg.request.goal_constraints.append(constraints)
            
            # 發送目標到 MoveIt
            if not self.move_group_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('MoveIt 動作伺服器未響應，無法發送夾爪規劃請求')
                return False
                
            self.get_logger().info(f'發送夾爪移動請求(位置: {target_value})...')
            
            
            # 同步發送和等待結果
            future = self.move_group_client.send_goal_async(goal_msg)
            time.sleep(0.8)
            self.get_logger().info('夾爪移動成功執行!')
            return True
           
                
        except Exception as e:
            self.get_logger().error(f"夾爪移動過程中出現錯誤: {str(e)}")
            return False

    def process_target_pose(self):
        """監聽 TF 並組合目標 Pose，返回處理結果"""
        try:
            # 監聽 object_grasp_frame 的座標
            self.get_logger().info('查詢目標物體 TF...')
            object_transform = self.tf_buffer.lookup_transform(
                'base_link', 'object_in_base', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)
            )

            # 監聽機器人末端執行器的姿態
            self.get_logger().info('查詢機器人末端執行器 TF...')
            end_effector_transform = self.tf_buffer.lookup_transform(
                'base_link', 'link6', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)
            )

            # 組合目標 Pose
            target_pose = PoseStamped()
            target_pose.header.frame_id = 'base_link'
            target_pose.header.stamp = self.get_clock().now().to_msg()

            # 使用目標物體的位置
            target_pose.pose.position.x = object_transform.transform.translation.x
            target_pose.pose.position.y = object_transform.transform.translation.y
            target_pose.pose.position.z = object_transform.transform.translation.z

            # 使用目標物體的姿態
            target_pose.pose.orientation = object_transform.transform.rotation

            self.get_logger().info(
                f"目標位置: [{target_pose.pose.position.x:.3f}, {target_pose.pose.position.y:.3f}, {target_pose.pose.position.z:.3f}]")

            # 發送目標到 MoveIt
            return self.send_target_goal(target_pose)

        except tf2_ros.TransformException as e:
            self.get_logger().error(f"無法獲取 TF 轉換: {str(e)}")
            return False
        except Exception as e:
            self.get_logger().error(f"處理目標姿態時出錯: {str(e)}")
            return False

    def send_target_goal(self, target_pose):
        """發送目標座標到 MoveIt 規劃器"""
        # 創建 MoveGroup 請求
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.arm_planning_group
        goal_msg.request.workspace_parameters.header.frame_id = 'base_link'
        goal_msg.request.workspace_parameters.min_corner.x = -1.0
        goal_msg.request.workspace_parameters.min_corner.y = -1.0
        goal_msg.request.workspace_parameters.min_corner.z = -1.0
        goal_msg.request.workspace_parameters.max_corner.x = 1.0
        goal_msg.request.workspace_parameters.max_corner.y = 1.0
        goal_msg.request.workspace_parameters.max_corner.z = 1.0
        goal_msg.request.allowed_planning_time = 5.0  # 增加到5秒
        goal_msg.request.num_planning_attempts = 10   # 增加嘗試次數
        # 設定目標約束
        goal_msg.request.goal_constraints.append(self.create_goal_constraints(target_pose))

        # 發送目標到 MoveIt
        self.get_logger().info('等待 MoveIt 動作伺服器...')

        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveIt 動作伺服器未響應，無法發送規劃請求')
            return False

        self.get_logger().info('發送手臂運動規劃請求到 MoveIt...')

        # 同步發送和等待結果
        future = self.move_group_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        return True
        
        try:
            result = result_future.result().result
            if result.error_code.val == result.error_code.SUCCESS:
                self.get_logger().info('手臂运动规划成功执行!')
                return True
            else:
                self.get_logger().error(f'手臂运动规划失败，错误码: {result.error_code.val}')
                return False
        except Exception as e:
            self.get_logger().error(f'获取结果时出错: {str(e)}')
            return False

    def create_goal_constraints(self, target_pose):
        """創建目標的位姿約束"""
        from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
        from shape_msgs.msg import SolidPrimitive

        constraints = Constraints()

        # Position Constraint
        position_constraint = PositionConstraint()
        position_constraint.header = target_pose.header
        position_constraint.link_name = 'link6'
        position_constraint.target_point_offset.x = 0.0
        position_constraint.target_point_offset.y = 0.0
        position_constraint.target_point_offset.z = 0.0

        bounding_box = SolidPrimitive()
        bounding_box.type = SolidPrimitive.BOX
        bounding_box.dimensions = [0.05, 0.05, 0.05]

        position_constraint.constraint_region.primitives.append(bounding_box)
        position_constraint.constraint_region.primitive_poses.append(target_pose.pose)
        constraints.position_constraints.append(position_constraint)

        # Orientation Constraint
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = target_pose.header
        orientation_constraint.link_name = 'link6'
        orientation_constraint.orientation = target_pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.3
        orientation_constraint.absolute_y_axis_tolerance = 0.3
        orientation_constraint.absolute_z_axis_tolerance = 0.3
        orientation_constraint.weight = 1.0
        constraints.orientation_constraints.append(orientation_constraint)

        return constraints

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'反饋: {feedback_msg.feedback.state}')


def main(args=None):
    rclpy.init(args=args)
    node = MoveItTFPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
