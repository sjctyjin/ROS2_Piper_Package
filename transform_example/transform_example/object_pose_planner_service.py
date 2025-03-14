import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
import tf2_ros
import numpy as np


class MoveItTFPlanner(Node):
    def __init__(self):
        super().__init__('moveit_tf_planner')

        # MoveGroup Action Client
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')

        # 設定規劃組的名稱（如 'arm'）
        self.planning_group = 'arm'

        # TF2 Buffer 和 Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 創建觸發服務
        self.trigger_service = self.create_service(
            Trigger,
            'trigger_plan',
            self.trigger_callback
        )

        self.get_logger().info('MoveIt TF Planner 已初始化，等待觸發服務呼叫...')

    def trigger_callback(self, request, response):
        """當收到觸發請求時執行規劃"""
        self.get_logger().info('收到觸發請求，開始規劃...')

        try:
            # 處理目標姿態並執行運動規劃
            result = self.process_target_pose()

            if result:
                response.success = True
                response.message = "運動規劃已成功執行"
            else:
                response.success = False
                response.message = "運動規劃失敗"

        except Exception as e:
            self.get_logger().error(f"規劃過程中出現錯誤: {str(e)}")
            response.success = False
            response.message = f"錯誤: {str(e)}"

        return response

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
        goal_msg.request.group_name = self.planning_group
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

        self.get_logger().info('發送運動規劃請求到 MoveIt...')

        self.future = self.move_group_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self.future.add_done_callback(self.goal_response_callback)
        return True

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

    def goal_response_callback(self, future):
        """處理目標響應"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('目標被動作伺服器拒絕')
            return

        self.get_logger().info('目標已接受，等待結果...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """處理結果"""
        result = future.result().result
        if result.error_code.val == result.error_code.SUCCESS:
            self.get_logger().info('運動規劃成功執行!')
        else:
            self.get_logger().error(f'運動規劃失敗，錯誤碼: {result.error_code.val}')


def main(args=None):
    rclpy.init(args=args)
    node = MoveItTFPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()