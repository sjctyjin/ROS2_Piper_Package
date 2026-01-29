import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient
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

        # 啟動定時器來處理目標
        self.timer = self.create_timer(1.0, self.process_target_pose)

    def process_target_pose(self):
        """監聽 TF 並組合目標 Pose"""
        try:
            # 監聽 object_in_base 的座標
            object_transform = self.tf_buffer.lookup_transform(
                'base_link', 'object_in_base', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)
            )

            # 監聽 link6 的姿態
            link6_transform = self.tf_buffer.lookup_transform(
                'base_link', 'link6', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)
            )

            # 組合目標 Pose
            target_pose = PoseStamped()
            target_pose.header.frame_id = 'base_link'
            target_pose.pose.position.x = 0.3
            target_pose.pose.position.y = object_transform.transform.translation.y
            target_pose.pose.position.z = object_transform.transform.translation.z

            # 使用 link6 的姿態
            target_pose.pose.orientation = link6_transform.transform.rotation

            self.get_logger().info(
                f"目標位置: [{target_pose.pose.position.x:.3f}, {target_pose.pose.position.y:.3f}, {target_pose.pose.position.z:.3f}]")

            # 發送目標到 MoveIt
            self.send_target_goal(target_pose)

        except Exception as e:
            self.get_logger().error(f"Failed to process target pose: {str(e)}")

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

        # 設定目標約束
        goal_msg.request.goal_constraints.append(self.create_goal_constraints(target_pose))

        # 發送目標到 MoveIt
        self.move_group_client.wait_for_server()
        self.future = self.move_group_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.future.add_done_callback(self.goal_response_callback)

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
        bounding_box.dimensions = [0.01, 0.01, 0.01]

        position_constraint.constraint_region.primitives.append(bounding_box)
        position_constraint.constraint_region.primitive_poses.append(target_pose.pose)
        constraints.position_constraints.append(position_constraint)

        # Orientation Constraint
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = target_pose.header
        orientation_constraint.link_name = 'link6'
        orientation_constraint.orientation = target_pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        orientation_constraint.weight = 1.0
        constraints.orientation_constraints.append(orientation_constraint)

        return constraints

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Feedback: {feedback_msg.feedback.state}')

    def goal_response_callback(self, future):
        """處理目標響應"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by the action server')
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """處理結果"""
        result = future.result().result
        if result.error_code.val == result.error_code.SUCCESS:
            self.get_logger().info('Motion plan executed successfully!')
        else:
            self.get_logger().error(f'Motion plan failed with error code: {result.error_code.val}')


def main(args=None):
    rclpy.init(args=args)
    node = MoveItTFPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

