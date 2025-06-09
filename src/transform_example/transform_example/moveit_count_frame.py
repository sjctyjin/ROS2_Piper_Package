#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
import time
from moveit.planning import MoveItPy
from geometry_msgs.msg import Pose, Point, Quaternion
import tf_transformations

class PickObjectNode(Node):
    def __init__(self):
        super().__init__('pick_object_node')
        
        # 初始化MoveItPy
        self.moveit = MoveItPy(node_name="moveit_py")
        self.arm_group = self.moveit.get_planning_component("arm")
        
        # 初始化夾爪控制器客戶端
        self.gripper_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            'gripper_controller/follow_joint_trajectory'
        )
        
        # 等待控制器可用
        self.get_logger().info('Waiting for gripper controller...')
        self.gripper_client.wait_for_server()
        self.get_logger().info('Connected to gripper controller')
    
    def move_to_pose(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        """移動機械臂到指定位置和姿態"""
        self.get_logger().info(f'Planning movement to position: {x}, {y}, {z}')
        
        # 設置目標位置
        q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        pose_target = Pose()
        pose_target.position = Point(x=x, y=y, z=z)
        pose_target.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
        # 設置目標姿態
        self.arm_group.set_goal_state(position=pose_target)
        
        # 規劃
        plan_result = self.arm_group.plan()
        if plan_result:
            self.get_logger().info('Planning succeeded, executing...')
            
            # 執行
            execute_result = self.arm_group.execute()
            if execute_result:
                self.get_logger().info('Movement executed successfully')
                return True
            else:
                self.get_logger().error('Failed to execute movement')
                return False
        else:
            self.get_logger().error('Planning failed')
            return False
    
    def control_gripper(self, position):
        """控制夾爪開合"""
        action = "Opening" if position > 0.01 else "Closing"
        self.get_logger().info(f'{action} gripper...')
        
        goal_msg = FollowJointTrajectory.Goal()
        
        # 設置軌跡
        trajectory = JointTrajectory()
        trajectory.joint_names = ['joint7']  # 夾爪關節
        
        # 設置位置
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start.sec = 1
        trajectory.points.append(point)
        
        # 設置目標
        goal_msg.trajectory = trajectory
        
        # 發送目標
        send_goal_future = self.gripper_client.send_goal_async(goal_msg)
        
        # 等待結果
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Gripper action was rejected')
            return False
        
        # 等待執行完成
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        
        self.get_logger().info(f'Gripper {action.lower()} completed')
        return True
    
    def pick_object(self, x, y, z):
        """執行完整抓取序列"""
        # 開啟夾爪
        if not self.control_gripper(0.04):  # 開啟位置
            return False
        
        # 移動到目標位置
        if not self.move_to_pose(x, y, z, 0.0, 1.57, 0.0):  # 姿態根據需要調整
            return False
        
        # 關閉夾爪進行抓取
        if not self.control_gripper(0.0):  # 關閉位置
            return False
        
        self.get_logger().info('Pick sequence completed successfully')
        return True

def main(args=None):
    rclpy.init(args=args)
    node = PickObjectNode()
    
    try:
        # 執行抓取
        success = node.pick_object(0.4, 0.0, 0.3)  # 替換為您的目標座標
        if success:
            node.get_logger().info('Object picked successfully')
        else:
            node.get_logger().error('Failed to pick object')
    except Exception as e:
        node.get_logger().error(f'Error occurred: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
