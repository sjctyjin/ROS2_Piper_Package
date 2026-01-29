#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from some_ik_solver import solve_ik  # 你自己的 IK 函数

class DirectCtlDemo(Node):
    def __init__(self):
        super().__init__('direct_ctl_demo')
        # 1) 发布器：arm 和 gripper
        self.arm_pub = self.create_publisher(JointTrajectory,
                                             '/arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(JointTrajectory,
                                                 '/gripper_controller/joint_trajectory', 10)

    def move_arm_to(self, target_pose: PoseStamped):
        # 2) 用你的 IK solver 把 target_pose 转成关节列表 q_goal
        q_goal = solve_ik(target_pose)

        traj = JointTrajectory()
        traj.joint_names = ['joint1','joint2','joint3','joint4','joint5','joint6']
        point = JointTrajectoryPoint()
        point.positions = q_goal
        point.time_from_start.sec = 3   # 3 秒到位
        traj.points = [point]

        self.arm_pub.publish(traj)
        self.get_logger().info('Arm trajectory sent, waiting …')
        # 简单等待（可改为订阅 /joint_states 确认）
        self.get_clock().sleep_for(Duration(seconds=3.5))

    def close_gripper(self):
        traj = JointTrajectory()
        traj.joint_names = ['joint7']
        point = JointTrajectoryPoint()
        point.positions = [0.0]           # 全闭
        point.time_from_start.sec = 1     # 1 秒到位
        traj.points = [point]

        self.gripper_pub.publish(traj)
        self.get_logger().info('Gripper command sent')

    def run(self):
        # 构造目标 PoseStamped
        target = PoseStamped()
        target.header.frame_id = 'base_link'
        target.pose.position.x = 0.5
        target.pose.position.y = 0.0
        target.pose.position.z = 0.3
        target.pose.orientation.w = 1.0

        self.move_arm_to(target)
        self.close_gripper()

def main(args=None):
    rclpy.init(args=args)
    node = DirectCtlDemo()
    node.run()
    rclpy.shutdown()

if __name__=='__main__':
    main()

