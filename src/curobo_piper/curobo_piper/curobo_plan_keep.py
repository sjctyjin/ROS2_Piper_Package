#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import tf2_ros
from rclpy.duration import Duration
import torch
import numpy as np
import time

from curobo.types.math import Pose
from curobo.types.robot import JointState as CuroboJointState
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig


class FollowYZNode(Node):
    def __init__(self):
        super().__init__('follow_yz_node')

        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7', 'joint8']
        self.latest_joint_state = None
        self.fixed_x = 0.1 # 固定 X 座標

        # cuRobo 設定
        self.dt = 0.05
        self.motion_gen = self.init_curobo()

        # TF 訂閱
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # joint state 訂閱
        self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)

        # joint_custom_state 發布
        self.publisher = self.create_publisher(JointState, '/joint_custom_state', 10)

        # 定時執行
        self.timer = self.create_timer(self.dt, self.track_object)

    def init_curobo(self):
        world_config = {
            "cuboid": {
                "dummy": {
                    "dims": [0.0001, 0.0001, 0.0001],
                    "pose": [10.0, 10.0, 10.0, 1, 0, 0, 0.0],
                },
            },
        }
        motion_gen_config = MotionGenConfig.load_from_robot_config("piper.yml", world_config, interpolation_dt=self.dt)
        motion_gen = MotionGen(motion_gen_config)
        motion_gen.warmup()
        return motion_gen

    def joint_callback(self, msg):
        self.latest_joint_state = msg

    def track_object(self):
        if self.latest_joint_state is None:
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                'base_link', 'object_in_base', rclpy.time.Time(), timeout=Duration(seconds=0.5)
            )

            # 提取 TF 資訊（只取 Y/Z）
            y = tf.transform.translation.y
            z = tf.transform.translation.z
            quat = tf.transform.rotation

            # 構建目標 pose，X 固定
            target_pose = Pose.from_list([self.fixed_x, y, z, 0.767, 0.0, 0.676, 0.0])

            # 取得起始 joint state
            joint_position = [self.latest_joint_state.position[self.latest_joint_state.name.index(j)]
                              for j in self.joint_names[:6]]

            start_state = CuroboJointState.from_position(
                torch.tensor([joint_position], device="cuda:0"),
                joint_names=self.joint_names[:6]
            )

            result = self.motion_gen.plan_single(start_state, target_pose, MotionGenPlanConfig(max_attempts=5))
            if not result.success:
                self.get_logger().warn("追蹤規劃失敗")
                return

            traj = result.get_interpolated_plan().position.cpu().numpy()
            padded_traj = np.hstack((traj, np.zeros((traj.shape[0], 2))))  # 補上 joint7/8

            # 發送最後一個位置點
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.position = padded_traj[-1].tolist()
            self.publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f"追蹤過程出錯: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = FollowYZNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

