#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
import tf2_ros
import numpy as np
import torch
import time
from curobo.types.math import Pose
from curobo.types.robot import JointState as CuroboJointState
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig

class CuroboPlanner(Node):
    def __init__(self):
        super().__init__('curobo_tf_planner')

        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "joint8"]
        self.latest_joint_state = None
        self.executing = False  # ✔️ protection flag

        self.publisher = self.create_publisher(JointState, '/joint_custom_state', 10)
        self.dt = 0.01

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.trigger_srv = self.create_service(Trigger, 'trigger_plan', self.trigger_callback)

        world_config = {
            "cuboid": {
                "dummy": {
                    "dims": [0.0001, 0.0001, 0.0001],
                    "pose": [10.0, 10.0, 10.0, 1, 0, 0, 0.0],
                },
            },
        }
        motion_gen_config = MotionGenConfig.load_from_robot_config("piper.yml", world_config, interpolation_dt=self.dt)
        self.motion_gen = MotionGen(motion_gen_config)
        self.motion_gen.warmup()

        self.get_logger().info("Curobo TF Planner ready. Waiting for trigger...")

    def joint_state_callback(self, msg):
        self.latest_joint_state = msg

    def trigger_callback(self, request, response):
        if self.executing:
            response.success = False
            response.message = "已有任務正在執行，請稍後再試"
            return response

        if self.latest_joint_state is None:
            response.success = False
            response.message = "尚未收到 joint state"
            return response

        self.executing = True

        try:
            tf = self.tf_buffer.lookup_transform(
                'base_link', 'object_in_base', rclpy.time.Time(), timeout=Duration(seconds=1.0))

            position = tf.transform.translation
            orientation = tf.transform.rotation
            target_pose = Pose.from_list([
                position.x, position.y, position.z,
                orientation.w, orientation.x, orientation.y, orientation.z
            ])

            joint_position = []
            for name in ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]:
                idx = self.latest_joint_state.name.index(name)
                joint_position.append(self.latest_joint_state.position[idx])

            start_state = CuroboJointState.from_position(
                torch.tensor([[0.0, 0.2, -0.2, 0.0, 0.1, 0.0]], device="cuda:0"),
                joint_names=self.joint_names[:6]
            )

            result = self.motion_gen.plan_single(start_state, target_pose, MotionGenPlanConfig(max_attempts=10))

            if not result.success:
                response.success = False
                response.message = "軌跡規劃失敗"
                return response

            trajectory = result.get_interpolated_plan().position.cpu().numpy()
            print(np.full((trajectory.shape[0], 2),0.8))
            padded_trajectory = np.hstack((trajectory, np.full((trajectory.shape[0], 2),0.8)))

            for i in range(len(padded_trajectory)):
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = self.joint_names
                msg.position = padded_trajectory[i].tolist()
                self.publisher.publish(msg)
                time.sleep(self.dt)  # ⚠️ avoid spin_once issues, simpler and safe

            response.success = True
            response.message = "已完成移動"
            self.get_logger().info("完成移動")

        except Exception as e:
            self.get_logger().error(f"規劃過程出錯: {str(e)}")
            response.success = False
            response.message = f"錯誤: {str(e)}"

        finally:
            self.executing = False

        return response

def main(args=None):
    rclpy.init(args=args)
    node = CuroboPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

