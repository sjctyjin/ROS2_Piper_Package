import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import time

import torch
import numpy as np
from curobo.types.math import Pose
from curobo.types.robot import JointState as CuroboJointState
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig

class TrajectoryService(Node):
    def __init__(self):
        super().__init__('curobo_trajectory_service')

        self.joint_names = [
            "joint1", "joint2", "joint3", "joint4",
            "joint5", "joint6", "joint7", "joint8"
        ]
        
        self.publisher = self.create_publisher(JointState, '/joint_custom_state', 10)

        self.declare_parameter('place_position_x', 0.12)
        self.declare_parameter('place_position_y', 0.04)
        self.declare_parameter('place_position_z', 0.353)
        
        self.place_position_x = self.get_parameter('place_position_x').get_parameter_value().double_value
        self.place_position_y = self.get_parameter('place_position_y').get_parameter_value().double_value
        self.place_position_z = self.get_parameter('place_position_z').get_parameter_value().double_value

        self.dt = 0.01  # 發送間隔
        self.traj_index = 0
        self.trajectory = None

        self.motion_gen = self.initialize_motion_gen()

        # 建立 ROS 服務
        self.srv = self.create_service(Trigger, 'trigger_plan', self.execute_trajectory_cb)
        self.get_logger().info('Trajectory service ready.')
        self.get_logger().info(f'X:{self.place_position_x},Y:{self.place_position_y},Z:{self.place_position_z}')

    def initialize_motion_gen(self):
        world_config = {
            "cuboid": {
                "dummy": {
                    "dims": [0.0001, 0.0001, 0.0001],
                    "pose": [10.0, 10.0, 10.0, 1, 0, 0, 0.0],
                },
            },
        }
        config = MotionGenConfig.load_from_robot_config(
            "piper.yml", world_config, interpolation_dt=self.dt
        )
        motion_gen = MotionGen(config)
        motion_gen.warmup()
        return motion_gen

    def compute_trajectory(self):
        goal_pose = Pose.from_list([self.place_position_x, self.place_position_y, self.place_position_z, 0.737, 0.000, 0.676, 0.000])
        start_state = CuroboJointState.from_position(
            torch.tensor([[0.0, 0.2, -0.2, 0.0, 0.1, 0.0]], device="cuda:0"),
            joint_names=self.joint_names[:6],
        )
        result = self.motion_gen.plan_single(start_state, goal_pose, MotionGenPlanConfig(max_attempts=5))

        if not result.success:
            self.get_logger().error("Motion planning failed.")
            return None

        traj_pos = result.get_interpolated_plan().position.cpu().numpy()
        padded_traj = np.hstack((traj_pos, np.zeros((traj_pos.shape[0], 2))))
        return padded_traj

    def execute_trajectory_cb(self, request, response):
        self.get_logger().info("Received service request to start trajectory execution.")
        
        self.trajectory = self.compute_trajectory()

        if self.trajectory is None:
            response.success = False
            response.message = "Trajectory computation failed."
            return response

        for i in range(len(self.trajectory)):
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.position = self.trajectory[i].tolist()
            self.publisher.publish(msg)
            time.sleep(self.dt)

        self.get_logger().info("Trajectory execution complete.")
        response.success = True
        response.message = "Trajectory executed."
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

