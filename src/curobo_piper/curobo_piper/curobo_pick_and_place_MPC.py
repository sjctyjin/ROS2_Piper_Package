#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
import tf2_ros
import numpy as np
import torch
import time
import threading
# CuRobo 导入
from curobo.types.math import Pose
from curobo.types.robot import JointState as CuroboJointState
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
from curobo.wrap.reacher.mpc import MpcSolver, MpcSolverConfig
from curobo.rollout.rollout_base import Goal
from curobo.util_file import get_world_configs_path, join_path, load_yaml
from curobo.geom.types import WorldConfig
from curobo.types.base import TensorDeviceType

# ROSBridge导入 (如果需要)
try:
    from rosbridge_websocket import init_rosbridge, publish_joint_state, close_rosbridge
except ImportError:
    print("警告: 未找到rosbridge_websocket模块，无法发送关节值到ROS1")

class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_and_place_node')
        self.callback_group = ReentrantCallbackGroup()
        
        # 初始化张量设备类型
        self.tensor_args = TensorDeviceType()
        
        # 声明参数
        self.declare_parameter('robot_config', 'piper.yml')
        self.declare_parameter('enable_rosbridge', False)
        self.declare_parameter('rosbridge_host', '192.168.3.125')
        self.declare_parameter('rosbridge_port', 9090)
        self.declare_parameter('rosbridge_topic', '/joint_custom_state')
        self.declare_parameter('frame_id', 'piper_single')
        #self.declare_parameter('place_position_x', 0.10)
        #self.declare_parameter('place_position_y', -0.245)
        #self.declare_parameter('place_position_z', 0.260)
        self.declare_parameter('place_position_x', 0.071)
        self.declare_parameter('place_position_y', -0.060)
        self.declare_parameter('place_position_z', 0.408)
        self.declare_parameter('place_joint_target', [
            1.580740392,
            1.7780669200000003,
            -0.690817288,
            1.7106981920000002,
            -1.1191895960000002,
            -0.31294536000000006,
        ])
        self.declare_parameter('place_joint_gripper', -0.012)
        self.declare_parameter('gripper_close_value', 0.0)
        self.declare_parameter('gripper_open_value', -0.05)
        self.declare_parameter('gripper_pick_threshold', 0.01)
        self.declare_parameter('gripper_check_delay_sec', 0.6)
        self.declare_parameter('gripper_check_sample_count', 3)
        self.declare_parameter('gripper_relief_effort_threshold', 0.8)
        self.declare_parameter('gripper_relief_command', -0.015)
        self.declare_parameter('tf_max_age_sec', 2.0)
        self.declare_parameter('target_object_frame', 'camera_object_frame')
        self.declare_parameter('tf_lookup_timeout_sec', 0.05)
        self.declare_parameter('post_home_tf_settle_sec', 1.0)
        self.declare_parameter('require_home_before_tf', True)
        self.declare_parameter('home_joint_tolerance', 0.06)
        self.declare_parameter('arm_target_tolerance', 0.04)
        self.declare_parameter('arm_target_timeout_sec', 15.0)
        self.declare_parameter('post_pick_settle_sec', 1.5)
        self.declare_parameter('post_grasp_hold_sec', 1.5)
        self.declare_parameter('use_home_orientation_for_pick', True)
        self.declare_parameter('enable_pick_orientation_fallback', True)
        self.declare_parameter('pick_orientation_fallback_deg', 5.0)
        self.declare_parameter('command_smoothing_enabled', True)
        self.declare_parameter('command_publish_rate_hz', 100.0)
        self.declare_parameter('command_max_velocity', 1.0)
        self.declare_parameter('command_max_acceleration', 4.0)
        self.declare_parameter('command_target_filter_alpha', 0.35)
        
        # 读取参数
        self.robot_config = self.get_parameter('robot_config').get_parameter_value().string_value
        self.enable_rosbridge = self.get_parameter('enable_rosbridge').get_parameter_value().bool_value
        self.rosbridge_host = self.get_parameter('rosbridge_host').get_parameter_value().string_value
        self.rosbridge_port = self.get_parameter('rosbridge_port').get_parameter_value().integer_value
        self.rosbridge_topic = self.get_parameter('rosbridge_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.place_position_x = self.get_parameter('place_position_x').get_parameter_value().double_value
        self.place_position_y = self.get_parameter('place_position_y').get_parameter_value().double_value
        self.place_position_z = self.get_parameter('place_position_z').get_parameter_value().double_value
        self.gripper_close_value = self.get_parameter('gripper_close_value').get_parameter_value().double_value
        self.gripper_open_value = self.get_parameter('gripper_open_value').get_parameter_value().double_value
        self.gripper_pick_threshold = self.get_parameter('gripper_pick_threshold').get_parameter_value().double_value
        self.gripper_check_delay_sec = self.get_parameter('gripper_check_delay_sec').get_parameter_value().double_value
        self.gripper_check_sample_count = self.get_parameter('gripper_check_sample_count').get_parameter_value().integer_value
        self.gripper_relief_effort_threshold = self.get_parameter('gripper_relief_effort_threshold').get_parameter_value().double_value
        self.gripper_relief_command = self.get_parameter('gripper_relief_command').get_parameter_value().double_value
        self.tf_max_age_sec = self.get_parameter('tf_max_age_sec').get_parameter_value().double_value
        self.target_object_frame = self.get_parameter('target_object_frame').get_parameter_value().string_value
        self.tf_lookup_timeout_sec = self.get_parameter('tf_lookup_timeout_sec').get_parameter_value().double_value
        self.post_home_tf_settle_sec = self.get_parameter('post_home_tf_settle_sec').get_parameter_value().double_value
        self.require_home_before_tf = self.get_parameter('require_home_before_tf').get_parameter_value().bool_value
        self.home_joint_tolerance = self.get_parameter('home_joint_tolerance').get_parameter_value().double_value
        self.arm_target_tolerance = self.get_parameter('arm_target_tolerance').get_parameter_value().double_value
        self.arm_target_timeout_sec = self.get_parameter('arm_target_timeout_sec').get_parameter_value().double_value
        self.post_pick_settle_sec = self.get_parameter('post_pick_settle_sec').get_parameter_value().double_value
        self.post_grasp_hold_sec = self.get_parameter('post_grasp_hold_sec').get_parameter_value().double_value
        self.use_home_orientation_for_pick = self.get_parameter('use_home_orientation_for_pick').get_parameter_value().bool_value
        self.enable_pick_orientation_fallback = self.get_parameter('enable_pick_orientation_fallback').get_parameter_value().bool_value
        self.pick_orientation_fallback_deg = self.get_parameter('pick_orientation_fallback_deg').get_parameter_value().double_value
        self.command_smoothing_enabled = self.get_parameter('command_smoothing_enabled').get_parameter_value().bool_value
        self.command_publish_rate_hz = self.get_parameter('command_publish_rate_hz').get_parameter_value().double_value
        self.command_max_velocity = self.get_parameter('command_max_velocity').get_parameter_value().double_value
        self.command_max_acceleration = self.get_parameter('command_max_acceleration').get_parameter_value().double_value
        self.command_target_filter_alpha = self.get_parameter('command_target_filter_alpha').get_parameter_value().double_value
        
        # 初始化ROSBridge
        if self.enable_rosbridge:
            try:
                self.rosbridge_client = init_rosbridge(self.rosbridge_host, self.rosbridge_port)
                self.get_logger().info(f'已初始化ROSBridge客户端，将发送关节值到 {self.rosbridge_topic}')
            except Exception as e:
                self.get_logger().error(f'初始化ROSBridge失败: {e}')
                self.enable_rosbridge = False
        
        # 定义关节名称 (根据机器人类型)
        self.is_piper = 'piper' in self.robot_config.lower()
        if self.is_piper:
            self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "joint8"]
        else:
            self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
        
        # 创建发布者
        self.publisher = self.create_publisher(JointState, '/joint_custom_state', 10)
        
        # 订阅当前关节状态
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.callback_group,
        )
        # 订阅馬達實際關節狀態
        
        self.real_subscription = self.create_subscription(
            JointState,
            '/joint_states_single',
            self.joint_callback,
            10,
            callback_group=self.callback_group,
        )
        
        
        
        # 创建TF监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self,
            spin_thread=True,
        )
        
        # 初始化CuRobo
        self.get_logger().info(f'正在加载机器人配置: {self.robot_config}')
        # 创建一个简单的世界配置
        world_config = {
            "cuboid": {
                "dummy": {
                    "dims": [0.0001, 0.0001, 0.0001],
                    "pose": [10.0, 10.0, 10.0, 1, 0, 0, 0.0],
                },
            },
        }
        
        self.dt = 0.01  # 插值时间步长
        motion_gen_config = MotionGenConfig.load_from_robot_config(
            self.robot_config, 
            world_config, 
            interpolation_dt=self.dt
        )
        self.motion_gen = MotionGen(motion_gen_config)
        self.motion_gen_enable_graph = True
        self.get_logger().info("正在预热运动规划器...")
        try:
            self.motion_gen.warmup(enable_graph=True)
        except Exception as exc:
            self.motion_gen_enable_graph = False
            self.get_logger().warn(f"MotionGen graph预热失败，改用non-graph模式: {exc}")
            self.motion_gen.warmup(enable_graph=False)
        self.get_logger().info("CuRobo运动规划器已就绪")
        # —— 初始化并预热 MPCSolver —— 
        # 这里的参数请根据你实际需要添加 collision_checker_type、collision_cache、use_mppi 等
        mpc_cfg = MpcSolverConfig.load_from_robot_config(
            self.robot_config,
            world_config,
            use_cuda_graph=True,
            use_cuda_graph_metrics=True,
            self_collision_check=True,
            collision_checker_type=None,      # 或者 curobo.geom.sdf.world.CollisionCheckerType.MESH
            collision_cache={"obb":30,"mesh":10},
            use_mppi=True,
            use_lbfgs=False,
            use_es=False,
            store_rollouts=True,
            step_dt=0.02,
            # override_particle_file="path/to/your/particle_mpc.yml"
            # 或者 base_cfg={"cost":{...}} 覆盖权重
        )
        self.mpc = MpcSolver(mpc_cfg)
        self.get_logger().info("正在预热 MPC 规划器...")
        self.get_logger().info("MPC 规划器已就绪")
        
        # 初始化状态和标志
        self.last_position = None
        self.executing = False
        self.step_counter = 0
        
        # Pick and Place 状态机
        self.STATE_IDLE = 0
        self.STATE_MOVE_TO_PICK = 1
        self.STATE_GRASP = 2
        self.STATE_MOVE_TO_PLACE = 3
        self.STATE_RELEASE = 4
        self.STATE_MOVE_TO_HOME = 5
        self.STATE_WATTING_GRIPPER=6
        self.current_state = self.STATE_IDLE
        self.previous_state = 0
        # 保存抓取位置
        self.pick_position = None
        self.pick_orientation = None
        self.pick_check = 0
        self.pre_pick_check = -1
        #保存初次的joint值
        self.latest_joint_state = None
        self.latest_real_joint_state = None
        #判斷是否成功末端值
        self.gripper_data_event = False    
        self.gripper_check_timer = 0 
        self.pending_place_after_home = False
        self.latest_gripper_position = 0.0
        self.latest_gripper_effort = 0.0
        self.gripper_pick_samples = []
        self.gripper_relief_applied = False
        self.tf_accept_after_time = None
        self.direct_home_after_failed_grasp = False
        #存放關節最後值
        self.current_joint_positions_globel = []
        self.command_lock = threading.Lock()
        self.latest_joint_target = None
        self.filtered_joint_target = None
        self.commanded_positions = None
        self.commanded_velocity = None
        self.command_smoother_running = True
        self.command_smoother_period = 1.0 / max(self.command_publish_rate_hz, 1.0)
        self.command_smoother_last_time = time.monotonic()
        self.command_smoother_thread = threading.Thread(
            target=self.command_smoother_loop,
            daemon=True,
        )
        self.command_smoother_thread.start()
        
        
        # 创建定时器，设置为10Hz
        self.timer = self.create_timer(0.1, self.pick_and_place_loop, callback_group=self.callback_group)

        self.timer2 = self.create_timer(0.1, self.gripper_eval_loop, callback_group=self.callback_group)
        
    def joint_state_callback(self, msg):
        """接收当前关节状态"""
        
        self.latest_joint_state = msg
        #self.get_logger().info(f"監聽JointState : {self.latest_joint_state}")
    	
    def get_home_orientation(self):
        return [0.685, 0.0, 0.729, 0.0]

    def get_home_pose(self, orientation=None):
        if orientation is None:
            orientation = self.get_home_orientation()
        return Pose.from_list([
            0.121, 0.0, 0.458,
            orientation[0], orientation[1], orientation[2], orientation[3],
        ])

    def get_place_pose_from_home(self):
        if self.pick_orientation is not None:
            orientation = [
                float(self.pick_orientation[0]),
                float(self.pick_orientation[1]),
                float(self.pick_orientation[2]),
                float(self.pick_orientation[3]),
            ]
        else:
            orientation = [0.685, 0.0, 0.729, 0.0]

        return Pose.from_list([
            self.place_position_x, self.place_position_y, self.place_position_z,
            orientation[0], orientation[1], orientation[2], orientation[3],
        ])

    def get_place_joint_target(self):
        place_joint_target = list(self.get_parameter('place_joint_target').value)
        place_joint_gripper = float(self.get_parameter('place_joint_gripper').value)
        if len(place_joint_target) != 6:
            self.get_logger().warn(
                f"place_joint_target length should be 6, got {len(place_joint_target)}; using default"
            )
            place_joint_target = [
                #1.580740392,
                #1.7780669200000003,
                #-0.690817288,
                #1.7106981920000002,
                #-1.1191895960000002,
                #-0.31294536000000006,
                0.0,
                0.75,
                -1.1,
                0.0,
                0.5,
                0.0,
            ]
        return place_joint_target, place_joint_gripper

    def get_home_joint_target(self):
        return [
            0.0,
                0.75,
                -1.1,
                0.0,
                0.5,
                0.0,
        ], 0.035

    def is_at_home_joint(self, current_joint_positions):
        home_joint_positions, _ = self.get_home_joint_target()
        arm_count = min(len(home_joint_positions), len(current_joint_positions))
        if arm_count == 0:
            return False, float('inf')

        current = np.array(current_joint_positions[:arm_count], dtype=np.float64)
        home = np.array(home_joint_positions[:arm_count], dtype=np.float64)
        max_error = float(np.max(np.abs(current - home)))
        return max_error <= self.home_joint_tolerance, max_error

    def build_joint_command(self, joint_positions, gripper_value):
        if isinstance(joint_positions, np.ndarray):
            joint_positions = joint_positions.tolist()

        positions = list(joint_positions)
        if len(positions) < len(self.joint_names):
            positions.extend([0.0] * (len(self.joint_names) - len(positions)))
        else:
            positions = positions[:len(self.joint_names)]

        if len(positions) > 6:
            positions[6] = gripper_value
        return np.array(positions, dtype=np.float64)

    def get_latest_full_joint_positions(self, fallback):
        feedback_msg = self.latest_real_joint_state or self.latest_joint_state
        if feedback_msg is None:
            return np.array(fallback, dtype=np.float64)

        positions = []
        for idx, name in enumerate(self.joint_names):
            if name in feedback_msg.name:
                msg_idx = feedback_msg.name.index(name)
                if msg_idx < len(feedback_msg.position):
                    positions.append(feedback_msg.position[msg_idx])
                    continue
            positions.append(fallback[idx] if idx < len(fallback) else 0.0)
        return np.array(positions, dtype=np.float64)

    def get_feedback_arm_joint_positions(self):
        if self.latest_real_joint_state is None and self.latest_joint_state is None:
            return None

        current_joint_positions = []
        for name in self.motion_gen.kinematics.joint_names:
            value_found = False
            for msg in (self.latest_real_joint_state, self.latest_joint_state):
                if msg is None or name not in msg.name:
                    continue
                msg_idx = msg.name.index(name)
                if msg_idx < len(msg.position):
                    current_joint_positions.append(float(msg.position[msg_idx]))
                    value_found = True
                    break
            if not value_found:
                self.get_logger().warning(f"找不到關節 {name}，使用預設值0.0")
                current_joint_positions.append(0.0)

        return current_joint_positions

    def wait_until_arm_target_reached(self, target_positions, timeout_sec=None, tolerance=None, label="target"):
        if timeout_sec is None:
            timeout_sec = self.arm_target_timeout_sec
        if tolerance is None:
            tolerance = self.arm_target_tolerance

        target = np.array(list(target_positions)[:len(self.motion_gen.kinematics.joint_names)], dtype=np.float64)
        deadline = time.monotonic() + max(float(timeout_sec), 0.1)
        last_log_time = 0.0

        while rclpy.ok() and time.monotonic() < deadline:
            current = self.get_feedback_arm_joint_positions()
            if current is not None:
                current_arr = np.array(current[:len(target)], dtype=np.float64)
                max_error = float(np.max(np.abs(current_arr - target)))
                if max_error <= tolerance:
                    self.get_logger().info(
                        f"{label} 已到位: max_error={max_error:.4f}, tolerance={tolerance:.4f}"
                    )
                    return True

                now = time.monotonic()
                if now - last_log_time > 0.5:
                    self.get_logger().info(
                        f"等待 {label} 到位: max_error={max_error:.4f}, tolerance={tolerance:.4f}"
                    )
                    last_log_time = now

            time.sleep(0.02)

        self.get_logger().warn(f"等待 {label} 到位逾時，繼續流程")
        return False

    def publish_joint_command_now(self, positions, arm_velocity=None):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [float(v) for v in positions]
        msg.velocity = [0.0] * len(self.joint_names)
        if arm_velocity is not None:
            arm_count = min(len(arm_velocity), len(msg.velocity))
            for i in range(arm_count):
                msg.velocity[i] = float(arm_velocity[i])

        self.publisher.publish(msg)
        if self.enable_rosbridge:
            publish_joint_state(
                self.rosbridge_topic,
                msg.position,
                self.joint_names,
                self.frame_id,
            )

    def update_command_feedback(self, positions):
        arm_positions = []
        for name in self.motion_gen.kinematics.joint_names:
            if name in self.joint_names:
                idx = self.joint_names.index(name)
                arm_positions.append(float(positions[idx]))
            else:
                arm_positions.append(0.0)
        self.current_joint_positions_globel = arm_positions

    def command_smoother_loop(self):
        while self.command_smoother_running and rclpy.ok():
            time.sleep(self.command_smoother_period)
            self.publish_smoothed_joint_command()

    def publish_smoothed_joint_command(self):
        if not self.command_smoothing_enabled:
            return

        with self.command_lock:
            if self.latest_joint_target is None:
                return

            now = time.monotonic()
            dt = max(now - self.command_smoother_last_time, self.command_smoother_period)
            self.command_smoother_last_time = now

            target = np.array(self.latest_joint_target, dtype=np.float64)
            if self.commanded_positions is None:
                self.commanded_positions = self.get_latest_full_joint_positions(target)
                self.commanded_velocity = np.zeros(len(self.motion_gen.kinematics.joint_names), dtype=np.float64)
            if self.filtered_joint_target is None:
                self.filtered_joint_target = target.copy()

            alpha = float(np.clip(self.command_target_filter_alpha, 0.0, 1.0))
            self.filtered_joint_target = alpha * target + (1.0 - alpha) * self.filtered_joint_target

            arm_count = len(self.motion_gen.kinematics.joint_names)
            q = self.commanded_positions[:arm_count]
            q_target = self.filtered_joint_target[:arm_count]
            v_prev = self.commanded_velocity[:arm_count]

            max_velocity = max(float(self.command_max_velocity), 1e-6)
            max_acceleration = max(float(self.command_max_acceleration), 1e-6)
            v_target = np.clip((q_target - q) / dt, -max_velocity, max_velocity)
            dv = np.clip(v_target - v_prev, -max_acceleration * dt, max_acceleration * dt)
            v_new = np.clip(v_prev + dv, -max_velocity, max_velocity)
            q_next = q + v_new * dt

            step = q_next - q
            remaining = q_target - q
            reached = np.abs(step) >= np.abs(remaining)
            q_next = np.where(reached, q_target, q_next)
            v_new = np.where(reached, 0.0, v_new)

            command = self.commanded_positions.copy()
            command[:arm_count] = q_next
            if len(command) > arm_count:
                command[arm_count:] = target[arm_count:]

            self.commanded_positions = command
            self.commanded_velocity = v_new

        self.publish_joint_command_now(command, v_new)
        self.update_command_feedback(command)

    def publish_joint_state_with_gripper(self, joint_positions, gripper_value):
        command_target = self.build_joint_command(joint_positions, gripper_value)

        if self.command_smoothing_enabled:
            with self.command_lock:
                self.latest_joint_target = command_target
                if self.commanded_positions is None:
                    self.commanded_positions = self.get_latest_full_joint_positions(command_target)
                    self.commanded_velocity = np.zeros(
                        len(self.motion_gen.kinematics.joint_names),
                        dtype=np.float64,
                    )
                if self.filtered_joint_target is None:
                    self.filtered_joint_target = command_target.copy()
        else:
            self.publish_joint_command_now(command_target)
            self.update_command_feedback(command_target)

        time.sleep(self.dt)
        return

    def publish_joint_state_with_gripper_legacy(self, joint_positions, gripper_value):
        """发布带有夹爪值的关节状态"""

        # 创建消息
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        
        # 确保关节位置是列表
        if isinstance(joint_positions, np.ndarray):
            joint_positions = joint_positions.tolist()
        joint_positions = list(joint_positions)
        if len(joint_positions) < len(self.joint_names):
            joint_positions.extend([0.0] * (len(self.joint_names) - len(joint_positions)))
        else:
            joint_positions = joint_positions[:len(self.joint_names)]

        self.get_logger().info(f"監聽 JointState : {joint_positions}")
        
        # 为joint7(夹爪)设置值
        positions_with_gripper = list(joint_positions)       
        positions_with_gripper[6] = gripper_value  # joint7是夹爪
        msg.position = positions_with_gripper
        #self.get_logger().info(f"joint_positions直－－－－－ ：{positions_with_gripper}")
        msg.velocity = [10.0] * len(self.joint_names)
        # 发布到ROS2
        self.publisher.publish(msg)
        #self.get_logger().info(f"夾爪直－－－－－ ：{gripper_value}")
        # 如果启用了ROSBridge，也发布到ROS1
        if self.enable_rosbridge:
            publish_joint_state(
                self.rosbridge_topic,
                positions_with_gripper,
                self.joint_names,
                self.frame_id
            )
        #刷新最後位置
        self.latest_joint_state.position = positions_with_gripper
        current_joint_positions = []
            
        for name in self.motion_gen.kinematics.joint_names:
            if name in self.latest_joint_state.name:
                    #self.get_logger().warning(f"关节 {name}，使用默认值0.0")
                idx = self.latest_joint_state.name.index(name)
                current_joint_positions.append(self.latest_joint_state.position[idx])                    
            else:
                self.get_logger().warning(f"找不到关节 {name}，使用默认值0.0")
                current_joint_positions.append(0.0)
        self.current_joint_positions_globel = current_joint_positions
        self.get_logger().info(f"current_joint 值  －－－－－ ：{self.current_joint_positions_globel}")
        # 等待一个时间步
        time.sleep(self.dt)
    
    def execute_trajectory(self, trajectory, gripper_value=None,mpc_mode=None):#待修正mpc加入判斷避免一直輸出關節資訊 12:00
        """执行轨迹，可选指定夹爪值"""
        if len(trajectory) == 0:
            self.get_logger().warning("轨迹为空，无法执行")
            return False
        if mpc_mode == None:    
            self.get_logger().info(f"开始执行 {len(trajectory)} 个轨迹点")
        if mpc_mode == None:    
            # 发布轨迹
            for i in range(len(trajectory)):            
            # 如果指定了夹爪值，使用指定值
                if gripper_value is not None:
                    self.publish_joint_state_with_gripper(trajectory[i], gripper_value)
                else:
                # 默认使用轨迹中的值
                    self.publish_joint_state_with_gripper(trajectory[i], trajectory[i][6] if len(trajectory[i]) > 6 else self.gripper_open_value)
            self.get_logger().info("轨迹执行完成")
        else:
            if gripper_value is not None:
                self.publish_joint_state_with_gripper(trajectory[0], gripper_value)
            else:
                # 默认使用轨迹中的值
                first_point = trajectory[0]
                self.publish_joint_state_with_gripper(
                    first_point,
                    first_point[6] if len(first_point) > 6 else self.gripper_open_value,
                )
        if mpc_mode is None:
            final_target = trajectory[-1]
            if isinstance(final_target, np.ndarray):
                final_target = final_target.tolist()
            return self.wait_until_arm_target_reached(final_target, label="trajectory target")

        return True
        
    
    def plan_and_execute(self, current_joints, target_pose, gripper_value=None):
        """规划并执行轨迹"""
        self.get_logger().info(f"规划到目标位置: {target_pose}")
        
        # 创建CuRobo关节状态
        cu_js = CuroboJointState(
            position=self.tensor_args.to_device(np.array(current_joints)),
            velocity=self.tensor_args.to_device(np.zeros_like(current_joints)),
            acceleration=self.tensor_args.to_device(np.zeros_like(current_joints)),
            jerk=self.tensor_args.to_device(np.zeros_like(current_joints)),
            joint_names=self.motion_gen.kinematics.joint_names
        )
        
        # 执行运动规划
        result = self.motion_gen.plan_single(
            cu_js.unsqueeze(0), 
            target_pose, 
            MotionGenPlanConfig(max_attempts=20, enable_graph=self.motion_gen_enable_graph)
        )
        
        if not result.success:
            self.get_logger().error("轨迹规划失败")
            return False
        
        # 获取轨迹
        trajectory = result.get_interpolated_plan().position.cpu().numpy()
        
        # 执行轨迹
        return self.execute_trajectory(trajectory, gripper_value)
        
        
    def plan_and_execute_mpc(self, current_joints, target_pose, gripper_value=None):
        """用 MPCSolver 规划并执行 Pick 阶段的轨迹"""
        self.get_logger().info(f"[MPC] 规划到目标位置: {target_pose}")
        # 1. 构造当前状态
        cu_js = CuroboJointState(
            position=self.tensor_args.to_device(np.array(current_joints)),
            velocity=self.tensor_args.to_device(np.zeros_like(current_joints)),
            acceleration=self.tensor_args.to_device(np.zeros_like(current_joints)),
            jerk=self.tensor_args.to_device(np.zeros_like(current_joints)),
            joint_names=self.mpc.rollout_fn.joint_names
        )
        # 2. 创建 Goal
        goal = Goal(current_state=cu_js,goal_state=cu_js, goal_pose=target_pose)
        # 3. Setup & update goal
        goal_buf = self.mpc.setup_solve_single(goal,1)
        self.mpc.update_goal(goal_buf)
        # 4. 循环 step, 累积轨迹
        
        max_iters = 100
        check_traj = True
        k = 0
        last_mpc_target = None
        for _ in range(max_iters):
            traj = []
            feedback_joint_positions = self.get_feedback_arm_joint_positions()
            if feedback_joint_positions is not None:
                self.current_joint_positions_globel = feedback_joint_positions
            cu_js = CuroboJointState(
                position=self.tensor_args.to_device(np.array(self.current_joint_positions_globel)),
                velocity=self.tensor_args.to_device(np.zeros_like(self.current_joint_positions_globel)),
                acceleration=self.tensor_args.to_device(np.zeros_like(self.current_joint_positions_globel)),
                jerk=self.tensor_args.to_device(np.zeros_like(self.current_joint_positions_globel)),
                joint_names=self.mpc.rollout_fn.joint_names
            )
            res = self.mpc.step(cu_js, max_attempts=2)           
            if not res.metrics.feasible.item():
                self.get_logger().warning("[MPC] 轨迹不可行，提前退出")
                check_traj = False
                break
            js_next = res.js_action
            last_mpc_target = js_next.position.cpu().numpy().tolist()
            traj.append(last_mpc_target)
            if traj !=[]:
                self.execute_trajectory(traj, gripper_value,"mpc")#指定mpc,避免一直輸出訊息
                k+=1
            # 可加收敛判定：位置误差或方向误差足够小时 break
        # 5. 連續性执行轨迹
        self.get_logger().info(f"轨迹执行完成-執行了{k}次規劃")
        if check_traj and last_mpc_target is not None:
            check_traj = self.wait_until_arm_target_reached(last_mpc_target, label="MPC pick target")
        return check_traj


    
    def operate_gripper(self, current_joints, gripper_value):
        """操作夹爪 - 仅改变夹爪值，保持其他关节不变"""
        self.get_logger().info(f"操作夹爪，设置值: {gripper_value}")
        
        # 创建一个1点轨迹，包含当前关节值
        trajectory = [current_joints]
        
        # 执行，指定夹爪值
        return self.execute_trajectory(trajectory, gripper_value)
    
    def get_current_arm_joint_positions(self):
        return self.get_feedback_arm_joint_positions()

    def apply_gripper_relief(self):
        if self.gripper_relief_applied:
            return True

        current_joint_positions = self.get_current_arm_joint_positions()
        if current_joint_positions is None:
            self.get_logger().warn("Cannot read current joints; skip gripper relief")
            return False

        relief_command = self.gripper_relief_command
        self.get_logger().info(
            f"夾爪減壓: position={self.latest_gripper_position:.5f}, "
            f"effort={self.latest_gripper_effort:.5f}, target={relief_command:.5f}"
        )

        if self.execute_trajectory([current_joint_positions], relief_command):
            self.gripper_relief_applied = True
            time.sleep(0.2)
            return True

        self.get_logger().warn("Gripper relief command failed")
        return False

    @staticmethod
    def normalize_quaternion_wxyz(quaternion):
        q = np.array(quaternion, dtype=np.float64)
        norm = np.linalg.norm(q)
        if norm < 1e-8:
            return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        return q / norm

    @staticmethod
    def quaternion_multiply_wxyz(q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ], dtype=np.float64)

    @staticmethod
    def axis_angle_to_quaternion_wxyz(axis, angle_deg):
        angle_rad = np.deg2rad(angle_deg)
        half_angle = angle_rad / 2.0
        axis = np.array(axis, dtype=np.float64)
        axis = axis / np.linalg.norm(axis)
        s = np.sin(half_angle)
        return np.array([np.cos(half_angle), axis[0] * s, axis[1] * s, axis[2] * s], dtype=np.float64)

    def get_pick_orientation_candidates(self, base_orientation):
        base = self.normalize_quaternion_wxyz(base_orientation)
        candidates = [base]

        if not self.enable_pick_orientation_fallback:
            return candidates

        angle = self.pick_orientation_fallback_deg
        axis_candidates = [
            ([1.0, 0.0, 0.0], angle),
            ([1.0, 0.0, 0.0], -angle),
            ([0.0, 1.0, 0.0], angle),
            ([0.0, 1.0, 0.0], -angle),
        ]

        for axis, delta_deg in axis_candidates:
            delta_q = self.axis_angle_to_quaternion_wxyz(axis, delta_deg)
            candidate = self.normalize_quaternion_wxyz(
                self.quaternion_multiply_wxyz(base, delta_q)
            )
            candidates.append(candidate)

        return candidates

    def plan_and_execute_pick_with_fallback(self, current_joints, pick_position, base_orientation, gripper_value=None):
        orientation_candidates = self.get_pick_orientation_candidates(base_orientation)

        for idx, orientation in enumerate(orientation_candidates):
            target_pose = Pose.from_list([
                pick_position[0], pick_position[1], pick_position[2],
                orientation[0], orientation[1], orientation[2], orientation[3],
            ])

            if idx == 0:
                self.get_logger().info("開始規劃")
            else:
                self.get_logger().info(
                    f"固定姿態失敗，嘗試候選姿態 {idx}/{len(orientation_candidates) - 1}: "
                    f"{orientation.tolist()}"
                )

            if self.plan_and_execute_mpc(current_joints, target_pose, gripper_value):
                self.pick_orientation = np.array(orientation, dtype=np.float64)
                return True

        return False

    def pick_and_place_loop(self):
        """Pick and Place状态机主循环"""
        self.step_counter += 1

        if self.executing:
            self.get_logger().info(f"暫停輸出..... 目前狀態 ： {self.current_state}")
            return
            
        # 如果还没有收到关节状态，等待
        if self.latest_joint_state is None:
            if self.step_counter % 50 == 0:  # 减少日志量，每5秒左右记录一次
                self.get_logger().info("等待接收关节状态...")
            return
            
        self.executing = True
        try:
            # 提取当前关节位置
            current_joint_positions = []
            
            for name in self.motion_gen.kinematics.joint_names:
                if name in self.latest_joint_state.name:
                    #self.get_logger().warning(f"关节 {name}，使用默认值0.0")
                    idx = self.latest_joint_state.name.index(name)
                    current_joint_positions.append(self.latest_joint_state.position[idx])                    
                else:
                    self.get_logger().warning(f"找不到关节 {name}，使用默认值0.0")
                    current_joint_positions.append(0.0)
            feedback_joint_positions = self.get_feedback_arm_joint_positions()
            if feedback_joint_positions is not None:
                current_joint_positions = feedback_joint_positions
            self.current_joint_positions_globel = current_joint_positions
            #self.get_logger().info(f"當前Joint直 --------： {self.latest_joint_state.position}")
            self.get_logger().info(f"當前Joint直 --------： {self.current_joint_positions_globel}\n self.joint_names值 :{self.joint_names}")
            # 状态机处理
            if self.current_state == self.STATE_IDLE:
                if self.require_home_before_tf:
                    at_home, home_error = self.is_at_home_joint(current_joint_positions)
                    if not at_home:
                        if self.step_counter % 10 == 0:
                            self.get_logger().info(
                                f"尚未回到Home joint，暫停TF檢查: "
                                f"max_error={home_error:.4f}, tolerance={self.home_joint_tolerance:.4f}"
                            )
                        return

                # 查找目标TF变换
                try:
                    try:
                        tf = self.tf_buffer.lookup_transform(
                            'base_link',
                            self.target_object_frame,
                            rclpy.time.Time(),
                            timeout=Duration(seconds=self.tf_lookup_timeout_sec),
                        )
                    except (
                        tf2_ros.LookupException,
                        tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException,
                    ) as exc:
                        if self.step_counter % 10 == 0:
                            self.get_logger().info(f"無法找到目標，繼續等待: {exc}")
                        return
                    # ✅ 插入這段檢查 TF 是否新鮮
                    now = self.get_clock().now()
                    tf_time = tf.header.stamp
                    tf_msg_time = rclpy.time.Time.from_msg(tf_time)
                    tf_age = now - tf_msg_time

                    if tf_age > Duration(seconds=self.tf_max_age_sec):
                        self.get_logger().warn(f"TF已過期 ({tf_age.nanoseconds/1e9:.2f}s)，忽略此次抓取")
                        return

                    if (
                        self.tf_accept_after_time is not None
                        and tf_msg_time <= self.tf_accept_after_time
                    ):
                        self.get_logger().info(
                            f"等待Home後的新TF... tf={tf_msg_time.nanoseconds/1e9:.2f}, "
                            f"gate={self.tf_accept_after_time.nanoseconds/1e9:.2f}"
                        )
                        return
                        
                    # 提取位置和方向
                    position = tf.transform.translation
                    orientation = tf.transform.rotation
                    
                    # 保存抓取位置和方向
                    self.pick_position = np.array([position.x, position.y, position.z])
                    tf_orientation = np.array(
                        [orientation.w, orientation.x, orientation.y, orientation.z],
                        dtype=np.float64,
                    )
                    if self.use_home_orientation_for_pick:
                        self.pick_orientation = np.array(self.get_home_orientation(), dtype=np.float64)
                    else:
                        self.pick_orientation = tf_orientation
                    
                    # 如果初次运行或目标位置变化显著，转入MOVE_TO_PICK状态
                    self.get_logger().info(f'开始新的Pick and Place，目标位置: {self.pick_position}')
                    self.last_position = self.pick_position.copy()
                    self.current_state = self.STATE_MOVE_TO_PICK
                    self.get_logger().info(f'狀態 ： {self.current_state}')
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
                    # 无法找到目标，继续等待
                    self.get_logger().info(f"无法找到目标，继续等待: {exc}")
                    
            elif self.current_state == self.STATE_MOVE_TO_PICK:
                if self.plan_and_execute_pick_with_fallback(
                    current_joint_positions,
                    self.pick_position,
                    self.pick_orientation,
                    self.gripper_open_value,
                ):

                    
                    self.current_state = self.STATE_GRASP
                    self.get_logger().info("規劃到夾取位置成功--- 等待2秒")
                    time.sleep(self.post_pick_settle_sec)
                else:
                    # 规划失败，回到IDLE状态
                    self.current_state = self.STATE_IDLE
                    self.get_logger().error(f"規劃失敗")
                
            elif self.current_state == self.STATE_GRASP:
                # 关闭夹爪
                if self.operate_gripper(current_joint_positions, self.gripper_close_value):                
                    self.get_logger().info("夹取成功，等待一段时间...")   
                    time.sleep(self.post_grasp_hold_sec)  # 等待夹爪确实夹紧
                    self.gripper_data_event = True
                    self.gripper_check_timer = 0
                    self.gripper_pick_samples = []
                    self.gripper_relief_applied = False
                    self.current_state = self.STATE_WATTING_GRIPPER                           
                
                else:
                    # 关闭夹爪失败，回到IDLE状态
                    self.current_state = self.STATE_IDLE
            elif self.current_state == self.STATE_WATTING_GRIPPER:
                self.get_logger().info("等待夾爪狀態回覆")   
            
            elif self.current_state == self.STATE_MOVE_TO_PLACE:
                place_joint_positions, place_gripper_value = self.get_place_joint_target()
                self.get_logger().info(
                    f"放置-關節目標{place_joint_positions}, gripper={place_gripper_value}"
                )
                if self.execute_trajectory([place_joint_positions], place_gripper_value):
                    self.current_state = self.STATE_RELEASE
                    self.get_logger().info("已移動到放置關節位置，等待3秒")
                    time.sleep(1.8)
                else:
                    self.pending_place_after_home = False
                    self.current_state = self.STATE_IDLE
                
            elif self.current_state == self.STATE_RELEASE:
                if self.operate_gripper(current_joint_positions, self.gripper_open_value):
                    self.get_logger().info("已打開夾爪，等待1.5秒後直接回Home joint")
                    time.sleep(1.5)
                    home_joint_positions, home_gripper_value = self.get_home_joint_target()
                    if self.execute_trajectory([home_joint_positions], home_gripper_value):
                        self.pending_place_after_home = False
                        self.get_logger().info("等待1.5秒，確認手臂回到Home joint後再重新接受TF")
                        time.sleep(1.5)
                        self.tf_accept_after_time = self.get_clock().now() + Duration(seconds=self.post_home_tf_settle_sec)
                        self.current_state = self.STATE_IDLE
                        self.previous_state = self.STATE_IDLE
                    else:
                        self.current_state = self.STATE_IDLE
                else:
                    self.current_state = self.STATE_IDLE
                
            elif self.current_state == self.STATE_MOVE_TO_HOME:
            
                self.get_logger().info("回到home點!")
                
                time.sleep(1.0)
                
                # 创建回到初始姿态的目标
                
                # 这里使用一个略高于抓取位置的点作为HOME位置
                
                home_pose = self.get_home_pose()
                if self.direct_home_after_failed_grasp:
                    home_joint_positions, home_gripper_value = self.get_home_joint_target()
                    if self.execute_trajectory([home_joint_positions], home_gripper_value):
                        self.direct_home_after_failed_grasp = False
                        self.pending_place_after_home = False
                        self.get_logger().info("夾取失敗後等待1.5秒，確認手臂回到Home joint")
                        time.sleep(1.5)
                        self.tf_accept_after_time = self.get_clock().now() + Duration(seconds=self.post_home_tf_settle_sec)
                        self.current_state = self.STATE_IDLE
                        self.previous_state = self.STATE_IDLE
                    else:
                        self.current_state = self.STATE_IDLE
                elif self.pending_place_after_home:
                    if self.plan_and_execute(current_joint_positions, home_pose, self.gripper_close_value):
                        self.pending_place_after_home = False
                        self.get_logger().info("已回到Home點，等待1.5秒後前往放置關節位置")
                        time.sleep(1.5)
                        self.current_state = self.STATE_MOVE_TO_PLACE
                        self.previous_state = self.STATE_MOVE_TO_PLACE
                    else:
                        # 规划失败，直接回到IDLE状态
                        self.current_state = self.STATE_IDLE            
                else:
                    # 规划并移动到HOME位置，夹爪保持打开
                    if self.plan_and_execute(current_joint_positions, home_pose, self.gripper_open_value):
                        self.pending_place_after_home = False
                        self.get_logger().info("Pick and Place完成--等待2秒!")
                        time.sleep(0.5)
                        self.tf_accept_after_time = self.get_clock().now() + Duration(seconds=self.post_home_tf_settle_sec)
                        self.current_state = self.STATE_IDLE                    
                    else:
                        # 规划失败，直接回到IDLE状态
                        self.current_state = self.STATE_IDLE
            
        except Exception as e:
            self.get_logger().error(f"Pick and Place出错: {e}")
            self.current_state = self.STATE_IDLE
            self.executing = False
        finally:
            self.executing = False
                
                
    def joint_callback(self, msg: JointState):
        self.latest_real_joint_state = msg
        # 取得 gripper 的位置

        try:
            gripper_index = msg.name.index('gripper')  # 找到 gripper 在 name 中的索引
            gripper_position = msg.position[gripper_index]  # 取得對應位置
            self.latest_gripper_position = gripper_position
            if len(msg.effort) > gripper_index:
                self.latest_gripper_effort = msg.effort[gripper_index]
            #self.get_logger().info(f'Gripper position: {gripper_position:.4f}')

            # 根據 gripper 開口程度判斷是否抓取成功（依據你的實際值調整閾值）
            if abs(gripper_position) > self.gripper_pick_threshold:
                #self.get_logger().info("✅ 夾取成功（Gripper 關閉）")
                self.pick_check = 1
            else:
                #self.get_logger().info("❌ 可能未成功夾取（Gripper 打開）")
                self.pick_check = 0

        except ValueError:
            self.get_logger().warn("找不到 'gripper' 關節名稱")
    def gripper_eval_loop(self):
        self.get_logger().info(
            f"Timer執行中...\n當前pre_pick_check:{self.pre_pick_check}\n當前pick_check:{self.pick_check}\n"
            f"當前current_state:{self.current_state}\n當前previous_state:{self.previous_state}"
        )

        if not self.gripper_data_event:
            return

        self.gripper_check_timer += 1
        if self.current_state != self.STATE_WATTING_GRIPPER:
            return

        delay_ticks = max(1, int(round(self.gripper_check_delay_sec / 0.1)))
        sample_count = max(1, int(self.gripper_check_sample_count))

        if self.gripper_check_timer < delay_ticks:
            self.get_logger().info(
                f"等待夾爪穩定中... {self.gripper_check_timer}/{delay_ticks}, "
                f"gripper={self.latest_gripper_position:.4f}"
            )
            return

        current_sample = 1 if abs(self.latest_gripper_position) > self.gripper_pick_threshold else 0
        self.gripper_pick_samples.append(current_sample)
        self.get_logger().info(
            f"夾爪檢測取樣 {len(self.gripper_pick_samples)}/{sample_count}, "
            f"gripper={self.latest_gripper_position:.4f}, sample={current_sample}"
        )

        if len(self.gripper_pick_samples) < sample_count:
            return

        self.pick_check = 1 if sum(self.gripper_pick_samples) >= (sample_count // 2 + 1) else 0

        if self.pick_check == 1:
            if (
                abs(self.latest_gripper_effort) >= self.gripper_relief_effort_threshold
                and not self.gripper_relief_applied
            ):
                self.apply_gripper_relief()
            self.get_logger().info(f"✅ 夾取成功，Pick_check = {self.pick_check}")
            self.direct_home_after_failed_grasp = False
            self.pending_place_after_home = True
            self.current_state = self.STATE_MOVE_TO_HOME
            self.previous_state = self.STATE_MOVE_TO_HOME
            self.pre_pick_check = self.pick_check
        else:
            self.get_logger().warn(f"❌ 夾取失敗，Pick_check = {self.pick_check}")
            self.direct_home_after_failed_grasp = True
            self.pending_place_after_home = False
            self.current_state = self.STATE_MOVE_TO_HOME
            self.pre_pick_check = self.pick_check

        self.gripper_data_event = False
        self.gripper_check_timer = 0
        self.gripper_pick_samples = []

    def T2(self):
        self.get_logger().info(f"Timer執行中...\n當前pre_pick_check:{self.pre_pick_check}\n當前pick_check:{self.pick_check}\n當前current_state:{self.current_state}\n當前previous_state:{self.previous_state}")
        # 轉發給 /web_tf topic
        if self.gripper_data_event:
            self.gripper_check_timer += 1#避免無限迴圈
            if self.current_state == self.STATE_WATTING_GRIPPER:#判斷當前主線程是否在等待
                    if self.pick_check == 1:
                        self.get_logger().info("✅ 夾取成功，Pick_check = 1")
                        self.current_state = self.STATE_MOVE_TO_PLACE
                        self.previous_state = self.STATE_MOVE_TO_HOME
                        self.pre_pick_check = self.pick_check#最後才shift                       
                    else:
                        self.get_logger().warn("❌ 夾取超時，Pick_check = 0")
                        self.current_state = self.STATE_MOVE_TO_HOME
                        self.pre_pick_check = self.pick_check#最後才shift 
                                                        
                    self.gripper_data_event = False                
                    self.gripper_check_timer = 0
                
            if self.gripper_check_timer >= 50:
                self.gripper_check_timer = 0
                self.get_logger().warn("❌ 判斷夾取超時，Pick_check = 0")
                
             
                
        
 

def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("收到键盘中断，正在关闭...")
    except Exception as e:
        node.get_logger().error(f"运行出错: {e}")
    finally:
        # 清理资源
        node.command_smoother_running = False
        if hasattr(node, 'command_smoother_thread'):
            node.command_smoother_thread.join(timeout=0.2)
        if node.enable_rosbridge:
            close_rosbridge()
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
