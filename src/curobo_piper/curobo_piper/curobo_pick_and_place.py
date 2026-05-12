#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
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
        self.declare_parameter('post_home_tf_settle_sec', 1.0)
        self.declare_parameter('enable_pick_orientation_fallback', True)
        self.declare_parameter('pick_orientation_fallback_deg', 5.0)
        
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
        self.post_home_tf_settle_sec = self.get_parameter('post_home_tf_settle_sec').get_parameter_value().double_value
        self.enable_pick_orientation_fallback = self.get_parameter('enable_pick_orientation_fallback').get_parameter_value().bool_value
        self.pick_orientation_fallback_deg = self.get_parameter('pick_orientation_fallback_deg').get_parameter_value().double_value
        
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
            10
        )
        # 订阅馬達實際關節狀態
        
        self.real_subscription = self.create_subscription(
            JointState,
            '/joint_states_single',
            self.joint_callback,
            10
        )
        
        
        
        # 创建TF监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
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
        #判斷是否成功末端值
        self.gripper_data_event = False    
        self.gripper_check_timer = 0 
        self.pending_place_after_home = False
        self.latest_gripper_position = 0.0
        self.latest_gripper_effort = 0.0
        self.gripper_pick_samples = []
        self.gripper_relief_applied = False
        self.tf_accept_after_time = self.get_clock().now()
        self.direct_home_after_failed_grasp = False
        
        
        # 创建定时器，设置为10Hz
        self.timer = self.create_timer(0.1, self.pick_and_place_loop)

        self.timer2 = self.create_timer(0.1, self.gripper_eval_loop)
        
    def joint_state_callback(self, msg):
        """接收当前关节状态"""
        
        self.latest_joint_state = msg
        #self.get_logger().info(f"監聽JointState : {self.latest_joint_state}")
    	
    def get_home_pose(self, orientation=None):
        if orientation is None:
            orientation = [0.685, 0.0, 0.729, 0.0]
        return Pose.from_list([
            0.121, 0.0, 0.458,
            orientation[0], orientation[1], orientation[2], orientation[3]
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
            orientation[0], orientation[1], orientation[2], orientation[3]
        ])

    def get_place_joint_target(self):
        place_joint_target = list(self.get_parameter('place_joint_target').value)
        place_joint_gripper = float(self.get_parameter('place_joint_gripper').value)
        if len(place_joint_target) != 6:
            self.get_logger().warn(
                f"place_joint_target 长度应为6，当前为{len(place_joint_target)}，改用预设值"
            )
            place_joint_target = [
                1.580740392,
                1.7780669200000003,
                -0.690817288,
                1.7106981920000002,
                -1.1191895960000002,
                -0.31294536000000006,
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

    def publish_joint_state_with_gripper(self, joint_positions, gripper_value):
        """发布带有夹爪值的关节状态"""
        # 创建消息
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        
        # 确保关节位置是列表
        if isinstance(joint_positions, np.ndarray):
            joint_positions = joint_positions.tolist()
        joint_positions.append(0.0)#加入joint7
        joint_positions.append(0.0)#加入joint8
        
        # 为joint7(夹爪)设置值
        positions_with_gripper = list(joint_positions)       
        positions_with_gripper[6] = gripper_value  # joint7是夹爪
        msg.position = positions_with_gripper
        #self.get_logger().info(f"joint_positions直－－－－－ ：{positions_with_gripper}")
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
        # 等待一个时间步
        time.sleep(self.dt)
    
    def execute_trajectory(self, trajectory, gripper_value=None):
        """执行轨迹，可选指定夹爪值"""
        if len(trajectory) == 0:
            self.get_logger().warning("轨迹为空，无法执行")
            return False
            
        self.get_logger().info(f"开始执行 {len(trajectory)} 个轨迹点")
        
        # 发布轨迹
        for i in range(len(trajectory)):
            
            # 如果指定了夹爪值，使用指定值
            if gripper_value is not None:

                self.publish_joint_state_with_gripper(trajectory[i], gripper_value)
            else:
                # 默认使用轨迹中的值
                self.publish_joint_state_with_gripper(trajectory[i], trajectory[i][6] if len(trajectory[i]) > 6 else self.gripper_open_value)
        #
        self.get_logger().info("轨迹执行完成")
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
    
    def operate_gripper(self, current_joints, gripper_value):
        """操作夹爪 - 仅改变夹爪值，保持其他关节不变"""
        self.get_logger().info(f"操作夹爪，设置值: {gripper_value}")
        
        # 创建一个1点轨迹，包含当前关节值
        trajectory = [current_joints]
        
        # 执行，指定夹爪值
        return self.execute_trajectory(trajectory, gripper_value)
    
    def get_current_arm_joint_positions(self):
        if self.latest_joint_state is None:
            return None

        current_joint_positions = []
        for name in self.motion_gen.kinematics.joint_names:
            if name in self.latest_joint_state.name:
                idx = self.latest_joint_state.name.index(name)
                current_joint_positions.append(self.latest_joint_state.position[idx])
            else:
                current_joint_positions.append(0.0)
        return current_joint_positions

    def apply_gripper_relief(self):
        if self.gripper_relief_applied:
            return True

        current_joint_positions = self.get_current_arm_joint_positions()
        if current_joint_positions is None:
            self.get_logger().warn("無法取得當前關節位置，略過夾爪減壓")
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

        self.get_logger().warn("夾爪減壓命令執行失敗")
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
                orientation[0], orientation[1], orientation[2], orientation[3]
            ])

            if idx == 0:
                self.get_logger().info("開始規劃")
            else:
                self.get_logger().info(
                    f"固定姿態失敗，嘗試候選姿態 {idx}/{len(orientation_candidates) - 1}: "
                    f"{orientation.tolist()}"
                )

            if self.plan_and_execute(current_joints, target_pose, gripper_value):
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
            
        try:
            # 提取当前关节位置
            current_joint_positions = []
            #self.get_logger().info(f"當前Joint直 --------： {current_joint_positions}")
            for name in self.motion_gen.kinematics.joint_names:
                if name in self.latest_joint_state.name:
                    idx = self.latest_joint_state.name.index(name)
                    current_joint_positions.append(self.latest_joint_state.position[idx])

                else:
                    self.get_logger().warning(f"找不到关节 {name}，使用默认值0.0")
                    current_joint_positions.append(0.0)
            
            # 状态机处理
            if self.current_state == self.STATE_IDLE:
                # 查找目标TF变换
                try:
                    #tf = self.tf_buffer.lookup_transform(
                    #    'base_link', 'object_in_base', rclpy.time.Time(), timeout=Duration(seconds=1.0))
                    tf = self.tf_buffer.lookup_transform(
                        'base_link', 'camera_object_frame', rclpy.time.Time(), timeout=Duration(seconds=1.0))
                    # ✅ 插入這段檢查 TF 是否新鮮
                    now = self.get_clock().now()
                    tf_time = tf.header.stamp
                    tf_msg_time = rclpy.time.Time.from_msg(tf_time)
                    tf_age = now - tf_msg_time

                    if tf_age > Duration(seconds=self.tf_max_age_sec):
                        self.get_logger().warn(f"⚠️ TF已過期 ({tf_age.nanoseconds/1e9:.2f}s)，忽略此次抓取")
                        return

                    if tf_msg_time <= self.tf_accept_after_time:
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
                    self.pick_orientation = np.array([orientation.w, orientation.x, orientation.y, orientation.z])
                    
                    self.get_logger().info(f'开始新的Pick and Place，目标位置: {self.pick_position}')
                    self.last_position = self.pick_position.copy()
                    self.current_state = self.STATE_MOVE_TO_PICK
                    self.get_logger().info(f'狀態 ： {self.current_state}')
                    self.executing = False
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    # 无法找到目标，继续等待
                    self.get_logger().debug("无法找到目标，继续等待")
                    
            elif self.current_state == self.STATE_MOVE_TO_PICK:
                if self.plan_and_execute_pick_with_fallback(
                    current_joint_positions,
                    self.pick_position,
                    self.pick_orientation,
                    self.gripper_open_value,
                ):
                    
                    self.current_state = self.STATE_GRASP
                    self.get_logger().info("規劃到夾取位置成功--- 等待2秒")
                    time.sleep(2.0)
                else:
                    # 规划失败，回到IDLE状态
                    self.current_state = self.STATE_IDLE
                    self.get_logger().error(f"規劃失敗")
                
            elif self.current_state == self.STATE_GRASP:
                # 关闭夹爪
                if self.operate_gripper(current_joint_positions, self.gripper_close_value):                
                    self.get_logger().info("夹取成功，等待一段时间...")   
                    time.sleep(1)  # 等待一秒确保夹紧             
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
                    time.sleep(3.0)
                else:
                    self.pending_place_after_home = False
                    self.current_state = self.STATE_IDLE
                return
                
                # 创建放置姿态
                #place_pose = Pose.from_list([
                #    self.place_position_x, self.place_position_y, self.place_position_z,
                #     0.459, 0.523, 0.568, -0.440
                #])
                place_joint_positions, place_gripper_value = self.get_place_joint_target()
                place_pose = place_joint_positions
                if False:
                    time.sleep(1.5)
                    home_joint_positions, home_gripper_value = self.get_home_joint_target()
                    if self.execute_trajectory([home_joint_positions], home_gripper_value):
                        self.pending_place_after_home = False
                        self.tf_accept_after_time = self.get_clock().now() + Duration(seconds=self.post_home_tf_settle_sec)
                        self.current_state = self.STATE_IDLE
                        self.previous_state = self.STATE_IDLE
                    else:
                        self.current_state = self.STATE_IDLE
                else:   
                    self.get_logger().info(f"放置-位置{place_pose}")
                    # 规划并移动到放置位置，夹爪保持关闭
                    if self.execute_trajectory([place_joint_positions], place_gripper_value):
                        self.current_state = self.STATE_RELEASE
                        self.get_logger().info(f"等待3秒到放置點")
                        time.sleep(3.0)
                    else:
                        # 规划失败，回到IDLE状态
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
                return
                # 打开夹爪
                if self.operate_gripper(current_joint_positions, self.gripper_open_value):
                    self.get_logger().info("释放成功，等待2秒回到home點...")
                    time.sleep(1.0)  # 等待一秒确保释放
                    self.current_state = self.STATE_MOVE_TO_HOME

                else:
                    # 打开夹爪失败，回到IDLE状态
                    self.current_state = self.STATE_IDLE
                
            elif self.current_state == self.STATE_MOVE_TO_HOME:
            
                self.get_logger().info("回到home點!")
                
                time.sleep(1.0)
                
                # 创建回到初始姿态的目标
                
                # 这里使用一个略高于抓取位置的点作为HOME位置
                
                home_pose = self.get_home_pose()
                if True:
                    0.121, 0.0, 0.458,  # 预设的HOME位置
                    0.685, 0.0, 0.729, 0.0  # 默认方向
                    pass
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
                        time.sleep(2.0)            
                        self.tf_accept_after_time = self.get_clock().now() + Duration(seconds=self.post_home_tf_settle_sec)
                        self.current_state = self.STATE_IDLE                    
                    else:
                        # 规划失败，直接回到IDLE状态
                        self.current_state = self.STATE_IDLE
            
        except Exception as e:
            self.get_logger().error(f"Pick and Place出错: {e}")
            self.current_state = self.STATE_IDLE
            self.executing = True
        finally:
            if self.current_state == self.STATE_IDLE:
                self.executing = False
                
                
    def joint_callback(self, msg: JointState):
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
    def T2(self):
        self.get_logger().info(f"Timer執行中...\n當前pre_pick_check:{self.pre_pick_check}\n當前pick_check:{self.pick_check}\n當前current_state:{self.current_state}\n當前previous_state:{self.previous_state}")
        # 轉發給 /web_tf topic
        if self.gripper_data_event:
            self.gripper_check_timer += 1#避免無限迴圈
            #if self.pre_pick_check != self.pick_check:
            if self.current_state == self.STATE_WATTING_GRIPPER:#判斷當前主線程是否在等待
                #if self.pre_pick_check != self.pick_check:
                    if self.pick_check == 1:
                        self.get_logger().info("✅ 夾取成功，Pick_check = 1")
                        self.pending_place_after_home = True
                        self.current_state = self.STATE_MOVE_TO_HOME
                        self.previous_state = self.STATE_MOVE_TO_HOME
                        self.pre_pick_check = self.pick_check#最後才shift                       
                    else:
                        self.get_logger().warn("❌ 夾取超時，Pick_check = 0")
                        self.pending_place_after_home = False
                        self.current_state = self.STATE_MOVE_TO_HOME
                        self.pre_pick_check = self.pick_check#最後才shift 
                                                        
                    self.gripper_data_event = False                
                    self.gripper_check_timer = 0
                
            if self.gripper_check_timer >= 50:
                self.gripper_check_timer = 0
                self.get_logger().warn("❌ 判斷夾取超時，Pick_check = 0")
                
             
                
        
 
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


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到键盘中断，正在关闭...")
    except Exception as e:
        node.get_logger().error(f"运行出错: {e}")
    finally:
        # 清理资源
        if node.enable_rosbridge:
            close_rosbridge()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
