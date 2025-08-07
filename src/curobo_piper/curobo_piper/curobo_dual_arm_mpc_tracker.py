#!/usr/bin/env python3
"""
雙臂獨立MPC跟蹤節點
功能：
1. 使用兩個獨立的MPC控制器分別控制左臂和右臂
2. 每個手臂有自己的ee_link和目標跟隨能力
3. 可以實現真正的雙臂協作控制
4. 避免單一MPC中一個手臂跟隨、另一個避障的問題
"""
from scipy.spatial.transform import Rotation as Rotation_R
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import JointState
import tf2_ros
import numpy as np
import torch
import time
import threading
import traceback
import math

# CuRobo 導入
from curobo.types.math import Pose
from curobo.types.robot import JointState as CuroboJointState
from curobo.wrap.reacher.mpc import MpcSolver, MpcSolverConfig
from curobo.rollout.rollout_base import Goal
from curobo.util_file import get_world_configs_path, join_path, load_yaml
from curobo.geom.types import WorldConfig
from curobo.types.base import TensorDeviceType

# ROSBridge導入 (如果需要)
try:
    from rosbridge_websocket import init_rosbridge, publish_joint_state, close_rosbridge
except ImportError:
    print("警告: 未找到rosbridge_websocket模块，無法發送關節值到ROS1")

class DualArmIndependentMPCTracker(Node):
    def __init__(self):
        super().__init__('dual_arm_independent_mpc_tracker')
        
        # 初始化張量設備類型
        self.tensor_args = TensorDeviceType()
        
        # 聲明參數
        self.declare_parameter('enable_rosbridge', False)
        self.declare_parameter('rosbridge_host', '192.168.3.125')
        self.declare_parameter('rosbridge_port', 9090)
        self.declare_parameter('frame_id', 'piper_single')
        self.declare_parameter('target_tf_frame', 'cam3_object_in_base')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tf_timeout', 0.5)
        self.declare_parameter('control_frequency', 20.0)
        self.declare_parameter('gripper_value', -0.05)
        self.declare_parameter('position_threshold', 0.003)
        
        # 左右手臂配置
        self.declare_parameter('left_arm_config', 'trip_piper_left.yml')  # 左臂配置（可能是雙臂）
        self.declare_parameter('right_arm_config', 'trip_piper_right.yml')  # 右臂配置（可能是雙臂）
        self.declare_parameter('mpc_max_iters', 30)
        
        # 協作策略參數
        self.declare_parameter('cooperation_mode', 'offset_distance')  # 'synchronized', 'offset', 'leader_follower', 'offset_distance'
        self.declare_parameter('arm1_offset_x', 0.0)   # arm1相對於目標的X偏移
        self.declare_parameter('arm1_offset_y', 0.0)   # arm1相對於目標的Y偏移  
        self.declare_parameter('arm1_offset_z', 0.0)   # arm1相對於目標的Z偏移
        self.declare_parameter('arm2_offset_x', 0.0)   # arm2相對於目標的X偏移
        self.declare_parameter('arm2_offset_y', 0.0)   # arm2相對於目標的Y偏移
        self.declare_parameter('arm2_offset_z', 0.0)   # arm2相對於目標的Z偏移
        self.declare_parameter('approach_distance', 0.15)  # 與目標保持的距離(m) - 增加到15cm
        self.declare_parameter('approach_strategy', 'extreme_left_right')  # 'left_right', 'front_back', 'up_down', 'circular', 'extreme_left_right'
        
        # 讀取參數
        self.enable_rosbridge = self.get_parameter('enable_rosbridge').get_parameter_value().bool_value
        self.rosbridge_host = self.get_parameter('rosbridge_host').get_parameter_value().string_value
        self.rosbridge_port = self.get_parameter('rosbridge_port').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.target_tf_frame = self.get_parameter('target_tf_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.tf_timeout = self.get_parameter('tf_timeout').get_parameter_value().double_value
        self.control_freq = self.get_parameter('control_frequency').get_parameter_value().double_value
        self.gripper_value = self.get_parameter('gripper_value').get_parameter_value().double_value
        self.position_threshold = self.get_parameter('position_threshold').get_parameter_value().double_value
        
        self.left_arm_config = self.get_parameter('left_arm_config').get_parameter_value().string_value
        self.right_arm_config = self.get_parameter('right_arm_config').get_parameter_value().string_value
        self.mpc_max_iters = self.get_parameter('mpc_max_iters').get_parameter_value().integer_value
        self.cooperation_mode = self.get_parameter('cooperation_mode').get_parameter_value().string_value
        self.arm1_offset_x = self.get_parameter('arm1_offset_x').get_parameter_value().double_value
        self.arm1_offset_y = self.get_parameter('arm1_offset_y').get_parameter_value().double_value
        self.arm1_offset_z = self.get_parameter('arm1_offset_z').get_parameter_value().double_value
        self.arm2_offset_x = self.get_parameter('arm2_offset_x').get_parameter_value().double_value
        self.arm2_offset_y = self.get_parameter('arm2_offset_y').get_parameter_value().double_value
        self.arm2_offset_z = self.get_parameter('arm2_offset_z').get_parameter_value().double_value
        self.approach_distance = self.get_parameter('approach_distance').get_parameter_value().double_value
        self.approach_strategy = self.get_parameter('approach_strategy').get_parameter_value().string_value
        
        # 初始化ROSBridge
        if self.enable_rosbridge:
            try:
                self.rosbridge_client = init_rosbridge(self.rosbridge_host, self.rosbridge_port)
                self.get_logger().info(f'已初始化ROSBridge客戶端')
            except Exception as e:
                self.get_logger().error(f'初始化ROSBridge失敗: {e}')
                self.enable_rosbridge = False
        
        # 定義關節名稱
        self.arm1_joint_names = [f"arm1_joint{i}" for i in range(1, 7)] + ['arm1_joint7', 'arm1_joint8']
        self.arm2_joint_names = [f"arm2_joint{i}" for i in range(1, 7)] + ['arm2_joint7', 'arm2_joint8']
        
        # 創建發布者
        self.arm1_publisher = self.create_publisher(JointState, 'arm1/joint_custom_state', 10)
        self.arm2_publisher = self.create_publisher(JointState, 'arm2/joint_custom_state', 10)
        
        # 訂閱關節狀態
        self.arm1_joint_sub = self.create_subscription(
            JointState, '/arm1/joint_states', self.arm1_joint_callback, 10)
        self.arm2_joint_sub = self.create_subscription(
            JointState, '/arm2/joint_states', self.arm2_joint_callback, 10)
        
        # 創建TF監聽器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 初始化關節狀態
        self.arm1_joint_state = None
        self.arm2_joint_state = None
        self.arm1_current_positions = None
        self.arm2_current_positions = None
        
        # 跟蹤狀態
        self.target_position = None
        self.target_orientation = None
        self.last_valid_tf_time = None
        self.tracking_enabled = False
        
        # MPC狀態
        self.arm1_mpc_running = False
        self.arm2_mpc_running = False
        self.arm1_goal_buffer = None
        self.arm2_goal_buffer = None
        
        # 初始化獨立的MPC控制器
        self.get_logger().info("正在初始化獨立雙臂MPC配置...")
        self.setup_independent_mpc_controllers()
        
        # 創建控制線程
        self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.control_thread_running = True
        self.control_thread.start()
        
        # 創建定時器用於狀態監控
        self.create_timer(1.0, self.status_monitor)
        
        # 合併關節狀態（用於雙臂配置）
        self.combined_joint_state = None
        self.create_timer(1/50.0, self.combine_joint_states)
        
        self.get_logger().info("雙臂獨立MPC跟蹤節點已啟動")
        self.get_logger().info(f"協作模式: {self.cooperation_mode}")
        self.get_logger().info(f"左臂配置: {self.left_arm_config}")
        self.get_logger().info(f"右臂配置: {self.right_arm_config}")
        
        if self.cooperation_mode == 'offset_distance':
            self.get_logger().info(f"🎯 距離偏移模式:")
            self.get_logger().info(f"  - 接近策略: {self.approach_strategy}")
            self.get_logger().info(f"  - 安全距離: {self.approach_distance*100:.1f}cm")
            self.get_logger().info(f"  - ARM1偏移: X={self.arm1_offset_x:.3f}, Y={self.arm1_offset_y:.3f}, Z={self.arm1_offset_z:.3f}")
            self.get_logger().info(f"  - ARM2偏移: X={self.arm2_offset_x:.3f}, Y={self.arm2_offset_y:.3f}, Z={self.arm2_offset_z:.3f}")
        elif self.cooperation_mode == 'offset':
            self.get_logger().info(f"ARM2偏移: X={self.arm2_offset_x}, Y={self.arm2_offset_y}, Z={self.arm2_offset_z}")
        elif self.cooperation_mode == 'leader_follower':
            self.get_logger().info("ARM1主要抓取，ARM2輔助支撐")
        
        # 等待關節狀態
        self.get_logger().info("等待關節狀態數據...")
        
        # 創建一個檢查線程來驗證設置
        threading.Thread(target=self._verify_setup, daemon=True).start()

    def _verify_setup(self):
        """驗證設置的線程"""
        time.sleep(3.0)  # 等待3秒讓關節狀態穩定
        
        if self.arm1_joint_state is None or self.arm2_joint_state is None:
            self.get_logger().warning("⚠️ 關節狀態未接收到，請檢查話題 /arm1/joint_states 和 /arm2/joint_states")
            return
        
        if self.combined_joint_state is None:
            self.get_logger().warning("⚠️ 合併關節狀態失敗")
            return
            
        self.get_logger().info("✅ 關節狀態接收正常")
        self.get_logger().info(f"ARM1關節數: {len(self.arm1_current_positions) if self.arm1_current_positions else 0}")
        self.get_logger().info(f"ARM2關節數: {len(self.arm2_current_positions) if self.arm2_current_positions else 0}")
        self.get_logger().info(f"合併關節數: {len(self.combined_joint_state.position) if self.combined_joint_state else 0}")
        
        # 檢查關節數值範圍
        if self.arm1_current_positions:
            arm1_range = [min(self.arm1_current_positions), max(self.arm1_current_positions)]
            self.get_logger().info(f"ARM1關節範圍: [{arm1_range[0]:.3f}, {arm1_range[1]:.3f}]")
        if self.arm2_current_positions:
            arm2_range = [min(self.arm2_current_positions), max(self.arm2_current_positions)]
            self.get_logger().info(f"ARM2關節範圍: [{arm2_range[0]:.3f}, {arm2_range[1]:.3f}]")
        
        # 驗證MPC期望
        left_expected = len(self.left_mpc.rollout_fn.joint_names)
        right_expected = len(self.right_mpc.rollout_fn.joint_names)
        self.get_logger().info(f"左臂MPC期望: {left_expected} 關節")
        self.get_logger().info(f"右臂MPC期望: {right_expected} 關節")
        
        if left_expected > 6 or right_expected > 6:
            self.get_logger().info("✅ 檢測到雙臂配置，將使用合併關節狀態")
        else:
            self.get_logger().info("✅ 檢測到獨立配置，將使用單臂關節狀態")
            
        # 測試一下MPC是否能正常工作
        self.get_logger().info("🔬 正在進行MPC快速測試...")
        
        # 測試左臂MPC
        try:
            if self.arm1_current_positions:
                test_full_pos = self.get_full_joint_state_for_mpc(
                    self.left_mpc, self.arm1_current_positions, 'arm1')
                self.get_logger().info(f"左臂測試：提供 {len(test_full_pos)} 個關節")
        except Exception as e:
            self.get_logger().error(f"左臂MPC測試失敗: {e}")
            
        # 測試右臂MPC
        try:
            if self.arm2_current_positions:
                test_full_pos = self.get_full_joint_state_for_mpc(
                    self.right_mpc, self.arm2_current_positions, 'arm2')
                self.get_logger().info(f"右臂測試：提供 {len(test_full_pos)} 個關節")
        except Exception as e:
            self.get_logger().error(f"右臂MPC測試失敗: {e}")
            
        self.get_logger().info("🎯 系統就緒，等待TF目標...")
        
        # 測試距離偏移計算
        if self.cooperation_mode == 'offset_distance':
            test_target = np.array([0.3, 0.0, 0.5])
            test_arm1, test_arm2 = self.calculate_distance_offset_positions(test_target)
            test_dist1 = np.linalg.norm(test_arm1 - test_target)
            test_dist2 = np.linalg.norm(test_arm2 - test_target)
            self.get_logger().info(f"🧪 距離偏移測試:")
            self.get_logger().info(f"  - 測試目標: {test_target}")
            self.get_logger().info(f"  - ARM1位置: {test_arm1}, 距離: {test_dist1*100:.1f}cm")
            self.get_logger().info(f"  - ARM2位置: {test_arm2}, 距離: {test_dist2*100:.1f}cm")

    def combine_joint_states(self):
        """合併兩隻手臂的關節狀態（用於雙臂配置）"""
        if self.arm1_joint_state is None or self.arm2_joint_state is None:
            return

        try:
            # 先取arm1的1-6，再取arm2的1-6（連續順序）
            arm1_positions = [self.arm1_joint_state.position[
                             self.arm1_joint_state.name.index(f'arm1_joint{i}')
                         ] for i in range(1, 7)]
        
            arm2_positions = [self.arm2_joint_state.position[
                             self.arm2_joint_state.name.index(f'arm2_joint{i}')
                         ] for i in range(1, 7)]

            # 連續順序：先arm1_1-6，再arm2_1-6
            combined_names = [f'arm1_joint{i}' for i in range(1, 7)] + [f'arm2_joint{i}' for i in range(1, 7)]
            combined_pos = arm1_positions + arm2_positions

            # 用 JointState 消息來存儲合併結果
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = combined_names
            js.position = combined_pos
            js.velocity = [0.0] * len(combined_pos)
            js.effort = [0.0] * len(combined_pos)

            # 最終賦值
            self.combined_joint_state = js
            
        except Exception as e:
            self.get_logger().warning(f"合併關節狀態失敗: {e}")

    def setup_independent_mpc_controllers(self):
        """設置兩個獨立的MPC控制器"""
        # 創建簡單的世界配置
        world_config = {
            "cuboid": {
                "dummy": {
                    "dims": [0.0001, 0.0001, 0.0001],
                    "pose": [10.0, 10.0, 10.0, 1, 0, 0, 0.0],
                },
            },
        }
        
        try:
            # 設置左臂MPC控制器
            self.get_logger().info(f"正在加載左臂MPC配置: {self.left_arm_config}")
            left_mpc_config = MpcSolverConfig.load_from_robot_config(
                self.left_arm_config,
                world_config,
                use_cuda_graph=False,
                use_cuda_graph_metrics=False,
                self_collision_check=True,
                collision_checker_type=None,
                collision_cache={"obb": 10, "mesh": 5},
                use_mppi=True,
                use_lbfgs=False,
                use_es=False,
                store_rollouts=False,
                step_dt=0.02,
            )
            self.left_mpc = MpcSolver(left_mpc_config)
            
            # 設置右臂MPC控制器
            self.get_logger().info(f"正在加載右臂MPC配置: {self.right_arm_config}")
            right_mpc_config = MpcSolverConfig.load_from_robot_config(
                self.right_arm_config,
                world_config,
                use_cuda_graph=False,
                use_cuda_graph_metrics=False,
                self_collision_check=True,
                collision_checker_type=None,
                collision_cache={"obb": 10, "mesh": 5},
                use_mppi=True,
                use_lbfgs=False,
                use_es=False,
                store_rollouts=False,
                step_dt=0.02,
            )
            self.right_mpc = MpcSolver(right_mpc_config)
            
            # 輸出配置信息並檢查關節數量
            left_joints = self.left_mpc.rollout_fn.joint_names
            right_joints = self.right_mpc.rollout_fn.joint_names
            
            self.get_logger().info(f"左臂MPC: {len(left_joints)} 關節, batch_size: {getattr(self.left_mpc.rollout_fn, 'batch_size', 'N/A')}")
            self.get_logger().info(f"右臂MPC: {len(right_joints)} 關節, batch_size: {getattr(self.right_mpc.rollout_fn, 'batch_size', 'N/A')}")
            self.get_logger().info(f"左臂關節: {left_joints}")
            self.get_logger().info(f"右臂關節: {right_joints}")
            
            # 判斷是否為真正的獨立配置還是雙臂配置
            self.left_is_dual_config = len(left_joints) > 6
            self.right_is_dual_config = len(right_joints) > 6
            
            if self.left_is_dual_config:
                self.get_logger().warning(f"⚠️ 左臂配置包含{len(left_joints)}個關節，似乎是雙臂配置")
            else:
                self.get_logger().info(f"✅ 左臂配置包含{len(left_joints)}個關節，獨立配置")
                
            if self.right_is_dual_config:
                self.get_logger().warning(f"⚠️ 右臂配置包含{len(right_joints)}個關節，似乎是雙臂配置")
            else:
                self.get_logger().info(f"✅ 右臂配置包含{len(right_joints)}個關節，獨立配置")
            
            self.get_logger().info("✅ 獨立雙臂MPC控制器已就緒")
            
        except Exception as e:
            self.get_logger().error(f"初始化獨立MPC控制器失敗: {e}")
            self.get_logger().error(traceback.format_exc())
            raise

    def arm1_joint_callback(self, msg):
        """ARM1關節狀態回調"""
        self.arm1_joint_state = msg
        self.arm1_current_positions = self._extract_joint_positions(msg, 'arm1')

    def arm2_joint_callback(self, msg):
        """ARM2關節狀態回調"""
        self.arm2_joint_state = msg
        self.arm2_current_positions = self._extract_joint_positions(msg, 'arm2')

    def _extract_joint_positions(self, joint_msg, arm_prefix):
        """從關節狀態消息中提取指定手臂的關節位置"""
        positions = []
        for i in range(1, 7):  # joint1-joint6
            joint_name = f"{arm_prefix}_joint{i}"
            if joint_name in joint_msg.name:
                idx = joint_msg.name.index(joint_name)
                positions.append(joint_msg.position[idx])
            else:
                positions.append(0.0)
        return positions

    def get_target_tf(self):
        """獲取目標TF座標"""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.target_tf_frame, 
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
            
            # 檢查TF是否新鮮
            now = self.get_clock().now()
            tf_time = tf.header.stamp
            tf_age = now - rclpy.time.Time.from_msg(tf_time)
            
            if tf_age > Duration(seconds=self.tf_timeout):
                return None, "TF過期"
            
            # 提取位置和方向
            position = np.array([
                tf.transform.translation.x,
                tf.transform.translation.y,
                tf.transform.translation.z
            ])
            orientation = np.array([
                tf.transform.rotation.w,
                tf.transform.rotation.x,
                tf.transform.rotation.y,
                tf.transform.rotation.z
            ])
            
            self.last_valid_tf_time = now
            return (position, orientation), None
            
        except Exception as e:
            return None, str(e)

    def create_cooperation_targets(self, base_position, base_orientation):
        """根據協作模式創建兩個手臂的目標"""
        
        if self.cooperation_mode == 'synchronized':
            # 同步模式：兩隻手臂到相同位置
            arm1_pos = base_position.copy()
            arm2_pos = base_position.copy()
            arm1_ori = base_orientation.copy()
            arm2_ori = base_orientation.copy()
            
        elif self.cooperation_mode == 'offset':
            # 偏移模式：arm2有固定偏移
            arm1_pos = base_position.copy()
            arm2_pos = base_position.copy()
            arm2_pos[0] += self.arm2_offset_x
            arm2_pos[1] += self.arm2_offset_y
            arm2_pos[2] += self.arm2_offset_z
            arm1_ori = base_orientation.copy()
            arm2_ori = base_orientation.copy()
            
        elif self.cooperation_mode == 'leader_follower':
            # 主從模式：arm1主導，arm2跟隨但有偏移
            arm1_pos = base_position.copy()
            arm2_pos = base_position.copy()
            arm2_pos[1] += self.arm2_offset_y  # 側向偏移
            arm2_pos[2] += self.arm2_offset_z  # 高度偏移
            arm1_ori = base_orientation.copy()
            arm2_ori = base_orientation.copy()
            
        else:
            # 默認：同步
            arm1_pos = base_position.copy()
            arm2_pos = base_position.copy()
            arm1_ori = base_orientation.copy()
            arm2_ori = base_orientation.copy()
            
        return (arm1_pos, arm1_ori), (arm2_pos, arm2_ori)

    def calculate_distance_offset_positions(self, target_position):
        """計算保持安全距離的位置"""
        
        if self.approach_strategy == 'left_right':
            # 左右分離策略
            arm1_pos = target_position.copy()
            arm2_pos = target_position.copy()
            
            # ARM1在左側，ARM2在右側
            arm1_pos[0] += self.arm1_offset_x  # 自定義X偏移
            arm1_pos[1] += self.arm1_offset_y - self.approach_distance  # 左側 + 距離
            arm1_pos[2] += self.arm1_offset_z  # 自定義Z偏移
            
            arm2_pos[0] += self.arm2_offset_x  # 自定義X偏移  
            arm2_pos[1] += self.arm2_offset_y + self.approach_distance  # 右側 + 距離
            arm2_pos[2] += self.arm2_offset_z  # 自定義Z偏移
            
        elif self.approach_strategy == 'front_back':
            # 前後分離策略
            arm1_pos = target_position.copy()
            arm2_pos = target_position.copy()
            
            # ARM1在前方，ARM2在後方
            arm1_pos[0] += self.arm1_offset_x + self.approach_distance  # 前方 + 距離
            arm1_pos[1] += self.arm1_offset_y
            arm1_pos[2] += self.arm1_offset_z
            
            arm2_pos[0] += self.arm2_offset_x - self.approach_distance  # 後方 + 距離
            arm2_pos[1] += self.arm2_offset_y
            arm2_pos[2] += self.arm2_offset_z
            
        elif self.approach_strategy == 'up_down':
            # 上下分離策略
            arm1_pos = target_position.copy()
            arm2_pos = target_position.copy()
            
            # ARM1在上方，ARM2在下方
            arm1_pos[0] += self.arm1_offset_x
            arm1_pos[1] += self.arm1_offset_y
            arm1_pos[2] += self.arm1_offset_z + self.approach_distance  # 上方 + 距離
            
            arm2_pos[0] += self.arm2_offset_x
            arm2_pos[1] += self.arm2_offset_y
            arm2_pos[2] += self.arm2_offset_z - self.approach_distance  # 下方 + 距離
            
        elif self.approach_strategy == 'circular':
            # 圓形分佈策略
            arm1_pos = target_position.copy()
            arm2_pos = target_position.copy()
            
            # ARM1在45度角，ARM2在225度角（對角）
            angle1 = math.pi / 4  # 45度
            angle2 = angle1 + math.pi  # 225度
            
            arm1_pos[0] += self.arm1_offset_x + self.approach_distance * math.cos(angle1)
            arm1_pos[1] += self.arm1_offset_y + self.approach_distance * math.sin(angle1)
            arm1_pos[2] += self.arm1_offset_z
            
            arm2_pos[0] += self.arm2_offset_x + self.approach_distance * math.cos(angle2)
            arm2_pos[1] += self.arm2_offset_y + self.approach_distance * math.sin(angle2)
            arm2_pos[2] += self.arm2_offset_z
            
        elif self.approach_strategy == 'extreme_left_right':
            # 極端左右分離策略 - 更大的分離距離
            arm1_pos = target_position.copy()
            arm2_pos = target_position.copy()
            
            # 使用更大的分離距離
            extreme_distance = self.approach_distance * 2  # 雙倍距離
            
            # ARM1在極左，ARM2在極右
            arm1_pos[0] += self.arm1_offset_x - extreme_distance/2  # X也偏移
            arm1_pos[1] += self.arm1_offset_y - extreme_distance  # 左側極大距離
            arm1_pos[2] += self.arm1_offset_z
            
            arm2_pos[0] += self.arm2_offset_x + extreme_distance/2  # X也偏移
            arm2_pos[1] += self.arm2_offset_y + extreme_distance  # 右側極大距離
            arm2_pos[2] += self.arm2_offset_z
            
        else:
            # 默認：簡單偏移
            arm1_pos = target_position.copy()
            arm2_pos = target_position.copy()
            
            arm1_pos[0] += self.arm1_offset_x
            arm1_pos[1] += self.arm1_offset_y - self.approach_distance/2
            arm1_pos[2] += self.arm1_offset_z
            
            arm2_pos[0] += self.arm2_offset_x
            arm2_pos[1] += self.arm2_offset_y + self.approach_distance/2
            arm2_pos[2] += self.arm2_offset_z
            
        return arm1_pos, arm2_pos

    def publish_joint_commands(self, arm_positions, arm_name):
        """發佈關節指令"""
        if arm_name == 'arm1':
            publisher = self.arm1_publisher
            joint_names = self.arm1_joint_names
        else:
            publisher = self.arm2_publisher
            joint_names = self.arm2_joint_names
        
        # 創建消息
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = joint_names
        
        # 數據驗證和清理
        try:
            # 確保arm_positions是numpy數組或列表
            if hasattr(arm_positions, 'tolist'):
                arm_positions = arm_positions.tolist()
            elif not isinstance(arm_positions, list):
                arm_positions = list(arm_positions)
            
            # 只取前6個關節，並確保為float類型
            positions = []
            for i in range(min(6, len(arm_positions))):
                val = float(arm_positions[i])
                # 檢查是否為有效數值
                if np.isnan(val) or np.isinf(val):
                    self.get_logger().warning(f"{arm_name}: 關節 {i} 包含無效值 {val}，使用0.0替代")
                    val = 0.0
                # 限制在合理範圍內（以弧度為單位）
                val = max(-6.28, min(6.28, val))  # 限制在 ±2π 範圍內
                positions.append(val)
            
            # 確保有6個關節位置
            while len(positions) < 6:
                positions.append(0.0)
            
            # 添加joint7和joint8
            positions.append(float(self.gripper_value))  # joint7是夾爪
            positions.append(0.0)  # joint8
            
            # 最終驗證所有值都是有效的float
            for i, pos in enumerate(positions):
                if not isinstance(pos, (int, float)) or np.isnan(pos) or np.isinf(pos):
                    self.get_logger().error(f"{arm_name}: 位置 {i} 無效: {pos}")
                    positions[i] = 0.0
            
            msg.position = positions
            msg.velocity = [10.0] * len(positions)
            
            # 發佈到ROS2
            publisher.publish(msg)
            
            # 如果啟用ROSBridge，也發佈到ROS1
            if self.enable_rosbridge:
                topic_name = f'{arm_name}/joint_custom_state'
                publish_joint_state(topic_name, positions, joint_names, self.frame_id)
                
        except Exception as e:
            self.get_logger().error(f"{arm_name}: 發佈關節指令時出錯: {e}")
            self.get_logger().error(f"{arm_name}: arm_positions類型: {type(arm_positions)}, 值: {arm_positions}")
            # 發佈安全的零位置
            safe_positions = [0.0] * 8
            safe_positions[6] = float(self.gripper_value)
            msg.position = safe_positions
            msg.velocity = [10.0] * len(safe_positions)
            publisher.publish(msg)

    def get_full_joint_state_for_mpc(self, mpc_controller, target_arm_positions, arm_name):
        """根據MPC配置獲取完整的關節狀態"""
        expected_joints = mpc_controller.rollout_fn.joint_names
        expected_count = len(expected_joints)
        
        if expected_count <= 6:
            # 真正的獨立配置，只需要目標手臂的6個關節
            return target_arm_positions[:6]
        else:
            # 雙臂配置，需要提供完整的12個關節
            if self.combined_joint_state is None:
                self.get_logger().warning(f"{arm_name}: 合併關節狀態不可用，使用零填充")
                return [0.0] * expected_count
            
            # 按照MPC期望的順序提取關節位置
            full_positions = []
            for joint_name in expected_joints:
                if joint_name in self.combined_joint_state.name:
                    idx = self.combined_joint_state.name.index(joint_name)
                    full_positions.append(self.combined_joint_state.position[idx])
                else:
                    self.get_logger().warning(f"{arm_name}: 找不到關節 {joint_name}，使用0.0")
                    full_positions.append(0.0)
            
            return full_positions

    def control_single_arm_mpc(self, mpc_controller, current_positions, target_pose, arm_name):
        """使用獨立MPC控制單個手臂"""
        if current_positions is None or len(current_positions) < 6:
            return False
            
        try:
            # 根據MPC配置獲取正確的關節狀態
            full_joint_positions = self.get_full_joint_state_for_mpc(
                mpc_controller, current_positions, arm_name)
            
            self.get_logger().debug(f"{arm_name}: 提供 {len(full_joint_positions)} 個關節位置給MPC")
            
            # 創建CuRobo關節狀態
            cu_js = CuroboJointState(
                position=self.tensor_args.to_device(np.array(full_joint_positions)),
                velocity=self.tensor_args.to_device(np.zeros_like(full_joint_positions)),
                acceleration=self.tensor_args.to_device(np.zeros_like(full_joint_positions)),
                jerk=self.tensor_args.to_device(np.zeros_like(full_joint_positions)),
                joint_names=mpc_controller.rollout_fn.joint_names
            )
            
            # 獲取或創建goal buffer
            if arm_name == 'arm1':
                if self.arm1_goal_buffer is None:
                    goal = Goal(current_state=cu_js, goal_state=cu_js, goal_pose=target_pose)
                    self.arm1_goal_buffer = mpc_controller.setup_solve_single(goal, 1)
                    self.get_logger().info("ARM1 goal buffer 已創建")
                goal_buffer = self.arm1_goal_buffer
            else:
                if self.arm2_goal_buffer is None:
                    goal = Goal(current_state=cu_js, goal_state=cu_js, goal_pose=target_pose)
                    self.arm2_goal_buffer = mpc_controller.setup_solve_single(goal, 1)
                    self.get_logger().info("ARM2 goal buffer 已創建")
                goal_buffer = self.arm2_goal_buffer
            
            # 更新目標
            goal_buffer.goal_pose.copy_(target_pose)
            mpc_controller.update_goal(goal_buffer)
            
            # MPC step循環
            max_iters = min(self.mpc_max_iters, 20)
            success_count = 0
            
            for i in range(max_iters):
                # 重新獲取當前關節狀態（關鍵修復）
                current_full_positions = self.get_full_joint_state_for_mpc(
                    mpc_controller, current_positions, arm_name)
                
                cu_js = CuroboJointState(
                    position=self.tensor_args.to_device(np.array(current_full_positions)),
                    velocity=self.tensor_args.to_device(np.zeros_like(current_full_positions)),
                    acceleration=self.tensor_args.to_device(np.zeros_like(current_full_positions)),
                    jerk=self.tensor_args.to_device(np.zeros_like(current_full_positions)),
                    joint_names=mpc_controller.rollout_fn.joint_names
                )
                
                # 執行MPC step
                result = mpc_controller.step(cu_js, max_attempts=1)
                
                # 調試MPC結果（僅前幾步） - 移除調試調用避免錯誤
                if i < 3 and success_count == 0:
                    self.get_logger().debug(f"{arm_name} MPC step {i}: 可行性 = {result.metrics.feasible.item()}")
                
                if result.metrics.feasible.item():
                    # 獲取新的關節位置
                    new_full_positions = result.js_action.position.cpu().numpy()
                    
                    # 數據驗證
                    if np.any(np.isnan(new_full_positions)) or np.any(np.isinf(new_full_positions)):
                        self.get_logger().warning(f"{arm_name} MPC step {i}: 結果包含無效值，跳過此步")
                        continue
                    
                    # 從完整位置中提取目標手臂的關節位置
                    expected_joints = mpc_controller.rollout_fn.joint_names
                    if len(expected_joints) <= 6:
                        # 獨立配置：直接使用前6個
                        target_arm_positions = new_full_positions[:6]
                    else:
                        # 雙臂配置：提取對應手臂的關節
                        if arm_name == 'arm1':
                            # 提取arm1的關節（通常是前6個或按名稱查找）
                            arm1_indices = [i for i, name in enumerate(expected_joints) if name.startswith('arm1')]
                            if len(arm1_indices) >= 6:
                                target_arm_positions = [new_full_positions[i] for i in arm1_indices[:6]]
                            else:
                                # 如果找不到足夠的arm1關節，使用前6個
                                target_arm_positions = new_full_positions[:6]
                        else:
                            # 提取arm2的關節
                            arm2_indices = [i for i, name in enumerate(expected_joints) if name.startswith('arm2')]
                            if len(arm2_indices) >= 6:
                                target_arm_positions = [new_full_positions[i] for i in arm2_indices[:6]]
                            else:
                                # 如果找不到足夠的arm2關節，使用後6個
                                target_arm_positions = new_full_positions[6:12] if len(new_full_positions) >= 12 else new_full_positions[:6]
                    
                    # 最終數據驗證
                    target_arm_positions = np.array(target_arm_positions)
                    if len(target_arm_positions) < 6:
                        self.get_logger().warning(f"{arm_name}: 提取的關節數不足，補零到6個")
                        target_arm_positions = np.pad(target_arm_positions, (0, 6-len(target_arm_positions)), 'constant')
                    
                    # 檢查數值有效性
                    if np.any(np.isnan(target_arm_positions)) or np.any(np.isinf(target_arm_positions)):
                        self.get_logger().warning(f"{arm_name}: 提取的關節位置包含無效值，使用上一次的位置")
                        continue
                    
                    self.get_logger().debug(f"{arm_name} step {i}: 位置 [{target_arm_positions[0]:.3f}, {target_arm_positions[1]:.3f}, {target_arm_positions[2]:.3f}, ...]")
                    
                    # 發佈關節指令
                    self.publish_joint_commands(target_arm_positions, arm_name)
                    
                    # 更新當前位置用於下一次迭代
                    current_positions = target_arm_positions[:6].tolist()
                    success_count += 1
                    
                    # 控制頻率
                    time.sleep(0.02)
                else:
                    self.get_logger().warning(f"{arm_name} MPC step {i} 不可行")
                    break
            
            self.get_logger().debug(f"{arm_name} MPC完成，成功步數: {success_count}/{max_iters}")
            return success_count > 0
                
        except Exception as e:
            self.get_logger().error(f"{arm_name} MPC控制出錯: {e}")
            self.get_logger().error(traceback.format_exc())
            return False

    def control_loop(self):
        """主控制迴圈"""
        while self.control_thread_running and rclpy.ok():
            try:
                # 獲取目標TF
                tf_result, error_msg = self.get_target_tf()
                
                if tf_result is not None:
                    position, orientation = tf_result
                    
                    # 檢查位置是否有顯著變化
                    if (self.target_position is None or 
                        np.linalg.norm(position - self.target_position) > self.position_threshold):
                        
                        # 如果MPC正在運行中，跳過這次更新
                        if self.arm1_mpc_running or self.arm2_mpc_running:
                            time.sleep(1.0 / self.control_freq)
                            continue
                            
                        self.target_position = position
                        self.target_orientation = orientation
                        self.tracking_enabled = True
                        
                        # 根據協作模式創建兩個手臂的目標
                        (arm1_pos, arm1_ori), (arm2_pos, arm2_ori) = self.create_cooperation_targets(
                            position, orientation)
                            
                        r_ori = Rotation_R.from_quat([arm1_ori[0],arm1_ori[1], arm1_ori[2], arm1_ori[3]])
                        r_ori_arm2 = Rotation_R.from_quat([arm2_ori[0],arm2_ori[1], arm2_ori[2], arm2_ori[3]])
                        
                        # 建立 Z 軸旋轉 ±90 度
                        r_z90 = Rotation_R.from_euler('z', 0, degrees=True)
                        r_z_90 = Rotation_R.from_euler('z',-90, degrees=True)

                        # 合成新的方向（注意乘法順序）
                        r_result_90 = r_z90 * r_ori
                        r_result_n90 = r_z_90 * r_ori_arm2
                        
                        # 轉回四元數格式
                        quat_90 = r_result_90.as_quat()    # [x, y, z, w]
                        quat_n90 = r_result_n90.as_quat()
                        
                        # 創建目標姿態
                        arm1_target_pose = Pose.from_list([
                            arm1_pos[0], arm1_pos[1]+0.1, arm1_pos[2],
                            #arm1_ori[0], arm1_ori[1], arm1_ori[2], arm1_ori[3]
                            quat_n90[0],quat_n90[1],quat_n90[2],quat_n90[3]
                        ])
                        
                        arm2_target_pose = Pose.from_list([
                            arm2_pos[0], arm2_pos[1]-0.1, arm2_pos[2],
                            #arm2_ori[0], arm2_ori[1], arm2_ori[2], arm2_ori[3]

                            quat_90[0],quat_90[1],quat_90[2],quat_90[3]
                        ])
                        
                        # 同步控制兩個手臂
                        if self.arm1_current_positions is not None:
                            self.arm1_mpc_running = True
                            threading.Thread(target=self._arm1_control_thread, 
                                           args=(arm1_target_pose,), daemon=True).start()
                        
                        if self.arm2_current_positions is not None:
                            self.arm2_mpc_running = True
                            threading.Thread(target=self._arm2_control_thread, 
                                           args=(arm2_target_pose,), daemon=True).start()
                            
                        if self.cooperation_mode == 'offset_distance':
                            self.get_logger().info(f"🎯 雙臂距離偏移跟蹤 ({self.approach_strategy}, {self.approach_distance*100:.0f}cm):")
                            self.get_logger().info(f"  - 原始目標: [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]")
                            self.get_logger().info(f"  - ARM1目標: [{arm1_pos[0]:.3f}, {arm1_pos[1]:.3f}, {arm1_pos[2]:.3f}]")
                            self.get_logger().info(f"  - ARM2目標: [{arm2_pos[0]:.3f}, {arm2_pos[1]:.3f}, {arm2_pos[2]:.3f}]")
                            # 計算實際距離
                            arm1_dist = np.linalg.norm(arm1_pos - position)
                            arm2_dist = np.linalg.norm(arm2_pos - position)
                            arm_separation = np.linalg.norm(np.array(arm1_pos) - np.array(arm2_pos))
                            self.get_logger().info(f"  - 與原點距離: ARM1={arm1_dist*100:.1f}cm, ARM2={arm2_dist*100:.1f}cm")
                            self.get_logger().info(f"  - 兩臂間距: {arm_separation*100:.1f}cm")
                            
                            # 確認目標真的不同
                            if np.allclose(arm1_pos, arm2_pos, atol=0.001):
                                self.get_logger().error("❌ 警告：兩隻手臂的目標位置幾乎相同！檢查偏移計算！")
                            else:
                                self.get_logger().info("✅ 確認：兩隻手臂目標位置不同")
                        else:
                            self.get_logger().info(f"啟動雙臂協作跟蹤: ARM1→[{arm1_pos[0]:.3f},{arm1_pos[1]:.3f},{arm1_pos[2]:.3f}], ARM2→[{arm2_pos[0]:.3f},{arm2_pos[1]:.3f},{arm2_pos[2]:.3f}]")
                        
                else:
                    # TF丟失或過期，停止跟蹤
                    if self.tracking_enabled:
                        self.get_logger().warning(f"TF跟蹤丟失: {error_msg}")
                        self.tracking_enabled = False
                        self.arm1_mpc_running = False
                        self.arm2_mpc_running = False
                
                # 控制頻率
                time.sleep(1.0 / self.control_freq)
                
            except Exception as e:
                self.get_logger().error(f"控制迴圈出錯: {e}")
                time.sleep(0.1)

    def _arm1_control_thread(self, target_pose):
        """ARM1控制線程"""
        try:
            success = self.control_single_arm_mpc(
                self.left_mpc, self.arm1_current_positions, target_pose, 'arm1')
            self.get_logger().info(f"ARM1控制完成: {'✅' if success else '❌'}")
        finally:
            self.arm1_mpc_running = False

    def _arm2_control_thread(self, target_pose):
        """ARM2控制線程"""
        try:
            success = self.control_single_arm_mpc(
                self.right_mpc, self.arm2_current_positions, target_pose, 'arm2')
            self.get_logger().info(f"ARM2控制完成: {'✅' if success else '❌'}")
        finally:
            self.arm2_mpc_running = False

    def status_monitor(self):
        """狀態監控定時器"""
        status_msg = f"跟蹤狀態: {'啟用' if self.tracking_enabled else '停用'} | "
        status_msg += f"ARM1: {'運行中' if self.arm1_mpc_running else '待機'} | "
        status_msg += f"ARM2: {'運行中' if self.arm2_mpc_running else '待機'} | "
        status_msg += f"模式: {self.cooperation_mode}"
        
        if self.cooperation_mode == 'offset_distance':
            status_msg += f" ({self.approach_strategy}, {self.approach_distance*100:.0f}cm)"
        
        if self.target_position is not None:
            status_msg += f" | 原始目標: [{self.target_position[0]:.3f}, {self.target_position[1]:.3f}, {self.target_position[2]:.3f}]"
            
            # 顯示計算後的實際目標位置
            if self.cooperation_mode == 'offset_distance':
                try:
                    arm1_pos, arm2_pos = self.calculate_distance_offset_positions(self.target_position)
                    status_msg += f" | ARM1目標: [{arm1_pos[0]:.3f}, {arm1_pos[1]:.3f}, {arm1_pos[2]:.3f}]"
                    status_msg += f" | ARM2目標: [{arm2_pos[0]:.3f}, {arm2_pos[1]:.3f}, {arm2_pos[2]:.3f}]"
                except Exception as e:
                    status_msg += f" | 距離計算錯誤: {e}"
        
        self.get_logger().info(status_msg)

    def destroy_node(self):
        """節點銷毀時的清理工作"""
        self.get_logger().info("正在清理資源...")
        
        # 停止控制線程
        self.control_thread_running = False
        self.arm1_mpc_running = False
        self.arm2_mpc_running = False
        
        if hasattr(self, 'control_thread') and self.control_thread.is_alive():
            self.control_thread.join(timeout=2.0)
            self.get_logger().info("控制線程已停止")
        
        # 清理ROSBridge
        if self.enable_rosbridge:
            try:
                close_rosbridge()
                self.get_logger().info("ROSBridge已關閉")
            except Exception as e:
                self.get_logger().warning(f"關閉ROSBridge時出現警告: {e}")
        
        try:
            super().destroy_node()
        except Exception as e:
            self.get_logger().warning(f"銷毀節點時出現警告: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DualArmIndependentMPCTracker()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("收到鍵盤中斷，正在關閉...")
    except Exception as e:
        print(f"運行出錯: {e}")
        traceback.print_exc()
    finally:
        try:
            node.destroy_node()
        except:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()
