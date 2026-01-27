#!/usr/bin/env python3
"""
增強版雙臂MPC跟隨節點
基於原有跟隨功能，新增二階段識別與採摘：
1. 階段1: cam3全局跟隨 (現有功能)
2. 階段2: 距離觸發 → cam1/cam2精細定位
3. 階段3: 協作採摘與運輸

更新日期：2025-01-15
基於：curobo_dual_arm_mpc_tracker.py
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
from enum import Enum

# CuRobo 導入
from curobo.types.math import Pose
from curobo.types.robot import JointState as CuroboJointState
from curobo.wrap.reacher.mpc import MpcSolver, MpcSolverConfig
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
from curobo.rollout.rollout_base import Goal
from curobo.util_file import get_world_configs_path, join_path, load_yaml
from curobo.geom.types import WorldConfig
from curobo.types.base import TensorDeviceType

# ROSBridge導入 (如果需要)
try:
    from rosbridge_websocket import init_rosbridge, publish_joint_state, close_rosbridge
except ImportError:
    print("警告: 未找到rosbridge_websocket模块，無法發送關節值到ROS1")

class OperationMode(Enum):
    """操作模式枚舉"""
    TRACKING = 0           # 跟隨模式 (原有功能)
    FINE_POSITIONING = 1   # 精細定位模式
    HARVESTING = 2         # 採摘模式
    TRANSPORT = 3          # 運輸模式 (左臂專用)
    IDLE = 4              # 空閒模式

class HarvestStage(Enum):
    """採摘階段枚舉"""
    GLOBAL_TRACKING = 0      # 全局跟隨
    APPROACHING_TARGET = 1   # 接近目標
    FINE_DETECTION = 2       # 精細偵測
    PRECISE_POSITIONING = 3  # 精確定位
    COORDINATED_GRASP = 4    # 協調抓取
    TRANSPORT_RELEASE = 5    # 運輸釋放
    RETURN_TRACKING = 6      # 返回跟隨

class EnhancedDualArmTracker(Node):

    def __init__(self):
        super().__init__('enhanced_dual_arm_tracker')
        
        # 初始化張量設備類型
        self.tensor_args = TensorDeviceType()
        
        # === 原有參數設置 ===
        self.setup_original_parameters()
        
        # === 新增：精細定位參數 ===
        self.setup_fine_positioning_parameters()
        
        # === 原有初始化 ===
        self.setup_original_components()
        
        # === 新增：精細定位組件 ===
        self.setup_fine_positioning_components()
        
        # === 新增：狀態機 ===
        self.setup_enhanced_state_machine()

        # 添加夾爪測試相關變量
        self.gripper_test_state = True  # True 代表開爪，False 代表閉爪
        
        # 創建夾爪測試定時器（1Hz）
        # self.create_timer(3.0, self.test_gripper_operation)
        
        self.cam1_initial_target_locked = False
        self.cam1_locked_target_pose = None
        self.cam1_lock_timestamp = 0
        self.cam1_target_lock_timeout = 50.0  # 10秒後解鎖，允許重新檢測

        self.get_logger().info("增強版雙臂MPC跟隨節點已啟動")
        self.get_logger().info("🎯 支援功能：全局跟隨 + 精細定位 + 協作採摘")


    def test_gripper_operation(self):
        """測試夾爪開合操作"""
        try:
            if self.gripper_test_state:
                # 開爪
                # self.operate_gripper('arm1', self.gripper_open_value)
                # self.operate_gripper('arm2', self.gripper_open_value)
                self.publish_joint_commands_with_specific_joint('arm1', 5, 0,0)
                self.get_logger().info("📢 測試：ARM2 夾爪開啟")
            else:
                # 閉爪
                self.publish_joint_commands_with_specific_joint('arm1', 5, 2.09,-0.05)
                # self.operate_gripper('arm1', self.gripper_close_value)
                # self.operate_gripper('arm2', self.gripper_close_value)
                self.get_logger().info("📢 測試：ARM2 夾爪關閉")
            
            # 切換狀態
            self.gripper_test_state = not self.gripper_test_state
            
        except Exception as e:
            self.get_logger().error(f"夾爪測試出錯: {e}")
            

    def setup_original_parameters(self):
        """設置原有參數（保持不變）"""
        # 基本參數
        self.declare_parameter('enable_rosbridge', False)
        self.declare_parameter('rosbridge_host', '192.168.3.125')
        self.declare_parameter('rosbridge_port', 9090)
        self.declare_parameter('frame_id', 'piper_single')
        self.declare_parameter('target_tf_frame', 'cam3_object_frame')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tf_timeout', 0.5)
        self.declare_parameter('control_frequency', 20.0)
        self.declare_parameter('gripper_value', -0.05)
        self.declare_parameter('position_threshold', 0.003)
        
        # 左右手臂配置
        self.declare_parameter('left_arm_config', 'trip_piper_left_limit_with_car.yml')
        self.declare_parameter('right_arm_config', 'trip_piper_right_limit_with_car.yml')
        self.declare_parameter('mpc_max_iters', 30)
        
        # 協作策略參數
        self.declare_parameter('cooperation_mode', 'offset')
        self.declare_parameter('arm1_offset_x', 0.1)
        self.declare_parameter('arm1_offset_y', 0.12)
        self.declare_parameter('arm1_offset_z', 0.1)
        self.declare_parameter('arm2_offset_x', 0.05)#數值越大 距離目標越遠
        self.declare_parameter('arm2_offset_y', 0.02)
        self.declare_parameter('arm2_offset_z', 0.19)
        self.declare_parameter('approach_distance', 0.15)
        self.declare_parameter('approach_strategy', 'extreme_left_right')
        
        # 讀取原有參數
        self.enable_rosbridge = self.get_parameter('enable_rosbridge').value
        self.rosbridge_host = self.get_parameter('rosbridge_host').value
        self.rosbridge_port = self.get_parameter('rosbridge_port').value
        self.frame_id = self.get_parameter('frame_id').value
        self.target_tf_frame = self.get_parameter('target_tf_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.tf_timeout = self.get_parameter('tf_timeout').value
        self.control_freq = self.get_parameter('control_frequency').value
        self.gripper_value = self.get_parameter('gripper_value').value
        self.position_threshold = self.get_parameter('position_threshold').value
        
        self.left_arm_config = self.get_parameter('left_arm_config').value
        self.right_arm_config = self.get_parameter('right_arm_config').value
        self.mpc_max_iters = self.get_parameter('mpc_max_iters').value
        self.cooperation_mode = self.get_parameter('cooperation_mode').value
        self.arm1_offset_x = self.get_parameter('arm1_offset_x').value
        self.arm1_offset_y = self.get_parameter('arm1_offset_y').value
        self.arm1_offset_z = self.get_parameter('arm1_offset_z').value
        self.arm2_offset_x = self.get_parameter('arm2_offset_x').value
        self.arm2_offset_y = self.get_parameter('arm2_offset_y').value
        self.arm2_offset_z = self.get_parameter('arm2_offset_z').value
        self.approach_distance = self.get_parameter('approach_distance').value
        self.approach_strategy = self.get_parameter('approach_strategy').value

    def setup_fine_positioning_parameters(self):
        """設置精細定位相關參數"""
        # 精細定位觸發參數
        self.declare_parameter('fine_trigger_distance', 0.2)     # 觸發精細定位的距離閾值
        self.declare_parameter('fine_positioning_timeout', 6.0)  # 精細定位超時時間
        self.declare_parameter('fine_mpc_steps', 30)            # 精細定位MPC最大步數
        
        # 採摘參數
        self.declare_parameter('grasp_approach_distance', 0.05)   # 抓取接近距離
        self.declare_parameter('support_approach_distance', 0.05) # 支撐接近距離
        self.declare_parameter('grasp_offset_z', -0.02)          # 抓取Z偏移
        self.declare_parameter('support_offset_z', 0.03)         # 支撐Z偏移
        
        # 運輸參數
        self.declare_parameter('drop_position_x', -0.0174)
        self.declare_parameter('drop_position_y', 0.321)
        self.declare_parameter('drop_position_z', 0.250)
        
        # 夾爪參數
        self.declare_parameter('gripper_open_value', -0.05)
        self.declare_parameter('gripper_close_value', 0.0)
        
        # 讀取新參數
        self.fine_trigger_distance = self.get_parameter('fine_trigger_distance').value
        self.fine_positioning_timeout = self.get_parameter('fine_positioning_timeout').value
        self.fine_mpc_steps = self.get_parameter('fine_mpc_steps').value
        self.fine_target_search_start_time = None

        self.grasp_approach_distance = self.get_parameter('grasp_approach_distance').value
        self.support_approach_distance = self.get_parameter('support_approach_distance').value
        self.grasp_offset_z = self.get_parameter('grasp_offset_z').value
        self.support_offset_z = self.get_parameter('support_offset_z').value
        
        self.drop_position = np.array([
            self.get_parameter('drop_position_x').value,
            self.get_parameter('drop_position_y').value,
            self.get_parameter('drop_position_z').value
        ])
        
        self.gripper_open_value = self.get_parameter('gripper_open_value').value
        self.gripper_close_value = self.get_parameter('gripper_close_value').value

    def setup_original_components(self):
        """設置原有組件（保持不變）"""
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

        # 添加夾爪狀態監控相關變量
        self.arm1_gripper_position = None
        self.arm2_gripper_position = None

        self.arm1_gripper_status = False
        self.arm2_gripper_status = False

        self.arm1_mpc_completed = False
        self.arm2_mpc_completed = False

        self.arm2_gripper_bag = None  # 用於存儲右臂袋子姿態
        
        # 添加兩個手臂的真實關節狀態訂閱
        self.arm1_real_joint_sub = self.create_subscription(
            JointState,
            '/arm1/joint_states_single',
            self.arm1_joint_states_single_callback,
            10
        )
        
        self.arm2_real_joint_sub = self.create_subscription(
            JointState,
            '/arm2/joint_states_single',
            self.arm2_joint_states_single_callback,
            10
        )
        # 初始化關節狀態
        self.arm1_joint_state = None
        self.arm2_joint_state = None
        self.arm1_current_positions = None
        self.arm2_current_positions = None
        
        # 跟蹤狀態（原有）
        self.target_position = None
        self.target_orientation = None
        self.last_valid_tf_time = None
        self.tracking_enabled = False
        
        # MPC狀態（原有）
        self.arm1_mpc_running = False
        self.arm2_mpc_running = False
        self.arm1_goal_buffer = None
        self.arm2_goal_buffer = None

        
        
        # 初始化獨立的MPC控制器
        self.get_logger().info("正在初始化獨立雙臂MPC配置...")
        self.setup_independent_mpc_controllers()

        # 初始化獨立的Motion_gen控制器
        self.get_logger().info("正在初始化獨立雙臂Motion_gen配置...")

        # 合併關節狀態（用於雙臂配置）
        self.combined_joint_state = None
        self.create_timer(1/50.0, self.combine_joint_states)
    def arm1_joint_states_single_callback(self, msg: JointState):
        """監控 arm1 真實關節狀態"""
        try:
            gripper_index = msg.name.index('arm1_gripper')
            self.arm1_gripper_position = msg.position[gripper_index]
            
            # 新增：記錄上一次的夾爪狀態
            if not hasattr(self, 'last_arm1_gripper_pos'):
                self.last_arm1_gripper_pos = self.arm1_gripper_position
            
            # 判斷夾爪狀態變化
            if (self.last_arm1_gripper_pos > 0.2 and  # 之前是開爪
                0 < self.arm1_gripper_position < 0.2):  # 現在是閉合且有夾到物體
                self.arm1_gripper_status = True
                # self.get_logger().info("✅ ARM1 成功夾取物體")
            elif self.arm1_gripper_position <= 0:  # 完全閉合，可能是夾空
                self.arm1_gripper_status = False
                # self.get_logger().info("❌ ARM1 可能夾空")
                
            self.last_arm1_gripper_pos = self.arm1_gripper_position
            
        except ValueError:
            self.get_logger().warn("無法在 arm1 找到夾爪關節")

    def arm2_joint_states_single_callback(self, msg: JointState):
        """監控 arm2 真實關節狀態"""
        try:
            gripper_index = msg.name.index('arm2_gripper')
            self.arm2_gripper_position = msg.position[gripper_index]
            
            # 新增：記錄上一次的夾爪狀態
            if not hasattr(self, 'last_arm2_gripper_pos'):
                self.last_arm2_gripper_pos = self.arm2_gripper_position
            
            # 判斷夾爪狀態變化
            if (self.last_arm2_gripper_pos > 0.2 and  # 之前是開爪
                0 < self.arm2_gripper_position < 0.2):  # 現在是閉合且有夾到物體
                self.arm2_gripper_status = True
                # self.get_logger().info("✅ ARM2 成功夾取物體")
            elif self.arm2_gripper_position <= 0:  # 完全閉合，可能是夾空
                self.arm2_gripper_status = False
                # self.get_logger().info("❌ ARM2 可能夾空")
                
            self.last_arm2_gripper_pos = self.arm2_gripper_position
                
        except ValueError:
            self.get_logger().warn("無法在 arm2 找到夾爪關節")

    def setup_fine_positioning_components(self):
        """設置精細定位組件"""
        # 精細定位TF目標
        self.left_fine_target_pose = None   # cam1_object_in_base
        self.right_fine_target_pose = None  # cam2_object_in_base
        
        # 距離監測
        self.arm1_distance_to_target = float('inf')
        self.arm2_distance_to_target = float('inf')
        
        # 精細定位計數器
        self.fine_positioning_steps = {'arm1': 0, 'arm2': 0}
        self.fine_positioning_start_time = 0
        
        # 夾爪狀態監測
        self.arm1_gripper_closed = False
        self.arm2_gripper_closed = False

    def setup_enhanced_state_machine(self):
        """設置增強狀態機"""
        # 操作模式
        self.current_mode = OperationMode.IDLE
        self.current_stage = HarvestStage.GLOBAL_TRACKING
        
        # 模式切換標誌
        self.mode_transition_pending = False
        self.fine_positioning_triggered = False
        self.harvest_triggered = False
        
        # 任務完成標誌
        self.both_arms_at_target = False
        self.fine_targets_detected = False
        self.harvest_completed = False
        
        # 創建增強控制線程
        self.enhanced_control_thread = threading.Thread(target=self.enhanced_control_loop, daemon=True)
        self.enhanced_control_thread_running = True
        self.enhanced_control_thread.start()
        
        # 創建原有控制線程
        self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.control_thread_running = True
        self.control_thread.start()
        
        # 狀態監控定時器
        self.create_timer(1.0, self.enhanced_status_monitor)

    def setup_independent_mpc_controllers(self):
        """設置兩個獨立的MPC控制器（原有函數，保持不變）"""
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
                self.left_arm_config, world_config,
                use_cuda_graph=False, use_cuda_graph_metrics=False,
                self_collision_check=False, collision_checker_type=None,
                collision_cache={"obb": 10, "mesh": 5},
                use_mppi=True, use_lbfgs=False, use_es=False,
                store_rollouts=False, step_dt=0.04,
            )
            self.left_mpc = MpcSolver(left_mpc_config)
            
            # 設置右臂MPC控制器
            self.get_logger().info(f"正在加載右臂MPC配置: {self.right_arm_config}")
            right_mpc_config = MpcSolverConfig.load_from_robot_config(
                self.right_arm_config, world_config,
                use_cuda_graph=False, use_cuda_graph_metrics=False,
                self_collision_check=False, collision_checker_type=None,
                collision_cache={"obb": 10, "mesh": 5},
                use_mppi=True, use_lbfgs=False, use_es=False,
                store_rollouts=False, step_dt=0.04,
            )
            self.right_mpc = MpcSolver(right_mpc_config)
            
            # 輸出配置信息
            left_joints = self.left_mpc.rollout_fn.joint_names
            right_joints = self.right_mpc.rollout_fn.joint_names
            
            self.get_logger().info(f"左臂MPC: {len(left_joints)} 關節")
            self.get_logger().info(f"右臂MPC: {len(right_joints)} 關節")
            
            # 判斷配置類型
            self.left_is_dual_config = len(left_joints) > 6
            self.right_is_dual_config = len(right_joints) > 6
            
            self.get_logger().info("✅ 獨立雙臂MPC控制器已就緒")
            
        except Exception as e:
            self.get_logger().error(f"初始化獨立MPC控制器失敗: {e}")
            raise

    # ==================== 原有回調函數 (保持不變) ====================
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
        for i in range(1, 7):
            joint_name = f"{arm_prefix}_joint{i}"
            if joint_name in joint_msg.name:
                idx = joint_msg.name.index(joint_name)
                positions.append(joint_msg.position[idx])
            else:
                positions.append(0.0)
        return positions

    def combine_joint_states(self):
        """合併兩隻手臂的關節狀態（原有函數，保持不變）"""
        if self.arm1_joint_state is None or self.arm2_joint_state is None:
            return

        try:
            arm1_positions = [self.arm1_joint_state.position[
                             self.arm1_joint_state.name.index(f'arm1_joint{i}')
                         ] for i in range(1, 7)]
        
            arm2_positions = [self.arm2_joint_state.position[
                             self.arm2_joint_state.name.index(f'arm2_joint{i}')
                         ] for i in range(1, 7)]

            combined_names = [f'arm1_joint{i}' for i in range(1, 7)] + [f'arm2_joint{i}' for i in range(1, 7)]
            combined_pos = arm1_positions + arm2_positions

            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = combined_names
            js.position = combined_pos
            js.velocity = [0.0] * len(combined_pos)
            js.effort = [0.0] * len(combined_pos)

            self.combined_joint_state = js
            
        except Exception as e:
            self.get_logger().warning(f"合併關節狀態失敗: {e}")

    # ==================== 原有TF和目標處理函數 (保持不變) ====================
    def get_target_tf(self):
        """獲取目標TF座標（原有函數，保持不變）"""
        # try:
        #     tf = self.tf_buffer.lookup_transform(
        #         self.base_frame, self.target_tf_frame, 
        #         rclpy.time.Time(), timeout=Duration(seconds=0.1))
            
        #     now = self.get_clock().now()
        #     tf_time = tf.header.stamp
        #     tf_age = now - rclpy.time.Time.from_msg(tf_time)
            
        #     if tf_age > Duration(seconds=self.tf_timeout):
        #         return None, "TF過期"
            
        #     position = np.array([
        #         tf.transform.translation.x,
        #         tf.transform.translation.y,
        #         tf.transform.translation.z
        #     ])
        #     orientation = np.array([
        #         tf.transform.rotation.w,
        #         tf.transform.rotation.x,
        #         tf.transform.rotation.y,
        #         tf.transform.rotation.z
        #     ])
            
        #     self.last_valid_tf_time = now
        #     return (position, orientation), None
            
        # except Exception as e:
        #     return None, str(e)

        """獲取目標TF座標（優先使用cam1，備用cam3）"""
        # 定義兩個相機的TF frame
        cam1_frame = 'cam2_object_frame'
        cam3_frame = self.target_tf_frame  # 'cam3_object_frame'

        # 先嘗試cam1
        cam1_result = self._get_single_camera_tf(cam1_frame, "CAM2")
        if cam1_result[0] is not None:
            position, orientation = cam1_result[0]
            # CAM3 的姿態調整（保持原有邏輯）
            self.current_camera_source = "CAM2"
            return (position, orientation), None
        # cam2 失敗，嘗試cam3
        cam3_result = self._get_single_camera_tf(cam3_frame, "CAM3")
        if cam3_result[0] is not None:
            position, orientation = cam3_result[0]
            # CAM3 的姿態調整（保持原有邏輯）
            self.current_camera_source = "CAM3"
            return (position, orientation), None

        # 兩個相機都失敗
        return None, f"CAM2: {cam1_result[1]}, CAM3: {cam3_result[1]}"

    def _get_single_camera_tf(self, frame_name, camera_name):
        """獲取單一相機的TF座標"""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, frame_name, 
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
            
            now = self.get_clock().now()
            tf_time = tf.header.stamp
            tf_age = now - rclpy.time.Time.from_msg(tf_time)
            
            if tf_age > Duration(seconds=self.tf_timeout):
                return None, f"{camera_name} TF過期"
            
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
            return None, f"{camera_name} 錯誤: {str(e)}"
    
    
    def get_arm1_tf(self):
        """獲取目標TF座標（原有函數，保持不變）"""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, 'arm1_gripper_point', 
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
            
            now = self.get_clock().now()
            tf_time = tf.header.stamp
            tf_age = now - rclpy.time.Time.from_msg(tf_time)
            
            if tf_age > Duration(seconds=self.tf_timeout):
                return None, "TF過期"
            
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

    def get_arm2_tf(self):
        """獲取目標TF座標（原有函數，保持不變）"""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, 'arm2_gripper_point', 
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
            
            now = self.get_clock().now()
            tf_time = tf.header.stamp
            tf_age = now - rclpy.time.Time.from_msg(tf_time)
            
            if tf_age > Duration(seconds=self.tf_timeout):
                return None, "TF過期"
            
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
    def get_arm1_link6_tf(self):

        """獲取目標TF座標（原有函數，保持不變）"""

        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, 'arm1_link6', 
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
            
            now = self.get_clock().now()
            tf_time = tf.header.stamp
            tf_age = now - rclpy.time.Time.from_msg(tf_time)
            
            if tf_age > Duration(seconds=self.tf_timeout):
                return None, "TF過期"
            
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
        """根據協作模式創建兩個手臂的目標（原有函數，保持不變）"""
        base_position = np.array(base_position)
        base_orientation = np.array(base_orientation)

        if self.cooperation_mode == 'synchronized':
            arm1_pos = base_position.copy()
            arm2_pos = base_position.copy()
            arm1_ori = base_orientation.copy()
            arm2_ori = base_orientation.copy()
            
        elif self.cooperation_mode == 'offset':
            arm1_pos = base_position.copy()
            arm2_pos = base_position.copy()

            arm1_pos[0] -= self.arm1_offset_x
            # arm1_pos[1] += self.arm1_offset_y
            arm1_pos[2] -= self.arm1_offset_z

            arm2_pos[0] -= self.arm2_offset_x
            arm2_pos[1] -= self.arm2_offset_y
            arm2_pos[2] -= self.arm2_offset_z
            arm1_ori = base_orientation.copy()
            arm2_ori = base_orientation.copy()
            
        elif self.cooperation_mode == 'leader_follower':
            arm1_pos = base_position.copy()
            arm2_pos = base_position.copy()
            arm2_pos[1] += self.arm2_offset_y
            arm2_pos[2] += self.arm2_offset_z
            arm1_ori = base_orientation.copy()
            arm2_ori = base_orientation.copy()
            
        else:
            arm1_pos = base_position.copy()
            arm2_pos = base_position.copy()
            arm1_ori = base_orientation.copy()
            arm2_ori = base_orientation.copy()
            
        return (arm1_pos, arm1_ori), (arm2_pos, arm2_ori)

    def calculate_distance_offset_positions(self, target_position):
        """計算保持安全距離的位置（原有函數，保持不變）"""
        if self.approach_strategy == 'extreme_left_right':
            arm1_pos = target_position.copy()
            arm2_pos = target_position.copy()
            
            extreme_distance = self.approach_distance * 2
            
            arm1_pos[0] += self.arm1_offset_x - extreme_distance/2
            arm1_pos[1] += self.arm1_offset_y - extreme_distance
            arm1_pos[2] += self.arm1_offset_z
            
            arm2_pos[0] += self.arm2_offset_x + extreme_distance/2
            arm2_pos[1] += self.arm2_offset_y + extreme_distance
            arm2_pos[2] += self.arm2_offset_z
            
        else:
            arm1_pos = target_position.copy()
            arm2_pos = target_position.copy()
            
            arm1_pos[0] += self.arm1_offset_x
            arm1_pos[1] += self.arm1_offset_y - self.approach_distance/2
            arm1_pos[2] += self.arm1_offset_z
            
            arm2_pos[0] += self.arm2_offset_x
            arm2_pos[1] += self.arm2_offset_y + self.approach_distance/2
            arm2_pos[2] += self.arm2_offset_z
            
        return arm1_pos, arm2_pos

    # ==================== 新增：精細定位TF處理 ====================
    def get_fine_target_tf(self, target_frame):
        """獲取精細定位TF座標"""
        try:
            if target_frame != 'cam3_object_frame':          
                tf = self.tf_buffer.lookup_transform(
                    self.base_frame, target_frame, 
                    rclpy.time.Time(), timeout=Duration(seconds=0.1))
                
                now = self.get_clock().now()
                tf_time = tf.header.stamp
                tf_age = now - rclpy.time.Time.from_msg(tf_time)
                
                if tf_age > Duration(seconds=self.tf_timeout):
                    return None
                
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

                # 方向處理（原有邏輯）
                r_ori = Rotation_R.from_quat([orientation[0], orientation[1], orientation[2],orientation[3]])
                
                # 建立 Z 軸旋轉 ±90 度
                r_x90 = Rotation_R.from_euler('x', 0, degrees=True)            
                r_z45 = Rotation_R.from_euler('z', 40, degrees=True)


                r_result_45 = r_z45 * r_ori

                # 進行旋轉補償
                # r_result_45 = r_result_45 * r_x90

                quat_90 = r_result_45.as_quat()

                pose = Pose.from_list([
                    position[0]+0.01, position[1], position[2],
                    #orientation[0], orientation[1], orientation[2],orientation[3]
                    quat_90[1], quat_90[2],quat_90[3], quat_90[0]

                ])

            if target_frame == 'cam3_object_frame':
                pose = Pose.from_list([
                    # position[0], position[1], position[2],
                    self.arm2_gripper_bag[0]-0.08, self.arm2_gripper_bag[1]+0.03, self.arm2_gripper_bag[2]-0.05,
                    # 0.572,-0.110, 0.811, 0.042
                    0.708, -0.07, 0.699, 0.071
                    #0.708,-0.070, 0.699, 0.071
                ])

            return pose
            
        except Exception :
            self.get_logger().warning(f"獲取{target_frame} TF失敗: {traceback.format_exc()}")
            return None

    def update_fine_targets(self):
        """更新精細目標座標"""
        # 更新cam1目標 (左臂)
        cam1_pose = self.get_fine_target_tf('cam1_object_frame')
        if cam1_pose is not None:
            self.left_fine_target_pose = cam1_pose
            
        # 更新cam2目標 (右臂)
        cam2_pose = self.get_fine_target_tf('cam3_object_frame')
        
        if cam2_pose is not None:
            self.right_fine_target_pose = cam2_pose

    def calculate_distance_to_target(self):
        """計算手臂到目標的距離"""
        # if self.target_position is None:
        #     return
            
        # # 計算當前協作目標位置
        # if self.cooperation_mode == 'offset_distance':
        #     arm1_target, arm2_target = self.calculate_distance_offset_positions(self.target_position)
        # else:
        #     (arm1_target, _), (arm2_target, _) = self.create_cooperation_targets(
        #         self.target_position, self.target_orientation)
        
        # # 簡化距離計算（這裡可以根據實際情況改進）
        # # 暫時使用目標位置作為估計
        # self.arm1_distance_to_target = np.linalg.norm(arm1_target - self.target_position)
        # self.arm2_distance_to_target = np.linalg.norm(arm2_target - self.target_position)
        if self.target_position is None:
            return

        # 取得arm1末端實際位置
        try:
            tf1 = self.tf_buffer.lookup_transform(
                self.base_frame, 'arm1_gripper_point', rclpy.time.Time(), timeout=Duration(seconds=0.1))
            arm1_pos = np.array([
                tf1.transform.translation.x,
                tf1.transform.translation.y,
                tf1.transform.translation.z
            ])
            self.arm1_distance_to_target = np.linalg.norm(arm1_pos - self.target_position)
        except Exception as e:
            self.get_logger().warning(f"取得arm1末端TF失敗: {e}")
            self.arm1_distance_to_target = float('inf')

        # 取得arm2末端實際位置
        try:
            tf2 = self.tf_buffer.lookup_transform(
                self.base_frame, 'arm2_gripper_point', rclpy.time.Time(), timeout=Duration(seconds=0.1))
            arm2_pos = np.array([
                tf2.transform.translation.x,
                tf2.transform.translation.y,
                tf2.transform.translation.z
            ])
            self.arm2_distance_to_target = np.linalg.norm(arm2_pos - self.target_position)
        except Exception as e:
            self.get_logger().warning(f"取得arm2末端TF失敗: {e}")
            self.arm2_distance_to_target = float('inf')

    # ==================== 原有運動控制函數 (保持不變) ====================
    def publish_joint_commands(self, arm_positions, arm_name):
        """發佈關節指令（原有函數，保持不變）"""
        if arm_name == 'arm1':
            publisher = self.arm1_publisher
            joint_names = self.arm1_joint_names
        else:
            publisher = self.arm2_publisher
            joint_names = self.arm2_joint_names
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = joint_names
        
        try:
            if hasattr(arm_positions, 'tolist'):
                arm_positions = arm_positions.tolist()
            elif not isinstance(arm_positions, list):
                arm_positions = list(arm_positions)
            
            positions = []
            for i in range(min(6, len(arm_positions))):
                val = float(arm_positions[i])
                if np.isnan(val) or np.isinf(val):
                    self.get_logger().warning(f"{arm_name}: 關節 {i} 包含無效值 {val}，使用0.0替代")
                    val = 0.0
                val = max(-6.28, min(6.28, val))
                positions.append(val)
            
            while len(positions) < 6:
                positions.append(0.0)
            
            positions.append(float(self.gripper_value))
            positions.append(0.0)
            
            for i, pos in enumerate(positions):
                if not isinstance(pos, (int, float)) or np.isnan(pos) or np.isinf(pos):
                    self.get_logger().error(f"{arm_name}: 位置 {i} 無效: {pos}")
                    positions[i] = 0.0
            
            msg.position = positions
            msg.velocity = [10.0] * len(positions)
            
            publisher.publish(msg)
            
            if self.enable_rosbridge:
                topic_name = f'{arm_name}/joint_custom_state'
                publish_joint_state(topic_name, positions, joint_names, self.frame_id)
                
        except Exception as e:
            self.get_logger().error(f"{arm_name}: 發佈關節指令時出錯: {e}")
            safe_positions = [0.0] * 8
            safe_positions[6] = float(self.gripper_value)
            msg.position = safe_positions
            msg.velocity = [10.0] * len(safe_positions)
            publisher.publish(msg)


    def get_full_joint_state_for_mpc(self, mpc_controller, target_arm_positions, arm_name):
        """根據MPC配置獲取完整的關節狀態（原有函數，保持不變）"""
        expected_joints = mpc_controller.rollout_fn.joint_names
        expected_count = len(expected_joints)
        
        if expected_count <= 6:
            return target_arm_positions[:6]
        else:
            if self.combined_joint_state is None:
                self.get_logger().warning(f"{arm_name}: 合併關節狀態不可用，使用零填充")
                return [0.0] * expected_count
            
            full_positions = []
            for joint_name in expected_joints:
                if joint_name in self.combined_joint_state.name:
                    idx = self.combined_joint_state.name.index(joint_name)
                    full_positions.append(self.combined_joint_state.position[idx])
                else:
                    self.get_logger().warning(f"{arm_name}: 找不到關節 {joint_name}，使用0.0")
                    full_positions.append(0.0)
            
            return full_positions
    def debug_tensor_shapes(self,mpc_controller, current_joints, target_pose):
        """调试张量形状"""
        try:
            self.get_logger().info("=== 张量形状调试 ===")
            
            # 检查输入
            current_joints_array = np.array(current_joints, dtype=np.float32)
            self.get_logger().info(f"输入关节形状: {current_joints_array.shape}")
            
            # 检查MPC内部状态
            retract_cfg = mpc_controller.rollout_fn.dynamics_model.retract_config
            self.get_logger().info(f"retract_config形状: {retract_cfg.shape}")
            
            # 检查关节名称
            joint_names = mpc_controller.rollout_fn.joint_names
            self.get_logger().info(f"MPC关节名称: {joint_names}")
            self.get_logger().info(f"MPC关节数量: {len(joint_names)}")
            
            # 尝试创建状态
            test_position = self.tensor_args.to_device(current_joints_array).unsqueeze(0)
            self.get_logger().info(f"测试位置形状: {test_position.shape}")
            
        except Exception as e:
            self.get_logger().error(f"调试失败: {e}")
            import traceback
            self.get_logger().error(f"调试错误详情: {traceback.format_exc()}")

    # 添加這個除錯函數來檢查配置
    def control_single_arm_mpc(self, mpc_controller, current_positions, target_pose, arm_name):
        """使用獨立MPC控制單個手臂 - 簡化版本"""
        self.get_logger().info(f"MPC启动调试:")
        self.get_logger().info(f"  - 关节数量: {len(current_positions)}")
        self.get_logger().info(f"  - 目标位置: {target_pose.position}")
        self.get_logger().info(f"  - MPC关节名: {mpc_controller.rollout_fn.joint_names}")
  
        if current_positions is None or len(current_positions) < 6:
            return False
        if arm_name == "arm2":
            self.get_logger().info(f"現在控制右臂 : {current_positions}")
        else:
            self.get_logger().info(f"現在控制左臂 : {current_positions}")
            
        try:
            # 直接使用前6個關節位置（因為已經是獨立配置）
            joint_positions = current_positions[:6]
            self.debug_tensor_shapes(mpc_controller,joint_positions, target_pose)
            cu_js = CuroboJointState(
                position=self.tensor_args.to_device(np.array(joint_positions)),
                velocity=self.tensor_args.to_device(np.zeros(6, dtype=np.float32)),
                acceleration=self.tensor_args.to_device(np.zeros(6, dtype=np.float32)),
                jerk=self.tensor_args.to_device(np.zeros(6, dtype=np.float32)),
                joint_names=mpc_controller.rollout_fn.joint_names
            )
            if arm_name == "arm2":
                self.get_logger().info(f"現在控制右臂 : {joint_positions}")
            else:
                self.get_logger().info(f"現在控制左臂 : {joint_positions}")
            self.arm2_goal_buffer = None
            if arm_name == 'arm1':
                if self.arm1_goal_buffer is None:
                    goal = Goal(current_state=cu_js, goal_state=cu_js, goal_pose=target_pose)
                    self.arm1_goal_buffer = mpc_controller.setup_solve_single(goal, 1)
                goal_buffer = self.arm1_goal_buffer
            else:
                if self.arm2_goal_buffer is None:
                    goal = Goal(current_state=cu_js, goal_state=cu_js, goal_pose=target_pose)
                    self.arm2_goal_buffer = mpc_controller.setup_solve_single(goal, 1)
                goal_buffer = self.arm2_goal_buffer


            self.get_logger().info(f"========================================")
            if arm_name == "arm2":
                self.get_logger().info(f"現在控制右臂 : {mpc_controller.rollout_fn.joint_names}")
            else:
                self.get_logger().info(f"現在控制左臂 : {cu_js}")

            goal_buffer.goal_pose.copy_(target_pose)
            mpc_controller.update_goal(goal_buffer)
            
            max_iters = min(self.mpc_max_iters, 30)
            success_count = 0
            current_joint_positions = joint_positions
            
            for i in range(max_iters):
                cu_js = CuroboJointState(
                    position=self.tensor_args.to_device(np.array(current_joint_positions)),
                    velocity=torch.zeros_like(self.tensor_args.to_device(np.zeros(6))),
                    acceleration=torch.zeros_like(self.tensor_args.to_device(np.zeros(6))),
                    jerk=torch.zeros_like(self.tensor_args.to_device(np.zeros(6))),
                    joint_names=mpc_controller.rollout_fn.joint_names
                )
                self.get_logger().info(f"通過MPC嘗試第 {i+1} 次")

                result = mpc_controller.step(cu_js, max_attempts=2)
                if result.metrics.feasible.item():
                    new_positions = result.js_action.position.cpu().numpy()[:6]
                    
                    if np.any(np.isnan(new_positions)) or np.any(np.isinf(new_positions)):
                        continue
                    
                    self.publish_joint_commands(new_positions, arm_name)
                    self.get_logger().info(f"{arm_name} 發布成功！！")
                    current_joint_positions = new_positions.tolist()
                    success_count += 1
                    time.sleep(0.02)
                else:
                    self.get_logger().warn(f"{arm_name} 發布失敗！！")  
                    break
            
            return success_count > 0
                
        except Exception as e:
            self.get_logger().error(f"{arm_name} MPC控制出錯: {e}")
            return False

    # ==================== 新增：精細定位MPC控制 ====================
    def control_fine_positioning_mpc(self, mpc_controller, current_positions, target_pose, arm_name, gripper_value=None):
        """精細定位MPC控制 - 獨立單臂版本"""
        self.get_logger().info(f"{arm_name} 精細定位開始（獨立單臂模式）")

        if current_positions is None or len(current_positions) < 6:
            self.get_logger().error(f"{arm_name}: 無法獲取有效的當前關節位置")
            return False
        if arm_name == "arm2":
            self.get_logger().info(f"arm2 current: {current_positions}")
        try:
            step_count = 0
            max_steps = self.fine_mpc_steps
            consecutive_failures = 0
            max_consecutive_failures = 10
            
            # 直接使用前6個關節位置（不需要複雜的合併邏輯）
            arm_joint_positions = current_positions[:6]
            
            while step_count < max_steps:
                # 創建當前關節狀態 - 簡化版本
                cu_js = CuroboJointState(
                    position=self.tensor_args.to_device(np.array(arm_joint_positions)),
                    velocity=self.tensor_args.to_device(np.zeros(6)),  # 固定6個關節
                    acceleration=self.tensor_args.to_device(np.zeros(6)),
                    jerk=self.tensor_args.to_device(np.zeros(6)),
                    joint_names=mpc_controller.rollout_fn.joint_names  # 應該只有6個關節名稱
                )
                
                # 獲取或創建 goal buffer
                if arm_name == 'arm1':
                    if self.arm1_goal_buffer is None:
                        goal = Goal(current_state=cu_js, goal_state=cu_js, goal_pose=target_pose)
                        self.arm1_goal_buffer = mpc_controller.setup_solve_single(goal, 1)
                    goal_buffer = self.arm1_goal_buffer
                else:
                    if self.arm2_goal_buffer is None:
                        goal = Goal(current_state=cu_js, goal_state=cu_js, goal_pose=target_pose)
                        self.arm2_goal_buffer = mpc_controller.setup_solve_single(goal, 1)
                    goal_buffer = self.arm2_goal_buffer
                
                goal_buffer.goal_pose.copy_(target_pose)
                mpc_controller.update_goal(goal_buffer)
                
                # 執行MPC步驟
                result = mpc_controller.step(cu_js, max_attempts=1)
                
                if result.metrics.feasible.item():
                    # 成功時重置失敗計數
                    consecutive_failures = 0
                    
                    # 獲取新的關節位置 - 直接使用前6個
                    new_positions = result.js_action.position.cpu().numpy()[:6]
                    
                    # 檢查數據有效性
                    if np.any(np.isnan(new_positions)) or np.any(np.isinf(new_positions)):
                        self.get_logger().warning(f"{arm_name}: 步驟 {step_count} 包含無效值，跳過")
                        step_count += 1
                        continue
                    
                    # 發布關節指令
                    if gripper_value is not None:
                        self.publish_joint_commands_with_gripper(new_positions, arm_name, gripper_value)
                    else:
                        self.publish_joint_commands(new_positions, arm_name)
                    
                    # 更新當前位置
                    arm_joint_positions = new_positions.tolist()
                    
                    # 更新計數器
                    self.fine_positioning_steps[arm_name] = step_count
                    
                    time.sleep(0.02)  # 50Hz控制頻率
                else:
                    consecutive_failures += 1
                    self.get_logger().warning(f"{arm_name} 精細定位MPC步驟 {step_count} 不可行 (連續失敗: {consecutive_failures})")
                    
                    # 連續失敗太多次就提早退出
                    if consecutive_failures >= max_consecutive_failures:
                        self.get_logger().error(f"{arm_name} 連續失敗 {consecutive_failures} 次，提早退出")
                        break
                
                step_count += 1
            
            success_rate = (step_count - consecutive_failures) / max(step_count, 1)
            self.get_logger().info(f"{arm_name} 精細定位完成: 執行 {step_count} 步，成功率 {success_rate:.1%}")
            
            return success_rate > 0.3  # 至少30%成功率才算成功
            
        except Exception as e:
            self.get_logger().error(f"{arm_name} 精細定位MPC控制出錯: {e}")
            import traceback
            self.get_logger().error(f"詳細錯誤: {traceback.format_exc()}")
            return False

    def publish_joint_commands_with_gripper(self, arm_positions, arm_name, gripper_value):
        """發布帶有夾爪值的關節指令"""
        if arm_name == 'arm1':
            publisher = self.arm1_publisher
            joint_names = self.arm1_joint_names
        else:
            publisher = self.arm2_publisher
            joint_names = self.arm2_joint_names
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = joint_names
        
        try:
            if hasattr(arm_positions, 'tolist'):#防呆
                arm_positions = arm_positions.tolist()
            elif not isinstance(arm_positions, list):
                arm_positions = list(arm_positions)
            
            positions = []
            for i in range(min(6, len(arm_positions))):
                val = float(arm_positions[i])
                if np.isnan(val) or np.isinf(val):
                    val = 0.0
                val = max(-6.28, min(6.28, val))
                positions.append(val)
            
            while len(positions) < 6:
                positions.append(0.0)
            
            # 設置夾爪值
            positions.append(float(gripper_value))  # joint7是夾爪
            positions.append(0.0)  # joint8
            
            msg.position = positions
            msg.velocity = [10.0] * len(positions)
            
            publisher.publish(msg)
            
            if self.enable_rosbridge:
                topic_name = f'{arm_name}/joint_custom_state'
                publish_joint_state(topic_name, positions, joint_names, self.frame_id)
                
        except Exception as e:
            self.get_logger().error(f"{arm_name}: 發佈夾爪指令時出錯: {e}")
    
    def publish_joint_commands_with_specific_joint(self, arm_name, joint_index, joint_value, gripper_value=None):
        """發布指定關節的控制指令，其他關節保持當前位置"""
        if arm_name == 'arm1':
            current_positions = self.arm1_current_positions
            publisher = self.arm1_publisher
            joint_names = self.arm1_joint_names
        else:
            current_positions = self.arm2_current_positions
            publisher = self.arm2_publisher
            joint_names = self.arm2_joint_names
        
        if current_positions is None:
            self.get_logger().error(f"{arm_name}: 無法獲取當前關節位置")
            return False
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = joint_names
        joint_limits = {
            0: [-2.62, 2.62],    # joint1: ±180°
            1: [0.0, 3.14],    # joint2: ±90°  
            2: [-2.96, 0.00],    # joint3: ±180°
            3: [-1.75, 1.75],    # joint4: ±180°
            4: [-1.22, 1.22],    # joint5: ±90°
            5: [-2.09, 2.09],    # joint6: ±180°
        }
        try:
            # 使用當前位置作為基準
            positions = list(current_positions[:6])
            
            # 修改指定關節（joint_index 從0開始，第六軸是索引5）
            if 0 <= joint_index < 6:
                joint_value = float(joint_value)
                min_val, max_val = joint_limits[joint_index]
                joint_value = max(min_val, min(max_val, joint_value))
                positions[joint_index] = joint_value
            else:
                self.get_logger().error(f"關節索引 {joint_index} 超出範圍 (0-5)")
                return False
            
            # 設置夾爪值
            if gripper_value is not None:
                positions.append(float(gripper_value))
            else:
                positions.append(float(self.gripper_value))
            positions.append(0.0)  # joint8
            
            msg.position = positions
            msg.velocity = [50.0] * len(positions)
            
            publisher.publish(msg)
            
            if self.enable_rosbridge:
                topic_name = f'{arm_name}/joint_custom_state'
                publish_joint_state(topic_name, positions, joint_names, self.frame_id)
            
            self.get_logger().info(f"{arm_name}: 第{joint_index+1}軸已設置為 {joint_value:.3f} 弧度")
            return True
            
        except Exception as e:
            self.get_logger().error(f"{arm_name}: 控制第{joint_index+1}軸時出錯: {e}")
            return False

    def operate_gripper(self, arm_name, gripper_value):
        """操作夾爪"""
        if arm_name == 'arm1':
            current_positions = self.arm1_current_positions
        else:
            current_positions = self.arm2_current_positions
        
        if current_positions is None:
            return False
        
        self.publish_joint_commands_with_gripper(current_positions, arm_name, gripper_value)
        return True

    def calculate_fine_approach_pose(self, fine_pose, arm_name):
        """計算精細接近位置"""
        # 先轉 numpy array
        import torch
        pos = fine_pose.position
        if isinstance(pos, torch.Tensor):
            approach_position = pos.detach().cpu().numpy()
        else:
            approach_position = np.array(pos)

        # 修正 shape 為 (3,)
        approach_position = np.array(approach_position).reshape(-1)
        if approach_position.shape[0] != 3:
            self.get_logger().error(f"approach_position shape 異常: {approach_position.shape}, value: {approach_position}")
            approach_position = np.pad(approach_position, (0, 3-approach_position.shape[0]), 'constant')
        # quaternion 檢查
        if hasattr(fine_pose, 'quaternion'):
            quat = np.array(fine_pose.quaternion)
            if quat.shape[0] < 4:
                self.get_logger().error(f"quaternion shape 異常: {quat.shape}, value: {quat}")
                quat = np.pad(quat, (0, 4-quat.shape[0]), 'constant')
        else:
            quat = [1.0, 0.0, 0.0, 0.0]

        if arm_name == 'arm1':
            approach_position[2] += self.grasp_offset_z
        else:
            approach_position[2] += self.support_offset_z

        return Pose.from_list([
            approach_position[0], approach_position[1], approach_position[2],
            quat[0], quat[1], quat[2], quat[3]
        ])
    # ==================== 原有控制循環 (保持不變，但添加模式檢查) ====================
    def control_loop(self):
        check_missing_tf = 0
        """主控制迴圈（原有，但添加模式檢查）"""
        while self.control_thread_running and rclpy.ok():
            try:
                # 只在跟隨模式下執行原有邏輯
                if self.current_mode != OperationMode.TRACKING:
                    time.sleep(1.0 / self.control_freq)
                    continue
                
                # 獲取目標TF
                tf_result, error_msg = self.get_target_tf()
                
                if tf_result is not None:
                    check_missing_tf = 0
                    position, orientation = tf_result
                    # 檢查手臂末端之間的距離
                    arm1_tf_result, arm1_error_msg = self.get_arm1_tf()
                    arm2_tf_result, arm2_error_msg = self.get_arm2_tf()
                    
                    if arm1_tf_result is not None and arm2_tf_result is not None:
                        arm1_position, _ = arm1_tf_result
                        arm2_position, _ = arm2_tf_result
                        
                        # 計算兩個手臂末端之間的距離
                        distance_between_arms = np.linalg.norm(arm1_position - arm2_position)
                        # self.get_logger().info(f"雙臂末端距離: {distance_between_arms:.3f}m")
                        
                        # 如果距離過近，重置到初始位置
                        if distance_between_arms < 0.08:  # 8cm
                            self.get_logger().warning("⚠️ 雙臂末端距離過近，重置到初始位置以避免碰撞")
                            self.reset_to_tracking_mode()
                            continue
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
                        
                        # 計算距離
                        self.calculate_distance_to_target()
                        
                        # 檢查是否觸發精細定位
                        if (self.arm1_distance_to_target < self.fine_trigger_distance and 
                           # self.arm2_distance_to_target < self.fine_trigger_distance and
                            not self.fine_positioning_triggered):
                            
                            self.get_logger().info(f"🎯 距離觸發精細定位！ARM1距離: {self.arm1_distance_to_target:.3f}m, ARM2距離: {self.arm2_distance_to_target:.3f}m")
                            self.fine_positioning_triggered = True
                            self.fine_target_search_start_time = None

                            self.current_mode = OperationMode.FINE_POSITIONING
                            continue
                        
                        # 原有的協作目標創建邏輯
                        (arm1_pos, arm1_ori), (arm2_pos, arm2_ori) = self.create_cooperation_targets(
                            position, orientation)
                            
                        # 方向處理（原有邏輯）
                        r_ori = Rotation_R.from_quat([arm1_ori[0],arm1_ori[1], arm1_ori[2], arm1_ori[3]])
                        r_ori_arm2 = Rotation_R.from_quat([arm2_ori[0],arm2_ori[1], arm2_ori[2], arm2_ori[3]])
                        
                        # 建立 Z 軸旋轉 ±90 度
                        r_x90 = Rotation_R.from_euler('x', 0, degrees=True)
                        r_x_90 = Rotation_R.from_euler('x',-90, degrees=True)
                        
                        r_z45 = Rotation_R.from_euler('y', -20, degrees=True)
                        r_z_45 = Rotation_R.from_euler('y', 60, degrees=True)
                        
                        

                        # 合成新的方向（注意乘法順序）
                        r_result_90 = r_x90 * r_ori
                        r_result_n90 = r_x_90 * r_ori_arm2

                        r_result_90 = r_z45 * r_result_90
                        r_result_n90 = r_z_45 * r_result_n90

                        
                        quat_90 = r_result_90.as_quat()
                        quat_n90 = r_result_n90.as_quat()
                        # self.arm2_gripper_bag = [quat_90[0],quat_90[1],quat_90[2],quat_90[3]]
                        # 創建目標姿態
                        arm1_target_pose = Pose.from_list([
                            arm1_pos[0], arm1_pos[1]+0.1, arm1_pos[2],
                            quat_n90[0],quat_n90[1],quat_n90[2],quat_n90[3]
                        ])

                        # arm1_tf_result, arm1_error_msg = self.get_arm1_tf()
                        # arm1_position, _ = arm1_tf_result
                        self.arm2_gripper_bag = [arm2_pos[0], arm2_pos[1], arm2_pos[2]]
                        arm2_target_pose = Pose.from_list([
                            # arm1_position[0]-0.03, arm1_position[1], arm1_position[2]-0.2,
                            # arm2_pos[0], arm2_pos[1]-0.1, arm2_pos[2],
                            arm2_pos[0], arm2_pos[1]-0.05, arm2_pos[2]-0.1,
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
                        else:
                            self.get_logger().warning("ARM2 當前關節位置不可用，無法啟動MPC")

                        self.get_logger().info(f"🚀 雙臂跟隨中... ARM1距離: {self.arm1_distance_to_target:.3f}m, ARM2距離: {self.arm2_distance_to_target:.3f}m")
                        
                else:
                    # TF丟失或過期，停止跟蹤
                    if self.tracking_enabled:
                        check_missing_tf +=1
                        if check_missing_tf >= 20:
                
                            self.get_logger().warning(f"TF跟蹤丟失: {error_msg}")
                            self.tracking_enabled = False
                            self.arm1_mpc_running = False
                            self.arm2_mpc_running = False
                            # 等待一下確保停止
                            time.sleep(3.0)
                            # 重置系統
                            self.reset_to_tracking_mode()
                            
                            # 提供更多信息
                            self.get_logger().info("系統已重置，等待新的 TF 數據")
                                
                # 控制頻率
                time.sleep(1.0 / self.control_freq)
                
            except Exception as e:
                self.get_logger().error(f"控制迴圈出錯: {e}")
                time.sleep(0.1)

    def _arm1_control_thread(self, target_pose):
        """ARM1控制線程（原有函數，保持不變）"""
        try:
            success = self.control_single_arm_mpc(
                self.left_mpc, self.arm1_current_positions, target_pose, 'arm1')
        finally:
            self.arm1_mpc_running = False

    def _arm2_control_thread(self, target_pose):
        """ARM2控制線程（原有函數，保持不變）"""
        try:
            success = self.control_single_arm_mpc(
                self.right_mpc, self.arm2_current_positions, target_pose, 'arm2')
        finally:
            self.arm2_mpc_running = False

    # ==================== 新增：增強控制循環 ====================
    def enhanced_control_loop(self):
        """增強控制循環（處理精細定位和採摘）"""
        while self.enhanced_control_thread_running and rclpy.ok():
            try:
                if self.current_mode == OperationMode.IDLE:
                    # 檢查是否有全局目標，如果有則切換到跟隨模式
                    tf_result, _ = self.get_target_tf()
                    if tf_result is not None:
                        self.current_mode = OperationMode.TRACKING
                        self.current_stage = HarvestStage.GLOBAL_TRACKING
                        self.get_logger().info("🔍 檢測到全局目標，切換到跟隨模式")
                
                elif self.current_mode == OperationMode.FINE_POSITIONING:
                    self.handle_fine_positioning_mode()
                
                elif self.current_mode == OperationMode.HARVESTING:
                    self.handle_harvesting_mode()
                
                elif self.current_mode == OperationMode.TRANSPORT:
                    self.handle_transport_mode()
                
                time.sleep(0.1)  # 10Hz
                
            except Exception as e:
                self.get_logger().error(f"增強控制循環出錯: {e}")
                time.sleep(0.1)

    def handle_fine_positioning_mode(self):
        """處理精細定位模式"""
        if not self.fine_positioning_triggered:
            return
        self.left_fine_target_pose = None
        # 暫停全局跟隨
        self.tracking_enabled = False
        self.arm1_mpc_running = False
        self.arm2_mpc_running = False
        time.sleep(1)  # 等待一段時間以確保狀態穩定
        # 更新精細目標
        self.update_fine_targets()

        if self.fine_target_search_start_time is None:
            self.fine_target_search_start_time = time.time()

        # 檢查精細目標是否可用
        if self.left_fine_target_pose is not None: #and self.right_fine_target_pose is not None:
            self.fine_target_search_start_time = time.time()  # 重置搜尋計時
            if not hasattr(self, 'fine_positioning_started') or not self.fine_positioning_started:
                self.get_logger().info("🎯 開始精細定位階段")
                self.fine_positioning_start_time = time.time()
                self.fine_positioning_started = True

                # 重置計數器
                self.fine_positioning_steps = {'arm1': 0, 'arm2': 0}

                # 計算精細接近位置
                # arm1_fine_target = self.calculate_fine_approach_pose(self.left_fine_target_pose, 'arm1')
                # arm2_fine_target = self.calculate_fine_approach_pose(self.right_fine_target_pose, 'arm2')
                arm1_fine_target = self.left_fine_target_pose
                arm2_fine_target = self.right_fine_target_pose


                # 啟動精細定位MPC（在線程中執行）
                if self.arm1_current_positions is not None:
                    threading.Thread(target=self._arm1_fine_positioning_thread, 
                                args=(arm1_fine_target,), daemon=True).start()
                # 設置右臂的目標為左臂末端的正下方
                # arm1_tf_result, arm1_error_msg = self.get_arm1_link6_tf()

                # if arm1_tf_result is not None:
                #     arm1_position, _ = arm1_tf_result
                #     arm2_target_position = arm1_position.copy()
                #     arm2_target_pose = Pose.from_list([
                #         arm2_target_position[0], arm2_target_position[1]-0.05, arm2_target_position[2]-0.2,
                #         0.708,-0.070, 0.699, 0.071
                #         #self.arm2_gripper_bag[0], self.arm2_gripper_bag[1], self.arm2_gripper_bag[2], self.arm2_gripper_bag[3]
                #     ])

                #     # 啟動右臂移動到接水果的位置
                #     if self.arm2_current_positions is not None:
                #         threading.Thread(target=self._arm2_fine_positioning_thread, 
                #                         args=(arm2_target_pose,), daemon=True).start()

                if self.arm2_current_positions is not None:
                    # threading.Thread(target=self._arm2_control_thread, 
                    threading.Thread(target=self._arm2_fine_positioning_thread, 
                                args=(arm2_fine_target,), daemon=True).start()

        else:
            # === 新增：如果搜尋超過指定秒數還沒找到，則回到跟隨模式 ===
            elapsed_search = time.time() - self.fine_target_search_start_time
            if elapsed_search > self.fine_positioning_timeout:
                self.get_logger().warning("⏰ 精細目標搜尋超時，返回跟隨模式")
                
                self.reset_to_tracking_mode()
                return

        if hasattr(self, 'fine_positioning_started') and self.fine_positioning_started:
            elapsed_time = time.time() - self.fine_positioning_start_time
            self.get_logger().info(f"夾爪開合狀態...arm1_mpc_completed: {self.arm1_mpc_completed}, arm2_mpc_completed: {self.arm2_mpc_completed}, elapsed_time: {elapsed_time:.2f}s")
            # 檢查是否完成 - 需要等待兩個 MPC 完全結束
            if (not self.arm1_mpc_running and not self.arm2_mpc_running and  
                self.fine_positioning_steps['arm1'] >= self.fine_mpc_steps * 0.6 
                and self.fine_positioning_steps['arm2'] >= self.fine_mpc_steps * 0.6):
            # if self.arm1_mpc_completed == True and self.arm2_mpc_completed == True:
                        # 重置 MPC 完成標記
                self.arm1_mpc_completed = False
                self.arm2_mpc_completed = False
                self.fine_positioning_started = False
                time.sleep(0.5)
                
                self.get_logger().info("開始執行夾爪操作...")
                
                # 1. 先確保夾爪完全開啟
                self.get_logger().info("1. 打開夾爪")
                # self.operate_gripper('arm1', self.gripper_open_value)
                # self.operate_gripper('arm2', self.gripper_open_value)
                time.sleep(2.0)  # 等待夾爪完全開啟
                
                # 記錄開爪時的位置
                initial_arm1_gripper_pos = self.arm1_gripper_position
                initial_arm2_gripper_pos = self.arm2_gripper_position
                
                # 2. 執行夾爪關閉
                self.get_logger().info("2. 關閉夾爪")
                # self.operate_gripper('arm1', self.gripper_close_value)
                # self.operate_gripper('arm2', self.gripper_close_value)
                
                # 3. 檢查夾取狀態
                self.get_logger().info("3. 檢查夾取狀態")
                grasp_success = False
                max_attempts = 2
                # for i in range(6):                    
                #     self.operate_gripper('arm1', self.gripper_close_value)
                #     self.operate_gripper('arm2', self.gripper_close_value)
                    # time.sleep(1.0)  # 等待夾爪動作完成

                for attempt in range(max_attempts):

                    self.get_logger().info(f"檢查夾取狀態 {attempt + 1}/{max_attempts}")
                    time.sleep(1.0)
                    
                    # 讀取當前夾爪位置
                    current_arm1_pos = self.arm1_gripper_position
                    current_arm2_pos = self.arm2_gripper_position
                    
                    # 計算夾爪位置變化
                    if current_arm1_pos is not None and current_arm2_pos is not None:
                        arm1_diff = abs(initial_arm1_gripper_pos - current_arm1_pos)
                        arm2_diff = abs(initial_arm2_gripper_pos - current_arm2_pos)
                        
                        self.get_logger().info(f"ARM1 夾爪變化: {abs(current_arm1_pos)}, ARM2 夾爪變化: {abs(current_arm2_pos)}")
                        
                        # 檢查是否有成功夾取（夾爪位置在合理範圍內）
                        arm1_grasped = 0.02 < abs(current_arm1_pos) < 0.038  # 調整這些閾值
                        arm2_grasped = 0.02 < abs(current_arm2_pos) < 0.038
                        
                        if arm1_grasped :#and arm2_grasped:
                            self.get_logger().info("✅ 檢測到成功夾取")
                            grasp_success = True
                            # break
                    
                    # 如果檢測失敗，重新發送夾取命令
                  
                # 4. 根據夾取結果決定下一步
                if grasp_success:
                    self.get_logger().info("✅ 夾取確認成功，準備回到初始位置")
                    time.sleep(1.0)  # 確保穩定夾持
                    self.publish_joint_commands_with_specific_joint('arm1', 5, 2.09, 0.0)
                    time.sleep(1.0)  # 等待夾爪回到安全位置
                    self.publish_joint_commands_with_specific_joint('arm1', 4, 1.09, 0.0)
                    time.sleep(1.5)
                    self.publish_joint_commands_with_specific_joint('arm1', 4, 0.7, 0.0)
                    time.sleep(1.5)
                    self.publish_joint_commands_with_specific_joint('arm1', 5, 2.09, -0.05)
                    time.sleep(1.0)  # 等待夾爪回到安全位置
                    self.publish_joint_commands_with_specific_joint('arm1', 5, 1.57)
                    time.sleep(3.0)  # 等待夾爪回到安全位置
                    self.move_arms_to_home()
                    self.reset_to_tracking_mode()
                    # self.current_mode = OperationMode.TRANSPORT
                    # self.current_stage = HarvestStage.TRANSPORT_RELEASE
                else:
                    self.get_logger().error("❌ 夾取可能失敗，重試")
                    # 重新開啟夾爪
                    self.operate_gripper('arm1', self.gripper_open_value)
                    self.operate_gripper('arm2', self.gripper_open_value)
                    time.sleep(1.0)
                    self.move_arms_to_home()
                    self.reset_to_tracking_mode()
                
            elif elapsed_time > self.fine_positioning_timeout:
                self.get_logger().warning(f"⏰ 精細定位執行超時，返回跟隨模式 arm1_mpc_complated{self.arm1_mpc_completed}")
                self.move_arms_to_home()
                time.sleep(3.0)
                self.reset_to_tracking_mode()


    def move_arms_to_home(self):

        """移動手臂回到 home 點"""
        try:
            # 建立 home 點位置
            home_positions_arm1 = [0.2, 0.40, -0.8, 0.0, 0.5, 1.57]
            home_positions_arm2 = [-0.2, 0.40, -0.8, 0.0, 0.5, 0.0]

            # 發送命令並等待
            self.publish_joint_commands(home_positions_arm1, 'arm1')
            self.publish_joint_commands(home_positions_arm2, 'arm2')
            
            # 等待手臂到達位置
            time.sleep(2.0)
            
            self.get_logger().info("✅ 雙臂已回到 home 點")
            
        except Exception as e:
            self.get_logger().error(f"移動到 home 點失敗: {e}")
            
    def _arm1_fine_positioning_thread(self, target_pose):
        """ARM1精細定位線程"""
        try:
            success = self.control_fine_positioning_mpc(
                self.left_mpc, self.arm1_current_positions, target_pose, 'arm1', self.gripper_open_value)
            self.operate_gripper('arm1', self.gripper_close_value)
            self.arm1_mpc_completed = True
            self.get_logger().info(f"ARM1精細定位完成: {'✅' if success else '❌'} arm1_mpc_completed: {self.arm1_mpc_completed}")

        except Exception as e:
            self.get_logger().error(f"ARM1精細定位線程出錯: {e}")
        # try:
        #     # 改為使用 Motion Generator
        #     success = self.control_fine_positioning_motion_gen(
        #         self.left_motion_gen, self.arm1_current_positions, target_pose, 'arm1', self.gripper_open_value)
            
        #     if success:
        #         self.operate_gripper('arm1', self.gripper_close_value)
            
        #     self.arm1_mpc_completed = True
        #     self.get_logger().info(f"ARM1 Motion Generator 精細定位完成: {'✅' if success else '❌'}")

        # except Exception as e:
        #     self.get_logger().error(f"ARM1 Motion Generator 精細定位線程出錯: {e}")

    def _arm2_fine_positioning_thread(self, target_pose):
        """ARM2精細定位線程"""
        try:
            
            success = self.control_fine_positioning_mpc(
                self.right_mpc, self.arm2_current_positions, target_pose, 'arm2', self.gripper_open_value)
            # self.operate_gripper('arm2', self.gripper_close_value)#操作夾爪
            self.arm2_mpc_completed = True
            self.get_logger().info(f"ARM2精細定位完成: {'✅' if success else '❌'} arm2_mpc_completed: {self.arm2_mpc_completed}")

        except Exception as e:
            self.get_logger().error(f"ARM2精細定位線程出錯: {e}")
        
        """ARM2精細定位線程（使用 Motion Generator）"""

        # try:
        #     # 改為使用 Motion Generator
        #     success = self.control_fine_positioning_motion_gen(
        #         self.right_motion_gen, self.arm2_current_positions, target_pose, 'arm2', self.gripper_open_value)
            
        #     self.arm2_mpc_completed = True
        #     self.get_logger().info(f"ARM2 Motion Generator 精細定位完成: {'✅' if success else '❌'}")

        # except Exception as e:
        #     self.get_logger().error(f"ARM2 Motion Generator 精細定位線程出錯: {e}")

    def handle_harvesting_mode(self):
        """處理採摘模式"""
        self.get_logger().info("🤝 開始協作採摘")
        
        # 同時操作雙臂夾爪
        arm1_success = self.operate_gripper('arm1', self.gripper_close_value)  # 左臂抓取
        arm2_success = self.operate_gripper('arm2', self.gripper_close_value)  # 右臂支撐
        
        if arm1_success and arm2_success:
            self.get_logger().info("✅ 雙臂夾爪操作成功")
            time.sleep(2.0)  # 等待夾緊
            
            # 左臂進入運輸模式，右臂返回跟隨模式
            self.current_mode = OperationMode.TRANSPORT
            self.current_stage = HarvestStage.TRANSPORT_RELEASE
        else:
            self.get_logger().error("❌ 夾爪操作失敗，返回跟隨模式")
            self.reset_to_tracking_mode()

    def handle_transport_mode(self):
        """處理運輸模式（左臂專用）"""
        self.get_logger().info("📦 左臂開始運輸")
        
        # 創建運輸目標
        transport_pose = Pose.from_list([
            *self.drop_position, 0.0, 0.0, 0.0, 1.0
        ])
        
        # 使用精細定位MPC移動到放置位置
        success = self.control_fine_positioning_mpc(
            self.left_mpc, self.arm1_current_positions, transport_pose, 'arm1', self.gripper_close_value)
        
        if success:
            time.sleep(1.0)
            
            # 打開夾爪釋放
            release_success = self.operate_gripper('arm1', self.gripper_open_value)
            if release_success:
                self.get_logger().info("✅ 運輸釋放完成")
                time.sleep(2.0)
            else:
                self.get_logger().error("❌ 釋放失敗")
        else:
            self.get_logger().error("❌ 運輸失敗")
        
        # 返回跟隨模式
        self.reset_to_tracking_mode()

    def reset_to_tracking_mode(self):

        """重置到跟隨模式"""
        self.get_logger().info("🔄 重置到跟隨模式")
        
        try:
            # 1. 先停止所有運動控制
            self.tracking_enabled = False
            self.arm1_mpc_running = False
            self.arm2_mpc_running = False
            
            # 2. 清除所有目標和狀態
            self.target_position = None
            self.target_orientation = None
            self.left_fine_target_pose = None
            self.right_fine_target_pose = None
            self.arm1_goal_buffer = None  # 重要：清除 goal buffer
            self.arm2_goal_buffer = None  # 重要：清除 goal buffer
            
            # 3. 發送回到初始位置的命令
            initial_positions = {
                'arm1': [0.2, 0.40, -0.8, 0.0, 0.5, 1.57, -0.04, 0.04],
                'arm2': [-0.2, 0.40, -0.8, 0.0, 0.5, 0.0, -0.04, 0.04]
            }
            
            # 多次發送確保執行
            for _ in range(3):
                self.publish_joint_commands(initial_positions['arm1'], 'arm1')
                self.publish_joint_commands(initial_positions['arm2'], 'arm2')
                time.sleep(0.5)
            
            # 4. 等待足夠時間讓手臂移動
            time.sleep(2.0)
            
            # 5. 強制更新 combined_joint_state
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = [f'arm1_joint{i}' for i in range(1, 7)] + [f'arm2_joint{i}' for i in range(1, 7)]
            js.position = initial_positions['arm1'][:6] + initial_positions['arm2'][:6]
            js.velocity = [0.0] * 12
            js.effort = [0.0] * 12
            self.combined_joint_state = js
            
            # 6. 重置所有標誌和計數器
            self.fine_positioning_triggered = False
            self.harvest_triggered = False
            self.both_arms_at_target = False
            self.fine_targets_detected = False
            self.harvest_completed = False
            self.fine_positioning_steps = {'arm1': 0, 'arm2': 0}
            if hasattr(self, 'fine_positioning_started'):
                self.fine_positioning_started = False
            
            # 7. 更新模式和階段
            self.current_mode = OperationMode.IDLE
            self.current_stage = HarvestStage.GLOBAL_TRACKING
            
            # 8. 重新啟用追蹤
            time.sleep(0.5)  # 給系統一些時間完全更新狀態
            self.tracking_enabled = True
            
            self.get_logger().info("✅ 系統已完全重置並準備好重新追蹤")
            
        except Exception as e:
            self.get_logger().error(f"重置過程出錯: {e}")
            # 確保基本狀態被重置
            self.tracking_enabled = True
            self.current_mode = OperationMode.TRACKING

    # ==================== 新增：增強狀態監控 ====================
    def enhanced_status_monitor(self):
        """增強狀態監控定時器"""
        mode_name = self.current_mode.name
        stage_name = self.current_stage.name
        
        status_msg = f"🎯 模式: {mode_name} | 階段: {stage_name}"
        
        if self.current_mode == OperationMode.TRACKING:
            status_msg += f" | 跟蹤: {'啟用' if self.tracking_enabled else '停用'}"
            status_msg += f" | ARM1運行: {'是' if self.arm1_mpc_running else '否'}"
            status_msg += f" | ARM2運行: {'是' if self.arm2_mpc_running else '否'}"
            
            if self.target_position is not None:
                status_msg += f" | 目標距離: ARM1={self.arm1_distance_to_target:.3f}m, ARM2={self.arm2_distance_to_target:.3f}m"
                
        elif self.current_mode == OperationMode.FINE_POSITIONING:
            if hasattr(self, 'fine_positioning_started') and self.fine_positioning_started:
                elapsed = time.time() - self.fine_positioning_start_time
                status_msg += f" | 精細定位進度: ARM1={self.fine_positioning_steps['arm1']}/{self.fine_mpc_steps}, ARM2={self.fine_positioning_steps['arm2']}/{self.fine_mpc_steps}"
                status_msg += f" | 已用時間: {elapsed:.1f}s/{self.fine_positioning_timeout}s"
            
            fine_targets = f"CAM1={'✓' if self.left_fine_target_pose else '✗'}, CAM2={'✓' if self.right_fine_target_pose else '✗'}"
            status_msg += f" | 精細目標: {fine_targets}"
        
        elif self.current_mode == OperationMode.HARVESTING:
            status_msg += f" | 夾爪狀態: ARM1={'關閉' if self.arm1_gripper_closed else '打開'}, ARM2={'關閉' if self.arm2_gripper_closed else '打開'}"
        
        elif self.current_mode == OperationMode.TRANSPORT:
            status_msg += f" | 運輸中..."
        
        self.get_logger().info(status_msg)

    # ==================== 原有清理函數 (保持不變) ====================
    def destroy_node(self):
        """節點銷毀時的清理工作"""
        self.get_logger().info("正在清理資源...")
        
        # 停止所有控制線程
        self.control_thread_running = False
        self.enhanced_control_thread_running = False
        self.arm1_mpc_running = False
        self.arm2_mpc_running = False
        
        if hasattr(self, 'control_thread') and self.control_thread.is_alive():
            self.control_thread.join(timeout=2.0)
        
        if hasattr(self, 'enhanced_control_thread') and self.enhanced_control_thread.is_alive():
            self.enhanced_control_thread.join(timeout=2.0)
        
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
        node = EnhancedDualArmTracker()
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
