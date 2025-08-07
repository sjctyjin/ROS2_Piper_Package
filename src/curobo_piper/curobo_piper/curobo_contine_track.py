#!/usr/bin/env python3
"""
雙臂即時TF跟蹤節點 (基於用戶MPC方法)
功能：
1. 使用單一MPC配置控制14軸(雙臂)即時跟蹤cam3_object_in_base TF座標
2. 將14軸MPC結果分別映射到arm1和arm2的關節狀態
3. 當TF座標停止發佈或過期時，手臂停在當前位置
4. 持續監聽並更新目標位置
"""

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

# CuRobo 導入
from curobo.types.math import Pose
from curobo.types.robot import JointState as CuroboJointState
from curobo.wrap.reacher.mpc import MpcSolver, MpcSolverConfig
from curobo.rollout.rollout_base import Goal
from curobo.util_file import get_world_configs_path, join_path, load_yaml
from curobo.geom.types import WorldConfig
from curobo.types.base import TensorDeviceType
from curobo.geom.sdf.world import CollisionCheckerType

# ROSBridge導入 (如果需要)
try:
    from rosbridge_websocket import init_rosbridge, publish_joint_state, close_rosbridge
except ImportError:
    print("警告: 未找到rosbridge_websocket模块，無法發送關節值到ROS1")

class DualArmMPCTracker(Node):
    def __init__(self):
        super().__init__('dual_arm_mpc_tracker')
        
        # 初始化張量設備類型
        self.tensor_args = TensorDeviceType()
        
        # 聲明參數
        self.declare_parameter('enable_rosbridge', False)
        self.declare_parameter('rosbridge_host', '192.168.3.125')
        self.declare_parameter('rosbridge_port', 9090)
        self.declare_parameter('frame_id', 'piper_single')
        self.declare_parameter('target_tf_frame', 'cam3_object_in_base')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tf_timeout', 0.5)  # TF過期時間(秒)
        self.declare_parameter('control_frequency', 20.0)  # 控制頻率(Hz)
        self.declare_parameter('gripper_value', -0.05)  # 夾爪值
        self.declare_parameter('position_threshold', 0.003)  # 位置變化閾值(m)
        self.declare_parameter('dual_config', 'trip_piper_right.yml')  # 雙臂配置文件
        self.declare_parameter('mpc_max_iters', 50)  # MPC最大迭代次數
        
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
        self.dual_config = self.get_parameter('dual_config').get_parameter_value().string_value
        self.mpc_max_iters = self.get_parameter('mpc_max_iters').get_parameter_value().integer_value
        
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
        self.latest_joint_state = None
        self.current_joint_positions_global = []
        
        # 跟蹤狀態
        self.target_position = None
        self.target_orientation = None
        self.last_valid_tf_time = None
        self.tracking_enabled = False
        self.mpc_running = False
        
        # MPC Goal Buffer (重用避免重複創建)
        self.goal_buffer = None
        
        # 初始化CuRobo MPC配置
        self.get_logger().info(f"正在初始化雙臂MPC配置: {self.dual_config}")
        self.setup_mpc_solver()
        
        # 定時器合併關節狀態
        self.create_timer(1/50.0, self.combine_joint_states)
        
        # 創建控制線程
        self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.control_thread_running = True
        self.control_thread.start()
        
        # 創建定時器用於狀態監控
        self.create_timer(1.0, self.status_monitor)
        
        self.dt = 0.01  # 時間步長
        
        self.get_logger().info("雙臂MPC跟蹤節點已啟動")

    def setup_mpc_solver(self):
        """設置單一MPC來控制雙臂"""
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
            self.get_logger().info(f"正在加載雙臂MPC配置: {self.dual_config}")
            mpc_config = MpcSolverConfig.load_from_robot_config(
                self.dual_config,
                world_config,
                use_cuda_graph=False,  # 避免形狀問題
                use_cuda_graph_metrics=False,
                self_collision_check=True,
                collision_checker_type=None,  # 可根據需要啟用
                collision_cache={"obb": 10, "mesh": 5},
                use_mppi=True,
                use_lbfgs=False,
                use_es=False,
                store_rollouts=False,
                step_dt=0.02,
            )
            self.mpc = MpcSolver(mpc_config)
            
            # 輸出配置信息
            joint_names = self.mpc.rollout_fn.joint_names
            self.get_logger().info(f"MPC配置關節名稱: {joint_names}")
            self.get_logger().info(f"總關節數: {len(joint_names)}")
            self.get_logger().info(f"MPC batch_size: {getattr(self.mpc.rollout_fn, 'batch_size', 'N/A')}")
            
            # 檢查關節順序並給出建議
            isaac_standard = [f'arm1_joint{i}' for i in range(1, 7)] + [f'arm2_joint{i}' for i in range(1, 7)]
            is_arm1_first = joint_names[0].startswith('arm1')
            
            if joint_names == isaac_standard:
                self.get_logger().info("✅ 使用標準關節順序 (arm1->arm2)")
            elif is_arm1_first:
                self.get_logger().info("✅ arm1在前，arm2在後的順序")
            else:
                self.get_logger().info("⚠️ arm2在前，arm1在後的順序")
                
            self.get_logger().info(f"關節映射: 前6軸->{'arm1' if is_arm1_first else 'arm2'}, 後6軸->{'arm2' if is_arm1_first else 'arm1'}")
            
            self.get_logger().info("CuRobo雙臂MPC控制器已就緒")
            
        except Exception as e:
            self.get_logger().error(f"初始化MPC控制器失敗: {e}")
            self.get_logger().error(traceback.format_exc())
            raise

    def arm1_joint_callback(self, msg):
        """ARM1關節狀態回調"""
        self.arm1_joint_state = msg

    def arm2_joint_callback(self, msg):
        """ARM2關節狀態回調"""
        self.arm2_joint_state = msg

    def combine_joint_states(self):
        """合併兩隻手臂的關節狀態"""
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

            # 最終賦值給 self.latest_joint_state
            self.latest_joint_state = js
            
            # 更新全局關節位置（按MPC期望順序）
            self.update_global_joint_positions()
            
        except Exception as e:
            self.get_logger().warning(f"合併關節狀態失敗: {e}")

    def update_global_joint_positions(self):
        """更新全局關節位置，按照MPC期望的順序"""
        if self.latest_joint_state is None:
            return
            
        current_joint_positions = []
        mpc_joint_names = self.mpc.rollout_fn.joint_names
        
        # 檢查MPC期望的關節順序
        is_arm1_first = mpc_joint_names[0].startswith('arm1')
        
        if is_arm1_first:
            # arm1_joint1-6, arm2_joint1-6 順序 (trip_piper_left.yml)
            expected_order = [f'arm1_joint{i}' for i in range(1, 7)] + [f'arm2_joint{i}' for i in range(1, 7)]
        else:
            # arm2_joint1-6, arm1_joint1-6 順序 (trip_piper_right.yml)  
            expected_order = [f'arm2_joint{i}' for i in range(1, 7)] + [f'arm1_joint{i}' for i in range(1, 7)]
        
        # 按照MPC期望的順序提取關節位置
        for name in expected_order:
            if name in self.latest_joint_state.name:
                idx = self.latest_joint_state.name.index(name)
                current_joint_positions.append(self.latest_joint_state.position[idx])
            else:
                self.get_logger().warning(f"找不到關節 {name}，使用默認值0.0")
                current_joint_positions.append(0.0)
        
        if current_joint_positions:
            self.current_joint_positions_global = current_joint_positions
            
        # 調試信息（可選）
        if len(current_joint_positions) == 12:
            self.get_logger().debug(f"關節順序: {expected_order[:2]}...{expected_order[-2:]}")
            self.get_logger().debug(f"關節位置: [{current_joint_positions[0]:.3f}, {current_joint_positions[1]:.3f}]...[{current_joint_positions[-2]:.3f}, {current_joint_positions[-1]:.3f}]")

    def publish_joint_state_with_gripper(self, joint_positions, arm_name, gripper_value):
        """發佈帶有夾爪值的關節狀態到指定手臂"""
        
        # 選擇對應的發布者和關節名稱
        if arm_name == 'arm1':
            publisher = self.arm1_publisher
            joint_names = self.arm1_joint_names
            rosbridge_topic = 'arm1/joint_custom_state'
        else:
            publisher = self.arm2_publisher  
            joint_names = self.arm2_joint_names
            rosbridge_topic = 'arm2/joint_custom_state'
        
        # 創建消息
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = joint_names
        
        # 確保關節位置是列表
        if isinstance(joint_positions, np.ndarray):
            joint_positions = joint_positions.tolist()
            
        positions_with_gripper = list(joint_positions[:6])  # 只取前6個關節
        
        # 添加joint7和joint8
        positions_with_gripper.append(gripper_value)  # joint7是夾爪
        positions_with_gripper.append(0.0)  # joint8
        
        msg.position = positions_with_gripper
        msg.velocity = [10.0] * len(joint_names)
        
        # 發佈到ROS2
        publisher.publish(msg)
        
        # 如果啟用了ROSBridge，也發佈到ROS1
        if self.enable_rosbridge:
            publish_joint_state(
                rosbridge_topic,
                positions_with_gripper,
                joint_names,
                self.frame_id
            )

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

    def plan_and_execute_mpc(self, target_pose):
        """用 MPC 规划并執行雙臂跟蹤"""
        if not self.current_joint_positions_global:
            self.get_logger().warning("當前關節位置為空，無法規劃")
            return False
            
        try:
            self.mpc_running = True
            
            # 1. 構造初始狀態
            cu_js = CuroboJointState(
                position=self.tensor_args.to_device(np.array(self.current_joint_positions_global)),
                velocity=self.tensor_args.to_device(np.zeros_like(self.current_joint_positions_global)),
                acceleration=self.tensor_args.to_device(np.zeros_like(self.current_joint_positions_global)),
                jerk=self.tensor_args.to_device(np.zeros_like(self.current_joint_positions_global)),
                joint_names=self.mpc.rollout_fn.joint_names
            )
            
            # 2. 創建 Goal (重用goal_buffer避免重複創建)
            if self.goal_buffer is None:
                goal = Goal(current_state=cu_js, goal_state=cu_js, goal_pose=target_pose)
                self.goal_buffer = self.mpc.setup_solve_single(goal, 1)
                self.get_logger().info("Goal buffer 已創建")
            
            # 3. 更新目標姿態
            self.goal_buffer.goal_pose.copy_(target_pose)
            self.mpc.update_goal(self.goal_buffer)
            
            # 4. MPC step循環（關鍵：每次都重新創建cu_js）
            max_iters = self.mpc_max_iters
            check_traj = True
            step_count = 0
            
            for i in range(max_iters):
                # ✅ 關鍵：每次循環都重新創建cu_js，使用最新的關節位置
                cu_js = CuroboJointState(
                    position=self.tensor_args.to_device(np.array(self.current_joint_positions_global)),
                    velocity=self.tensor_args.to_device(np.zeros_like(self.current_joint_positions_global)),
                    acceleration=self.tensor_args.to_device(np.zeros_like(self.current_joint_positions_global)),
                    jerk=self.tensor_args.to_device(np.zeros_like(self.current_joint_positions_global)),
                    joint_names=self.mpc.rollout_fn.joint_names
                )
                
                # 執行MPC step
                res = self.mpc.step(cu_js, max_attempts=1)
                
                if not res.metrics.feasible.item():
                    self.get_logger().warning("[MPC] 軌跡不可行，提前退出")
                    check_traj = False
                    break
                
                # 獲取下一步關節位置
                js_next = res.js_action
                next_positions = js_next.position.cpu().numpy()
                
                # 根據MPC配置的關節順序分別發佈到兩隻手臂
                mpc_joint_names = self.mpc.rollout_fn.joint_names
                is_arm1_first = mpc_joint_names[0].startswith('arm1')
                
                if is_arm1_first:
                    # trip_piper_left.yml: arm1在前，arm2在後
                    arm1_positions = next_positions[:6]
                    arm2_positions = next_positions[6:12]
                else:
                    # trip_piper_right.yml: arm2在前，arm1在後
                    arm2_positions = next_positions[:6]  
                    arm1_positions = next_positions[6:12]
                
                # 發佈到兩隻手臂
                self.publish_joint_state_with_gripper(arm1_positions, 'arm1', self.gripper_value)
                self.publish_joint_state_with_gripper(arm2_positions, 'arm2', self.gripper_value)
                
                # 更新當前位置用於下一次迭代
                self.current_joint_positions_global = next_positions.tolist()
                
                step_count += 1
                
                # 可選：添加收斂判定
                # TODO: 檢查是否到達目標位置，如果到達則提前退出
                
                # 控制頻率，避免過快
                time.sleep(0.02)  # 50Hz
            
            self.get_logger().info(f"MPC跟蹤完成，執行了{step_count}次step")
            return check_traj
                
        except Exception as e:
            self.get_logger().error(f"MPC規劃執行出錯: {e}")
            self.get_logger().error(traceback.format_exc())
            return False
        finally:
            self.mpc_running = False

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
                        if self.mpc_running:
                            time.sleep(1.0 / self.control_freq)
                            continue
                            
                        self.target_position = position
                        self.target_orientation = orientation
                        self.tracking_enabled = True
                        
                        # 創建目標姿態
                        target_pose = Pose.from_list([
                            position[0], position[1], position[2],
                            orientation[0], orientation[1], orientation[2], orientation[3]
                        ])
                        
                        # 使用MPC規劃並執行雙臂運動
                        success = self.plan_and_execute_mpc(target_pose)
                        
                        if not success:
                            self.get_logger().warning("MPC雙臂跟蹤失敗")
                else:
                    # TF丟失或過期，停止跟蹤
                    if self.tracking_enabled:
                        self.get_logger().warning(f"TF跟蹤丟失: {error_msg}")
                        self.tracking_enabled = False
                
                # 控制頻率
                time.sleep(1.0 / self.control_freq)
                
            except Exception as e:
                self.get_logger().error(f"控制迴圈出錯: {e}")
                time.sleep(0.1)

    def status_monitor(self):
        """狀態監控定時器"""
        status_msg = f"跟蹤狀態: {'啟用' if self.tracking_enabled else '停用'} | "
        status_msg += f"MPC: {'運行中' if self.mpc_running else '待機'} | "
        status_msg += f"關節數: {len(self.current_joint_positions_global)}"
        
        if self.target_position is not None:
            status_msg += f" | 目標位置: [{self.target_position[0]:.3f}, {self.target_position[1]:.3f}, {self.target_position[2]:.3f}]"
        
        self.get_logger().info(status_msg)

    def destroy_node(self):
        """節點銷毀時的清理工作"""
        self.get_logger().info("正在清理資源...")
        
        # 停止控制線程
        self.control_thread_running = False
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
        node = DualArmMPCTracker()
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
