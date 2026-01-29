#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np
import time
import subprocess
import os
import tempfile
import time
from collections import deque
import traceback
# CuRobo imports
from curobo.types.math import Pose
from curobo.types.robot import JointState as CuroboJointState
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
from curobo.wrap.reacher.mpc import MpcSolver, MpcSolverConfig
from curobo.rollout.rollout_base import Goal
from curobo.types.base import TensorDeviceType
from scipy.spatial.transform import Rotation as Rot
from std_msgs.msg import Bool

# VR Reader import
from oculus_reader import OculusReader
from tf2_ros import Buffer, TransformListener


class VRMPCControlNode(Node):
    def __init__(self, launch_rviz=True):
        super().__init__('vr_mpc_control_node')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # ============ 配置參數 ============
        self.robot_config = 'piper.yml'
        self.vr_ip = '192.168.1.126'
        self.use_usb = True
        self.control_rate = 50.0
        self.gripper_sensitivity = 0.07
        self.gripper_open_value = -0.09
        self.gripper_close_value = 0.0
        self.drift_multiplier = 100
        self.history_length = 8
        self.launch_rviz = launch_rviz
        self.past_pose = None
        
        
        # ============ 初始化 ============
        self.tensor_args = TensorDeviceType()
        self.get_logger().info("🚀 初始化 VR MPC Control Node...")
        
        # 初始化VR Reader
        self.init_vr_reader()
        
        # 初始化TF廣播器 (修復ROS2版本兼容性)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # 關節名稱
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "joint8"]
        
        # ROS發布者和訂閱者
        self.joint_publisher = self.create_publisher(JointState, '/joint_custom_state', 10)
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
            
            
        
        #存放關節最後值
        self.current_joint_positions_globel = []    
        # 發布VR TF座標
        
        self.VR_R_TF_publisher = self.create_publisher(TransformStamped, '/vr_right_hand', 10)
        
        
        # 初始化CuRobo MPC
        self.init_mpc()
        
        # 控制狀態
        self.latest_joint_state = None
        self.current_joint_positions = []
        self.vr_control_active = False
        self.reference_position = None
        self.current_gripper_value = self.gripper_open_value
        
        # 按鍵狀態
        self.last_a_state = False
        self.last_b_state = False
        
        # 漂移檢測
        self.position_history = deque(maxlen=self.history_length)
        self.distance_history = deque(maxlen=self.history_length-1)
        self.goal_buffer = None
        # 統計
        self.mpc_steps = 0
        self.drift_blocks = 0
        self.last_stats_time = time.time()
        
        # 定時器
        control_period = 1.0 / self.control_rate
        self.control_timer = self.create_timer(control_period, self.control_loop)
        self.stats_timer = self.create_timer(5.0, self.print_stats)
        
        # 啟動RViz
        #if self.launch_rviz:
        #    self.launch_rviz2()
        
        self.get_logger().info("✅ VR MPC Control Node 初始化完成")
        self.get_logger().info("🎮 控制說明:")
        self.get_logger().info("   A鍵: 設置參考位置")
        self.get_logger().info("   B鍵: 開始/停止VR控制")  
        self.get_logger().info("   右扳機: 夾爪控制")
        if self.launch_rviz:
            self.get_logger().info("📺 RViz2 已自動啟動，可查看 vr_right_hand TF座標")
    
    def create_rviz_config(self):
        """創建RViz配置文件"""
        rviz_config = """
Panels:
  - Class: rviz_common/Displays
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /TF1
        - /TF1/Frames1
      Splitter Ratio: 0.5
    Tree Height: 557
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
  - Class: rviz_common/Time
    Name: Time
    SyncMode: 0
    SyncSource: ""

Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 0.1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.03
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 20
      Reference Frame: <Fixed Frame>
      Value: true
      
    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: true
        base_link:
          Value: true
        vr_right_hand:
          Value: true
      Marker Scale: 0.2
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: true
      Tree:
        base_link:
          vr_right_hand:
            {}
      Update Interval: 0
      Value: true

  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Default Light: true
    Fixed Frame: base_link
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 1.5
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.06
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0.2
        Y: 0
        Z: 0.3
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.01
      Pitch: 0.5
      Target Frame: <Fixed Frame>
      Yaw: 1.0
    Saved: ~

Window Geometry:
  Displays:
    collapsed: false
  Height: 800
  Hide Left Dock: false
  Hide Right Dock: false
  Width: 1200
  X: 100
  Y: 100
"""
        return rviz_config
    
    def launch_rviz2(self):
        """啟動RViz2"""
        try:
            # 創建臨時配置文件
            rviz_config = self.create_rviz_config()
            self.rviz_config_file = tempfile.NamedTemporaryFile(mode='w', suffix='.rviz', delete=False)
            self.rviz_config_file.write(rviz_config)
            self.rviz_config_file.close()
            
            # 啟動RViz2
            self.rviz_process = subprocess.Popen([
                'rviz2', '-d', self.rviz_config_file.name
            ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            
            self.get_logger().info(f"📺 RViz2 已啟動 (PID: {self.rviz_process.pid})")
            
        except Exception as e:
            self.get_logger().warn(f"⚠️ 無法啟動RViz2: {e}")
            self.get_logger().info("💡 你可以手動運行: rviz2")
    
    def cleanup_rviz(self):
        """清理RViz資源"""
        try:
            if hasattr(self, 'rviz_process') and self.rviz_process.poll() is None:
                self.rviz_process.terminate()
                self.get_logger().info("📺 RViz2 已關閉")
            
            if hasattr(self, 'rviz_config_file'):
                os.unlink(self.rviz_config_file.name)
                
        except Exception as e:
            self.get_logger().warn(f"清理RViz資源時出錯: {e}")
            
    def init_vr_reader(self):
        """初始化VR Reader"""
        try:
            if self.use_usb:
                self.vr_reader = OculusReader()
                self.get_logger().info("📱 VR Reader 初始化完成 (USB)")
            else:
                self.vr_reader = OculusReader(ip_address=self.vr_ip)
                self.get_logger().info(f"📶 VR Reader 初始化完成 (WiFi: {self.vr_ip})")
        except Exception as e:
            self.get_logger().error(f"❌ VR Reader 初始化失敗: {e}")
            raise
    
    def init_mpc(self):
        """初始化MPC求解器"""
        try:
            self.get_logger().info(f"🔧 載入機器人配置: {self.robot_config}")
            
            # 簡單世界配置
            world_config = {
                "cuboid": {
                    "dummy": {
                        "dims": [0.0001, 0.0001, 0.0001],
                        "pose": [10.0, 10.0, 10.0, 1, 0, 0, 0.0],
                    },
                },
            }
            motion_gen_config = MotionGenConfig.load_from_robot_config(
                self.robot_config, 
                world_config, 
                interpolation_dt=0.01
            )
            
            
            self.motion_gen = MotionGen(motion_gen_config)
            # MPC配置
            mpc_cfg = MpcSolverConfig.load_from_robot_config(
                self.robot_config,
                world_config,
                use_cuda_graph=True,
                use_cuda_graph_metrics=True,
                self_collision_check=True,
                collision_checker_type=None,
                collision_cache={"obb": 30, "mesh": 10},
                use_mppi=True,
                use_lbfgs=False,
                use_es=False,
                store_rollouts=True,
                step_dt=0.02,
            )
            
            self.mpc = MpcSolver(mpc_cfg)
            self.get_logger().info("🧠 MPC求解器初始化完成")
            
        except Exception as e:
            self.get_logger().error(f"❌ MPC初始化失敗: {e}")
            raise
    
    def joint_state_callback(self, msg):
        """關節狀態回調"""
        self.latest_joint_state = msg
        
        # 提取關節位置
        self.current_joint_positions = []
        for name in self.mpc.rollout_fn.joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_joint_positions.append(msg.position[idx])
            else:
                self.current_joint_positions.append(0.0)
    
    def adjustment_matrix(self, transform):
        """VR坐標系調整"""
        adj_mat = np.array([
            [0, 0, -1, 0],
            [-1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 0, 1]
        ])
        
        r_adj = np.array([
            [0, 0, 1, 0],
            [0, -1, 0, 0],
            [1, 0, 0, 0],
            [0, 0, 0, 1]
        ])
        
        return adj_mat @ transform @ r_adj
    
    def publish_tf(self, transform_matrix, child_frame_id):
        """發布TF"""
        try:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'base_link'
            t.child_frame_id = child_frame_id
            
            # 位置
            t.transform.translation.x = float(transform_matrix[0, 3])+0.121
            t.transform.translation.y = float(transform_matrix[1, 3])
            t.transform.translation.z = float(transform_matrix[2, 3])+0.458
            
            # 旋轉矩陣轉四元數
            R = transform_matrix[:3, :3]
            Rx_90 = np.array([
                [1, 0, 0],
                [0, 0, 1],
                [0, -1, 0]
            ])
            theta = np.deg2rad(-45)  # 將角度轉為弧度
            Ry_45 = np.array([
                [ np.cos(theta),  0, np.sin(theta)],
                [ 0,              1, 0],
                [-np.sin(theta),  0, np.cos(theta)]
            ])
            R = R @ Rx_90
            R = R @ Ry_45
            trace = np.trace(R)
            
            if trace > 0:
                s = np.sqrt(trace + 1.0) * 2
                w = 0.25 * s
                x = (R[2, 1] - R[1, 2]) / s
                y = (R[0, 2] - R[2, 0]) / s
                z = (R[1, 0] - R[0, 1]) / s
            else:
                if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                    s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
                    w = (R[2, 1] - R[1, 2]) / s
                    x = 0.25 * s
                    y = (R[0, 1] + R[1, 0]) / s
                    z = (R[0, 2] + R[2, 0]) / s
                elif R[1, 1] > R[2, 2]:
                    s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
                    w = (R[0, 2] - R[2, 0]) / s
                    x = (R[0, 1] + R[1, 0]) / s
                    y = 0.25 * s
                    z = (R[1, 2] + R[2, 1]) / s
                else:
                    s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
                    w = (R[1, 0] - R[0, 1]) / s
                    x = (R[0, 2] + R[2, 0]) / s
                    y = (R[1, 2] + R[2, 1]) / s
                    z = 0.25 * s
            
            # 正規化四元數
            norm = np.sqrt(w*w + x*x + y*y + z*z)
            if norm > 1e-6:
                w, x, y, z = w/norm, x/norm, y/norm, z/norm
            else:
                w, x, y, z = 1.0, 0.0, 0.0, 0.0
            
            t.transform.rotation.x = x
            t.transform.rotation.y = y
            t.transform.rotation.z = z
            t.transform.rotation.w = w
            
            self.tf_broadcaster.sendTransform(t)
            
        except Exception as e:
            self.get_logger().error(f"TF發布失敗: {e}")
    
    def detect_drift(self, position):
        """檢測漂移"""
        current_pos = np.array(position[:3])
        
        if len(self.position_history) == 0:
            self.position_history.append(current_pos)
            return False, "第一個位置"
        
        last_pos = self.position_history[-1]
        current_distance = np.linalg.norm(current_pos - last_pos)
        
        if len(self.distance_history) < 3:
            self.position_history.append(current_pos)
            self.distance_history.append(current_distance)
            return False, f"建立歷史 {len(self.distance_history)}/3"
        
        avg_distance = np.mean(list(self.distance_history))
        is_drift = current_distance > (avg_distance * self.drift_multiplier)
        
        if is_drift:
            self.drift_blocks += 1
            reason = f"漂移! 當前={current_distance:.3f}m 最後 ： {last_pos}> 平均*{self.drift_multiplier}={avg_distance*self.drift_multiplier:.3f}m"
            return True, reason
        else:
            self.position_history.append(current_pos)
            self.distance_history.append(current_distance)
            return False, "正常"
    
    def calc_relative_pose(self, base_pose, current_pose):
        """計算相對姿態"""
        def create_matrix(x, y, z, rx, ry, rz):
            T = np.eye(4)
            T[:3, 3] = [x, y, z]
            
            # 簡化的旋轉矩陣計算
            cr, sr = np.cos(rx), np.sin(rx)
            cp, sp = np.cos(ry), np.sin(ry)
            cy, sy = np.cos(rz), np.sin(rz)
            
            T[0, 0] = cy * cp
            T[0, 1] = cy * sp * sr - sy * cr
            T[0, 2] = cy * sp * cr + sy * sr
            T[1, 0] = sy * cp
            T[1, 1] = sy * sp * sr + cy * cr
            T[1, 2] = sy * sp * cr - cy * sr
            T[2, 0] = -sp
            T[2, 1] = cp * sr
            T[2, 2] = cp * cr
            
            return T
        
        # 基準矩陣
        base_matrix = create_matrix(*base_pose[:6])
        
        # 工作空間偏移
        offset_matrix = create_matrix(0.121, 0.000, 0.458, 0, 0, 0)
        #offset_matrix = create_matrix(0.0, 0.000, 0.0, 0, 0, 0)
        # 當前矩陣
        current_matrix = create_matrix(*current_pose[:6])
        
        # 相對變換
        result_matrix = offset_matrix @ np.linalg.inv(base_matrix) @ current_matrix
        
        # 提取位置
        return result_matrix[:3, 3]
    
    def publish_joint_state(self, joint_positions, gripper_value):
        """發布關節狀態"""
        try:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            
            positions = list(joint_positions) if isinstance(joint_positions, np.ndarray) else joint_positions
            #self.get_logger().error(f"關節狀態發布: {positions}")
            # 確保有8個關節
            while len(positions) < 8:
                positions.append(0.0)
            
            # 設置夾爪值
            positions[6] = gripper_value  # joint7
            positions[7] = 0.0           # joint8
            
            msg.position = [float(x) for x in positions]
            msg.velocity = [10.0] * len(self.joint_names)
            
            self.joint_publisher.publish(msg)
            self.latest_joint_state.position = [float(x) for x in positions]
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
  
            
        except Exception as e:
            self.get_logger().error(f"關節狀態發布失敗: {e}")
    
    def execute_mpc_step(self, target_pose, gripper_value):
        """執行MPC步驟"""
        try:
            # 當前關節狀態
            cu_js = CuroboJointState(
                position=self.tensor_args.to_device(np.array(self.current_joint_positions_globel)),
                velocity=self.tensor_args.to_device(np.zeros_like(self.current_joint_positions_globel)),
                acceleration=self.tensor_args.to_device(np.zeros_like(self.current_joint_positions_globel)),
                jerk=self.tensor_args.to_device(np.zeros_like(self.current_joint_positions_globel)),
                joint_names=self.mpc.rollout_fn.joint_names
            )
            
            # 創建目標
            goal = Goal(current_state=cu_js, goal_state=cu_js, goal_pose=target_pose)
            
            # MPC求解
            goal_buf = self.mpc.setup_solve_single(goal, 1)
            self.mpc.update_goal(goal_buf)
            res = self.mpc.step(cu_js, max_attempts=2)
            
            if res.metrics.feasible.item():
                next_joints = res.js_action.position.cpu().numpy()
                self.publish_joint_state(next_joints, gripper_value)
                self.mpc_steps += 1
                return True
            else:
                self.get_logger().warn("MPC軌跡不可行")
                return False
                
        except Exception as e:
            self.get_logger().error(f"MPC執行失敗: {e}")
            return False
    
    def control_loop(self):
        """主控制循環"""
        #self.get_logger().info(f"執行中")
        try:
            # 提取当前关节位置
            current_joint_positions = []
            
            
            
            transformations, buttons = self.vr_reader.get_transformations_and_buttons()
            
            # 檢查關節狀態
            if self.latest_joint_state is None or len(self.current_joint_positions) == 0:
                self.get_logger().info(f"失敗A")
                return
            
            # 獲取VR數據
            #transformations, buttons = self.vr_reader.get_transformations_and_buttons()
            
            #if 'r' not in transformations or transformations['r'] is None:
            if 'r' not in transformations or transformations['r'] is None:
                self.get_logger().error("❌ VR transformations['r'] 尚未取得，rotation 無效")
                return
            
            # 調整坐標系
            adjusted_transform = self.adjustment_matrix(transformations['r'])
            
            rotation_matrix = adjusted_transform[:3, :3]
            # 轉為四元數（格式為 [x, y, z, w]）
            quat = Rot.from_matrix(rotation_matrix).as_quat()


            # 發布TF
            self.publish_tf(adjusted_transform, 'vr_right_hand')
                                           
            # 按鍵檢測
            a_pressed = buttons.get('A', False)
            b_pressed = buttons.get('B', False)
            trigger_value = buttons.get('rightTrig', [0.0])[0]
            
            
            #a_pressed = buttons.get('X', False)
            #b_pressed = buttons.get('Y', False)
            #trigger_value = buttons.get('leftTrig', [0.0])[0]
            
            # 更新夾爪值
            if trigger_value > 0.5:
                self.current_gripper_value = 0.0

            else:
                self.current_gripper_value = -0.09
                
            self.get_logger().info(f"夾爪直 ： {self.current_gripper_value}")
            # A鍵：設置參考位置
            if a_pressed and not self.last_a_state:
                for name in self.motion_gen.kinematics.joint_names:
                    if name in self.latest_joint_state.name:

                        idx = self.latest_joint_state.name.index(name)
                        current_joint_positions.append(self.latest_joint_state.position[idx])                    
                    else:
                        self.get_logger().warning(f"找不到关节 {name}，使用默认值0.0")
                        current_joint_positions.append(0.0)
                    self.current_joint_positions_globel = current_joint_positions
                try:
                
                    msg = JointState()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.name = self.joint_names       
                    msg.position = [0.0, 0.75, -1.1, 0.0, 0.5, 0.0, 0.035, 0.0]
                    msg.velocity = [10.0] * len(self.joint_names)
            
                    self.joint_publisher.publish(msg)

            
                except Exception as e:
                    self.get_logger().error(f"關節狀態發布失敗: {e}")
                self.get_logger().info("🏠 參考位置已設置")
                #position = adjusted_transform[:3, 3]
                position = [float(adjusted_transform[0, 3]),float(adjusted_transform[1, 3]),float(adjusted_transform[2, 3])+0.5]#adjusted_transform[:3, 3]
                
                time.sleep(1)
                # 簡化的歐拉角提取
                R = adjusted_transform[:3, :3]
                sy = np.sqrt(R[0,0]**2 + R[1,0]**2)
                if sy > 1e-6:
                    x = np.arctan2(R[2,1], R[2,2])
                    y = np.arctan2(-R[2,0], sy)
                    z = np.arctan2(R[1,0], R[0,0])
                else:
                    x = np.arctan2(-R[1,2], R[1,1])
                    y = np.arctan2(-R[2,0], sy)
                    z = 0
                
                self.reference_position = np.concatenate([position, [x, y, z]])
                self.position_history.clear()
                self.distance_history.clear()
                self.get_logger().info(f"🏠 參考位置已設置----{self.current_joint_positions}")
                cu_js = CuroboJointState(
                    position=self.tensor_args.to_device(np.array(self.current_joint_positions_globel)),
                    velocity=self.tensor_args.to_device(np.zeros_like(self.current_joint_positions_globel)),
                    acceleration=self.tensor_args.to_device(np.zeros_like(self.current_joint_positions_globel)),
                    jerk=self.tensor_args.to_device(np.zeros_like(self.current_joint_positions_globel)),
                    joint_names=self.mpc.rollout_fn.joint_names
                )
                target_pose = Pose(
                    position=self.tensor_args.to_device([float(0.055),float(0.00),float(0.203)]),
                    quaternion=self.tensor_args.to_device([0.737,0.000, 0.676, -0.000]),
                )
                # 創建目標
                goal = Goal(current_state=cu_js, goal_state=cu_js, goal_pose=target_pose)
                
                # MPC求解
                self.goal_buffer = self.mpc.setup_solve_single(goal, 1)
                self.mpc.update_goal(self.goal_buffer)
                self.mpc.step(cu_js, max_attempts=2)

                self.past_pose = None
                res = self.mpc.step(cu_js, max_attempts=2)
                
                #if res.metrics.feasible.item():
                #    next_joints = res.js_action.position.cpu().numpy()
                #    self.publish_joint_state(next_joints, self.current_gripper_value)
                #else:
                #    self.get_logger().warn("MPC軌跡不可行")


                self.get_logger().info("🏠 參考位置已設置")
            
            # B鍵：切換控制狀態
            if b_pressed and not self.last_b_state:
                if self.reference_position is not None:
                    self.vr_control_active = not self.vr_control_active
                    status = "🟢啟動" if self.vr_control_active else "🔴停止"
                    self.get_logger().info(f"🎮 VR控制 {status}")
                else:
                    self.get_logger().warn("⚠️ 請先按A鍵設置參考位置")
            
            self.last_a_state = a_pressed
            self.last_b_state = b_pressed
            
            # VR控制執行
            if self.vr_control_active and self.reference_position is not None:
                try:
                    trans = self.tf_buffer.lookup_transform(
                    target_frame='base_link',        # ✅ 你要的目標座標
                    source_frame='vr_right_hand',    # ✅ 查詢 vr_right_hand 在 base_link 中的位置
                    time=rclpy.time.Time())          # 最新的時間

                    # 取得 translation
                    TF_x = trans.transform.translation.x
                    TF_y = trans.transform.translation.y
                    TF_z = trans.transform.translation.z
                    # 取得四元數並轉成旋轉矩陣或Euler
                    TF_qx = trans.transform.rotation.x
                    TF_qy = trans.transform.rotation.y
                    TF_qz = trans.transform.rotation.z
                    TF_qw = trans.transform.rotation.w
                
     
                    
                    self.get_logger().info(f"📤 已發送 TF 座標至 WebSocket")
            
                except:
                    self.get_logger().info(f"等待TF座標")
                    self.get_logger().error(f"出错: {traceback.print_exc()}")
                    
                #t.transform.translation.x = float(transform_matrix[0, 3])
                #t.transform.translation.y = float(transform_matrix[1, 3])
                #t.transform.translation.z = float(transform_matrix[2, 3])+0.5
                position = [float(TF_x)+0.121,float(TF_y),float(TF_z)+0.458]#adjusted_transform[:3, 3]
                
                # 漂移檢測
                is_drift, reason = self.detect_drift(position)
                
                if not is_drift:
                    self.VR_R_TF_publisher.publish(trans)
                    
                    
                    # 提取當前姿態
                    R = adjusted_transform[:3, :3]
                    sy = np.sqrt(R[0,0]**2 + R[1,0]**2)
                    if sy > 1e-6:
                        x = np.arctan2(R[2,1], R[2,2])
                        y = np.arctan2(-R[2,0], sy)
                        z = np.arctan2(R[1,0], R[0,0])
                    else:
                        x = np.arctan2(-R[1,2], R[1,1])
                        y = np.arctan2(-R[2,0], sy)
                        z = 0
                    
                    current_pose = np.concatenate([position, [x, y, z]])
                    
                    # 計算相對位置
                    relative_pos = self.calc_relative_pose(self.reference_position, current_pose)
                    if self.past_pose is None:
                        self.past_pose = relative_pos + 1.0
                    if np.linalg.norm(relative_pos - self.past_pose) > 1e-3:
                        print(f"目标移动到 {relative_pos}, 规划新轨迹...")
                        #cu_js = CuroboJointState(
                        #   position=self.tensor_args.to_device(np.array(self.current_joint_positions)),
                        #   velocity=self.tensor_args.to_device(np.zeros_like(self.current_joint_positions)),
                        #   acceleration=self.tensor_args.to_device(np.zeros_like(self.current_joint_positions)),
                        #   jerk=self.tensor_args.to_device(np.zeros_like(self.current_joint_positions)),
                        #   joint_names=self.mpc.rollout_fn.joint_names
                        #)
                        
                        ik_goal = Pose(
                            position=self.tensor_args.to_device([float(TF_x),float(TF_y),float(TF_z)]),
                            quaternion=self.tensor_args.to_device([TF_qw, TF_qx, TF_qy,TF_qz]),
                        )
                        #goal = Goal(current_state=cu_js, goal_state=cu_js, goal_pose=ik_goal)
                        self.goal_buffer.goal_pose.copy_(ik_goal)
                        self.mpc.update_goal(self.goal_buffer)
                        self.past_pose = relative_pos

                    
                    # 執行MPC
                    # 當前關節狀態
                    cu_js = CuroboJointState(
                        position=self.tensor_args.to_device(np.array(self.current_joint_positions_globel[:6])),
                        velocity=self.tensor_args.to_device(np.zeros_like(self.current_joint_positions_globel[:6])),
                        acceleration=self.tensor_args.to_device(np.zeros_like(self.current_joint_positions_globel[:6])),
                        jerk=self.tensor_args.to_device(np.zeros_like(self.current_joint_positions_globel[:6])),
                        joint_names=self.mpc.rollout_fn.joint_names
                    )
                    
                    # 創建目標

                    
                    # MPC求解
                    res = self.mpc.step(cu_js, max_attempts=2)
                    
                    if res.metrics.feasible.item():
                        next_joints = res.js_action.position.cpu().numpy()
                        self.publish_joint_state(next_joints, self.current_gripper_value )
                        #return True
                    else:
                        self.get_logger().warn("MPC軌跡不可行")
                        #return False
                    
                
                else:
                    self.get_logger().warn(f"{reason}")
        
        except Exception as e:
            self.get_logger().error(f"控制循環錯誤: {e}")
            self.get_logger().error(f"Pick and Place出错: {traceback.print_exc()}")
    
    def print_stats(self):
        """打印統計"""
        current_time = time.time()
        duration = current_time - self.last_stats_time
        
        if duration > 0:
            mpc_rate = self.mpc_steps / duration
            control_status = "🟢ON" if self.vr_control_active else "🔴OFF"
            self.get_logger().info(
                f"📊 MPC步數={self.mpc_steps}, 頻率={mpc_rate:.1f}Hz, "
                f"VR控制={control_status}, 夾爪={self.current_gripper_value:.3f}"
            )
        
        self.mpc_steps = 0
        self.drift_blocks = 0
        self.last_stats_time = current_time

def main(args=None):
    import sys
    
    rclpy.init(args=args)
    
    # 檢查命令行參數
    launch_rviz = True
    if '--no-rviz' in sys.argv:
        launch_rviz = False
        sys.argv.remove('--no-rviz')
    
    try:
        node = VRMPCControlNode(launch_rviz=launch_rviz)
        
        # 打印啟動信息
        print("=" * 60)
        print("🎯 VR MPC Control Node 已啟動!")
        print("=" * 60)
        print("🎮 操作說明:")
        print("  A 鍵 = 設置參考位置 (必須先設置)")
        print("  B 鍵 = 開始/停止 VR 控制")
        print("  右扳機 = 夾爪控制 (按下=關閉, 鬆開=打開)")
        if launch_rviz:
            print("📺 RViz2 已自動啟動，可查看 'vr_right_hand' TF座標")
        print("🔴 按 Ctrl+C 安全退出")
        print("=" * 60)
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n👋 正在安全關閉...")
    except Exception as e:
        print(f"❌ 錯誤: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'node' in locals():
            try:
                node.cleanup_rviz()  # 清理RViz資源
                node.vr_reader.stop()
            except:
                pass
            node.destroy_node()
        rclpy.shutdown()
        print("✅ 節點已關閉")

if __name__ == '__main__':
    main()
