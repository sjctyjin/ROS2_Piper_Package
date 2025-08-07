#!/usr/bin/env python3
"""
更新說明 ： 2025-07-18
新增函數 home_set() 同一制定Home點
修正流程：
1. 統一由cam1先發現object後,先讓右臂前往扶住枝頭
2. 左臂靠近cam1的點位後,啟動cam2近距離重新定位 
3. 左臂根據cam2 object位置 進行精準採摘

"""
#此支程式
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
        
        # 初始化张量设备类型
        self.tensor_args = TensorDeviceType()
        
        # 声明参数

        self.declare_parameter('enable_rosbridge', False)
        self.declare_parameter('rosbridge_host', '192.168.3.125')
        self.declare_parameter('rosbridge_port', 9090)
        self.declare_parameter('rosbridge_topic', '/joint_custom_state')
        self.declare_parameter('frame_id', 'piper_single')
        #self.declare_parameter('place_position_x', 0.10)
        #self.declare_parameter('place_position_y', -0.245)
        #self.declare_parameter('place_position_z', 0.260)
        
        self.declare_parameter('place_position_x', -0.0174)
        self.declare_parameter('place_position_y', 0.321)
        self.declare_parameter('place_position_z', 0.250)
        self.declare_parameter('gripper_close_value', 0.0)
        self.declare_parameter('gripper_open_value', -0.05)
        self.declare_parameter('arm_prefix', 'arm1')  # 新增: 關節前綴參數
        self.declare_parameter('cam_prefix', 'cam1')  # 新增: 相機前綴參數
        
        self.declare_parameter('is_left_arm', False)  # 預設為右手臂
        # 新增：取得左右手臂參數,左臂採摘 右臂輔助
        self.is_left_arm = self.get_parameter('is_left_arm').get_parameter_value().bool_value
        
        if self.is_left_arm:
            #self.declare_parameter('robot_config', 'dual_piper_left.yml') # 原始雙臂
            self.declare_parameter('robot_config', 'trip_piper_left.yml') # 三手臂
        else:
            #self.declare_parameter('robot_config', 'dual_piper_right.yml')
            self.declare_parameter('robot_config', 'trip_piper_right.yml')
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
        self.arm_prefix = self.get_parameter('arm_prefix').get_parameter_value().string_value  # 取得前綴參數
        self.cam_prefix = self.get_parameter('cam_prefix').get_parameter_value().string_value  # 取得前綴參數
        self.tf_locked_until_idle = False #TF座標鎖定機制
        
        
        
        # 根據左右手臂設定不同的四元數
        if self.is_left_arm:
            self.pick_quaternion = [0.881, 0.010, 0.472, 0.007]  # 左臂pick採摘四元數
            #self.home_quaternion = [0.597, -0.453, 0.588, 0.303]   # 左臂Home位置四元數
            self.home_quaternion = [ 0.796,0.060, 0.603, 0.011]
            #self.home_quaternion = [0.074,0.320, -0.153, 0.932] #反手
            self.get_logger().info("設定為左手臂，使用左臂四元數")
        else:
            self.pick_quaternion = [0.560, 0.000, 0.829, -0.000]  # 右臂pick協作四元數 (原始值)
            self.home_quaternion =[ 0.652 ,0.221, 0.695, 0.207]      # 右臂Home位置四元數 (原始值)
            self.get_logger().info("設定為右手臂，使用右臂四元數")
            
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
        #if self.is_piper:
        #    self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "joint8"]
        #else:
        #    self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
            
        self.joint_names = [f"{self.arm_prefix}_joint{i+1}" for i in range(6)] + [f'{self.arm_prefix}_joint7', f'{self.arm_prefix}_joint8']


        # 创建发布者
        self.publisher = self.create_publisher(JointState, f'{self.arm_prefix}/joint_custom_state', 10)
        
        # 订阅当前关节状态
        self.joint_state_sub = self.create_subscription(
            JointState,
            f'/arm1/joint_states',
            self.cb1,
            10
        )
        self.joint_state_sub = self.create_subscription(
            JointState,
            f'/arm2/joint_states',
            self.cb2,
            10
        )
        
        # 定时合并（50Hz）
        self.create_timer(1/50.0, self.timer_cb)
        
        # 订阅馬達實際關節狀態
        
        self.real_subscription = self.create_subscription(
            JointState,
            f'{self.arm_prefix}/joint_states_single',
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
        self.get_logger().info("正在预热运动规划器...")
        self.motion_gen.warmup(enable_graph=True)
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
        self.STATE_REFINE_GRASP = 7 #左臂二次定位
        self.current_state = self.STATE_IDLE
        self.previous_state = 0
        #協作手臂 ： 判斷前手是否已到位,後手是否已離開
        self.arm1_gripper_pos = 0 #
        self.arm2_gripper_pos = 0 
        # 保存抓取位置
        self.pick_position = None
        self.pick_orientation = None
        self.pick_check = 0
        self.pre_pick_check = -1
        #保存初次的joint值
        self.latest_joint_state = None
        self.latest_joint_state_cb1 = None
        self.latest_joint_state_cb2 = None
        #判斷是否成功末端值
        self.gripper_data_event = False    
        self.gripper_check_timer = 0 
        #存放關節最後值
        self.current_joint_positions_globel = []
        #二次確認時間
        self.refine_check_time = 0
        self.right_state_check = 0 #右臂狀態確認,若左臂超時 則退回HOME點
        
        
        # 创建定时器，设置为10Hz
        self.timer = self.create_timer(0.1, self.pick_and_place_loop)

        self.timer2 = self.create_timer(0.1, self.T2)
        
    def cb1(self, msg):
        """接收当前关节状态"""
        
        self.latest_joint_state_cb1 = msg
        
        if "arm2_joint7" in msg.name:
            idx = msg.name.index("arm2_joint7")
            arm1_joint7_pos = msg.position[idx]
            if arm1_joint7_pos == 0.0:
                self.arm1_gripper_pos = 1 # 表示右手已抓住枝頭
            else:
                self.arm1_gripper_pos = 0
            #print(f"arm2_joint7 的位置是: {arm2_joint7_pos}")
        #self.get_logger().info(f"監聽JointState CB1: {self.latest_joint_state_cb1}\n訂閱內容 :{self.arm_prefix}/joint_states")
        
    def cb2(self, msg):
        """接收当前关节状态"""
        self.latest_joint_state_cb2 = msg
        
        if "arm1_joint7" in msg.name:
            idx = msg.name.index("arm1_joint7")
            arm2_joint7_pos = msg.position[idx]
            if arm2_joint7_pos == 0.0:
                self.arm2_gripper_pos = 1 # 表示左手已抓住物體
            else:
                self.arm2_gripper_pos = 0
            print(f"arm1_joint7 的位置是: {arm1_joint7_pos}")
        #self.get_logger().info(f"監聽JointState CB2: {self.latest_joint_state}\n訂閱內容 :{self.arm_prefix}/joint_states")
    	
    def timer_cb(self):
        if self.latest_joint_state_cb1 is None or self.latest_joint_state_cb2 is None:
            return

        # 先取完arm1的1-6，再取arm2的1-6（連續順序）
        arm1_positions = [self.latest_joint_state_cb1.position[
                         self.latest_joint_state_cb1.name.index(f'arm1_joint{i}')
                     ] for i in range(1, 7)]
    
        arm2_positions = [self.latest_joint_state_cb2.position[
                         self.latest_joint_state_cb2.name.index(f'arm2_joint{i}')
                     ] for i in range(1, 7)]

        # 連續順序：先arm1_1-6，再arm2_1-6
        if self.is_left_arm:
            combined_names = [f'arm2_joint{i}' for i in range(1, 7)] + [f'arm1_joint{i}' for i in range(1, 7)]
            combined_pos = arm2_positions + arm1_positions
        else:
            combined_names = [f'arm1_joint{i}' for i in range(1, 7)] + [f'arm2_joint{i}' for i in range(1, 7)]
            combined_pos = arm1_positions + arm2_positions
        

        # 3) 用 JointState 消息来存储合并结果
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()    # 或者用任意一个原始消息的 header
        js.name     = combined_names
        js.position = combined_pos
        # 如果后续逻辑会读 velocity/effort，也可以填0
        js.velocity = [0.0] * len(combined_pos)
        js.effort   = [0.0] * len(combined_pos)

        # 4) 最终赋值给 self.latest_joint_state
        self.latest_joint_state = js

        #self.get_logger().info(f'Combined JointState: names={js.name}, pos={js.position}')
        
    def publish_joint_state_with_gripper(self, joint_positions, gripper_value):
        """发布带有夹爪值的关节状态"""
        
        # 创建消息
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        
        # 确保关节位置是列表
        if isinstance(joint_positions, np.ndarray):
            joint_positions = joint_positions.tolist()
            
        positions_with_gripper = list(joint_positions) 
        
        #第一隻手夾爪
        joint_positions.insert(7,0.0)
        joint_positions.insert(8,0.0)
        
        joint_positions.append(0.0)#加入joint7
        joint_positions.append(0.0)#加入joint8
        positions_with_gripper.append(0.0)
        positions_with_gripper.append(0.0)
        # 为joint7(夹爪)设置值
        send_to_msg = list(joint_positions)       
        send_to_msg[6] = gripper_value  # joint7是夹爪
        msg.position = send_to_msg
        self.get_logger().info(f"positions_with_gripper 直－－－－－ ：{positions_with_gripper}")
        msg.velocity = [10.0] * len(self.joint_names)
        # 发布到ROS2
        self.publisher.publish(msg)
        self.get_logger().info(f"最後位置 latest_joint_state－－－－－ ：{self.latest_joint_state.position}")
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
        # 建立 name->idx 快速查詢
        """
        name_to_idx = {n: i for i, n in enumerate(self.latest_joint_state.name)}
        # 依照self.joint_names的順序抽出
        current_joint_positions = []
        #self.get_logger().info(f"前 - 監聽 JointState : {name_to_idx}")
        for n in self.joint_names:
            if n in name_to_idx:
                try:
                    current_joint_positions.append(self.latest_joint_state.position[name_to_idx[n]])
                except:
                    current_joint_positions.append(0.0)    
            else:
                self.get_logger().warning(f"找不到關節 {n}，補0")
                current_joint_positions.append(0.0)
        self.current_joint_positions_globel = current_joint_positions
        """
        self.get_logger().info(f"後 - 監聽 JointState : {self.latest_joint_state}")
        current_joint_positions = []            
        if self.is_left_arm: 
            kine_joint = self.motion_gen.kinematics.joint_names[6:]+self.motion_gen.kinematics.joint_names[:6]
        else:
            kine_joint = self.motion_gen.kinematics.joint_names[:6]+self.motion_gen.kinematics.joint_names[6:]         
        #kine_joint = self.motion_gen.kinematics.joint_names[6:]+self.motion_gen.kinematics.joint_names[:6]
        for name in kine_joint:
            #names = f"{self.arm_prefix}_{name}"
            if name in self.latest_joint_state.name:
                    #self.get_logger().warning(f"关节 {name}，使用默认值0.0")
                idx = self.latest_joint_state.name.index(name)
                current_joint_positions.append(self.latest_joint_state.position[idx])                    
            else:
                self.get_logger().warning(f"找不到关节 {name}，使用默认值0.0")
                current_joint_positions.append(0.0)
        if current_joint_positions != []:
            self.current_joint_positions_globel = current_joint_positions
        self.get_logger().info(f"後 - 監聽 current_joint_positions_globel : {self.current_joint_positions_globel}")
        
        # 等待一个时间步
        time.sleep(self.dt)
    
    def execute_trajectory(self, trajectory, gripper_value=None,mpc_mode=None):
        """执行轨迹，可选指定夹爪值"""
        self.get_logger().info(f"當前current_joint_global狀態 : {self.current_joint_positions_globel}")
        self.get_logger().info(f"當前trajectory : {trajectory}")
        
        if len(trajectory) == 0:
            self.get_logger().warning("轨迹为空，无法执行")
            return False

        if mpc_mode == None:    
            self.get_logger().info(f"开始执行 {len(trajectory)} 个轨迹点")
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
                self.publish_joint_state_with_gripper(trajectory[0], trajectory[i][6] if len(trajectory[i]) > 6 else self.gripper_open_value)       
        
        return True
        
    
    def plan_and_execute(self, current_joints, target_pose, gripper_value=None):
        """规划并执行轨迹"""
        self.get_logger().info(f"规划到目标位置: {target_pose}")

        
        # 创建CuRobo关节状态
        cu_js = CuroboJointState(
            position=self.tensor_args.to_device(np.array(self.current_joint_positions_globel)),
            velocity=self.tensor_args.to_device(np.zeros_like(self.current_joint_positions_globel)),
            acceleration=self.tensor_args.to_device(np.zeros_like(self.current_joint_positions_globel)),
            jerk=self.tensor_args.to_device(np.zeros_like(self.current_joint_positions_globel)),
            joint_names=self.motion_gen.kinematics.joint_names
        )
        self.get_logger().info(f"规划cu_js: {cu_js}")
        # 执行运动规划
        result = self.motion_gen.plan_single(
            cu_js.unsqueeze(0), 
            target_pose, 
            MotionGenPlanConfig(max_attempts=40, enable_graph=True)
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
        """
        self.get_logger().info(f"MPC配置信息:")
        self.get_logger().info(f"  - rollout_fn.batch_size: {getattr(self.mpc.rollout_fn, 'batch_size', 'N/A')}")
        self.get_logger().info(f"  - solver batch_size: {getattr(self.mpc, 'batch_size', 'N/A')}")
        self.get_logger().info(f"  - 期望關節數: {len(self.mpc.rollout_fn.joint_names)}")
        """
        expected_names = self.mpc.rollout_fn.joint_names
        isaac_standard = [f'arm1_joint{i}' for i in range(1, 7)] + [f'arm2_joint{i}' for i in range(1, 7)]

        if expected_names == isaac_standard:
            self.get_logger().info("✅ 與Isaac Sim順序一致")
        else:
            self.get_logger().warning(f"⚠️ 順序不一致! MPC期望: {expected_names}")
        self.get_logger().info(f"[MPC] 规划到目标位置: {target_pose}")
        # 1. 构造当前状态
        cu_js = CuroboJointState(
            position=self.tensor_args.to_device(np.array(current_joints)),
            velocity=self.tensor_args.to_device(np.zeros_like(current_joints)),
            acceleration=self.tensor_args.to_device(np.zeros_like(current_joints)),
            jerk=self.tensor_args.to_device(np.zeros_like(current_joints)),
            joint_names=self.mpc.rollout_fn.joint_names
        )
        self.get_logger().info(f"更新前 ：{cu_js}")
        # 2. 创建 Goal
        goal = Goal(current_state=cu_js,goal_state=cu_js, goal_pose=target_pose)
        # 3. Setup & update goal
        goal_buf = self.mpc.setup_solve_single(goal,1)
        self.mpc.update_goal(goal_buf)
        # 4. 循环 step, 累积轨迹
        
        max_iters = 150
        check_traj = True
        k = 0
        for _ in range(max_iters):
            traj = []
            cu_js = CuroboJointState(
                position=self.tensor_args.to_device(np.array(self.current_joint_positions_globel)),
                velocity=self.tensor_args.to_device(np.zeros_like(self.current_joint_positions_globel)),
                acceleration=self.tensor_args.to_device(np.zeros_like(self.current_joint_positions_globel)),
                jerk=self.tensor_args.to_device(np.zeros_like(self.current_joint_positions_globel)),
                joint_names=self.mpc.rollout_fn.joint_names
            )

            self.get_logger().info(f"更新後 ：{cu_js}")
            res = self.mpc.step(cu_js, max_attempts=2)  
            cu_js = res.js_action.clone()         
            if not res.metrics.feasible.item():
                self.get_logger().warning("[MPC] 轨迹不可行，提前退出")
                check_traj = False
                break
            js_next = res.js_action
            self.get_logger().info(f"輸出參數 ： {js_next.position}")
            traj.append(js_next.position.cpu().numpy().tolist())
            if traj !=[]:                
                self.execute_trajectory(traj, gripper_value,"mpc")#指定mpc,避免一直輸出訊息
                k+=1
            # 可加收敛判定：位置误差或方向误差足够小时 break
        # 5. 連續性执行轨迹
        self.get_logger().info(f"轨迹执行完成-執行了{k}次規劃")
        return check_traj


    
    def operate_gripper(self, current_joints, gripper_value):
        """操作夹爪 - 仅改变夹爪值，保持其他关节不变"""
        self.get_logger().info(f"操作夹爪，设置值: {gripper_value}")
        
        # 创建一个1点轨迹，包含当前关节值
        trajectory = [current_joints]
        
        # 执行，指定夹爪值
        return self.execute_trajectory(trajectory, gripper_value)
    
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
            # 建立 name->idx 快速查詢
            """
            name_to_idx = {n: i for i, n in enumerate(self.latest_joint_state.name)}
            # 依照self.joint_names的順序抽出
            current_joint_positions = []
            #self.get_logger().info(f"當前Joint直 --------： {name_to_idx}\n self.joint_names值 :{self.joint_names}")
            for n in self.joint_names:
                if n in name_to_idx:
                    #self.get_logger().info(f"當前 N 直:{n} ---\n 當前laset_joint : {self.latest_joint_state.position}") 
                    try:
                        current_joint_positions.append(self.latest_joint_state.position[name_to_idx[n]])
                    except:
                        current_joint_positions.append(0.0)
                else:
                    self.get_logger().warning(f"找不到關節 {n}，補0")
                    current_joint_positions.append(0.0)
            self.current_joint_positions_globel = current_joint_positions
            """
            
            
            # 状态机处理
            if self.current_state == self.STATE_IDLE and not self.tf_locked_until_idle:
                #提取当前关节位置
                
                try:
                    tf = self.tf_buffer.lookup_transform(
                        f'base_link', f'cam3_object_in_base', rclpy.time.Time(), timeout=Duration(seconds=1.0))
                    # ✅ 插入這段檢查 TF 是否新鮮
                    now = self.get_clock().now()
                    tf_time = tf.header.stamp
                    tf_age = now - rclpy.time.Time.from_msg(tf_time)

                    if tf_age > Duration(seconds=0.5):
                        self.get_logger().warn(f"⚠️ TF已過期 ({tf_age.nanoseconds/1e9:.2f}s)，忽略此次抓取")
                        return
                        
                    # 提取位置和方向
                    position = tf.transform.translation
                    orientation = tf.transform.rotation
                    
                    # 保存抓取位置和方向
                    self.pick_position = np.array([position.x, position.y, position.z])
                    self.pick_orientation = np.array([orientation.w, orientation.x, orientation.y, orientation.z])
                    
                    # 如果初次运行或目标位置变化显著，转入MOVE_TO_PICK状态
                    if self.last_position is None or np.linalg.norm(self.pick_position - self.last_position) > 1e-3:
                        self.get_logger().info(f'开始新的Pick and Place，目标位置: {self.pick_position}')
                        self.last_position = self.pick_position.copy()
                        self.current_state = self.STATE_MOVE_TO_PICK
                        self.get_logger().info(f'狀態 ： {self.current_state}')
                        self.executing = False
                        self.tf_locked_until_idle = True
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    # 无法找到目标，继续等待
                    self.get_logger().info(f"无法找到目标，继续等待")
                    self.get_logger().error(f"等待出错: {traceback.print_exc()}")
                    pass
                    
            elif self.current_state == self.STATE_MOVE_TO_PICK:
            
                # 创建目标姿态
                
                self.get_logger().info("開始規劃")
                
                current_joint_positions = []
                
                self.get_logger().info(f"收关节状态前... : {self.current_joint_positions_globel}")
                
                if self.is_left_arm: 
                    kine_joint = self.motion_gen.kinematics.joint_names[6:]+self.motion_gen.kinematics.joint_names[:6]
                else:
                    kine_joint = self.motion_gen.kinematics.joint_names[:6]+self.motion_gen.kinematics.joint_names[6:]         
                               
                for name in kine_joint:
                    #names = f"{self.arm_prefix}_{name}"
                    if name in self.latest_joint_state.name:
                        #self.get_logger().warning(f"关节 {name}，使用默认值0.0")
                        idx = self.latest_joint_state.name.index(name)
                        current_joint_positions.append(self.latest_joint_state.position[idx])                    
                    else:
                        self.get_logger().warning(f"找不到关节 {name}，使用默认值0.0")
                        current_joint_positions.append(0.0)
                    
                self.current_joint_positions_globel = current_joint_positions
                self.get_logger().info(f"收关节状态後... : {self.current_joint_positions_globel}")
                
                # 查找目标TF变换
                self.get_logger().info(f'{self.arm_prefix}_base_link   {self.cam_prefix}_object_in_base')
                
                if self.is_left_arm == True:#左臂等待右臂完成抓定再移動
                    target_pose = Pose.from_list([
                        self.pick_position[0], self.pick_position[1], self.pick_position[2],
                        self.pick_orientation[0], self.pick_orientation[1], self.pick_orientation[2], self.pick_orientation[3]
                        #self.home_quaternion[0], self.home_quaternion[1], self.home_quaternion[2], self.home_quaternion[3]
                    ])
                    time.sleep(2) 
                    
                else:#右臂直接移動
                    target_pose = Pose.from_list([
                        self.pick_position[0], self.pick_position[1], self.pick_position[2],
                        self.home_quaternion[0], self.home_quaternion[1], self.home_quaternion[2], self.home_quaternion[3]
                        #self.pick_orientation[0], self.pick_orientation[1], self.pick_orientation[2], self.pick_orientation[3]
                    ])
                # 规划并移动到抓取位置，夹爪保持打开
                if self.is_left_arm == False and self.arm2_gripper_pos == 0: #右臂狀態:確保左臂在工作原點
                
                    if self.plan_and_execute_mpc(current_joint_positions, target_pose, self.gripper_open_value):
                        self.current_state = self.STATE_GRASP
                        self.get_logger().info("規劃到夾取位置成功--- 等待2秒")
                        time.sleep(1.0)
                    else:
                    # 规划失败，回到IDLE状态
                        self.current_state = self.STATE_IDLE
                        self.get_logger().error(f"規劃失敗")
                elif self.is_left_arm == True and self.arm1_gripper_pos == 1:# 左臂
                    if self.plan_and_execute_mpc(current_joint_positions, target_pose, self.gripper_open_value):                   
                        #self.current_state = self.STATE_GRASP#原本是直接夾取
                        self.current_state = self.STATE_REFINE_GRASP
                        self.get_logger().info("規劃二次定位成功--- 等待2秒")
                        #time.sleep(1.0)
                    else:
                    # 规划失败，回到IDLE状态
                        self.current_state = self.STATE_IDLE
                        self.get_logger().error(f"規劃失敗")

            elif self.current_state == self.STATE_REFINE_GRASP:#二次定位用
                self.refine_check_time += 1
                if self.refine_check_time <= 15:
                    try:
                        tf = self.tf_buffer.lookup_transform(
                            'base_link', 'cam2_object_in_base', rclpy.time.Time(), timeout=Duration(seconds=1.0))
                        
                        # 時間戳檢查（可選）
                        now = self.get_clock().now()
                        tf_time = tf.header.stamp
                        tf_age = now - rclpy.time.Time.from_msg(tf_time)
                        if tf_age > Duration(seconds=0.5):
                            self.get_logger().warn(f"⚠️ cam2 TF 過期，跳過 refinement")
                            #self.current_state = self.STATE_GRASP
                            return

                        self.pick_position = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])
                        self.pick_orientation = np.array([tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z])
                        self.get_logger().info(f"🧠 二次定位成功，新的 grasp 位置：{self.pick_position}")

                        target_pose = Pose.from_list([
                            self.pick_position[0], self.pick_position[1], self.pick_position[2],
                            self.pick_orientation[0], self.pick_orientation[1], self.pick_orientation[2], self.pick_orientation[3]
                        ])

                        if self.plan_and_execute_mpc(self.current_joint_positions_globel, target_pose, self.gripper_open_value):
                            self.get_logger().info("✅ refinement 後 grasp 位置已移動完成")
                            self.current_state = self.STATE_GRASP
                            time.sleep(1.0)
                        else:
                            self.get_logger().error("❌ refinement 後移動失敗，回到IDLE")
                            self.current_state = self.STATE_IDLE
                    except Exception as e:
                        self.get_logger().error(f"二次 refinement TF 查詢失敗: {e}")
                else:
                    self.refine_check_time = 0
                    self.current_state = self.STATE_GRASP
    
            elif self.current_state == self.STATE_GRASP:
                # 关闭夹爪
                if self.operate_gripper(self.current_joint_positions_globel, self.gripper_close_value):                
                    self.get_logger().info("夹取成功，等待一段时间...")   
                    time.sleep(1)  # 等待一秒确保夹紧             
                    self.gripper_data_event = True   
                    self.current_state = self.STATE_WATTING_GRIPPER                           
                
                else:
                    # 关闭夹爪失败，回到IDLE状态
                    self.current_state = self.STATE_IDLE
            elif self.current_state == self.STATE_WATTING_GRIPPER:
                self.get_logger().info("等待夾爪狀態回覆")   
            
            elif self.current_state == self.STATE_MOVE_TO_PLACE:
                
                # 创建放置姿态
                #place_pose = Pose.from_list([
                #    self.place_position_x, self.place_position_y, self.place_position_z,
                #     0.459, 0.523, 0.568, -0.440
                #])
                place_pose = Pose.from_list([
                    self.place_position_x, self.place_position_y, self.place_position_z,
                    0.143, -0.631, 0.753, 0.116
                ])
                if self.previous_state == self.STATE_MOVE_TO_HOME:
                    self.current_state = self.STATE_MOVE_TO_HOME
                else:   
                    self.get_logger().info(f"放置-位置{place_pose}")
                    # 规划并移动到放置位置，夹爪保持关闭
                    if self.plan_and_execute(self.current_joint_positions_globel, place_pose, self.gripper_close_value):
                        self.current_state = self.STATE_RELEASE
                        self.get_logger().info(f"等待3秒到放置點")
                        time.sleep(3.0)
                    else:
                        # 规划失败，回到IDLE状态
                        self.current_state = self.STATE_IDLE
                
            elif self.current_state == self.STATE_RELEASE:
                # 打开夹爪
                if self.operate_gripper(self.current_joint_positions_globel, self.gripper_open_value):
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
                
                home_pose = Pose.from_list([
                    0.231, -0.225, 0.501,  # 预设的HOME位置
                    0.685, 0.0, 0.729, -0.0  # 默认方向
                ])
                
                if self.previous_state == self.STATE_MOVE_TO_HOME:
                    #if self.plan_and_execute(self.current_joint_positions_globel, home_pose, self.gripper_close_value):
                    #    self.current_state = self.STATE_MOVE_TO_PLACE
                    #    self.previous_state = self.STATE_MOVE_TO_PLACE
                    #else:
                        # 规划失败，直接回到IDLE状态
                    #    self.current_state = self.STATE_IDLE      
                    """    
                    msg = JointState()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.name = self.joint_names
                    joint_positions = self.latest_joint_state.position
                    # 确保关节位置是列表
                    if isinstance(joint_positions, np.ndarray):
                        joint_positions = joint_positions.tolist()
                            
                        

                    send_to_msg = joint_positions
                     
                    #第一隻手夾爪
                    joint_positions.insert(7,0.0)
                    joint_positions.insert(8,0.0)
                       
                    joint_positions.append(0.0)#加入joint7
                    joint_positions.append(0.0)#加入joint8


                    # 为joint7(夹爪)设置值
                    send_to_msg = list(joint_positions)       
                    send_to_msg[6] = -0.09  # joint7是夹爪
                    send_to_msg[:6] = [0.0, 0.75, -1.1, 0.0, 0.5, 0.0]
                    msg.position = send_to_msg

                    msg.velocity = [10.0] * len(joint_positions)
                        
                    # 发布到ROS2
                    self.publisher.publish(msg)  
                    """
                    self.home_set()
                else:
                    # 规划并移动到HOME位置，夹爪保持打开                                        
                    self.right_state_check += 1

                    if self.is_left_arm == True: #and self.arm2_gripper_pos == 1:
                        self.get_logger().info(f"左臂回歸 Home 點 --------------------------------------------------------------\n\n\n\n\n\n")
                        self.home_set()
                          
                    elif self.is_left_arm == False and self.arm2_gripper_pos == 1:
                        time.sleep(5.0)
                        self.home_set()
                    else:
                        if self.right_state_check >= 10:
                            self.home_set()
                            self.right_state_check = 0
                        
                    if self.is_left_arm:
                        self.arm2_gripper_pos = 0
                    else:
                        self.arm1_gripper_pos = 0
                    """
                    if self.plan_and_execute_mpc(self.current_joint_positions_globel, home_pose, self.gripper_open_value):
                        self.get_logger().info("Pick and Place完成--等待4秒!")
                        time.sleep(4.0)            
                        self.current_state = self.STATE_IDLE                    
                    else:
                        # 规划失败，直接回到IDLE状态
                        self.current_state = self.STATE_IDLE
                    """
            
        except Exception as e:
            self.get_logger().error(f"Pick and Place出错: {e}")
            self.get_logger().error(f"Pick and Place出错: {traceback.print_exc()}")
            self.current_state = self.STATE_IDLE
            self.executing = True
        finally:
            if self.current_state == self.STATE_IDLE:
                self.executing = False
                
    def home_set(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        joint_positions = self.latest_joint_state.position

        # 确保关节位置是列表
        if isinstance(joint_positions, np.ndarray):
            joint_positions = joint_positions.tolist()              
        send_to_msg = joint_positions     

        #第一隻手夾爪
        joint_positions.insert(7,0.0)
        joint_positions.insert(8,0.0)

        joint_positions.append(0.0)#加入joint7
        joint_positions.append(0.0)#加入joint8
        
        # 为joint7(夹爪)设置值
        
        send_to_msg = list(joint_positions)       
        
        send_to_msg[6] = -0.09  # joint7是夹爪
        
        if self.is_left_arm:

            send_to_msg[:6] = [0.2, 0.4, -0.8, 0.0, 0.5, 0.0]
        else:

            send_to_msg[:6] = [-0.2, 0.4, -0.8, 0.0, 0.5, 0.0]
        
        msg.position = send_to_msg

        msg.velocity = [10.0] * len(joint_positions)
        # 发布到ROS2
        self.publisher.publish(msg)
        time.sleep(3.0)            
        self.current_state = self.STATE_IDLE  
        self.tf_locked_until_idle = False     

    def joint_callback(self, msg: JointState):
        # 取得 gripper 的位置

        try:
            gripper_index = msg.name.index(f'{self.arm_prefix}_gripper')  # 找到 gripper 在 name 中的索引
            gripper_position = msg.position[gripper_index]  # 取得對應位置
            #self.get_logger().info(f'Gripper position: {gripper_position:.4f}')

            # 根據 gripper 開口程度判斷是否抓取成功（依據你的實際值調整閾值）
            if abs(gripper_position) > 0.02:
                #self.get_logger().info("✅ 夾取成功（Gripper 關閉）")
                self.pick_check = 1
            else:
                #self.get_logger().info("❌ 可能未成功夾取（Gripper 打開）")
                self.pick_check = 0

        except ValueError:
            self.get_logger().warn("找不到 'gripper' 關節名稱")
    def T2(self):
        #self.get_logger().info(f"Timer執行中...\n當前pre_pick_check:{self.pre_pick_check}\n當前pick_check:{self.pick_check}\n當前current_state:{self.current_state}\n當前previous_state:{self.previous_state}")
        # 轉發給 /web_tf topic
        if self.gripper_data_event:
            self.gripper_check_timer += 1#避免無限迴圈
            if self.current_state == self.STATE_WATTING_GRIPPER:#判斷當前主線程是否在等待
                    if self.is_left_arm == False:
                        self.current_state = self.STATE_MOVE_TO_HOME
                        self.pre_pick_check = self.pick_check#最後才shift 
                        time.sleep(4)                              
                        self.gripper_data_event = False                
                        self.gripper_check_timer = 0
                    else:
                        if self.gripper_check_timer >= 20:    
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
