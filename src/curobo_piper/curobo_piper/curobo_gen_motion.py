#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import JointState
import tf2_ros
import numpy as np
import torch
import time

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

class MotionGenNode(Node):
    def __init__(self):
        super().__init__('motion_gen_node')
        
        # 初始化张量设备类型
        self.tensor_args = TensorDeviceType()
        
        # 声明参数
        self.declare_parameter('robot_config', 'piper.yml')
        self.declare_parameter('enable_rosbridge', False)
        self.declare_parameter('rosbridge_host', '192.168.3.125')
        self.declare_parameter('rosbridge_port', 9090)
        self.declare_parameter('rosbridge_topic', '/joint_custom_state')
        self.declare_parameter('frame_id', 'piper_single')
        
        # 读取参数
        self.robot_config = self.get_parameter('robot_config').get_parameter_value().string_value
        self.enable_rosbridge = self.get_parameter('enable_rosbridge').get_parameter_value().bool_value
        self.rosbridge_host = self.get_parameter('rosbridge_host').get_parameter_value().string_value
        self.rosbridge_port = self.get_parameter('rosbridge_port').get_parameter_value().integer_value
        self.rosbridge_topic = self.get_parameter('rosbridge_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        
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
        self.latest_joint_state = None
        
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
        
        # 初始化状态和标志
        self.last_position = None
        self.executing = False
        self.step_counter = 0
        
        # 创建定时器，设置为10Hz
        self.timer = self.create_timer(0.1, self.motion_planning_loop)
        self.get_logger().info("运动规划节点已初始化，等待目标TF和关节状态...")
        
    def joint_state_callback(self, msg):
        """接收当前关节状态"""
        self.latest_joint_state = msg
    
    def motion_planning_loop(self):
        """主要的运动规划循环，查找TF并进行规划"""
        self.step_counter += 1
        
        if self.executing:
            return
            
        # 如果还没有收到关节状态，等待
        if self.latest_joint_state is None:
            if self.step_counter % 50 == 0:  # 减少日志量，每5秒左右记录一次
                self.get_logger().info("等待接收关节状态...")
            return
            
        try:
            # 查找目标TF变换
            tf = self.tf_buffer.lookup_transform(
                'base_link', 'object_in_base', rclpy.time.Time(), timeout=Duration(seconds=1.0))
                
            # 提取位置和方向
            position = tf.transform.translation
            orientation = tf.transform.rotation
            current_position = np.array([position.x, position.y, position.z])
            
            # 如果初次运行或目标位置变化显著，则进行规划
            if self.last_position is None or np.linalg.norm(current_position - self.last_position) > 1e-3:
                self.get_logger().info(f'目标移动到 {current_position}，开始规划新轨迹...')
                
                # 标记正在执行
                self.executing = True
                
                # 创建目标姿态
                
                target_pose = Pose.from_list([
                    position.x, position.y, position.z,
                    orientation.w, orientation.x, orientation.y, orientation.z
                ])
                #確定可執行座標姿態
                #target_pose = Pose.from_list([
                #    0.2733, 0.1380, 0.3019,
                #    0.496, 0.0, 0.868, 0
                #])
                # 从当前关节状态创建起始状态
                # 提取实际关节位置值
                joint_positions = []
                for name in self.motion_gen.kinematics.joint_names:
                    if name in self.latest_joint_state.name:
                        idx = self.latest_joint_state.name.index(name)
                        joint_positions.append(self.latest_joint_state.position[idx])
                    else:
                        self.get_logger().warning(f"找不到关节 {name}，使用默认值0.0")
                        joint_positions.append(0.0)
                
                # 创建CuRobo关节状态
                cu_js = CuroboJointState(
                    position=self.tensor_args.to_device(np.array(joint_positions)),
                    velocity=self.tensor_args.to_device(np.zeros_like(joint_positions)),
                    acceleration=self.tensor_args.to_device(np.zeros_like(joint_positions)),
                    jerk=self.tensor_args.to_device(np.zeros_like(joint_positions)),
                    joint_names=self.motion_gen.kinematics.joint_names
                )
                print("起始座標 ：",cu_js.unsqueeze(0))
                print("末端座標 ：",target_pose)
                # 执行运动规划
                
                result = self.motion_gen.plan_single(
                    cu_js.unsqueeze(0), 
                    target_pose, 
                    MotionGenPlanConfig(max_attempts=20)
                )
                
                if not result.success:
                    self.get_logger().error("轨迹规划失败")
                    self.executing = False
                    return
                
                # 获取轨迹
                trajectory = result.get_interpolated_plan().position.cpu().numpy()
                
                # 对于Piper机器人，添加joint7和joint8的值
                if self.is_piper:
                    padded_trajectory = np.hstack((trajectory, np.full((trajectory.shape[0], 2), -0.035)))
                else:
                    padded_trajectory = np.hstack((trajectory, np.full((trajectory.shape[0], 1), 0.0)))
                
                # 发布轨迹
                self.get_logger().info(f"开始执行 {len(padded_trajectory)} 个轨迹点")
                for i in range(len(padded_trajectory)):
                    # 创建ROS2消息
                    msg = JointState()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.name = self.joint_names
                    msg.position = padded_trajectory[i].tolist()
                    
                    # 发布到ROS2
                    self.publisher.publish(msg)
                    
                    # 如果启用了ROSBridge，也发布到ROS1
                    if self.enable_rosbridge:
                        publish_joint_state(
                            self.rosbridge_topic,
                            padded_trajectory[i].tolist(),
                            self.joint_names,
                            self.frame_id
                        )
                    
                    # 等待一个时间步
                    time.sleep(self.dt)
                
                self.get_logger().info("轨迹执行完成")
                self.last_position = current_position
                
        except tf2_ros.LookupException as e:
            # TF查找异常，不打印日志，因为可能很频繁
            pass
        except tf2_ros.ConnectivityException as e:
            self.get_logger().warning(f"TF连接异常: {e}")
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().warning(f"TF外推异常: {e}")
        except Exception as e:
            self.get_logger().error(f"运动规划出错: {e}")
        finally:
            self.executing = False

def main(args=None):
    rclpy.init(args=args)
    node = MotionGenNode()
    
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
