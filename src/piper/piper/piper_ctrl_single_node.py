#!/usr/bin/env python3
# -*-coding:utf8-*-
# This file controls a single robotic arm node and handles the movement of the robotic arm with a gripper.
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import time
import threading
import argparse
import math
import numpy as np
from piper_sdk import *
from piper_sdk import C_PiperInterface
from piper_msgs.msg import PiperStatusMsg, PosCmd
from piper_msgs.srv import Enable
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R  # For Euler angle to quaternion conversion
from numpy import clip


class PiperRosNode(Node):
    """ROS2 node for the robotic arm"""

    def __init__(self) -> None:
        super().__init__('piper_ctrl_single_node')
        # ROS parameters
        self.declare_parameter('can_port', 'can0')
        self.declare_parameter('auto_enable', False)
        self.declare_parameter('gripper_exist', True)
        self.declare_parameter('gripper_val_mutiple', 1)
        self.declare_parameter('joint_smoothing_enabled', True)
        self.declare_parameter('joint_smoothing_rate_hz', 200.0)
        self.declare_parameter('joint_smoothing_kp', 10.0)
        self.declare_parameter('joint_smoothing_max_vel_deg_s', [90.0, 90.0, 100.0, 130.0, 130.0, 180.0])
        self.declare_parameter('joint_smoothing_max_acc_deg_s2', [360.0, 360.0, 420.0, 540.0, 540.0, 720.0])
        self.declare_parameter('joint_smoothing_deadband_deg', [0.05, 0.05, 0.06, 0.08, 0.08, 0.10])
        self.declare_parameter('joint_smoothing_motion_speed', 50)
        self.declare_parameter('gripper_smoothing_max_step', 2500)

        self.can_port = self.get_parameter('can_port').get_parameter_value().string_value
        self.auto_enable = self.get_parameter('auto_enable').get_parameter_value().bool_value
        self.gripper_exist = self.get_parameter('gripper_exist').get_parameter_value().bool_value
        self.gripper_val_mutiple = self.get_parameter('gripper_val_mutiple').get_parameter_value().integer_value
        self.gripper_val_mutiple = max(0, min(self.gripper_val_mutiple, 10))
        self.joint_smoothing_enabled = self.get_parameter('joint_smoothing_enabled').value
        self.joint_smoothing_rate_hz = float(self.get_parameter('joint_smoothing_rate_hz').value)
        self.joint_smoothing_kp = float(self.get_parameter('joint_smoothing_kp').value)
        self.joint_smoothing_max_vel = np.deg2rad(
            self.get_float_array_parameter(
                'joint_smoothing_max_vel_deg_s',
                [90.0, 90.0, 100.0, 130.0, 130.0, 180.0],
            )
        )
        self.joint_smoothing_max_acc = np.deg2rad(
            self.get_float_array_parameter(
                'joint_smoothing_max_acc_deg_s2',
                [360.0, 360.0, 420.0, 540.0, 540.0, 720.0],
            )
        )
        self.joint_smoothing_deadband = np.deg2rad(
            self.get_float_array_parameter(
                'joint_smoothing_deadband_deg',
                [0.05, 0.05, 0.06, 0.08, 0.08, 0.10],
            )
        )
        self.joint_smoothing_motion_speed = int(self.get_parameter('joint_smoothing_motion_speed').value)
        self.joint_smoothing_motion_speed = int(clip(self.joint_smoothing_motion_speed, 1, 100))
        self.gripper_smoothing_max_step = int(self.get_parameter('gripper_smoothing_max_step').value)
        self.gripper_smoothing_max_step = max(1, self.gripper_smoothing_max_step)

        self.get_logger().info(f"can_port is {self.can_port}")
        self.get_logger().info(f"auto_enable is {self.auto_enable}")
        self.get_logger().info(f"gripper_exist is {self.gripper_exist}")
        self.get_logger().info(f"gripper_val_mutiple is {self.gripper_val_mutiple}")
        self.get_logger().info(f"joint_smoothing_enabled is {self.joint_smoothing_enabled}")
        if self.joint_smoothing_enabled:
            self.get_logger().info(
                f"joint smoothing servo: {self.joint_smoothing_rate_hz:.1f} Hz, "
                f"kp={self.joint_smoothing_kp:.2f}, speed={self.joint_smoothing_motion_speed}"
            )
        # Publishers
        self.joint_pub = self.create_publisher(JointState, 'joint_states_single', 1)
        self.joint_ctrl_pub = self.create_publisher(JointState, 'joint_ctrl', 1)
        self.arm_status_pub = self.create_publisher(PiperStatusMsg, 'arm_status', 1)
        self.end_pose_pub = self.create_publisher(Pose, 'end_pose', 1)
        # Service
        self.motor_srv = self.create_service(Enable, 'enable_srv', self.handle_enable_service)
        # Joint
        self.joint_states = JointState()
        self.joint_states.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
        self.joint_states.position = [0.0] * 7
        self.joint_states.velocity = [0.0] * 7
        self.joint_states.effort = [0.0] * 7
        # Joint ctrl
        self.joint_ctrl = JointState()
        self.joint_ctrl.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
        self.joint_ctrl.position = [0.0] * 7
        self.joint_ctrl.velocity = [0.0] * 7
        self.joint_ctrl.effort = [0.0] * 7
        # Enable flag
        self.__enable_flag = False
        self.command_lock = threading.Lock()
        self.joint_servo_lock = threading.Lock()
        self.joint_servo_target = None
        self.joint_servo_q = None
        self.joint_servo_v = np.zeros(6, dtype=np.float64)
        self.gripper_servo_target = 0
        self.gripper_servo_value = 0
        self.gripper_servo_effort = 1000
        self.joint_servo_running = True
        # Create piper class and open CAN interface
        self.piper = C_PiperInterface(can_name=self.can_port)
        self.piper.ConnectPort()

        # Start subscription thread
        self.create_subscription(PosCmd, 'pos_cmd', self.pos_callback, 1)
        self.create_subscription(JointState, 'joint_ctrl_single', self.joint_callback, 1)
        self.create_subscription(Bool, 'enable_flag', self.enable_callback, 1)

        self.publisher_thread = threading.Thread(target=self.publish_thread)
        self.publisher_thread.start()
        self.joint_servo_thread = threading.Thread(target=self.joint_servo_loop, daemon=True)
        self.joint_servo_thread.start()

    def GetEnableFlag(self):
        return self.__enable_flag

    def get_float_array_parameter(self, name, default):
        value = self.get_parameter(name).value
        if value is None:
            return np.array(default, dtype=np.float64)
        values = list(value)
        if len(values) != 6:
            self.get_logger().warn(
                f"{name} should contain 6 values, got {len(values)}. Using default."
            )
            values = default
        return np.array(values, dtype=np.float64)

    def get_current_arm_joint_positions(self):
        raw_to_rad = math.pi / 180000.0
        try:
            joint_state = self.piper.GetArmJointMsgs().joint_state
            return np.array(
                [
                    joint_state.joint_1,
                    joint_state.joint_2,
                    joint_state.joint_3,
                    joint_state.joint_4,
                    joint_state.joint_5,
                    joint_state.joint_6,
                ],
                dtype=np.float64,
            ) * raw_to_rad
        except Exception:
            return None

    def get_current_gripper_raw(self):
        try:
            return int(abs(self.piper.GetArmGripperMsgs().gripper_state.grippers_angle))
        except Exception:
            return 0

    def parse_joint_command(self, joint_data):
        target = np.zeros(6, dtype=np.float64)
        for idx, joint_name in enumerate(joint_data.name):
            if idx >= len(joint_data.position):
                continue
            if joint_name.startswith('joint'):
                try:
                    joint_idx = int(joint_name.replace('joint', '')) - 1
                except ValueError:
                    continue
                if 0 <= joint_idx < 6:
                    target[joint_idx] = float(joint_data.position[idx])

        gripper_raw = 0
        if len(joint_data.position) >= 7:
            gripper_raw = int(abs(round(joint_data.position[6] * 1000 * 1000)))
            gripper_raw = gripper_raw * self.gripper_val_mutiple

        gripper_effort = 1000
        if len(joint_data.effort) >= 7:
            effort = clip(joint_data.effort[6], 0.5, 3)
            if not math.isnan(effort):
                gripper_effort = int(round(effort * 1000))

        return target, gripper_raw, gripper_effort

    def sdk_trajectory_step(self, q, v, target, dt):
        err = target - q
        v_des = np.clip(
            self.joint_smoothing_kp * err,
            -self.joint_smoothing_max_vel,
            self.joint_smoothing_max_vel,
        )
        dv = np.clip(
            v_des - v,
            -self.joint_smoothing_max_acc * dt,
            self.joint_smoothing_max_acc * dt,
        )
        v_new = v + dv
        q_new = q + v_new * dt

        overshoot = (target - q) * (target - q_new) <= 0.0
        settled = (
            (np.abs(target - q_new) <= self.joint_smoothing_deadband)
            & (np.abs(v_new) <= np.deg2rad(1.0))
        )
        snap = overshoot | settled
        q_new = np.where(snap, target, q_new)
        v_new = np.where(snap, 0.0, v_new)
        return q_new, v_new

    def send_joint_raw_command(self, joints_rad, gripper_raw, gripper_effort, speed=None):
        factor = 180000.0 / math.pi
        q_raw = np.round(np.asarray(joints_rad, dtype=np.float64) * factor).astype(int)
        motion_speed = self.joint_smoothing_motion_speed if speed is None else int(clip(speed, 1, 100))
        with self.command_lock:
            self.piper.MotionCtrl_2(0x01, 0x01, motion_speed)
            self.piper.JointCtrl(
                int(q_raw[0]),
                int(q_raw[1]),
                int(q_raw[2]),
                int(q_raw[3]),
                int(q_raw[4]),
                int(q_raw[5]),
            )
            if self.gripper_exist:
                self.piper.GripperCtrl(abs(int(gripper_raw)), int(gripper_effort), 0x01, 0)

    def joint_servo_loop(self):
        period = 1.0 / max(1.0, self.joint_smoothing_rate_hz)
        last_tick = time.time()

        while self.joint_servo_running and rclpy.ok():
            time.sleep(period)
            self.joint_smoothing_enabled = bool(self.get_parameter('joint_smoothing_enabled').value)
            if not self.joint_smoothing_enabled or not self.GetEnableFlag():
                with self.joint_servo_lock:
                    self.joint_servo_q = None
                    self.joint_servo_v = np.zeros(6, dtype=np.float64)
                last_tick = time.time()
                continue

            with self.joint_servo_lock:
                if self.joint_servo_target is None:
                    last_tick = time.time()
                    continue

                target = self.joint_servo_target.copy()
                target_gripper = int(self.gripper_servo_target)
                target_effort = int(self.gripper_servo_effort)

                if self.joint_servo_q is None:
                    current_q = self.get_current_arm_joint_positions()
                    self.joint_servo_q = current_q if current_q is not None else target.copy()
                    self.joint_servo_v = np.zeros(6, dtype=np.float64)
                    self.gripper_servo_value = self.get_current_gripper_raw()

                now = time.time()
                dt = max(1e-3, now - last_tick)
                last_tick = now

                q_next, v_next = self.sdk_trajectory_step(
                    self.joint_servo_q,
                    self.joint_servo_v,
                    target,
                    dt,
                )
                gripper_delta = int(
                    clip(
                        target_gripper - self.gripper_servo_value,
                        -self.gripper_smoothing_max_step,
                        self.gripper_smoothing_max_step,
                    )
                )
                self.gripper_servo_value += gripper_delta
                self.joint_servo_q = q_next
                self.joint_servo_v = v_next
                command_q = q_next.copy()
                command_gripper = int(self.gripper_servo_value)

            try:
                self.send_joint_raw_command(command_q, command_gripper, target_effort)
            except Exception as exc:
                self.get_logger().warn(f"Joint smoothing command failed: {exc}")

    def publish_thread(self):
        """Publish messages from the robotic arm
        """
        rate = self.create_rate(200)  # 200 Hz
        enable_flag = False
        # Set timeout (seconds)
        timeout = 5
        # Record the time before entering the loop
        start_time = time.time()
        elapsed_time_flag = False
        while rclpy.ok():
            if(self.auto_enable):
                while not (enable_flag):
                    elapsed_time = time.time() - start_time
                    print("--------------------")
                    enable_flag = self.piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status
                    print("Enable status:", enable_flag)
                    self.piper.EnableArm(7)
                    self.piper.GripperCtrl(0, 1000, 0x01, 0)
                    if(enable_flag):
                        self.__enable_flag = True
                    print("--------------------")
                    # Check if the timeout has been exceeded
                    if elapsed_time > timeout:
                        print("Timeout....")
                        elapsed_time_flag = True
                        enable_flag = True
                        break
                    time.sleep(1)
                    pass
            if(elapsed_time_flag):
                print("Automatic enable timeout, exiting program")
                exit(0)

            self.PublishArmState()
            self.PublishArmJointAndGripper()
            self.PublishArmCtrlAndGripper()
            self.PublishArmEndPose()

            rate.sleep()

    def PublishArmState(self):
        arm_status = PiperStatusMsg()
        arm_status.ctrl_mode = self.piper.GetArmStatus().arm_status.ctrl_mode
        arm_status.arm_status = self.piper.GetArmStatus().arm_status.arm_status
        arm_status.mode_feedback = self.piper.GetArmStatus().arm_status.mode_feed
        arm_status.teach_status = self.piper.GetArmStatus().arm_status.teach_status
        arm_status.motion_status = self.piper.GetArmStatus().arm_status.motion_status
        arm_status.trajectory_num = self.piper.GetArmStatus().arm_status.trajectory_num
        arm_status.err_code = self.piper.GetArmStatus().arm_status.err_code
        arm_status.joint_1_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_1_angle_limit
        arm_status.joint_2_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_2_angle_limit
        arm_status.joint_3_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_3_angle_limit
        arm_status.joint_4_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_4_angle_limit
        arm_status.joint_5_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_5_angle_limit
        arm_status.joint_6_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_6_angle_limit
        arm_status.communication_status_joint_1 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_1
        arm_status.communication_status_joint_2 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_2
        arm_status.communication_status_joint_3 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_3
        arm_status.communication_status_joint_4 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_4
        arm_status.communication_status_joint_5 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_5
        arm_status.communication_status_joint_6 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_6
        self.arm_status_pub.publish(arm_status)

    def PublishArmJointAndGripper(self):
        # Assign timestamp
        self.joint_states.header.stamp = self.get_clock().now().to_msg()
        # Here, you can set the joint positions to any value you want
        # The raw data obtained is in degrees multiplied by 1000. To convert to radians, divide by 1000, multiply by π/180, and limit to 5 decimal places
        joint_0: float = (self.piper.GetArmJointMsgs().joint_state.joint_1 / 1000) * 0.017444
        joint_1: float = (self.piper.GetArmJointMsgs().joint_state.joint_2 / 1000) * 0.017444
        joint_2: float = (self.piper.GetArmJointMsgs().joint_state.joint_3 / 1000) * 0.017444
        joint_3: float = (self.piper.GetArmJointMsgs().joint_state.joint_4 / 1000) * 0.017444
        joint_4: float = (self.piper.GetArmJointMsgs().joint_state.joint_5 / 1000) * 0.017444
        joint_5: float = (self.piper.GetArmJointMsgs().joint_state.joint_6 / 1000) * 0.017444
        joint_6: float = self.piper.GetArmGripperMsgs().gripper_state.grippers_angle / 1000000
        vel_0: float = self.piper.GetArmHighSpdInfoMsgs().motor_1.motor_speed / 1000
        vel_1: float = self.piper.GetArmHighSpdInfoMsgs().motor_2.motor_speed / 1000
        vel_2: float = self.piper.GetArmHighSpdInfoMsgs().motor_3.motor_speed / 1000
        vel_3: float = self.piper.GetArmHighSpdInfoMsgs().motor_4.motor_speed / 1000
        vel_4: float = self.piper.GetArmHighSpdInfoMsgs().motor_5.motor_speed / 1000
        vel_5: float = self.piper.GetArmHighSpdInfoMsgs().motor_6.motor_speed / 1000
        effort_0:float = self.piper.GetArmHighSpdInfoMsgs().motor_1.effort/1000
        effort_1:float = self.piper.GetArmHighSpdInfoMsgs().motor_2.effort/1000
        effort_2:float = self.piper.GetArmHighSpdInfoMsgs().motor_3.effort/1000
        effort_3:float = self.piper.GetArmHighSpdInfoMsgs().motor_4.effort/1000
        effort_4:float = self.piper.GetArmHighSpdInfoMsgs().motor_5.effort/1000
        effort_5:float = self.piper.GetArmHighSpdInfoMsgs().motor_6.effort/1000
        effort_6:float = self.piper.GetArmGripperMsgs().gripper_state.grippers_effort/1000
        self.joint_states.position = [joint_0,joint_1, joint_2, joint_3, joint_4, joint_5,joint_6]
        self.joint_states.velocity = [vel_0, vel_1, vel_2, vel_3, vel_4, vel_5]
        self.joint_states.effort = [effort_0, effort_1, effort_2, effort_3, effort_4, effort_5, effort_6]
        # 发布所有消息
        self.joint_pub.publish(self.joint_states)

    def PublishArmCtrlAndGripper(self):
        self.joint_ctrl.header.stamp = self.get_clock().now().to_msg()
        joint_0: float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_1/1000) * 0.017444
        joint_1: float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_2/1000) * 0.017444
        joint_2: float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_3/1000) * 0.017444
        joint_3: float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_4/1000) * 0.017444
        joint_4: float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_5/1000) * 0.017444
        joint_5: float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_6/1000) * 0.017444
        joint_6: float = self.piper.GetArmGripperCtrl().gripper_ctrl.grippers_angle/1000000
        self.joint_ctrl.position = [joint_0, joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]  # Example values
        self.joint_ctrl_pub.publish(self.joint_ctrl)

    def PublishArmEndPose(self):
        # End effector pose
        endpos = Pose()
        endpos.position.x = self.piper.GetArmEndPoseMsgs().end_pose.X_axis / 1000000
        endpos.position.y = self.piper.GetArmEndPoseMsgs().end_pose.Y_axis / 1000000
        endpos.position.z = self.piper.GetArmEndPoseMsgs().end_pose.Z_axis / 1000000
        roll = self.piper.GetArmEndPoseMsgs().end_pose.RX_axis / 1000
        pitch = self.piper.GetArmEndPoseMsgs().end_pose.RY_axis / 1000
        yaw = self.piper.GetArmEndPoseMsgs().end_pose.RZ_axis / 1000
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)
        quaternion = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()
        endpos.orientation.x = quaternion[0]
        endpos.orientation.y = quaternion[1]
        endpos.orientation.z = quaternion[2]
        endpos.orientation.w = quaternion[3]
        self.end_pose_pub.publish(endpos)

    def pos_callback(self, pos_data):
        """Callback function for subscribing to the end effector pose

        Args:
            pos_data (): The position data
        """
        factor = 180 / 3.1415926
        self.get_logger().info(f"Received PosCmd:")
        self.get_logger().info(f"x: {pos_data.x}")
        self.get_logger().info(f"y: {pos_data.y}")
        self.get_logger().info(f"z: {pos_data.z}")
        self.get_logger().info(f"roll: {pos_data.roll}")
        self.get_logger().info(f"pitch: {pos_data.pitch}")
        self.get_logger().info(f"yaw: {pos_data.yaw}")
        self.get_logger().info(f"gripper: {pos_data.gripper}")
        self.get_logger().info(f"mode1: {pos_data.mode1}")
        self.get_logger().info(f"mode2: {pos_data.mode2}")
        x = round(pos_data.x*1000) * 1000
        y = round(pos_data.y*1000) * 1000
        z = round(pos_data.z*1000) * 1000
        rx = round(pos_data.roll*1000*factor)
        ry = round(pos_data.pitch*1000*factor)
        rz = round(pos_data.yaw*1000*factor)
        if(self.GetEnableFlag()):
            self.piper.MotionCtrl_1(0x00, 0x00, 0x00)
            self.piper.MotionCtrl_2(0x01, 0x02, 50)
            self.piper.EndPoseCtrl(x, y, z, rx, ry, rz)
            gripper = round(pos_data.gripper * 1000 * 1000)
            if pos_data.gripper > 80000:
                gripper = 80000
            if pos_data.gripper < 0:
                gripper = 0
            if self.gripper_exist:
                self.piper.GripperCtrl(abs(gripper), 1000, 0x01, 0)
            self.piper.MotionCtrl_2(0x01, 0x00, 50)

    def joint_callback(self, joint_data):
        """Callback function for joint angles

        Args:
            joint_data (): The joint data
        """
        self.joint_smoothing_enabled = bool(self.get_parameter('joint_smoothing_enabled').value)
        if self.joint_smoothing_enabled:
            target, gripper_raw, gripper_effort = self.parse_joint_command(joint_data)
            with self.joint_servo_lock:
                self.joint_servo_target = target
                self.gripper_servo_target = gripper_raw
                self.gripper_servo_effort = gripper_effort
            return

        factor = 57324.840764  # 1000*180/3.14
        # self.get_logger().info(f"Received Joint States:")

        # 创建一个字典来存储关节名称与位置的映射
        joint_positions = {}
        joint_6 = 0

        # 遍历joint_data.name来映射位置
        for idx, joint_name in enumerate(joint_data.name):
            # self.get_logger().info(f"{joint_name}: {joint_data.position[idx]}")
            joint_positions[joint_name] = round(joint_data.position[idx] * factor)
        
        # 获取第7个关节的位置
        if len(joint_data.position) >= 7:
            # self.get_logger().info(f"joint_7: {joint_data.position[6]}")
            joint_6 = round(joint_data.position[6] * 1000 * 1000)
            joint_6 = joint_6 * self.gripper_val_mutiple

        # 控制电机速度
        if self.GetEnableFlag():
            if joint_data.velocity != []:
                all_zeros = all(v == 0 for v in joint_data.velocity)
            else:
                all_zeros = True
            if not all_zeros:
                lens = len(joint_data.velocity)
                if lens == 7:
                    vel_all = clip(round(joint_data.velocity[6]), 1, 100)
                    self.get_logger().info(f"vel_all: {vel_all}")
                    self.piper.MotionCtrl_2(0x01, 0x01, vel_all)
                else:
                    self.piper.MotionCtrl_2(0x01, 0x01, 30)
            else:
                self.piper.MotionCtrl_2(0x01, 0x01, 30)

            # 使用关节名称来动态控制关节
            self.piper.JointCtrl(
                joint_positions.get('joint1', 0),
                joint_positions.get('joint2', 0),
                joint_positions.get('joint3', 0),
                joint_positions.get('joint4', 0),
                joint_positions.get('joint5', 0),
                joint_positions.get('joint6', 0)
            )

            # 夹爪控制
            if self.gripper_exist:
                if len(joint_data.effort) >= 7:
                    gripper_effort = clip(joint_data.effort[6], 0.5, 3)
                    # self.get_logger().info(f"gripper_effort: {gripper_effort}")
                    if not math.isnan(gripper_effort):
                        gripper_effort = round(gripper_effort * 1000)
                    else:
                        # self.get_logger().warning("Gripper effort is NaN, using default value.")
                        gripper_effort = 0  # 设置默认值
                    self.piper.GripperCtrl(abs(joint_6), gripper_effort, 0x01, 0)
                else:
                    self.piper.GripperCtrl(abs(joint_6), 1000, 0x01, 0)


    def enable_callback(self, enable_flag: Bool):
        """Callback function for enabling the robotic arm

        Args:
            enable_flag (): Boolean flag
        """
        self.get_logger().info(f"Received enable flag:")
        self.get_logger().info(f"enable_flag: {enable_flag.data}")
        if enable_flag.data:
            self.__enable_flag = True
            self.piper.EnableArm(7)
            if self.gripper_exist:
                self.piper.GripperCtrl(0, 1000, 0x01, 0)
        else:
            self.__enable_flag = False
            self.piper.DisableArm(7)
            if self.gripper_exist:
                self.piper.GripperCtrl(0, 1000, 0x00, 0)

    def handle_enable_service(self, req, resp):
        """Handle enable service for the robotic arm"""
        self.get_logger().info(f"Received request: {req.enable_request}")
        enable_flag = False
        loop_flag = False
        # Set timeout duration (seconds)
        timeout = 5
        # Record the time before entering the loop
        start_time = time.time()
        while not loop_flag:
            elapsed_time = time.time() - start_time
            self.get_logger().info(f"--------------------")
            enable_list = []
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status)
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status)
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status)
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status)
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status)
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status)

            if req.enable_request:
                enable_flag = all(enable_list)
                self.piper.EnableArm(7)
                self.piper.GripperCtrl(0, 1000, 0x01, 0)
            else:
                enable_flag = any(enable_list)
                self.piper.DisableArm(7)
                self.piper.GripperCtrl(0, 1000, 0x02, 0)

            self.get_logger().info(f"Enable status: {enable_flag}")
            self.__enable_flag = enable_flag
            self.get_logger().info(f"--------------------")

            if enable_flag == req.enable_request:
                loop_flag = True
                enable_flag = True
            else:
                loop_flag = False
                enable_flag = False

            # Check if timeout duration has been exceeded
            if elapsed_time > timeout:
                self.get_logger().info(f"Timeout...")
                enable_flag = False
                loop_flag = True
                break

            time.sleep(0.5)

        resp.enable_response = enable_flag
        self.get_logger().info(f"Returning response: {resp.enable_response}")
        return resp


def main(args=None):
    rclpy.init(args=args)
    piper_single_node = PiperRosNode()
    try:
        rclpy.spin(piper_single_node)
    except KeyboardInterrupt:
        pass
    finally:
        piper_single_node.joint_servo_running = False
        if hasattr(piper_single_node, 'joint_servo_thread'):
            piper_single_node.joint_servo_thread.join(timeout=0.5)
        piper_single_node.destroy_node()
        rclpy.shutdown()
