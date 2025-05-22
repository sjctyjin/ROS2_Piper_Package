#!/usr/bin/env python3
# dual_arm_piper_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 獲取 piper_description 包的路徑
    piper_description_path = os.path.join(
        get_package_share_directory('scout_description'),
        'launch',
        'demo.launch.py'
    )
    
    # 定義 launch 參數
    # 左側手臂 (arm1) 參數
    arm1_can_port_arg = DeclareLaunchArgument(
        'arm1_can_port',
        default_value='can_piper_1',
        description='CAN port for the left robot arm (arm1)'
    )
    
    arm1_auto_enable_arg = DeclareLaunchArgument(
        'arm1_auto_enable',
        default_value='true',
        description='Enable left robot arm (arm1) automatically'
    )
    
    arm1_gripper_exist_arg = DeclareLaunchArgument(
        'arm1_gripper_exist',
        default_value='true',
        description='Left gripper (arm1) existence flag'
    )
    
    arm1_gripper_val_mutiple_arg = DeclareLaunchArgument(
        'arm1_gripper_val_mutiple',
        default_value='2',
        description='Left gripper (arm1) value multiple'
    )
    
    # 右側手臂 (arm2) 參數
    arm2_can_port_arg = DeclareLaunchArgument(
        'arm2_can_port',
        default_value='can_piper_2',  # 假設右側手臂用 can1
        description='CAN port for the right robot arm (arm2)'
    )
    
    arm2_auto_enable_arg = DeclareLaunchArgument(
        'arm2_auto_enable',
        default_value='true',
        description='Enable right robot arm (arm2) automatically'
    )
    
    arm2_gripper_exist_arg = DeclareLaunchArgument(
        'arm2_gripper_exist',
        default_value='true',
        description='Right gripper (arm2) existence flag'
    )
    
    arm2_gripper_val_mutiple_arg = DeclareLaunchArgument(
        'arm2_gripper_val_mutiple',
        default_value='2',
        description='Right gripper (arm2) value multiple'
    )
    
    # 通用參數
    rviz_ctrl_flag_arg = DeclareLaunchArgument(
        'rviz_ctrl_flag',
        default_value='true',
        description='Start rviz flag.'
    )
    
    # 包含 display_xacro.launch.py
    display_xacro_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(piper_description_path),
        launch_arguments={
            'rviz_ctrl_flag': LaunchConfiguration('rviz_ctrl_flag')
        }.items()
    )
    
    # 定義左側手臂 (arm1) 節點
    arm1_ctrl_node = Node(
        package='piper',
        executable='piper_dual_ctrl',
        name='arm1_ctrl_node',
        output='screen',
        parameters=[
            {'can_port': LaunchConfiguration('arm1_can_port')},
            {'auto_enable': LaunchConfiguration('arm1_auto_enable')},
            {'gripper_val_mutiple': LaunchConfiguration('arm1_gripper_val_mutiple')},
            {'gripper_exist': LaunchConfiguration('arm1_gripper_exist')},
            {'arm_prefix': 'arm1_'}  # 添加前綴參數，用於區分不同手臂的關節
        ],
        remappings=[
            ('joint_ctrl_single', '/arm1/joint_states')  # 修改話題名稱以區分不同手臂
        ]
    )
    
    # 定義右側手臂 (arm2) 節點
    arm2_ctrl_node = Node(
        package='piper',
        executable='piper_dual_ctrl',
        name='arm2_ctrl_node',
        output='screen',
        parameters=[
            {'can_port': LaunchConfiguration('arm2_can_port')},
            {'auto_enable': LaunchConfiguration('arm2_auto_enable')},
            {'gripper_val_mutiple': LaunchConfiguration('arm2_gripper_val_mutiple')},
            {'gripper_exist': LaunchConfiguration('arm2_gripper_exist')},
            {'arm_prefix': 'arm2_'}  # 添加前綴參數，用於區分不同手臂的關節
        ],
        remappings=[
            ('joint_ctrl_single', '/arm2/joint_states')  # 修改話題名稱以區分不同手臂
        ]
    )
    
    # 這是單次的 TF 發布，可以放在 launch 裡
    initial_joint_pub_node = Node(
        package='transform_example',
        executable='joint_gui_pub_dual',
        name='joint_gui_pub_dual',
        output='screen'
    )
    
    
    # 返回包含上述所有元素的 LaunchDescription 對象
    return LaunchDescription([
        # 左側手臂參數
        arm1_can_port_arg,
        arm1_auto_enable_arg,
        arm1_gripper_exist_arg,
        arm1_gripper_val_mutiple_arg,
        
        # 右側手臂參數
        arm2_can_port_arg,
        arm2_auto_enable_arg,
        arm2_gripper_exist_arg,
        arm2_gripper_val_mutiple_arg,
        
        # 通用參數
        rviz_ctrl_flag_arg,
        
        # 啟動文件和節點
        display_xacro_launch,
        arm1_ctrl_node,
        arm2_ctrl_node,
        initial_joint_pub_node,
    ])
