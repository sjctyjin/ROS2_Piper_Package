#!/usr/bin/env python3
# dual_arm_piper_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
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
    # ➤ 相機 1（手臂左側 D405）
    cam1_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='cam1',
        name='cam1',
        parameters=[
            {'camera_name':'cam1'},
            {'serial_no': '_218622270498'},  # <- 替換為你的實際序號
            {'align_depth.enable': True},
            {'pointcloud.enable': True},
        ],
        output='screen'
    )

    # ➤ 相機 2（手臂右側 D405）
    cam2_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='cam2',
        name='cam2',
        parameters=[
            {'serial_no': '_218722270604'},
            {'align_depth.enable': True},
            {'pointcloud.enable': True},
        ],
        output='screen'
    )

    # ➤ 相機 3（車體 D435）
    """
    cam3_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        #namespace='cam3',
        name='cam3',
        parameters=[
            #{'serial_no': '_3456789CDEFA'},
            {'serial_no': '_036222070160'},
            {'align_depth.enable': True},
            {'pointcloud.enable': True},
            {'base_frame_id': 'cam3_d435_base_link'},
        ],
        output='screen'
    )
    """
    

    
    realsense_launch_dir = os.path.join(
        FindPackageShare('realsense2_camera').find('realsense2_camera'),
        'examples',
        'dual_camera'
    )
    
    cam3_realsense_launch_dir = os.path.join(
        get_package_share_directory('realsense2_camera'), 'launch')
        
    cam3_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            cam3_realsense_launch_dir, '/rs_launch.py'  # <--- 注意：這裡用的是 rs_launch.py
        ]),
        launch_arguments={
            'serial_no': '_036222070160',
            'camera_name': 'cam3',
            'camera_namespace': 'cam3', 
            'pointcloud.enable': 'true',
            'align_depth.enable': 'true',
            'enable_sync': 'true',
            # 您可以為每支相機設定不同的解析度或幀率
            # 'depth_module.profile': '640x480x30',
            # 'rgb_camera.profile': '640x480x30',
        }.items()
    )
    dual_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            realsense_launch_dir, '/rs_dual_camera_launch.py'
        ]),
        launch_arguments={
            'serial_no1': '_218622270498',
            'serial_no2': '_218722270604',
            'camera_name1': 'cam1',
            'camera_name2': 'cam2',
            'camera_namespace1': 'cam1',
            'camera_namespace2': 'cam2',
            'pointcloud.enable': 'true',
            'align_depth.enable': 'true',
        }.items()
    )
    
    # 啟動偵測
    
    cam1_yolo = Node(
        package='transform_example',
        executable='yolov8_detect_dual',
        name='cam1_yolo',
        parameters=[
            {'namespace': 'cam1'},
            {'arm': 'arm1'}
        ],
        output='screen'
    )
    
    cam2_yolo = Node(
        package='transform_example',
        executable='yolov8_detect_dual',
        name='cam2_yolo',
        parameters=[
            {'namespace': 'cam2'},
            {'arm': 'arm2'}
        ],
        output='screen'
    )
    
    cam3_yolo = Node(
        package='transform_example',
        executable='yolov8_detect_dual_SAM',
        name='cam3_yolo',
        parameters=[
            {'namespace': 'cam3'},
            {'arm': 'arm3'}
        ],
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
        dual_camera_launch,
        #cam1_node,
        #cam2_node,
        cam3_node,
        cam1_yolo,
        cam2_yolo,
        cam3_yolo,
    ])
