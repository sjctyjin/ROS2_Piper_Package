# bringup_and_run.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    #os.environ['ROS_PACKAGE_PATH'] = '/home/ros2/ros2_workspace/official_piper_ws/src/piper_ros/src'
    
    #pkg_share = os.path.join(
    #   os.environ['ROS_PACKAGE_PATH'].split(':')[0],
    #    'piper_description'
    #)
    
    #pkg_moveit_share = os.path.join(
    #   os.environ['ROS_PACKAGE_PATH'].split(':')[0],
    #    'piper_moveit',
    #    'piper_with_gripper_moveit'
    #)

    #urdf_file = os.path.join(pkg_share, 'urdf', 'piper_description_d405.xacro')
    #controller_yaml = os.path.join(pkg_moveit_share, 'config', 'ros2_controllers.yaml')
    
    

    pkg_share      = get_package_share_directory('piper_description')
    pkg_moveit     = get_package_share_directory('piper_with_gripper_moveit')
    urdf_file      = os.path.join(pkg_share, 'urdf', 'piper_description_d405.xacro')
    controller_yaml= os.path.join(pkg_moveit, 'config', 'ros2_controllers.yaml')

    return LaunchDescription([
        # 1) 发布 robot_description
        DeclareLaunchArgument(
            'robot_description',
            default_value=Command(['xacro ', urdf_file])
        ),

        # 2) robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': LaunchConfiguration('robot_description')}]
        ),

        # 3) ros2_control_node
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'robot_description': LaunchConfiguration('robot_description')},
                        controller_yaml],
            output='screen'
        ),

        # 4) spawn controllers
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
        ),

        # 5) 直接运行你的控制节点
        #Node(
        #    package='your_pkg',
        #    executable='direct_ctl_node',  # 你自己写的 ROS2 节点，可发布 JointTrajectory
        #    name='direct_control',
        #    output='screen'
        #),
    ])

