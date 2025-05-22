import os
import launch
import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 設定 package 目錄
    pkg_scout_description = FindPackageShare("scout_description").find("scout_description")

    # URDF / Xacro 設定
    default_model_path = os.path.join(pkg_scout_description, "urdf", "combined_robot.xacro")
    default_rviz_config_path = os.path.join(pkg_scout_description, "rviz", "model_display.rviz")

    # robot_description 參數 (執行 xacro 轉換)
    robot_description = Command(
        [FindExecutable(name="xacro"), " ", default_model_path]
    )

    return LaunchDescription([
        # 定義 robot_description 參數
        launch_ros.actions.Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),

        # RViz 節點
        launch_ros.actions.Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", default_rviz_config_path],
            output="screen"
        ),

        # joint_state_publisher (非 GUI 版)
        launch_ros.actions.Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher",
            output="screen"
        ),
    ])
