import os
import launch
import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_scout_description = FindPackageShare("scout_description").find("scout_description")
    default_model_path = os.path.join(pkg_scout_description, "urdf", "combined_robot_doublearm.xacro")
    default_rviz_config_path = os.path.join(pkg_scout_description, "rviz", "model_display.rviz")

    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", default_model_path]),
        value_type=str
    )

    return LaunchDescription([
        launch_ros.actions.Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),

        launch_ros.actions.Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", default_rviz_config_path],
            output="screen"
        ),

        launch_ros.actions.Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher",
            output="screen"
        ),
    ])

