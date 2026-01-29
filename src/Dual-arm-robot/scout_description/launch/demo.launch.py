import os
import launch
import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. 尋找 scout_description 套件路徑
    pkg_scout_description = FindPackageShare("scout_description").find("scout_description")

    # 2. 設定 Xacro 檔案路徑
    #default_model_path = os.path.join(pkg_scout_description, "urdf", "combined_robot_doublearm.xacro")#含車子
    #default_model_path = os.path.join(pkg_scout_description, "urdf", "piper_dual.xacro")#單兩手臂

    #default_model_path = os.path.join(pkg_scout_description, "urdf", "combined_robot_triplearm_nocar.xacro")#含機構手臂
    default_model_path = os.path.join(pkg_scout_description, "urdf", "combined_robot_triplearm_havecar.xacro")#含機構手臂
    default_rviz_config_path = os.path.join(pkg_scout_description, "rviz", "model_display.rviz")

    # 3. 執行 xacro 產生 XML (robot_description_content)
    robot_description_content = Command([
        FindExecutable(name="xacro"),
        " ",
        default_model_path
    ])

    # 4. 將 robot_description 包裝為字串，避免 ROS2 解析 YAML
    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content,
            value_type=str
        )
    }

    # 5. 建立各節點：robot_state_publisher, rviz2, joint_state_publisher_gui
    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )
    
    pub_state_publisher_node = launch_ros.actions.Node(
        package="transform_example",
        executable="joint_gui_pub_dual",
        output="screen",
        parameters=[robot_description],
    )


    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", default_rviz_config_path]
    )

    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher",
        output="screen"
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node,
        #pub_state_publisher_node
        #joint_state_publisher_gui_node
    ])

