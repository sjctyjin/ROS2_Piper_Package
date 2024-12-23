from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 節點 1: transform_point
        Node(
            package='transform_example',
            executable='transform_point',
            name='transform_point_node',
            output='screen'
        ),
        # 節點 2: detection_to_moveit
        #Node(
        #    package='transform_example',
        #    executable='detection_to_moveit',
        #    name='detection_to_moveit_node',
        #    output='screen'
        #),
        # 節點 3: yolov8_detect
        Node(
            package='transform_example',
            executable='yolov8_detect',
            name='yolov8_detect_node',
            output='screen'
        )
    ])

