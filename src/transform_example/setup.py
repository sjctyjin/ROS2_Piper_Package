from setuptools import find_packages, setup

package_name = 'transform_example'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/detect.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros2',
    maintainer_email='ros2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'transform_point = transform_example.transform_point:main',#將yolov8_point輸出的座標結果轉換為base_link的相關座標
             'detection_to_moveit = transform_example.object_pose_planner:main',#將取得的物件座標轉換到手臂執行器
             'pose_point          = transform_example.pose_point:main',#POST座標發布範例
             'detect_point        = transform_example.detect_object:main',#座標發布範例，發布一個TF座標
             'yolov8_detect       = transform_example.yolov8_point:main',#讀取d435節點並輸出深度XYZ的TF座標
             'moveit_frame        = transform_example.moveit_count_frame:main',#moveit 座標發布範例
             'object_pose_planner_service        = transform_example.object_pose_planner_service:main',#moveit 目標物座標發布-Call Service
             # 控制抓取 : ros2 service call /trigger_plan std_srvs/srv/Trigger {}
             'yolov8_detect_pose = transform_example.yolov8_point_pose:main',#yolo偵測 取得物體點雲姿態
             'gripper_pose_service = transform_example.gripper_pose_planner_service:main', # 夾爪 gripper規劃組 控制開合 服務     
             # 控制開 :ros2 service call /open_gripper std_srvs/srv/Trigger
             # 控制關 :ros2 service call /close_gripper std_srvs/srv/Trigger
             'gripper_pose_joint = transform_example.gripper_pose_joint:main', #夾爪 joint7 /joint_states 控制開合 服務
             # 控制開 : ros2 service call /gripper/set_open std_srvs/srv/SetBool "{data: true}"
	     # 控制關 : ros2 service call /gripper/set_open std_srvs/srv/SetBool "{data: false}"
        ],
    },
)
