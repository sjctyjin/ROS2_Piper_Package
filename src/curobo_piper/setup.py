from setuptools import find_packages, setup

package_name = 'curobo_piper'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
         'curobo_test = curobo_piper.curobo_test:main',#curobo計算固定的座標 發送到joint_state
         'curobo_plan = curobo_piper.curobo_plan:main',#curobo計算軌跡服務-監聽joint_custom_state
         'curobo_plan_keep = curobo_piper.curobo_plan_keep:main',#curobo計算軌跡並持續跟蹤-監聽joint_custom_state
         'curobo_gen_motion = curobo_piper.curobo_gen_motion:main',#curobo gen計算軌跡並持續跟蹤-監聽joint_custom_state
         'curobo_pick_and_place = curobo_piper.curobo_pick_and_place:main',#curobo gen計算軌跡 抓取放
         'curobo_pick_and_place_mpc = curobo_piper.curobo_pick_and_place_MPC:main',#curobo MPC計算軌跡 抓取放
         'curobo_pick_and_place_mpc_dual = curobo_piper.curobo_pick_and_place_MPC_dual:main',#curobo MPC計算軌跡 抓取放(雙臂)
         'curobo_pick_and_place_mpc_dual_dual = curobo_piper.curobo_pick_and_place_MPC_dual_dual:main',# 計算雙臂碰撞
         'curobo_vr_mpc = curobo_piper.curobo_vr_mpc:main',#透過quest3手柄 搖操作-單臂  ros2 run curobo_piper curobo_vr_mpc
         #單純雙臂mpc持續跟蹤 無其他任何操作
         'curobo_dual_arm_mpc_tracker = curobo_piper.curobo_dual_arm_mpc_tracker:main',
         #單臂mpc持續跟蹤(但雙臂會一起移動) 可指定 trip_piper_right.yml 或 trip_piper_left.yml 
         'curobo_continue_track = curobo_piper.curobo_contine_track:main',
         #ros2 run curobo_piper curobo_continue_track --ros-args -p dual_config:=trip_piper_right.yml
         #透過quest3手柄 搖操作-雙臂 搖桿控制XY旋轉 側邊控制Z軸旋轉 ros2 run curobo_piper curobo_vr_mpc --ros-args -p hand:=right
         'curobo_vr_mpc_dual = curobo_piper.curobo_vr_mpc_dual:main',
         #雙臂 雙採摘 二次定位：右臂扶支 左臂扭轉
         'curobo_pick_and_place_mpc_trip = curobo_piper.curobo_pick_and_plack_Trip:main',
         #雙臂 單採摘二次定位：左臂剪刀手 右臂接水果 
         'curobo_pick_and_place_mpc_mode_2 = curobo_piper.curobo_pick_and_place_mpc_mode_2:main',
         #雙臂 單採摘二次定位：左臂剪刀手 右臂接水果 limit joint版本
         'curobo_pick_and_place_mpc_mode_2_limit = curobo_piper.curobo_pick_and_place_mpc_mode_2_fast:main',

        ],
    },
)
