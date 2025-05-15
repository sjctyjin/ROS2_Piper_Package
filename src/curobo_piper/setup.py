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
        ],
    },
)
