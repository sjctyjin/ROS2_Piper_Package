### 確認numpy執行環境
```bash
pip install numpy==1.21.1
```
### ROS package
```bash
sudo apt install ros-humble-tf-transformations
```
### 前置設定
```bash
python 安裝 curobo
sudo apt install git-lfs
git clone https://github.com/NVlabs/curobo.git
cd curobo
pip install -e . --no-build-isolation

mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src

git clone https://github.com/sjctyjin/ROS2_Piper_Package.git

cd ..

colcon build --symlink-install

source install/setup.bash

```
### 1. 先啟動realsense ros節點
```bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true pointcloud.enable:=true
# 查看模型
ros2 launch realsense2_description view_model.launch.py model:=test_d405_camera.urdf.xacro

```
#### 03-28 奧比中光 Debai相機
```bash
ros2 launch orbbec_camera dabai.launch.py enable_point_cloud:=true enable_colored_point_cloud:=true  align_mode:=HW
```
### 2. 再啟動剛剛建置的yolov8_detect
```bash
ros2 run transform_example yolov8_detect

2025-03-14加入SAM分割後 點雲 姿態估測

ros2 run transform_example yolov8_detect_pose
```
### 3. 最後啟動piper官方launch包
```bash
ros2 launch piper start_single_piper_rviz.launch.py
```
### 4. 最後啟動curobo控制抓取放-自動執行
```bash
ros2 run curobo_piper curobo_pick_and_place

# 自定义放置位置
ros2 launch motion_gen_ros2 pick_and_place_launch.py --ros-args -p place_position_x:=0.0 -p place_position_y:=0.3 -p place_position_z:=0.25

# 启用ROSBridge
ros2 launch motion_gen_ros2 pick_and_place_launch.py --ros-args -p enable_rosbridge:=true -p rosbridge_host:='127.0.0.1'

```

### 5. 指令監聽手臂末端座標及姿態

```bash
ros2 run tf2_ros tf2_echo base_link link6 #監聽手臂末端座標及姿態值
```
### 指令動作規劃
```bash 

此指令動作包含 :
1. 將object_in_base座標及姿態提供給手臂規劃組
抓取示例:透過object_in_base物體座標 call plan 規劃移動
ros2 run curobo_piper curobo_plan
or
移動示例:透過 call plan 物體座標規劃移動到指定座標
ros2 run curobo_piper curobo_test --ros-args -p place_position_x:=0.0 -p place_position_y:=0.3 -p place_position_z:=0.25

2. arm規劃組進行規畫並移動
移動順序
1. 開爪 (gripper open)
2. 手臂末端移動至該座標
3. 合爪 (gripper close)

ros2 run transform_example gripper_pose_joint

```
### 下執行指令

```bash
moveit移動指令
ros2 service call /trigger_plan std_srvs/srv/Trigger {}
夾爪指令
ros2 service call /open_gripper std_srvs/srv/Trigger
```
