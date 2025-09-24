### 確認numpy執行環境
```bash
pip install numpy==1.23.5
pip install torch==2.6.0+cu126
pip install torchaudio==2.6.0+cu126
pip install torchvision==0.21.0+cu126
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
### 1. 先啟動piper官方launch包
```bash
ros2 launch piper start_single_piper_rviz.launch.py
```
### 啟動雙臂

不要同時，請前後將CAN to USB 接上電腦，後執行指令查看
```bash
bash find_all_can_port.sh
```
應該會出現類似訊息

```bash
Interface xxxx is inserted in USB port 3-6.2:1.0
Interface xxxx is inserted in USB port 3-6.3:1.0
```
記下port號碼 3-6.2:1.0，執行下面指令命名激活
```bash
bash can_activate.sh can_piper_1 1000000 "3-6.2:1.0"
bash can_activate.sh can_piper_2 1000000 "3-6.3:1.0"
```
可執行ifconfig查看是否有can_piper_1 與 can_piper2

完成can的命名與激活後，即可個別launch piper1 與 piper2
```bash
ros2 launch piper start_single_piper.launch.py can_port:=can_piper_1
ros2 launch piper start_single_piper.launch.py can_port:=can_piper_2
```
雙臂同時啟動請參考

https://github.com/sjctyjin/ROS2_Piper_Package/blob/piper_official_humble/src/Dual-arm-robot/README.md
```bash
啟動雙臂(含底層節點)
ros2 launch scout_description piper_dual_arm.launch.py arm1_can_port:=can_piper_1 arm2_can_port:=can_piper_2
啟動dummy底層節點
cd ~/github/dummy/src/dummy2_ws
source install/setup.bash
ros2 launch dummy2_can2eth dummy2_can2eth_server.launch.py

```

### 2. 啟動realsense ros節點
```bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true pointcloud.enable:=true

# 查看模型
ros2 launch realsense2_description view_model.launch.py model:=test_d405_camera.urdf.xacro
#### 03-28 奧比中光 Debai相機
ros2 launch orbbec_camera dabai.launch.py enable_point_cloud:=true enable_colored_point_cloud:=true  align_mode:=HW
```
### 3. 再啟動yolov8_detect偵測節點
```bash
ros2 run transform_example yolov8_detect
```
#### 2025-03-14加入SAM分割後 點雲 姿態估測
```bash
ros2 run transform_example yolov8_detect_pose
```
### 4. 最後啟動curobo控制抓取放-自動執行
```bash
ros2 run curobo_piper curobo_pick_and_place
or 使用mpc抓取版本
ros2 run curobo_piper curobo_pick_and_place_mpc
# 自定义放置位置
ros2 run curobo_piper curobo_pick_and_place --ros-args -p place_position_x:=0.0 -p place_position_y:=0.3 -p place_position_z:=0.25

# 启用ROSBridge
ros2 run curobo_piper curobo_pick_and_place --ros-args -p enable_rosbridge:=true -p rosbridge_host:='127.0.0.1'


#跟隨節點 左右臂同時
ros2 run curobo_piper curobo_dual_arm_mpc_tracker   --ros-args   -p left_arm_config:=trip_piper_left.yml   -p right_arm_config:=trip_piper_right.yml   -p cooperation_mode:=offset_distance   -p approach_distance:=0.10   -p approach_strategy:=left_right

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

## VR手柄遙操作(單臂)
啟動手臂控制節點後，執行VR節點，啟動前須先確認adb device是否存在(Quest3插上typeC後接到電腦)
```bash

ros2 launch piper start_single_piper_rviz.launch.py

. The expected output:
List of devices attached
    ce0551e7                device

adb devices
adb tcpip 5555
adb shell ip route
adb connect 192.168.1.126:5555

ros2 run curobo_piper curobo_vr_mpc

```

## VR手柄遙操作(雙臂)

```bash
開啟雙臂控制節點，要注意can分的左手右手在哪
ros2 launch scout_description piper_dual_arm.launch.py arm1_can_port:=can_piper_1 arm2_can_port:=can_piper_2

# 啟動vr跟隨，分左右手
ros2 run curobo_piper curobo_vr_mpc_dual --ros-args -p hand:=right
ros2 run curobo_piper curobo_vr_mpc_dual --ros-args -p hand:=left

右手控制器:
A : 回歸原點
B : 開始跟蹤

左手控制器:
X : 回歸原點
Y : 開始跟蹤

```
### 下執行指令

```bash
moveit移動指令
ros2 service call /trigger_plan std_srvs/srv/Trigger {}
夾爪指令
ros2 service call /open_gripper std_srvs/srv/Trigger
```
