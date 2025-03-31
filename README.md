### 確認numpy執行環境
```bash
pip install numpy==1.21.1
```
### ROS package
```bash
sudo apt install ros-humble-tf-transformations

```
### 1. 先啟動realsense ros節點
```bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true pointcloud.enable:=true
```
#### 03-28 奧比中光 Debai相機
```bash
ros2 launch orbbec_camera dabai.launch.py enable_point_cloud:=true enable_colored_point_cloud:=true  align_mode:=HW
```
### 2. 再啟動剛剛建置的yolov8_detect
```bash
ros2 run transform_example yolov8_detect
```
### 2.1 2025-03-14加入SAM分割後 點雲 姿態估測
```bash
ros2 run transform_example yolov8_detect_pose
```
### 3. 最後啟動piper的xarco，就能得到一開始的結果，如只需要啟動moveit，可直接執行第4點。
```bash
ros2 launch piper_description display_xacro.launch.py
```
### 4. 最後啟動moveit
```bash
ros2 launch piper_moveit_config demo.launch.py
```
### 5. 啟動座標轉換將yolov8_detect偵測到的坐標系object_frame 轉換到相對於base_link的object_in_base
```bash
ros2 run transform_example transform_point
```
### 6. 啟動detection_to_moveit將偵測的object_in_base轉換到moveit執行
```bash
ros2 run transform_example detection_to_moveit
```
### 7. 指令監聽手臂末端座標及姿態
```bash
ros2 run tf2_ros tf2_echo base_link link6 #監聽手臂末端座標及姿態值
```

### 第六點 為持續輸出moveit指令，若要透過call service來移動，則執行以下指令
```bash
ros2 run transform_example object_pose_planner_service 
```
### 下執行指令

```bash
ros2 service call /trigger_plan std_srvs/srv/Trigger {}
```
