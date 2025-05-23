## 啟動雙臂機器人Rviz
```bash
預設can port啟動 can_piper_1,can_piper_2
ros2 launch scout_description piper_dual_arm.launch.py 
指定can port
ros2 launch scout_description piper_dual_arm.launch.py arm1_can_port:=can_piper_1 arm2_can_port:=can_piper_2

```
## 啟動多個realsense鏡頭
camera_name照urdf中定義的名稱去寫，cam1就對應cam1_link，若未指定，camera_name則預設為:=camera 對應 camera_link
```base
ros2 launch realsense2_camera rs_launch.py \
  camera_name:=cam2 \
  serial_no:=<你的相機序號> \
  pointcloud.enable:=true \
  align_depth.enable:=true

ros2 launch realsense2_camera rs_launch.py camera_name:=cam1 serial_no:=_218622270498  pointcloud.enable:=true align_depth.enable:=true
ros2 launch realsense2_camera rs_launch.py camera_name:=cam2 serial_no:=_218722270604  pointcloud.enable:=true align_depth.enable:=true
ros2 launch realsense2_camera rs_launch.py camera_name:=cam3 serial_no:=_339222070644  pointcloud.enable:=true align_depth.enable:=true


```
查看相機序號
```bash
rs-enumerate-devices
```
同時啟動雙鏡頭
```bash
ros2 launch realsense2_camera rs_dual_camera_launch.py   serial_no1:=_218622270498   serial_no2:=_218722270604 camera_name1:=cam1 camera_name2:=cam2  camera_namespace1:=cam1 camera_namespace2:=cam2 enable_rviz:=false

```
啟動偵測
```bash
ros2 run transform_example yolov8_detect_dual --ros-args -p namespace:=cam1 -p arm:=arm1

ros2 run transform_example yolov8_detect_dual --ros-args -p namespace:=cam2 -p arm:=arm2
```
啟動抓取放
```bash
ros2 run curobo_piper curobo_pick_and_place_mpc_dual --ros-args -p arm_prefix:=arm1 -p cam_prefix:=can1
```
節點啟動後的topic總覽
```bash
ros2@ros2-WS:~/ros2_workspace/official_piper_ws$ ros2 topic list
/arm1/joint_custom_state
/arm1/joint_states
/arm1_arm_status
/arm1_enable_flag
/arm1_end_pose
/arm1_pos_cmd
/arm2/joint_custom_state
/arm2/joint_states
/arm2_arm_status
/arm2_enable_flag
/arm2_end_pose
/arm2_pos_cmd
/cam1/cam1/color/camera_info
/cam1/cam1/color/image_rect_raw
/cam1/cam1/color/image_rect_raw/compressed
/cam1/cam1/color/image_rect_raw/compressedDepth
/cam1/cam1/color/image_rect_raw/theora
/cam1/cam1/color/metadata
/cam1/cam1/depth/camera_info
/cam1/cam1/depth/color/points
/cam1/cam1/depth/image_rect_raw
/cam1/cam1/depth/image_rect_raw/compressed
/cam1/cam1/depth/image_rect_raw/compressedDepth
/cam1/cam1/depth/image_rect_raw/theora
/cam1/cam1/depth/metadata
/cam1/cam1/extrinsics/depth_to_color
/cam2/cam2/color/camera_info
/cam2/cam2/color/image_rect_raw
/cam2/cam2/color/image_rect_raw/compressed
/cam2/cam2/color/image_rect_raw/compressedDepth
/cam2/cam2/color/image_rect_raw/theora
/cam2/cam2/color/metadata
/cam2/cam2/depth/camera_info
/cam2/cam2/depth/color/points
/cam2/cam2/depth/image_rect_raw
/cam2/cam2/depth/image_rect_raw/compressed
/cam2/cam2/depth/image_rect_raw/compressedDepth
/cam2/cam2/depth/image_rect_raw/theora
/cam2/cam2/depth/metadata
/cam2/cam2/extrinsics/depth_to_color
/clicked_point
/client_count
/connected_clients
/goal_pose
/initialpose
/joint_ctrl
/joint_custom_state
/joint_states
/joint_states_single
/parameter_events
/robot_description
/rosout
/tf
/tf_static

```
