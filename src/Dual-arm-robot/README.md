## 啟動雙臂機器人Rviz
```bash
ros2 launch scout_description piper_dual_arm.launch.py
```
## 啟動多個realsense鏡頭
camera_name照urdf中定義的名稱去寫，cam1就對應cam1_link，若未指定，camera_name則預設為:=camera 對應 camera_link
```base
ros2 launch realsense2_camera rs_launch.py \
  camera_name:=cam2 \
  serial_no:=<你的相機序號> \
  pointcloud.enable:=true \
  align_depth.enable:=true
```
