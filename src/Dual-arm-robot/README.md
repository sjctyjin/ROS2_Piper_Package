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
```
