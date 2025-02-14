pip install numpy==1.21.1


先啟動realsense ros節點
1. 
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true pointcloud.enable:=true
再啟動剛剛建置的yolov8_detect
2 . 
ros2 run transform_example yolov8_detect
最後啟動piper的xarco，就能得到一開始的結果。
3. ros2 launch piper_description display_xacro.launch.py


最後啟動moveit

4. ros2 launch piper_moveit_config demo.launch.py

啟動座標轉換將yolov8_detect偵測到的坐標系object_frame 轉換到相對於base_link的object_in_base

5. ros2 run transform_example transform_point

啟動detection_to_moveit將偵測的object_in_base轉換到moveit執行

6. ros2 run transform_example detection_to_moveit

7. ros2 run tf2_ros tf2_echo base_link link6 #監聽手臂末端座標及姿態值
