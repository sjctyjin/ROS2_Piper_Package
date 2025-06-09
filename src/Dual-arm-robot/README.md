## clone
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src

git clone https://github.com/sjctyjin/Dual-arm-robot.git
```
## 編譯專案
```bash
cd ..
colcon build --symlink-install
```

## 啟動機器人Rviz
```bash
ros2 launch scout_description demo.launch.py
```
## 啟動moveit
```bash
ros2 launch scout_robot_moveit_config demo.launch.py
```
