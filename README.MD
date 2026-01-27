# Dual-Arm Piper + cuRobo MPC 快速啟動指南

本文件說明如何在 ROS 2 Humble 環境下，完成  
雙臂 Piper 機械手臂 + 三相機 + cuRobo MPC 採摘流程 的最小可執行設定。


--------------------------------------------------
## 1. 確認 Python / CUDA 執行環境
--------------------------------------------------

請先確認已安裝對應 CUDA Driver，並使用以下版本：
```bash
pip install numpy==1.23.5
pip install torch==2.6.0+cu126
pip install torchaudio==2.6.0+cu126
pip install torchvision==0.21.0+cu126
```

--------------------------------------------------
## 2. 安裝必要 ROS 套件
--------------------------------------------------
```bash
sudo apt install ros-humble-tf-transformations
```

--------------------------------------------------
## 3. 前置設定（cuRobo + ROS2 Workspace）
--------------------------------------------------

### 3.1 安裝 cuRobo
```bash
sudo apt install git-lfs
git clone https://github.com/NVlabs/curobo.git
cd curobo
pip install -e . --no-build-isolation
```

### 3.2 建立 ROS 2 Workspace 並編譯 Piper 套件
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

git clone https://github.com/sjctyjin/ROS2_Piper_Package.git -b triple_arm_dev

cd ..
colcon build --symlink-install
source install/setup.bash
```

--------------------------------------------------
## 4. 啟動雙臂（CAN to USB 設定）
--------------------------------------------------

注意事項：
請勿同時插入兩個 CAN to USB
請依序插入並確認 USB Port 編號


### 4.1 查詢 CAN 裝置
```bash
bash find_all_can_port.sh
```
預期會出現類似訊息：
```bash
Interface xxxx is inserted in USB port 3-6.2:1.0
Interface xxxx is inserted in USB port 3-6.3:1.0
```
請記下對應的 USB Port，例如：
```bash
3-6.2:1.0
3-6.3:1.0
```

### 4.2 命名並啟用 CAN 介面
```bash
bash can_activate.sh can_piper_1 1000000 "3-6.2:1.0"
bash can_activate.sh can_piper_2 1000000 "3-6.3:1.0"
```
確認是否成功：
```bash
ifconfig
```
應能看到：
```bash
can_piper_1
can_piper_2
```

--------------------------------------------------
## 5. 啟動系統（雙臂 + 三相機 + MPC）
--------------------------------------------------

### 5.1 啟動雙臂與相機控制節點（Mode 2）
```bash
ros2 launch scout_description piper_dual_arm_mode2.launch.py arm1_can_port:=can_piper_1 arm2_can_port:=can_piper_2
```
此 launch 會同時啟動：
- 左右雙臂 Piper
- 三顆 RGB-D 相機
- TF / Joint State / 底層控制節點


### 5.2 啟動 cuRobo MPC 採摘流程（Mode 2）
```bash
ros2 run curobo_piper curobo_pick_and_place_mpc_mode_2_limit
```
此節點負責：
- 接收 object_in_base 目標座標
- 使用 cuRobo MPC 即時規劃
- 控制雙臂完成抓取與放置


--------------------------------------------------
6. 常用除錯指令（選用）
--------------------------------------------------

監聽手臂末端位姿：
```bash
ros2 run tf2_ros tf2_echo base_link link6

```
--------------------------------------------------
系統流程摘要
--------------------------------------------------

1. 相機偵測目標並發布 object_in_base
2. cuRobo MPC 根據當前 joint state 即時規劃軌跡
3. Piper 雙臂依序執行：
   - 開爪
   - 移動至目標
   - 合爪
   - 移動至放置位置
