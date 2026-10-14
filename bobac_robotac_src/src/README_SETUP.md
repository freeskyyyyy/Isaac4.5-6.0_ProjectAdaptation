# Bobac 机器人抓取系统 - 安装与配置指南

本文档详细说明如何从零开始安装和配置 Bobac 机器人抓取系统。

---

## 目录

- [系统要求](#系统要求)
- [安装步骤](#安装步骤)
  - [1. 安装 ROS2 Humble](#1-安装-ros2-humble)
  - [2. 安装 MoveIt2](#2-安装-moveit2)
  - [3. 安装 Python 依赖](#3-安装-python-依赖)
  - [4. 创建工作空间](#4-创建工作空间)
  - [5. 克隆功能包](#5-克隆功能包)
  - [6. 编译工作空间](#6-编译工作空间)
  - [7. 配置环境变量](#7-配置环境变量)
- [验证安装](#验证安装)
- [常见问题](#常见问题)

---

## 系统要求

### 硬件要求
- **CPU**: Intel Core i5 或更高（推荐 i7/i9）
- **内存**: 至少 8GB RAM（推荐 16GB+）
- **GPU**: NVIDIA GPU（支持 CUDA 11.x+）用于 YOLOE 推理
- **存储**: 至少 20GB 可用空间

### 软件要求
- **操作系统**: Ubuntu 22.04 LTS
- **ROS2 版本**: Humble Hawksbill
- **Python 版本**: 3.10+
- **CUDA 版本**: 11.x 或 12.x（用于 GPU 加速）

---

## 安装步骤

### 1. 安装 ROS2 Humble

#### 1.1 设置 locale
```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

#### 1.2 添加 ROS2 apt 源
```bash
# 添加 ROS2 GPG key
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# 添加 ROS2 repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

#### 1.3 安装 ROS2 Humble
```bash
sudo apt update
sudo apt upgrade -y

# 安装 ROS2 Humble 桌面版（包含 RViz、rqt 等工具）
sudo apt install -y ros-humble-desktop

# 安装开发工具
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  build-essential \
  git
```

#### 1.4 初始化 rosdep
```bash
sudo rosdep init
rosdep update
```

#### 1.5 配置环境变量
```bash
# 添加到 ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### 1.6 验证安装
```bash
# 检查 ROS2 版本
ros2 --version

# 测试 ROS2
ros2 run demo_nodes_cpp talker
# 在另一个终端运行
ros2 run demo_nodes_cpp listener
```

---

### 2. 安装 MoveIt2

MoveIt2 是 ROS2 的运动规划框架，是本项目的核心依赖。

#### 2.1 安装 MoveIt2 核心包
```bash
sudo apt update
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-moveit-ros-planning \
  ros-humble-moveit-ros-planning-interface \
  ros-humble-moveit-kinematics \
  ros-humble-moveit-planners-ompl \
  ros-humble-moveit-simple-controller-manager \
  ros-humble-moveit-ros-visualization \
  ros-humble-ompl \
  libtinyxml-dev \
  libtinyxml2-dev
```

#### 2.2 安装 MoveIt2 相关工具
```bash
sudo apt install -y \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-tf2-tools
```

#### 2.3 安装图像处理相关包
```bash
sudo apt install -y \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-vision-msgs \
  ros-humble-sensor-msgs
```

#### 2.4 验证 MoveIt2 安装
```bash
# 检查 MoveIt2 包
ros2 pkg list | grep moveit

# 应该看到类似输出：
# moveit
# moveit_common
# moveit_core
# moveit_kinematics
# moveit_planners_ompl
# moveit_ros_planning
# moveit_ros_planning_interface
# ...
```

---

### 3. 安装 Python 依赖

本项目使用 YOLOE 进行物体检测，需要安装相关 Python 库。

#### 3.1 更新 pip
```bash
python3 -m pip install --upgrade pip
```

#### 3.2 安装 Ultralytics YOLOE
```bash
# 安装 Ultralytics（包含 YOLOE）
python3 -m pip install -U ultralytics
```

#### 3.3 安装 NumPy（注意版本限制）
```bash
# 注意：必须使用 NumPy < 2.0，因为 ROS2 Humble 的 cv_bridge 不兼容 NumPy 2.x
# 同时需要降级 OpenCV 到兼容 NumPy 1.x 的版本
python3 -m pip install "numpy<2.0" "opencv-python<4.10.0"
```

#### 3.4 安装 OpenAI CLIP
```bash
# YOLOE 需要 CLIP 进行某些功能
python3 -m pip install -U openai-clip
```

#### 3.5 安装 PyTorch

```bash
# 1. 检查你的 CUDA 版本
nvidia-smi
# 查看右上角 "CUDA Version: X.X"

# 2. 根据你的 CUDA 版本选择对应的安装命令：

# CUDA 11.x (比如下面是：11.8 则可以使用下面的命令来安装 11.8 版本的 torch-cuda):
python3 -m pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

# CUDA x.x :
# python3 -m pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cuxxx

# 没有 NVIDIA GPU (不推荐，推理速度慢):
# python3 -m pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu

# 3. 验证安装
python3 -c "import torch; print(f'PyTorch: {torch.__version__}, CUDA available: {torch.cuda.is_available()}')"
```


*也可以进入 `https://download.pytorch.org/whl/` 中查看一下是否有指定可安装的版本*


**说明**：
- CUDA 11.x 用户 → 安装 cu118 版本
- CUDA 12.x 用户 → 安装 cu121 版本
- 如果不确定，使用 cu118（兼容性更好）


#### 3.6 验证 Python 依赖
```bash
# 验证 Ultralytics
python3 -c "from ultralytics import YOLOE; print('Ultralytics OK')"

# 验证 NumPy 版本
python3 -c "import numpy; print(f'NumPy version: {numpy.__version__}')"
# 应该显示 1.x.x，不是 2.x.x

# 验证 PyTorch 和 CUDA
python3 -c "import torch; print(f'PyTorch: {torch.__version__}'); print(f'CUDA available: {torch.cuda.is_available()}')"
# 应该显示 CUDA available: True

# 验证 OpenCV
python3 -c "import cv2; print(f'OpenCV version: {cv2.__version__}')"
```

---

### 4. 创建工作空间

#### 4.1 创建工作空间目录
```bash
# 创建工作空间根目录
mkdir -p ~/bobac_ws/src

# 进入工作空间
cd ~/bobac_ws
```

#### 4.2 初始化工作空间
```bash
# 确保已 source ROS2 环境
source /opt/ros/humble/setup.bash

# 初始化工作空间（可选，colcon build 会自动创建）
colcon build
```

---

### 5. 克隆功能包

**克隆整个仓库**
```bash
cd ~/bobac_ws/src

# 克隆仓库（替换为实际的仓库地址）
git clone http://192.168.1.28/airobotic/algorithm.git -b dev-gzr .

# 注意：最后的 . 表示克隆到当前目录（src），而不是创建子目录
```


#### 5.2 验证功能包结构
```bash
cd ~/bobac_ws/src
ls -l

# 应该看到以下目录：
# eco65b_grasp_interfaces/
# eco65b_moveit_config/
# eco65b_moveit_description/
# eco65b_moveit_grasp/
# grasp_control/
# joint_state_filter/
# yoloe_grasp/
# README.md
# README_SETUP.md
```

---



<br>
<br>
<br>
<br>

# bobac3_navigation 部署与安装指南

详细说明如何部署和安装 `bobac3_navigation` 功能包及其所有依赖环境。

## 目录

- [系统要求](#系统要求)
- [依赖项安装](#依赖项安装)
- [环境配置](#环境配置)
- [编译与安装](#编译与安装)
- [验证安装](#验证安装)
- [常见问题](#常见问题)

## 系统要求

### 硬件要求

- **CPU**: Intel i5 或更高（推荐 i7/i9 或 AMD Ryzen 5/7/9）
- **内存**: 最低 8GB RAM（推荐 16GB 或更高）
- **GPU**: NVIDIA GPU（用于 Isaac Sim，推荐 RTX 系列）
  - 最低: GTX 1060 6GB
  - 推荐: RTX 3060 或更高
- **存储**: 至少 50GB 可用空间

### 软件要求

- **操作系统**: Ubuntu 22.04 LTS (Jammy Jellyfish)
- **ROS 2 版本**: Humble Hawksbill
- **Python**: 3.10 或更高
- **NVIDIA Isaac Sim**: 2023.1.0 或更高（可选，用于仿真）

## 依赖项安装
### 1. 安装核心构建工具和客户端库

```bash
# 安装 CMake 构建工具
sudo apt install -y \
    ros-humble-ament-cmake \
    ros-humble-ament-cmake-python

# 安装 ROS 2 客户端库
sudo apt install -y \
    ros-humble-rclcpp \
    ros-humble-rclpy

# 安装 ROS 2 核心组件
sudo apt install -y \
    ros-humble-rclcpp-components \
    ros-humble-rclpy-common-interfaces
```

### 2. 安装 Nav2 导航栈及所有必需组件

```bash
# 安装 Nav2 元包和核心包
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-core \
    ros-humble-nav2-util \
    ros-humble-nav2-msgs

# 安装 AMCL 定位
sudo apt install -y ros-humble-nav2-amcl

# 安装规划器
sudo apt install -y \
    ros-humble-nav2-planner \
    ros-humble-nav2-navfn-planner \
    ros-humble-nav2-smac-planner

# 安装控制器
sudo apt install -y \
    ros-humble-nav2-controller \
    ros-humble-nav2-dwb-controller \
    ros-humble-nav2-mppi-controller

# 安装代价地图
sudo apt install -y \
    ros-humble-nav2-costmap-2d \
    ros-humble-nav2-voxel-grid

# 安装行为树和导航器
sudo apt install -y \
    ros-humble-nav2-bt-navigator \
    ros-humble-nav2-behavior-tree

# 安装行为服务器
sudo apt install -y ros-humble-nav2-behaviors

# 安装地图服务器
sudo apt install -y ros-humble-nav2-map-server

# 安装路径平滑器
sudo apt install -y ros-humble-nav2-smoother

# 安装速度平滑器
sudo apt install -y ros-humble-nav2-velocity-smoother

# 安装生命周期管理器
sudo apt install -y ros-humble-nav2-lifecycle-manager

# 安装路点跟随器
sudo apt install -y ros-humble-nav2-waypoint-follower

# 安装 RViz2 插件
sudo apt install -y ros-humble-nav2-rviz-plugins
```

### 3. 安装消息类型包

```bash
# 安装传感器消息
sudo apt install -y ros-humble-sensor-msgs

# 安装几何消息
sudo apt install -y ros-humble-geometry-msgs

# 安装导航消息
sudo apt install -y ros-humble-nav-msgs

# 安装标准消息
sudo apt install -y \
    ros-humble-std-msgs \
    ros-humble-std-srvs

# 安装 TF2 变换库
sudo apt install -y \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-tf2-sensor-msgs \
    ros-humble-tf2-tools
```

### 4. 安装可视化工具

```bash
# 安装 RViz2
sudo apt install -y \
    ros-humble-rviz2 \
    ros-humble-rviz-common \
    ros-humble-rviz-default-plugins \
    ros-humble-rviz-rendering

# 安装 joint state publisher
sudo apt install -y \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui
```

### 5. 安装其他工具和可选依赖

```bash
# 安装点云处理工具（可选）
sudo apt install -y ros-humble-pointcloud-to-laserscan

# 安装机器人定位工具（可选，用于传感器融合）
sudo apt install -y ros-humble-robot-localization

# 安装 SLAM 工具（可选，用于建图）
sudo apt install -y ros-humble-slam-toolbox

# 安装诊断工具
sudo apt install -y \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-rqt-robot-plugins \
    ros-humble-rqt-graph

# 安装性能分析工具
sudo apt install -y \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers
```

<br>
<br>
<br>
<br>


# 编译工作空间

### 6 编译功能包

**方法 1: 编译所有功能包**
```bash
cd ~/bobac_ws
source /opt/ros/humble/setup.bash

# 编译所有功能包
colcon build

# 如果遇到内存不足，可以限制并行任务数
# colcon build --parallel-workers 2
```

**方法 2: 只编译核心功能包（推荐）**
```bash
cd ~/bobac_ws
source /opt/ros/humble/setup.bash

# 只编译核心功能包
colcon build --packages-select \
  eco65b_grasp_interfaces \
  eco65b_moveit_grasp \
  yoloe_grasp \
  grasp_control \
  joint_state_filter \
  eco65b_moveit_config \
  eco65b_moveit_description
```

**方法 3: 逐个编译（用于调试）**
```bash
cd ~/bobac_ws
source /opt/ros/humble/setup.bash

# 1. 先编译接口包（其他包依赖它）
colcon build --packages-select eco65b_grasp_interfaces

# 2. 编译描述包
colcon build --packages-select eco65b_moveit_description

# 3. 编译配置包
colcon build --packages-select eco65b_moveit_config

# 4. 编译其他功能包
colcon build --packages-select eco65b_moveit_grasp
colcon build --packages-select yoloe_grasp
colcon build --packages-select grasp_control
colcon build --packages-select joint_state_filter
```

#### 6.1 处理编译错误

**常见错误 1: spdlog/fmt 链接冲突**
```
错误信息: undefined reference to `fmt::...`
```
**解决方法**: 在干净的终端中编译（不要加载 IsaacSim 环境）
```bash
# 打开新终端
source /opt/ros/humble/setup.bash
cd ~/bobac_ws
colcon build
```

**常见错误 2: 找不到 MoveIt2 头文件**
```
错误信息: fatal error: moveit/move_group_interface/move_group_interface.h: No such file or directory
```
**解决方法**: 安装 MoveIt2 开发包
```bash
sudo apt install -y ros-humble-moveit-ros-planning-interface
```

**常见错误 3: Python 模块导入失败**
```
错误信息: ModuleNotFoundError: No module named 'ultralytics'
```
**解决方法**: 确保 Python 依赖已安装
```bash
python3 -m pip install -U ultralytics "numpy<2.0" openai-clip
```

---

### 7. 配置环境变量

#### 7.1 Source 工作空间
```bash
cd ~/bobac_ws
source install/setup.bash
```

#### 7.2 添加到 ~/.bashrc（可选）
```bash
# 自动 source 工作空间
echo "source ~/bobac_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```



## 可选配置或验证



#### 7.3 配置 YOLOE 模型路径

**下载 YOLOE 模型**（如果还没有，默认已有）：
```bash
# 下载 YOLOE 模型到根目录
cd /
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yoloe-26l-seg.pt

# 或者使用其他模型
# wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yoloe-26m-seg.pt
```

**验证模型路径**：
```bash
ls -lh /yoloe-26l-seg.pt
# 应该显示模型文件
```

*一般已经不需要重新安装权重文件，因为需要使用的权重文件已经存放在了 yoloe_grasp 功能包的 weights 文件夹中，可以直接在 yoloe_grasp 功能包中的 config 文件夹中的 yaml 文件中配置更换为别的权重文件以平衡性能与识别效果*

---

## 验证安装(可选)

### 1. 检查功能包
```bash
# 检查所有功能包是否正确安装
ros2 pkg list | grep eco65b
ros2 pkg list | grep yoloe
ros2 pkg list | grep grasp

# 应该看到：
# eco65b_grasp_interfaces
# eco65b_moveit_config
# eco65b_moveit_description
# eco65b_moveit_grasp
# grasp_control
# yoloe_grasp
```

### 2. 检查可执行文件
```bash
# 检查节点是否可用
ros2 run eco65b_moveit_grasp grasp_moveit_node --help
ros2 run yoloe_grasp yoloe_grasp_node --help
ros2 run grasp_control gripper_control_node --help
ros2 run grasp_control grasp_state_machine --help
```

### 3. 检查 Action 接口
```bash
# 检查 Action 接口是否正确编译
ros2 interface show eco65b_grasp_interfaces/action/GraspToPoint
ros2 interface show eco65b_grasp_interfaces/action/DetectObject
ros2 interface show eco65b_grasp_interfaces/action/GripperControl
```

### 4. 测试 MoveIt2 启动
```bash
# 测试 MoveIt2 是否能正常启动
ros2 launch eco65b_moveit_config move_group.launch.py

# 如果看到以下输出，说明启动成功：
# [move_group-...] You can start planning now!
```

### 5. 测试 YOLOE 模型加载
```bash
# 测试 YOLOE 模型是否能正常加载
python3 << EOF
from ultralytics import YOLOE
model = YOLOE('/yoloe-26l-seg.pt')
print('YOLOE model loaded successfully!')
EOF
```

---

## 常见问题

### Q1: 编译时提示 "package 'xxx' not found"
**A**: 使用 rosdep 安装缺失的依赖：
```bash
cd ~/bobac_ws
rosdep install --from-paths src --ignore-src -r -y
```

### Q2: 运行时提示 "No module named 'ultralytics'"
**A**: 确保 Python 依赖已安装：
```bash
python3 -m pip install -U ultralytics "numpy<2.0" openai-clip
```

### Q3: CUDA 不可用
**A**: 检查 NVIDIA 驱动和 CUDA 是否正确安装：
```bash
# 检查 NVIDIA 驱动
nvidia-smi

# 检查 CUDA 版本
nvcc --version

# 检查 PyTorch CUDA
python3 -c "import torch; print(torch.cuda.is_available())"
```

### Q4: MoveIt2 启动失败
**A**: 检查 URDF/SRDF 文件是否正确：
```bash
# 检查 URDF 语法
check_urdf ~/bobac_ws/install/eco65b_moveit_description/share/eco65b_moveit_description/urdf/eco65b.urdf

# 查看 MoveIt2 日志
ros2 launch eco65b_moveit_config move_group.launch.py --log-level debug
```

### Q5: 找不到 TF 变换
**A**: 检查 TF 树是否完整：
```bash
# 查看 TF 树
ros2 run tf2_tools view_frames

# 检查特定变换
ros2 run tf2_ros tf2_echo base_link_arm link_6
```

### Q6: NumPy 版本冲突
**A**: 确保使用 NumPy < 2.0 并降级 OpenCV：
```bash
# 卸载不兼容的版本
python3 -m pip uninstall numpy opencv-python -y

# 安装兼容的版本
python3 -m pip install "numpy<2.0" "opencv-python<4.10.0"

# 验证版本
python3 -c "import numpy; print(f'NumPy: {numpy.__version__}')"
python3 -c "import cv2; print(f'OpenCV: {cv2.__version__}')"

# 应该显示：
# NumPy: 1.26.x
# OpenCV: 4.9.x 或更低
```

### Q7: 编译时内存不足
**A**: 限制并行编译任务数：
```bash
# 只使用 2 个并行任务
colcon build --parallel-workers 2

# 或者只使用 1 个任务（最慢但最省内存）
colcon build --parallel-workers 1
```

### Q8: IsaacSim 环境冲突
**A**: 在干净的终端中编译和运行：
```bash
# 不要 source IsaacSim 环境
# 只 source ROS2 环境
source /opt/ros/humble/setup.bash
source ~/bobac_ws/install/setup.bash
```

---

## 下一步

安装完成后，请参考 [README.md](README.md) 了解如何启动和使用抓取系统。

---

## 获取帮助

如果遇到问题，请：
1. 查看本文档的"常见问题"部分
2. 查看 [README.md](README.md) 的"故障排查"部分
3. 寻求工程师的帮助

---

**最后更新**: 2025-05-26
