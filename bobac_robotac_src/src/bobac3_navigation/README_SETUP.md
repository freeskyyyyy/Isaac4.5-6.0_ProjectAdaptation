# bobac3_navigation 部署与安装指南

本文档详细说明如何部署和安装 `bobac3_navigation` 功能包及其所有依赖环境。

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

### 1. 安装 ROS 2 Humble

如果尚未安装 ROS 2 Humble，请按照以下步骤操作：

```bash
# 设置语言环境
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 添加 ROS 2 apt 仓库
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装 ROS 2 Humble 桌面版
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
```

### 2. 安装核心构建工具和客户端库

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

### 6. 安装构建工具

```bash
# 安装 colcon 构建工具
sudo apt install python3-colcon-common-extensions

# 安装 rosdep
sudo apt install python3-rosdep

# 初始化 rosdep（如果尚未初始化）
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update
```

### 7. 安装 Python 依赖

```bash
# 安装 Python 3 和 pip
sudo apt install -y python3-pip python3-dev

# 安装 ROS 2 Python 客户端库（通常已包含在 ros-humble-desktop 中）
sudo apt install -y \
    python3-rclpy \
    python3-rclpy-common-interfaces

# 安装 Python 工具
pip3 install -U argcomplete
pip3 install -U vcstool

# 安装 Python YAML 库（用于参数文件解析）
sudo apt install -y python3-yaml
```

### 8. 安装 NVIDIA Isaac Sim（可选，用于仿真）

如果需要使用 Isaac Sim 进行仿真测试：

**系统要求**:
- NVIDIA GPU（推荐 RTX 系列）
- CUDA 11.8 或更高
- Ubuntu 22.04

**安装步骤**:

1. 访问 [NVIDIA Omniverse](https://www.nvidia.com/en-us/omniverse/)
2. 下载并安装 Omniverse Launcher
3. 通过 Launcher 安装 Isaac Sim 2023.1.0 或更高版本
4. 配置 Isaac Sim 的 ROS 2 桥接

详细安装步骤请参考：
- [Isaac Sim 安装指南](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_basic.html)
- [Isaac Sim ROS 2 集成](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_ros2_navigation.html)

### 9. 一键安装脚本（推荐）

为了简化安装过程，可以使用以下一键安装脚本：

```bash
#!/bin/bash
# 保存为 install_dependencies.sh

set -e

echo "=========================================="
echo "安装 bobac3_navigation 所有依赖"
echo "=========================================="

# 更新系统
echo "更新系统包..."
sudo apt update

# 安装核心构建工具
echo "安装核心构建工具..."
sudo apt install -y \
    ros-humble-ament-cmake \
    ros-humble-ament-cmake-python \
    ros-humble-rclcpp \
    ros-humble-rclpy

# 安装 Nav2 完整导航栈
echo "安装 Nav2 导航栈..."
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-core \
    ros-humble-nav2-util \
    ros-humble-nav2-msgs \
    ros-humble-nav2-amcl \
    ros-humble-nav2-planner \
    ros-humble-nav2-navfn-planner \
    ros-humble-nav2-smac-planner \
    ros-humble-nav2-controller \
    ros-humble-nav2-dwb-controller \
    ros-humble-nav2-mppi-controller \
    ros-humble-nav2-costmap-2d \
    ros-humble-nav2-voxel-grid \
    ros-humble-nav2-bt-navigator \
    ros-humble-nav2-behavior-tree \
    ros-humble-nav2-behaviors \
    ros-humble-nav2-map-server \
    ros-humble-nav2-smoother \
    ros-humble-nav2-velocity-smoother \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-waypoint-follower \
    ros-humble-nav2-rviz-plugins

# 安装消息类型包
echo "安装消息类型包..."
sudo apt install -y \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-std-msgs \
    ros-humble-std-srvs

# 安装 TF2 库
echo "安装 TF2 变换库..."
sudo apt install -y \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-tf2-sensor-msgs \
    ros-humble-tf2-tools

# 安装可视化工具
echo "安装可视化工具..."
sudo apt install -y \
    ros-humble-rviz2 \
    ros-humble-rviz-common \
    ros-humble-rviz-default-plugins \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui

# 安装可选工具
echo "安装可选工具..."
sudo apt install -y \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-robot-localization \
    ros-humble-slam-toolbox \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-rqt-robot-plugins

# 安装构建工具
echo "安装构建工具..."
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    python3-yaml

# 初始化 rosdep
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    echo "初始化 rosdep..."
    sudo rosdep init
fi
rosdep update

# 安装 Python 工具
echo "安装 Python 工具..."
pip3 install -U argcomplete vcstool

echo "=========================================="
echo "所有依赖安装完成！"
echo "=========================================="
echo ""
echo "下一步："
echo "1. 将 bobac3_navigation 包复制到 ~/humble_ws/src/"
echo "2. cd ~/humble_ws"
echo "3. rosdep install --from-paths src --ignore-src -r -y"
echo "4. colcon build --packages-select bobac3_navigation"
echo "5. source install/setup.bash"
```

保存并运行：

```bash
chmod +x install_dependencies.sh
./install_dependencies.sh
```

### 1. 创建工作空间

```bash
# 创建 ROS 2 工作空间
mkdir -p ~/humble_ws/src
cd ~/humble_ws/src
```

### 2. 克隆或复制 bobac3_navigation 包

```bash
# 如果从 Git 仓库克隆
# git clone <repository_url> bobac3_navigation

# 或者直接复制包到 src 目录
# cp -r /path/to/bobac3_navigation ~/humble_ws/src/
```

### 3. 安装包依赖

```bash
cd ~/humble_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 4. 配置环境变量

将以下内容添加到 `~/.bashrc` 文件：

```bash
# ROS 2 Humble 环境
source /opt/ros/humble/setup.bash

# 工作空间环境
source ~/humble_ws/install/setup.bash

# 可选：设置默认 RMW 实现
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 可选：Isaac Sim 环境变量（如果使用 Isaac Sim）
# export ISAACSIM_PATH=/path/to/isaac_sim
```

使环境变量生效：

```bash
source ~/.bashrc
```

### 5. 配置 DDS（可选但推荐）

为了获得更好的性能，建议使用 Cyclone DDS：

```bash
# 安装 Cyclone DDS
sudo apt install ros-humble-rmw-cyclonedds-cpp

# 创建 Cyclone DDS 配置文件
mkdir -p ~/.ros
cat > ~/.ros/cyclonedds.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain id="any">
        <General>
            <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
            <AllowMulticast>true</AllowMulticast>
        </General>
        <Internal>
            <MinimumSocketReceiveBufferSize>10MB</MinimumSocketReceiveBufferSize>
        </Internal>
        <Tracing>
            <Verbosity>warning</Verbosity>
        </Tracing>
    </Domain>
</CycloneDDS>
EOF

# 在 ~/.bashrc 中添加
echo 'export CYCLONEDDS_URI=file://$HOME/.ros/cyclonedds.xml' >> ~/.bashrc
source ~/.bashrc
```

## 编译与安装

### 1. 编译功能包

```bash
cd ~/humble_ws

# 清理之前的构建（可选）
rm -rf build install log

# 编译所有包
colcon build

# 或者只编译 bobac3_navigation 包
colcon build --packages-select bobac3_navigation

# 使用多线程加速编译
colcon build --parallel-workers 4
```

### 2. 使编译结果生效

```bash
source ~/humble_ws/install/setup.bash
```

### 3. 验证编译

```bash
# 检查包是否正确安装
ros2 pkg list | grep bobac3_navigation

# 查看包信息
ros2 pkg xml bobac3_navigation

# 列出包中的启动文件
ros2 pkg executables bobac3_navigation
```

## 验证安装

### 1. 检查依赖项

```bash
# 检查 ROS 2 版本
ros2 --version
# 应该显示: ros2 cli version: 0.18.x

# 检查所有 Nav2 包是否安装
ros2 pkg list | grep nav2

# 应该看到以下关键包：
# nav2_amcl - AMCL 定位
# nav2_bt_navigator - 行为树导航器
# nav2_controller - 控制器服务器
# nav2_planner - 规划器服务器
# nav2_smac_planner - SMAC 规划器（全局规划）
# nav2_mppi_controller - MPPI 控制器（局部规划）
# nav2_costmap_2d - 代价地图
# nav2_map_server - 地图服务器
# nav2_smoother - 路径平滑器
# nav2_velocity_smoother - 速度平滑器
# nav2_behaviors - 行为服务器
# nav2_lifecycle_manager - 生命周期管理器
# nav2_waypoint_follower - 路点跟随器

# 检查 RViz2
rviz2 --version

# 检查 bobac3_navigation 包
ros2 pkg list | grep bobac3_navigation

# 检查 Python 节点是否可执行
ros2 run bobac3_navigation laser_qos_relay.py --help
```

### 2. 测试启动文件

```bash
# 测试启动导航（不连接 Isaac Sim）
ros2 launch bobac3_navigation bobac3_navigation.launch.py

# 如果成功，你应该看到以下节点启动：
# [INFO] [map_server]: Map server started
# [INFO] [amcl]: AMCL node started
# [INFO] [planner_server]: Planner server started (SmacPlanner2D)
# [INFO] [controller_server]: Controller server started (MPPI Controller)
# [INFO] [bt_navigator]: Behavior tree navigator started
# [INFO] [smoother_server]: Smoother server started
# [INFO] [behavior_server]: Behavior server started
# [INFO] [velocity_smoother]: Velocity smoother started
# [INFO] [lifecycle_manager_localization]: Lifecycle manager started
# [INFO] [lifecycle_manager_navigation]: Lifecycle manager started
# [INFO] [rviz2]: RViz2 window opened

# 检查是否有错误信息
# 常见的警告（可以忽略）：
# - "Waiting for /odom" - 正常，等待 Isaac Sim 发布里程计
# - "Waiting for /laser_scan_fuse" - 正常，等待激光雷达数据
```

### 3. 检查话题和服务

在另一个终端中：

```bash
# 查看活动话题
ros2 topic list

# 应该看到以下关键话题：
# /map - 静态地图
# /cmd_vel - 速度控制命令
# /odom - 里程计数据（等待 Isaac Sim）
# /laser_scan_fuse - 激光雷达数据（等待 Isaac Sim）
# /amcl_pose - AMCL 定位位姿
# /particle_cloud - AMCL 粒子云
# /plan - 全局路径
# /local_plan - 局部路径
# /global_costmap/costmap - 全局代价地图
# /local_costmap/costmap - 局部代价地图
# /tf - TF 变换
# /tf_static - 静态 TF 变换

# 查看可用服务
ros2 service list | grep nav

# 应该看到：
# /navigate_to_pose - 导航到目标位姿
# /navigate_through_poses - 通过多个位姿导航
# /reinitialize_global_localization - 重新初始化全局定位
# /global_costmap/clear_entirely_global_costmap - 清除全局代价地图
# /local_costmap/clear_entirely_local_costmap - 清除局部代价地图

# 查看可用 Action
ros2 action list

# 应该看到：
# /navigate_to_pose
# /navigate_through_poses
# /follow_path
# /compute_path_to_pose
# /smooth_path
# /spin
# /backup
# /wait
```

### 4. 检查 TF 树

```bash
# 安装 TF 工具（如果尚未安装）
sudo apt install ros-humble-tf2-tools

# 查看 TF 树
ros2 run tf2_tools view_frames

# 这会生成一个 frames.pdf 文件
# 查看 PDF 文件
evince frames.pdf

# 或使用命令行查看 TF 关系
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link

# 预期的 TF 树结构：
# map
#  └─ odom (由 AMCL 发布)
#      └─ base_link (由 Isaac Sim/机器人发布)
#          └─ fuse_lidar_link (由机器人描述发布)
```

### 5. 验证关键组件

```bash
# 验证 AMCL 定位节点
ros2 node info /amcl
# 应该显示订阅的话题、发布的话题和参数

# 验证 MPPI Controller
ros2 node info /controller_server
# 检查是否使用 MPPIController 插件

# 验证 SMAC Planner
ros2 node info /planner_server
# 检查是否使用 SmacPlanner2D 插件

# 查看 AMCL 参数
ros2 param list /amcl
ros2 param get /amcl robot_model_type
# 应该返回: nav2_amcl::OmniMotionModel

# 查看控制器参数
ros2 param get /controller_server FollowPath.plugin
# 应该返回: nav2_mppi_controller::MPPIController

# 查看规划器参数
ros2 param get /planner_server GridBased.plugin
# 应该返回: nav2_smac_planner/SmacPlanner2D
```

## 常见问题

### 问题 1: 找不到包

**错误信息**: `Package 'bobac3_navigation' not found`

**解决方案**:
```bash
# 确保已 source 工作空间
source ~/humble_ws/install/setup.bash

# 重新编译
cd ~/humble_ws
colcon build --packages-select bobac3_navigation
source install/setup.bash
```

### 问题 2: 依赖项缺失

**错误信息**: `Could not find a package configuration file provided by "nav2_xxx"`

**解决方案**:
```bash
# 安装缺失的依赖
sudo apt install ros-humble-nav2-*

# 或使用 rosdep
cd ~/humble_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 问题 3: QoS 不兼容

**错误信息**: `New subscription discovered on topic '/scan', requesting incompatible QoS`

**解决方案**:
```bash
# 启动 QoS 中继节点
ros2 run bobac3_navigation laser_qos_relay.py

# 或在参数文件中修改 scan_topic
# 将 scan_topic: /laser_scan_fuse 改为 scan_topic: /laser_scan_fuse_best_effort
```

### 问题 4: 地图加载失败

**错误信息**: `Failed to load map from file`

**解决方案**:
```bash
# 检查地图文件是否存在
ls ~/humble_ws/src/navigation/bobac3_navigation/maps/

# 确保地图文件路径正确
# 在启动时指定完整路径
ros2 launch bobac3_navigation bobac3_navigation.launch.py \
    map:=/full/path/to/map.yaml
```

### 问题 5: RViz2 无法启动

**错误信息**: `Could not load RViz config file`

**解决方案**:
```bash
# 重新安装 RViz2 和插件
sudo apt install --reinstall ros-humble-rviz2
sudo apt install --reinstall ros-humble-nav2-rviz-plugins

# 或不使用配置文件启动
rviz2
```

### 问题 6: 编译错误

**错误信息**: `CMake Error` 或 `Could not find ament_cmake`

**解决方案**:
```bash
# 确保已安装构建工具
sudo apt install ros-humble-ament-cmake
sudo apt install python3-colcon-common-extensions

# 清理并重新编译
cd ~/humble_ws
rm -rf build install log
colcon build
```

### 问题 7: Isaac Sim 连接问题

**症状**: 导航启动但收不到传感器数据

**解决方案**:
```bash
# 检查 Isaac Sim 是否正在运行
# 检查话题是否发布
ros2 topic list
ros2 topic echo /laser_scan_fuse --once

# 确保 use_sim_time 参数正确
ros2 launch bobac3_navigation bobac3_navigation.launch.py use_sim_time:=True

# 检查时钟话题
ros2 topic echo /clock
```

### 问题 8: 权限问题

**错误信息**: `Permission denied`

**解决方案**:
```bash
# 给脚本添加执行权限
chmod +x ~/humble_ws/src/navigation/bobac3_navigation/scripts/*.py

# 重新编译
cd ~/humble_ws
colcon build --packages-select bobac3_navigation
```

## 性能优化建议

### 1. 使用 Cyclone DDS

如前所述，Cyclone DDS 通常比默认的 Fast DDS 性能更好。

### 2. 调整 QoS 设置

根据网络条件和应用需求调整 QoS 策略。

### 3. 优化参数配置

根据机器人和环境特性选择合适的参数配置文件：
- 开阔环境：使用 `bobac3_navigation_params_faster_best.yaml`
- 复杂环境：使用 `bobac3_navigation_params_slow_better.yaml`

### 4. 减少可视化开销

如果不需要可视化，禁用 RViz2：
```bash
ros2 launch bobac3_navigation bobac3_navigation.launch.py rviz:=False
```

## 更新和维护

### 更新 ROS 2 包

```bash
sudo apt update
sudo apt upgrade
```

### 更新工作空间

```bash
cd ~/humble_ws
git pull  # 如果使用 Git
colcon build
source install/setup.bash
```

### 清理构建文件

```bash
cd ~/humble_ws
rm -rf build install log
colcon build
```

## 卸载

如果需要卸载 bobac3_navigation：

```bash
# 删除包目录
rm -rf ~/humble_ws/src/navigation/bobac3_navigation

# 重新编译工作空间
cd ~/humble_ws
colcon build

# 或完全删除工作空间
rm -rf ~/humble_ws
```

## 获取帮助

如果遇到本文档未涵盖的问题：

1. 查看 [Nav2 官方文档](https://navigation.ros.org/)
2. 访问 [ROS 2 Humble 文档](https://docs.ros.org/en/humble/)
3. 查看 [Isaac Sim 论坛](https://forums.developer.nvidia.com/c/omniverse/simulation/69)
4. 提交 Issue 到项目仓库

## 附录：完整安装脚本

以下是一个自动化安装脚本，可以一键完成大部分安装步骤：

```bash
#!/bin/bash

# bobac3_navigation 自动安装脚本

set -e

echo "开始安装 bobac3_navigation 依赖..."

# 更新系统
sudo apt update
sudo apt upgrade -y

# 安装 ROS 2 Humble（如果尚未安装）
if ! command -v ros2 &> /dev/null; then
    echo "安装 ROS 2 Humble..."
    sudo apt install software-properties-common -y
    sudo add-apt-repository universe -y
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
    sudo apt install ros-humble-desktop -y
fi

# 安装 Nav2 和依赖
echo "安装 Nav2 和相关依赖..."
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-* \
    ros-humble-rviz2 \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-joint-state-publisher \
    ros-humble-robot-localization \
    ros-humble-slam-toolbox \
    ros-humble-rmw-cyclonedds-cpp

# 安装构建工具
echo "安装构建工具..."
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip

# 初始化 rosdep
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

# 配置环境变量
echo "配置环境变量..."
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi

if ! grep -q "source ~/humble_ws/install/setup.bash" ~/.bashrc; then
    echo "source ~/humble_ws/install/setup.bash" >> ~/.bashrc
fi

if ! grep -q "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" ~/.bashrc; then
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
fi

# 创建工作空间
echo "创建工作空间..."
mkdir -p ~/humble_ws/src

echo "安装完成！"
echo "请运行以下命令使环境变量生效："
echo "source ~/.bashrc"
echo ""
echo "然后将 bobac3_navigation 包复制到 ~/humble_ws/src/ 并运行："
echo "cd ~/humble_ws"
echo "rosdep install --from-paths src --ignore-src -r -y"
echo "colcon build"
echo "source install/setup.bash"
```

保存为 `install_bobac3_navigation.sh` 并运行：

```bash
chmod +x install_bobac3_navigation.sh
./install_bobac3_navigation.sh
```

---

**注意**: 本文档假设使用 Ubuntu 22.04 和 ROS 2 Humble。其他版本可能需要调整命令和包名。
