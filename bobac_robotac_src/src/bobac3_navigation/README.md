# bobac3_navigation

基于 ROS 2 Humble 和 Nav2 的 BOBAC3 全向移动机器人自主导航功能包，专为 NVIDIA Isaac Sim 仿真环境设计。

## 目录

- [功能概述](#功能概述)
- [系统架构](#系统架构)
- [包结构](#包结构)
- [核心组件详解](#核心组件详解)
- [数据流与处理逻辑](#数据流与处理逻辑)
- [启动参数详解](#启动参数详解)
- [参数配置方案](#参数配置方案)
- [完整启动流程](#完整启动流程)
- [代码实现细节](#代码实现细节)
- [故障排除](#故障排除)
- [依赖项](#依赖项)

## 功能概述

本功能包为 BOBAC3 全向移动机器人提供完整的自主导航解决方案，核心功能包括：

- **自主导航**: 基于 Nav2 导航栈的端到端自主导航系统
- **精确定位**: AMCL (Adaptive Monte Carlo Localization) 自适应蒙特卡洛定位
- **智能路径规划**: 
  - 全局规划器：SmacPlanner2D (A* 算法变体)
  - 局部规划器：MPPI Controller (Model Predictive Path Integral)
- **动态避障**: 基于代价地图的实时障碍物检测与规避
- **地图管理**: 支持静态地图加载与发布
- **可视化**: RViz2 实时导航状态监控
- **QoS 适配**: 激光雷达数据 QoS 策略转换，解决 Isaac Sim 兼容性问题
- **自定义行为树**: 针对全向机器人优化的无旋转导航行为

## 系统架构

```mermaid
graph TB
    subgraph Isaac["Isaac Sim 仿真环境"]
        Robot["BOBAC3 机器人<br/>麦克纳姆轮"]
        Lidar["激光雷达<br/>360度扫描"]
        Odom["里程计<br/>位姿估计"]
    end
    
    subgraph Nav["bobac3_navigation 功能包"]
        QoS["QoS 中继节点<br/>laser_qos_relay.py<br/>RELIABLE → BEST_EFFORT"]
        
        AMCL["定位模块 AMCL<br/>粒子滤波 300-1500<br/>输出: /amcl_pose, TF"]
        
        GlobalCostmap["全局代价地图<br/>静态层 + 膨胀层<br/>膨胀半径: 0.22m"]
        
        GlobalPlanner["全局路径规划器<br/>SmacPlanner2D<br/>算法: Hybrid A*/A*"]
        
        LocalCostmap["局部代价地图<br/>障碍物层 + 膨胀层<br/>25x25m, 分辨率 0.05m<br/>检测范围: 6m"]
        
        LocalPlanner["局部路径规划器<br/>MPPI Controller<br/>2000 采样轨迹<br/>7 个评价器"]
        
        VelSmoother["速度平滑器<br/>限制加速度和速度<br/>20 Hz"]
        
        BTNav["行为树导航器<br/>BT Navigator<br/>协调所有组件<br/>无旋转恢复模式"]
    end
    
    Lidar -->|/laser_scan_fuse<br/>RELIABLE QoS| QoS
    QoS -->|/laser_scan_fuse_best_effort<br/>BEST_EFFORT QoS| AMCL
    QoS -->|激光数据| LocalCostmap
    
    Odom -->|/odom| AMCL
    Odom -->|里程计| LocalCostmap
    
    AMCL -->|TF: map→odom<br/>/amcl_pose| GlobalPlanner
    AMCL -->|定位信息| LocalPlanner
    
    GlobalCostmap -->|代价地图| GlobalPlanner
    GlobalPlanner -->|/plan<br/>全局路径| LocalPlanner
    
    LocalCostmap -->|实时障碍物| LocalPlanner
    LocalPlanner -->|/local_plan<br/>局部轨迹| VelSmoother
    
    VelSmoother -->|/cmd_vel<br/>平滑速度命令| Robot
    
    BTNav -.->|协调| AMCL
    BTNav -.->|协调| GlobalPlanner
    BTNav -.->|协调| LocalPlanner
    
    style Isaac fill:#e1f5ff
    style Nav fill:#fff4e1
    style Robot fill:#ffcccb
    style BTNav fill:#d4edda
```

## 包结构

```
bobac3_navigation/
├── behavior_trees/                    # 行为树配置文件
│   └── navigate_to_pose_no_spin.xml  # 自定义无旋转导航行为树
├── launch/                            # 启动文件
│   └── bobac3_navigation.launch.py   # 主启动文件
├── maps/                              # 地图文件
│   ├── map.yaml                       # 地图元数据配置
│   └── map.png                        # 地图图像文件
├── params/                            # 参数配置文件
│   ├── bobac3_navigation_params.yaml              # 标准配置（平衡）
│   ├── bobac3_navigation_params_faster_best.yaml  # 快速配置
│   ├── bobac3_navigation_params_slow_better.yaml  # 精确配置
│   ├── bobac3_navigation_params_fast_bester.yaml  # 极速配置
│   └── qos_overrides.yaml                         # QoS 覆盖配置
├── rviz2/                             # RViz2 配置文件
│   └── bobac3_navigation.rviz        # 导航可视化配置
├── scripts/                           # Python 脚本
│   └── laser_qos_relay.py            # QoS 中继节点
├── CMakeLists.txt                     # CMake 构建配置
├── package.xml                        # ROS 2 包清单
└── README.md                          # 本文档
```

## 核心组件详解

### 1. 定位模块 (AMCL)

**功能**: 使用自适应蒙特卡洛定位算法估计机器人在地图中的位置

**关键参数**:
- `robot_model_type: "nav2_amcl::OmniMotionModel"` - 全向运动模型，适配 BOBAC3 的麦克纳姆轮
- `min_particles: 300` / `max_particles: 1500` - 粒子滤波器粒子数量范围
- `laser_model_type: "likelihood_field"` - 激光雷达观测模型
- `max_beams: 360` - 使用的激光束数量（1度分辨率）
- `update_min_d: 0.15` - 触发更新的最小移动距离（米）
- `update_min_a: 0.1` - 触发更新的最小旋转角度（弧度）

**输入数据**:
- `/laser_scan_fuse` - 激光雷达扫描数据
- `/odom` - 里程计数据
- `/map` - 静态地图

**输出数据**:
- `/amcl_pose` - 估计的机器人位姿
- `/particle_cloud` - 粒子云可视化
- TF 变换: `map` → `odom`

**算法流程**:
1. 接收里程计数据，预测粒子位置
2. 接收激光雷达数据，计算每个粒子的权重
3. 根据权重重采样粒子
4. 估计机器人位姿并发布 TF 变换

### 2. 全局路径规划器 (SmacPlanner2D)

**功能**: 在全局代价地图上规划从起点到终点的最优路径

**关键参数**:
- `plugin: "nav2_smac_planner/SmacPlanner2D"` - 使用 SMAC 2D 规划器
- `tolerance: 0.25` - 目标容差（米）
- `max_planning_time: 10.0` - 最大规划时间（秒）
- `allow_unknown: true` - 允许路径穿过未知区域
- `smooth_path: true` - 启用路径平滑
- `cost_travel_multiplier: 2.0` - 代价地图权重乘数

**算法**: 基于 A* 的搜索算法，考虑代价地图信息

**输出**: `/plan` - 全局路径（nav_msgs/Path）

### 3. 局部路径规划器 (MPPI Controller)

**功能**: 基于模型预测路径积分控制算法，生成局部轨迹并输出速度命令

**关键参数**:
- `plugin: "nav2_mppi_controller::MPPIController"`
- `time_steps: 50` - 预测时间步数
- `model_dt: 0.05` - 模型时间步长（秒）
- `batch_size: 2000` - 采样轨迹数量
- `motion_model: "Omni"` - 全向运动模型
- 速度限制:
  - `vx_max: 1.25` / `vx_min: -1.25` - X 方向速度（m/s）
  - `vy_max: 1.25` / `vy_min: -1.25` - Y 方向速度（m/s）
  - `wz_max: 0.3` - 角速度（rad/s）

**评价器 (Critics)**:
1. **ConstraintCritic** (权重 3.0): 约束违反惩罚
2. **ObstaclesCritic** (权重 50.0): 障碍物碰撞惩罚，碰撞边距 0.35m
3. **CostCritic** (权重 3.5): 代价地图代价惩罚
4. **PathAlignCritic** (权重 5.0): 路径对齐奖励
5. **PathFollowCritic** (权重 20.0): 路径跟随奖励
6. **GoalCritic** (权重 20.0): 目标距离奖励
7. **GoalAngleCritic** (权重 10.0): 目标角度奖励

**算法流程**:
1. 采样 2000 条候选轨迹
2. 对每条轨迹计算 7 个评价器的加权总分
3. 选择得分最高的轨迹
4. 提取第一个时间步的速度命令并发布

### 4. 代价地图系统

#### 全局代价地图
- **尺寸**: 根据地图文件自动调整
- **分辨率**: 0.05m
- **更新频率**: 5 Hz
- **发布频率**: 2 Hz
- **层级**:
  - `static_layer`: 从地图文件加载静态障碍物
  - `inflation_layer`: 障碍物膨胀，半径 0.22m

#### 局部代价地图
- **尺寸**: 25m × 25m
- **分辨率**: 0.05m
- **更新频率**: 20 Hz
- **发布频率**: 20 Hz
- **类型**: 滚动窗口（随机器人移动）
- **层级**:
  - `front_rplidar_obstacle_layer`: 实时激光雷达障碍物检测
    - 最大障碍物范围: 6.0m
    - 光线追踪范围: 8.0m
  - `inflation_layer`: 障碍物膨胀，半径 0.21m

**机器人足迹**: 正方形 0.38m × 0.38m（边长从中心到角的距离 0.19m）

### 5. 速度平滑器 (Velocity Smoother)

**功能**: 对控制器输出的速度命令进行平滑处理，限制加速度

**关键参数**:
- `smoothing_frequency: 20.0` - 平滑频率（Hz）
- `max_velocity: [1.5, 1.5, 0.25]` - 最大速度 [vx, vy, wz]
- `min_velocity: [-1.5, -1.5, -0.25]` - 最小速度
- `max_accel: [4.0, 4.0, 0.6]` - 最大加速度
- `max_decel: [-4.0, -4.0, -0.6]` - 最大减速度
- `deadband_velocity: [0.01, 0.01, 0.03]` - 死区速度

### 6. 行为树导航器 (BT Navigator)

**功能**: 使用行为树协调所有导航组件，处理导航失败和恢复

**自定义行为树** ([navigate_to_pose_no_spin.xml](behavior_trees/navigate_to_pose_no_spin.xml)):

```mermaid
graph TD
    A[NavigateRecovery<br/>重试 6 次] --> B[NavigateWithReplanning<br/>管道序列]
    A --> C[RecoveryFallback<br/>恢复行为]
    
    B --> D[RateController<br/>1 Hz 限流]
    B --> E[RecoveryNode<br/>路径跟随恢复]
    
    D --> F[RecoveryNode<br/>路径规划恢复]
    F --> G[ComputePathToPose<br/>计算全局路径]
    F --> H[ClearEntireCostmap<br/>失败时清除全局代价地图]
    
    E --> I[FollowPath<br/>跟随路径]
    E --> J[ClearEntireCostmap<br/>失败时清除局部代价地图]
    
    C --> K[GoalUpdated<br/>检查目标更新]
    C --> L[RoundRobin<br/>轮询恢复动作]
    
    L --> M[Sequence<br/>清除动作]
    L --> N[Wait<br/>等待 5 秒]
    L --> O[BackUp<br/>后退 0.3m]
    
    M --> P[ClearEntireCostmap<br/>清除局部]
    M --> Q[ClearEntireCostmap<br/>清除全局]
    
    style A fill:#ffcccc
    style B fill:#ccffcc
    style C fill:#ffeecc
    style G fill:#cce5ff
    style I fill:#cce5ff
```

**特点**: 移除了默认的 Spin（原地旋转）恢复行为，因为全向机器人不需要旋转即可移动

### 7. QoS 中继节点 (laser_qos_relay.py)

**功能**: 解决 Isaac Sim 和 Nav2 之间的 QoS 策略不兼容问题

**代码实现** ([scripts/laser_qos_relay.py](scripts/laser_qos_relay.py)):
```python
class LaserQoSRelay(Node):
    def __init__(self):
        # 订阅 RELIABLE QoS (匹配 Isaac Sim)
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # 发布 BEST_EFFORT QoS (匹配 Nav2 costmap)
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.subscription = self.create_subscription(
            LaserScan, '/laser_scan_fuse', self.laser_callback, sub_qos)
        
        self.publisher = self.create_publisher(
            LaserScan, '/laser_scan_fuse_best_effort', pub_qos)
    
    def laser_callback(self, msg):
        self.publisher.publish(msg)  # 直接转发消息
```

**问题背景**: Isaac Sim 发布的激光雷达数据使用 RELIABLE QoS，而 Nav2 的代价地图默认订阅 BEST_EFFORT QoS，导致数据无法匹配

**解决方案**: 中继节点订阅 RELIABLE 数据并重新发布为 BEST_EFFORT

## 数据流与处理逻辑

### 数据流图

```mermaid
sequenceDiagram
    participant IS as Isaac Sim
    participant QoS as QoS 中继
    participant AMCL as AMCL 定位
    participant GC as 全局代价地图
    participant GP as 全局规划器
    participant LC as 局部代价地图
    participant LP as 局部规划器
    participant VS as 速度平滑器
    participant BT as 行为树导航器
    
    Note over IS: 仿真环境运行
    IS->>QoS: /laser_scan_fuse (RELIABLE, ~20Hz)
    IS->>AMCL: /odom (~50Hz)
    
    QoS->>AMCL: /laser_scan_fuse_best_effort (BEST_EFFORT)
    QoS->>LC: 激光雷达数据
    
    AMCL->>AMCL: 粒子滤波定位
    AMCL->>GP: TF: map→odom + /amcl_pose (~5Hz)
    AMCL->>LP: 机器人位姿
    
    Note over BT: 用户发送导航目标
    BT->>GP: 请求全局路径规划
    
    GC->>GP: 全局代价地图
    GP->>GP: A* 搜索 (0.5-2s)
    GP->>LP: /plan (全局路径, ~1Hz)
    
    LC->>LC: 更新障碍物 (20Hz)
    LC->>LP: 局部代价地图
    
    loop 控制循环 (30Hz)
        LP->>LP: MPPI 采样 2000 轨迹
        LP->>LP: 评价器打分
        LP->>VS: 最优速度命令
        VS->>VS: 加速度限制 (20Hz)
        VS->>IS: /cmd_vel (平滑速度)
    end
    
    IS->>IS: 执行速度命令
    IS->>AMCL: 更新 /odom
```

### 导航处理流程

1. **初始化阶段**:
   - 加载地图文件 (map.yaml, map.png)
   - 启动 Map Server 发布地图到 `/map` 话题
   - 启动 AMCL，设置初始位姿 `(-3.783, 2.137, -1.5708)`
   - 初始化全局和局部代价地图
   - 启动所有导航服务器

2. **定位阶段**:
   - AMCL 接收激光雷达数据和里程计数据
   - 使用粒子滤波算法估计机器人在地图中的位置
   - 发布 `map` → `odom` TF 变换
   - 发布 `/amcl_pose` 和 `/particle_cloud`

3. **导航请求阶段** (用户发送目标位姿):
   - BT Navigator 接收 `/navigate_to_pose` Action 请求
   - 行为树开始执行 `NavigateRecovery` 节点

4. **全局路径规划阶段**:
   - SmacPlanner2D 在全局代价地图上搜索路径
   - 使用 A* 算法找到从当前位置到目标的最优路径
   - 路径平滑处理
   - 发布路径到 `/plan` 话题

5. **局部路径规划阶段** (循环执行，30 Hz):
   - MPPI Controller 接收全局路径
   - 采样 2000 条候选轨迹（时间跨度 2.5 秒）
   - 对每条轨迹评分:
     - 检查约束违反（速度、加速度限制）
     - 检查障碍物碰撞（基于局部代价地图）
     - 计算代价地图代价
     - 计算路径跟随误差
     - 计算目标距离和角度
   - 选择最优轨迹
   - 提取第一个控制命令

6. **速度平滑阶段** (20 Hz):
   - 接收 MPPI Controller 输出的速度命令
   - 检查加速度限制，必要时限制速度变化率
   - 应用死区过滤
   - 发布平滑后的速度命令到 `/cmd_vel`

7. **执行与监控阶段**:
   - Isaac Sim 接收 `/cmd_vel` 并驱动机器人
   - 局部代价地图持续更新障碍物信息
   - AMCL 持续更新机器人位姿
   - Progress Checker 监控导航进度
   - Goal Checker 检查是否到达目标

8. **恢复阶段** (如果导航失败):
   - 清除局部和全局代价地图
   - 等待 5 秒
   - 后退 0.3 米
   - 重新规划路径
   - 最多重试 6 次

### 数据传输格式

| 话题 | 消息类型 | 频率 | 说明 |
|------|---------|------|------|
| `/laser_scan_fuse` | sensor_msgs/LaserScan | ~20 Hz | 激光雷达扫描数据 |
| `/odom` | nav_msgs/Odometry | ~50 Hz | 里程计数据 |
| `/cmd_vel` | geometry_msgs/Twist | 20 Hz | 速度命令 |
| `/amcl_pose` | geometry_msgs/PoseWithCovarianceStamped | ~5 Hz | 定位位姿 |
| `/plan` | nav_msgs/Path | ~1 Hz | 全局路径 |
| `/local_plan` | nav_msgs/Path | 30 Hz | 局部路径 |
| `/particle_cloud` | nav2_msgs/ParticleCloud | ~5 Hz | AMCL 粒子云 |
| `/map` | nav_msgs/OccupancyGrid | 一次性 | 静态地图 |

## 启动参数详解

### Launch 文件参数

**文件**: [launch/bobac3_navigation.launch.py](launch/bobac3_navigation.launch.py)

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `map` | string | `<share_path>/maps/map.yaml` | 地图文件的完整路径，包含地图元数据 |
| `params_file` | string | `<share_path>/params/bobac3_navigation_params.yaml` | Nav2 参数配置文件的完整路径 |
| `use_sim_time` | bool | `True` | 是否使用仿真时间。True: 使用 Isaac Sim 时钟；False: 使用系统时钟 |
| `rviz` | bool | `True` | 是否启动 RViz2 可视化工具 |
| `rviz_config` | string | `<share_path>/rviz2/bobac3_navigation.rviz` | RViz2 配置文件的完整路径 |

### Launch 文件实现逻辑

```python
def generate_launch_description():
    # 1. 获取包的安装路径
    share_path = get_package_share_directory("bobac3_navigation")
    
    # 2. 声明启动参数
    declare_map_arg = DeclareLaunchArgument("map", default_value=...)
    declare_params_file_arg = DeclareLaunchArgument("params_file", ...)
    # ... 其他参数
    
    # 3. 包含 Nav2 定位启动文件
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_localization_launch]),
        launch_arguments={
            "map": map_dir,
            "params_file": param_dir,
            "use_sim_time": use_sim_time,
            # 启动 map_server 和 amcl
        }.items()
    )
    
    # 4. 包含 Nav2 导航启动文件
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_navigation_launch]),
        launch_arguments={
            "params_file": param_dir,
            "use_sim_time": use_sim_time,
            # 启动 planner_server, controller_server, bt_navigator 等
        }.items()
    )
    
    # 5. 启动 RViz2 节点（条件启动）
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        condition=conditions.IfCondition(rviz)
    )
    
    # 6. 返回启动描述
    return LaunchDescription([
        declare_map_arg,
        declare_params_file_arg,
        declare_use_sim_time_arg,
        declare_rviz_arg,
        declare_rviz_config_arg,
        localization,  # 定位模块
        navigation,    # 导航模块
        rviz2,         # 可视化
    ])
```

**启动的节点**:
1. `map_server` - 地图服务器
2. `amcl` - 定位节点
3. `planner_server` - 全局规划服务器
4. `controller_server` - 局部规划服务器
5. `bt_navigator` - 行为树导航器
6. `smoother_server` - 路径平滑服务器
7. `behavior_server` - 行为服务器（spin, backup 等）
8. `velocity_smoother` - 速度平滑器
9. `lifecycle_manager` (x2) - 生命周期管理器（定位和导航各一个）
10. `rviz2` - 可视化工具（可选）

## 参数配置方案

本功能包提供 4 种预配置的参数方案，适用于不同的应用场景：

### 1. 标准配置 (`bobac3_navigation_params.yaml`)

**适用场景**: 大多数室内导航场景，平衡速度和精度

**关键参数**:
- 最大速度: vx=1.25 m/s, vy=1.25 m/s, wz=0.3 rad/s
- MPPI 批次大小: 2000
- 控制器频率: 30 Hz
- 局部代价地图更新: 20 Hz
- 障碍物检测范围: 6.0m
- 膨胀半径: 0.21m (局部), 0.22m (全局)

**特点**:
- 响应速度适中
- 避障安全性高
- 路径跟随精度好
- CPU 占用适中

### 2. 快速配置 (`bobac3_navigation_params_faster_best.yaml`)

**适用场景**: 开阔环境，障碍物较少，需要快速到达目标

**关键参数调整**:
- 最大速度: vx=1.5 m/s, vy=1.5 m/s, wz=0.4 rad/s
- 加速度限制: 5.0 m/s²
- 目标容差: 0.3m (更宽松)
- 路径跟随权重降低

**特点**:
- 移动速度更快
- 路径跟随容差更大
- 适合直线距离较长的场景
- 可能在狭窄空间表现不佳

### 3. 精确配置 (`bobac3_navigation_params_slow_better.yaml`)

**适用场景**: 复杂环境，障碍物密集，需要精确导航

**关键参数调整**:
- 最大速度: vx=0.8 m/s, vy=0.8 m/s, wz=0.2 rad/s
- 目标容差: 0.03m (更严格)
- 障碍物权重增加
- 路径跟随权重增加
- AMCL 粒子数增加

**特点**:
- 移动速度较慢
- 定位精度更高
- 避障更保守
- 适合狭窄通道和精确停靠

### 4. 极速配置 (`bobac3_navigation_params_fast_bester.yaml`)

**适用场景**: 简单环境，无障碍物，追求最快速度

**关键参数调整**:
- 最大速度: vx=2.0 m/s, vy=2.0 m/s, wz=0.5 rad/s
- 加速度限制: 6.0 m/s²
- 代价地图更新频率降低
- 路径规划容差增大

**特点**:
- 最快移动速度
- 最低 CPU 占用
- 安全性降低
- 仅适合测试或空旷环境

### 参数对比表

| 配置 | 最大线速度 | 最大角速度 | 目标容差 | 适用场景 | CPU 占用 |
|------|-----------|-----------|---------|---------|---------|
| 标准 | 1.25 m/s | 0.3 rad/s | 0.05m | 通用室内 | 中等 |
| 快速 | 1.5 m/s | 0.4 rad/s | 0.3m | 开阔环境 | 中等 |
| 精确 | 0.8 m/s | 0.2 rad/s | 0.03m | 复杂环境 | 较高 |
| 极速 | 2.0 m/s | 0.5 rad/s | 0.5m | 空旷环境 | 较低 |

## 完整启动流程

### 前置条件

1. **ROS 2 Humble 环境已安装**
2. **Nav2 导航栈已安装**
3. **Isaac Sim 已启动并加载 BOBAC3 机器人模型**
4. **功能包已编译**

### 步骤 1: 编译功能包

```bash
# 进入工作空间
cd /root/humble_ws

# 编译 bobac3_navigation 包
colcon build --packages-select bobac3_navigation

# 加载环境变量
source install/setup.bash
```

### 步骤 2: 启动 Isaac Sim（在另一个终端）

```bash
# 启动 Isaac Sim 并加载机器人模型
# 确保以下话题正在发布：
# - /laser_scan_fuse (激光雷达数据)
# - /odom (里程计数据)
# 确保 Isaac Sim 订阅 /cmd_vel (速度命令)
```

### 步骤 3: 启动导航系统

#### 方式 1: 标准启动（推荐）

```bash
ros2 launch bobac3_navigation bobac3_navigation.launch.py
```

这将启动：
- Map Server (地图服务器)
- AMCL (定位节点)
- Planner Server (全局规划器)
- Controller Server (局部规划器)
- BT Navigator (行为树导航器)
- Smoother Server (路径平滑器)
- Behavior Server (行为服务器)
- Velocity Smoother (速度平滑器)
- Lifecycle Managers (生命周期管理器)
- RViz2 (可视化工具)

#### 方式 2: 使用自定义地图

```bash
ros2 launch bobac3_navigation bobac3_navigation.launch.py \
    map:=/path/to/your/map.yaml
```

#### 方式 3: 使用不同的参数配置

```bash
# 快速模式（速度优先）
ros2 launch bobac3_navigation bobac3_navigation.launch.py \
    params_file:=/root/humble_ws/src/navigation/bobac3_navigation/params/bobac3_navigation_params_faster_best.yaml

# 精确模式（精度优先）
ros2 launch bobac3_navigation bobac3_navigation.launch.py \
    params_file:=/root/humble_ws/src/navigation/bobac3_navigation/params/bobac3_navigation_params_slow_better.yaml

# 极速模式（最快速度）
ros2 launch bobac3_navigation bobac3_navigation.launch.py \
    params_file:=/root/humble_ws/src/navigation/bobac3_navigation/params/bobac3_navigation_params_fast_bester.yaml
```

#### 方式 4: 不启动 RViz2（节省资源）

```bash
ros2 launch bobac3_navigation bobac3_navigation.launch.py rviz:=False
```

#### 方式 5: 使用真实机器人（非仿真）

```bash
ros2 launch bobac3_navigation bobac3_navigation.launch.py use_sim_time:=False
```

#### 方式 6: 组合参数

```bash
ros2 launch bobac3_navigation bobac3_navigation.launch.py \
    map:=/custom/path/map.yaml \
    params_file:=/custom/path/params.yaml \
    use_sim_time:=True \
    rviz:=True \
    rviz_config:=/custom/path/config.rviz
```

### 步骤 4: 启动 QoS 中继节点（如果需要）

如果遇到激光雷达数据 QoS 不兼容问题：

```bash
# 在新终端中
source /root/humble_ws/install/setup.bash
ros2 run bobac3_navigation laser_qos_relay.py
```

然后修改参数文件中的 `scan_topic` 为 `/laser_scan_fuse_best_effort`。

### 步骤 5: 验证系统状态

```bash
# 检查所有节点是否正常运行
ros2 node list

# 应该看到以下节点：
# /amcl
# /bt_navigator
# /controller_server
# /planner_server
# /smoother_server
# /behavior_server
# /velocity_smoother
# /map_server
# /lifecycle_manager_localization
# /lifecycle_manager_navigation
# /rviz2

# 检查话题
ros2 topic list

# 检查 TF 树
ros2 run tf2_tools view_frames
```

### 步骤 6: 设置初始位姿（如果需要）

在 RViz2 中：
1. 点击工具栏的 "2D Pose Estimate" 按钮
2. 在地图上点击并拖动，设置机器人的初始位置和朝向
3. 观察粒子云是否收敛到正确位置

或使用命令行：

```bash
ros2 topic pub --once /initialpose geometry_msgs/PoseWithCovarianceStamped \
'{
  header: {frame_id: "map"},
  pose: {
    pose: {
      position: {x: -3.783, y: 2.137, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: -0.7071, w: 0.7071}
    }
  }
}'
```

### 步骤 7: 发送导航目标

#### 方法 1: 使用 RViz2（推荐）

1. 点击工具栏的 "Nav2 Goal" 或 "2D Goal Pose" 按钮
2. 在地图上点击并拖动，设置目标位置和朝向
3. 机器人将自动开始导航

#### 方法 2: 使用命令行

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
'{
  pose: {
    header: {frame_id: "map"},
    pose: {
      position: {x: 5.0, y: 3.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  }
}'
```

#### 方法 3: 使用 Python 脚本

```python
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

rclpy.init()
node = rclpy.create_node('navigation_client')
client = ActionClient(node, NavigateToPose, '/navigate_to_pose')

goal_msg = NavigateToPose.Goal()
goal_msg.pose.header.frame_id = 'map'
goal_msg.pose.pose.position.x = 5.0
goal_msg.pose.pose.position.y = 3.0
goal_msg.pose.pose.orientation.w = 1.0

client.wait_for_server()
client.send_goal_async(goal_msg)
rclpy.spin(node)
```

### 步骤 8: 监控导航状态

```bash
# 查看机器人位姿
ros2 topic echo /amcl_pose

# 查看速度命令
ros2 topic echo /cmd_vel

# 查看全局路径
ros2 topic echo /plan

# 查看局部路径
ros2 topic echo /local_plan

# 查看导航状态
ros2 action list
ros2 action info /navigate_to_pose
```

### 完整启动脚本示例

创建一个启动脚本 `start_navigation.sh`:

```bash
#!/bin/bash

# 设置环境变量
source /opt/ros/humble/setup.bash
source /root/humble_ws/install/setup.bash

# 启动导航系统
ros2 launch bobac3_navigation bobac3_navigation.launch.py \
    map:=/root/humble_ws/src/navigation/bobac3_navigation/maps/map.yaml \
    params_file:=/root/humble_ws/src/navigation/bobac3_navigation/params/bobac3_navigation_params.yaml \
    use_sim_time:=True \
    rviz:=True
```

使用方法：

```bash
chmod +x start_navigation.sh
./start_navigation.sh
```

## 代码实现细节

### 1. Launch 文件实现 ([launch/bobac3_navigation.launch.py](launch/bobac3_navigation.launch.py))

**核心逻辑**:
- 使用 `IncludeLaunchDescription` 包含 Nav2 官方的 `localization_launch.py` 和 `navigation_launch.py`
- 通过 `launch_arguments` 传递参数到子启动文件
- 使用 `DeclareLaunchArgument` 声明可配置参数
- 使用 `conditions.IfCondition` 实现条件启动（如 RViz2）

**关键代码段**:
```python
# 获取包路径
share_path = get_package_share_directory("bobac3_navigation")

# 包含定位启动文件
localization = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([nav2_localization_launch]),
    launch_arguments={
        "namespace": "",
        "use_namespace": "False",
        "autostart": "True",
        "use_composition": "False",
        "map": map_dir,
        "use_sim_time": use_sim_time,
        "params_file": param_dir,
    }.items(),
)
```

### 2. QoS 中继节点实现 ([scripts/laser_qos_relay.py](scripts/laser_qos_relay.py))

**设计模式**: 发布-订阅中继模式

**核心逻辑**:
1. 创建两个不同 QoS 策略的端点
2. 订阅端使用 RELIABLE QoS 匹配 Isaac Sim
3. 发布端使用 BEST_EFFORT QoS 匹配 Nav2
4. 回调函数直接转发消息，无数据处理

**关键代码段**:
```python
def __init__(self):
    super().__init__('laser_qos_relay')
    
    # 订阅 RELIABLE QoS
    sub_qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        durability=DurabilityPolicy.VOLATILE
    )
    
    # 发布 BEST_EFFORT QoS
    pub_qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        durability=DurabilityPolicy.VOLATILE
    )
    
    self.subscription = self.create_subscription(
        LaserScan, '/laser_scan_fuse', self.laser_callback, sub_qos)
    
    self.publisher = self.create_publisher(
        LaserScan, '/laser_scan_fuse_best_effort', pub_qos)

def laser_callback(self, msg):
    self.publisher.publish(msg)  # 零拷贝转发
```

**性能特点**:
- 低延迟：直接转发，无数据处理
- 低 CPU 占用：仅消息传递
- 内存效率：ROS 2 零拷贝机制

### 3. 行为树实现 ([behavior_trees/navigate_to_pose_no_spin.xml](behavior_trees/navigate_to_pose_no_spin.xml))

**设计理念**: 针对全向机器人优化，移除不必要的旋转行为

**行为树结构**:
```
RecoveryNode (重试 6 次)
├── PipelineSequence (主导航流程)
│   ├── RateController (1 Hz 限流)
│   │   └── RecoveryNode (路径规划恢复)
│   │       ├── ComputePathToPose (计算路径)
│   │       └── ClearEntireCostmap (失败时清除全局代价地图)
│   └── RecoveryNode (路径跟随恢复)
│       ├── FollowPath (跟随路径)
│       └── ClearEntireCostmap (失败时清除局部代价地图)
└── ReactiveFallback (恢复行为)
    ├── GoalUpdated (检查目标更新)
    └── RoundRobin (轮询恢复动作)
        ├── Sequence (清除动作)
        │   ├── ClearEntireCostmap (清除局部)
        │   └── ClearEntireCostmap (清除全局)
        ├── Wait (等待 5 秒)
        └── BackUp (后退 0.3m)
```

**与默认行为树的区别**:
- **移除**: Spin (原地旋转) 恢复行为
- **原因**: 全向机器人可以横向移动，不需要旋转来调整朝向
- **优势**: 减少不必要的旋转，提高导航效率

### 4. 参数文件结构 ([params/bobac3_navigation_params.yaml](params/bobac3_navigation_params.yaml))

**组织方式**: 按节点分组，每个节点有独立的参数命名空间

**主要节点配置**:
1. `amcl` - 定位参数
2. `bt_navigator` - 行为树参数
3. `controller_server` - 控制器参数
4. `planner_server` - 规划器参数
5. `local_costmap` - 局部代价地图参数
6. `global_costmap` - 全局代价地图参数
7. `velocity_smoother` - 速度平滑器参数
8. `behavior_server` - 行为服务器参数

**参数继承**: 所有节点都继承 `use_sim_time` 参数

## 话题、服务和 Action 接口

### 发布的话题

| 话题 | 消息类型 | 频率 | 说明 |
|------|---------|------|------|
| `/cmd_vel` | geometry_msgs/Twist | 20 Hz | 速度控制命令（发送到机器人） |
| `/plan` | nav_msgs/Path | ~1 Hz | 全局路径 |
| `/local_plan` | nav_msgs/Path | 30 Hz | 局部路径 |
| `/global_costmap/costmap` | nav_msgs/OccupancyGrid | 2 Hz | 全局代价地图 |
| `/local_costmap/costmap` | nav_msgs/OccupancyGrid | 20 Hz | 局部代价地图 |
| `/particle_cloud` | nav2_msgs/ParticleCloud | ~5 Hz | AMCL 粒子云 |
| `/amcl_pose` | geometry_msgs/PoseWithCovarianceStamped | ~5 Hz | 定位位姿估计 |
| `/laser_scan_fuse_best_effort` | sensor_msgs/LaserScan | ~20 Hz | QoS 转换后的激光雷达数据 |

### 订阅的话题

| 话题 | 消息类型 | 说明 |
|------|---------|------|
| `/laser_scan_fuse` | sensor_msgs/LaserScan | 激光雷达扫描数据（来自 Isaac Sim） |
| `/odom` | nav_msgs/Odometry | 里程计数据（来自 Isaac Sim） |
| `/map` | nav_msgs/OccupancyGrid | 静态地图（来自 map_server） |
| `/initialpose` | geometry_msgs/PoseWithCovarianceStamped | 初始位姿设置 |

### 提供的服务

| 服务 | 服务类型 | 说明 |
|------|---------|------|
| `/reinitialize_global_localization` | std_srvs/Empty | 重新初始化全局定位（粒子分散） |
| `/global_costmap/clear_entirely_global_costmap` | nav2_msgs/ClearEntireCostmap | 清除全局代价地图 |
| `/local_costmap/clear_entirely_local_costmap` | nav2_msgs/ClearEntireCostmap | 清除局部代价地图 |
| `/global_costmap/clear_around_global_costmap` | nav2_msgs/ClearCostmapAroundRobot | 清除机器人周围的全局代价地图 |
| `/local_costmap/clear_around_local_costmap` | nav2_msgs/ClearCostmapAroundRobot | 清除机器人周围的局部代价地图 |

### 提供的 Action

| Action | Action 类型 | 说明 |
|--------|------------|------|
| `/navigate_to_pose` | nav2_msgs/NavigateToPose | 导航到目标位姿 |
| `/navigate_through_poses` | nav2_msgs/NavigateThroughPoses | 通过多个位姿导航 |
| `/follow_path` | nav2_msgs/FollowPath | 跟随给定路径 |
| `/compute_path_to_pose` | nav2_msgs/ComputePathToPose | 计算到目标位姿的路径 |
| `/smooth_path` | nav2_msgs/SmoothPath | 平滑给定路径 |
| `/spin` | nav2_msgs/Spin | 原地旋转 |
| `/backup` | nav2_msgs/BackUp | 后退 |
| `/wait` | nav2_msgs/Wait | 等待 |

### TF 变换树

```mermaid
graph TD
    map["map<br/>全局地图坐标系"]
    odom["odom<br/>里程计坐标系"]
    base_link["base_link<br/>机器人中心"]
    lidar["fuse_lidar_link<br/>激光雷达"]
    
    map -->|AMCL 发布| odom
    odom -->|Isaac Sim/机器人发布| base_link
    base_link -->|机器人描述发布| lidar
    
    style map fill:#ffcccc
    style odom fill:#ccffcc
    style base_link fill:#cce5ff
    style lidar fill:#ffffcc
```

## 与 Isaac Sim 集成

### 集成架构

本功能包专为 NVIDIA Isaac Sim 仿真环境设计，实现了完整的仿真-导航闭环：

```mermaid
graph LR
    subgraph Isaac["Isaac Sim 仿真环境"]
        Robot["BOBAC3 机器人<br/>麦克纳姆轮"]
        Lidar["激光雷达<br/>360度扫描"]
        Odom["里程计<br/>位姿估计"]
        Physics["物理引擎<br/>碰撞检测"]
    end
    
    subgraph ROS2["ROS 2 Navigation"]
        Nav["Nav2 Stack<br/>AMCL + Planners<br/>Controllers + Costmaps"]
        BT["Behavior Tree<br/>导航协调"]
    end
    
    Lidar -->|/laser_scan_fuse| Nav
    Odom -->|/odom| Nav
    Nav -->|/cmd_vel| Robot
    Robot --> Physics
    Physics --> Robot
    
    Nav <-->|状态同步| BT
    
    Clock["/clock<br/>仿真时间"] -.->|时间同步| Nav
    Clock -.->|时间同步| BT
    
    style Isaac fill:#e1f5ff
    style ROS2 fill:#fff4e1
    style Clock fill:#ffe1e1
```

### 关键集成特性

1. **仿真时间同步**
   - 所有节点使用 `use_sim_time: True`
   - 订阅 Isaac Sim 的 `/clock` 话题
   - 确保传感器数据、控制命令和导航算法时间一致

2. **激光雷达数据格式**
   - Isaac Sim 发布标准 `sensor_msgs/LaserScan` 消息
   - 360 度扫描，1 度分辨率
   - 最大测距 10m
   - QoS 策略: RELIABLE

3. **QoS 策略兼容性**
   - Isaac Sim 默认使用 RELIABLE QoS
   - Nav2 costmap 默认期望 BEST_EFFORT QoS
   - 使用 `laser_qos_relay.py` 进行 QoS 转换
   - 或修改参数文件中的 QoS 设置

4. **全向移动机器人模型**
   - AMCL 使用 `OmniMotionModel`
   - MPPI Controller 使用 `Omni` 运动模型
   - 支持 X、Y、θ 三自由度独立控制
   - 适配麦克纳姆轮运动学

5. **坐标系对齐**
   - Isaac Sim: `base_link` 为机器人中心
   - ROS 2: 标准 REP-105 坐标系约定
   - TF 树: `map` → `odom` → `base_link` → `fuse_lidar_link`

### 典型工作流程

#### 1. 启动 Isaac Sim

```bash
# 启动 Isaac Sim 并加载场景
# 确保以下组件已配置：
# - BOBAC3 机器人模型
# - 激光雷达传感器 (发布到 /laser_scan_fuse)
# - 差分驱动控制器 (订阅 /cmd_vel)
# - 里程计发布器 (发布到 /odom)
# - 时钟发布器 (发布到 /clock)
```

#### 2. 启动导航系统

```bash
# 终端 1: 启动导航
source /root/humble_ws/install/setup.bash
ros2 launch bobac3_navigation bobac3_navigation.launch.py
```

#### 3. 验证数据流

```bash
# 终端 2: 检查话题
ros2 topic hz /laser_scan_fuse  # 应该 ~20 Hz
ros2 topic hz /odom             # 应该 ~50 Hz
ros2 topic hz /clock            # 应该 ~100 Hz

# 检查 TF 树
ros2 run tf2_tools view_frames
# 应该看到完整的 TF 树: map → odom → base_link → fuse_lidar_link
```

#### 4. 设置初始位姿

在 RViz2 中：
- 点击 "2D Pose Estimate"
- 在地图上点击机器人的实际位置
- 拖动设置朝向
- 观察粒子云收敛

#### 5. 发送导航目标

在 RViz2 中：
- 点击 "Nav2 Goal" 或 "2D Goal Pose"
- 在地图上点击目标位置
- 拖动设置目标朝向
- 机器人开始自主导航

#### 6. 监控导航过程

在 RViz2 中观察：
- 绿色路径: 全局规划路径
- 蓝色路径: 局部规划路径
- 彩色网格: 代价地图
- 红色粒子: AMCL 定位粒子
- 机器人模型: 实时位置

### 常见集成问题

#### 问题 1: 激光雷达数据不匹配

**症状**: 代价地图没有障碍物，或 AMCL 不更新

**原因**: QoS 策略不兼容

**解决方案**:
```bash
# 方法 1: 启动 QoS 中继节点
ros2 run bobac3_navigation laser_qos_relay.py

# 然后修改参数文件中的 scan_topic:
# scan_topic: /laser_scan_fuse_best_effort

# 方法 2: 修改参数文件中的 QoS 设置
# 在 local_costmap 的 scan 配置中添加:
# reliability: 1  # RELIABLE
```

#### 问题 2: 时间不同步

**症状**: TF 查找失败，"Transform timeout" 错误

**原因**: 节点未使用仿真时间

**解决方案**:
```bash
# 确保所有节点都设置了 use_sim_time: True
# 检查参数文件中所有节点的配置

# 验证时间源
ros2 topic echo /clock
```

#### 问题 3: 机器人不移动

**症状**: 导航命令发送成功，但机器人静止

**原因**: Isaac Sim 未订阅 `/cmd_vel`

**解决方案**:
```bash
# 检查 /cmd_vel 是否有订阅者
ros2 topic info /cmd_vel

# 应该看到至少一个订阅者（Isaac Sim）
# 如果没有，检查 Isaac Sim 中的控制器配置
```

#### 问题 4: 定位漂移

**症状**: AMCL 位姿与实际位置不符

**原因**: 地图与环境不匹配，或初始位姿错误

**解决方案**:
```bash
# 1. 重新设置初始位姿（在 RViz2 中）
# 2. 检查地图是否与 Isaac Sim 场景匹配
# 3. 调整 AMCL 参数（增加粒子数）
# 4. 使用全局定位重新初始化
ros2 service call /reinitialize_global_localization std_srvs/srv/Empty
```

## 故障排除

### 1. 激光雷达数据不匹配

**症状**:
- 局部代价地图没有显示障碍物
- AMCL 不更新位姿
- 终端显示 "QoS mismatch" 警告

**诊断**:
```bash
# 检查激光雷达话题的 QoS 设置
ros2 topic info /laser_scan_fuse -v

# 检查是否有订阅者
ros2 topic info /laser_scan_fuse
```

**解决方案 1**: 启动 QoS 中继节点
```bash
ros2 run bobac3_navigation laser_qos_relay.py
```
然后修改参数文件中的 `scan_topic` 为 `/laser_scan_fuse_best_effort`。

**解决方案 2**: 修改参数文件
在 `local_costmap` 的 `scan` 配置中添加：
```yaml
scan:
  topic: /laser_scan_fuse
  reliability: 1  # RELIABLE
```

### 2. 定位不准确或漂移

**症状**:
- 机器人在地图上的位置与实际不符
- 粒子云分散，不收敛
- 导航路径偏离预期

**诊断**:
```bash
# 查看粒子云分布
ros2 topic echo /particle_cloud

# 查看定位位姿
ros2 topic echo /amcl_pose

# 检查 TF 变换
ros2 run tf2_ros tf2_echo map base_link
```

**解决方案**:

1. **重新设置初始位姿**:
   - 在 RViz2 中使用 "2D Pose Estimate" 工具
   - 或使用命令行发布初始位姿

2. **全局定位重新初始化**:
```bash
ros2 service call /reinitialize_global_localization std_srvs/srv/Empty
```

3. **调整 AMCL 参数**:
   - 增加粒子数量: `max_particles: 2000`
   - 减小运动噪声: `alpha1-5: 0.1`
   - 增加激光束数量: `max_beams: 720`

4. **检查地图匹配**:
   - 确保地图文件与 Isaac Sim 场景一致
   - 检查地图分辨率和原点设置

### 3. 路径规划失败

**症状**:
- 终端显示 "Failed to create plan"
- 机器人不移动
- RViz2 中没有显示全局路径

**诊断**:
```bash
# 检查规划器状态
ros2 node info /planner_server

# 手动调用规划服务
ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
'{
  goal: {
    pose: {
      header: {frame_id: "map"},
      pose: {position: {x: 5.0, y: 3.0, z: 0.0}, orientation: {w: 1.0}}
    }
  }
}'
```

**解决方案**:

1. **检查目标点可达性**:
   - 确保目标点在自由空间内（白色区域）
   - 目标点不能在障碍物上或未知区域

2. **清除代价地图**:
```bash
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap
```

3. **调整规划器参数**:
   - 增加规划时间: `max_planning_time: 20.0`
   - 允许未知区域: `allow_unknown: true`
   - 增加目标容差: `tolerance: 0.5`

4. **检查代价地图**:
```bash
# 查看全局代价地图
ros2 topic echo /global_costmap/costmap --once
```

### 4. 机器人震荡或不稳定

**症状**:
- 机器人左右摇摆
- 速度命令频繁变化
- 无法平滑跟随路径

**诊断**:
```bash
# 查看速度命令
ros2 topic echo /cmd_vel

# 查看局部路径
ros2 topic echo /local_plan
```

**解决方案**:

1. **降低速度限制**:
```yaml
velocity_smoother:
  max_velocity: [0.8, 0.8, 0.2]  # 降低最大速度
```

2. **调整 MPPI 控制器参数**:
```yaml
FollowPath:
  vx_max: 0.8
  vy_max: 0.8
  wz_max: 0.2
  temperature: 0.35  # 增加温度，减少抖动
  PathFollowCritic:
    cost_weight: 30.0  # 增加路径跟随权重
```

3. **增加速度平滑**:
```yaml
velocity_smoother:
  max_accel: [2.0, 2.0, 0.4]  # 降低加速度限制
  smoothing_frequency: 30.0   # 增加平滑频率
```

4. **调整目标容差**:
```yaml
stopped_goal_checker:
  xy_goal_tolerance: 0.1  # 增加位置容差
  yaw_goal_tolerance: 0.1  # 增加角度容差
```

### 5. 机器人卡住不动

**症状**:
- 导航命令已发送，但机器人静止
- 局部路径显示，但无速度输出
- 终端显示 "Robot is stuck"

**诊断**:
```bash
# 检查控制器状态
ros2 node info /controller_server

# 查看速度命令
ros2 topic echo /cmd_vel

# 查看局部代价地图
rviz2  # 观察局部代价地图中的障碍物
```

**解决方案**:

1. **清除局部代价地图**:
```bash
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap
```

2. **手动恢复行为**:
```bash
# 后退
ros2 action send_goal /backup nav2_msgs/action/BackUp "{target: {x: -0.3}}"
```

3. **调整障碍物检测**:
```yaml
ObstaclesCritic:
  collision_margin_distance: 0.25  # 减小碰撞边距
  cost_weight: 40.0  # 降低障碍物权重
```

4. **检查机器人足迹**:
   - 确保足迹大小正确: `footprint: "[ [0.19, 0.19], [0.19, -0.19], [-0.19, -0.19], [-0.19, 0.19] ]"`
   - 足迹不应过大或过小

### 6. TF 变换错误

**症状**:
- 终端显示 "Transform timeout"
- "Could not transform from map to base_link"
- 导航无法启动

**诊断**:
```bash
# 查看 TF 树
ros2 run tf2_tools view_frames

# 检查特定变换
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link

# 查看 TF 话题
ros2 topic echo /tf
ros2 topic echo /tf_static
```

**解决方案**:

1. **检查时间同步**:
   - 确保所有节点使用 `use_sim_time: True`
   - 检查 `/clock` 话题是否发布

2. **增加变换容差**:
```yaml
amcl:
  transform_tolerance: 5.0  # 增加容差

controller_server:
  FollowPath:
    transform_tolerance: 0.5  # 增加容差
```

3. **检查 TF 发布**:
   - 确保 Isaac Sim 发布 `odom` → `base_link`
   - 确保 AMCL 发布 `map` → `odom`

### 7. RViz2 显示异常

**症状**:
- 地图不显示
- 机器人模型不显示
- 路径或代价地图不显示

**解决方案**:

1. **检查固定坐标系**:
   - 在 RViz2 中设置 Fixed Frame 为 `map`

2. **检查话题订阅**:
   - 确保各个显示项订阅了正确的话题
   - 检查 QoS 设置是否匹配

3. **重新加载配置**:
```bash
ros2 launch bobac3_navigation bobac3_navigation.launch.py \
    rviz_config:=/root/humble_ws/src/navigation/bobac3_navigation/rviz2/bobac3_navigation.rviz
```

### 8. 性能问题（高 CPU 占用）

**症状**:
- CPU 占用率过高（>80%）
- 导航响应延迟
- 仿真运行缓慢

**解决方案**:

1. **降低更新频率**:
```yaml
local_costmap:
  update_frequency: 10.0  # 从 20 降到 10
  publish_frequency: 5.0  # 从 20 降到 5

controller_server:
  controller_frequency: 20.0  # 从 30 降到 20
```

2. **减少 MPPI 采样数**:
```yaml
FollowPath:
  batch_size: 1000  # 从 2000 降到 1000
  time_steps: 30    # 从 50 降到 30
```

3. **减少 AMCL 粒子数**:
```yaml
amcl:
  max_particles: 1000  # 从 1500 降到 1000
  max_beams: 180       # 从 360 降到 180
```

4. **使用快速配置**:
```bash
ros2 launch bobac3_navigation bobac3_navigation.launch.py \
    params_file:=.../bobac3_navigation_params_fast_bester.yaml
```

### 调试工具和命令

```bash
# 查看所有节点
ros2 node list

# 查看节点详细信息
ros2 node info /bt_navigator

# 查看所有话题
ros2 topic list

# 查看话题频率
ros2 topic hz /cmd_vel

# 查看话题内容
ros2 topic echo /amcl_pose

# 查看所有服务
ros2 service list

# 调用服务
ros2 service call /reinitialize_global_localization std_srvs/srv/Empty

# 查看所有 Action
ros2 action list

# 查看 Action 状态
ros2 action info /navigate_to_pose

# 查看参数
ros2 param list /amcl
ros2 param get /amcl max_particles

# 设置参数（运行时）
ros2 param set /amcl max_particles 2000

# 查看日志
ros2 run rqt_console rqt_console

# 录制数据包
ros2 bag record -a

# 回放数据包
ros2 bag play <bag_file>
```

## 依赖项

### 系统依赖

- **操作系统**: Ubuntu 22.04 (Jammy)
- **ROS 版本**: ROS 2 Humble Hawksbill
- **Python 版本**: Python 3.10+

### ROS 2 包依赖

#### 核心依赖
- `ament_cmake` - CMake 构建工具
- `rclpy` - ROS 2 Python 客户端库
- `rclcpp` - ROS 2 C++ 客户端库

#### Nav2 导航栈
- `navigation2` - Nav2 元包
- `nav2_amcl` - 自适应蒙特卡洛定位
- `nav2_bringup` - Nav2 启动文件
- `nav2_bt_navigator` - 行为树导航器
- `nav2_costmap_2d` - 2D 代价地图
- `nav2_core` - Nav2 核心接口
- `nav2_dwb_controller` - DWB 控制器（未使用，但作为依赖）
- `nav2_mppi_controller` - MPPI 控制器（实际使用）
- `nav2_lifecycle_manager` - 生命周期管理器
- `nav2_map_server` - 地图服务器
- `nav2_behaviors` - 导航行为（spin, backup 等）
- `nav2_planner` - 规划器服务器
- `nav2_msgs` - Nav2 消息定义
- `nav2_navfn_planner` - NavFn 规划器（未使用）
- `nav2_smac_planner` - SMAC 规划器（实际使用）
- `nav2_rviz_plugins` - RViz2 插件
- `nav2_behavior_tree` - 行为树库
- `nav2_util` - Nav2 工具库
- `nav2_voxel_grid` - 体素网格
- `nav2_controller` - 控制器服务器
- `nav2_waypoint_follower` - 路点跟随器
- `nav2_smoother` - 路径平滑器
- `nav2_velocity_smoother` - 速度平滑器

#### 可视化
- `rviz2` - 3D 可视化工具
- `joint_state_publisher` - 关节状态发布器

#### 消息类型
- `sensor_msgs` - 传感器消息（LaserScan）
- `geometry_msgs` - 几何消息（Twist, Pose 等）
- `nav_msgs` - 导航消息（Path, Odometry, OccupancyGrid）
- `std_msgs` - 标准消息
- `tf2_ros` - TF2 变换库

#### 其他工具
- `pointcloud_to_laserscan` - 点云转激光扫描（可选）

### 仿真环境依赖

- **NVIDIA Isaac Sim** - 物理仿真环境
  - 版本: 2023.1.0 或更高
  - 需要 NVIDIA GPU（推荐 RTX 系列）
  - CUDA 11.8 或更高

### 安装依赖

#### 1. 安装 ROS 2 Humble

```bash
# 添加 ROS 2 仓库
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装 ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop -y
```

#### 2. 安装 Nav2

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup -y
```

#### 3. 安装其他依赖

```bash
sudo apt install \
    ros-humble-rviz2 \
    ros-humble-joint-state-publisher \
    ros-humble-tf2-tools \
    python3-colcon-common-extensions \
    -y
```

#### 4. 设置环境变量

```bash
# 添加到 ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### 5. 编译功能包

```bash
cd /root/humble_ws
colcon build --packages-select bobac3_navigation
source install/setup.bash
```

### 验证安装

```bash
# 检查 ROS 2 安装
ros2 --version

# 检查 Nav2 安装
ros2 pkg list | grep nav2

# 检查功能包安装
ros2 pkg list | grep bobac3_navigation

# 检查启动文件
ros2 launch bobac3_navigation --show-args bobac3_navigation.launch.py
```

## 性能指标

### 计算资源需求

| 组件 | CPU 占用 | 内存占用 | 备注 |
|------|---------|---------|------|
| AMCL | 5-10% | ~100 MB | 取决于粒子数量 |
| MPPI Controller | 15-25% | ~200 MB | 取决于批次大小 |
| Costmap (Local) | 10-15% | ~150 MB | 取决于更新频率 |
| Costmap (Global) | 2-5% | ~100 MB | 更新频率较低 |
| Planner Server | 5-10% | ~100 MB | 按需计算 |
| 总计 | 40-70% | ~650 MB | 标准配置 |

### 导航性能

| 指标 | 标准配置 | 快速配置 | 精确配置 |
|------|---------|---------|---------|
| 最大线速度 | 1.25 m/s | 1.5 m/s | 0.8 m/s |
| 最大角速度 | 0.3 rad/s | 0.4 rad/s | 0.2 rad/s |
| 定位精度 | ±5 cm | ±10 cm | ±3 cm |
| 路径规划时间 | 0.5-2 s | 0.3-1 s | 1-3 s |
| 控制器频率 | 30 Hz | 30 Hz | 30 Hz |
| 避障距离 | 0.35 m | 0.3 m | 0.4 m |

### 延迟分析

| 阶段 | 延迟 | 说明 |
|------|------|------|
| 传感器数据采集 | ~50 ms | Isaac Sim → ROS 2 |
| 定位更新 | ~100 ms | AMCL 处理 |
| 全局路径规划 | 500-2000 ms | 按需触发 |
| 局部路径规划 | ~33 ms | 30 Hz 控制循环 |
| 速度平滑 | ~50 ms | 20 Hz 平滑器 |
| 端到端延迟 | ~200 ms | 传感器到执行器 |

## 许可证

Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property and proprietary rights in and to this software, related documentation and any modifications thereto. Any use, reproduction, disclosure or distribution of this software and related documentation without an express license agreement from NVIDIA CORPORATION is strictly prohibited.

## 维护者

**Isaac Sim Maintainers**  
Email: isaac-sim-maintainers@nvidia.com

## 参考资源

### 官方文档
- [Nav2 官方文档](https://navigation.ros.org/) - Nav2 导航栈完整文档
- [Nav2 配置指南](https://navigation.ros.org/configuration/index.html) - 参数配置详解
- [Isaac Sim 文档](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html) - Isaac Sim 使用指南
- [ROS 2 Humble 文档](https://docs.ros.org/en/humble/index.html) - ROS 2 官方文档

### 社区资源
- [NVIDIA 开发者论坛](https://forums.developer.nvidia.com/c/omniverse/simulation/69) - Isaac Sim 技术支持
- [Nav2 GitHub](https://github.com/ros-planning/navigation2) - Nav2 源代码和问题追踪
- [ROS Discourse](https://discourse.ros.org/) - ROS 社区讨论

### 相关论文
- **MPPI Controller**: Williams, G., et al. "Information theoretic MPC for model-based reinforcement learning." ICRA 2017.
- **AMCL**: Thrun, S., et al. "Probabilistic Robotics." MIT Press, 2005.
- **SMAC Planner**: Dolgov, D., et al. "Practical search techniques in path planning for autonomous driving." AAAI 2008.

### 教程和示例
- [Nav2 入门教程](https://navigation.ros.org/getting_started/index.html)
- [Isaac Sim ROS 2 集成](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_ros2_navigation.html)
- [MPPI Controller 调优指南](https://navigation.ros.org/configuration/packages/configuring-mppic.html)

## 常见问题 (FAQ)

### Q1: 为什么需要 QoS 中继节点？
**A**: Isaac Sim 默认使用 RELIABLE QoS 发布激光雷达数据，而 Nav2 的代价地图默认期望 BEST_EFFORT QoS。QoS 策略不匹配会导致数据无法订阅。中继节点解决了这个兼容性问题。

### Q2: 如何选择合适的参数配置？
**A**: 
- **标准配置**: 适合大多数室内场景
- **快速配置**: 适合开阔环境，需要快速移动
- **精确配置**: 适合狭窄通道，需要精确停靠
- **极速配置**: 仅用于测试或空旷环境

### Q3: 为什么使用 MPPI Controller 而不是 DWB？
**A**: MPPI Controller 更适合全向移动机器人，因为它：
- 原生支持全向运动模型
- 采样效率更高
- 避障性能更好
- 路径跟随更平滑

### Q4: 如何提高定位精度？
**A**: 
1. 增加 AMCL 粒子数量
2. 减小运动噪声参数
3. 增加激光束采样数量
4. 确保地图与环境精确匹配
5. 使用更高分辨率的地图

### Q5: 导航失败时如何调试？
**A**: 
1. 检查 RViz2 中的可视化（路径、代价地图、粒子云）
2. 查看终端日志输出
3. 使用 `ros2 topic echo` 检查数据流
4. 使用 `ros2 node info` 检查节点状态
5. 录制 rosbag 进行离线分析

### Q6: 可以在真实机器人上使用吗？
**A**: 可以。只需：
1. 设置 `use_sim_time:=False`
2. 确保真实机器人发布 `/odom` 和 `/laser_scan_fuse`
3. 确保真实机器人订阅 `/cmd_vel`
4. 根据实际机器人调整参数（速度限制、足迹大小等）

### Q7: 如何创建自己的地图？
**A**: 使用 Nav2 的 SLAM 工具链：
```bash
# 启动 SLAM
ros2 launch slam_toolbox online_async_launch.py

# 驱动机器人探索环境
# 在 RViz2 中查看地图构建过程

# 保存地图
ros2 run nav2_map_server map_saver_cli -f my_map
```

### Q8: 支持多机器人导航吗？
**A**: 当前版本不直接支持。需要：
1. 为每个机器人使用独立的命名空间
2. 修改启动文件支持命名空间参数
3. 配置多机器人协调机制

## 贡献指南

欢迎提交问题报告和改进建议！

### 报告问题
1. 在 GitHub Issues 中创建新问题
2. 提供详细的问题描述
3. 包含复现步骤
4. 附上相关日志和截图

### 提交改进
1. Fork 本仓库
2. 创建特性分支
3. 提交代码并编写测试
4. 发起 Pull Request

### 代码规范
- 遵循 ROS 2 代码风格指南
- 使用有意义的变量和函数名
- 添加必要的注释
- 更新相关文档

## 版本历史

### v0.1.0 (2024-05-20)
- ✨ 初始版本发布
- ✅ 基础导航功能
- ✅ AMCL 定位
- ✅ MPPI 局部规划器
- ✅ SMAC 全局规划器
- ✅ 4 种参数配置方案
- ✅ QoS 中继节点
- ✅ 自定义无旋转行为树
- ✅ Isaac Sim 集成支持
- ✅ RViz2 可视化配置

### 未来计划
- 🔄 多机器人导航支持
- 🔄 动态障碍物预测
- 🔄 语义地图集成
- 🔄 自适应参数调优
- 🔄 Web 界面监控

---

**最后更新**: 2024-05-27  
**文档版本**: 1.0  
**功能包版本**: 0.1.0
