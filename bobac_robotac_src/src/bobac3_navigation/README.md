# bobac3_navigation（ROS 2 Jazzy）

当前 USD 世界地图与桌边站位候选见 [WORLD_SCENE_ALIGNMENT.md](WORLD_SCENE_ALIGNMENT.md)：独立世界坐标入口、正确 reset 初值、定位/规划两种地图及抓取可达性限制。原默认入口和旧地图保留；A 已验证新地图定位与全局路径净距；真实运动净距失败并取消，最新局部圆包络、低速运输配置和轨迹偏差门禁待 A 验收。

Bobac 全向底盘的 AMCL、SmacPlanner2D、MPPI、Nav2 行为树和速度平滑配置。迁移后的实际入口为 `bobac3_navigation.launch.py`，公共课程入口包含此 launch。构建/离线检查通过不等于 Isaac Sim 6.0 导航运行验收通过，当前验证证据与待办见项目 `reports/parallel-navigation.md`。

## 接口与启动

| 接口 | 要求 |
| --- | --- |
| `/clock` | 仿真时间持续前进 |
| `/odom` | `nav_msgs/msg/Odometry`，frame `odom`，child `base_link` |
| `/laser_scan_fuse` | `sensor_msgs/msg/LaserScan`，frame `fuse_lidar_link` |
| TF | 场景提供 `odom → base_link → fuse_lidar_link`；AMCL 提供 `map → odom` |
| `/cmd_vel` | `geometry_msgs/msg/Twist`，场景底盘订阅 |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` |

在 A 安排的仿真验收中，先播放场景、确认原始数据与 TF，然后启动导航：

```bash
export ROS_DOMAIN_ID=44
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch bobac3_navigation bobac3_navigation.launch.py rviz:=false
```

launch 为子进程固定 domain 44、Cyclone DDS，所有节点使用 `use_sim_time=true`。命令链明确为 controller/behavior → `/cmd_vel_nav` → velocity smoother → `/cmd_vel`。只启动课程所需的 9 个 lifecycle 节点，不随上游 bringup 版本引入课程没有配置的其他服务器。

launch 默认执行只读就绪检查，90 秒未就绪则输出具体缺项并关闭本次导航 launch；节点非零退出也关闭本次 launch。`autostart:=false` 用于 A 手动调试 lifecycle 时，应同时设置 `check_readiness:=false`。诊断工具不发送目标、不发布底盘命令。

| 参数 | 默认值 / 含义 |
| --- | --- |
| `params_file` | 安装目录 `params/bobac3_navigation_params.yaml` |
| `map` | 安装目录 `maps/map.yaml`；必须传完整路径 |
| `use_sim_time` | `true`（课程只允许 `true`/`True`） |
| `rviz` / `rviz_config` | `true` / 安装目录的 Bobac RViz 配置 |
| `autostart` | `true` |
| `use_scan_relay` | `false`；原始话题直连 AMCL/costmap |
| `check_readiness` / `readiness_timeout` | `true` / `90` 秒（墙钟计时） |

可选调参文件包括 `bobac3_navigation_params_faster_best.yaml`、`bobac3_navigation_params_slow_better.yaml`、`bobac3_navigation_params_fast_bester.yaml` 和历史 `bobac3_navigation_params copy.yaml`。各文件保持原课程调参，统一修复 Jazzy 插件/接口。launch 将所选文件的 BT 路径和地图路径重写到本次安装目录，不依赖启动工作目录。

```bash
nav_share="$(ros2 pkg prefix bobac3_navigation)/share/bobac3_navigation"
ros2 launch bobac3_navigation bobac3_navigation.launch.py \
  params_file:="$nav_share/params/bobac3_navigation_params_slow_better.yaml" \
  map:="$nav_share/maps/map.yaml" rviz:=false
```

## 雷达 QoS

AMCL 和 obstacle layer 的传感器订阅可接收 RELIABLE 或 BEST_EFFORT 发布者，默认不需要 relay。旧 `params/qos_overrides.yaml` 没有被 launch 加载，其内容不代表实际 costmap QoS；`scan.reliability` 也不是 obstacle layer 的 QoS 参数。

需要保留历史 relay 输出话题时，设置 `use_scan_relay:=true`。launch 同时启动 relay，并将 AMCL 和 local costmap 输入切换为 `/laser_scan_fuse_best_effort`。relay 输入/输出均为 BEST_EFFORT + VOLATILE，保持消息 header、frame、时间戳和 ranges。

```bash
ros2 launch bobac3_navigation bobac3_navigation.launch.py use_scan_relay:=true rviz:=false
# 单独启动 relay 时也要使用 domain 44 / Cyclone DDS；不要与 launch 重复启动。
ros2 run bobac3_navigation laser_qos_relay.py
```

## 就绪与目标诊断

```bash
ros2 run bobac3_navigation navigation_readiness.py --timeout 60 --report /tmp/navigation-ready.json
# 若启用了 relay：
ros2 run bobac3_navigation navigation_readiness.py --timeout 60 \
  --scan-topic /laser_scan_fuse_best_effort --report /tmp/navigation-ready.json
```

成功判据：退出 0，JSON `status=passed`、`errors=[]`；全部 lifecycle 节点 `active`、`use_sim_time=true`，数据与时间戳新鲜、必要 TF 可查、导航 action 可用、速度平滑器有 `/cmd_vel` Twist 发布端及底盘订阅端。这是启动就绪检查，不是目标执行验收。

A 确认地图、初始位姿和可达目标后，可使用 RViz Navigation Goal 或 `ros2 action send_goal ... --feedback` 发送目标；必须记录目标 UUID、最终状态、错误码、轨迹及机器人实际到位证据。查询一个已经发送的目标结果：

```bash
ros2 run bobac3_navigation navigation_readiness.py --timeout 120 \
  --goal-id <发送目标时记录的UUID> --report /tmp/navigation-goal.json
```

`--goal-id` 只调用该现有目标的 GetResult，不发送新目标。仅在 `goal.status=4`（SUCCEEDED）、`goal.error_code=0` 时返回成功；未知、拒绝、取消、终止或超时均失败。Nav2 会在有限时间后清理结果，需及时查询。启用 relay 时继续传相应 `--scan-topic`。

故障定位：无扫描先查 Play、domain、QoS；无 `map → odom` 查 AMCL 初始位姿/地图/扫描；lifecycle 非 active 查对应节点插件与配置日志；无 `/cmd_vel` 底盘订阅端交由场景维护者修复；目标失败检查 BT 的 compute_path/follow_path/backup 错误码及 Nav2 日志。默认 BT 使用 v4 格式，保留清图、等待和后退恢复，移除 Spin 恢复。

本次未更改速度、采样、代价地图尺寸或机器人 footprint 调参。`controller_frequency=30` 与 MPPI `model_dt=0.05` 会产生周期警告并禁用控制序列平移，A 应在真实 Spark 性能与目标执行验证后决定调参，当前不据离线检查改变课程运动行为。

## 局部构建与检查

见 [README_SETUP.md](README_SETUP.md)。测试不会启动仿真/导航控制栈；真实功能验收由 A 安排。
