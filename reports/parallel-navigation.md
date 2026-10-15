# 会话 C：Nav2 导航迁移

首次检查：2026-10-14；最近复验：2026-10-15（Asia/Shanghai）。**导航配置修复与局部验证完成；真实导航尚未执行验收。** 修改仅限 `bobac_robotac_src/src/bobac3_navigation/` 和本报告。没有启动共享仿真/导航控制栈、发送底盘运动命令、重启容器、重建镜像、写共享 `.runtime`、暂存文件、切换分支或修改项目 TODO。

## 基线和实际入口

- 已读取 MIGRATION_TODO.md、DGX_SPARK_MIGRATION_PLAN.md、根 README.md。
- 起始其他会话修改 `.gitignore`、`tools/course.sh`、`tools/migrate_scene.py`、`tools/run_sim.py`，新增 `tools/rebuild_graphs.py`；导航包起始无修改。结束时出现的 Dockerfile/compose/course_bringup/视觉/场景等修改均未触碰。
- 实际入口：`course_bringup/launch/course.launch.py` → `bobac3_navigation.launch.py` → 默认 `params/bobac3_navigation_params.yaml`；另四个调参文件可通过 `params_file` 选择。
- 宿主实际版本：Nav2 1.3.11、BehaviorTree.CPP 4.9.0。初始可见 `bobac-isaac6-migration-preflight`，准备容器验证时该容器已不存在；没有使用 teleop/real-robot 容器或创建替代容器。
- bwrap 在执行命令前报 `Failed RTM_NEWADDR: Operation not permitted`，限定范围命令使用自动审查允许的沙箱外执行，无自动审批拒绝项。

## 修改文件和实际问题

下表路径均相对导航包，最后一行除外。

| 文件 | 问题、处理和验证边界 |
| --- | --- |
| `launch/bobac3_navigation.launch.py` | 安装版上游 navigation launch 额外启动未配置的 route/collision monitor/docking，并改变输出链。现在明确启动课程必需的 9 个 lifecycle 节点及 2 个 manager；固定 controller/behavior → `/cmd_vel_nav` → velocity smoother → `/cmd_vel`。子进程固定 domain 44、Cyclone DDS、sim time true。保留原启动参数，增加 relay/readiness/autostart 参数；非零进程退出或 readiness 超时触发 Shutdown，被公共 launch 包含时会关闭整个公共 launch。 |
| 同上 | **原 no-spin BT 路径重写实际不生效**：默认 YAML 没有 `default_nav_to_pose_bt_xml`，叶名称重写不能新增参数，可能仍加载默认含 Spin 的树。改用完整 `bt_navigator.ros__parameters.default_nav_to_pose_bt_xml` 路径注入，同时注入地图完整路径；覆盖全部变体。 |
| `params/bobac3_navigation_params.yaml`、`_faster_best.yaml`、`_slow_better.yaml`、`_fast_bester.yaml`、`bobac3_navigation_params copy.yaml` | 原 `::` 插件名称、progress checker 列表已兼容，保留。behavior server 改用 local/global costmap/footprint 参数及 `local_frame: odom` / `global_frame: map`；Spin 最小角速度修正为正幅值 0.1（原 -0.3）。移除不起作用的 scan.reliability/queue_size 和 MPPI vy_min（Omni 横向为 ±vy_max）。保留 Twist 和课程主要调参。 |
| `behavior_trees/navigate_to_pose_no_spin.xml` | 保留 BTCPP v4 和无 Spin 恢复，新增 compute_path/follow_path/backup 错误码端口；实际动态插件注册确认节点和端口兼容。新增 Wait 错误码端口被实际插件检查拒绝，已移除。 |
| `scripts/laser_qos_relay.py` | 原 RELIABLE 订阅不兼容 BEST_EFFORT 源，改为 BEST_EFFORT/VOLATILE 输入/输出，完整保留消息，默认 sim time true。默认直连原扫描；`use_scan_relay:=true` 启动 relay 并同步切换 AMCL/local costmap 到既有 `/laser_scan_fuse_best_effort`。 |
| `scripts/navigation_readiness.py`（新增） | 只读检查时钟、数据/时间戳新鲜度、雷达几何/有效返回、odom/scan frame、TF、9 个 lifecycle active、sim time、action、`/cmd_vel` Twist 发布/底盘订阅端。墙钟超时、JSON/退出码明确，lifecycle/参数答复过期 3 秒不视为就绪。`--goal-id` 只查询已有目标 GetResult，不发送目标。 |
| `CMakeLists.txt`、`package.xml` | 安装 relay/readiness 与 BT/配置，排除旧字节码；补齐实际运行依赖（MPPI、Smac、smoother、rclpy、TF、Cyclone 等），配置包只保留 ament_cmake 构建依赖，移除未使用的旧 build_depend。 |
| `maps/map_older.yaml` | 旧元数据错指新 `map.png`，改为同目录 `map_older.png`；默认地图不变。 |
| `tests/test_navigation_config.py`（新增） | 检查实际安装 launch 的全部参数变体/relay 选择、路径、插件注册表和命令重映射。 |
| `tests/bt_registration_check.cpp`（新增） | 动态加载 10 个 Nav2 BT 插件、注册 XML 并检查实际 manifest 端口，不实例化动作节点。 |
| `tests/test_navigation_diagnostics.py`（新增） | 两种发布可靠性下的局部 relay 转发和消息完整性，以及 mock 就绪/失效判定。 |
| `README.md`、`README_SETUP.md` | 替换 Humble 说明，提供 Jazzy 接口、独立构建、诊断命令和验证边界。 |
| 项目 `reports/parallel-navigation.md` | 本轮记录和交接。当前 `.gitignore:32` 排除 `/reports/*`，报告存在本地但未列为 untracked；由 A 按需显式纳入版本控制。本轮不修改 gitignore/不暂存。 |

保留默认 `/laser_scan_fuse`、`/odom`、TF frame 和 `/cmd_vel` Twist 类型。旧 `params/qos_overrides.yaml` 不被入口加载，不代表实际 QoS；AMCL/obstacle layer 原生 sensor-data 订阅可接两种发布可靠性。

## 验证命令和结果

独立产物全部位于宿主 `/tmp/bobac-nav-c/{build,install,log,ros-logs}`。项目根目录：

```bash
source /opt/ros/jazzy/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR=/tmp/bobac-nav-c/ros-logs
colcon --log-base /tmp/bobac-nav-c/log build \
  --base-paths bobac_robotac_src/src/bobac3_navigation --packages-select bobac3_navigation \
  --build-base /tmp/bobac-nav-c/build --install-base /tmp/bobac-nav-c/install
source /tmp/bobac-nav-c/install/setup.bash
ros2 launch bobac3_navigation bobac3_navigation.launch.py --show-args
python3 bobac_robotac_src/src/bobac3_navigation/tests/test_navigation_config.py
g++ -std=c++17 bobac_robotac_src/src/bobac3_navigation/tests/bt_registration_check.cpp \
  -I/opt/ros/jazzy/include -L/opt/ros/jazzy/lib -Wl,-rpath,/opt/ros/jazzy/lib \
  -lbehaviortree_cpp -ltinyxml2 -o /tmp/bobac-nav-c/bt_registration_check
/tmp/bobac-nav-c/bt_registration_check \
  /tmp/bobac-nav-c/install/bobac3_navigation/share/bobac3_navigation/behavior_trees/navigate_to_pose_no_spin.xml \
  /opt/ros/jazzy/lib
```

结果：构建退出 0（1 package finished）；show-args 退出 0；5 个配置测试通过（含 5 套参数 × 两种 relay 选择）；10 个实际 BT 插件注册/XML/端口检查通过。上游 RewrittenYaml 有未关闭输入文件的 ResourceWarning，测试通过，未修改上游。

```bash
export ROS_DOMAIN_ID=44
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>none</ParticipantIndex></Discovery></Domain></CycloneDDS>'
python3 bobac_robotac_src/src/bobac3_navigation/tests/test_navigation_diagnostics.py
ros2 run bobac3_navigation navigation_readiness.py --timeout 2 \
  --report /tmp/bobac-nav-c/readiness-negative.json
unset CYCLONEDDS_URI
```

结果：3 个诊断/relay 测试通过；BEST_EFFORT 和 RELIABLE 扫描均完整转发。无数据诊断退出 **1（预期）**，JSON failed，列出缺失 clock/odom/scan、TF、lifecycle、action 和底盘接口。局部 DDS 限 loopback、关闭发现，没有连接共享机器人；只发布合成扫描，没有速度命令。首次诊断发现 `Node.clients` 只读属性名称冲突，改为 `service_clients` 后复验通过。

另通过 Python AST、manifest XML、GetResult 请求接口和限定路径 git diff --check。GetResult 的真实成功/失败响应仍待运行验证。日志：`/tmp/bobac-nav-c/build-final.log`、`config-check.log`、`show-args.log`、`readiness-negative.json`；本报告保留命令供重现。

## 2026-10-15 继续工作复验

- 重新读取当前报告、导航入口与 git 状态，其他会话修改保留。
- `/tmp/bobac-nav-c` 已被环境清理，先前日志当时已不存在；重新创建该独立目录并干净构建，1 package finished [1.00s]，退出 0。当前目录中的日志均为此次重新生成。
- show-args、5 个配置测试（0.638s）、3 个诊断/relay 测试（0.084s）、10 个动态 BT 插件注册/XML/端口检查全部通过。
- 已安装的 navigation_readiness 隔离负向检查退出 1（预期），JSON failed，明确报告 29 项缺失条件；AST、package XML 和限定路径 git diff --check 通过。
- 当前 docker ps 没有运行中的容器；没有为了补齐容器证据启动任何容器、仿真或导航栈。容器和真实目标验收仍由 A 安排。

## A 后续真实导航验收（本会话未执行）

A 在指定课程容器和 overlay 中重复独立构建/离线检查，取消局部测试 DDS 隔离配置，保留课程 DDS 设置。播放场景后确认默认 AMCL 初始位姿 `(-3.783, 2.137, -1.5708)` 与 reset pose/地图原点一致，然后由 A 安排启动：

```bash
export ROS_DOMAIN_ID=44
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch bobac3_navigation bobac3_navigation.launch.py rviz:=false
# 另一个已 source 相同课程 overlay 的终端：
ros2 run bobac3_navigation navigation_readiness.py --timeout 60 --report /tmp/navigation-ready.json
ros2 param get /bt_navigator default_nav_to_pose_bt_xml
ros2 topic info /cmd_vel --verbose
ros2 topic info /laser_scan_fuse --verbose
```

成功判据：readiness 退出 0、status=passed、errors=[]；9 个 lifecycle active、sim time true；TF `map → odom → base_link → fuse_lidar_link` 连通，数据与时间戳新鲜；导航 action 可用，velocity_smoother 发布端及 Isaac 底盘订阅端均为 `/cmd_vel` Twist；BT 路径指向安装的 no-spin XML。relay 模式须加 `--scan-topic /laser_scan_fuse_best_effort` 并检查 relay 输出。

**以下命令发送运动目标，仅由 A 确认可达目标后执行。** 设置 NAV_GOAL_X/Y/YAW 为地图坐标中的真实验收目标；本轮不提供未经场景验证的固定坐标：

```bash
: "${NAV_GOAL_X:?Set a verified reachable map x}"
: "${NAV_GOAL_Y:?Set a verified reachable map y}"
: "${NAV_GOAL_YAW:?Set verified yaw in radians}"
export NAV_GOAL_X NAV_GOAL_Y NAV_GOAL_YAW
NAV_GOAL_JSON=$(python3 - <<'GOAL_PY'
import json, math, os
x, y, yaw = [float(os.environ[key]) for key in ['NAV_GOAL_X', 'NAV_GOAL_Y', 'NAV_GOAL_YAW']]
print(json.dumps({'pose': {'header': {'frame_id': 'map'}, 'pose': {
    'position': {'x': x, 'y': y, 'z': 0.0},
    'orientation': {'z': math.sin(yaw / 2), 'w': math.cos(yaw / 2)}}}, 'behavior_tree': ''}))
GOAL_PY
)
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "$NAV_GOAL_JSON" --feedback | tee /tmp/navigation-goal-cli.log
# 及时复制发送命令记录的 UUID（32 位 hex 或带连字符）：
ros2 run bobac3_navigation navigation_readiness.py --timeout 120 \
  --goal-id <目标UUID> --report /tmp/navigation-goal.json
```

目标验收判据：被接受，反馈距离/路径推进，最终 SUCCEEDED/error_code=0；只读查询 goal.status=4、goal.error_code=0。结合 TF/odom/RViz/场景证据确认实际到位、无碰撞、停止后速度归零。默认 xy/yaw 容差 0.05 m / 0.04 rad，切换 preset 时按其参数验收。保存 UUID、坐标、路径、Nav2/Isaac 日志及到位证据；action/lifecycle 就绪不能代替目标成功。及时查询，Nav2 会清理过期结果。

## 待验证与 A 配合事项

- 场景提供 `/clock`、`/odom`（odom → base_link）、`/laser_scan_fuse`（fuse_lidar_link）、TF `odom → base_link → fuse_lidar_link`、`/cmd_vel` Twist 底盘订阅。AMCL 独占 map → odom，避免场景重复发布。实际 frame/QoS 仍需运行核对。
- 公共入口目前只在传感器 readiness 后并行启动课程节点；本包 gate 可关闭启动失败的公共 launch，任务触发前 A 仍需显式等待 navigation_readiness 成功。默认 90 秒超时；autostart=false 调试时配套 check_readiness=false。
- 容器内构建、lifecycle configure/activate 中插件对象创建与 BT 实例化、真实地图/定位、目标结果与实际运动均未验证。离线 BT 注册没有实例化 action 节点。
- 保留原 controller_frequency=30/model_dt=0.05；官方 MPPI 对较短控制周期警告并关闭序列平移，不是本轮阻塞性加载错误。A 按 Spark 性能和反馈决定周期调参。
- footprint、25m/20Hz 局部地图、MPPI batch 2000、速度和恢复后退距离保持课程值；实测性能、物理足迹和稳定性由 A 验证。本轮仅修正 Spin 最低角速度非法负幅值。
- 旧地图引用已修正，使用旧图时重新核对定位。A 安排断流/缺 TF/插件或 BT configure 失败等故障诊断，以及固定初始条件重复导航与课程闭环。
- 报告被 gitignore 排除，若提交请 A 显式添加；不要使用 git add . 混入各会话修改。

## 官方核对依据

本次以本机实际注册表/动态 BT manifest 为最终离线依据，并核对官方 Jazzy 源码：[behavior_server 参数](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_behaviors/src/behavior_server.cpp)、[MPPI 周期与横向约束](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_mppi_controller/src/optimizer.cpp)、[BT Navigator 插件加载](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_bt_navigator/src/bt_navigator.cpp)、[obstacle layer 订阅](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_costmap_2d/plugins/obstacle_layer.cpp)。目标容器仍应按其安装版本重复检查。

## 可直接转发给 A

会话 C 已完成导航包限定范围修复：修复 no-spin BT 路径重写不生效、Jazzy behavior 参数、未安装 relay 与 QoS，明确 9 个 lifecycle 节点及 Twist `/cmd_vel` 输出链，补齐依赖/地图路径，新增只读 navigation_readiness（默认启动 gate，超时关闭公共 launch；可查询已有目标结果）。独立宿主构建、全部 5 套参数与两种 relay 选择、5 个配置测试、3 个诊断/relay 测试、10 个 BT 插件注册/XML/端口检查通过；无数据诊断预期退出 1。产物在 `/tmp/bobac-nav-c`，未启动共享导航/仿真、未发送运动命令、未改公共文件或 TODO。preflight 容器已不存在，容器验证和真实导航请 A 安排。请在任务触发前等待 navigation_readiness 成功，并保证原话题、TF 和 Twist 底盘订阅端。验收命令及判据见本报告；报告被 gitignore 排除，需 A 按需显式纳入版本控制。
