# 会话 C：Nav2 导航迁移

首次检查：2026-10-14；最近复验：2026-10-16（Asia/Shanghai）。**A 最新低速目标因预测跟踪保护取消，记录区间保住0.55m净距/0.05m偏差门槛，但未到达。C已修正控制路径起点、初段评分与仿真时钟CLOSED_LOOP平滑；独立构建及新增离线检查通过，现有场景来源哈希门禁失败保留，需A复核并实测。** 修改仅限 `bobac_robotac_src/src/bobac3_navigation/` 和本报告。没有启动共享仿真/导航控制栈、发送底盘运动命令、重启容器、重建镜像、写共享 `.runtime`、暂存文件、切换分支或修改项目 TODO。

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

## 2026-10-16 世界坐标/地图专项

- 用户与 A 的已保存证据确认：旧入口真实 Nav2 启动、9 个 lifecycle active、navigation readiness 已通过；尚未发送导航目标。这不证明旧地图定位正确。证据 `reports/navigation-readiness-live.json`、`navigation-readiness-common-stack.json`。
- 本会话离线 USD 0.25.11 验证 source/course 两个场景，报告中 13 个碰撞体顶点最大误差 0 m；未启动 Kit。源场景 SHA256 `a859a876e4c1829ffe12b0258771ef2f6f0a095cf3a3a03ec37ec30d25b47686`，course SHA256 `9383aec555c3111e13a12e7811f46bae9566ee94a18a3dd4d793022b31c86237`。远程 FlatGrid 引用在此离线解析中警告，未纳入地图覆盖的附加几何。
- 旧图 map.yaml 覆盖 [-4.6525,4.7975) × [-2.975,3.025)，spawn (8,1.082) 在图外。围墙跨度约 10.5 × 15.1 m，旧图 9.45 × 6 m，不能通过单位比例的平移/旋转覆盖完整当前世界；未知旧图的局部配准，不臆造刚体对应。
- 新地图明确选择 map XY = USD world XY（坐标约定，不额外发布 TF）。base_link 初始精确 XY=(7.999990895,1.082003594)，yaw=-0.000035778 rad。A 的 base-control-saved-scene.json 里初始 odom XY 约 (0,0)，因此 map→odom 应约 (8,1.082,0)，不能同时假定 odom=world。
- 雷达 z=0.229147m，桌腿到 z=.4、桌板 z=.4~.6。新增定位切片图与全碰撞投影规划图；独立 world_scene_navigation.launch.py 用第二个只读地图服务器发布 /world_obstacles，保持旧入口/图与原有跨模块接口。此候选入口共10个 lifecycle 节点（旧9+world_obstacle_map_server），真实启动由A验证。
- 已新增坐标、投影、路线余量、旧地图哈希和可复现生成测试：先4项预期失败，再新增地图后全部5项通过。正在补桌边候选的机械臂几何依据与最终独立构建/交接。

### 本轮新增/修改文件

仅导航包：`maps/world_scene{,_lidar}.{pgm,yaml}`、`maps/world_scene_geometry.json`、`maps/world_scene_candidates.json`、`maps/world_scene_overview.svg`；`params/bobac3_navigation_world_params.yaml`；`launch/world_scene_navigation.launch.py`；`scripts/generate_world_map.py`、`scripts/analyze_world_candidates.py`；`tests/test_world_scene_map.py`；以及 `CMakeLists.txt`、`README.md`、`WORLD_SCENE_ALIGNMENT.md`。只读 readiness 增加可选 `--extra-lifecycle-node`，测试文件 `tests/test_navigation_diagnostics.py` 同步覆盖第10个地图节点，原9节点默认行为保留。原 `map.png/map.yaml/map_older.png/map_older.yaml` 4个SHA256未变；默认旧参数/入口未修改。其他会话新增的 arm_motion_check/arm_offset_check/场景/公共文件均未修改。

### map/world 结论与桌边目标

- 新 map/world 的平面关系按构造为单位 SE(2)，栅格原点 [-1,-10.5,0]，resolution=.05，230×320，覆盖 [-1,10.5)×[-10.5,5.5)。这不是运行时 AMCL 已对齐的证据，也不发布 world TF。
- 正确初值采用真实 authored base_link：x=7.999990895390511、y=1.0820035936354835、yaw=-0.00003577751974137747（近似8,1.082,0）。AMCL z=0；map→odom由AMCL按实际odom初值推导，已有A证据显示odom初值约0，因此map→odom约8,1.082,0。
- 强行将旧AMCL初值配到新spawn得到 world_from_old_map≈(10.137112,4.864935,1.570764)；物体映射到旧图(-4.265215,8.736975)，仍在旧图外。这只是单点锚定，不是配准。没有足够地标证据给旧图一个可信的全局 map↔world 对应。
- table2完整桌板边界x=.7~1.7、y=-.65~1.85、z=.4~.6；原雷达z=.229，定位图只保留腿，规划图保留整桌，避免AMCL和规划共用错误高度投影。
- 候选朝向统一yaw=π（世界-X，四元数0,0,1,0）。优先首次导航桌前暂驻(2.35,.6,π)，包络含padding到桌东边名义净距.39m；较近(2.15,.6,π)净距.19m；更近(2.10,.6,π)净距.14m。净距未扣定位/到位/栅格误差，不等于运行安全余量。
- 不能直线导航穿过Wall_02（x5.45~5.95，y-3.7~2.3）。候选北绕中心线(8,1.082)→(6.6,3)→(4.8,3)→(2.35,.6)，离线逐段201点采样，保守栅格距障碍/未知至少.50m，可容当前姿态约.45m圆包络。未调用Nav2规划或执行。
- **机械臂约束需单独处理**：base→arm_mount≈(+.1,0,+.294)m、安装yaw≈π。真实URDF零关节FK与A记录匹配。在物体z=.612738、工具偏移.166、垂直向下且Y沿物体长轴的名义抓取假设下，16个有界种子数值IK对2.15/2.10位姿未找到解（最佳位置残差约.0886/.0519m），不能称其抓取就绪，也不能据搜索失败证明不可达。
- (2.02,.6,π)有数值IK解（位置残差3.2e-8m、旋转误差2e-9rad），但名义净距仅.06m，旧.05m到位容差再叠加定位/栅格/角度误差不足；标为tight_diagnostic_only，不推荐直接发送。成功IK不包括自碰撞/桌面碰撞、可操作度或轨迹检查。
- A应先用含table2的MoveIt PlanningScene做plan-only；可评估不同抓取姿态、精确近桌方案或把物体移到东缘x≈1.55~1.60（仍需物体完全在桌内）。任何场景调整交A，本会话未修改物体/场景。

### 本轮复现与验收命令

完整依据、来源哈希、候选说明和命令见导航包 `WORLD_SCENE_ALIGNMENT.md`；SVG见 `maps/world_scene_overview.svg`。生成工具均离线、无ROS/Kit连接。

```bash
# 项目根目录：
source /opt/ros/jazzy/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR=/tmp/bobac-nav-c/ros-logs
colcon --log-base /tmp/bobac-nav-c/log build \
  --base-paths bobac_robotac_src/src/bobac3_navigation --packages-select bobac3_navigation \
  --build-base /tmp/bobac-nav-c/build --install-base /tmp/bobac-nav-c/install
source /tmp/bobac-nav-c/install/setup.bash
ros2 launch bobac3_navigation world_scene_navigation.launch.py --show-args
python3 bobac_robotac_src/src/bobac3_navigation/tests/test_navigation_config.py
python3 bobac_robotac_src/src/bobac3_navigation/tests/test_world_scene_map.py
# diagnostics测试仍须按前文设置loopback/关闭发现的隔离Cyclone DDS配置。
```

以下只交A执行，不在本会话运行：

```bash
export ROS_DOMAIN_ID=44
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# A在确认没有重复导航节点的条件下启动：
ros2 launch bobac3_navigation world_scene_navigation.launch.py rviz:=true
# 独立终端、相同overlay：
ros2 run bobac3_navigation navigation_readiness.py --timeout 90 \
  --extra-lifecycle-node world_obstacle_map_server --report /tmp/world-navigation-ready.json
ros2 lifecycle get /world_obstacle_map_server
ros2 param get /amcl initial_pose.x
ros2 param get /amcl initial_pose.y
ros2 param get /amcl initial_pose.yaw
ros2 topic echo /amcl_pose --once
ros2 topic echo /odom --once
```

A发送任何导航目标前的判据：10个lifecycle active，reset实际base_link≈(8,1.082,0)，RViz用map固定帧叠加激光/TF，外墙、竖墙等至少三个方向/地标重合且AMCL不持续漂移；两图原点/分辨率/尺寸一致，costmap整桌保留、未知外部不通行，运输姿态/包络和实际环境核对一致。先用ComputePathToPose仅规划确认北绕路线，再由A决定发送桌前暂驻目标；SUCCEEDED/error_code=0并有实际到位/停止/无碰撞证据后才计目标成功。readiness和离线几何均不能代替地图匹配或导航目标验收。近桌2.02位姿与完整抓取/提升没有验收结论。

### 2026-10-16 最终局部验证结果与最新交接

- 独立构建退出0，1 package finished [0.24s]；世界入口show-args退出0，资源路径来自独立install。
- 原配置回归5项、新地图/坐标/来源/FK/入口检查8项、诊断/relay检查4项，合计17项通过。额外地图lifecycle缺失测试先失败、实现后通过。
- 已安装诊断在loopback/关闭发现的DDS环境下运行，第10个地图节点必须active，负向退出1（预期），列出31项具体缺失条件。没有访问共享栈。AST、manifest XML、限定路径diff --check通过。
- 日志在 `/tmp/bobac-nav-c/world-map-build.log`、`world-launch-args.log`、`world-config-regression.log`、`world-map-tests.log`、`world-diagnostics-tests.log`、`world-readiness-negative.{json,log}`。
- 其他会话在本轮期间将报告纳入git跟踪（当前git状态显示M reports/parallel-navigation.md），此前“被gitignore排除”的描述仅为历史状态；本轮没有暂存、提交或修改gitignore。
- A应先在(2.35,.6)暂驻并完成朝西转向，再评估更近站位，不能在近桌处任意旋转；离线北绕的圆包络余量不适用于最终贴桌/展开机械臂阶段。

可直接转发A：C已核验几何报告与当前source/course场景13个碰撞体顶点一致，旧图尺寸/原AMCL初值不能为当前spawn与table2提供可信全局配准。新增独立world_scene_navigation.launch.py及world参数，不改旧入口/旧图；新map XY按构造等于USD world，初始base_link≈(8,1.082,0)，AMCL map→odom按实际odom推导（odom初值约0，map→odom约8,1.082）。定位图按雷达z=.229切片，规划图完整覆盖桌板，新增只读world_obstacle_map_server后共10个lifecycle，gate同步检查10个。建议首次桌前导航候选(2.35,.6,π)，绕竖墙北端经(6.6,3)/(4.8,3)；更近(2.15,.6,π)名义包络净距.19m但垂直抓取IK未找到。物体仍在(1.4,.6)时(2.02,.6,π)虽有名义IK解，净距仅.06m，不建议直接按旧容差导航；请用含桌子的MoveIt plan-only评估姿态/精度，或由A决定移动目标近东边缘。17项局部测试和独立构建通过，未切换真实地图/发布定位/发送运动。详见导航包WORLD_SCENE_ALIGNMENT.md及本报告；真实地图、定位、规划/目标验收仍由A执行。

## 2026-10-16：运输包络规划约束与整段路径净距门禁

### 新证据与失败根因

已读取 `reports/world-navigation-alignment-plan-3d.json`（SHA256 `39a626371f9f333bc7cd3aca92a85b629c20b846f8eacd229be7f0031773257d`）。该报告真实激光900点全部在地图障碍0.10m内、p95约0.03468m；初始定位位置误差0.0000458m、yaw误差0.0000487rad；北绕ComputePathToPose状态4/error_code=0。**这些是A已取得的证据，本轮C没有运行共享栈。**

原报告同时记录最小中心净距0.183010m和0.45m圆包络检查false，顶层仍passed；因此不能将顶层passed作为运输路径验收。原SmacPlanner2D使用圆形近似，global矩形footprint的内切半径/0.22m膨胀范围不能覆盖约0.45m全姿态运输圆包络，导致北端切角路线被接受。当前矩形含padding最远角约hypot(.37,.25)=.4465m；0.45m仅适用已核验运输姿态，不是机械臂运动扫掠包络。

### 本轮修改（全部在导航包内，另有本报告）

- `params/bobac3_navigation_world_params.yaml`：仅独立world profile的global删除多边形footprint，使用 `robot_radius=0.64`、padding=0，覆盖0.45运输圆+0.10预留+0.09离散化缓冲；global/local inflation半径均0.85，global scaling=3。local仍保留真实方向多边形及padding，保留接口、原地图、旧入口、use_sim_time=true、Cyclone/domain44。
- 同文件：关闭Smac2D内部smooth_path避免切角，规划容差0.25→0.05m；实际BT不调用SmoothPath。局部控制不因此获得0.10m运行余量保证。近桌2.15/2.10/2.02位姿不能继续当成该运输profile的可达目标，禁止为强行近桌缩小包络。
- 新增 `scripts/check_path_clearance.py`：读取成功规划JSON，离线整段插值≤0.01m，未知和图外阻塞；扣除0.05m网格半对角线及半采样间隔，按1-Lipschitz距离界得到整段保守下界。门禁为中心净距下界≥0.55m，即扣0.45包络后≥0.10余量；失败退出1，不继承输入顶层passed。不连接ROS、不发任何action/速度。
- 新增 `tests/test_path_clearance.py`：6项，覆盖已保存失败路径、参考北绕、端点自由但中段穿墙、未知/图外、无效输入和配置包络约束。
- 新增 `maps/world_scene_path_clearance.json`：保存原真实路径与离线参考折线的净距结果、输入/地图SHA256和算法扣除项。
- `CMakeLists.txt` 安装检查器；`package.xml`声明python3-numpy/scipy/pil/yaml运行依赖。
- `WORLD_SCENE_ALIGNMENT.md`补充根因、参数范围、净距算法、A命令和验收边界；`README.md`及本报告顶部更新A真实静态对齐已通过、运输净距失败的当前状态。历史记录保留。

### 验证命令与结果

```bash
source /opt/ros/jazzy/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR=/tmp/bobac-nav-c/ros-logs
colcon --log-base /tmp/bobac-nav-c/log build \
  --base-paths bobac_robotac_src/src/bobac3_navigation --packages-select bobac3_navigation \
  --build-base /tmp/bobac-nav-c/build --install-base /tmp/bobac-nav-c/install
source /tmp/bobac-nav-c/install/setup.bash
python3 bobac_robotac_src/src/bobac3_navigation/tests/test_path_clearance.py
python3 bobac_robotac_src/src/bobac3_navigation/tests/test_world_scene_map.py
python3 bobac_robotac_src/src/bobac3_navigation/tests/test_navigation_config.py
ros2 launch bobac3_navigation world_scene_navigation.launch.py --show-args
```

- 独立构建最终退出0：1 package finished [0.26s]；新增6项+世界地图8项最终重跑通过，配置5项此前同轮通过，合计19项。配置测试有上游RewrittenYaml既有ResourceWarning，不影响结果。没有重新运行不受影响的ROS诊断测试。
- 原真实路径CLI预期退出1：856个整段采样，中心净距下界**0.176533m**，扣0.45包络余量**-0.273467m**；最差采样约(6.025,2.467)，接近竖墙北端。
- 参考折线(8,1.082)→(6.6,3)→(4.8,3)→(2.35,.6) CLI退出0：764个采样，中心净距下界**0.635107m**，扣包络余量**0.185107m**，比0.10m门槛多0.085107m；最差在暂驻终点。这里只证明静态参考折线存在余量，**不是修改后Nav2真实重规划结果**。
- 安装版 `ros2 run bobac3_navigation check_path_clearance.py --reference-north-detour --map bobac_robotac_src/src/bobac3_navigation/maps/world_scene.yaml --report /tmp/bobac-nav-c/installed-reference-clearance.json` 同结果退出0。无ROS节点被创建。
- Python AST全量、package XML及限定路径git diff --check通过；旧图哈希、当前场景/机器人来源哈希检查通过。临时结果在 `/tmp/bobac-nav-c/{old-path-clearance,reference-path-clearance,installed-reference-clearance}.json`，参数展开日志`clearance-launch-args.log`。未动A/B已有修改、共享.runtime和公共工具。

### 待A执行及成功判据

A安排应用最新world profile并重新完成10节点就绪，核对global costmap robot_radius=0.64、footprint为空、inflation=0.85、GridBased.smooth_path=false；仅规划到(2.35,.6,π)，再运行离线门禁：

```bash
# 以下是交接命令，C未运行：
export ROS_DOMAIN_ID=44 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python3 tools/world_navigation_check.py --report /tmp/world-replanned.json
python3 bobac_robotac_src/src/bobac3_navigation/scripts/check_path_clearance.py \
  --alignment-report /tmp/world-replanned.json \
  --map bobac_robotac_src/src/bobac3_navigation/maps/world_scene.yaml \
  --report /tmp/world-replanned-clearance.json
```

必须确认运行图与门禁完整规划图一致、运输姿态未变、定位仍匹配；新规划成功且净距门禁退出0/passed=true/中心下界≥0.55m后，才由A决定真实目标执行。A仍需检查终点位置与朝向、动态障碍、控制跟踪偏差、停止、无碰撞和实际到位；静态0.10m策略预留不是实测误差上限。更近桌边/抓取规划仍需独立验收。

需要A配合的具体事项：`tools/world_navigation_check.py`目前仅报告净距false但可能仍返回passed；该工具超出C范围，请A在流程中串接新检查器并传播非零退出，或将净距失败纳入公共工具失败条件。不要仅依赖原JSON顶层status。C未重启/启动栈、请求真实规划、发布定位、发送NavigateToPose或底盘命令。

可直接转发A：已依据你的plan-3d报告修正独立world规划约束，global改为0.64m圆形半径（当前0.45运输包络+0.10预留+0.09离散化缓冲），global/local膨胀0.85m，Smac2D关闭平滑，终点容差0.05m。新增离线check_path_clearance.py门禁，未知/图外阻塞并核验整段折线，中心净距下界必须≥0.55m。你保存的路径复算下界0.176533m失败；经(6.6,3)/(4.8,3)的参考折线下界0.635107m、扣运输包络后余0.185107m，仅离线可行性参考。独立构建与19项测试通过，未启动共享栈或发送目标。请应用最新world参数重规划到(2.35,.6,π)，将新报告交检查器，净距门禁退出0才进入运动验收；原world_navigation_check顶层passed不代表净距通过，需要你串接/修正公共工具。详情与命令见WORLD_SCENE_ALIGNMENT.md最新“北绕路径运输余量修正”一节。

## 2026-10-16：A 真实运动反馈后的局部包络、低速和跟踪约束

### 真实证据与纠正

读取 `reports/world-navigation-integration-feedback.json` 及其所列 `world-navigation-replanned.json`、`.clearance.json`、`world-navigation-staging-goal.json`、`world-navigation-after-cancel-stop.json`、`world-profile-runtime-parameters.json`。A已通过全局路径净距门禁（下界0.635107546m），但真实执行在(6.38018,2.42934)附近中心净距仅0.44528058m而取消，后来停止于(6.11727,2.84030)；当前A已停止导航栈、无活动目标。未取得导航目标成功或全程无碰撞证据。

**明确纠正本报告上一轮错误结论：`GridBased.smooth_path=false` 在A安装的Jazzy Smac2D中未声明、未生效，上一轮不能称已关闭平滑。** 源码始终调用内部smoother，真实支持的跳过机制为 `GridBased.smoother.max_iterations=0`。安装头文件 `SmootherParams::get`声明这个整数，官方Jazzy `smoother.cpp`的zero-iteration分支直接返回，不改路径。本轮已替换参数和修正文档；此前旧交接中要求检查smooth_path的命令已被本节取代。旧常规调参预设保持不变，仅修正独立world profile。

局部控制根因证据：此前local仍是较小矩形+padding，而global为0.64m圆；CostCritic.consider_footprint=true会进行多边形碰撞检查，不能把global圆的预留自动应用到local。旧MPPI每轴1.25m/s、平滑器1.5m/s及4m/s²，且30Hz控制/.05s模型/20Hz平滑器不同频；PathFollow权重20、Goal权重20与较弱PathAlign5使配置偏向追赶前方。不能仅靠全局路径成功推断实际轨迹。没有足够证据将偏差归因于单个权重或确认物理制动响应。

### 本轮确切修改（导航包内）

1. `params/bobac3_navigation_world_params.yaml`：local/global均 `robot_radius=0.64`、无footprint覆盖、padding=0、inflation=.85/scaling=3；不缩小已验证运输包络。
2. 同文件：MPPI和velocity_smoother每轴XY±0.08m/s、XY加减速±0.15m/s²，角速度±0.15rad/s、角加减速±0.30rad/s²；控制20Hz/模型dt=.05/平滑20Hz，60步3s预测，batch2000，XY std=.04/wz std=.06、visualize=false。合平移速度上界0.113137m/s，合加速度上界0.212132m/s²，不把每轴限幅误称合速度限幅。
3. 同文件：MPPI open_loop=false；平滑器CLOSED_LOOP、scale_velocities=true、odom_duration=.1s、velocity_timeout=.25s、deadband[.001,.001,.005]；停止阈值平移.015m/s、转动.02rad/s；failure_tolerance=0。实际odom时效性、控制周期与制动响应须A验证。
4. 同文件：圆形CostCritic `consider_footprint=false`、trajectory_point_step=1、collision_cost=1e6、weight6，识别中心253代价为碰撞；去除冗余ObstaclesCritic及未加载的PathAngle/PreferForward配置。PathAlign weight20/stride1/offset2/threshold.15；PathFollow weight5/offset2/threshold.2；Goal weight5/threshold.3，GoalAngle threshold.2，prune_distance=.8m（超过3s最大平移预测约.339m）。这些是预测评分和候选参数，不是硬跟踪保证，MPPI加权/滤波输出和执行仍需A检验。
5. 同文件：删除无效smooth_path，设置 `GridBased.smoother.max_iterations=0`、do_refinement=false。保留终点容差.05m及既有话题/TF/Twist/时间/DDS/domain。
6. 新增 `scripts/analyze_transport_trace.py`，由CMake安装：纯离线读取真实trace/world_xy与获准参考planner.points_xy，计入报告捕获的map_base起点连接，避免栅格首点量化误判；整段实际/参考净距均须≥.55m，最大点到参考折线几何偏差≤.05m，输入运动与参考规划亦须成功才passed。失败退出1。它不创建ROS节点、不发送action/命令，不证明速度/加速度或采样间物理轨迹。
7. 新增 `tests/test_world_transport_control.py`（9项）核对局部/全局包络、圆形评分、速度/加速度/反馈一致性、理想停车距离、跟踪配置、有效平滑参数和跟踪几何；修改 `tests/test_path_clearance.py` 改为核验真实有效平滑机制。
8. 新增 `tests/transport_plugin_check.cpp` 和 `tests/run_transport_plugin_check.py`：编译调用宿主已安装Jazzy真实库，无rclcpp::init/节点/DDS；验证max_iterations=0保留30个路径pose、圆形CostCritic将253/254/255判碰撞、0.64m离散圆的内切半径0.627703m及网格保守下界0.556992m，Omni正负方向每步加速度受限。算法界不包含物理跟踪误差。
9. 新增 `maps/world_transport_trace_review.json` 保存A旧真实失败轨迹的离线复核、地图和输入SHA256。`WORLD_SCENE_ALIGNMENT.md`与`README.md`更新最新结论、参数表、官方源码依据、A验收要求；本报告持续更新。

### 独立验证结果

```bash
source /opt/ros/jazzy/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR=/tmp/bobac-nav-c/ros-logs
colcon --log-base /tmp/bobac-nav-c/log build \
  --base-paths bobac_robotac_src/src/bobac3_navigation --packages-select bobac3_navigation \
  --build-base /tmp/bobac-nav-c/build --install-base /tmp/bobac-nav-c/install
source /tmp/bobac-nav-c/install/setup.bash
python3 bobac_robotac_src/src/bobac3_navigation/tests/test_world_transport_control.py
python3 bobac_robotac_src/src/bobac3_navigation/tests/test_path_clearance.py
python3 bobac_robotac_src/src/bobac3_navigation/tests/test_world_scene_map.py
python3 bobac_robotac_src/src/bobac3_navigation/tests/test_navigation_config.py
python3 bobac_robotac_src/src/bobac3_navigation/tests/run_transport_plugin_check.py
ros2 launch bobac3_navigation world_scene_navigation.launch.py --show-args
```

- 最终独立构建退出0，1 package finished [0.51s]。Python回归9+6+8+5=**28项通过**，新增配置5项先红后绿；真实库C++算法检查独立通过。编译适配安装版头文件布局和calculateMinAndMaxDistances返回pair后通过；只有GCC ABI说明note，不是运行错误。既有上游RewrittenYaml ResourceWarning仍存在。
- 旧真实轨迹44个样本，相对已获准参考折线最大偏差**0.345402m**；整段中心净距下界**0.440299m**，因此负向检查退出1。真实轨迹本身已失败，该数据不能变成成功验收。单条参考路线不代表后续所有实时重规划路径，A需提供实际重规划历史。
- 源码和已安装版 `ros2 run bobac3_navigation analyze_transport_trace.py` 对相同输入均预期退出1；无ROS初始化。输出 `/tmp/bobac-nav-c/transport-trace-review.json`、`installed-transport-trace-review.json`；包内副本保存来源哈希。
- 全部Python AST、manifest XML、限定路径diff --check通过；launch仅show-args，不执行节点。日志 `/tmp/bobac-nav-c/transport-test_world_transport_control.py.log`、`transport-test_path_clearance.py.log`、`transport-test_world_scene_map.py.log`、`transport-test_navigation_config.py.log`、`transport-run_transport_plugin_check.py.log`、`transport-launch-args.log`。
- 未重新测试与本轮无关的ROS诊断/relay节点；没有启动共享栈、调用真实规划/导航action、发布定位/速度、改共享构建或.runtime、改场景/公共工具、暂存提交或覆盖A/B工作。

### 要求A配合的运行验收

- A当前机器人已停在(6.11727,2.84030)附近，不能重启后盲用(8,1.082)初值。请由A按自己的流程reset到核验初值，或根据当前真实位姿初始化定位并重做规划。
- 在A调度的真实overlay构建/应用最新world参数，确认10个lifecycle就绪；记录运行local/global radius=.64、footprint=[]、inflation=.85/scaling3，MPPI与平滑器实际速度/加速度、CLOSED_LOOP和正确平滑参数的声明和值。CLOSED_LOOP必须有新鲜、sim stamp一致的odom；控制20Hz、60步/batch2000/每步代价评分应无持续计算超时。不得因参数不生效临时缩小运输包络。
- 重新仅规划到(2.35,.6,π)，通过实际整段净距门禁才安排低速实跑。录制带仿真时间戳的clock、odom、cmd_vel_nav、cmd_vel、实际plan重规划序列、真实位姿及运输姿态；用仿真时间核验每轴和合速度/加减速，不能用墙钟差替代。
- 理想最大对角速度刹停约.03017m，加.25s命令超时移动后约.05845m，尚未计入反馈延迟/执行惯性/定位误差；只是配置预算，A必须测定。全程中心净距下界≥.55m、跟踪试验偏差≤.05m；获准路线下界.635107m可为.05m跟踪+.03m定位/测量预算保留约.005m，但这些真实误差上界尚未证明。
- A原运动监控取消阈值.48m过晚，请在其公共工具范围内改为基于实测速度、制动距离、采样/反馈延迟的提前停车条件；低于.55m才取消不能保证全程达标。本会话不修改其工具。每次重规划后的实际路径要进入验收证据；离线单参考轨迹工具不能代替在线全程保护。
- 示例仅交A运行（输入须为新真实证据）：

```bash
export ROS_DOMAIN_ID=44 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 param get /planner_server GridBased.smoother.max_iterations  # 0，须已声明
ros2 param get /planner_server GridBased.smoother.do_refinement   # false
ros2 param get /local_costmap/local_costmap robot_radius           # .64
ros2 param get /local_costmap/local_costmap footprint              # []
ros2 param get /controller_server FollowPath.CostCritic.consider_footprint  # false
ros2 param get /controller_server FollowPath.ax_max                # .15
ros2 param get /velocity_smoother feedback                         # CLOSED_LOOP
python3 bobac_robotac_src/src/bobac3_navigation/scripts/analyze_transport_trace.py \
  --motion-report /tmp/new-transport-goal.json \
  --reference-report /tmp/new-transport-plan.json \
  --map bobac_robotac_src/src/bobac3_navigation/maps/world_scene.yaml \
  --report /tmp/new-transport-trace-review.json
```

只有SUCCEEDED/error_code0、真实xy/yaw到位/停止、运输姿态不变、全程余量与速度跟踪均通过才能计导航目标成功。低速/仿真实时率可能需要A延长观察timeout，不能以提高速度/减小包络替代。原x1.4物体的2.02名义IK经A证实link_2撞桌；x1.65是假设目标、物体未移动。近桌抓取不在这份运输配置的通过范围。

可直接转发A：C已统一world local/global为0.64m圆、padding0、inflation.85/scaling3，CostCritic改圆形中心代价并每预测步检查，避免原小矩形绕过运输预留。MPPI和平滑器每轴XY±.08m/s、加减速±.15m/s²，角速度±.15rad/s、角加减速±.3rad/s²，控制/模型/平滑统一20Hz；CLOSED_LOOP用真实odom；强化贴路径、减少远目标牵引。此前smooth_path=false确实无效，已改为Jazzy支持的GridBased.smoother.max_iterations=0并用安装库确认保留路径。独立构建、28项回归和真实库纯算法检查通过，未运行共享栈/发目标。新增analyze_transport_trace.py复核你旧轨迹最大偏离参考.345402m、中心净距下界.440299m，负向退出1。请按实际当前位姿/reset重做定位，验证真实新参数后重规划/低速执行，采集sim stamp速度/制动/重规划历史，维持中心下界≥.55m、试验跟踪偏差≤.05m；原.48m取消阈值须在你的工具里改成足够提前停车。预测评分不是物理保证，近桌目标不许缩包络强行通过。详见WORLD_SCENE_ALIGNMENT.md末节及本报告。

## 2026-10-16：最新低速初段反馈（C，离线修复完成；真实验收待A）

本节接续 `world-navigation-integration-feedback.json` 的新一轮反馈，覆盖前节“待低速运行”状态。A 的最新真实目标被预测跟踪保护取消，不能计作导航成功；记录包含制动后中心净距下界 1.437889m、最大偏差 0.037636m，原门槛仍为中心下界≥0.55m、偏差≤0.05m。当前机器人约 `(7.997826,1.119579,0.119910)`；下次由 A reset 或按实际位姿重定位。

已确认的机制与证据：

- 首次运行 `/plan` 首点 `(7.9500001,1.0500002)` 与实测起点之间 **0.0593316m**。A 为门禁加入的 `observed_start_connector_xy` 仅在报告中，原 Smac 返回给 MPPI 的路径没有该段，控制与验收参考并不相同。
- Jazzy `PathAlignCritic::score()` 在 `furthest_reached_path_point < offset_from_furthest` 时直接跳过；原 5cm 格点（斜边约7.1cm）、offset2 与低速短预测范围使初段路径对齐存在空档。Omni 的路径朝向评分关闭，途中 GoalAngle 不激活，也没有 Twirling 评分，初段转动缺少专门惩罚。它们是已确认的控制缺口；尚不能声称单凭离线证据解释了全部物理响应。
- 两级命令普通段周期中位数约 **0.033333s 仿真 / 0.050s 墙钟**；原生 odom 约0.016667s 仿真。上游 Jazzy smoother 使用 wall timer、每次增量按 `accel/frequency` 计算，`use_sim_time=true` 不会将该计时器改为仿真计时。MPPI 的固定 `model_dt=.05` 也不是实测执行周期。
- 分离预测停车前的数据后，普通 `/cmd_vel` 差分峰值约 `[.08240,.09981,.37372]`，`/cmd_vel_nav` 约 `[.07545,.15687,.28774]`；普通 native odom 约 `[.20715,.33806,1.23379]`。紧急覆盖段放大了全段导数，但不能把普通段也判为满足物理加速度限制。Twist 仍是接收时间，非精确源时间。
- 新离线工具及结果：`scripts/analyze_control_feedback.py`、`maps/world_startup_control_review.json`；保留输入 SHA、时间基准、非正间隔计数、全段/普通段分离和 .05/.1s 原生 odom 窗口均值。不改变任何验收判据。

正在实现的有界修正：仅 opt-in world profile 使用 `bobac3_navigation::ConnectedSmacPlanner2D` 包装上游 Smac2D，补准确请求起点，保留全部旧顶点，按≤1cm共线细分，不平滑/切角；连接不得超过一个地图格对角线，拒绝非法帧、非有限值及代价图中阻塞/未知采样。仍必须走 A 独立整段净距检查。PathAlign offset改1、PathFollow改10以保留约0.1m前瞻；加入途中 TwirlingCritic（目标位置容差内由上游跳过）。CLOSED_LOOP 保留，smoother odom窗口 .1→.05s、deadband三轴归零，保留初段微小纠偏。速度/加速度参数、双代价图0.64m运输圆、0.55/0.05门槛、话题/TF/domain44/Cyclone/仿真时间保持。

已完成红灯：新增 world 控制回归因缺少 ConnectedSmac 插件失败；连接段纯几何回归因缺少实现失败；反馈分段诊断2项因工具缺少失败。后续构建和完整离线验证结果在本节下方补记。没有启动共享栈、导航或发送命令，也没有修改场景和公共工具。

### 本轮最终变更与验证

本轮实际修改/新增文件（均位于导航包，另有本报告）：

- `CMakeLists.txt`、`package.xml`、`planner_plugins.xml`：编译/安装全局planner插件、仿真时钟平滑器，增加编译依赖与离线CTest入口。
- `src/connected_smac_planner.cpp`、`include/bobac3_navigation/connected_path.hpp`：补真实起点连接、保留旧顶点和共线细分，采样检查代价与取消请求；未变更原始地图。
- `src/sim_time_velocity_smoother.cpp`、`include/bobac3_navigation/feedback_step.hpp`：继承上游生命周期及闭环算法，替换为ROS仿真时钟timer，增量按实际仿真dt并以上限周期封顶；最新odom时间戳年龄超过.15s、超前超过.05s、时间戳零或时钟非前进时，已有命令输出零并清除缓存。无指令时不主动发布。故障零命令不属于普通限加速度制动证明。
- `launch/bobac3_navigation.launch.py` 新增默认false的 `use_sim_time_smoother` 选择；`launch/world_scene_navigation.launch.py` 固定传true。节点仍叫 `velocity_smoother`，话题remap与10节点lifecycle名单不变。**仅传world参数给旧入口不等于启用了新平滑器**；A须使用world入口或显式选择该参数。
- `params/bobac3_navigation_world_params.yaml`：ConnectedSmac插件、PathAlign offset1/PathFollow offset10、Twirling weight10、闭环窗口.05s/反馈年龄.15s/deadband0。原0.64m圆、速度/加速度上限、0.55/0.05验收门槛不变。
- `scripts/analyze_control_feedback.py`、`maps/world_startup_control_review.json`：不初始化ROS的历史反馈诊断，区分普通/停车后段及时间基准，含原始数据SHA。
- `tests/connected_path_check.cpp`、`feedback_step_check.cpp`、`planner_plugin_load_check.cpp`、`test_control_feedback.py`；扩展 `tests/transport_plugin_check.cpp`、`test_navigation_config.py`、`test_world_transport_control.py`：纯几何、时序故障判定、真实Jazzy插件加载/评分、配置与诊断回归。C++检查在Release下也不因NDEBUG失效。
- `README.md`、`WORLD_SCENE_ALIGNMENT.md`：最新状态、行为边界、来源失败、A验收命令。未修改公共工具、course_bringup或任何场景；共享工作区出现的其他改动属于其他会话，未覆盖或撤销。

独立验证命令（C已执行，不启动ROS节点/栈）：

```bash
source /opt/ros/jazzy/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR=/tmp/bobac-nav-c/ros-logs
colcon --log-base /tmp/bobac-nav-c/log build \
  --base-paths bobac_robotac_src/src/bobac3_navigation --packages-select bobac3_navigation \
  --build-base /tmp/bobac-nav-c/build --install-base /tmp/bobac-nav-c/install \
  --cmake-args -DBUILD_TESTING=ON
source /tmp/bobac-nav-c/install/setup.bash
ctest --test-dir /tmp/bobac-nav-c/build/bobac3_navigation --output-on-failure
python3 bobac_robotac_src/src/bobac3_navigation/tests/run_transport_plugin_check.py
ros2 launch bobac3_navigation world_scene_navigation.launch.py --show-args
ros2 run bobac3_navigation analyze_control_feedback.py \
  --motion-report reports/world-low-speed-motion.json \
  --report /tmp/bobac-nav-c/installed-startup-review.json
```

结果：

- 最终独立构建退出0，1包完成，1.69s；安装有 `libconnected_smac_planner.so` 与 `sim_time_velocity_smoother`。
- CTest **7组通过、1组失败**。Python共32项，31项通过，唯一失败为原有 `test_geometry_evidence_matches_current_scene_and_asset_files`；其余新旧离线用例通过，三个C++检查通过。扩展world实际RewrittenYaml/插件registry检查后单独重跑config组，6项通过。完整日志 `/tmp/bobac-nav-c/startup-final-ctest.log`，构建日志 `startup-final-build.log`。
- 失败原因不是测试放宽：当前 `scenes/test_scene_bobac_6.0.course.usda` SHA256=`43ae57fc3405a83769731fa8d4ce4eb01aede5524b76099b2f148a09a0cde88a`，几何记录是`9383aec555c3111e13a12e7811f46bae9566ee94a18a3dd4d793022b31c86237`。**未改写哈希或跳过用例**；A需确认加载场景并复核碰撞顶点与运输姿态。哈希不同本身既不能证明几何相同，也不能证明几何不同。
- 真实安装库算法检查通过：复现旧offset2在furthest1时零评分；新offset1启用，细分路径评分偏好沿线预测，Twirling惩罚途中转动；原Smac零迭代、圆cost253/254/255拦截、Omni正负加速度预测检查仍通过。日志 `startup-real-algorithms.log`；它们不是物理执行测试。
- 安装诊断输出与源结果 `cmp` 一致；`--show-args` 退出0；AST/XML和scoped `git diff --check` 通过。
- 保存的旧取消目标仍被 `analyze_transport_trace.py` 判失败并退出1；几何下界和.05m门槛没有变更，未把取消目标误算成功。输出 `startup-old-trace-gate.json`。
- 上游 RewrittenYaml 仍有既有unclosed-file ResourceWarning、独立算法编译有GCC pair ABI提示，未作为失败。未重跑需创建ROS节点的 `test_navigation_diagnostics.py`；未运行共享导航、目标、底盘命令、暂停/reset或真实规划。未声称整个测试集全绿或真实导航通过。

### A 下一轮验收与可转发交接

> C 已提交到共享文件的修正（未git暂存/提交）：world用ConnectedSmac补准确起点与≤1cm共线采样；PathAlign offset1、PathFollow offset10及途中Twirling；world launch自动选择同名lifecycle的仿真时钟CLOSED_LOOP适配器，odom窗口.05s、年龄门禁.15s、deadband0。速度/加速度标称上限、0.64m运输圆与中心净距≥.55m/偏差≤.05m门槛保持。独立构建、插件加载与新增离线检查通过；完整CTest只有场景来源哈希用例失败，当前course SHA43ae57…与旧几何证据9383ae…不同，必须先确认加载场景和碰撞顶点/包络，不能直接替换哈希。请A在自己的overlay构建，用当前真实位姿重定位或reset后启动world入口，确认10节点active、新planner类型和日志`Simulation-clock CLOSED_LOOP smoother`。激活时参数不可动态修改，需停用后重配。核实真实 `/plan` 首点及连接，不仅在JSON补字段；每条重规划先过整段净距门禁，再按现有预测保护试验。输出应约20Hz仿真，普通sim-stamped odom/两级Twist与紧急覆盖分开，核验初段沿参考、无途中自转漂移、反馈年龄、仿真dt、物理速度/加减速/制动、最终action SUCCEEDED/error0及到位停稳。控制器仍用上游调度，实际性能和物理响应未由C证明。保留SDK源stamp0和请求/接收时钟区间，不放宽保护或删除初段样本；失败即保留失败。A的公共运动工具/场景变更仍由A负责。

详细A命令和成功判据见导航包 `WORLD_SCENE_ALIGNMENT.md` 最后一节。新平滑器的时钟回退/反馈过期停车逻辑只做了纯函数验证和编译；真正ROS定时器、生命周期转换、暂停/恢复、普通闭环制动与目标执行均待A验证。没有真实修改后仿真证据，导航运行验收保持未通过。
