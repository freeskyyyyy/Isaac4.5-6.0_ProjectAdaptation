# DGX Spark 单容器迁移与端到端验收计划

## 目标与范围

在 DGX Spark 上新建一个独立容器，容器内安装并运行 Isaac Sim 6.0.0、ROS 2 Jazzy、Nav2、MoveIt 2、YOLOE、PyTorch，以及课程所需的全部系统与 Python 依赖。完成以下完整流程：机器人导航到桌边 → 视觉识别目标 → 机械臂接近并抓取 → 提升物体。

验收必须在固定初始条件下连续成功三次，并在容器重启后可以复现。宿主机提供 Docker、NVIDIA 容器运行时、GPU 驱动、显示服务，以及用户明确要求挂载的完整 Isaac Sim 安装目录 `/home/nvidia/桌面/IsaacSim-6.0.0`（容器内路径 `/isaac-sim`）；ROS、模型环境及课程依赖留在容器内。不包含 RTX 4090 适配工作。

## 当前基线与状态

- 源项目位于 `bobac_robotac_src/src`，USD 入口为 `test_scene_bobac.usd`。
- 已选定官方多架构镜像摘要 `nvcr.io/nvidia/isaac-sim:6.0.0@sha256:68735a60b6c15c85e0dd0098570c6d2cc79e928f2d068ce2790aa43284ac165d`；目标只构建和运行 ARM64。
- 工作区已有 Dockerfile、Compose、容器入口脚本和视觉依赖清单作为实施脚手架。它们尚不能证明 Spark 上镜像构建、驱动兼容或课程闭环已通过。
- `reports/host-jazzy-build.log` 的最新宿主机 Jazzy 预检已成功编译 9 个包。首次 ROS Action 接口生成失败由中文工作路径引起，移至 ASCII 临时目录后解决；随后发现的 MoveIt 轨迹字段改名错误也已修复。该结果仅代表宿主机预检通过，不等于目标 ARM64 容器构建验收；容器内仍须干净构建和复核。
- `reports/assets-source.json` 记录桌面 ZIP 的 616 个条目 CRC 全部通过，资产已同步到项目 `isaacsimassets-dev`。最新 `reports/scene-audit.json` 的 Isaac Sim 6.0 静态审计结果为 `missing_assets=[]`。`mobot` 和 Kaya 位于 inactive 分支，不是活跃依赖；缺失虚线 JPEG 已在迁移副本以浅色替代并记录。此前资产阻塞已消除，但场景图加载和完整闭环仍待运行验证。
- 当前记录的宿主机驱动为 `580.82.09`；虽然低于 Isaac Sim 6.0 Spark 文档测试版本 `580.142`，但 `reports/compatibility-check.log` 的实际兼容性检查结果为 `PASSED`，当前无需改驱动。仍以容器内实际启动结果作为最终运行验证。
- 官方镜像已下载，`reports/compatibility-check.log` 明确显示兼容性检查 `PASSED`，当前无需更改宿主机驱动。官方 SDK 约 17 GB 已复制到 `/home/nvidia/桌面/IsaacSim-6.0.0`，Compose 已加入其到 `/isaac-sim` 的绑定挂载；仍需验证挂载权限和容器内运行。`tools/setup_desktop_sdk.sh` 负责初始化桌面 SDK。课程镜像当前处于 apt 安装阶段，目标容器尚未完成运行验收。

## 实施阶段

### 1. 锁定 ARM64 基础镜像并完成兼容性检查

1. 在 DGX Spark 确认 CPU 架构、系统版本、GPU、驱动版本、Docker 与 NVIDIA Container Toolkit 可用状态；复核 `reports/compatibility-check.log` 的 `PASSED` 记录。
2. 使用已下载的官方 Isaac Sim 6.0.0 摘要镜像，记录 Docker 实际选择的平台和镜像摘要，确认运行架构为 ARM64。
3. 核对镜像的 Ubuntu/Python/CUDA 基线与 ROS 2 Jazzy 的 Ubuntu 24.04 支持条件；锁定容器 apt 与 pip 依赖版本，并保存解析后的依赖清单。
4. 验证宿主桌面 SDK 已完整挂载到容器 `/isaac-sim`，容器用户具备所需读写/执行权限；运行 `tools/setup_desktop_sdk.sh` 初始化并验证 SDK。随后检查 GPU 可见、Isaac Sim 启动及 GUI/显示，并记录日志。现有兼容性检查已通过，若运行时仍有错误则按日志定位。

**验收条件：** ARM64 平台信息和镜像摘要可复核；SDK 目录正确挂载且权限满足启动需求；初始化脚本成功；容器内 GPU 可见；Isaac Sim 6.0.0 能启动并打开测试场景；驱动兼容性结论有日志佐证。

### 2. 审计并迁移 USD 场景副本

1. 以 `reports/scene-audit.json` 的 6.0 静态审计结果为基线，从入口 `test_scene_bobac.usd` 复核 USD 资产引用、机器人模型、材质/网格、Action Graph、传感器及 ROS 发布/订阅配置。
2. 确认 `isaacsimassets-dev` 已同步资产可由容器/SDK 解析；将审计发现的虚线 JPEG 浅色替代保留在迁移副本并记录。`mobot` 与 Kaya 属于 inactive 分支，不作为当前活跃依赖处理。
3. 检查 Bobac 资产继承层中承载 ROS 行为的图和传感器配置。对缺失、旧版或 6.0 不兼容项记录名称、依赖、修复方式和验证结果。
4. 保留所有源 USD 和源代码原件；在独立迁移副本中重定向资产路径、升级/修复图和传感器配置，并在容器内验证无 unresolved reference、机器人可见、传感器持续发布数据。

**验收条件：** 静态审计 `missing_assets=[]`；迁移副本依赖可从容器内解析；机器人与传感器正常；关键 Action Graph 节点、输入输出及 ROS 话题有审计记录；原件未被覆盖。静态资产审计通过不代表场景图已运行或闭环已完成。

### 3. 迁移 ROS 2 Jazzy 下的导航、规划与接口

1. 修复工作区在 Jazzy 上的构建问题，包括 `eco65b_grasp_interfaces` 的 Action 定义/接口生成错误；在 ARM64 容器中干净构建全工作区并保存日志。
2. 检查 Nav2 插件名称与 Jazzy 版本兼容性，核对 BTCPP v4 的节点注册、行为树 XML 语法、运行时插件加载和行为树安装路径。
3. 检查 MoveIt 2 的机器人模型、SRDF、关节组、控制器接口、运动学插件参数和规划适配器配置；确保机械臂状态与仿真控制接口一致。
4. 对照 Isaac Sim ROS 图与 ROS 节点逐项核对话题名、消息类型、frame_id、TF 树、QoS、`use_sim_time` 和时钟来源。固定传感器与 Nav2/MoveIt 使用的仿真时间策略。
5. 分别做接口构建、Nav2 lifecycle/插件加载、行为树加载、TF 连通、MoveIt 规划和控制器执行检查；每项保存命令、结果和日志位置。

**验收条件：** ARM64 容器内全工作区可重复构建；Nav2 插件与 BT 可加载并完成目标导航；MoveIt 可规划且仿真控制器执行到位；关键话题、TF、QoS 和时间配置一致且有记录。

### 4. 单容器进程布局与统一启动

1. 在一个容器中分别运行 Isaac Sim 和 ROS 2 课程节点；为 YOLOE 建立独立视觉 Python venv，使用容器内 PyTorch/CUDA，并通过固定入口选择该解释器，避免污染 ROS 系统 Python。
2. 配置容器内 ROS 环境、DDS、共享内存和显示；使用统一启动入口管理进程与日志，保证异常退出可见且重启流程确定。
3. 启动入口按 readiness gate 顺序等待：Isaac 场景和仿真时钟就绪 → 传感器图像/点云等数据到达 → 必需 TF 可查询 → Nav2 与 MoveIt lifecycle/服务就绪 → YOLOE 模型加载完成。超时应报告具体缺项并退出，不得静默进入抓取流程。
4. 仅在 readiness 全部通过后显式触发一次课程任务；状态机应记录目标、导航结果、视觉检测结果、机械臂接近、夹爪闭合和提升结果。不得因启动节点而自动反复抓取。
5. 验证停止、启动和容器重启后资源、缓存、模型路径、配置和日志行为符合预期；容器外不依赖临时手工启动的 ROS 节点。

**验收条件：** 单容器入口可报告每个 readiness 条件；未就绪时安全失败；就绪后显式触发任务可完成全流程；容器停止/重启后的启动行为可解释且可重复。

### 5. 分层验收、依赖锁定与交付

按以下次序逐层记录结果，任何一层失败都先定位，不跳过直接宣称闭环成功：

1. **镜像层：** ARM64 摘要、驱动/GPU、Isaac Sim 启动与场景加载。
2. **数据层：** USD 引用完整、传感器数据稳定、时钟与 TF 连续。
3. **软件层：** ROS 工作区干净构建；Nav2、行为树、MoveIt、YOLOE 分别通过启动/功能检查。
4. **集成层：** 显式任务完成“导航到桌边→识别→接近→抓取→提升”，保存每步状态和关键话题/图像/日志证据。
5. **稳定性层：** 固定机器人初始姿态、目标物体、桌面/地图、随机种子与启动参数，连续完成三次；之后重启容器，重新通过 readiness gate 并复现完整流程。记录每次成功/失败、耗时及失败阶段。

交付以下材料：镜像与 apt/pip 依赖锁定清单、ARM64 构建和运行说明、迁移后资产清单及 USD 引用审计结果、ROS 图/话题/TF/QoS 对照表、统一启动与 readiness 使用教程、每层验收日志和三次连续运行及重启复现记录、已知限制和未关闭问题。

**最终通过标准：** 固定初始条件下完整流程连续三次成功，容器重启后再次成功；所有依赖和运行入口均在容器内，关键资产引用完整，验收证据随交付材料保存。

## 当前阻塞与处理顺序

1. **场景图及完整闭环待运行验证。** 资产 ZIP 的 616 条目 CRC 全部通过，资产已同步，静态审计 `missing_assets=[]`；当前资产缺失阻塞已解除。仍需在 Isaac Sim 6.0 中实际加载场景、检查图和传感器并完成运行时验证。
2. **目标容器构建尚待验证。** 宿主机预检已成功编译 9 个包；中文路径导致的 Action 接口生成问题和 MoveIt 轨迹字段改名问题均已修复。课程镜像正在 apt 安装，仍需在目标 ARM64 容器中干净构建，不能用宿主机结果替代。
3. **桌面 SDK 挂载与运行待验证。** 官方 SDK 已复制，Compose 已绑定 `/home/nvidia/桌面/IsaacSim-6.0.0` 到 `/isaac-sim`；仍需核实权限、执行 `tools/setup_desktop_sdk.sh` 并在容器中启动。
4. **端到端执行与稳定性尚未验收。** 当前配置文件和局部代码只能作为准备工作；在前述阻塞关闭、全链路首次通过前，不开始三次连续与重启复现的最终计数。

## 官方参考

- [Isaac Sim 6.0 migration strategy](https://docs.isaacsim.omniverse.nvidia.com/latest/migration_guides/isaac_sim_6_0/migration_strategy.html)
- [Isaac Sim 6.0 container installation](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/installation/install_container.html)
- [ROS 2 OmniGraph migration](https://docs.isaacsim.omniverse.nvidia.com/latest/migration_guides/isaac_sim_6_0/ros2_omnigraph_migration.html)
- [ROS 2 sensor graph migration](https://docs.isaacsim.omniverse.nvidia.com/latest/migration_guides/isaac_sim_6_0/ros2_sensor_graph_migration.html)
- [Isaac Sim 6.0 system requirements](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/installation/requirements.html)
