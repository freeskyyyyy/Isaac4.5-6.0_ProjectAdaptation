> 历史快照：已于 2026-10-14 16:10 归档。最新进度请查看 [MIGRATION_TODO.md](../../MIGRATION_TODO.md)。

# DGX Spark 单容器迁移项目进度报告

**报告快照：2026-10-14 16:06:55，Asia/Shanghai（UTC+08:00）。** 最新待办清单、Compose 解析和镜像构建状态于 16:06 只读核对；资产、审计、兼容性与宿主预检引用已有证据。主实施工作仍在继续，快照后的结果应以新增验收证据更新。

完成项与后续顺序统一记录在 [持续更新的迁移待办清单](../../MIGRATION_TODO.md)：01–08 已完成，09 课程镜像构建进行中，10–15 待完成。清单的配置完成项不代表实际挂载、GUI 或闭环运行验收通过。

## 目标与当前结论

在 DGX Spark 上新建一个课程容器，容器内安装并运行 Isaac Sim 6.0.0、ROS 2 Jazzy、Nav2、MoveIt 2、YOLOE/PyTorch 和全部课程依赖，完成“导航到桌边 → 识别目标 → 接近并抓取 → 提升物体”。最终验收要求固定初始条件下连续成功三次，并在容器重启后复现。项目没有 RTX 4090 部署任务。

当前已完成容器与启动工具的实现、主要 Jazzy 兼容修复、宿主机 9 个 ROS 包的构建预检、官方 ARM64 基础镜像下载和兼容性检查。用户已授权使用完整资产 ZIP，资产已同步到项目；官方 6.0 SDK 的静态场景审计不再报告缺失必需资产。**课程镜像仍在 Dockerfile 构建的 apt 阶段；课程容器、程序目录实际挂载、图运行、传感器和端到端物理闭环尚未验收。**

## 已完成的实现

| 工作项 | 已落地内容 | 文件依据 |
| --- | --- | --- |
| 单容器配置 | 基础镜像固定为 Isaac Sim 6.0.0 官方摘要；安装 Jazzy、Nav2、MoveIt 2 及课程依赖；视觉 Python venv 与 ROS 系统环境位于同一容器 | `Dockerfile`、`compose.yaml`、`docker/entrypoint.sh` |
| 桌面与程序目录绑定 | 官方镜像的完整程序目录已复制到 `/home/nvidia/桌面/IsaacSim-6.0.0`；SDK 读写绑定到 `/isaac-sim`，完整宿主桌面只读绑定到 `/workspace/Desktop`，项目读写绑定到 `/workspace/course`；配置解析通过，实际挂载和 GUI 仍待验收 | 桌面程序目录、`compose.yaml`、`tools/course.sh` |
| 视觉依赖声明 | 声明 PyTorch 2.9.0、torchvision 0.24.0、CUDA 13.0 wheel 来源和 Ultralytics 等版本；构建时计划导出 apt/pip 解析清单 | `docker/vision-requirements.txt`、`Dockerfile` |
| 容器操作入口 | 提供构建、启动、兼容检查、编译、审计、迁移、仿真、ROS、readiness、视觉检查、抓取及依赖清单导出命令；启动准备阶段的 `--mount` 已移至镜像名称前，待实际容器验证 | `tools/course.sh` |
| 场景迁移工具 | 增加 USD 依赖/图审计、路径草稿生成、迁移和仿真启动工具；USD CPU 审计可使用打包库而不启动 Kit | `tools/scene_audit.py`、`tools/migrate_scene.py`、`tools/run_sim.py`、`docker/usd-python.sh` |
| 资产与工作副本修复 | 完整资产同步到项目；Bobac 引用重定向到项目相对路径；inactive 分支资产不计为必需依赖；缺失虚线 JPEG 在工作副本用浅色材质替代并记录 | `isaacsimassets-dev/`、`scenes/test_scene_bobac_6.0.draft.usda`、`reports/scene-audit.json` |
| Nav2 兼容修复 | 调整插件名称、BTCPP v4 格式、行为树安装/加载路径与进度检查器配置 | `bobac_robotac_src/src/bobac3_navigation/` |
| MoveIt 兼容修复 | 调整运动学参数、规划适配器配置及 Jazzy 轨迹 API/字段使用 | `bobac_robotac_src/src/eco65b_moveit_config/`、`eco65b_moveit_grasp/src/grasp_moveit_node.cpp` |
| 数据接口修复 | 对齐机械臂 JointState 话题与传感器 QoS，修正 `16UC1` 深度毫米到米的转换及视觉包合并安装路径 | `joint_state_filter/`、`yoloe_grasp/` 及相关 launch 配置 |
| 统一 ROS 启动 | 新增 `course_bringup` 包；先检查仿真数据和 TF，通过后启动导航、规划、视觉与夹爪节点；显式任务前检查 Action 服务 | `course_bringup/launch/course.launch.py`、`course_bringup/course_bringup/readiness.py`、`tools/action_readiness.py` |
| 视觉检查工具 | 编写 CUDA 运算、torchvision NMS、cv_bridge/NumPy 互通及 YOLOE 推理检查 | `tools/vision_check.py` |

上述是实现完成情况，不代表相关节点在课程容器中的运行验收已经通过。源代码原件保留于 `reports/original_source/`，入口 USD 保留；`scenes/test_scene_bobac_6.0.draft.usda` 已完成路径与外观替代处理并通过 6.0 静态审计，仍不是运行验证完成的迁移场景。

## 已有验证证据与适用范围

1. **宿主 Jazzy 构建预检通过。** `reports/host-jazzy-build.log` 最新结果为 `Summary: 9 packages finished [16.2s]`；`reports/host-jazzy-build.json` 明确记录 `host preflight only; container execution pending`。该结果证明当前源码可在此次宿主 Jazzy 预检环境编译，不能替代 ARM64 目标容器内的干净构建。
2. **课程 launch 参数解析通过。** 在宿主 Jazzy 与 `/tmp/bobac-course-jazzy.DtiZ2T/install/setup.bash` 环境执行 `ros2 launch course_bringup course.launch.py --show-args`，返回码为 0，解析出 `readiness_timeout=60` 和 `rviz=true`。这没有启动或验证仿真、Nav2、MoveIt、视觉节点。
3. **官方 ARM64 基础镜像已下载。** 基础镜像为 `nvcr.io/nvidia/isaac-sim:6.0.0@sha256:68735a60b6c15c85e0dd0098570c6d2cc79e928f2d068ce2790aa43284ac165d`；15:55:06 的 `docker image inspect` 返回 `arm64 linux` 和该摘要。课程镜像构建日志 `reports/container-build.log` 已进入 apt 阶段，但尚无课程镜像构建完成结果。
4. **完整资产来源已校验并同步。** 用户已授权解压和使用桌面资产 ZIP。`reports/assets-source.json` 记录 ZIP 大小 `1409011447` 字节、616 条目、状态 `complete_zip_crc_passed`，源目录为桌面 `isaacsimassets-dev`，项目目录为本工作区 `isaacsimassets-dev`。所有 ZIP 条目 CRC 校验通过。`reports/assets-extraction.json` 中截断 ZIP 恢复 429 文件的 `partial_recovery` 是历史阶段记录，不再作为当前资产来源结论或阻塞。
5. **官方 6.0 静态场景审计通过，运行待验证。** 最新 `reports/scene-audit.json` 审计对象为迁移工作副本，记录 `audit_usd_version=[0,25,11]`、`audit_sdk_version=6.0.0-rc.59+release.41464.5f2772bc.gl`、`status=static_audit_passed_runtime_pending`、`missing_assets=[]`、`errors=[]`，仍明确 `draft_is_validated_scene=false`。Bobac 资产已解析；mobot 及其 Kaya 引用属于 inactive 分支，标记 `inactive_not_required`。缺失的 xuxian 虚线 JPEG 在工作副本用浅色实心材质替代，几何保留，并记录 `cosmetic_fallback`。静态审计不能证明 ROS 图、传感器或物理执行正常。
6. **兼容性检查通过。** `reports/compatibility-check.log` 末尾明确为 `System checking result: PASSED`，并显示兼容性检查应用 `app ready`。当前 GB10 宿主驱动 `580.82.09` 下该项检查通过，目前无需升级宿主驱动。检查日志未检测到显示设备，因此课程 GUI/显示与实际场景加载仍需单独验证。
7. **桌面 Isaac Sim 程序目录已落地，挂载配置解析通过。** `/home/nvidia/桌面/IsaacSim-6.0.0/VERSION` 与上述官方 6.0 SDK 版本一致，`python.sh` 已存在且可执行。Compose 将 `${ISAAC_SIM_HOST_DIR:-../IsaacSim-6.0.0}` 读写绑定到 `/isaac-sim`，将 `${COURSE_DESKTOP_DIR:-..}` 所指完整宿主桌面只读绑定到 `/workspace/Desktop`，将当前项目读写绑定到 `/workspace/course`；16:06 的 `docker compose config --quiet` 返回码为 0。实际容器挂载、用户读写权限和 GUI 运行仍待验收。
8. **课程镜像仍未构建完成。** 16:06 的本地镜像列表没有 `bobac-course:isaac6.0-jazzy-spark`；`reports/container-build.log` 已到 `#10` apt 下载阶段，正在准备 ROS/Jazzy、Nav2、MoveIt 及相关系统依赖。配置解析与下载进度不能作为课程镜像或容器运行成功证据。

目前尚无课程镜像构建完成、课程容器内 Jazzy 干净构建、视觉功能检查、真实仿真 ROS readiness 或闭环运行的通过证据，也没有三次连续与重启复现记录。隔离 ROS 域下的 readiness 负向测试不作为真实仿真数据就绪或闭环成功证据。

## 进行中的工作与待验证项

- **课程镜像正在构建。** 官方基础镜像已就绪，`reports/container-build.log` 当前处于 `#10` apt 下载 ROS/Jazzy、Nav2、MoveIt 及相关依赖阶段，尚无课程镜像。后续需完成 pip/视觉依赖和课程包构建，保存完整结果，再启动 `bobac-isaac6-course`。
- **程序目录挂载与权限需实测。** 桌面程序目录已复制、Compose 已配置绑定及宿主组；仍须验证容器用户实际读写、缓存路径、解释器/启动脚本和挂载后的 Isaac 程序运行。
- **场景图、传感器及物理执行待运行验证。** 必需资产的静态缺失阻塞已解除；TF、JointState、时钟、里程计、相机/雷达输出、图兼容和机器人动力学仍须在 6.0 场景中实测，不能据静态审计宣称迁移完成。
- **运行时资产解析仍须验证。** `OmniPBR.mdl` 标记 `resolver_required`，6.0 默认环境 URL 标记 `remote_unverified`；须在场景加载时确认解析和远程可用性。虚线材质替代的视觉效果也需实际查看。
- **课程功能与重复性尚未验收。** Nav2、MoveIt、YOLOE、夹爪及完整任务都需要容器运行结果，随后才能做连续三次和重启复现。

## 验收矩阵

状态含义：**通过**表示相应范围已有成功证据；**进行中**表示正在执行且尚无完成结果；**待验证**表示缺少对应运行结果。已解除的历史资产阻塞不再列为当前阻塞。

| 验收项 | 状态 | 当前依据或未满足条件 |
| --- | --- | --- |
| 官方 6.0 基础镜像下载、摘要与 ARM64 架构 | 通过 | 本地 `docker image inspect` 返回 arm64/linux 和固定摘要 |
| 完整资产 ZIP CRC 与项目同步 | 通过 | 1409011447 字节、616 条目 CRC 通过，资产来源报告已保存 |
| 官方 6.0 工作副本静态引用审计 | 通过 | `missing_assets=[]`，状态明确为运行待验证；含 cosmetic fallback |
| GB10 宿主兼容性检查 | 通过 | 官方 6.0 检查末尾 PASSED，当前无需升级驱动 |
| 宿主 Jazzy 9 个课程包编译 | 通过 | 宿主构建日志和 JSON；仅限宿主预检 |
| 宿主课程 launch 参数解析 | 通过 | `--show-args` 返回码 0；仅限参数解析 |
| 新课程 Dockerfile 镜像构建 | 进行中 | 日志 `#10` apt 下载阶段，本地尚无课程镜像 |
| SDK/桌面/项目挂载配置解析 | 通过 | `docker compose config --quiet` 返回码 0；仅限配置解析 |
| SDK/桌面/项目实际挂载、权限与单容器运行 | 待验证 | SDK rw、桌面 ro、项目 rw 已配置，实际容器挂载尚未验收 |
| 容器内全课程依赖安装与干净编译 | 待验证 | 需要目标容器日志和 apt/pip 锁定清单 |
| Isaac Sim 6.0 GUI/显示与课程场景加载 | 待验证 | 兼容检查通过，课程 GUI 与场景仍需运行验证 |
| PyTorch CUDA、NMS、YOLOE 与 cv_bridge 互通 | 待验证 | 检查工具已编写，尚无容器执行结果 |
| 机器人/传感器正常、ROS 图迁移与物理行为 | 待验证 | 静态引用审计通过，图和物理运行尚未验收 |
| 仿真时钟、JointState、TF、相机/雷达 readiness | 待验证 | 尚无真实 6.0 仿真数据就绪的通过证据 |
| Nav2 运行、BT 加载和到桌边导航 | 待验证 | 配置已改，实际场景导航未验收 |
| MoveIt 规划、夹爪控制、抓取与提升执行 | 待验证 | 源码编译预检通过，实际仿真接口未验证 |
| 导航 → 识别 → 抓取 → 提升完整流程 | 待验证 | 没有端到端验收证据 |
| 固定初始条件下连续三次成功 | 待验证 | 完整流程尚未完成一次验收 |
| 容器重启后完整流程复现 | 待验证 | 尚无课程容器重启后的闭环验收 |

## 下一步与交付证据

1. 完成正在进行的课程镜像构建，保存构建与依赖解析清单；验证桌面程序目录权限和实际挂载，启动新课程容器并在容器中干净编译 9 个课程包。
2. 在实际挂载的程序环境验证 Isaac Sim GUI/显示与课程场景加载，执行 `vision-check`，记录 PyTorch/YOLOE 和 ROS 图像接口结果；沿用当前驱动，依据后续真实错误再判断兼容处理。
3. 基于已通过静态审计的工作副本完成运行迁移，在 6.0 中核对继承图、传感器、JointState、时钟、TF 和物理行为，确认 resolver/远程资源与浅色材质替代效果，保留原件。
4. 启动仿真与课程 ROS 栈，通过 readiness，再分别验证 Nav2、MoveIt、检测 Action 与夹爪接口，并保存分层结果。
5. 在固定条件下完成完整流程，记录每阶段结果；连续成功三次后重启容器复现，保存运行日志、图像/状态及耗时记录。

实施顺序和最终通过标准参见 [DGX Spark 迁移计划](../../DGX_SPARK_MIGRATION_PLAN.md)，逐项完成状态参见 [迁移待办清单](../../MIGRATION_TODO.md)。本报告已归档，不再更新；最新进度统一见 MIGRATION_TODO.md。运行验收须以课程容器内的实际结果为依据。
