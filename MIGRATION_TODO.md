# DGX Spark 课程迁移待办清单

更新时间：2026-10-14 16:28（Asia/Shanghai）

目标：只在 DGX Spark 的新课程容器内运行 Isaac Sim 6.0、ROS 2 Jazzy、Nav2、MoveIt2 和 YOLOE，跑通导航、视觉定位、抓取、提升的完整课程。

本文件是唯一持续更新的进度入口，统一记录待办状态、验证证据、问题和更新历史。迁移设计依据另见 [迁移计划](DGX_SPARK_MIGRATION_PLAN.md)。

规则：`[x]` 表示该项已有验证证据；`[ ]` 表示进行中或待验证。配置文件已写好不等于运行验收通过。每完成一项，更新本清单的勾选、时间、证据及相关问题；不再维护另一份重复的进度报告。

## 已完成

- [x] **01. 盘点机器、课程和迁移范围。** 确认 DGX Spark / ARM64 / GB10，保留原始场景及 Action 接口。本轮只验收 Spark。
- [x] **02. 下载官方 Isaac Sim 6.0 ARM64 镜像并固定摘要。** 镜像摘要见 `Dockerfile`。官方镜像内 `VERSION` 实际为 `6.0.0-rc.59+release.41464.5f2772bc.gl`，保留该版本记录。
- [x] **03. 将完整 Isaac SDK 放到桌面。** 路径：`/home/nvidia/桌面/IsaacSim-6.0.0`；完成记录：`reports/sdk-desktop-setup.log`。
- [x] **04. 执行官方 GPU 兼容性检查。** 当前驱动 `580.82.09`，检查结果 `PASSED`；证据：`reports/compatibility-check.json`、`reports/compatibility-check.log`。GUI 尚未验证。
- [x] **05. 校验并补齐课程资产。** 完整 ZIP 的 616 个条目通过 CRC 校验；桌面解压内容已复制到项目 `isaacsimassets-dev/`；证据：`reports/assets-source.json`。
- [x] **06. 完成原始 USD 审计及独立迁移草稿。** 使用真实 6.0 SDK 审计；草稿有效的本地资产缺失为 0，停用机器人引用不计入运行依赖。缺失的地面标线贴图采用纯色替代。远程环境资产和图执行仍待运行验证。证据：`reports/scene-audit.json`；草稿：`scenes/test_scene_bobac_6.0.draft.usda`。
- [x] **07. 完成 ROS 课程代码首轮 Jazzy 适配及宿主机编译预检。** 9 个包编译通过；已修改 MoveIt 接口、Nav2 配置、关节话题、仿真时间、视觉输入与启动检查。证据：`reports/host-jazzy-build.json`、`reports/host-jazzy-build.log`。容器内编译仍待完成。
- [x] **08. 编写容器、桌面挂载及统一启动配置。** `Dockerfile`、`compose.yaml`、`tools/course.sh` 已生成。SDK 挂载到 `/isaac-sim`；桌面只读挂载到 `/workspace/Desktop`；项目读写挂载到 `/workspace/course`。实际挂载和 GUI 可用性待第 10 项验证。

## 进行中与后续顺序

- [ ] **09. 完成课程镜像构建。〔进行中〕** 正在下载、安装 ROS 2 Jazzy、Nav2、MoveIt2 等依赖；随后安装 ARM64 CUDA PyTorch、torchvision、YOLOE，并在镜像内编译全部课程包。日志：`reports/container-build.log`。当前尚无完成的课程镜像。
- [ ] **10. 启动新课程容器并验证环境。〔下一步〕** 验证 GPU、SDK/桌面/项目挂载、Isaac GUI、RViz、容器内课程包、PyTorch GPU 运算、torchvision 与 YOLOE 推理；验证后导出依赖锁定记录。
- [ ] **11. 在真实 6.0 运行时完成 Action Graph 迁移。** 连接 Compute Transform Tree 和 Read Joint State，统一机械臂话题及相机话题，处理现有 PhysX 雷达兼容性、控制图冲突和传感器频率；检查远程依赖；保存正式新版场景并重新打开验证。当前只有草稿，尚无通过运行验收的最终场景。
- [ ] **12. 验证仿真通信与三类控制。** 验证播放、暂停、重置，及 `/clock`、TF、里程计、雷达、RGB、深度、CameraInfo、关节状态；实际执行底盘、机械臂、夹爪命令。确认统一 `use_sim_time=true`、Cyclone DDS、`ROS_DOMAIN_ID=44` 与 QoS。
- [ ] **13. 跑通一次完整课程。** 完成导航到桌边、视觉定位、机械臂接近、抓取和提升；以物体实际随夹爪离开桌面作为成功证据，记录日志和结果。
- [ ] **14. 完成重复性与故障诊断验收。** 固定初始条件，重置后连续成功 3 次；重启容器后再次成功；验证资产缺失、话题无数据、TF 缺失的明确诊断。无数据负向诊断已预检通过，但本项整体尚未完成。
- [ ] **15. 完成交付及教程。** 提供最终场景、可复现构建配置、经过验证的版本锁定、统一启动命令、更新教程和完整验收记录。

## 当前结论

已完成环境检查、资源准备、配置编写和代码预检；课程镜像仍在构建。尚未验证 Isaac GUI、最终场景图执行、容器内视觉推理或完整课程闭环。

## 验证范围与待解决问题

- **编译和启动预检范围：** 宿主机构建结果为 `9 packages finished [16.2s]`。`ros2 launch course_bringup course.launch.py --show-args` 返回 0，只验证参数解析，没有启动课程节点。容器内构建、节点启动和功能验收分别在步骤 09–13 记录。
- **启动诊断预检：** 无仿真数据时的 readiness 负向测试按预期退出 1，并列出缺失话题、时钟和 TF，记录见 `reports/readiness-negative.json`。该报告的 `failed` 是预期诊断结果，不表示已经验证真实仿真就绪。实际故障注入验收属于步骤 14。
- **场景资产解析：** `OmniPBR.mdl` 依赖 Kit 运行时搜索路径，远程默认环境尚未验证可用性；步骤 11 必须确认实际解析。虚线贴图的纯色替代也需检查显示效果。早期截断 ZIP 的部分恢复报告仅作历史记录，当前资产来源以 `reports/assets-source.json` 为准。
- **图和控制兼容：** 课程使用 PhysX 雷达，需要验证 6.0 下的相关扩展与图节点；同时处理可能与 ROS 轮速控制冲突的键盘控制图。TF 和关节发布节点必须在真实运行时完成迁移，静态引用检查不能证明其能执行。
- **视觉版本验证：** 当前声明 PyTorch 2.9.0、torchvision 0.24.0、CUDA 13.0 wheel 及 YOLOE 依赖；这是候选组合。步骤 10 的 GPU 运算、NMS、cv_bridge 与 YOLOE 推理通过后才作为已验证版本锁定。
- **桌面与显示验证：** Compose 解析已通过，但 SDK、桌面和项目的实际挂载权限、X11 显示、Isaac GUI 与 RViz 仍待步骤 10 验证。GPU 兼容检查是无窗口检查，不能代替 GUI 验收。

旧进度报告仅作为历史快照归档于 [reports/archive/PROJECT_PROGRESS_2026-10-14_160655.md](reports/archive/PROJECT_PROGRESS_2026-10-14_160655.md)，不再更新；原始日志和 JSON 验证报告继续保存在 `reports/`。

## GitHub 仓库备份

- [x] 初始化本地 Git 仓库与 `main` 分支，整理源码、容器配置、迁移草稿和精选审计记录；原始首个提交为 `3d6c313`。
- [x] 添加根目录 README、本地资产恢复说明和 578 个本地文件的 SHA-256 清单；校验全部通过。排除 SDK、资产、模型、缓存、凭据和原始运行日志，保留在本机。
- [x] 暂存文件检查通过：初始 115 个文件约 3.28 MiB，Python AST、JSON、Shell 语法及 Compose 解析通过，未发现匹配的凭据模式。
- [ ] 在 `freeskyyyyy` 下新建公开仓库 `Isaac4.5-6.0_ProjectAdaptation`，推送并核对远端提交。用户已明确选择公开；GitHub CLI 设备登录成功，账号已核对为 `freeskyyyyy`，正在创建和上传。

## 更新记录

- 2026-10-14 16:05：根据当前报告、构建日志和 Docker 状态建立清单。步骤 01–08 完成；步骤 09 正在进行；步骤 10–15 待完成。
- 2026-10-14 16:10：按用户提出的文档重合问题，将独有验证范围和待解决问题并入本文件，旧进度报告归档；今后只维护本文件中的进度。镜像仍在 ROS 等依赖下载阶段，完成项未新增。
- 2026-10-14 16:18：完成 GitHub 上传的本地整理、首个提交和资源清单校验；远端创建与推送等待设备登录。课程镜像依赖下载继续进行。
- 2026-10-14 16:28：GitHub CLI 登录成功，用户要求将新仓库设为公开，开始创建与推送。
