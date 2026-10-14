# Isaac Sim 4.5 → 6.0：DGX Spark 课程迁移

将现有 Bobac 移动机械臂课程迁移到 DGX Spark 的独立 Docker 容器，使用 Isaac Sim 6.0、ROS 2 Jazzy、Nav2、MoveIt2 和 YOLOE，目标为导航、视觉定位、抓取和提升的完整流程。

**当前仍在迁移中，尚未通过完整课程运行验收。** 官方 GPU 兼容检查、课程资产静态审计及宿主 Jazzy 的 9 个 ROS 包编译预检已通过；目标课程镜像构建、GUI、图节点、视觉 GPU 推理和物理闭环仍需验证。最新进度只维护在 [MIGRATION_TODO.md](MIGRATION_TODO.md)。

## 仓库内容

| 路径 | 内容 |
| --- | --- |
| `Dockerfile`、`compose.yaml` | 固定官方镜像摘要的课程环境与挂载配置 |
| `bobac_robotac_src/src/` | ROS 课程源码、机器人描述、导航地图和统一启动包 |
| `test_scene_bobac.usd` | 保留的原始课程入口场景 |
| `scenes/` | 独立迁移草稿；正式迁移场景需运行迁移工具生成 |
| `tools/`、`docker/` | 环境、审计、迁移、启动和验证工具 |
| `docs/` | 本地资产恢复说明与 SHA-256 清单 |
| `reports/` | 已提交的审计摘要及历史快照；完整运行日志留在本机 |
| [DGX_SPARK_MIGRATION_PLAN.md](DGX_SPARK_MIGRATION_PLAN.md) | 迁移依据与验收标准 |

旧课程子目录中的说明保留作参考；迁移后的状态与入口以根目录文档为准。

## 本地文件与容器路径

默认将仓库放在桌面。克隆仓库不包含完整 Isaac SDK、约 1.8 GB 的课程资产和模型权重，需按 [本地资产说明](docs/LOCAL_ASSETS.md) 恢复。恢复后再构建，避免镜像内缺少视觉模型。

| 宿主路径 | 容器路径 | 权限 |
| --- | --- | --- |
| `../IsaacSim-6.0.0` | `/isaac-sim` | 读写 |
| `..`（默认桌面） | `/workspace/Desktop` | 只读 |
| 当前仓库 | `/workspace/course` | 读写 |
| `.runtime/arm64/` | 编译目录、安装目录及各类缓存 | 读写 |

可使用 `ISAAC_SIM_HOST_DIR`、`COURSE_DESKTOP_DIR` 覆盖默认目录。SDK 是从官方镜像导出的程序目录，运行环境由容器提供；在桌面保存 SDK 不等于在宿主机完成独立安装。

## 运行入口（仍需完成运行验收）

宿主机需要配置好 NVIDIA 驱动、Docker、NVIDIA Container Toolkit 和 Docker Compose，并可访问本机桌面显示。当前部署脚本仅面向 DGX Spark ARM64。

```bash
# 如尚无桌面 SDK，从 Dockerfile 固定的官方镜像导出。
bash tools/setup_desktop_sdk.sh

# 先恢复 docs/LOCAL_ASSETS.md 中的课程资产和权重，再构建、启动。
bash tools/course.sh build
bash tools/course.sh up
bash tools/course.sh check
bash tools/course.sh compile
bash tools/course.sh vision-check

# 静态审计与真实运行时图迁移；失败时先查看 reports/ 中的诊断。
bash tools/course.sh audit
bash tools/course.sh migrate

# 终端 1：打开场景，在 GUI 中按 Play。
bash tools/course.sh sim

# 终端 2：数据与 TF 就绪后启动课程 ROS 节点。
bash tools/course.sh ros

# 终端 3：检查数据；导航和规划验证完成后再显式触发抓取。
bash tools/course.sh ready
bash tools/course.sh grasp
```

这些命令是已实现的入口，不代表上述所有阶段已经验证成功。完成状态和每项验收证据见 TODO；抓取成功需要物体实际离开桌面的证据。

默认 `ROS_DOMAIN_ID=44`、Cyclone DDS、`use_sim_time=true`。机械臂原始状态为 `/arm/joint_states`，六轴过滤状态为 `/arm/joint_state_deal`，保留整机 `/joint_states` 和 `/hand_command` 接口。

## 验收与版本

官方基础镜像固定为 `nvcr.io/nvidia/isaac-sim:6.0.0@sha256:68735a60b6c15c85e0dd0098570c6d2cc79e928f2d068ce2790aa43284ac165d`。该镜像内 SDK 的 `VERSION` 实际为 `6.0.0-rc.59+release.41464.5f2772bc.gl`，保留此实际版本信息。

候选视觉依赖见 `docker/vision-requirements.txt`；通过真实 GPU 验证后导出解析版本。最终验收包含完整课程连续成功三次及容器重启后复现，当前尚未完成。

课程原有源码的许可证声明保留在各包中；未给整个课程或第三方资产额外授予许可证。NVIDIA SDK、机器人资产和模型遵循各自的使用条款。
