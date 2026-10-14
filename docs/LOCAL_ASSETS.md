# 本地资产和模型恢复

Git 仓库保存源码、构建配置、小型场景和迁移说明；以下大型文件只保留在本机，不上传到普通 Git 历史。

## Isaac Sim SDK

在项目根目录执行 `bash tools/setup_desktop_sdk.sh`，从固定摘要的官方镜像导出完整 SDK 到项目旁的 `IsaacSim-6.0.0/`。也可将已有的同版本完整 SDK 放到该目录，或通过 `ISAAC_SIM_HOST_DIR` 指定。

SDK 不在 `local-assets.sha256` 中，其来源由 `Dockerfile` 中的官方镜像摘要固定。运行需要 NVIDIA 容器环境。

## 课程资产

当前来源为用户提供、CRC 校验通过的 `isaacsimassets-dev.zip`（1409011447 字节、616 个 ZIP 条目）。将完整解压目录恢复到仓库根目录：

```text
isaacsimassets-dev/
  assets/
    robot/bobac.usd
    ...
  integrated_runtime/
    ...
```

原机器的解压目录位于 `/home/nvidia/桌面/isaacsimassets-dev`，项目内已有一份完整副本。新机器需从自己的合法课程副本恢复；本仓库没有托管下载链接。`test_scene_bobac.usd` 是入口文件，仍依赖外部机器人及纹理，单独克隆入口文件不能完整运行场景。

## 视觉模型

在构建镜像前将以下文件放入 `bobac_robotac_src/src/yoloe_grasp/weights/`：

- `yoloe-26l-seg.pt`
- `yoloe-26m-seg.pt`
- `yoloe-26s-seg.pt`
- `mobileclip2_b.ts`

原机器保留了这些权重。新机器可从原课程副本恢复；替换模型时需要重新验证 YOLOE 与 MobileCLIP 的组合。

## 校验

恢复完毕后，在仓库根目录执行：

```bash
sha256sum --check docs/local-assets.sha256
```

清单记录这次整理时课程资产和模型的实际内容。所有文件均应通过；缺少文件时先恢复资源，再运行场景审计、构建与视觉验证。缓存、SDK 和日志不在此清单中。
