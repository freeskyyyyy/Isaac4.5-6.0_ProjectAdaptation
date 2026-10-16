# GitHub 上传检查点：2026-10-16

仓库：https://github.com/freeskyyyyy/Isaac4.5-6.0_ProjectAdaptation

这是迁移开发检查点，不是完整课程验收。用户最新要求“先能跑通，再处理精度”；后续主线为固定底盘的真实视觉→MoveIt→夹爪→物体离桌，再整合导航。详细进度以 `MIGRATION_TODO.md` 为准。

## 本次内容

- 导航：ConnectedSmac起终点连接、独立RPP配置、仿真时间闭环速度平滑器、反馈诊断和对应回归；保留默认配置与历史失败。
- 公共门禁：非有限关节值拒绝、同一已验路径的跟踪观测、仿真/墙钟双重反馈与TF等待约束。
- 夹爪：原Action和 `/hand_command` 保留，新增实测关节反馈、超时、互斥、取消与仿真回退处理。位置到位及发送关闭命令都不代表物体已被抓住。
- 独立恢复、轮径候选及demo场景；保留原始USD。demo将笔放在桌内x=1.65，是实验布局，不是原布局验收。
- 全局失败分析、分层恢复计划、原课程范围审查、关键成功与失败记录及实验脚本。实验脚本含设置物体和运动操作，仅为复现实验准备，上传不会运行这些脚本。

## 已有真实结果及边界

1. `course-demo-navigate-retry.json`：近桌NavigateToPose成功，终点约37.6mm、yaw误差约0.0051rad。不是从reset起点完成整套路线；首轮无效四元数实验保持失败。
2. `course-demo-detect-lookat.json`：实际相机的DetectObject识别到笔，返回机械臂基座坐标；没有用场景真值替代检测输入。
3. `course-demo-grasp-execute.json`：接近、闭合指令及机械臂提升已执行，但物体未离桌，结果failed。不能用Action成功代替物理抓取证据。
4. `course-demo-gripper-open-check.json`：半开时左右主关节约0.3896/0.3897rad。完整抓取、三次成功及容器重启复现尚未完成，TODO13–15不勾选。

## 本次上传前重新执行的验证

| 检查 | 结果 | 范围 |
| --- | --- | --- |
| `tools/test_navigation_guard.py` | 14项通过 | 课程容器内软件回归，DDS域147 |
| `tools/test_readiness_values.py` | 5项通过 | 课程容器内非法关节数据检查，DDS域147 |
| `grasp_control/test/test_gripper_feedback.py` | 7项通过 | DDS域146独立Action及合成反馈；不操作共享仿真 |
| 导航构建目录CTest | 7/8组通过，退出8 | 场景来源SHA断言仍失败，日志 `github-upload-navigation-ctest.log` |
| 本次源文件及选入实验脚本Python语法 | 40个文件通过 | 静态检查，不替代目标运行验收 |

导航CTest的失败来自本地 `scenes/test_scene_bobac_6.0.course.usda` 修改，与地图记录来源SHA不同；未改断言、未宣称全绿。该文件来源尚未完成审查，本次不暂存其修改，保留在本机。独立候选另行提交。即使克隆后没有这份本地修改，也需要重新验证地图来源，不能把此检查点视为可一键复现的最终交付。

本次未重建最终课程镜像，已有overlay及真实运行证据不代表镜像内已包含全部修复。视觉B源码保持原状。原始大资产、模型权重、Docker镜像、缓存及原始录包不上传；资产恢复方式见 `docs/LOCAL_ASSETS.md`。选入证据哈希见 `github-upload-2026-10-16-manifest.json`，其余完整试验保留本机。

## 下一步

在已取得真实检测的条件下尝试更直接的垂直抓取，核对夹爪接触高度，取得物体实际离桌和随动证据；再处理状态机真实完成判断、统一入口、全流程及重复性验证。不会为本次上传而继续导航精度调参。
