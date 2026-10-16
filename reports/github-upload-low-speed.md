# 2026-10-16 低速导航集成快照

本次归档视觉固定时间戳 TF 等待、机械臂指尖偏移和方向拒绝修复、world 地图与低速运输配置、公共净距与预测停车门禁，以及真实容器验收证据。

真实新路径中心净距下界 0.635688m，门禁退出0。低速目标在跟踪偏差0.019537m时预测取消，制动后最大记录偏差0.037636m、记录轨迹净距下界1.437889m。目标未到达，TODO13–15保持未完成。实际加减速及完整课程均未通过。速度/制动/重规划完整记录见 world-low-speed-motion.json，汇总见 world-low-speed-motion-review.json；运行限制和给C的具体要求见 world-navigation-integration-feedback.json。

上传前在目标课程容器复验：视觉52项、运输控制9项、路径净距6项、world地图8项、公共预测停车5项，共80项通过。它们不代替真实运动验收。日志：github-upload-validation.log。

明确排除SDK、课程资产和权重、持久缓存、实验drive场景，以及16MB的早期TF预检原始记录；该预检保留本机。公开仓库不包含完整可运行资产，恢复方式仍参照docs/LOCAL_ASSETS.md及marker-asset-source.json。
