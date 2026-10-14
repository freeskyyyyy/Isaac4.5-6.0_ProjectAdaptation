# 验证记录

本目录提交精选的 JSON 审计摘要与历史文档，保留验证范围和迁移依据。完整构建、兼容性检查和运行日志留在原机器的同名目录，按 `.gitignore` 排除，不随克隆下载。

- `compatibility-check.json`：官方 6.0 ARM64 兼容性检查通过；不代表 GUI 验收。
- `host-jazzy-build.json`：宿主 Jazzy 的 9 个包编译预检通过；不代表容器内构建通过。
- `assets-source.json`：完整课程 ZIP 的 CRC 校验与本地来源记录。
- `scene-audit.json`：6.0 SDK 静态审计结果；场景图和物理运行仍待验证。
- `readiness-negative.json`：无仿真数据时的预期失败诊断，不能作为真实仿真就绪证据。
- `archive/`：已停止更新的进度报告历史快照。

这些文件中的绝对路径是产生记录时的本机路径。当前唯一持续进度入口为 [MIGRATION_TODO.md](../MIGRATION_TODO.md)。后续生成的运行记录应在核对内容和验证范围后选择提交。
