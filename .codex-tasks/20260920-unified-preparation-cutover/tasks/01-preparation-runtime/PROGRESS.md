- 任务：建立统一 preparation 协议与 runner 生命周期
- 形态：single-full（Epic 子任务）
- 进度：4/4 步骤完成，DONE
- 当前：中央运行时和协议测试已完成独立审查、验证及记录审计
- 验证：simulation 53 passed；architecture 44 passed；组合门禁97 passed；scoped ruff、全仓 ty 和 git diff --check 通过
- 文件：`.codex-tasks/20260920-unified-preparation-cutover/tasks/01-preparation-runtime/`

## 恢复信息

父级 PROGRESS 已逐条记录全部验收命令、实际退出码和 scratch 日志路径。独立审查 b608f382-fd4a-474c-981a-aa48f43aa65c 无运行时功能缺陷；指出的复用描述冲突已按 EPIC 上下文合并原则统一，七个 family 的 prepare_request 导出契约及任务03 runner新键接线均已补齐。

下一步执行任务02六个真实 family；任务01的替身测试不代表真实family集成完成。整车 legacy prepared 的真实适配与 runner decoder 上下文迁移由任务03收口。尚未删除旧模块。
