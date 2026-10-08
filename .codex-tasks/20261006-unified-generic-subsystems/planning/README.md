# 计划使用说明

本目录只保存本 Epic 的规划和审查记录，不代表源码已经实施。

执行顺序：先完成 `planning/review.md` 的独立审查并修订阻断项，再按 `SUBTASKS.csv` 的依赖顺序逐个执行子任务。每个子任务使用自己的 `SPEC.md`、`TODO.csv` 和 `PROGRESS.md`，只有验证命令通过后才能标记 DONE。

本轮请求仅落盘方案；审核通过也不自动开始源码实施。最新方案为 `packages/suspension_multibody/docs/unified_generic_multibody_plan.md`。读取新 Epic 和该方案；上轮 `generic_multibody_evolution_plan.md` 仅作已实施历史证据。

计划自检：`uv run --no-sync python .codex-tasks/20261006-unified-generic-subsystems/planning/validate_plan.py`。`checks.py --dry-run` 只核对未来命令；本次不运行源码回归。

实施时运行叶验证并保存证据：`uv run --no-sync python .codex-tasks/20261006-unified-generic-subsystems/planning/checks.py step <child_id> <step_id>`。实际执行的是 TODO 行的 validation_command；成功日志、退出码、命令和日志哈希写入 raw/validation，再更新 CSV 的 completed_at/notes/status。validator 拒绝无证据的 DONE、依赖未完成就开始子任务或前步骤未完成就启动删除。

artifacts.csv 校验的是声明交付责任及命令覆盖，不追踪文件编辑者。任务协议约束执行顺序，不阻止会话外的任意文件写入；仍须遵守库存先于产品修改的协作约束。
