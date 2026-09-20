# PROGRESS

- 任务：完成兼容代码删除与最终 strict 门禁
- 形态：single-full（Epic 子任务）
- 进度：6/6 步骤完成
- 当前：任务 8 已完成；Epic 已完成收口
- 文件：`.codex-tasks/20260919-public-api-simulation-cutover/tasks/08-final-deletion-gates/`

## 已完成

1. 完成全仓旧入口与旁路扫描；strict allowlist 为 0，生产 `kernel.run_contract()` 仅由 `NativeContractBackend` 调用。
2. 删除六个 cases 层 `run_*_contract` 兼容 facade、失效顶层导出和重复入口。
3. 将 cases、K&C 测试、结果测试、Adams、probe、parity 脚本统一迁移到 `SimulationRequest → run_request()`；`results.raw` 内部 decoder 私有化；`_CaseSequence.__call__` 兼容层删除。
4. 扩展 architecture gate，使测试源码也检查 direct kernel/case/raw decoder 旁路；strict gate 通过。
5. 修正一处与当前 `exact_part_mapping` 语义不一致的 Adams fixture 断言；未修改 Adams 生产模型。
6. 完成专项回归与静态门禁。

## 验证

- `uv run --package suspension-multibody pytest packages/suspension_multibody/tests -q` → `566 passed, 47 skipped, 1 xfailed`
- `uv run --package suspension-multibody pytest packages/suspension_multibody/tests/architecture packages/suspension_multibody/tests/cli -q` → `49 passed`
- `uv run --all-packages ruff check packages/suspension_multibody/src packages/suspension_multibody/tests` → passed
- `uv run --all-packages python -m compileall -q packages/suspension_multibody/src packages/suspension_multibody/tests packages/suspension_multibody/scripts` → passed
- `uv run --all-packages ty check .` → passed
- `git diff --check` → passed
- strict `LEGACY_ALLOWLIST.toml` → 0 entries / 0 findings

## 保留边界

`schema.DynamicResultBundle`、`load_dynamic_result()`、`history_from_dynamic_bundle()` 仍仅用于历史 artifact 读取；新生产路径不创建旧 bundle。`results.decoder.decode_result()` 是公开结果解码入口，raw contract 适配器仅保留为 results 内部私有实现。
