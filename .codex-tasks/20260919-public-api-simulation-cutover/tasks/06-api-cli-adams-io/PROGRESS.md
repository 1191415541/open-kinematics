# PROGRESS

- 任务：完成统一 artifact owner 与 Public API / CLI / Adams / IO 全面切换
- 形态：single-full（Epic 子任务）
- 进度：6/6 步骤完成
- 当前：统一 artifact owner、Public API、CLI、Adams、脚本和 IO 调用方已完成迁移
- 文件：`.codex-tasks/20260919-public-api-simulation-cutover/tasks/06-api-cli-adams-io/`
- 下一步：进入任务 07，收口 metrics 注册、统一 artifact IO 和旧 writer 停产

## 完成记录

- `io.artifacts.write_artifact/read_artifact` 已成为唯一新 artifact 写出/读取 owner，覆盖 native、time-series、success、partial、failed 和空嵌套表字段。
- `api.py`、`cli.py`、顶层导出、Adams K/C/时域入口、`axle_equivalence.py`、parity/probe/performance 脚本均已切换到统一 runner 或统一 artifact IO。
- `run_dynamic_case()` 保持任务03固定的 `TimeSeriesResult` 契约；`DynamicResultBundle` 仅保留历史读取兼容。
- 架构 allowlist 已从历史基线精确收缩到当前15条兼容性 finding；未引入新的 native 旁路。

## 验证

- `uv run --all-packages pytest packages/suspension_multibody/tests/api packages/suspension_multibody/tests/cli packages/suspension_multibody/tests/analysis packages/suspension_multibody/tests/contract -q`：20 passed。
- `uv run --all-packages pytest packages/suspension_multibody/tests/results packages/suspension_multibody/tests/metrics packages/suspension_multibody/tests/io -q`：25 passed。
- `uv run --all-packages pytest packages/suspension_multibody/tests/adams -q`：206 passed, 1 skipped。
- `uv run --all-packages pytest packages/suspension_multibody/tests/architecture/test_public_api_boundary_gate.py -q`：4 passed。
- `uv run --all-packages python -m compileall -q packages/suspension_multibody/src packages/suspension_multibody/scripts`：通过。
- `git diff --check`：通过，仅保留 CRLF normalization warnings。
