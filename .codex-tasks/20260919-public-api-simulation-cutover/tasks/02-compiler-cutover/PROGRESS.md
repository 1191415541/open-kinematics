# PROGRESS

- 任务：完成 compiler 与统一 runner 全面切换
- 形态：single-full（Epic 子任务）
 - 进度：5/5 步骤完成
 - 当前：任务 02 已完成；七类 compiler、统一 runner、兼容 wrapper 和 partial/failed 传播均已验证
 - 文件：`.codex-tasks/20260919-public-api-simulation-cutover/tasks/02-compiler-cutover/`
 - 下一步：进入任务 03，核对 RawContractResult、results.decoder 与 TimeSeriesResult 的生产接线


## 恢复信息

先按七类工况核对 request kind、compiler、assembly 和 payload，再收敛 `run_request()` / `run_compiled()`；每次迁移后运行 architecture audit，减少对应 allowlist 条目。

## 验证

`uv run --all-packages pytest packages/suspension_multibody/tests/simulation packages/suspension_multibody/tests/cases -q && uv run --all-packages pytest packages/suspension_multibody/tests/architecture -q`
