- 任务：完成整轴 service 入口迁移
- 形态：single-full（Epic 子任务）
- 进度：5/5 步骤完成
- 当前：任务 04 已完成；任务 05 同步完成，任务 06 为下一个可执行依赖
- 文件：`.codex-tasks/20260919-public-api-simulation-cutover/tasks/04-axle-cutover/`
- 下一步：进入统一 artifact owner 与 Public API / CLI / Adams / IO 切换

## 恢复信息

`run_axle_dynamics()` 已切换为 `SimulationRequest → simulation.run_request() → results.decoder → AxleDynamicsResult`，并接入 axle metrics；保留 `AxleResult` 兼容别名、异常语义、partial/failed evidence、diagnostics 和 performance。任务 04 不切换 api.py、CLI、Adams 外部调用方，也不产生新 artifact。

## 验证

`uv run --all-packages pytest packages/suspension_multibody/tests/axle_dynamics packages/suspension_multibody/tests/adams/test_time_domain_axle.py packages/suspension_multibody/tests/adams/test_axle_dynamics_equivalence.py -q && uv run --all-packages pytest packages/suspension_multibody/tests/architecture -q`

- axle/Adams：163 passed
- architecture：41 passed
- compileall：通过
- git diff --check：通过（仅保留 CRLF normalization warnings）
