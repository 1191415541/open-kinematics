- 任务：完成整车 service 入口迁移
- 形态：single-full（Epic 子任务）
- 进度：5/5 步骤完成
- 当前：任务 05 已完成；任务 06 为下一个可执行依赖
- 文件：`.codex-tasks/20260919-public-api-simulation-cutover/tasks/05-vehicle-cutover/`
- 下一步：进入统一 artifact owner 与 Public API / CLI / Adams / IO 切换

## 恢复信息

`run_vehicle_dynamics()` 已切换为 `SimulationRequest → simulation.run_request() → results.decoder → VehicleDynamicsResult/VehicleResult`，并接入 vehicle metrics；保留显式多体整车模型、`axle`、`steering_output`、属性转发、partial/failed evidence、diagnostics 和 performance。公共 API 与 artifact 写出在任务 06 切换。

## 验证

`uv run --all-packages pytest packages/suspension_multibody/tests/vehicle packages/suspension_multibody/tests/cases/test_vehicle_dynamic_contract.py packages/suspension_multibody/tests/cases/test_vehicle_kc.py packages/suspension_multibody/tests/analysis/test_vehicle_dynamic.py packages/suspension_multibody/tests/adams/test_full_vehicle_model.py -q && uv run --all-packages pytest packages/suspension_multibody/tests/architecture -q`

- vehicle/cases/analysis/Adams：99 passed, 1 skipped, 1 xfailed
- architecture：41 passed
- results/metrics/IO regression：21 passed
- compileall：通过
- git diff --check：通过（仅保留 CRLF normalization warnings）
