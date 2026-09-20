## 当前状态

- 任务：将 Public API / CLI / Adams / IO 全面切换到统一 simulation/results/metrics/artifact 架构
- 形态：epic
- 进度：8/8 子任务完成
- **当前**：任务 01–08 全部完成，Epic 已收口
- **文件**：`.codex-tasks/20260919-public-api-simulation-cutover/`
- **下一步**：无；最终 strict 门禁与专项回归已完成

## 已完成

- [x] 固化唯一生产链路：`Public API / CLI → SimulationRequest → cases/<family> compiler → simulation.run_request() → NativeContractBackend → ContractRun → RawContractResult → results.decoder → AxleResult / VehicleResult → metrics → 统一 artifact IO`
- [x] 明确时域/replay 聚合使用 `TimeSeriesResult`，旧 `DynamicResultBundle` 只保留历史读取兼容
- [x] 固化 `io.artifacts` 作为唯一新 artifact writer/reader owner
- [x] 固化 compiler、simulation、results、metrics、artifact IO 的职责边界和禁止依赖
- [x] 注册 8 个有依赖关系的子任务，并完成独立方案审查与阻断项修订
- [x] 完成任务 01：边界盘点、基线 allowlist 与审计式架构门禁
- [x] 完成任务 02：七类 compiler、统一 runner、backend 唯一 native 提交点和 partial/failed 传播
- [x] 完成任务 03：RawContractResult、assembly decoder、TimeSeriesResult、统一结果 IO 回归和历史 bundle 读取边界
- [x] 完成任务 04：整轴 service 入口迁移、AxleResult/metrics 接线和失败证据回归
- [x] 完成任务 05：整车 service 入口迁移、VehicleResult/metrics 接线和显式多体兼容回归
- [x] 完成任务 06：统一 artifact owner 与 Public API / CLI / Adams / IO 全面切换
- [x] 完成任务 07：metrics 注册、K&C/vehicle physics 派生指标迁移、统一 artifact IO 收口和重读回归
- [x] 完成任务 08：兼容 facade、旧 decoder 暴露、测试/脚本旁路与 `_CaseSequence.__call__` 清理；strict 门禁和最终回归通过

## 最终收口证据

- 专项回归：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests -q` → `566 passed, 47 skipped, 1 xfailed`
- architecture/CLI：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests/architecture packages/suspension_multibody/tests/cli -q` → `49 passed`
- strict `LEGACY_ALLOWLIST.toml` → `0 entries / 0 findings`
- `uv run --all-packages ruff check packages/suspension_multibody/src packages/suspension_multibody/tests` → passed
- `uv run --all-packages python -m compileall -q packages/suspension_multibody/src packages/suspension_multibody/tests packages/suspension_multibody/scripts` → passed
- `uv run --all-packages ty check .` → passed
- `git diff --check` → passed

## 保留边界

`schema.DynamicResultBundle`、`load_dynamic_result()`、`history_from_dynamic_bundle()` 仍仅用于历史 artifact 读取；新生产路径不创建旧 bundle。`results.decoder.decode_result()` 是公开结果解码入口，raw contract 适配器仅保留为 results 内部私有实现。

## 恢复协议

1. 先读取 `EPIC.md`、`SUBTASKS.csv` 和本文件。
2. 找到第一个依赖已满足且状态不是 `DONE` 的子任务。
3. 再读取对应子任务目录中的 `SPEC.md`、`TODO.csv`、`PROGRESS.md`。
4. 每完成一个子任务，先执行其验证命令，再更新子任务和本文件状态。
5. 任务 2–7 每次减少 `LEGACY_ALLOWLIST.toml` 条目；任务 8 切换 strict mode。
6. 所有子任务已为 `DONE`，Epic 已完成。
