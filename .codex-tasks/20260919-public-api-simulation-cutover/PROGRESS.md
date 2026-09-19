## 当前状态

- 任务：将 Public API / CLI / Adams / IO 全面切换到统一 simulation/results/metrics/artifact 架构
- 形态：epic
- 进度：7/8 子任务完成
- **当前**：任务 01–07 已完成，任务 08 为下一个可执行依赖
- **文件**：`.codex-tasks/20260919-public-api-simulation-cutover/tasks/08-final-deletion-gates/`
- **下一步**：执行兼容代码删除与最终 strict 门禁

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

## 未完成

- [x] 任务 1：边界盘点、基线 allowlist 与审计式架构门禁
- [x] 任务 2：compiler 与统一 runner 全面切换
- [x] 任务 3：RawContractResult、results.decoder 与 TimeSeriesResult 接线
- [x] 任务 4：整轴 service 入口迁移
- [x] 任务 5：整车 service 入口迁移
- [x] 任务 6：统一 artifact owner 与 Public API / CLI / Adams / IO 全面切换
- [x] 任务 7：metrics 与统一 artifact IO 收口
- [ ] 任务 8：兼容代码删除与最终门禁

## 恢复协议

1. 先读取 `EPIC.md`、`SUBTASKS.csv` 和本文件。
2. 找到第一个依赖已满足且状态不是 `DONE` 的子任务。
3. 再读取对应子任务目录中的 `SPEC.md`、`TODO.csv`、`PROGRESS.md`。
4. 每完成一个子任务，先执行其验证命令，再更新子任务和本文件状态。
5. 任务 2–7 每次减少 `LEGACY_ALLOWLIST.toml` 条目；任务 8 才能切换 strict mode。
6. 只有所有子任务为 `DONE` 且最终门禁通过，才允许将 Epic 标记为 `DONE`。

## 全局验证基线

- `uv run --all-packages pytest`
- `uv run --all-packages ruff check packages/suspension_multibody/src packages/suspension_multibody/tests`
- `uv run --all-packages ty check .`
- `uv run --all-packages python -m compileall -q packages/suspension_multibody/src packages/suspension_multibody/tests scripts`
- `git diff --check`
- contract parity、K/C probe、动态/时域/整车回归、Adams 门、性能门
- 全仓搜索旧入口、旧 facade、重复解码、重复 writer 和失效文档命令
