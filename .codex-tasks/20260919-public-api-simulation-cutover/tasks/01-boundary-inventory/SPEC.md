# 子任务 01：边界盘点与架构门禁设计

## 目标
建立完整的生产调用图、入口矩阵、符号清单、删除候选清单和已知违规基线，并把统一链路约束固化为可执行的审计式架构门禁。

## 范围
- `packages/suspension_multibody/src/suspension_multibody`
- 顶层 CLI、Adams、probe、性能门、脚本和相关测试
- `.codex-tasks/20260919-public-api-simulation-cutover/` 内迁移真源

## 必须产出
1. `Public API / CLI / Adams / script → SimulationRequest → compiler → run_request → backend → ContractRun → RawContractResult → decoder → result → metrics → artifact` 调用图。
2. 正式入口、兼容 facade、重复解码、重复 writer、直接 native 调用的符号矩阵。
3. 每个工况族对应 compiler、request payload、result assembly、时域聚合和 case-specific metrics 的清单。
4. 旧入口删除候选及保留理由；历史 artifact 读取兼容与新写出路径的区分。
5. 基线 allowlist：当前已知违规文件/符号/原因/计划删除任务/删除条件，必须逐条列出，不得使用目录级或通配符豁免。
6. 审计式架构门禁：任务 1 阶段允许 allowlist 中的既有违规，但必须失败于新增旁路；任务 2–7 每次减少 allowlist；任务 8 切换 strict mode。
7. 与现有测试目录匹配的门禁测试或检查脚本，并明确重写 `test_unified_simulation_boundaries.py` 中仍断言兼容 facade 存在的测试。
8. 产物文件：`BOUNDARY.md`、`SYMBOL_MATRIX.csv`、`LEGACY_ALLOWLIST.toml`。

## 约束
- 只盘点和定义边界，不提前删除生产兼容代码。
- 不修改 C++、ABI、contract version、通道名、顺序、单位和 hash。
- 不把不同工况族合并成一个 compiler。
- 不把当前旧路径全部静默标记为允许；每个 allowlist 条目都必须绑定后续子任务和删除条件。

## 验收
- 调用图和符号矩阵可由冷启动恢复。
- 所有后续子任务的目标文件、依赖、owner 和删除条件明确。
- 门禁能区分生产代码、测试、历史读取兼容和允许的 backend 唯一调用点。
- 门禁在当前基线通过，并能用临时 fixture 证明新增旁路会失败。

## 验证
`uv run --all-packages pytest packages/suspension_multibody/tests/architecture -q && git diff --check`
