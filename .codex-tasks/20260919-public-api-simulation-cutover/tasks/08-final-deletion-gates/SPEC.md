# 子任务 08：兼容代码删除与最终门禁

## 目标
在所有调用方完成迁移、结果/指标/artifact 已进入生产链路且 strict 架构门禁通过后，删除旧入口、兼容 facade、重复解码和重复 writer，完成 Epic 收口。

## 范围
- `axle_dynamics.native`
- 旧 `run_*_contract()` 入口和直接 native 调用
- `schema.DynamicResultBundle` 新生产创建路径及历史适配边界
- `io/results.py`、`axle_dynamics/io.py`、`vehicle_dynamics.py` 的重复 writer
- 内联结果解码和重复指标计算
- `tests/architecture/test_unified_simulation_boundaries.py` 中对兼容 facade 存在性的旧断言
- 顶层导出、CLI、Adams、脚本、测试和文档残留
- Epic / SUBTASKS / PROGRESS 真源状态

## 必须完成
1. 全仓扫描确认 `kernel.run_contract()` 的生产唯一调用点是 `NativeContractBackend`。
2. 全仓扫描确认生产代码不再直接调用 `cases.*.run_*_contract()`、`results.raw.decode_contract_run()` 或 `axle_dynamics.native`。
3. 删除已无调用方的兼容 facade、旧入口、重复 decoder、重复 writer 和失效导入。
4. 重写或删除仍断言 `axle_dynamics.native` 兼容导出存在的架构测试；最终 strict gate 不得把兼容 facade 误判为必需。
5. 对仍需保留的历史 artifact 读取兼容、公开异常和公开属性转发，写明保留边界和删除条件。
6. 更新顶层导出、测试、脚本、文档命令和架构门禁，清理失效引用。
7. 完成成功、partial、failed、diagnostics、performance、failure evidence、结果快照、TimeSeriesResult 和 artifact 重读回归。
8. 更新 8 个子任务状态、父级 `SUBTASKS.csv`、`PROGRESS.md` 和 `EPIC.md`；仅在全部通过后将 Epic 标记为 `DONE`。

## 删除前硬门槛

- `results.decoder`、`results.axle`、`results.vehicle`、`results.timeseries` 和四类 metrics 均有生产调用证据；
- CLI、Adams、probe、性能门、contract parity、动态/时域/整车回归通过；
- strict architecture gate、`ruff`、`py_compile`、`ty check`、`git diff --check` 通过；
- 全仓无旧入口、死 facade、重复 writer、失效文档命令；
- 对外 API 返回类型变化、DynamicResultBundle 历史读取边界和兼容异常变更已记录；
- 不修改 C++ native、ABI、contract version、物理方程、通道顺序和 manifest hash。

## 验收
- 旧实现已删除或明确列为历史读取兼容，不再产生新格式。
- 全局验收命令和专项门禁全部通过。
- Epic 真源与实际代码状态一致，可从冷启动恢复。

## 验证
`uv run --all-packages pytest && uv run --all-packages ruff check packages/suspension_multibody/src packages/suspension_multibody/tests && uv run --all-packages ty check . && git diff --check`
