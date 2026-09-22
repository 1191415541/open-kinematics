- 任务：迁移剩余调用方并删除无生产调用的旧模块、同步文档与打包（A1 修订：不设必须删完）
- 形态：single-full（Epic 子任务）
- 进度：7/7 步骤 DONE
- 当前：子任务完成。已删无生产调用者的旧模块；A1/A2 保留项逐项登记；文档同步；删除后残留全部为保留项。
- 文件：`.codex-tasks/20260921-architecture-deviation-closure/tasks/20260921-08-delete/`
- 验证：gates 145 passed、kernel 15 passed、全量 737 passed / 47 skipped / 1 xfailed（新增失败 0）；构建 0；动态哈希 26/26 逐位一致（组合哈希 `e7407656…8d48e` 未变）；`--strict` 分层门 0；8 family 0；`legacy_surface --check` 0；两包 `uv build` 0 且清单不含已删模块；ruff 0；ty 0。证据在 `raw/step7_*.log`。

## 恢复信息

前置：01 的 `SYMBOL_MATRIX.csv` 与 `VALIDATION.md` 已冻结；02 的删除门禁与扫描器模式就绪；03–07 已完成全部迁移且旧调用方清零。旧调用方未清零时不得开工本任务。

本任务删除 `core`/`elements`/`model`/`analysis`/`metrics` 旧目录与旧顶层 `pac2002_scope.py`，改造旧物理测试为 native 契约测试，同步当前文档与打包，并在本任务目录交付扫描器脚本 `legacy_reference_scan.py`。

下一步：无（7/7 DONE）。09 可启动；其终局验收须按 EPIC Done-When 逐条核对，并注意两条已登记口径：`legacy_surface --check --final` 在 A1 保留旧包时**必然退 1**（门禁自测断言该行为），`--check` 退 0 才是判据。

删除清单：`core/rank.py`、`core/reactions.py`、顶层 `pac2002_scope.py`、`model/` 整包、`metrics/` 整包、`analysis/{_geometry,benchmarks,compliance,metrics,time_domain_physics,time_signals,vehicle_kc_time_domain}.py`、`tests/core/test_reactions.py`，以及 `tests/model/test_dynamic_mass.py` 的 2 条质量矩阵断言。

保留清单（file:line + 阻断原因 + 解除条件见 `raw/step3_deletion_record.md`）：`elements/` 整包（A1，native 通道对固定体早退）、`core/{constraints,rigid_body,spatial}`（A1 连带：`elements` 与 `preparation/assembly/types` 的依赖；`residual`/`jacobian` 的消费者是 `ConstraintSystem`，只删会让其立即 `NameError`）、`analysis/vehicle_physics.py`（A2 静轮荷 + `compute_vehicle_roll_centers` 的反向边登记）。

扫描 before/after：110（08 开工基线）→ 89（07 切换调用方后的 HEAD）→ 30（删除后），残留 30 条**全部**为 A1/A2 保留项或 README 的迁移说明；指向已删模块（`model`、`metrics`、顶层 `pac2002_scope`、`analysis` 报告模块、`core.rank`、`core.reactions`）的引用为**零**。

三处上报冲突的裁决记录在 `raw/step6_scan_after.md` §7：`--final` 退 1（A1 下正确，不放宽门禁）；`residual`/`jacobian` 保留（否则半删）；`ty check .` 退 1 由主代理在 `pyproject.toml` 的 `[tool.ty.src] exclude` 增加 `.codex-tasks/` 修复（归档证据脚本按设计引用已退役模块，非受检面）。
