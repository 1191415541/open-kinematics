- 任务：迁移 Python 作者层声明与作者层归位（第 6 项按 A1 处置为保留 + 待删除登记）
- 形态：single-full（Epic 子任务）
- 进度：8/8 步骤 DONE（步骤 6 按 A1 登记保留，未删除）
- 当前：子任务完成。第 2-5 项完成作者层归位；第 6 项因 05 实测阻断（native 通道无法承载固定体端反力）按 A1 保留 + 登记；第 7 项回归通过。
- 文件：`.codex-tasks/20260921-architecture-deviation-closure/tasks/20260921-06-authoring/`
- 验证：全量套件 722 passed / 47 skipped / 1 xfailed（基线 712，新增 10 为 05 与本任务新测试），新增失败 0；构建 0；动态哈希 26/26 逐位一致（组合哈希 `e7407656…8d48e` 未变）；`--strict` 分层门 0；8 family 0；legacy_surface `--check` 0；ruff 0；ty 0。证据在 `raw/step7_*.log`。

## 恢复信息

前置：01 的 `VALIDATION.md` 已冻结；02 职责边界门禁就绪；03/04 C++ 模块归属完成；05 的 native 通道证据与 results 解码边界完成。05 未完成时不得开工本任务。

本任务写 `preparation/**`、`kernel/capabilities.py`、`schema/**`、`adams/**`、`api.py`、`elements/{elastic,assembly}.py` 以及相关测试归属调整；不建 `report`，不删除任何目录或 `pac2002_scope.py` 文件本体。

下一步：无（8/8 DONE）。可写面已释放给 07（`report` 新建、replay、基准夹具）。本任务未建 `report`。

实施结果（2026-09-22，提交 `a734038`）：

- 新增 `preparation/assembly/{__init__,types,front_axle,vehicle}.py`：`model/front_axle`、`model/vehicle` 逐字搬运（含 `_condense_welded_bodies` 焊接体凝聚，A1 明确保留在 Python 作者层，未迁 native）；`types.py` 只含数据对象，实测不含 `residual`/`jacobian`/`evaluate` 方法（`[]`）。
- 新增 `preparation/geometry.py`（现役输入侧坐标/四元数/力旋量转换）、`preparation/signals.py`（`time_signals` 5 个符号逐字搬迁）、`results/geometry.py`（结果侧边界占位，不复制实现、不做 re-export，避免 `results`→`preparation` 反向边）。
- 新增 `kernel/capabilities.py`（`_kernel_scope` lru_cache + 惰性常量 + `__getattr__`，实测保持惰性）、`schema/pac2002_scope.py`（校验辅助）、`adams/pac2002_evidence.py`（Adams 证据常量）。
- `core/constraints.py`：11 个类的 `residual`/`jacobian` 方法改为模块级函数 `residual(constraint, state)` / `jacobian(constraint, state)`（分派逻辑逐字保留），数据对象改为从 `preparation.assembly.types` 引入并 re-export；消费者仅 `core/reactions.py`（该文件无生产调用者）。
- `core/{spatial,rigid_body}.py` 精简为残留实现 + re-export 新树同名类，保证两条路径是**同一类对象**（实测 `spatial.SE3 is geometry.SE3 == True`、`rigid_body.RigidBodyState is types.RigidBodyState == True`），避免双份类型在 `ty` 下冲突。
- `legacy_surface_registry.json` 由 31 条收缩到 14 条（现存生产+脚本 finding 15 条全部登记，未放宽扫描）。新增 2 条 `preparation/assembly/{front_axle,vehicle}.py [elements]`——作者层装配仍需构造力元件对象，`elements/` 因 A1 阻断不可动。
- 测试只改归属不改断言意图（`tests/core/{test_constraints,test_rigid_body}.py`、`tests/cases/kc_quasi_static/test_contract_boundary.py`、`tests/adams/*` 3 个、`tests/vehicle/*` 2 个）。
- 补 06 验收 1 的架构断言（`tests/architecture/test_unified_simulation_boundaries.py`：`test_preparation_authors_data_and_never_solves_or_decodes`、`test_preparation_does_not_reach_the_solver_entry_points`）；已实测该断言对注入的 `..kernel` 依赖会失败（非空过）。

第 6 项处置登记：`raw/step6_pending_deletion_registration.md`（待删除清单 file:line、阻断原因三类、解除条件三项）。**未做**：未改 `elements/**` 逻辑、未改 `api.py:737-777` 取值来源、未启用可选通道、未建 `report`、未删除任何旧目录或 `pac2002_scope.py` 文件本体。

一处登记的落地偏差：`pac2002_unsupported_native_reasons` 落 `kernel/capabilities.py` 而非 `adams/`——`schema/pac2002_scope.py` 的校验需调用它，而 `schema` 不得 import `adams`（否则默认导入路径会拉起整个 `adams` 包，破坏 `test_default_imports_do_not_load_adams` 的边界）；`adams/` 保留的是 Adams 证据常量。
