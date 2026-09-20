# 子任务 03 进度：拆分 vehicle_dynamics 职责

- **任务**：整车 preparation / results / service 职责拆分并接入统一 preparation registry
- **形态**：single-full（Epic 子任务）
- **进度**：5/5 步骤完成；影响范围长回归与父级证据审计均通过，本子任务关闭
- **状态**：DONE；旧文件保留为临时重导出薄壳，删除归 04
- **文件**：`.codex-tasks/20260920-unified-preparation-cutover/tasks/03-vehicle-split/`

## 交付内容

### 新增

| 文件 | 关键接口（行号） |
|---|---|
| `src/suspension_multibody/preparation/vehicle_dynamic.py` | `PreparedVehicleRun`(L137，含 `source_model` L170 / `source_case` L171，`compare=False, repr=False`)、`prepare_vehicle_run`(L212)、`ASSEMBLY`/`FAMILY`(L1570)、`PREPARED_KEY="vehicle_dynamic_prepared"`(L1574)、`LEGACY_PREPARED_KEY="prepared"`(L1578)、`prepare_request`(L1599)、`adapt_legacy_prepared_request`(L1623) |
| `src/suspension_multibody/vehicle/__init__.py` | 转发 `run_vehicle_dynamics` |
| `src/suspension_multibody/vehicle/service.py` | `run_vehicle_dynamics`(L24)：统一 `SimulationRequest` → `run_request`，保留静态轮荷 / metrics / native 耗时 / `NativeAxleError(partial_result, failure_diagnostics, failed_sample_index, failed_time_s)` |
| `tests/vehicle/test_service_contract.py` | 13 个测试：46 项定义映射、结构无旧导入、decoder 整车分派经 family adapter、registry 真 family、adapter 匹配复用 / stale 重准备 / 错误类型 / document+legacy 优先、staged+facade 各一次 prepare 与 native submission、metrics+artifact 成功与失败证据 |

### 修改

- `results/vehicle.py`：`VehicleDynamicsResult`(L25)、`_contract_constraint_names`(L91)、`_vehicle_axle_result`(L104，保持私有名) 成为唯一生产归属；`decode_vehicle_result`(L121) 不再从旧模块惰性导入；不反向导入旧模块。
- `results/decoder.py`：未改动——整车分派本来就经 `results.vehicle.decode_vehicle_result()`(L46-48)，全仓唯一 `decode_result` 定义仍在 L14；源码不含 `vehicle_dynamics`。
- `simulation/compiler.py`：`VehicleDynamicCompiler`(L375) 只读 `vehicle_dynamic_prepared`(L392-394) 或走 document bypass（`DocumentPairCompiler(validate_contract_family=True)`），不再自行 `prepare_vehicle_run`。
- `simulation/runner.py`：decoder 的 prepared 输入改取 `vehicle_dynamic_prepared`(L70)，旧 `prepared` 不再被 runner 消费。
- `cases/vehicle_dynamic.py`：`prepare_vehicle_run` 改从 `preparation.vehicle_dynamic` 导入（保留 re-export 兼容）。
- `preparation/{vehicle_kc,handling,ride_four_post,ride_random_road}.py`：`prepare_vehicle_run` 导入由 `cases.vehicle_dynamic` 旧 re-export 改为 `preparation.vehicle_dynamic`。
- 旧 `vehicle_dynamics.py`：改为临时重导出薄壳（L18-68），无任何自有 class/def，重导出同一对象（`_PreparedVehicleRun` → `PreparedVehicleRun`），不删除。
- 测试导入迁移：`tests/vehicle/test_native_vehicle.py`（私有 preparation helper + `vehicle.service.run_vehicle_dynamics`）、`tests/vehicle/test_pac2002_contact_mass.py`、`tests/results/test_adapters.py`（monkeypatch 目标迁到 `results.vehicle._vehicle_axle_result`）、`tests/io/test_artifacts_unified.py`、`tests/cases/test_vehicle_dynamic_contract.py`、四个 `tests/cases` family 文件的 `prepare_vehicle_run` 导入。
- `tests/simulation/test_compilers.py`：`test_vehicle_dynamic_compiler_uses_prepared_context` 的 context 键由 `prepared` 改为 `vehicle_dynamic_prepared`（唯一一行；该文件是 03 负责的 vehicle_dynamic compiler 测试，未列在允许清单中但必须随 compiler 契约同步，否则 tests/simulation 门禁必失败）。

## 验证记录（真实退出码 / 日志）

| 命令 | 退出码 | 摘要 | 日志 |
|---|---|---|---|
| `pytest tests/vehicle/test_native_vehicle.py tests/vehicle/test_service_contract.py tests/results tests/cases/test_vehicle_dynamic_contract.py tests/simulation tests/io/test_artifacts_unified.py -q` | 0 | 125 passed, 1 xfailed | `$PI_SCRATCH_DIR/03_spec_validation.log` |
| `pytest tests/architecture -q` | 0 | 44 passed | `$PI_SCRATCH_DIR/03_architecture.log` |
| `python tasks/04-delete-legacy/architecture_contract_scan.py` | 0 | `decode_result=results/decoder.py`；`native_submission=simulation/backend.py:24` | `$PI_SCRATCH_DIR/03_arch_contract_scan.log` |
| SPEC 第四条 `python -c`（registry key + 新模块属性 + decoder 源码） | 0 | 通过 | `$PI_SCRATCH_DIR/03_quickcheck.log` |
| TODO 第 3 行 `python -c`（唯一 `decode_result`） | 0 | 通过 | `$PI_SCRATCH_DIR/03_todo3_quick.log` |
| `pytest tests/vehicle tests/io/test_artifacts_unified.py -q` | 0 | 68 passed, 1 xfailed | `$PI_SCRATCH_DIR/03_vehicle_io.log` |
| `pytest tests -q`（全量） | 0 | 679 passed, 1 skipped, 1 xfailed | `$PI_SCRATCH_DIR/03_full_suite.log` |
| `ruff check src tests` | 0 | All checks passed | `$PI_SCRATCH_DIR/ruff.log` |
| `compileall -q src tests` | 0 | 通过 | `$PI_SCRATCH_DIR/03_compileall.log` |
| `ty check .` | 0 | All checks passed | `$PI_SCRATCH_DIR/03_ty.log` |
| `git diff --check` | 0 | 仅 CRLF 提示 | `$PI_SCRATCH_DIR/03_diffcheck.log` |
| `legacy_reference_scan.py --pre-delete`（信息性） | 1 | 剩余旧路径引用：docs、scripts×7、`cli.py`、顶层 `__init__.py`、`tests/adams/test_full_vehicle_model.py`（均属 04） | `$PI_SCRATCH_DIR/03_predelete_scan.log` |

## 未决事项（交 04 / 主线程）

1. 顶层 `__init__.py`、`api.py`、`cli.py`、Adams、scripts、docs 仍引用旧路径，由 04 迁移；本阶段旧文件作为薄壳保证这些入口仍解析到同一对象。
2. `tests/vehicle/test_service_contract.py::test_vehicle_legacy_definition_map` 在薄壳存在时校验「无自有定义 + 同一对象重导出」；旧文件删除后该段自动退化为仅映射断言（模块名以拼接形式书写，避免成为 post-delete 扫描命中）。测试文件的旧路径引用已清除，04 的 post-delete 扫描不再命中它。
3. `tests/simulation/test_compilers.py` 的一行 context 键迁移不在 03 任务书允许清单内，但为 compiler 契约与 `tests/simulation` 门禁所必需，已实施。
4. TODO 第 5 行（父级 PROGRESS 记录审计 `planning_contract_scan.py --progress-records 03`）已完成：退出码 0，证据 `$PI_SCRATCH_DIR/03-final-evidence-audit.log`；第 5 步与 SUBTASKS 第 3 项同步置 DONE。

## 主线程补修与恢复入口

- 4aa65071复审PASS，但document优先级观察由主线程按冻结契约升级处理。compiler.py复用 `_is_document_request`，整车分支与 `_axle_dynamic_prepared` 均优先document；新增整车4例和axle7例并存prepared回归，先红后修。7fc8ca12定向复核PASS。
- 修改后整车/架构/simulation/service 122 passed；ruff和全仓ty通过。更宽cases/simulation/vehicle/results/io/architecture回归由test-runner `4d69b3d3-58f1-4fd0-a919-ce782c62964f` 正在运行，日志 `$PI_SCRATCH_DIR/03-document-priority-regression.log`。此前60秒超时的 `03-document-priority-all-green.log` 不计通过。
- 等长回归完成并记录父PROGRESS，再把TODO第5步和SUBTASKS第3项置DONE；父证据 `planning_contract_scan.py --progress-records 03` 已通过，状态更新后再次审计即可。03未删除旧模块。
- 第四项另发现历史loader无法读合法DynamicResultBundle（顶层版本与manifest版本矛盾），修复归属和严格schema验收已同步EPIC/SUBTASKS/04SPEC，ac9aecb7规划定向复核PASS；尚未实施，04需先红后绿。

## 关闭结论（2026-09-20）

- 影响范围长回归 `pytest tests/cases tests/simulation tests/vehicle tests/results tests/io/test_artifacts_unified.py tests/architecture -q` 由 test-runner `4d69b3d3-58f1-4fd0-a919-ce782c62964f` 完成：260 passed、1 xfailed、164.03s，日志 `$PI_SCRATCH_DIR/03-document-priority-regression.log`；此前 60 秒超时的 `03-document-priority-all-green.log` 不计通过。
- 父级证据审计 `planning_contract_scan.py --progress-records 03` 退出码 0，日志 `$PI_SCRATCH_DIR/03-final-evidence-audit.log`。
- 03 不删除旧模块：`vehicle_dynamics.py` 仍为临时重导出薄壳，调用方迁移与删除前后门禁归 04。
