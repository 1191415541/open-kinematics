- 任务：迁移调用方并删除旧模块
- 形态：single-full（Epic 子任务）
- 进度：5/5 步骤已完成；删除后完整门禁与阶段记录审计全部通过，04 已关闭
- 当前：vehicle_dynamics.py 已删除；删除后功能门禁与六项静态门禁逐条 exit0 并即时写入父级 PROGRESS 的 子任务 04-post-delete 记录；--progress-records 04-post-delete 与 04 均 exit 0
- 验证：pre-delete 功能 364 passed、1 skipped、1 xfailed；post-delete 功能 364 passed、1 skipped、1 xfailed（533.42s）；ruff/compileall/ty/diff/reference/architecture 及阶段审计均 exit 0；独立复审 e0a6b676 PASS
- 文件：`.codex-tasks/20260920-unified-preparation-cutover/tasks/04-delete-legacy/`

## 已完成（步骤 1-3）

### 调用方迁移（新归属：`preparation.vehicle_dynamic` / `results.vehicle` / `vehicle.service`）

| 文件 | 改动 |
|---|---|
| `src/suspension_multibody/__init__.py` | `VehicleDynamicsResult` 改自 `results.vehicle`，`run_vehicle_dynamics` 改自 `vehicle.service`；`__all__` 不变 |
| `src/suspension_multibody/cli.py` | `run_vehicle_dynamics_command` 内的两处导入同步迁移 |
| `tests/adams/test_full_vehicle_model.py` | `run_vehicle_dynamics` 改自 `vehicle.service` |
| `scripts/case_parity_check.py` | `run_vehicle_dynamics` 改自 `vehicle.service`；四处 `prepare_vehicle_run` 改自 `preparation.vehicle_dynamic` |
| `scripts/diagnose_native_initial_pac_moments.py`、`scripts/measure_native_fiala_solver_time.py`、`scripts/run_full_native_three_model_comparison.py` | `run_vehicle_dynamics` 改自 `vehicle.service` |
| `scripts/diagnose_native_pac2002_contact_frame.py` | 四个私有 preparation helper 改自 `preparation.vehicle_dynamic` |
| `scripts/run_native_tire_rig.py` | 两处注释中的旧模块引用改为 `preparation/vehicle_dynamic` |
| `scripts/kc_legacy_path_check.py` | `NATIVE_PATHS` 由旧 `vehicle_dynamics.py` 改为 `preparation/vehicle_dynamic.py`、`vehicle/service.py`、`axle_dynamics`，docstring 同步；删除后该检查仍可失败 |
| `docs/axle_dynamics_architecture.md` | 模块清单新增 `preparation/vehicle_dynamic.py`、`cases/vehicle_dynamic.py`、`vehicle/service.py`、`results/vehicle.py`，装配段落改指 `preparation/vehicle_dynamic.py` |

未改动：`io/artifacts.py` 的 `vehicle_dynamics_result` 是 artifact 类型标识，按要求保留；`tests/vehicle/test_service_contract.py` 中拼接形式书写的旧模块名保留（03 归属，删除后自动退化）。

### 历史 `DynamicResultBundle` 读取兼容（步骤 4 的前置）

- 新增 `tests/schema/test_dynamic_result_compat.py`：先红后绿。
- 红（修复前）：同一文件 9 failed、9 passed，退出码 1；合法 bundle 经公开 `load_dynamic_result` 报 `unsupported schema_version None; expected 1`，根因是 `schema/loader.py::_read` 要求顶层版本，而 `DynamicResultBundle` 只允许 `manifest` 版本。
- 最小修复：`_read(path, *, version_path=("schema_version",))` + `_version_at()`；`load_dynamic_result` 传入 `version_path=("manifest", "schema_version")`。其它五个 loader 调用不变，schema 未放宽：bundle 顶层 `schema_version` 仍按未知字段（`extra_forbidden`）拒绝，manifest 版本缺失/非1 被拒。
- 新增测试覆盖：生产 `model_dump_json` 往返、schema 合法 JSON/YAML 样例、manifest 版本缺失/非1、bundle/manifest/sample 未知字段、bundle 顶层 `schema_version`、五个输入 loader 的顶层版本反向约束，以及 `adams/time_domain.py` 历史 adapter 的 body 过滤、时间排序、缺 body/metrics 失败和统一 `TimeSeriesResult` 分支。
- 无真实历史 fixture，因此只声明「schema 合法样例往返」兼容，不声明真实历史 artifact 兼容。

## 当前执行与恢复信息

- 正式删除前日志位于 scratch 的 04-pre-gate-*，删除后日志位于 scratch 的 04-post-gate-*；完整命令、退出码、摘要见父 PROGRESS。删除动作独立于门禁，于全部门禁通过后用 Edit REM 执行。
- 复审后补修：cases/vehicle_dynamic.py仅更新docstring旧路径；kc_legacy_path_check.py把results/vehicle.py补入扫描范围，定向脚本与完整静态检查通过。
- 删除后功能门禁由 7ddac9f6-9e79-452c-ad0d-6f243ef0d853 运行，364 passed/1 skipped/1 xfailed（533.42s），日志 04-post-gate-pytest.log 与 .exit；随后 ruff、compileall、ty、diff、architecture scan、post-delete reference scan 逐条执行并分别即时写入父记录。
- 04 已收口：TODO 第 5 步 DONE、SUBTASKS 第 4 行 DONE、Epic/父 PROGRESS 恢复信息 4/5；`--progress-records 04-post-delete` 与 `04` 均 exit 0。终局回归归子任务 05。

## 本步骤记录索引

| 命令 | 退出码 | 摘要 | 日志 |
|---|---|---|---|
| `pytest tests/schema/test_dynamic_result_compat.py -q`（修复前） | 1 | 9 failed、9 passed；复现合法 bundle 读取失败 | `$PI_SCRATCH_DIR/04-dynamic-result-compat-red.log` |
| `pytest tests/schema/test_dynamic_result_compat.py -q`（修复后） | 0 | 18 passed | `$PI_SCRATCH_DIR/04-dynamic-result-compat-green.log` |
| `pytest tests/schema/test_dynamic_result_compat.py tests/io/test_artifacts_unified.py -q` | 0 | 24 passed | `$PI_SCRATCH_DIR/04-history-artifact.log` |
| `pytest tests/cli -q` | 0 | 5 passed | `$PI_SCRATCH_DIR/04-cli.log` |
| `ruff check src tests scripts` | 0 | All checks passed（首轮 8 条 I001/D213 已修复） | `$PI_SCRATCH_DIR/04-ruff.log` |
| `compileall`（迁移文件） | 0 | 通过 | `$PI_SCRATCH_DIR/04-compileall-scoped.log` |
| `ty check .` | 0 | All checks passed | `$PI_SCRATCH_DIR/04-ty.log` |
| 顶层公开导出同一性 `python -c` | 0 | `public-api-identity=ok` | `$PI_SCRATCH_DIR/04-public-api-identity.log` |
| `python scripts/kc_legacy_path_check.py` | 0 | native 路径引用 0 | `$PI_SCRATCH_DIR/04-kc-legacy-path-check.log` |
| `pytest tests/architecture -q` | 0 | 44 passed | `$PI_SCRATCH_DIR/04-architecture.log` |
| `pytest tests/adams/test_full_vehicle_model.py -q` | 0 | 37 passed、1 skipped | `$PI_SCRATCH_DIR/04-adams-full-vehicle.log` |
| `pytest tests/adams tests/vehicle tests/cases -q` | 0 | 350 passed、1 skipped、1 xfailed | `$PI_SCRATCH_DIR/04-adams-vehicle-cases.log` |
| `legacy_reference_scan.py --pre-delete` | 0 | 无命中（迁移前同一命令退出码 1、命中 9 文件） | `$PI_SCRATCH_DIR/04-predelete-migration.log` |
| `architecture_contract_scan.py` | 0 | 唯一 `decode_result`、唯一 native 提交点 | `$PI_SCRATCH_DIR/04-architecture-contract-scan.log` |
| `planning_contract_scan.py --progress-records 04-pre-delete` | 0 | 阶段记录可识别、pre/post 未混用 | `$PI_SCRATCH_DIR/04-pre-delete-records-audit.log` |
| `pytest vehicle+results+cases/adams/cli/architecture/schema/io -q`（删除后） | 0 | 364 passed、1 skipped、1 xfailed（533.42s） | `$PI_SCRATCH_DIR/04-post-gate-pytest.log` |
| `ruff check src tests scripts`（删除后） | 0 | All checks passed | `$PI_SCRATCH_DIR/04-post-gate-ruff.log` |
| `compileall src tests scripts`（删除后） | 0 | 通过，无输出 | `$PI_SCRATCH_DIR/04-post-gate-compileall.log` |
| `ty check .`（删除后） | 0 | All checks passed | `$PI_SCRATCH_DIR/04-post-gate-ty.log` |
| `git diff --check`（删除后） | 0 | 无空白错误 | `$PI_SCRATCH_DIR/04-post-gate-diff.log` |
| `architecture_contract_scan.py`（删除后） | 0 | 唯一 decode_result、唯一 native 提交点 | `$PI_SCRATCH_DIR/04-post-gate-architecture.log` |
| `legacy_reference_scan.py --post-delete` | 0 | 目标文件不存在，交付目录无命中 | `$PI_SCRATCH_DIR/04-post-gate-references.log` |
| `planning_contract_scan.py --progress-records 04-post-delete` | 0 | 阶段记录可识别、pre/post 未混用 | `$PI_SCRATCH_DIR/04-post-delete-records-audit.log` |
| `planning_contract_scan.py --progress-records 04` | 0 | 04 全部原子命令成功记录齐全 | `$PI_SCRATCH_DIR/04-records-audit.log` |
