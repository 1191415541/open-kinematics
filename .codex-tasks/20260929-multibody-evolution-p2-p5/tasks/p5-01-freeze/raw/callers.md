# p5-01 冻结现状 · 调用者盘点（改造前基线）

计数口径：`grep -rn` 逐行命中数（含 import 行、类型注解、定义/文档行）；文件数用 `grep -rl`。
所有命令与退出码见 `run_log.md`。**这是 p5-02 之前的基线**，p5-02 只须证明「历史调用者全部可用」，不要求计数不变（SPEC.md:73）。

## 1. `FrontAxleModel`

### 1.1 生产代码

命令：
```bash
grep -rn --include=*.py "FrontAxleModel" packages/*/src src
grep -rlc --include=*.py ... | grep -v ":0$"
```
**逐行命中 82 行，落在 25 个文件**。（`src/` 下无命中，全部在 `packages/suspension_multibody/src/`。）

| 文件 | 命中行数 |
|---|---|
| `adams/full_vehicle_model.py` | 5 |
| `adams/reference.py` | 2 |
| `adams/strict_c.py` | 6 |
| `adams/strict_k.py` | 2 |
| `adams/time_domain_gate.py` | 3 |
| `adams/vehicle_kc_time_domain.py` | 2 |
| `adapters/geometry_contract.py` | 3 |
| `api.py` | 5 |
| `authoring/bridge.py` | 5 |
| `authoring/solver.py` | 6 |
| `preparation/kc_quasi_static.py` | 3 |
| `schema/loader.py` | 3 |
| `schema/model.py` | 2 |
| `schema/vehicle.py` | 3 |
| `schema/__init__.py` | 2 |
| `simulation/replay.py` | 2 |
| `studies/assembly.py` | 4 |
| `subsystems/assembler.py` | 2 |
| `subsystems/chassis.py` | 1 |
| `subsystems/entry.py` | 2 |
| `subsystems/explicit.py` | 4 |
| `subsystems/si_assembly.py` | 5 |
| `subsystems/types.py` | 3 |
| `vehicle/roll_centers.py` | 4 |
| `__init__.py` | 3 |

（路径前缀均为 `packages/suspension_multibody/src/suspension_multibody/`。）

**扣掉 import 行与 `class` 定义行后的真实使用点：65 行**。关键锚点：
- `schema/model.py:157` — 定义处
- `schema/vehicle.py:257-258` — `front_axle: FrontAxleModel` / `rear_axle: FrontAxleModel`（EPIC F18 写的 `:252-253` 已过期，现为 `:257-258`）
- `api.py:101` / `:155` / `:193` / `:354` — 四个签名位置
- `schema/loader.py:59` — `load_model(...) -> FrontAxleModel`
- `subsystems/assembler.py:124` — `axle: FrontAxleModel`
- `subsystems/si_assembly.py:154/167/432/715`
- `authoring/bridge.py:115`、`authoring/solver.py:549/568/762/777`
- `vehicle/roll_centers.py:70/81/103`

### 1.2 测试

命令：
```bash
grep -rl --include=*.py "FrontAxleModel" \
  packages/suspension_multibody/tests packages/suspension_kernel/tests \
  packages/suspension_contracts/tests tests
grep -rl --include=test_*.py "FrontAxleModel" packages/suspension_multibody/tests
```
- 全部命中文件：**53 个**
- 其中文件名匹配 `test_*.py`：**48 个** ← 与 EPIC F18（`EPIC.md:162`）写的「48 个测试文件命中」**一致**
- 余下 5 个是非 `test_` 前缀的公共设施（夹具/门禁探针）：
  - `tests/architecture/isolated_native_probe.py`（2）
  - `tests/benchmark_fixture.py`（3）
  - `tests/cases/kc_quasi_static/kc_fixtures.py`（4）
  - `tests/composable/fixtures.py`（3）
  - `tests/conftest.py`（5）
- 逐行命中总数（上述四个 tests 根）：**161 行**

48 个 `test_*.py` 全清单（按逐行命中数降序）：
```
5  tests/subsystems/test_explicit_in_composition.py
4  tests/api/test_api.py
4  tests/cases/kc_quasi_static/test_drives_come_from_the_rig.py
4  tests/model/test_vehicle.py
4  tests/simulation/test_no_production_bridge.py
4  tests/simulation/test_orthogonal_requests.py
4  tests/subsystems/test_optional_symmetry.py
4  tests/vehicle/test_native_vehicle.py
3  tests/adams/test_strict_c.py
3  tests/adams/test_time_domain_axle.py
3  tests/adams/test_time_domain_vehicle_kc.py
3  tests/analysis/test_axle_dynamic.py
3  tests/analysis/test_k_metrics.py
3  tests/analysis/test_vehicle_dynamic.py
3  tests/connections/test_policy.py
3  tests/connections/test_policy_is_wired_into_production.py
3  tests/joints/test_any_joint_encodes.py
3  tests/model/test_axle_road_surface.py
3  tests/model/test_force_assembly.py
3  tests/model/test_front_axle.py
3  tests/rigs/test_the_rig_reaches_the_product.py
3  tests/schema/test_model_case.py
3  tests/schema/test_vehicle.py
3  tests/simulation/test_quasi_static_tire.py
3  tests/simulation/test_replay.py
3  tests/simulation/test_wheel_centre_selection.py
3  tests/studies/test_one_assembly_two_studies.py
3  tests/subsystems/test_availability_matrix.py
3  tests/subsystems/test_bench_in_composition.py
3  tests/subsystems/test_drive_subsystem.py
3  tests/subsystems/test_rig_link.py
3  tests/subsystems/test_runtime_face.py
3  tests/subsystems/test_si_composition.py
3  tests/subsystems/test_the_rig_is_not_invasive.py
3  tests/subsystems/test_vehicle_assembler.py
3  tests/subsystems/test_vehicle_composition.py
3  tests/subsystems/test_wheel_lifecycle.py
3  tests/templates/test_template_drives_the_subsystem.py
3  tests/vehicle_assembly/test_vehicle_assembly.py
2  tests/api/test_decoding_belongs_to_results.py
2  tests/composable/test_extension_proof.py
2  tests/e2e/test_e2e.py
2  tests/model/test_dynamic_mass.py
2  tests/results/test_element_wrench.py
2  tests/schema/test_dynamic_result_compat.py
2  tests/subsystems/test_assembly_matches_snapshot.py
2  tests/subsystems/test_steering_can_be_absent.py
2  tests/subsystems/test_three_axle_assembly.py
```
（前缀 `packages/suspension_multibody/`。）

**差异说明**：F18 说「生产约 20 处」——实测 25 个生产文件 / 82 行 / 扣 import 后 65 行调用点。数量级一致，差异来源是「处」的口径（文件数 vs 行数）；本行以实测为准，父文件不改。

## 2. `run_case`

命令：`grep -rn --include=*.py "\brun_case\b" packages src tests scripts` → **54 行，17 个文件**。

真实调用点（扣掉 `def`、`__init__.py` 的登记行与文档行）：

生产 / 脚本（2 个文件，10 处调用）：
```
packages/suspension_multibody/src/suspension_multibody/cli.py:11   from .api import run_case, run_dynamic_case
packages/suspension_multibody/src/suspension_multibody/cli.py:55   bundle = run_case(load_model(model), load_case(case), out)
packages/suspension_multibody/scripts/accept_composable_architecture.py:341,546,616,1186,1200,1274,1313,1332
```
（`api.py:371` 是文档字符串里提到 `run_case`，不是调用。）

测试（15 个文件）：
```
tests/adapters/test_geometry_contract.py:124(import),131
tests/api/test_api.py:5(import),68,92,110
tests/api/test_decoding_belongs_to_results.py:195,210
tests/api/test_no_steering_shrinks_rack.py:116,132,142,156,162,188
tests/architecture/isolated_native_probe.py:31(import),68
tests/authoring/test_provenance_and_bench.py:56,70
tests/authoring/test_solver_integration.py:215,495
tests/composable/test_extension_proof.py:145,175,213,257,385,386,426,427
tests/e2e/test_e2e.py:5(import),26
tests/outputs/test_bypass.py:42  （只在「禁止调用名」元组里列字面量，不是调用）
tests/rigs/test_k_grid_shrinks.py:153(import),169,179
tests/rigs/test_the_rig_reaches_the_product.py:339
tests/simulation/test_quasi_static_tire.py:236(import),240
```
登记行：`__init__.py:21`（`TYPE_CHECKING`）、`:56`（`_PUBLIC_NAMES`）、`:76`（`__all__`）。

## 3. `run_dynamic_case`

命令：`grep -rn --include=*.py "\brun_dynamic_case\b" packages src tests scripts` → **21 行，10 个文件**。

生产（3 个文件，5 处调用）：
```
packages/suspension_multibody/src/suspension_multibody/adams/time_domain_gate.py:15(import),149
packages/suspension_multibody/src/suspension_multibody/adams/vehicle_kc_time_domain.py:11(import),44
packages/suspension_multibody/src/suspension_multibody/cli.py:11(import),85
```
定义：`api.py:154`。登记：`__init__.py:21/57/77`。

测试（7 个文件）：
```
tests/adams/test_time_domain_axle.py:11(import),87
tests/adams/test_time_domain_vehicle_kc.py:17(import),95
tests/analysis/test_axle_dynamic.py:7(import),62,90
tests/analysis/test_vehicle_dynamic.py:7(import),66,80
tests/outputs/test_bypass.py:43  （禁止调用名字面量，不是调用）
```

## 4. 谁直接调 `run_contract`（SPEC.md:61 单列一类）

命令：`grep -rn --include=*.py "\brun_contract\b" packages src tests scripts`

定义与绑定：
```
packages/suspension_multibody/src/suspension_multibody/kernel/__init__.py:186  def run_contract(
packages/suspension_multibody/src/suspension_multibody/kernel/__init__.py:30   __all__ 登记
```
**生产代码里唯一的调用点**：
```
packages/suspension_multibody/src/suspension_multibody/simulation/backend.py:7   from ..kernel import ContractRun, run_contract
packages/suspension_multibody/src/suspension_multibody/simulation/backend.py:23  def run(self, compiled: CompiledSimulation) -> ContractRun:
packages/suspension_multibody/src/suspension_multibody/simulation/backend.py:24      return run_contract(
```
即 `NativeContractBackend.run`（`simulation/backend.py:20-24`），与 `test_public_api_boundary_gate.py:74-79 _is_backend_owner`
（`production` + `.../simulation/backend.py` + `function_scope == "run"`）**逐字对上**。**唯一归属成立。**

其余命中全部是门禁扫描器自身的规则常量和测试夹具字符串：
- `tests/architecture/legacy_surface_gate.py:70`（`SOLVE_NAMES` 元组，扫描器规则常量）
- `tests/architecture/test_public_api_boundary_gate.py:53/150/173/188/289/292/304/307/321`
- `tests/architecture/test_legacy_surface_gate.py:330/333/358/360`
- `tests/architecture/test_unified_simulation_boundaries.py:141`
- `tests/outputs/test_bypass.py:55/95/97/102`（禁止调用名字面量）
- `tests/simulation/test_compilers.py:340`（`monkeypatch.setattr(backend_module, "run_contract", ...)`，借道 backend）
