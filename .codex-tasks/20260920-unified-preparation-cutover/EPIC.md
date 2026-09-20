# Epic：多体装配前处理统一入口与整车旧模块删除

- **任务编号**：20260920-unified-preparation-cutover
- **创建日期**：2026-09-20
- **状态**：DONE
- **范围**：`packages/suspension_multibody` 的 simulation preparation、cases compiler、results、整车 service、Public API、CLI、Adams、脚本、测试和相关文档
- **形态**：Epic
- **前置条件**：`20260919-public-api-simulation-cutover` 已完成；既有 `SimulationRequest → compile_request() → NativeContractBackend` 运行骨架、results decoder、metrics 和统一 artifact IO 已可用

## 用户需求

> 按上述模块形态实现，迁移完毕后删除旧模块。

这里的“上述模块形态”指：建立统一的 preparation 生命周期和 registry；各 family 保留自己的物理语义和准备对象；`simulation` 负责调度，`cases` 负责 contract 文档编写，`results` 负责结果类型和解码；整车旧 `vehicle_dynamics.py` 在所有职责迁移且引用清零后删除。

## Goal

1. 新增统一的 preparation 协议、准备结果和 registry，使领域请求可通过统一入口执行 preparation。
2. 固定目标链路为 `SimulationRequest → prepare_request() → family compiler → run_request(compiled) → NativeContractBackend → RawContractResult → results.decoder → typed result → metrics/artifact`；同时保留 `run_request(SimulationRequest)` 作为一次性便捷 facade，内部严格按同一顺序执行。
3. 将 axle dynamic、vehicle K/C、K/C quasi-static、handling、ride four-post 和 ride random-road 的装配前处理分别归属明确的 preparation/cases 模块；`vehicle_dynamic` 的完整 preparation 迁移由整车职责拆分子任务收口，不把不同物理语义合并为一个巨型准备器。
4. 将 `vehicle_dynamics.py` 的准备、结果类型、结果映射和高层运行 service 分别迁移到 `preparation/vehicle_dynamic.py`、`results/vehicle.py` 和 vehicle service/API 边界；统一解码仍唯一归属 `results/decoder.py`。
5. 迁移生产、测试、脚本、CLI、Adams、导出和文档引用后，删除 `packages/suspension_multibody/src/suspension_multibody/vehicle_dynamics.py`。
6. 保持统一 runner、native 唯一提交点、结果通道/单位/异常/partial evidence、artifact 协议和历史 `DynamicResultBundle` 读取兼容不变。

## 目标链路

显式分阶段调用使用以下顺序：

```text
Domain model/case
        ↓
SimulationRequest
        ↓
prepare_request() / PreparationRegistry
        ↓
family compiler / compile_request(prepared.request)
        ↓
run_request(compiled)
        ↓
NativeContractBackend
        ↓
RawContractResult → results.decoder → typed result → metrics/artifact
```

领域调用也可以直接执行 `run_request(SimulationRequest)`；该 facade 只是在一个函数内依次调用上述 preparation、family compiler 和 compiled-request submission 阶段，不改变阶段顺序。已是 contract document 的请求仍走：

```text
Contract documents → SimulationRequest → prepare_request() document bypass → compiler validation → run_request(compiled) → NativeContractBackend
```

## 架构边界

### simulation

负责 `PreparedSimulation`、preparation registry、准备结果注入和统一生命周期调度；不理解具体物理方程，不提交 native，不解码结果。

### preparation

负责领域模型的装配、单位归一化、输入信号映射、solver 配置和 compiler 所需的准备上下文。每个 family 可以有独立准备器和独立准备类型。

### cases

只负责将准备结果和领域输入写成 model/case contract document、payload、layout、metadata；不提交 native，不解码结果，不计算指标。

### results

`results.decoder.decode_result()` 是从 `RawContractResult` 到 typed result 的唯一统一分派入口；`results.vehicle` 只负责 `VehicleDynamicsResult`、整车结果映射和 `decode_vehicle_result()` family adapter，不重新定义统一 `decode_result()`，不执行 preparation，不提交仿真。`results.decoder` 的整车分派必须接线到 `results.vehicle.decode_vehicle_result()`，且不得再导入旧 `vehicle_dynamics` 模块；该接线与旧导入清理归子任务 03。

### service/API

负责构造请求、调用统一 runner、补充 metrics、错误证据和 artifact 写出；不重复实现 contract 编译或 native 调用。
## 冻结的 preparation 契约

统一运行时契约固定如下，子任务不得自行改变签名、键名或 bypass 规则：

```python
@dataclass(frozen=True)
class PreparedSimulation:
    request: SimulationRequest
    value: Any = None
    context: Mapping[str, Any] = field(default_factory=dict)
    metadata: Mapping[str, Any] = field(default_factory=dict)

class Preparation(Protocol):
    assembly: str
    family: str

    def prepare(self, request: SimulationRequest) -> PreparedSimulation: ...

def prepare_request(
    request: SimulationRequest,
    *,
    registry: PreparationRegistry | None = None,
) -> PreparedSimulation: ...

def run_request(
    request: SimulationRequest | CompiledSimulation,
    *,
    preparation_registry: PreparationRegistry | None = None,
    registry: CompilerRegistry | None = None,
    backend: SimulationBackend | None = None,
) -> SimulationRun: ...
```

- registry key 规范化固定为 `str(value).strip().casefold()`；空的 assembly/family 必须抛出 `ValueError`，不定义额外别名，registry 内只保存规范化后的二元组；未注册键必须抛出包含该二元组的可诊断 `KeyError`。
- `prepare_request()` 将 preparation 返回的 `context` 与原 request context 合并，并以 preparation context 覆盖同名内部键；compiler 消费合并后的 request。
- 复用匹配的 `prepared_simulation` 时仍合并当前 request context；不保证返回包装与缓存实例身份相同，不重复 family preparation，准备值保持复用且值图无环。
- `run_request(SimulationRequest)` 必须先调用 `prepare_request()`，再调用 `compile_request()`，最后提交编译对象；`run_request(CompiledSimulation)` 是目标链路中的 submission stage，不查 preparation registry、不重复编译。
- 若 request context 含 `model_document`、`case_document`、`model_document_pair`、`model_payload` 或 `case_payload`，或 model/case 已是 contract document，则视为 document request：不查 preparation registry，不重复装配，只做 compiler contract 校验；document bypass 优先于其它 preparation 路由。
- 若 request context 含 `prepared_simulation`，只有其值为 `PreparedSimulation`、其 request 的规范化 assembly/family 与当前 request 相同，且 `prepared.request.model is request.model`、`prepared.request.case is request.case` 时才可复用；否则必须重新 preparation。旧的 family-specific `prepared` 键仅由迁移适配器包装为 `prepared_simulation`，不得成为新的 family 协议。
`prepared` 迁移适配责任固定为：`preparation/vehicle_dynamic.py::adapt_legacy_prepared_request` 只处理 `("vehicle", "vehicle_dynamic")` 的旧 `prepared` 输入，将其包装为带有规范化 request 身份和 `vehicle_dynamic_prepared` context 的 `PreparedSimulation`；`simulation/preparation.py::prepare_request()` 在 document bypass 之后调用该适配器，再按统一匹配规则处理 `prepared_simulation`。旧键不得被其它 family 或 compiler 直接消费。
任务 01 的 `tests/simulation/test_preparation.py` 必须在六个非整车 family 和 `preparation/vehicle_dynamic.py` 尚未实现的阶段独立通过：用可注入的 `PreparationRegistry`/`Preparation` 替身验证 adapter 协议、legacy `prepared` 包装、document request + legacy `prepared` 时 document bypass 优先、包装后匹配复用和 stale identity 重新 preparation，并以计数 backend 证明 staged/facade 各只提交一次。01 不得捕获或改写 `ImportError`/`ModuleNotFoundError`，不得在生产代码引入占位 family；真实 family 与 legacy adapter 的集成由任务 02（六个非整车 family）和任务 03（`vehicle_dynamic`）分别验证，终局门禁不得以替身覆盖替代真实 family 验证。
- `simulation/preparation.py` 负责唯一的默认 registry 组装：通过固定的延迟 import 表按七个规范化 key 注册七个 family，key 枚举、document bypass 和 `prepared_simulation` 复用检查不触发 family import；只有对某个 key 执行 family preparation 时才 import 对应模块。family 模块只定义自己的 preparation 类型/实现，不直接修改共享 registry。
- 七个 family 模块均导出 `prepare_request(request: SimulationRequest) -> PreparedSimulation`，这是中央延迟 import 表的固定调用契约。
- preparation 只返回装配/归一化/输入映射上下文；native 提交、结果解码、metrics 和 artifact 写出仍分别归属 compiler、backend、results、metrics 和 io.artifacts。
## Family preparation 归属矩阵

完整矩阵见 `PREPARATION_MATRIX.md`。以下文件级归属是本 Epic 的固定目标，不得以“合适归属”替代：

| registry key | preparation module / type | domain input | compiler / document owner | 验证范围 |
|---|---|---|---|---|
| `("axle", "axle_dynamic")` | `preparation/axle_dynamic.py::AxleDynamicPrepared` | `AxleDynamicsModel`, `AxleDynamicsCase` | `AxleDynamicCompiler`, `cases/axle_dynamic.py` | axle dynamic contract |
| `("axle", "kc_quasi_static")` | `preparation/kc_quasi_static.py::KcQuasiStaticPrepared` | front-axle model/case or explicit documents | `KcQuasiStaticCompiler`, `cases/kc_quasi_static/` | K/C contract + API |
| `("vehicle", "vehicle_kc")` | `preparation/vehicle_kc.py::VehicleKcPrepared` | vehicle assembly, wheel corners and K/C inputs | `VehicleKcCompiler`, `cases/vehicle_kc.py` | vehicle K/C |
| `("vehicle", "handling")` | `preparation/handling.py::HandlingPrepared` | vehicle model and steering shapes | `HandlingCompiler`, `cases/handling.py` | handling |
| `("vehicle", "ride_four_post")` | `preparation/ride_four_post.py::RideFourPostPrepared` | vehicle model and four-post signals | `RideFourPostCompiler`, `cases/ride_four_post.py` | four-post |
| `("vehicle", "ride_random_road")` | `preparation/ride_random_road.py::RideRandomRoadPrepared` | vehicle model and road components | `RideRandomRoadCompiler`, `cases/ride_random_road.py` | random-road |
| `("vehicle", "vehicle_dynamic")` | `preparation/vehicle_dynamic.py::PreparedVehicleRun` | `VehicleModel`, `VehicleDynamicCase` | `VehicleDynamicCompiler`, `cases/vehicle_dynamic.py` | vehicle dynamic |

`results/vehicle.py` 是 `VehicleDynamicsResult`、`VehicleResult` 和整车结果映射的唯一生产归属；高层执行 service 的固定目标是 `vehicle/service.py`。如果项目已有等价 API 文件，允许保留其公开入口，但实现必须转发到该 service，不能重新承载 preparation 或 contract 编译。

### `vehicle_dynamics.py` 完整职责清单

删除前必须按 `PREPARATION_MATRIX.md` 中的逐项清单核对当前旧文件的全部顶层定义；不得只靠路径扫描判断迁移完成。当前源文件 `packages/suspension_multibody/src/suspension_multibody/vehicle_dynamics.py` 的职责归属如下：

| 旧定义组 | 完整符号 | 新归属 |
|---|---|---|
| 常量 | `_WHEEL_NAMES`, `_ROAD_KIND`, `_PRESCRIBED_STEERING_TYPES` | `preparation/vehicle_dynamic.py` |
| preparation 数据对象 | `_VehicleSteeringBuffers`, `_VehicleRoadBuffers`, `_NativeVehicleModel`, `_PreparedVehicleRun`, `_BodyFrame` | `preparation/vehicle_dynamic.py`；`_PreparedVehicleRun` 对外改名为 `PreparedVehicleRun` |
| preparation 入口与校验 | `_select_assembly_mode`, `_validate_steering_topology`, `prepare_vehicle_run`, `_length_scale`, `_validate_units` | `preparation/vehicle_dynamic.py` |
| preparation 构建辅助 | `_build_static_rotation_gauges`, `_uses_horizontal_static_gauge`, `_output_times`, `_initial_body_state`, `_resolve_vehicle_body`, `_rotation_from_quaternion`, `_tuple3`, `_tuple4`, `_matrix3`, `_shift_point`, `_build_aerodynamic_drags`, `_build_joints`, `_build_coordinate_couplers`, `_build_elements`, `_damper_curve`, `_length_force_curve`, `_bushing_force_curves`, `_spring_force_curve`, `_tuple6`, `_wheel_forward_local`, `_build_tires`, `_build_steering`, `_resolve_named_body`, `_resolve_steering_rack`, `_steering_target_value`, `_steering_target_rate`, `_build_road`, `_build_wheel_torque_signals`, `_native_solver_settings` | `preparation/vehicle_dynamic.py` |
| typed result 与映射 | `VehicleDynamicsResult`, `_contract_constraint_names`, `_vehicle_axle_result` | `results/vehicle.py`；统一 `decode_result` 仍只在 `results/decoder.py` |
| 高层执行 service | `run_vehicle_dynamics` | `vehicle/service.py`；顶层公开入口只允许转发 |

子任务 03 必须用 `test_vehicle_legacy_definition_map` 和 import/architecture 测试证明上述全部定义组均已落入指定新归属；子任务 04 删除前必须再次核对清单，删除后不得从旧路径导入。

## 文件归属与共享写范围

- 子任务 01 唯一负责：`simulation/preparation.py`、`simulation/runner.py`、`simulation/__init__.py` 及 `tests/simulation/test_preparation.py`；其中 `simulation/preparation.py` 还负责固定七个 family 的延迟 import registry 组装和调用迁移 adapter，测试覆盖 document bypass 优先于 legacy adapter，不包含 family 物理逻辑。01 阶段的协议测试用可注入替身完成，门禁为 `tests/simulation` + `tests/architecture`，可独立通过。
- 子任务 02 唯一负责：新增 `preparation/` 包中 axle dynamic、K/C quasi-static、vehicle K/C、handling、ride four-post、ride random-road 六个 preparation 文件、对应 cases/compiler section 和现有 family contract 测试；不得修改 `simulation/preparation.py`、`tests/simulation/test_preparation.py`、`preparation/vehicle_dynamic.py`、`results/vehicle.py`、`results/decoder.py` 或整车 service。
- 子任务 03 唯一负责：`preparation/vehicle_dynamic.py`、`results/vehicle.py`、`results/decoder.py`（仅整车分派接线到 `results.vehicle.decode_vehicle_result()` 以及旧 `vehicle_dynamics` 导入清理，统一 `decode_result()` 定义保持唯一）、`vehicle/service.py`（如需新增）、`cases/vehicle_dynamic.py`、`simulation/compiler.py` 中 `VehicleDynamicCompiler` section 及 vehicle dynamic compiler/cases/results/service 测试；提供第七个 registry key 所需的 preparation 实现和 `adapt_legacy_prepared_request`，由子任务 01 的中央 registry/lifecycle 统一加载；不删除旧文件。
- 子任务 03 另负责 `simulation/runner.py` 的单点解码上下文迁移：改读 `vehicle_dynamic_prepared`，不得再由 runner 消费旧 `prepared`；仅此接线按 01→03 顺序共享，无并行写入。
- 子任务 04 唯一负责：顶层 `__init__.py`、`api.py`、CLI、Adams、scripts、docs、architecture tests、历史 `DynamicResultBundle` 专项测试、历史读取边界 `schema/loader.py` 的最小兼容修复、`tasks/04-delete-legacy/legacy_reference_scan.py` 门禁维护和删除旧文件；不得修改前述 family preparation/compiler/results 文件或 `tests/simulation/test_preparation.py`。
- 子任务 05 只运行验证并更新 `EPIC.md`、`SUBTASKS.csv`、父级 `PROGRESS.md` 和子任务真源，不与前四项并行写代码。

`simulation/compiler.py` 是按符号划分的顺序共享文件：子任务 02 只修改 `AxleDynamicCompiler`、`KcQuasiStaticCompiler`、`VehicleKcCompiler`、`HandlingCompiler`、`RideFourPostCompiler`、`RideRandomRoadCompiler`；子任务 03 只修改 `VehicleDynamicCompiler`。两个子任务不得并行，且不得改动对方 section。

family 测试由拥有对应 family 的子任务修改；中央 preparation 协议测试只归子任务 01；子任务 04 只新增独立历史兼容测试和 architecture 门禁，不接管既有 family 测试文件。
不得并行修改共享注册表、`simulation/__init__.py`、顶层 `__init__.py` 或同一测试文件；按依赖顺序串行集成。
阶段验证策略：子任务 01 完成时六个非整车 family 与 `preparation/vehicle_dynamic.py` 尚不存在，01 的注册表只登记延迟 import 条目，其门禁以替身验证协议与生命周期；`tests/cases`、`tests/results` 等依赖真实 family 的领域回归由 02/03 的门禁负责，终局 04/05 门禁必须在真实 family 存在时运行，不得以替身或跳过用例收口。
## 删除扫描与职责门禁

删除前后统一调用 `tasks/04-delete-legacy/legacy_reference_scan.py`；helper 扫描 `packages`、`docs`、`.github`、根目录 `scripts`、`README.md`、`pyproject.toml` 和 `justfile`，覆盖 Python、文档、脚本、CI 和常见配置后缀，排除 `.codex-tasks`、`.git`、`__pycache__`、build/dist、缓存和 node/site 生成目录以及目标文件自身，并检查完整的旧模块 import/path 形式。删除前必须先完成调用方迁移并保留目标文件；删除后必须断言目标文件不存在。

删除前扫描及门禁：

```bash
uv run --package suspension-multibody python .codex-tasks/20260920-unified-preparation-cutover/tasks/04-delete-legacy/legacy_reference_scan.py --pre-delete
```

删除后扫描：

```bash
uv run --package suspension-multibody python .codex-tasks/20260920-unified-preparation-cutover/tasks/04-delete-legacy/legacy_reference_scan.py --post-delete
```

历史 `DynamicResultBundle` 兼容必须由独立测试文件 `packages/suspension_multibody/tests/schema/test_dynamic_result_compat.py` 验证，不以旧模块字符串扫描代替：

```bash
uv run --package suspension-multibody pytest packages/suspension_multibody/tests/schema/test_dynamic_result_compat.py packages/suspension_multibody/tests/io/test_artifacts_unified.py -q
```

- 终局不仅要求子任务 DONE，还必须在父级 `PROGRESS.md` 的“验证记录”中逐条记录完整命令、退出码和摘要：统一 preparation 测试对 registry key、document bypass、legacy `prepared` adapter、single-call 复用的断言（01 以可注入替身证明协议，02/03 以真实 family 模块证明集成，终局不得只依赖替身）；七个 family 的矩阵对应测试（真实 family 实现，非替身）；旧模块完整职责逐项归属；vehicle result/service 的 metrics、错误证据和 artifact sink 端到端测试；删除前后扫描；`results.decoder` 中恰好唯一的统一 `decode_result` 定义且整车分派经 `results.vehicle.decode_vehicle_result()`；`NativeContractBackend.run` 为唯一 native contract submission point 且 staged/facade 各只提交一次；历史读取回归；以及全量专项 pytest、scoped ruff、compileall、全仓 ty 和 diff check 的实际输出。


## 门禁执行与记录顺序

- 所有门禁命令逐条独立运行，每条命令执行后立即在父级 `PROGRESS.md` 写入一条验证记录（完整命令、真实退出码、摘要和存在的证据文件）；证据文件允许绝对 scratch 路径或工作区相对路径，不强制 `tasks/<id>/raw/`。
- 记录标签统一为 `子任务 NN`（子任务）和 `子任务 04-pre-delete` / `子任务 04-post-delete`（删除前后阶段）；`子任务 04` 的记录集合同时包含这两个阶段标签，两个阶段不得混用。
- `validation_command` 的 `&&` 链按引号感知拆分为原子命令，每个功能/静态原子命令都要有同任务的成功记录；`planning_contract_scan.py` 记录审计命令自身不作为执行证据，但不得据此排除任何功能或静态验证。
- `--final-preclose` 是收口前检查：01-04 DONE、05 前 4 步 DONE 且有真实证据，并核对 `子任务 04-pre-delete`/`子任务 04-post-delete` 记录，收口步骤无需自证；预收口通过后才写入 DONE 状态，最后运行 `--final` 做事后全量状态与证据核对，失败则回退状态。

## Non-Goals

- 不修改 C++ native 内核、ABI、contract version 或物理方程。
- 不把不同 family 合并为一个 compiler 或一个共享物理模型。
- 不删除历史 `DynamicResultBundle` 读取兼容。
- 不改变公开结果的通道名、顺序、单位、时间语义和失败证据。
- 不顺手重命名与本迁移无关的模块。

## 子任务依赖

```text
01 preparation protocol + runner lifecycle
        ↓
02 six non-vehicle family preparation migration
        ↓
03 vehicle_dynamic preparation/result/service split
        ↓
04 caller/export/reference migration → pre-delete gates → old module deletion → post-delete scan
        ↓
05 full validation and Epic close
```

## 删除门槛

只有以下条件全部满足才允许删除 `vehicle_dynamics.py`：

1. 子任务 03 已通过整车 preparation、typed result、metrics/error evidence 和 service 测试；`results.decoder.decode_result()` 是全仓唯一统一解码入口，`results.vehicle` 只提供 family adapter，且 `results.decoder` 的整车分派只调用 `results.vehicle.decode_vehicle_result()`、不再导入旧 `vehicle_dynamics`。
2. 子任务 04 的删除前门禁已通过：vehicle/cases/results/Adams/CLI/architecture、历史兼容测试、scoped ruff、compileall、全仓 `ty check .`、`git diff --check` 和 `legacy_reference_scan.py --pre-delete`。
3. 交付目录无旧模块导入或路径引用；扫描覆盖 `packages`、`docs`、`.github`、根目录 `scripts`、`README.md`、`pyproject.toml`、`justfile` 和常见配置/脚本/文档后缀。
4. `PREPARATION_MATRIX.md` 的旧模块完整职责清单均已有新归属测试：`prepare_vehicle_run`/`PreparedVehicleRun` 及 preparation helpers 归属 `preparation.vehicle_dynamic`，`VehicleDynamicsResult`/`_vehicle_axle_result`/结果映射归属 `results.vehicle`，`run_vehicle_dynamics` 归属 `vehicle.service`。
5. `SimulationRequest → prepare_request() → family compiler → run_request(compiled) → NativeContractBackend` 的整车路径通过；便捷 `run_request(SimulationRequest)` 也只执行一次 preparation 和一次 native submission。
6. 已编译 document 请求不会被重复 preparation，历史 `DynamicResultBundle` 读取边界仍通过独立回归。
7. 删除后再次执行 `legacy_reference_scan.py --post-delete`、import/compile 回归和架构静态扫描，无失效引用或重复统一入口。

## 终局验收

除逐项子任务完成外，必须独立执行：

- `uv run --package suspension-multibody pytest packages/suspension_multibody/tests -q`
- `uv run --package suspension-multibody pytest packages/suspension_multibody/tests/architecture packages/suspension_multibody/tests/cli packages/suspension_multibody/tests/simulation -q`
- `uv run --package suspension-multibody pytest packages/suspension_multibody/tests/io/test_artifacts_unified.py packages/suspension_multibody/tests/adams/test_time_domain_axle.py packages/suspension_multibody/tests/adams/test_time_domain_vehicle_kc.py -q`
- `uv run --package suspension-multibody pytest packages/suspension_multibody/tests/schema/test_dynamic_result_compat.py packages/suspension_multibody/tests/io/test_artifacts_unified.py -q`
- `uv run --all-packages ruff check packages/suspension_multibody/src packages/suspension_multibody/tests packages/suspension_multibody/scripts`
- `uv run --all-packages python -m compileall -q packages/suspension_multibody/src packages/suspension_multibody/tests packages/suspension_multibody/scripts`
- `uv run --all-packages ty check .`
- `git diff --check`
- `uv run --package suspension-multibody python .codex-tasks/20260920-unified-preparation-cutover/tasks/04-delete-legacy/legacy_reference_scan.py --post-delete`
- `uv run --package suspension-multibody python .codex-tasks/20260920-unified-preparation-cutover/tasks/04-delete-legacy/architecture_contract_scan.py`

## 恢复信息

- **任务**：统一多体 preparation 入口并删除整车旧模块
- **形态**：epic
- **进度**：5/5 子任务完成
- **当前**：01/02/03/04/05 全部 DONE；05 预收口首轮 exit1（证据文件字段把「退出码副本」说明并入路径）已保留失败记录并收敛为单一路径，r2 `--final-preclose` exit0，状态写入后 `--final` 独立执行 exit0。
- **文件**：`.codex-tasks/20260920-unified-preparation-cutover/`
- **下一步**：无；Epic 已收口，无 git 提交。

## 子任务03阶段接缝补充

- 为保持03门禁中包导入和结果类身份一致，03可把旧 `vehicle_dynamics.py` 改为仅重导出新 preparation/results/service 符号的临时薄壳，不保留第二份类型或运行实现。04仍独占顶层调用方最终迁移和旧文件删除；删除前门禁不变。
- `PreparedVehicleRun` 保留所有物理字段，并持有来源 model/case 对象引用以兑现 legacy adapter 的 identity 校验；这些引用不参与相等性/repr，也不写入contract。
- results/vehicle 既有 monkeypatch 目标随归属迁移，测试必须直接断言新 preparation/results/service 无旧模块导入。
