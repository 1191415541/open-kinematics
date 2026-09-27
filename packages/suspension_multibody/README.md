# suspension-multibody

Independent quasi-static suspension K&C and load solver for a symmetric front
double-wishbone suspension with rack steering.  The package has no dependency
on `suspension_kinematics`; shared geometry enters through
`suspension_contracts`.

## Run

```powershell
uv run --project packages/suspension_multibody suspension-multibody validate `
  --model model.yaml --case case.yaml
uv run --project packages/suspension_multibody suspension-multibody run `
  --model model.yaml --case case.yaml --out results
```

Each case selects exactly one mode: `K` for ideal suspension joints or `C`
for linear 6x6 compliant mounts.  K supports wheel-center drives; a contact-point drive is refused rather than answered
with a wheel-center result, because the two are different questions and the
difference is a force-application offset. C supports explicit six-component loads
and symmetric/opposite/single-side load modes.  Results contain a manifest plus independent states,
component-load, bushing and diagnostic Parquet/CSV tables.

The solve is native: K and C states come from the C++ kernel through the contract
boundary (`suspension_multibody.kernel`), and what remains in Python is the
authoring model, the result schema and the reporting. The fixed local performance
gate is `scripts/kc_perf_gate.py`, which measures the native `k-100` and `c-66`
workloads against a recorded budget; the retired Python benchmarks covered 100 K
states and 6600 C states, and that 6600-state C workload was a deliberately
nonphysical proxy with no native analogue.

## Python 模块结构（组合架构之后）

模块按依赖方向分层，每层的边界由门禁检查而不是由约定维持：

```text
src/suspension_multibody/
  modeling/        低层：稳定实体标识、端口值对象、ModelFragment、
                   Assembly/SimulationAssembly、单位边界。
                   modeling/primitives/ 是空间代数与关节/刚体声明的唯一实现。
  templates/       模板：声明构件、连接、属性槽与输出，或命名一个 builder；
                   两条作者路径都产出同一个 ModelFragment。
  connections/     连接：端口匹配、歧义拒绝、自适应安装几何、全局 D3 规则。
  rigs/            试验台：rig.py 声明驱动/研究/输出，bench.py 产出实体，
                   compose.py 做“接口收缩到总成能力”。
  subsystems/      六类子系统模板与 SI 总成（si_assembly.py、composition.py）。
  compilation/     编译：plan.py 说这次运行是什么，model_view.py 说模型是什么，
                   compile.py 说两者蕴含的文档；family 名只作为选 emitter 的注册键。
  studies/         研究方式与输入适配（准静态网格 vs 时间历史）。
  schema/ results/ outputs/ report/ io/
                   契约、结果解码、衍生输出、报告与检查点。
  simulation/      runner、request 与原生后端；api.py 只做薄编排。
  cases/           各族契约文档的作者层。
  preparation/     域输入适配（assembly/、axle_dynamic.py、vehicle_dynamic.py 等）。
  vehicle/         整车级服务与派生量（service.py、static_loads.py、roll_centers.py）。
  elements/        A1 保留：仅剩 evaluate_generalized_forces（Python 侧力元求值）。
```

`modeling/` 不得反向依赖 `templates`、`subsystems`、`rigs`、`connections`、
`preparation`、`simulation`、`kernel` 或 `report`；这条边界由
`tests/architecture/test_import_boundaries.py` 在独立子进程里逐入口检查。

已删除：`model/`（其内容先迁至作者层的 `assembly` 子包，该子包随组合层落地整体退役，见下）、
`metrics/`（迁至 `report/metrics/`）、
`core/`（spatial 代数与关节/刚体数据迁至 `modeling/primitives/`；`rank.py` 与
`reactions.py` 无生产调用者，其物理断言转为
`tests/axle_dynamics/test_solver_invariants.py` 的 native 契约断言）、顶层
`pac2002_scope.py`（迁至 `schema/pac2002_scope.py` 与 `kernel/capabilities.py`），
以及 `analysis/`（报告类模块迁至 `report/`、`preparation/signals.py`、
`simulation/replay.py`；`compute_static_wheel_loads` 迁至
`vehicle/static_loads.py`，`compute_vehicle_roll_centers` 迁至
`vehicle/roll_centers.py`）。02 留在 `preparation/geometry.py` 与
`preparation` 下两个转发壳（`geometry.py` 与旧 `assembly/types.py`）也已删除，调用方直接导入
`modeling/primitives/`。

作者层的 `assembly` 子包整体退役：轴与整车一律经组合层构建，
入口是 `subsystems/entry.py`（`compose_axle` / `compose_vehicle`），
机制代码留在同层的 `subsystems/vehicle_parts.py`，
元素构造在 `subsystems/element_build.py`，硬点与镜像在 `subsystems/geometry.py`。

### 仍保留 elements/ 的理由

它现在只剩一个函数 `evaluate_generalized_forces`，被 `api.py` 用来重算组件载荷与
衬套结果——而同一批力元 native 已经算过。当前 import 点、阻断原因与解除条件如下；
`legacy_surface_gate.py --final` 的对应发现已登记，
`scripts/check_composable_release.py` 会核对登记表与实测发现一致，新增或消失都会
失败。

| 保留项 | 现役 import 点 | 阻断原因 | 解除条件 |
|---|---|---|---|
| `elements/`（A1） | `api.py`（`evaluate_generalized_forces`） | 三条，第一条最重：① KC 契约只发出 bushing（`contract.py:190-192`，且仅 C 模式）与 tire（`_tire_entries`），**弹簧、减振器、横向稳定杆从不发出**，native 对它们没有任何事实——实测同一根轴上 K 模式 `api.py` 报 7 个力元而 native 只收到 2 个、C 模式报 17 个只收到 12 个，两种模式都缺同样 5 个；② 固定体端那一行被 `cpp/src/element/assembly_primitives.cpp:16` 的早退留在 NaN；③ 力矩参考点两侧不一致（native 对受力体原点，Python 对世界原点） | KC 契约扩出弹簧/减振器/稳定杆声明（K 模式为规定运动，声明力元不得进入残差）+ 固定端开始记录 + 力矩参考点口径对齐 + 通道默认状态（契约版本）确认后，`api.py` 改走 `results/element_wrench.py` 解码，本包随之删除 |

力元件的**声明**已经不在这里了：`LinearSpringElement` 等八个类连同
`ForceEvaluation`、`ElementError` 迁至 `modeling/primitives/elements.py`，与关节和
刚体声明并列，所以作者层（`preparation/`）不再 import 任何退役包。留在本包的只有
Python 侧求值这一件事。

原 `analysis/`（A2）已不再是保留项。它的两个构造都是整车级派生量而不是内核求解：
静力轮荷是四个未知量对三个平衡方程的最小范数解，native 静力求解器只分解方阵，
装不下这个欠定问题；侧倾中心是前视几何作图。两者都只读装配结果，因此迁入
`vehicle/`（唯一允许依赖 `preparation/` 的整车层），算法逐字未改，也不需要原先
登记的「扩 ABI 导出面」。

### 扩展示例

`docs/composable_extension_examples.md` 给出「新增子系统模板 / 新增试验台 /
硬点更新」三个示例，每个都可执行：`scripts/check_composable_release.py` 会抽出文
档里的 `python runnable` 代码块，在独立解释器里逐块运行，跑不过即发布检查失败。

## Native 整车动力学

`run_vehicle_dynamics` 使用与整轴相同的 native DAE 内核，求解车身、前悬架、后悬架和四个轮端的刚体状态。模型支持悬架理想关节、弹簧、阻尼器、限位块、轮胎接触、路面高度、转向输入，以及直接施加到轮端的驱动力矩和制动力矩；动力系统和制动系统在通用整车模型中按外部输入信号简化。使用 Adams 源显式模型时，源动力总成刚体、驱动轴、三脚架和差速器输出体均保留，传动轴通过非完整 `CONVEL` 速度约束连接；驱动/制动仍按项目规格作为轮端外部力矩输入，不伪装成 Adams 内部传动或液压系统。

Adams/Car 数据可通过 `load_adams_full_vehicle_input` 导入，并由 `build_adams_native_vehicle_model` 构造 native 模型。导入层记录源文件哈希和单位声明，并把几何、质量、惯量、轮胎垂向参数及力曲线统一到 `mm/kg/N/s`；弹簧、压缩限位、回弹限位和六轴衬套曲线均可通过版本化整车 ABI 传入求解器，存在源衬套时整车路径选择带衬套的 C 装配模式。当前 PAC2002 路径实现纯滑移、标准 RBX/RBY/RVY 联合滑移、dfz/dpi/外倾/压力/载荷缩放、QV2/QFC/VXLOW 垂向修正、QBZ/QCZ/QDZ/QEZ/QHZ/SSZ 回正力矩、Mx/My/陀螺力矩、USE_MODE 0 与 1-4/11-14/23-25 语义、模式 25 的转滑松弛集与 Q* 驻车因子、deflection/bottoming 曲线、输入有效性区间钳位和负 USE_MODE/TYRESIDE 镜像，并保留 Adams 源 `PHX/PHY/PVX/PVY` 零滑移偏置；PAC-MC、带动力学、非点接触模型、纯轴 advanced-transient 模式 21/22、二阶转滑尾系数与 Maxwell 非滚动垂向单元仍按 fail-closed 处理。这份范围不是手写的：内核通过 `suspension_kernel_capabilities` 声明它能算什么、必须拒绝什么，`schema/pac2002_scope` 读它来拒绝越界轮胎。源模型中的驱动/制动 SFORCE 会写入配对清单；当前 native 可选择逐轮回放已求得的驱动/制动转矩，但仍未实现 Adams 内部控制与液压状态，因此真实 Adams 整车数值对标继续由门禁报告为 `BLOCKED`，直到源力律、力元映射和完整 PAC2002 轮胎完成等价实现。

若需要复现 Adams 已运行的动力输入，可用 `direct_wheel_torque_signals_from_adams_result` 从 `.res` 的 `differential.output_torque_left_rear/right_rear` 和 `brake_torques` 四个通道提取逐轮 `TimeSignal`，再传给 `build_adams_vehicle_case` 的 `wheel_drive_torque`、`wheel_brake_torque` 参数。源结果中的驱动/制动转矩按 Adams 工程单位读取，进入 native ABI 前由模型单位缩放；这属于可追溯的轮端输入回放，不表示已实现 Adams 内部控制律、制动液压或完整力律等价。

## Adams validation

The strict K gate discovers Adams/Car 2024.1 through the environment, `PATH`, or
the Windows uninstall registry and verifies the license with a real unattended
`acar` product start. It creates temporary suspension and steering subsystem
copies with kinematic joints, generates the fixed 3x3 wheel-travel/rack grid,
and reruns every state with Adams Solver `simulate/kinematics`. The independent
`suspension_multibody` result is generated from the same normalized hardpoint input;
neither runner can read the other result.

```powershell
uv run --project packages/suspension_multibody suspension-multibody validate-adams `
  --profile adams-car-2024.1 --strict-k --require-installed `
  --evidence-dir artifacts/adams/strict-k
```

The strict C gate writes and executes a native Adams Solver model from the
same canonical hardpoints and element set: eight diagonal 6x6 inboard
bushings, ideal outboard/tie-rod joints, a neutral locked rack, a left
wheel-center six-axis wrench, and no gravity, contact, spring, damper, stop,
or stock-template compliance objects. Each physical bushing is emitted as a
reversed pair of half-rate native `BUSHING` elements so Adams' moving-J-frame
asymmetry does not alter the shared 6x6 constitutive law. It compares all 66 load states across
left/right wheel-center translation, rotation vector, toe, and camber.
The native command first solves zero load, then preconditions to the negative
endpoint before recording the 11-state negative-to-positive response sweep;
this keeps the largest moment paths on a continuous quasi-static equilibrium
branch.

    uv run --project packages/suspension_multibody suspension-multibody validate-adams `
      --profile adams-car-2024.1 --strict-c --require-installed `
      --evidence-dir artifacts/adams/strict-c

It is intentionally not a comparison against the stock TR template's complete
ride system, whose springs, dampers, stops, and additional bushings are outside
the current MBD element set. The legacy `--full` command also remains available
for regression evidence, but its built-in C fields are left/right symmetry
residuals rather than full compliance magnitudes and therefore do not
constitute strict C/load acceptance.

`--reference` and `--runner` override the built-in baseline and batch runner.
An external runner receives the request JSON and output directory as its final
two arguments (also exposed as `SUSPENSION_MULTIBODY_ADAMS_REQUEST` and
`SUSPENSION_MULTIBODY_ADAMS_OUTPUT`) and writes `adams_results.json` or CSV. Missing
groups or fields fail the gate instead of producing a profile-only pass.

## Geometry contract

`suspension_multibody.adapters.front_axle_model_from_contract` consumes Geometry
Contract V1. Mass properties remain multibody-specific input, so the contract
does not silently define compliance, force elements, tires, or solver settings.
