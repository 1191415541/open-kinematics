# suspension-multibody

声明式通用多体建模与 native 求解。悬架、整车和非汽车机构使用相同的
模板、子系统、端口、编译器和结果模型；包不依赖 `suspension_kinematics`。

## Run

```powershell
uv run --project packages/suspension_multibody suspension-multibody validate `
  --assembly assembly.json --case case.json
uv run --project packages/suspension_multibody suspension-multibody run `
  --assembly assembly.json --case case.json --out results
```

Assembly 通过配置激活表选择理想关节或柔性连接。Study/Case 指定采样、求解器、
输入和运动边界。悬架 K/C 台架锁定 wheel 的相对 spin；整车动态释放同一
spin 坐标，wheel/suspension 定义不变。轮胎接触 frame 绑定非自转承载体。

Python 负责声明、解析、契约编译和结果查询，C++ 内核负责物理求解和力元状态。
输出包含 manifest、原生通道和 NPZ 数据；通过 ResultEnvelope 按实体 ID 读取。
`scripts/kc_perf_gate.py` 核验 native `k-100` 与 `c-66` 的冻结性能预算。

## Python 模块结构

所有业务都走同一条声明式路径：

```text
Template -> Subsystem -> Assembly -> Study/Case
        -> DocumentLoader -> ResolvedModel -> Compiler
        -> Native Backend -> ResultEnvelope
```

`authoring/` 保存模板、子系统、总成、端口绑定和 v1 离线迁移；`modeling/` 提供刚体、运动副、力元和空间代数；`rigs/` 只声明台架实体、边界和测量，不创建第二个 wheel 或 tire。Wheel 子系统拥有 wheel body 和 tire element，Brake、Drive 与 Suspension、Steering 是同级子系统。`studies/` 和 `cases/` 只声明研究边界与输入，`compilation/` 始终把同一个解析后模型提交给 `simulation/`，结果统一从 `results/ResultEnvelope` 按稳定实体 ID 查询。

文件输入与 Python 对象输入使用相同的 `api.validate()` / `api.simulate()`；旧 v1 文件只能由 `authoring.migration` 离线转换，已退役的 preparation、subsystems、旧 solver、结果分派和 vehicle service 不属于生产运行路径。

`modeling/` 的依赖边界由架构门检查。运行快速门：

```powershell
just check-fast
```

### 力元与扩展示例

力元声明位于 `modeling/primitives/` 和 Template 数据中；Python 不重复计算 native 力律。`ResultEnvelope` 统一暴露 element state、wrench、诊断和能量通道，固定支座反力由内核记录。车辆静力轮荷和侧倾中心属于 `vehicle/` 中的派生量。

`docs/composable_extension_examples.md` 提供普通机构、普通 Rig、属性变化、纯内存建模及同一车轮锁止/滚动五个可执行示例。`scripts/check_composable_release.py` 在独立解释器中逐块运行它们。

## Native 整车动力学

`simulate` 的整车动力学路径使用与整轴相同的 native DAE 内核，求解车身、前悬架、后悬架和四个轮端的刚体状态。模型支持悬架理想关节、弹簧、阻尼器、限位块、轮胎接触、路面高度、转向输入，以及直接施加到轮端的驱动力矩和制动力矩；动力系统和制动系统在通用整车模型中按外部输入信号简化。使用 Adams 源显式模型时，源动力总成刚体、驱动轴、三脚架和差速器输出体均保留，传动轴通过非完整 `CONVEL` 速度约束连接；驱动/制动仍按项目规格作为轮端外部力矩输入，不伪装成 Adams 内部传动或液压系统。

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
