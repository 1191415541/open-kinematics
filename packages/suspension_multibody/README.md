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
for linear 6x6 compliant mounts.  K supports wheel-center and contact-point
drives; C supports six-component load paths and symmetric/opposite/single-side
load modes.  Results contain a manifest plus independent states,
component-load, bushing and diagnostic Parquet/CSV tables.

The fixed local performance gates are available through
`suspension_multibody.analysis.benchmarks` and cover 100 K states and 6600 C states.

## Native 整车动力学

`run_vehicle_dynamics` 使用与整轴相同的 native DAE 内核，求解车身、前悬架、后悬架和四个轮端的刚体状态。模型支持悬架理想关节、弹簧、阻尼器、限位块、轮胎接触、路面高度、转向输入，以及直接施加到轮端的驱动力矩和制动力矩；动力系统和制动系统在通用整车模型中按外部输入信号简化。使用 Adams 源显式模型时，源动力总成刚体、驱动轴、三脚架和差速器输出体均保留，传动轴通过非完整 `CONVEL` 速度约束连接；驱动/制动仍按项目规格作为轮端外部力矩输入，不伪装成 Adams 内部传动或液压系统。

Adams/Car 数据可通过 `load_adams_full_vehicle_input` 导入，并由 `build_adams_native_vehicle_model` 构造 native 模型。导入层记录源文件哈希和单位声明，并把几何、质量、惯量、轮胎垂向参数及力曲线统一到 `mm/kg/N/s`；弹簧、压缩限位、回弹限位和六轴衬套曲线均可通过版本化整车 ABI 传入求解器，存在源衬套时整车路径选择带衬套的 C 装配模式。当前 PAC2002 路径实现带一阶松弛的纯滑移、选定 RBX/RBY/RVY 联合滑移项、基于 QBZ/QCZ/QDZ/QEZ/QHZ/SSZ 的法向回正力矩，并保留 Adams 源 `PHX/PHY/PVX/PVY` 零滑移偏置；尚未实现完整联合滑移参数集、完整外倾角/压力/载荷缩放和 Adams 的完整接触力律。源模型中的驱动/制动 SFORCE 会写入配对清单；当前 native 可选择逐轮回放已求得的驱动/制动转矩，但仍未实现 Adams 内部控制与液压状态，因此真实 Adams 整车数值对标继续由门禁报告为 `BLOCKED`，直到源力律、力元映射和完整 PAC2002 轮胎完成等价实现。

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
