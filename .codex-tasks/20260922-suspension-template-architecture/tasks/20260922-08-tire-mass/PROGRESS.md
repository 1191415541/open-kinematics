- 任务：轮胎质量归属轮胎与内核惯量耦合
- 形态：single-full（Epic 子任务）
- 进度：0/9 步骤 TODO，尚未实施
- 当前：未开工。前置 01（冻结基线与现状清单）未完成；未冻结判据前不得开始。
- 文件：`.codex-tasks/20260922-suspension-template-architecture/tasks/20260922-08-tire-mass/`
- 验证：未运行。

## 恢复信息

**本轮交付为规划，未写任何生产代码，`raw/` 为空（不预填未执行的证据）。**

开工前必须核验：

- 01 已完成并产出 `raw/baseline_commands.md`（ABI/构建/数值门的实测退出码）、`raw/tire_mass_inventory.md`（质量现状路径）与 `raw/baseline_values.md`（各基线当前值）。
- 用户裁决 **D2=乙** 有效：tire 是独立惯量来源，求解器显式耦合；这不是把质量从 body 复制一份。
- 父 `EPIC.md` 的冻结约束有效：ABI 七符号与版本（15/30/1/1）不变；`check_module_layering.py --strict --final` 保持绿；D4 允许**受控**重录基线（逐项登记前后值与判定）。

## 本任务的现状事实（制定计划时实测，实施时复核）

- 内核 `Tire` 当前无质量字段：`packages/suspension_kernel/cpp/include/mb_model/types.hpp:101-146`（字段为 body/frame_body/drive_torque_*/center/radius/k/c/mu_*/brush_k_*/relaxation_*/model_kind/state_slot_width/contact_mass/maxwell_enabled）。
- 质量现挂轮端 body：`schema/vehicle.py:60` 的 `WheelSpec.mass`（与 `:61` 的 `axial_inertia`）→ `preparation/assembly/vehicle.py:653,722` 合并进轮端刚体；总质量/质心聚合在 `preparation/assembly/vehicle.py:374-397`。
- 生效的 native 边界是**文档契约**而非 struct ABI：`kernel/__init__.py:186-205` 的 `run_contract` 传两份 JSON payload（`argtypes` 全是指针）。因此 D2 主要落在「契约文档字段 + 内核解析 + 求解器耦合」。
- 契约文档 tire entry：`packages/suspension_contracts/src/suspension_contracts/contracts/multibody_model.schema.json:152-167` 的 `$defs.tire`，`required: [name, model, body]`，`parameters` 是自由 object，且 `additionalProperties: false`。
- 内核 tire 解析：`cpp/src/cases/contract_model.cpp:935-1130`；`fill()` 在 `:1489`（`axle.tire_radius`）与 `:1575`（`input.tire_model_kind`）。
- ABI 版本常量单一来源：`packages/suspension_kernel/cpp/include/mb_config/version.hpp`（`kAxleKernelAbiVersion = 15`、`kVehicleKernelAbiVersion = 30`、`kCoreKernelAbiVersion = 1`）；架构门为 `tests/architecture/test_kernel_abi_version_single_source.py` 与 `test_core_abi.py`。
- 求解器惯量/质量装配路径：`cpp/src/assembly/build_model.cpp` 与 `cpp/src/abi/kernel_core.cpp` 从 `body_mass`/`body_inertia_body_3x3` 建体；残余/雅可比在 `cpp/src/solve_dynamic/kernel_integrator_residual.cpp`、`kernel_integrator_newton.cpp`。

## 写范围与并行约束（硬条件）

- 本任务只写 C++ 内核（`cpp/**`）、契约文档（`suspension_contracts/src/**` 与 `tests/**`）、native 镜像（构建产物）与测试（`suspension_kernel/tests/**`、新增 `tests/tire_mass/**`）。
- **不得写** `preparation/`、`subsystems/`、`templates/`、`outputs/`（03–07 写范围）与 `cases/**`（02 写范围）。作者层把 `WheelSpec.mass` 写进 tire entry 的发射点由主代理在 02/04–06 之后单独排期或并入 09；本任务以文档级 fixture 证明字段被接受、被解析、质量守恒成立。
- 结构体字段 append-only；若必须给 `AxleInput`/`VehicleInput` 追加数组字段，属于 ABI 冻结解除，须停止并单独裁决留痕。

## 步骤顺序（固定）

1. 现状与 ABI 基线（只读）→ 2. 契约字段 → 3. 内核解析与 `Tire` 字段 → 4. **质量守恒断言先行** → 5. 求解器显式耦合 → 6. 数值差异评估与 D4 登记 → 7. ABI 门 → 8. 分层门与两包构建 → 9. 全量回归与隔离 wheel。

第 4 步是后续所有数值判断的前提：总质量与世界质心不变，才说明质量只是换了归属；第 6 步的任何重录都必须有前后值与判定。

## 下一步

等 01 完成后，从 `TODO.csv` 第 1 行开始。父 `SUBTASKS.csv` 第 08 行的状态由主代理回填。
