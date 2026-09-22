# 子任务 08：轮胎质量归属轮胎与内核惯量耦合

## 目标

按用户裁决 **D2=乙**（tire 成为独立惯量来源，求解器显式耦合）把轮胎质量从轮端刚体迁到轮胎元素：

1. **契约文档** tire entry 新增质量/惯量字段：`packages/suspension_contracts/src/suspension_contracts/contracts/multibody_model.schema.json` 的 `$defs.tire`（`:152-167`，当前 `required: [name, model, body]`，`parameters` 为自由 object，`additionalProperties: false`），并在 `suspension_contracts/multibody.py` 的校验路径与契约测试中固化。
2. **C++ 内核** `Tire` 结构体（`packages/suspension_kernel/cpp/include/mb_model/types.hpp:101-146`）加质量/惯量，内核解析文档字段（`cpp/src/cases/contract_model.cpp:935-1130` 的 tire 读取与 `:1489`/`:1575` 的 `fill`），并在**求解器中显式耦合**轮胎惯量（进入残余 / 雅可比路径，不再由轮端 body 承担）。
3. **质量守恒断言先行**：同一模型在「质量挂轮端 body」与「质量挂 tire」两种归属下，整车/整轴总质量与世界质心一致。这是「质量只是换了归属」的判据，必须先于任何数值差异评估。
4. ABI 七符号与版本常量（15/30/1/1）保持；若必须给 `AxleInput`/`VehicleInput` 追加数组字段，须 append-only 并**单独裁决留痕**（EPIC 冻结约束）。
5. 两包构建与隔离 wheel 复验。

## 非目标

- 不改轮胎力律的物理定义（`fiala`/`pac2002`/`native_brush` 的力与滑移计算不动），只改质量归属与耦合路径。
- 不改 ABI 导出面（七符号不增不减，`suspension_kernel_free` 不新增）。
- **不动 Python 作者层装配**：`preparation/**`、`subsystems/**`、`templates/**`、`outputs/**` 一律不改。作者层把 `WheelSpec.mass`（`schema/vehicle.py:60`）写进 tire entry 的发射点（`cases/vehicle_dynamic.py:327` 的 `_tire_entry`、`cases/axle_dynamic.py:342` 的 `tires` 数组，数据源在 `preparation/vehicle_dynamic.py` 的轮胎 spec）不在本任务写范围内——`preparation/` 属 04–06、`cases/` 属 02，须由主代理在它们之后单独排期或并入 09（09 依赖本任务并负责 kc 发轮胎）。本任务以**文档级 fixture** 证明「字段被契约接受、被内核解析、质量守恒成立」。
- 不新增依赖，不改积分器、步长控制、牛顿/线搜索的算法本身（轮胎惯量耦合除外）。

## 约束

- 写范围仅 **C++ 内核**（`packages/suspension_kernel/cpp/**`）+ **契约文档**（`packages/suspension_contracts/src/**` 与 `tests/**`）+ **native 镜像**（`build_axle_native.py` 重生成的 `packages/suspension_multibody/src/suspension_multibody/native/suspension_kernel.dll`，不手改）+ 测试（`packages/suspension_kernel/tests/**`、新增 `packages/suspension_multibody/tests/tire_mass/**`）。
- **不得改** `packages/suspension_multibody/src/suspension_multibody/preparation/`、`subsystems/`、`templates/`、`outputs/`（与 03–07 并行）；不得改 `cases/**`（02 写范围）。若确需改动这些路径，必须停止并上报，由主代理串行排期。
- C++ 新增代码必须落进现有 23 模块且不新增反向边；`check_module_layering.py --strict --final` 保持绿。
- 结构体字段 **append-only**：`Tire` 的新字段加在末尾，既有字段偏移不变；`AxleInput`/`VehicleInput` 的既有字段不得移动。若必须给它们追加数组字段，`kAxleKernelAbiVersion`/`kVehicleKernelAbiVersion`（`cpp/include/mb_config/version.hpp`）的变化属于 ABI 冻结解除，须单独裁决并登记。
- 轮胎惯量耦合属**物理改变**：动态字节门的任何变化必须逐项登记「基线文件 + 导致重录的步骤 + 重录前后的值 + 判定为物理改变或数值路径改变」（D4）；禁止先改基线让门变绿。
- 固定编译器、浮点选项、线程、后端与计算顺序（`native_build.json` 冻结值）。

## 范围与文件归属

- 可写：
  - `packages/suspension_contracts/src/suspension_contracts/contracts/multibody_model.schema.json`
  - `packages/suspension_contracts/src/suspension_contracts/multibody.py`（校验路径，仅在需要时）
  - `packages/suspension_contracts/tests/**`（轮胎质量字段的存在/类型错/未知字段三类覆盖）
  - `packages/suspension_kernel/cpp/include/mb_model/types.hpp`、`cpp/include/mb_model/**`（访问器声明）
  - `packages/suspension_kernel/cpp/src/cases/contract_model.cpp`
  - `packages/suspension_kernel/cpp/src/model/**`、`cpp/src/element/**`、`cpp/src/force/**`、`cpp/src/solve_dynamic/**`（仅在耦合必需处）
  - `packages/suspension_multibody/src/suspension_multibody/native/suspension_kernel.dll`（构建产物镜像）
  - 新增 `packages/suspension_multibody/tests/tire_mass/**`
  - `packages/suspension_kernel/tests/**`（内核侧质量守恒与绑定）
  - `packages/suspension_multibody/tests/architecture/**`（仅当 ABI 门确需扩展时，且须显式登记）
- 只读：`packages/suspension_multibody/src/suspension_multibody/preparation/**`、`cases/**`、`subsystems/**`、`templates/**`、`outputs/**`、`packages/suspension_kernel/layering_baseline.json`、父 `EPIC.md`/`SUBTASKS.csv`、01 的 `raw/baseline_commands.md` 与 `raw/tire_mass_inventory.md`。
- 不写：`preparation/**`、`cases/**`、`subsystems/**`、`templates/**`、`outputs/**`；父级 `EPIC.md`/`SUBTASKS.csv`/`PROGRESS.md` 归主代理。

## 依赖

- 前置：01（`raw/tire_mass_inventory.md` 给出质量现状路径，`raw/baseline_values.md` 给出各基线当前值，`raw/baseline_commands.md` 给出 ABI/构建/数值门的实测退出码）。
- 可与 02–07 并行：写范围（C++ 内核 + 契约文档 + native 镜像）与它们的 Python 写范围不相交。
- 后续：09 依赖本任务（准静态发轮胎与质量归属）；11 的终局验收需要 ABI 七符号与版本门证据。

## 验收标准

1. 契约文档 tire entry 含质量/惯量字段，schema 与 Python 校验一致；契约测试覆盖字段存在、类型错误、未知字段三种情况（`additionalProperties: false` 下新增字段必须先改 schema）。
2. 内核 `Tire` 含质量/惯量且文档字段被解析；新增字段 append-only（有测试或字段偏移断言证明既有字段偏移不变）。
3. **质量守恒断言通过**：同一模型在「质量挂轮端 body」与「质量挂 tire」两种归属下，整车/整轴总质量与世界质心在容差内一致；该断言先于任何数值差异评估执行。
4. 求解器显式耦合轮胎惯量：有测试证明轮胎惯量参与残余 / 雅可比（关闭耦合后结果变化，而耦合开启时质量守恒断言仍成立）。
5. ABI 七符号与版本常量 15/30/1/1 不变；架构 ABI 门（`tests/architecture/test_kernel_abi_version_single_source.py`、`test_core_abi.py`）与 kernel 测试保持绿。
6. `check_module_layering.py --strict --final` 退出 0；两包 `uv build` 退出 0；隔离 wheel 内 import/CLI/七符号/native 真实运行/artifact 往返通过。
7. `dynamic_hash_sentinel`、`kc_parity_check`、`case_parity_check` 三门在重录后重新通过，且每次重录在 PROGRESS 中登记前后值与判定；无「先改基线让门变绿」的痕迹。

## 验证协议

```bash
uv run python packages/suspension_multibody/scripts/build_axle_native.py
uv run --package suspension-contracts pytest packages/suspension_contracts/tests -q
uv run --package suspension-kernel pytest packages/suspension_kernel/tests -q
uv run --package suspension-multibody pytest packages/suspension_multibody/tests/tire_mass -q
uv run --package suspension-multibody pytest packages/suspension_multibody/tests/architecture -q
uv run python packages/suspension_kernel/scripts/check_module_layering.py --strict --final
uv run python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check
uv run python packages/suspension_multibody/scripts/kc_parity_check.py --check
uv run python packages/suspension_multibody/scripts/case_parity_check.py
uv build --package suspension-kernel
uv build --package suspension-multibody
uv run --package suspension-multibody pytest packages/suspension_multibody/tests -q
```

质量守恒断言（第 3 条）必须在任何数值差异评估与动态字节门的任何重录**之前**跑；数值差异必须登记为「物理改变或数值路径改变」，不得默默吸收。
