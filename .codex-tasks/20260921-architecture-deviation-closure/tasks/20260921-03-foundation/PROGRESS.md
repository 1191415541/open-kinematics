- 任务：拆分基础层并迁移 linear/joint 与静动态求解模块名称
- 形态：single-full（Epic 子任务）
- 进度：7/7 步骤 DONE
- 当前：子任务完成。旧模块 mb_base/mb_linalg/mb_constraint/mb_integrator/mb_static 全部消失（无转发壳）；新模块 mb_config/mb_numeric/mb_dual/mb_linear/mb_joint/mb_input/mb_solve_dynamic/mb_solve_static 落地；终局缺口从 35 条降到 12 条，全部归子任务 04（mb_vehicle/mb_suspension 拆解）。
- 文件：`.codex-tasks/20260921-architecture-deviation-closure/tasks/20260921-03-foundation/`
- 验证：构建 0 错误；`--strict` 退出 0；动态哈希 26/26 逐位一致；K/C parity、8 family parity 通过；kernel 15 passed、架构测试 86 passed；ABI 七符号 15/30/1/1 ctypes 实测；ruff/ty/git diff --check 通过。

## 恢复信息

子任务 03 已完成。04 可开工（依赖已满足）。剩余终局阻断（04 的消除对象）：`mb_vehicle`、`mb_suspension` 两个旧模块及其路径；目标模块缺失 `mb_element`、`mb_assembly`、`mb_force`；mutual 边 `mb_solve_dynamic<->mb_tire_state`、`mb_solve_static<->mb_vehicle`；环 `mb_solve_dynamic<->mb_solve_static<->mb_tire*<->mb_vehicle`。

## 执行记录

步骤 2-4（mb_base 三拆，原子迁移一次验证）：
- 11 个头文件按归属表分配：vector/util/monotone_cubic → mb_numeric；dual/dual_geometry → mb_dual；prelude/version/env/diagnostics/constants → mb_config。
- `functions.hpp` 聚合声明按符号表拆为三个模块头（26 config + 41 numeric + 38 dual 声明块）。
- `kernel_base.cpp` 拆为 `config/kernel_config.cpp`（26 个 env 开关函数）+ `numeric/kernel_numeric.cpp`（57 个数值函数）；angles.cpp → numeric；curves/dual_geometry/kernel_dual_algebra → dual。
- 39 个 TU 的 `mb_base/functions.hpp` 聚合 include 按符号使用逐一替换为所需模块头。
- 90 条新边全部审核登记：85 条为旧 `X->mb_base` 边的机械拆分，5 条为拆分内部边（numeric→config、dual→numeric、dual→config），方向与步骤 1 冻结表一致，无环。

步骤 5（linalg/constraint 重命名 + 职责修正）：
- 目录迁移 + 27 处 include 改写，无函数体改动。
- 关键职责修正：`constraint_rows` 从 `model/kernel_model_accessors.cpp` 迁到 `joint/registry.cpp`（关节职责离开装配层），直接消除了 `mb_joint <-> mb_model` 环和一条 mutual 边（3 → 2）。

步骤 6（求解模块重命名 + 共享输入采样迁移）：
- `interpolate_input`（两个重载）+ `next_prescribed_input_breakpoint` 迁到新模块 `mb_input`（`src/input/kernel_input.cpp` + `include/mb_input/functions.hpp`）；static/output/abi/events/integrator/step 六处调用方改 include mb_input。
- `NewtonLinearizationCache`、`pose_candidate*`、`state_from_unknown`、`unknown_from_state` 留在 `mb_solve_dynamic` 新 TU `kernel_integrator_state.cpp`。
- `mb_solve_static` 的 TU 不再 include `mb_solve_dynamic/functions.hpp`：静态层对动态层的实现依赖消除（SPEC 验收标准 2 达成）。
- 56 条新边审核登记：全部为重命名推导 + 新 mb_input 依赖。

步骤 7（四份清单同步）：
- CMakeLists：20 个模块目标，`mb_config/mb_numeric/mb_dual/mb_linear/mb_joint/mb_input/mb_solve_dynamic/mb_solve_static` 全部注册，IPO/foreach/link 列表同步。
- MODULES.md 模块表完全重写（旧名零残留）。
- `layering_baseline.json`：legacy_modules 收缩到 `mb_suspension/mb_vehicle`；kept_modules 增加全部新模块；reviewed_new_edges 台账 90+29+56 条全部带人工审核结论。
- `binding/build.py` SOURCE_RELATIVES 与 3 个测试硬编码同步（`test_kernel_abi_version_single_source.py` 的 VERSION_HEADER 路径与 TU 名单、`test_binding.py` 的层序断言、`test_module_layering_gate.py` 的 source-only 边断言）。

## 证据（raw/，全部真实退出码）

step1_mb_base_ownership.md（归属表）；step2_4_{layering_strict,dynamic_hash,kc_parity,case_parity,kernel_tests,arch_tests}.log；step5_{...}.log；step6_{...}.log；step7_{build,layering_strict,dynamic_hash,kc_parity,case_parity,kernel_tests,arch_tests,final_gate,ruff,ty}.log。

每步均执行完整门：构建 + `--strict` + 动态哈希 + K/C parity + family parity + kernel 测试 + 架构测试。动态 acceptance 底层退出码 1 为子任务 01 冻结的既有状态，未掩盖。

## 边界遵守

- 未修改任何 Python 生产代码（`api.py`、包 `__init__.py` 零触碰）。
- ABI 七符号与版本（15/30/1/1）ctypes 实测不变，`suspension_kernel_free` 未新增。
- 未改容差、未重录数值基线；动态数组 26/26 逐位一致。
- 遇到的数值语义风险点（无）：本任务纯结构迁移，函数体逐字搬运（脚本化的 def 级切分，括号平衡校验）。
