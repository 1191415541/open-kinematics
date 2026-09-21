# 子任务 03：拆分基础层并迁移 linear/joint 与静动态求解模块名称

## 目标

把 C++ 基础层按职责拆开，消除混合职责与反向依赖，不能靠重命名掩盖职责混合：

1. `mb_base` → `mb_numeric`（向量、四元数、旋转、普通曲线）+ `mb_dual`（`dual.hpp`、`dual_geometry` 与对偶代数）+ `mb_config`（`env`、版本、运行时诊断配置）。
2. `mb_linalg` → `mb_linear`、`mb_constraint` → `mb_joint`：线性算法与关节残差不得混入装配。
3. `mb_integrator` → `mb_solve_dynamic`、`mb_static` → `mb_solve_static`，并把共享输入采样移到 `mb_input`，去掉静态层对动态实现的依赖。
4. 版本字面量仍单一来源（`version.hpp`），标准库 prelude 不制造反向边，不出现 `numeric`→`dual` 环。

现有事实（01 冻结前即可核对）：源文件在 `packages/suspension_kernel/cpp/src/base/{kernel_base,curves,angles,dual_geometry,kernel_dual_algebra}.cpp`，头文件在 `cpp/include/mb_base/`；`MODULES.md` 记录 `mb_static` 当前依赖 `mb_integrator`，这正是要去掉的接缝。

## 非目标

- 不动 `abi` 导出符号与签名，不新增 `suspension_kernel_free`，不改 ABI 版本号。
- 不做 04 的 `mb_vehicle` 拆解，不迁移 `mb_suspension`/轮胎语义，不处理 Python 侧。
- 不改积分器算法、K/C 稳态算法、轮胎物理、计算顺序；纯结构阶段只要求动态数组逐位一致。
- 不修改 `mb_model`/`mb_contract`/`mb_cases`/`mb_output` 的职责。

## 约束

- 每一步都要"构建 → 架构门 → 数值门"三件事全过再进下一步；禁止先积累全任务 diff 再验证。
- 数值门容差与命令引用父级 `VALIDATION.md`（01 冻结）；纯结构阶段要求动态数组逐位一致，不得改容差或重录基线使失败消失。
- `CMakeLists.txt`、`MODULES.md`、`layering_baseline.json`、测试中硬编码的模块名清单必须在本任务内同步；这三个文件与 04 共享，04 依赖本任务完成后才开工。
- 迁移期旧名映射只在 02 的迁移阶段模式中解释，本任务结束时旧基础模块不得作为转发壳保留。

## 范围与文件归属

- 可写：`packages/suspension_kernel/cpp/include/{mb_numeric,mb_dual,mb_config,mb_linear,mb_joint,mb_input,mb_solve_static,mb_solve_dynamic}/**`、`packages/suspension_kernel/cpp/src/{base,linalg,constraint,integrator,static,input}/**`、`packages/suspension_kernel/CMakeLists.txt`、`packages/suspension_kernel/MODULES.md`、`packages/suspension_kernel/layering_baseline.json`、`packages/suspension_kernel/tests/**` 与 `packages/suspension_multibody/tests/architecture/**` 中引用这些模块名的断言。
- 只读：`packages/suspension_kernel/cpp/src/abi/**`（签名不变）、`packages/suspension_kernel/cpp/src/{model,contract,cases,output}/**`、父 `EPIC.md`、`VALIDATION.md`。
- 不写：Python 生产代码与 `api.py`（06/07 负责）；父 `EPIC.md`、`SUBTASKS.csv`、`PROGRESS.md`。

## 依赖

- 前置：01（`VALIDATION.md` 冻结工具链与数值门）、02（源码级分层门禁与迁移阶段模式）。
- 后续：04 依赖本任务完成（共享 `CMakeLists.txt`、`MODULES.md`、`layering_baseline.json`，必须串行）；05 依赖本任务后的 ABI 与求解模块边界稳定。

## 验收标准

1. `mb_numeric`、`mb_dual`、`mb_config`、`mb_linear`、`mb_joint`、`mb_input`、`mb_solve_static`、`mb_solve_dynamic` 各自拥有明确职责与头文件，`mb_base`/`mb_linalg`/`mb_constraint`/`mb_integrator`/`mb_static` 不再作为模块存在（无转发壳）。
2. 依赖图为 DAG：无 `numeric`→`dual` 环，无 `mb_solve_static`→`mb_solve_dynamic` 实现依赖；共享输入采样位于 `mb_input`。
3. `check_module_layering.py --strict` 在 cpp 与头文件两侧通过，边集与重写后的 baseline 一致。
4. 每一步的构建退出码为 0，DLL 已同步到 `packages/suspension_multibody/src/suspension_multibody/native/`，动态数组逐位一致、K/C parity 与 family parity 通过（命令取 `VALIDATION.md` 原文）。
5. ABI 七符号与版本字面量单一来源门通过，版本号仍为 15/1/30。
6. `CMakeLists.txt` 的目标集合、`MODULES.md` 的模块表、`layering_baseline.json` 的边集与测试中硬编码的模块名清单四者一致。
7. 无任何 Python 生产代码改动。

## 验证协议

按模块子步执行，每步先构建再跑架构门与数值门：

```bash
uv run python packages/suspension_multibody/scripts/build_axle_native.py
uv run python packages/suspension_kernel/scripts/check_module_layering.py --strict
uv run python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check
uv run python packages/suspension_multibody/scripts/kc_parity_check.py --check
uv run python packages/suspension_multibody/scripts/case_parity_check.py
uv run --package suspension-kernel pytest packages/suspension_kernel/tests -q
uv run --package suspension-multibody pytest packages/suspension_multibody/tests/architecture -q
```

每一步的验收都要求"构建成功 + 架构门通过 + 数值门通过"同时成立；某步数值漂移立即停止该步并检查编译选项、LTO 与运算顺序，不得改容差。
