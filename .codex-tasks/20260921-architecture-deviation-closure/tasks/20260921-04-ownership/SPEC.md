# 子任务 04：拆解 mb_vehicle 并落地 assembly/force/element/tire 的职责归属

## 目标

消除 `mb_vehicle` 这一职责混合体，把它的四类职责分别落到新模块，并清空 `mb_vehicle`：

1. 车辆注册 11 个函数、`build_model`、element reader → `mb_assembly`：装配只读中性输入，不得反向依赖 `abi` 或求解器。
2. `mb_suspension` + steering/drive_brake/aerodynamic 的力元 → `mb_element`：标量与方向导数实现一起迁移，不得拆散。
3. `external_force_vector` / `external_force_directional` 两条总线、外力重力与广义力汇总 → `mb_force`：本构留 `mb_element`/`mb_tire`，标量与对偶总线保持次序。
4. `cpp/src/vehicle/kernel_directional.cpp` 的轮胎语义 → `mb_tire`：与轮胎接触/内部状态共同维护，总线不得残留轮胎力律。
5. `mb_model` 中性数据化、`build_model` 末尾 `audit_constraint_system` 归 joint 审计或 ABI 装配后校验；登记05需要迁入 `mb_assembly` 的凝聚计算与 body ID 映射要求，本任务不实施凝聚算法迁移。

现有事实：`build_model` 在 `packages/suspension_kernel/cpp/src/abi/kernel_model_build.cpp:23`；车辆注册在 `cpp/src/vehicle/kernel_registration.cpp`；力装配与对偶实现在 `cpp/src/vehicle/{force_assembly,kernel_directional}.cpp`；`MODULES.md` 记录 `mb_vehicle` 依赖 `mb_model`、`mb_tire`、`mb_constraint`、`mb_energy`、`mb_input`。

## 非目标

- 不迁移或新建焊接体凝聚算法及其身份映射实现（统一由05负责），本任务只交接装配接口与来源清单。
- 不改轮胎物理、Adams 等价范围、积分器与稳态算法。
- 不强制把轮胎各实现合并成一个静态库：`mb_tire` 作为职责边界保留内部子目标。
- 不动 `abi` 导出符号与签名，不新增导出，不改 ABI 版本号。

## 约束

- 每一步都要"构建 → 架构门 → 数值门"三过再进下一步；禁止先积累全任务 diff 再验证。
- 数值门命令与容差引用父级 `VALIDATION.md`（01 冻结）；纯结构阶段要求动态数组逐位一致，不得重录基线。
- `CMakeLists.txt`、`MODULES.md`、`layering_baseline.json` 与测试硬编码模块名清单在本任务内同步；这三个文件与 03 共享，本任务必须在 03 完成并冻结后才开工。
- 禁止新增 `mb_assembly`→`abi`、`mb_assembly`→`mb_solve_static` 反向边；`audit_constraint_system` 的原有失败时机与诊断信息必须保留。
- 本任务结束前 `mb_vehicle` 不得以转发壳形式存在。

## 范围与文件归属

- 可写：`packages/suspension_kernel/cpp/include/{mb_assembly,mb_force,mb_element,mb_model,mb_tire,mb_joint}/**`、`packages/suspension_kernel/cpp/src/{vehicle,suspension,tire,model,abi}/**`、`packages/suspension_kernel/CMakeLists.txt`、`packages/suspension_kernel/MODULES.md`、`packages/suspension_kernel/layering_baseline.json`、两侧 `tests/**` 中引用这些模块名或模块职责的断言。
- 只读：`packages/suspension_kernel/cpp/src/{base,linalg,constraint,integrator,static,input}/**`（03 已定稿）、父 `EPIC.md`、`VALIDATION.md`、Python 侧 `packages/suspension_multibody/src/**`（只读用于核对凝聚映射）。
- 不写：Python 生产代码与 `api.py`；父 `EPIC.md`、`SUBTASKS.csv`、`PROGRESS.md`。

## 依赖

- 前置：01（`VALIDATION.md`）、02（门禁）、03（基础层模块与共享构建文件已冻结，必须串行）。
- 后续：05依赖本任务明确的 mb_assembly 归属与现有输出接口，负责凝聚迁移和身份映射实现；06依赖本任务后 mb_element/mb_tire 本构边界稳定。

## 验收标准

1. 11 个车辆注册函数、`build_model`、element reader 全部归 `mb_assembly`，装配只读中性输入，无 `mb_assembly`→`abi` 与 `mb_assembly`→`mb_solve_static` 反向边。
2. 力元标量与方向导数实现同属 `mb_element`；两条外力总线、外力重力与广义力汇总属 `mb_force`，标量与对偶总线的通道次序不变。
3. `kernel_directional.cpp` 的轮胎语义归 `mb_tire`，总线中不残留轮胎力律。
4. `mb_model` 为纯数据边界；质量矩阵装配归 `mb_assembly`、线性分解归 `mb_linear`；`audit_constraint_system` 归 joint 审计或 ABI 装配后校验，失败时机与诊断不变。
5. 凝聚计算来源及原 body ID 到凝聚体 ID 映射的需求已登记给05；本任务不得提前实现05的算法迁移或宣称映射测试已通过。
6. `mb_vehicle`、`mb_suspension` 不再存在（无转发壳）；`check_module_layering.py --strict` 通过，边集与 baseline 一致。
7. 每步构建退出码为 0，DLL 已同步，动态数组逐位一致、K/C parity 与 family parity 通过，ABI 七符号与版本号不变。

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

数值漂移立即停止当前步并检查编译选项、LTO 与运算顺序；回退范围只限本任务改动。
