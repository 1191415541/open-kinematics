# 目标模块 DAG 与分层约束

- 日期：2026-10-10。本文件记录 P0 冻结的当前结构与目标结构的依赖方向，作为 P1–P6 的实现边界。
- 三类结构门（秒级）是本文件断言的执行者，不是本文件的替代：
  - `packages/suspension_multibody/tests/architecture/legacy_surface_gate.py --check`（Python 责任/退役面）
  - `packages/suspension_kernel/scripts/check_module_layering.py --strict --final`（native 模块 DAG）
  - `packages/suspension_multibody/scripts/check_composable_release.py --skip-isolation`（迁移清单与文档）
- 修改结构后必须重跑三者；本文件描述的方向必须与它们的判定一致，不一致以脚本为准。

## 1. Python 侧：唯一链与依赖方向

唯一生产链（不得分叉、不得新建第二入口）：

```
authoring / schema / presets
        ↓
modeling/resolved.py            ResolvedModel（不可变 SI 模型图，编码所有权）
        ↓
compilation/resolved.py         plan_from_case → compile_resolved → CompiledSimulation
        ↓
simulation/runner.py            run_compiled
        ↓
kernel/（ctypes） → native      suspension_kernel
        ↓
results/envelope.py             ResultEnvelope（查询）
        ↓
outputs / report / io          纯代数派生、报告组织、持久化
```

- `modeling/` 是低层数据层，**不得**反向导入 authoring/compilation/report 层。这是本仓库最关键的结构性质，由 `legacy_surface_gate.py` 与分层约定共同守护。
- `results/envelope.py` 是查询层：只读解码结果，不发起求解、不做物理。
- `report/` 不得导入 native/kernel/solver，不得调用 solve 或 native 入口，不得运行 preparation/authoring，不得评估力元本构（`legacy_surface_gate.py` 的 `report_native_import` / `report_native_call` / `report_preparation_import` / `report_preparation_call` / `report_constitutive_call`）。
- 不得存在「只做 re-export 的转发行壳」非包模块（`legacy_forwarding_shell`）。

## 2. 新增 `characteristics` 层的位置（目标结构）

设计 §5.2 的目标：特性算法归 `characteristics`，只依赖低层数据与空间代数。

```
characteristics/
├── measurements.py     R1：frame_position / toe / camber 等测量算子
├── centers.py          R2/R3：几何中心、等效作用线、旧 roll_center_legacy_v1
├── motion.py           R4/R5：Twist / Screw / 瞬时轴 / 刚体运动数学
├── response.py         P2/P4a：局部响应算子（具名模式/测量通道）
└── (P4b) descriptor.py 连续动态描述符
```

允许方向：

```
characteristics/  →  modeling/（不可变数据）、results/（只读查询）、contracts
characteristics/  ✗→  authoring/ compilation/ simulation/ report/ io/
```

`io/characteristics.py`（P6）负责表布局与持久化，方向为 `io/ → characteristics/`（读其数据结构），不得让 `characteristics/` 反向依赖 `io/`。

约束（来自设计 §5.2、§10）：

- 特性是**输出请求**，不是 Study / Case family / 第二入口。请求进 `ResolvedSolvePlan`，不改编译与执行路径。
- `characteristics/` 不得调用 native 或重建资源；查询不得触发求解（P6 硬要求「冷读取/查询/导出不产生物理求解」）。
- 指标的生产算法在 `characteristics/` **只有一份**；`results`、`outputs`、`report` 只委托，不保留副本。

## 3. native 侧：模块分层现状（`check_module_layering.py --strict --final`）

`TARGET_MODULES`（11 个，模块化目标必须提供）：

```
mb_numeric  mb_dual  mb_config  mb_linear  mb_joint  mb_input
mb_solve_static  mb_solve_dynamic  mb_element  mb_assembly  mb_force
```

`LEGACY_MODULES`（7 个，只允许在 `MIGRATION_MAP` 可追溯时存在）：

```
mb_base  mb_vehicle  mb_suspension  mb_integrator  mb_static  mb_linalg  mb_constraint
```

迁移映射（代码内定义，baseline 编辑不能凭空发明解释）：

| legacy | 后继 |
|---|---|
| `mb_base` | `mb_numeric`, `mb_dual`, `mb_config` |
| `mb_linalg` | `mb_linear` |
| `mb_constraint` | `mb_joint` |
| `mb_suspension` | `mb_element` |
| `mb_vehicle` | `mb_element`, `mb_force`, `mb_assembly` |
| `mb_integrator` | `mb_solve_dynamic` |
| `mb_static` | `mb_solve_static` |

门禁的五条规则：

1. 模块图必须是 DAG，不得有环（`find_cycles`）。
2. 模块头不得 include 另一模块的 `functions.hpp` 聚合头（`strict` 下必须为空）。
3. 头文件不得 include 自己。
4. **任何不在 `layering_baseline.json` 中记录的边都失败**（header 边与 source 边分别比较）；反向新边报 `reverse_edge`；source 边必须带 `source_edge_evidence`；baseline 之后的边必须在人工审阅的 `reviewed_new_edges` 或 `header_only_edges` 里。
5. legacy 模块只能在 `MIGRATION_MAP` 可追溯时存在；`mutual_edges` 非空即报 `reverse_edge`；未注册模块报 `unregistered_module`。

基线现状：`version: 2`，`header_edges: 119`，`mutual_edges: 0`，`legacy_modules: ["mb_suspension", "mb_vehicle"]`。**允许的依赖方向集合 = baseline 记录的边集**；新增任何跨模块边都必须重录 baseline 并留审阅说明。

## 4. 目标物理归属（设计 §5.2、§10、R7/R8）

```
mb_joint / mb_element / mb_force      解析约束、力元、方向导数
        ↓
mb_linearization（新增责任）            共享物理线性化：力/约束/接触闭合 + 方向导数组装
        ↓
mb_solve_static / mb_solve_dynamic     求解算法：残量尺度、活动集、gauge/pin、
                                       步长/alpha、离散约束行、Newton 重试与缓存
```

- R7：`static_residual` / `static_jacobian` 内被抽出的组装副本归 `mb_linearization`；两函数保留静态求解适配。`load_scale`、活动集策略、残量尺度、gauge/pin、静态行选择仍归 solver。
- R8：`residual` / `fill_analytic_jacobian_columns` 中被新服务替代的组装副本归共享物理原语；`ResidualContext`、积分状态混合、步长/alpha 系数、离散约束行、Newton 重试与缓存留 solver。**无离散系数泄漏到物理响应。**
- 共享提取完成后旧位置必须**委托**，不能长期双算并把其中一份称为校验实现；独立 oracle 只放测试范围。

## 5. P0 阶段的结构断言边界

P0 只做契约与台账，**不改结构**，因此：

- 本文件描述的是**当前已存在**的结构与**目标**方向，不是已完成的迁移。
- `characteristics/` 目录当前**不存在**（已验证）。
- `mb_linearization` 当前**不存在**于 `TARGET_MODULES`，是本设计提出的新责任归属，需要在 P2 实施时同步基线并重录边集说明。
- P0 不新增任何跨模块边，故 `layering_baseline.json` 与 `legacy_surface_gate` 的注册表在 P0 均**不需要改动**。P1 起每次结构变动必须重跑三条门并同步基线。
