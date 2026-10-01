# p4-04 第一步：读阶段一 04 的实际交付，落盘实测结论

`EPIC.md:271` 与 `EPIC.md:75` 要求开工第一步先读阶段一 04 的 `PROGRESS.md` 与 `raw/`。
本文件是**本行实测**的结论（阶段一 04 的自报只作对照目标）。

## 1. 阶段一 04 已交付的动作（本行**不得重做**）

来源：`.codex-tasks/20260929-assembly-layer-rework/tasks/20260929-04-wheel-lifecycle/PROGRESS.md`
的 Final Summary 段与其 `raw/wheel_file_unification.md`。

| 动作 | 阶段一 04 的处置 | 本行实测复核 |
|---|---|---|
| 单轴与整车读入同一份 wheel 子系统**文件** | 该行声明「在 04 写范围内不可实现」，**重读为**「`subsystems/wheel.py` 是轮端唯一生产者」，跨文件读 wheel 模板**另立 04b** | 见 `stage1_04_reverification.md`：实测成立 |
| `assembler.py` 的类型过滤移除 | 删除 `isinstance(element, VerticalTireElement)` 过滤与其 import，改用**角色**（`_AXLE_ROLES_IN_A_VEHICLE`）表达 | `grep -c isinstance assembler.py` = **0** |
| 单轴侧凝结 | `RigSpec.supplies_wheels=True` 时把车轮体刚性凝结进轮毂 | 未复跑（属 04 自身判据，本行不重做） |
| K/C 受力激活断言 | 04 交付 | 未复跑 |

**结论：阶段一 04 已要求的动作本行一律不重做。**

## 2. 明确剩余项（本行开工时实测，逐条给出 `file:line`）

| # | 项 | 实测 | 来源 |
|---|---|---|---|
| 1 | `AssemblyRequest` 的 wheel 模板载体字段 | **已存在**：`subsystems/types.py:174 wheel_template: object \| None = None` | 本文实测 |
| 2 | `_ROLE_TEMPLATE_FIELD` 的 wheel 表项 | **已存在**：`subsystems/types.py:287 "wheel": "wheel_template"` | 本文实测 |
| 3 | `authoring/solver.py::_FILE_ROLE_TEMPLATES` 是否放开 wheel | **已放开**：`authoring/solver.py:726 _FILE_ROLE_TEMPLATES = ("steering", "chassis", "wheel")` | 本文实测 |
| 4 | 装配路径的 `VerticalTireElement` 过滤 | **已消失**：`grep -rn VerticalTireElement …/subsystems/assembler.py` 只命中两处**注释**（`:36` / `:94`，都在解释「它被移除了」），无代码引用 | 本文实测 |
| 5 | `assembler.py` 的 `isinstance` 过滤 | **不存在**：`grep -c isinstance …/assembler.py` = **0** | 本文实测 |

**剩余项清单：空。** 阶段一 04（含 04b）已把这四项全部兑现，故本行按 SPEC 收缩为
**独立复验**（`EPIC.md:271(b)` 允许清单为空）。

## 3. 判据调整

**无需调整**，因此**未提请**父 Epic 修订 `SUBTASKS.csv`（本行也从未直接改父表）。

## 4. F14 命名约定（遵守）

仓库里**没有** `*.subsystem.json`；实际约定是 `{name}.tpl.json` / `{name}.sub.json` /
`{name}.asy.json`（`authoring/security.py:168/242/292`）。本行未按不存在的前例施工。
