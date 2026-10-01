# PROGRESS：p4-03 悬架 arb_mount 语义端口与配对插接

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p4-03`

## Session Start

- **Date**: 2026-10-01
- **Task name**: p4-03-arb-ports
- **Task dir**: `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p4-03-arb-ports/`
- **Spec**: 见 `SPEC.md`
- **Plan**: 见 `TODO.csv`（5 步，全部 `DONE`）
- **Environment**: Python 3.12 / uv / pytest
- **Status**: **DONE**

## Context Recovery Block

- **Current milestone**: #5 — 终局与验收（全 5 步完成）
- **Last completed**: 四份证据落 `raw/`，判据命令 + ruff/ty 实跑通过
- **Current artifact**: `raw/`（ports_declared / pairing_cases / body_aliases_disposition / run_log + 可复跑探针）
- **Key context**: 前置 p4-02 已交付防倾杆模板的 4 个端口声明与悬架硬编码体名的删除；
  本行把**悬架一侧**的对接面（`arb_mount_L/R`）变成模板声明，并删掉别名表。

## 交付物

| 文件 | 性质 |
|---|---|
| `templates/builtin.py` | 新增 `_PORTS`（`arb_mount_L/R` 声明），挂到 `DOUBLE_WISHBONE.ports` |
| `subsystems/suspension.py` | 新增 `declared_ports(context)`，返回模板声明的端口 |
| `subsystems/si_assembly.py` | 新增 `_declared_ports(...)`，并把声明端口并入悬架贡献；`Any` 导入 |
| `subsystems/geometry.py` | **删除** `_BODY_ALIASES` 整表与其两处使用 |
| `tests/subsystems/test_arb_mount_ports.py` | **新增**，6 用例 |

## 四条判据的落点

| 判据 | 证据文件 | 结果 |
|---|---|---|
| (a) 悬架声明含 `arb_mount_L/R` 且产物含这两个名字 | `raw/ports_declared.md` | 产物实测 `axle/arb_mount_L` (owner `axle/lower_arm_L`) 等 |
| (b) 配对段决定插接位置（双叉臂/麦弗逊各一断言） | `raw/pairing_cases.md` | 双叉臂 → `lower_arm_L`；麦弗逊 → `strut_L`；左右不乱 |
| (c) `_BODY_ALIASES` 收口并登记 | `raw/body_aliases_disposition.md` | **整表删除**，494 passed 无一条依赖它 |
| 缺失配对行为有定义 | `raw/pairing_cases.md` 用例 3/4 | 无候选点名拒绝；owner 不在本装配则跳过 |

## 两处刻意的判断（记录下来，便于复核）

1. **`_ports_for_bodies` 的合成端口保留不删**，声明的端口与之**并列**。
   理由：删除会让既有按 `role="body"` 绑定的消费者失去端口，收益为零；
   本行的目标是「让语义化端口存在并到达产物」，不是「消灭 body 端口」。
2. **`_BODY_ALIASES` 整表删除**（而非只删 `wheel→upright`）：其余别名
   （`uca`/`lca`/`tie` 等）是同一类**按名字猜身份**的规则，而它们的身份现在由声明回答。
   删除后 494 条测试全过，证明无人依赖——这是实测，不是推断。

## 未做（明确记账）

- **未把 `anti_roll_bar` 接进 `si_assembly.py` 的角色派发链**。本行 TODO 第 1 行的范围是
  「悬架声明语义化端口与端口合成段改造」；让防倾杆子系统在装配里真的被构造（即给它一个
  `SubsystemContribution` 派发分支）属收尾范围（p4-04/p4-05）。本行交付的是
  **端口侧对接面已就位**：`arb_mount_L/R` 在产物里，防倾杆的 4 个端口也能与之配对。
- 麦弗逊的 `arb_mount` 声明建在**测试**里：本行写范围只允许动 `builtin.py` 的端口段，
  新增整个模板超范围；最小声明足以证明「换拓扑即换落点」。
- **未跑** `just gate-numeric` 的 `case_parity_check.py` 与 `kc_perf_gate.py`：
  本行不改求解路径，归 p4-05 与 Epic 收尾。未重录任何基线。

---

## Final Summary

p4-03 **DONE**。`arb_mount_L` / `arb_mount_R` 成为悬架模板**声明的**语义端口
（双叉臂落在下臂），经 `si_assembly.py::_declared_ports` 到达装配产物
（实测 `axle/arb_mount_L`，owner `axle/lower_arm_L`，各带左右标签）。
配对走既有 `match_requirements`：左需求只落左端口；无候选时点名拒绝。
同一角色在麦弗逊上落在减振筒外筒，证明落点由声明决定而非模块写死。
`geometry.py` 的别名表整表删除（含 `wheel→upright`），494 条测试未有一条依赖它。
