# PROGRESS：p4-02 anti_roll_bar 独立子系统

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p4-02`

## Session Start

- **Date**: 2026-10-01
- **Task name**: p4-02-arb-subsystem
- **Task dir**: `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p4-02-arb-subsystem/`
- **Spec**: 见 `SPEC.md`
- **Plan**: 见 `TODO.csv`（6 步，全部 `DONE`）
- **Environment**: Python 3.12 / uv / pytest
- **Status**: **DONE**

## Context Recovery Block

- **Current milestone**: #6 — 终局与验收（全 6 步完成）
- **Last completed**: 五份证据落 `raw/`，三条判据命令 + ruff/ty 实跑通过
- **Current artifact**: `raw/`（role_sync_manifest / arb_topology / arb_ports_declared /
  upright_grep / role_assertion_update / run_log + 可复跑探针）

## 交付物

| 文件 | 性质 |
|---|---|
| `src/suspension_multibody/templates/roles.py` | 新增 `anti_roll_bar` RoleSpec；`_check_roles` 的 expected 集合改七元 |
| `src/suspension_multibody/templates/builtin.py` | 新增 `ANTI_ROLL_BAR` 模板（4 部件 + 4 端口）、`ANTI_ROLL_BAR_NAME`、`__all__`、`BUILTINS` |
| `src/suspension_multibody/subsystems/anti_roll_bar.py` | **新增**子系统模块 |
| `src/suspension_multibody/subsystems/suspension.py` | 删 `upright_L`/`upright_R` 硬编码，改读模板声明 |
| F12 的 8 处同步点 | 角色名集合逐项加入新角色 |
| `tests/subsystems/test_anti_roll_bar_subsystem.py` | **新增**，6 用例 |
| `tests/templates/test_template_model.py` | 角色断言六→七并改名 |

## 五条判据的落点

| `EPIC.md:263` 判据 | 证据文件 | 结果 |
|---|---|---|
| (a) 角色与模板按 F12 全清单同步 | `raw/role_sync_manifest.md` | 13 处逐项落地；import 期无 `RoleSpecError` |
| (b) 扭杆+吊杆选型理由与 native 关系 | `raw/arb_topology.md` | 选**弹性连杆力元**；native 扭杆与 `RotationalTorqueElement` 各自说明为何不用 |
| (c) 4 个端口暴露 | `raw/arb_ports_declared.md` | 四名逐字声明；缺一即注册失败 |
| (d) `upright_L`/`upright_R` 删除 | `raw/upright_grep.md` | grep 退出 0（零命中）；端体改读模板声明 |
| (e) 角色断言同步 + 理由 | `raw/role_assertion_update.md` | 六→七并改名，理由逐条登记 |

## 两处刻意的判断（记录下来，便于复核）

1. **`DEFAULT_AXLE_SUBSYSTEMS` / `DEFAULT_VEHICLE_SUBSYSTEMS` 不加新角色**：
   两个集合描述「某装配实际建了什么」，加进去会让没声明防倾杆的整车装配**谎报能力**
   （`EPIC.md:232` 的同源错误）；且 `done-when` 的同步点清单里没有这两项。
   理由写在 `raw/role_sync_manifest.md`。
2. **不用 `RotationalTorqueElement` 表示扭杆**：它是被驱动的执行器（幅值来自 demand，
   无势能，方向由速率定），而扭杆是弹性构件。用它会把物理说错。理由写在
   `raw/arb_topology.md` 与模块 docstring。

## 过程中的一处自身缺陷（诚实记账）

端口首版把 `droplink_mount` 的 `owner` 误写为 `torsion_bar`（从 `chassis_mount` 复制），
导致 `droplink_bodies()` 返回两个扭杆半体。新增的
`test_the_subsystem_declares_a_bar_half_and_a_droplink_per_side` 立即失败并指出：
改为 `droplink` 后通过。该断言确在起作用。

## 未做（明确记账）

- **未改 `si_assembly.py` 的端口合成段**：把模板声明接到装配产物端口集合是 **p4-03** 的
  写范围（`SUBTASKS.csv` 的 `p4-03` 明写）。本行只保证**防倾杆一侧的四个端口已声明**。
- **未跑** `tests/architecture/` 整目录、`tests/adams/`、`just gate-numeric` 的
  `case_parity_check.py` 与 `kc_perf_gate.py`。数值门三项归 p4-05 与 Epic 收尾。
- 未重录任何基线；`tests/data/` 无改动；无新增 skip/xfail。

---

## Final Summary

p4-02 **DONE**。`anti_roll_bar` 成为独立角色与独立子系统：模板 `anti_roll_bar_simplified`
声明 `torsion_bar_L/R` 与 `droplink_L/R` 四个部件，暴露
`chassis_mount_L/R`、`droplink_mount_L/R` 四个端口（各自带左右标签，缺一无法注册）。
悬架模块里 `upright_L`/`upright_R` 两个硬编码体名删除，改从**悬架模板自己的连接声明**
读出轮端体——换模板即换答案，声明里没有轮端体则点名拒绝，不再按名字猜。
`grep` 在 `suspension.py` 零命中；内建双叉臂的杆元件（端体、作用点、刚度）**与改造前逐位相同**。
