# PROGRESS：p4-01 冻结现状事实与判据（阶段四）

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p4-01`

## 状态

`DONE`（2026-10-01）。五条判据全部实测，证据落 `raw/`。本行是**纯只读冻结行**：未写任何生产代码，
未改 `packages/**`、`EPIC.md`、`SUBTASKS.csv`。

## 交付物

| 判据 | 文件 | 内容 |
|---|---|---|
| (1) ARB 两套物理 | `raw/arb_two_physics.md`（104 行） | A 侧 Python 力元的力律/作用点/阻尼/装配分派，B 侧内核扭杆的力律/ABI 编组/结构，两者的逐条对照表与交叉拒绝证据 |
| (2) 角色真源全清单 | `raw/role_table_sync.md`（168 行） | 新增 `anti_roll_bar` 必须同步的 **14 个行位置**（13 独立 + 1 测试断言）逐条 `file:line` + 原文 + 关联断言 + 同名非同步点清单 |
| (3) 端口缺失事实 | `raw/arb_mount_absence.md`（62 行） | 三个语义端口在源码零命中的 grep 证据 + 今天端口的装配期合成路径 + `DOUBLE_WISHBONE` 的 `ports`/`needs` 实际值 |
| (4) wheel 文件链与 VerticalTireElement | `raw/wheel_file_chain.md`（148 行） | wheel 模板来源、`_FILE_ROLE_TEMPLATES` 现值、`AssemblyRequest` 模板字段、`VerticalTireElement` 全清单、`git ls-files` 无实体文件证明 |
| (5) ARB 冻结产物 | `raw/arb_baseline.md`（55 行） | 两个 sha256.json 的 `anti_roll_output` 取值 + `dynamic_hash_baseline`/`kc_baseline` 的 ARB 命中数 |
| — | `raw/run_log.md`（41 行） | 22 条命令 + 退出码 + 关键输出；锚点漂移记录 |

## 实测要点（供 p4-02 / p4-03 / p4-04 使用）

- **两套 ARB 物理是不同物理，p4-02 的 SPEC 必须点名关系**：
  - **A 侧（Python）** `AntiRollBarElement`（`modeling/primitives/elements.py:593-621`）：`difference=(right[2]-left[2])-ref`，
    `scalar = stiffness * difference`，两端**等大反向线力** `[0,0,±scalar]`，**无阻尼、有作用点**。
    装配分派 `subsystems/element_build.py:66-67`，实现 `:124-133`，**只读 `left_link_point`/`right_link_point`**；
    `schema/elements.py:224-234` 声明了 **6 个硬点字段**但装配期只读 2 个。声明处
    `subsystems/suspension.py:696-709` **硬编码 `body_a="upright_L"` / `body_b="upright_R"`**（这就是 G5 要消除的）。
  - **B 侧（内核 native 扭杆）** `cpp/src/element/anti_roll.cpp:43-63`：`tau = -stiffness*angle - damping*rate`，
    **纯力偶、无作用点**（注释 `:52-53`），**含阻尼**。ABI 编组 `mb_input/types.hpp:578-584`、
    `build_model.cpp:333-352`、`element_reader.cpp:333-350`；结构 `mb_model/types.hpp:142-147`。
  - **交叉拒绝已存在于代码里**：`preparation/vehicle_dynamic.py:936-939` 明写两者「is not equivalent」。
  - **对照用例只覆盖 B 侧**：`tests/axle_dynamics/test_api.py:315`（断言 `:377-382`）。
  - **`packages/suspension_kernel/tests/` 无任何 anti-roll 用例**（grep 零命中）——内核侧无测试网。
- **新增 `anti_roll_bar` 角色的同步点实测 14 个行位置**（清单与原文见 `role_table_sync.md`）：
  `roles.py:70` + `roles.py:158`（import 期硬断言）、`authoring/documents.py:60`、
  `subsystems/types.py:70/77/84`、`subsystems/capabilities.py:39`、`subsystems/composition.py:63/313`、
  `connections/policy.py:48`、三份契约 schema 的 `functional_role` enum
  （`template.schema.json:11`、`subsystem.schema.json:8`、`assembly.schema.json:22`）、
  `tests/templates/test_template_model.py:57`。全仓一行式六角色字面量 grep 只有 10 行命中，逐位置核对**无遗漏**。
  **关联断言**：`test_template_model.py:69-71 test_only_brake_and_drive_carry_a_torque_channel`
  断言 `torque == {"brake", "drive"}`——新角色 `has_torque_channel=False` 则不受影响，但仍属关联面。
  **同名但非同步点（不要误改）**：`suspension.py:64` / `wheel.py:54` 是子系统自身 `role` 常量；
  `authoring/solver.py:698/779` 是按角色分流。
- **端口今天不存在，是运行期合成的**（确认 F13）：`arb_mount` / `droplink_mount` / `chassis_mount`
  在源码**零命中**（只有路线图文档与一处测试示例字符串）。合成路径：`subsystems/si_assembly.py:87-117
  _ports_for_bodies`（每个 body 一个 `role="body"` port，upright 额外一个 `role="wheel_centre"`）与
  `:120-150 _wheel_centre_needs`。`templates/builtin.py:397-414` 的 `DOUBLE_WISHBONE` **无 `ports=`/`needs=`**，
  取 `templates/model.py:311/313` 的默认 `()`。→ **p4-03 要先造出语义化端口，不是把已有端口接上。**
- **F14 与 F15 已重过期（按实测记录，p4-04 只需独立复验）**：
  - `authoring/solver.py:726 _FILE_ROLE_TEMPLATES` 实测 `("steering", "chassis", "wheel")`——**已含 wheel**；
    `AssemblyRequest.wheel_template` **已存在**（`subsystems/types.py:166`，映射 `:275-280`）；
    `_ROLE_TEMPLATE_FIELD` 实测 `:275`（F14 写 `:249`）。这属阶段一 04b 的交付。
  - `git ls-files '*.subsystem.json' '*.sub.json' '*.tpl.json' '*.asy.json'` **输出为空**：仓库确无实体装配文件。
  - `subsystems/assembler.py` 的 `isinstance` 过滤**已删除**（grep 零命中），替代为
    `assembler.py:95 _AXLE_ROLES_IN_A_VEHICLE`；反向断言在 `tests/subsystems/test_wheel_lifecycle.py:225-226`。
  - `rig_link.py` 的 `_is_replaced_tire`/`_reown_tires` **源码零命中（已删除）**，属阶段一 05 的交付。
  - **VerticalTireElement 现存引用点**：定义 `modeling/primitives/elements.py:567`；构造
    `subsystems/element_build.py:31/68-69/136-138`；native 拒绝 `preparation/vehicle_dynamic.py:55/940-944`；
    按名分流 `compilation/model_view.py:179`、`studies/bridge.py:125/334`、`cases/kc_quasi_static/contract.py:354`；
    试验台 `subsystems/rig_link.py:297`。
- **ARB 冻结产物与 F17 一致**：`tests/data/axle_dynamics_baseline/sha256.json` 的 `anti_roll_output`（13 行）与
  `tests/data/vehicle_dynamics_baseline/sha256.json`（8 行）**值均为空字节的 sha256**
  `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855`（即基线记录「无 ARB」）；
  `dynamic_hash_baseline.json` grep `anti_roll|arb` 零命中；`kc_baseline/` 零命中（退出码 1）。
  **仍不得据此放松**：任何产物变化按 D5 逐项登记。

## 锚点漂移（只记录，未改父文件）

| Epic 原锚点 | 实测 |
|---|---|
| F11 `suspension.py:682-695` | `:696-709` |
| F11 `element_build.py:211` | 入口 `:216` |
| F13 `si_assembly.py:71 _ports_for_bodies` | `:87` |
| F13 `si_assembly.py:104 _wheel_centre_needs` | `:120` |
| F14 `_FILE_ROLE_TEMPLATES = ("steering","chassis")` | `:726 = ("steering","chassis","wheel")`（**已过期**） |
| F14 `_ROLE_TEMPLATE_FIELD:249` | `:275` |
| F15 `assembler.py:231 isinstance` | **不存在**（已删除，替代为 `:95 _AXLE_ROLES_IN_A_VEHICLE`） |
| F15 `vehicle_dynamic.py:900` 拒绝 | `:940` |

## 未做（本行边界）

- **未跑内核 C++ 单测**：`packages/suspension_kernel/tests/` 实测只有 `test_binding.py`、
  `test_registry_consistency.py`、`test_tire_mass.py`，**无 anti-roll 用例**可跑。
- 未跑全量测试、未跑数值门（SPEC 只要求 `tests/modeling`/`tests/templates` 的 `-q`；实跑 **106 passed**、退出码 0）。

## 环境观察

本行执行期间工作区已有**另一并发写入者**在改 `packages/suspension_kernel/cpp/**`（p2-02 的内核力矩元方向，
含 `mb_input/types.hpp` 等）。本行全部写入只落在本目录 `raw/`；实测命令与结论只依赖源码文本读取，
不依赖内核运行期，**证据未受污染**。本行未修改任何 `packages/**` 文件。
