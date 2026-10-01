# run_log：p4-01 实际执行的命令、退出码与输出末尾关键行

仓库根 `/e/杂件/open-kinematics`，HEAD `8d8c5c0`。所有命令经 Git Bash 执行；python/pytest 一律 `uv run --no-sync`。
**只记已执行的结果。**

| # | 命令（摘要） | 退出码 | 输出末尾关键行 |
|---|---|---|---|
| 1 | `git rev-parse --short HEAD` | 0 | `8d8c5c0` |
| 2 | `ls .codex-tasks/.../tasks/p4-01-freeze/` | 0 | `PROGRESS.md SPEC.md TODO.csv raw`（raw 为空） |
| 3 | `grep -rn "anti_roll\|AntiRoll\|anti-roll" packages/suspension_kernel/cpp/` | 0 | 命中 `mb_element/functions.hpp:33 assemble_anti_roll_forces`、`mb_model/types.hpp:142 struct AntiRollBar`、`mb_input/types.hpp:578-584`、`element/anti_roll.cpp:26` |
| 4 | `grep -rln "anti_roll\|anti-roll\|AntiRoll" packages/suspension_kernel/tests/` | 1 | 空（kernel/tests 无 anti-roll 用例） |
| 5 | `find packages/suspension_kernel/tests -type f` | 0 | 仅 `test_binding.py`/`test_registry_consistency.py`/`test_tire_mass.py` 及 pyc |
| 6 | `sed -n '315,382p' tests/axle_dynamics/test_api.py` | 0 | `test_anti_roll_bar_reports_physical_angle_rate_and_torque`：断言 `output[2] == -stiffness*angle - damping*angular_rate` |
| 7 | `grep -rn '"suspension", "steering", "wheel", "chassis", "brake", "drive"' --include=*.py --include=*.json .`（去 .codex-tasks） | 0 | 10 个行命中（3 schema + documents.py:60 + policy.py:48 + capabilities.py:39 + composition.py:313 + types.py:70 + types.py:84 + roles.py:158） |
| 8 | `grep -rn "arb_mount\|droplink_mount\|chassis_mount" .`（去 .codex-tasks） | 0 | 仅 `docs/multibody_architecture_evolution.md:93/195` 与 `tests/authoring/test_assembly_pairings.py:48/50`（示例字符串） |
| 9 | `grep -rn "VerticalTireElement" packages/ docs/`（去 .pyc） | 0 | 定义 `elements.py:567`；构造 `element_build.py:31/68-69/136-138`；`vehicle_dynamic.py:55/940`；`model_view.py:179`；`bridge.py:125/334`；`contract.py:354`；`rig_link.py:297` |
| 10 | `grep -n "isinstance" packages/.../subsystems/assembler.py` | 1 | **空**（唯一 isinstance 过滤已删除） |
| 11 | `sed -n '416,422p' assembler.py` | 0 | 注释：`It used to have its vertical tires deleted here ... no longer decides the wheel's lifecycle by a type test.` |
| 12 | `git ls-files '*.subsystem.json' '*.sub.json' '*.tpl.json' '*.asy.json'` | 0 | **空**（count 0） |
| 13 | `find . -name '*.subsystem.json' ...` | 0 | **空** |
| 14 | `grep -n "anti_roll_output" tests/data/axle_dynamics_baseline/sha256.json` | 0 | 13 行，值均 `e3b0c442...b7852b855`（空字节 sha256） |
| 15 | `grep -n "anti_roll_output" tests/data/vehicle_dynamics_baseline/sha256.json` | 0 | 8 行，同值 |
| 16 | `grep -in "anti_roll\|arb" tests/data/dynamic_hash_baseline.json` | 0 | **空**（无 ARB 字段） |
| 17 | `grep -rin "anti_roll\|arb" tests/data/kc_baseline/` | 1 | **空**（零命中） |
| 18 | `grep -n "_FILE_ROLE_TEMPLATES" packages/.../authoring/solver.py` | 0 | `:726 _FILE_ROLE_TEMPLATES = ("steering", "chassis", "wheel")`（**含 wheel**） |
| 19 | `grep -rn "wheel_template" packages/`（去 .pyc） | 0 | `types.py:166`/`:279`、`wheel.py:67` 等（字段**已存在**） |
| 20 | `grep -n "ports=\|needs=" packages/.../templates/builtin.py` | 1 | **空**（DOUBLE_WISHBONE 用默认 `()`） |
| 21 | `sed -n '57,66p' tests/templates/test_template_model.py` | 0 | `def test_six_roles_are_declared()` 断言六角色元组 |
| 22 | `uv run --no-sync pytest packages/suspension_multibody/tests/modeling packages/suspension_multibody/tests/templates -q -p no:cacheprovider` | **0** | `106 passed in 1.96s` |

未跑全量测试、未跑数值门（本行只读冻结，且 SPEC 仅要求 `tests/modeling`/`tests/templates` 的 `-q`）。
`git status --short` 在开工时已显示工作区**脏**（`.codex-tasks/` 多处修改、`docs/multibody_architecture_evolution.md` 未跟踪），与本行无关。

## 本行与父 Epic 的锚点漂移（只记录，不改父文件）

1. F11：`suspension.py:682-695` → 实测 `:696-709`；`element_build.py:211` → 实测入口 `:216`。
2. F12：`roles.py:158-162` 实测一致；六角色同步点实测 14 个行位置（见 `role_table_sync.md`）。
3. F13：`si_assembly.py:71 _ports_for_bodies` → 实测 `:87`；`:104 _wheel_centre_needs` → 实测 `:120`。
4. F14（**重**）：`_FILE_ROLE_TEMPLATES` 实测为 `("steering","chassis","wheel")`——**已含 wheel**，与 F14 的 `("steering","chassis")` 不符；`AssemblyRequest.wheel_template` 字段**已存在**（`types.py:166`）；`_ROLE_TEMPLATE_FIELD` 实测 `:275`（F14 写 `:249`）。`*.subsystem.json` 实体文件确认不存在。
5. F15（**重**）：`assembler.py` 的**唯一 `isinstance` 过滤已删除**（`grep isinstance` 零命中），替代为 `_AXLE_ROLES_IN_A_VEHICLE`（`assembler.py:95`）；`rig_link.py` 的 `_reown_tires`/`_is_replaced_tire` **已删除**；`vehicle_dynamic.py` 拒绝在 `:940`（非 `:900`）。这些属**阶段一 04/05 已交付**，p4-04 只需独立复验。
6. F17：四个基线文件实测与 F17 一致（无 ARB）。
