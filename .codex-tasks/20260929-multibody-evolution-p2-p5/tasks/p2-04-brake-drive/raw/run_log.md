# 执行记录（只记已执行的结果）

任务目录：`.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p2-04-brake-drive/`
环境：Windows / Git Bash / Python 3.12.13 / pytest 8.3.4 / uv（全部命令都是 `uv run --no-sync ...`）。
所有命令在仓库根 `E:\杂件\open-kinematics` 执行。日期：2026-09-29（会话日）。

---

## 1. 五条验收命令（真实退出码）

| # | 命令 | 输出摘要 | 退出码 |
|---|---|---|---|
| 1 | `uv run --no-sync pytest packages/suspension_multibody/tests/subsystems/test_brake_subsystem.py packages/suspension_multibody/tests/subsystems/test_drive_subsystem.py packages/suspension_multibody/tests/templates -q -p no:cacheprovider` | `82 passed in 1.69s` | **0** |
| 2 | `uv run --no-sync pytest packages/suspension_multibody/tests/subsystems packages/suspension_multibody/tests/templates -q -p no:cacheprovider` | `232 passed in 4.84s` | **0** |
| 3 | `uv run --no-sync pytest packages/suspension_multibody/tests/architecture -q -p no:cacheprovider` | `147 passed in 587.86s (0:09:47)` | **0** |
| 4 | `uv run --no-sync python packages/suspension_kernel/scripts/check_module_layering.py --strict --final` | `OK: layering matches the recorded baseline` | **0** |
| 5 | `uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check` | 见下（combined sha256 一致） | **0** |

第 1 条在写完 `raw/` 与 PROGRESS/TODO 之后复跑过一次，仍为 `82 passed in 1.67s`（退出码 0）。

第 5 条的完整关键输出：

```
solver self-convergence: FAILED
adams accuracy: BLOCKED
BLOCKED: no real Adams evidence supplied: ...
BLOCKED: the frozen median-of-N timing protocol was not run: pass --performance to collect it
  static_equilibrium: PASSED
  road_step_finite_rise: FAILED
  ... (acceptance_report 里的用例判定)
artifacts hashed : 26
combined sha256  : fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9
acceptance exit  : 1
failed cases     : ['combined_load', 'in_phase_road', 'large_amplitude_high_frequency',
                    'opposite_phase_road', 'road_pulse', 'road_sine',
                    'road_step_finite_rise', 'single_wheel_road', 'tire_liftoff_and_recontact']

OK: dynamic output matches the frozen baseline byte-for-byte
```

**sentinel 的 combined sha256 = `fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9`，与冻结值一致。** 脚本自身退出码 0（`OK: ... byte-for-byte`）；上面那些 FAILED/BLOCKED 行是脚本**读取**的 `artifacts/axle-dynamics-acceptance/acceptance_report.json` 里被冻结的既有结论，与本次改动无关（哈希一致即证明它们没变）。

```
$ git status --short -- packages/suspension_multibody/tests/data/
（无输出）
DATA_CLEAN_EXIT=0
```

## 2. 静态门

```
$ uv run --no-sync ruff check packages/.../subsystems/brake.py packages/.../subsystems/drive.py \
      packages/.../subsystems/types.py packages/.../templates/roles.py packages/.../templates/builtin.py \
      packages/.../tests/subsystems/test_brake_subsystem.py packages/.../tests/subsystems/test_drive_subsystem.py
All checks passed!
RUFF_EXIT=0

$ uv run --no-sync ruff check .
Found 19 errors.
RUFF_EXIT=1
```

仓库级 `ruff check .` 的 19 条**全部落在本行写范围之外**，逐文件计数：

```
      6 .codex-tasks/.../tasks/p3-01-freeze/probe_axle_rollcenter.py
      4 .codex-tasks/.../tasks/p3-01-freeze/probe_three.py
      3 .codex-tasks/.../tasks/p2-01-freeze/raw/rear_steer_probe.py
      2 packages/suspension_multibody/tests/physics/test_roll_centres_by_topology.py
      2 .codex-tasks/.../tasks/p3-01-freeze/probe_configs.py
      1 .codex-tasks/.../tasks/p3-01-freeze/probe_trailingarm_vehicle.py
      1 .codex-tasks/.../tasks/p3-01-freeze/probe_explicit5.py
```

规则分布：`E402`×5、`F401`×6、`E401`×2、`E702`×2、`F541`×2、`D401`×2。其中 `packages/suspension_multibody/tests/physics/test_roll_centres_by_topology.py`（2 条）属于**并发 agent** 正在改的文件（`tests/physics/**`），其余是其他子任务的 `raw/` 探针脚本。本行的 7 个文件零命中。

```
$ uv run --no-sync ty check .
All checks passed!
TY_EXIT=0
```

（第一次跑时 ty 报了两条 `invalid-argument-type`，都在本行：`ports: Mapping[str, object]` 与 `pair_torque_bodies` 期望的 `Mapping[str, PortSpec]` 不符。已把两个 `wheel_torque_element` 的形参注解收窄为 `Mapping[str, PortSpec]`（`brake.py` 从 `..modeling.ports` 导入 `PortSpec`），之后 ty 全绿——这是本行唯一一处为静态门做的改动，同时让签名更准确。）

其他结构门：

```
$ uv run --no-sync python packages/suspension_multibody/tests/architecture/legacy_surface_gate.py --check
OK: no unregistered Python boundary violation
G1_EXIT=0

$ uv run --no-sync python packages/suspension_multibody/scripts/check_composable_release.py --skip-isolation
OK: 3 release checks passed
G2_EXIT=0
```

## 3. 快速测试集（AGENTS.md 第 1 节，作为回归证据）

```
$ uv run --no-sync pytest packages/suspension_multibody/tests \
    --ignore=packages/suspension_multibody/tests/adams \
    --ignore=packages/suspension_multibody/tests/architecture \
    --ignore=packages/suspension_multibody/tests/cases -q -p no:cacheprovider
1153 passed, 1 xfailed in 39.43s
FAST_EXIT=0
```

`1 xfailed` 是既有的 `tests/vehicle/test_native_vehicle.py::test_native_brake_opposes_the_instantaneous_wheel_spin`（marker 在改动前就存在，见 `raw/contract_test_updates.md` C1）。**没有新增 skip/xfail**。

注：AGENTS.md 记的基线是「854 passed / 1 xfailed，约 25 秒」；本次多出的是**其他并发 agent** 在同一工作区新增的 `tests/physics/**`、`tests/axle_dynamics/**` 用例（例如 `test_roll_centres_by_topology.py` 是本次新出现的文件名）。本行的 7 个文件在这一轮里全部通过。

## 4. 判据 3 的定向命令

```
$ uv run --no-sync pytest \
    ".../test_brake_subsystem.py::test_the_reaction_body_comes_from_the_matched_port" \
    ".../test_brake_subsystem.py::test_a_missing_reaction_port_is_refused_by_role" \
    ".../test_brake_subsystem.py::test_the_brake_path_names_no_body_and_no_reacting_part" \
    ".../test_drive_subsystem.py::test_the_reaction_body_comes_from_the_matched_port" \
    ".../test_drive_subsystem.py::test_the_drive_path_names_no_body_and_no_reacting_part" \
    -v -p no:cacheprovider
... 5 passed in 1.44s
```

```
$ for f in .../brake.py .../drive.py; do for p in upright chassis; do echo "$f $p -> $(grep -ic $p $f)"; done; done
brake.py upright -> 0
brake.py chassis -> 0
drive.py upright -> 0
drive.py chassis -> 0
```

## 5. 判据 2 的定向命令

```
$ git grep -c "wheel_torque_amplitudes" HEAD -- packages/
HEAD:.../subsystems/brake.py:2
HEAD:.../subsystems/drive.py:3
HEAD:.../tests/subsystems/test_brake_subsystem.py:4
HEAD:.../tests/subsystems/test_drive_subsystem.py:4

$ grep -rn --include=*.py "wheel_torque_amplitudes" \
    packages/suspension_multibody/src packages/suspension_multibody/tests \
    packages/suspension_kernel packages/suspension_contracts
wta_src_exit=1            # 0 命中
```

## 6. 判据 1 的定向命令

```
$ uv run --no-sync python "$PI_SCRATCH_DIR/negative_slot.py"        # 七个槽逐个摘除
--- remove 'piston_area'; suspension_multibody.templates.model.TemplateError
    message: template 'brake_4wdisk_simplified' does not satisfy role 'brake': missing property slot(s) ['piston_area']
    layer:   model.py:363 (check_role_contract)
... (四个 brake 槽 + 三个 drive 槽，均点名缺的那个槽，EXIT=0)

$ uv run --no-sync python -c "from suspension_multibody.templates import ROLES; print(tuple(ROLES['brake'].required_slots)); print(tuple(ROLES['drive'].required_slots))"
('piston_area', 'effective_radius', 'friction_coeff', 'rotor_inertia')
('gear_ratio', 'efficiency', 'max_torque')
EXIT=0

$ uv run --no-sync python -c "...template_document_from(BRAKE/DRIVE)..."
brake_4wdisk_simplified has needs key: False
powertrain_simplified has needs key: False
EXIT=0
```

## 7. 判据 4 的定向命令

```
$ uv run --no-sync pytest ".../test_native_vehicle.py::test_brake_signal_is_a_nonnegative_magnitude" \
    ".../test_native_vehicle.py::test_native_brake_opposes_the_instantaneous_wheel_spin" -q -p no:cacheprovider
.x                                                                       [100%]
1 passed, 1 xfailed in 2.37s

$ uv run --no-sync pytest ".../test_native_vehicle.py::test_native_vehicle_combines_trim_road_steering_and_drive" \
    ".../test_native_vehicle.py::test_direct_wheel_torque_signals_override_global_distribution" \
    "packages/suspension_multibody/tests/cases/test_vehicle_dynamic_contract.py::test_the_case_document_carries_the_road_and_the_steering" \
    -q -p no:cacheprovider
...                                                                      [100%]
3 passed in 1.56s
```

## 8. 文件清单

新增（未跟踪）：

```
?? .codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p2-04-brake-drive/raw/
    template_slots.md
    wheel_torque_amplitudes.md
    reaction_paths.md
    contract_test_updates.md
    run_log.md
```

修改：

```
 M packages/suspension_multibody/src/suspension_multibody/subsystems/brake.py
 M packages/suspension_multibody/src/suspension_multibody/subsystems/drive.py
 M packages/suspension_multibody/src/suspension_multibody/subsystems/types.py
 M packages/suspension_multibody/src/suspension_multibody/templates/builtin.py
 M packages/suspension_multibody/src/suspension_multibody/templates/roles.py
 M packages/suspension_multibody/tests/subsystems/test_brake_subsystem.py
 M packages/suspension_multibody/tests/subsystems/test_drive_subsystem.py
```

本行 7 个文件的 `git diff --stat`：

```
 packages/.../subsystems/brake.py   | 318 ++++++++++++++-------
 packages/.../subsystems/drive.py   | 288 ++++++++++++++-----
 packages/.../subsystems/types.py            |   8 +
 packages/.../templates/builtin.py           |  45 ++-
 packages/.../templates/roles.py             |  25 +-
 packages/.../tests/subsystems/test_brake_subsystem.py | 308 +++++++++++++++-----
 packages/.../tests/subsystems/test_drive_subsystem.py | 300 +++++++++++++----
7 files changed, 965 insertions(+), 327 deletions(-)
```

未触碰的禁区（用 `git status` / `git diff` 自证为空）：`subsystems/element_build.py`、`subsystems/composition.py`、`subsystems/capabilities.py`、`preparation/vehicle_dynamic.py`、`cases/**`、`connections/policy.py`、`authoring/documents.py`、`packages/suspension_kernel/**`、`kernel/native.py`、`api.py`、包根 `__init__.py`、三份契约 schema、`tests/data/**`、`SUBTASKS.csv`、`EPIC.md`；并发 agent 的 `vehicle/roll_centers.py`、`vehicle/screw_kinematics.py`、`tests/physics/**`、`tests/vehicle/**` 未被本行写入。

## 9. 未做到 / 不确定

- **组合层未接线**（见 `raw/reaction_paths.md` §5.1）：本行的元素没有任何生产调用点，因此 §1 的五条命令里没有一条会跑到力矩元本身；判据 3 的断言证明的是子系统构造路径。
- **模板 `needs` 会被导出/读回丢掉**（同上 §5.2）：实测证据给出，本行未加 `needs`。
- **内核文档路由不接受 `rotational_torque`**（同上 §5.3）：`contract_model.cpp:831`，本行不属修复范围。
- **驱动元素的符号**（同上 §5.4）：力律是阻力律，本行断言而不绕过。
- **仓库级 `ruff check .` 退出码为 1**，19 条全部来自并发 agent 与其他子任务的 `raw/` 脚本（见 §2）。本行的 7 个文件零命中；本行不修改这些文件。
- **`ty check .` 与 `ruff`（本行 7 文件）退出码 0**，可作静态门证据。
