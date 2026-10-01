# `wheel_torque_amplitudes()` 的消费点与处置（判据 2）

父锚点：`EPIC.md` F1 行 112（`subsystems/brake.py:141` / `drive.py:110`）、行 243 (b)、行 251 (b)。

## 1. 改造前全量命中（`git grep` 打在 `HEAD` 上，即本行改前的提交态）

命令：

```
$ git grep -c "wheel_torque_amplitudes" HEAD -- packages/
HEAD:packages/suspension_multibody/src/suspension_multibody/subsystems/brake.py:2
HEAD:packages/suspension_multibody/src/suspension_multibody/subsystems/drive.py:3
HEAD:packages/suspension_multibody/tests/subsystems/test_brake_subsystem.py:4
HEAD:packages/suspension_multibody/tests/subsystems/test_drive_subsystem.py:4
```

逐行展开（`git grep -n`，13 处）：

| # | 文件:行 | 性质 | 处置 |
|---|---|---|---|
| 1 | `subsystems/brake.py:70` | `__all__` 里的导出名 | **删除**：函数没了，导出项随之删除（`brake.py` 现导出 `brake_amplitude` / `torque_parameters` / `wheel_torque_element`）。 |
| 2 | `subsystems/brake.py:141` | 函数定义 | **删除**，由 `brake_amplitude(parameters, *, demand, share=1.0)` + `torque_parameters(...)` + `wheel_torque_element(...)` 取代。 |
| 3 | `subsystems/drive.py:59` | `__all__` 里的导出名 | **删除**，同上（现导出 `wheel_shares` / `drive_amplitude` / `torque_parameters` / `wheel_torque_element`）。 |
| 4 | `subsystems/drive.py:81` | 模块级注释里提到该名字 | **改写**：注释段落已重写为「轮子是驱动轮因为它有元素」，不再提一个不存在的函数。 |
| 5 | `subsystems/drive.py:110` | 函数定义 | **删除**，由 `wheel_shares(driveline)` + `drive_amplitude(...)` + `torque_parameters(...)` + `wheel_torque_element(...)` 取代。 |
| 6 | `tests/subsystems/test_brake_subsystem.py:95` | `test_the_amplitude_matches_the_adams_sforce_shape` 里的调用 | **迁移**：改为 `_element(wheel=..., share=...)` 读元素（`test_brake_subsystem.py:182-194`）。 |
| 7 | `tests/subsystems/test_brake_subsystem.py:120` | 同一测试族里的 `full = ...` | **迁移**：改为 `brake.brake_amplitude(..., demand=1.0, share=0.6)`（`:206`）。 |
| 8 | `tests/subsystems/test_brake_subsystem.py:123` | 同一测试族里的 `half = ...` | **迁移**：改为 `brake.brake_amplitude(..., demand=0.5, share=0.6)`（`:207`）。 |
| 9 | `tests/subsystems/test_brake_subsystem.py:137` | 越界 demand 负例 | **迁移**：改为 `brake.brake_amplitude(ADAMS_PARAMETERS, demand=bad, share=0.6)`（`:233`），仍断言消息点名 `brake_input`。 |
| 10 | `tests/subsystems/test_drive_subsystem.py:175` | `..._matches_the_live_build_value_for_value` 里的调用 | **迁移**：改为 `_element(driveline, wheel=name, drive_input=0.75).spec.stiffness`，与 `_build_wheel_torque_signals` 逐值精确比较（`:254-292`）。 |
| 11 | `tests/subsystems/test_drive_subsystem.py:206` | 符号保持测试里的调用 | **迁移**：改为 `drive.drive_amplitude(FRONT_DRIVE, slots=..., wheel="front_left", drive=-1.0)`（`:315-322`），并断言元素侧只承载幅值。 |
| 12 | `tests/subsystems/test_drive_subsystem.py:217` | 越界 drive 负例 | **迁移**：改为 `drive.drive_amplitude(...)`（`:364-369`）。 |
| 13 | `tests/subsystems/test_drive_subsystem.py:241` | 「有扭矩无驱动轮」负例 | **迁移**：改为 `drive.wheel_shares(driveline)`（`:388`）。 |

## 2. 改造后剩余命中

```
$ grep -rn --include=*.py "wheel_torque_amplitudes" \
    packages/suspension_multibody/src packages/suspension_multibody/tests \
    packages/suspension_kernel packages/suspension_contracts
SRC exit=1          # 命中 0 处
```

全仓（含文档）只剩任务目录里的历史文本：

```
$ grep -rln "wheel_torque_amplitudes" . | grep -v "^./.git/"
./.codex-tasks/20260922-suspension-template-architecture/tasks/20260922-04-subsystems/raw/brake_torque_evidence.md
./.codex-tasks/20260922-suspension-template-architecture/tasks/20260922-04-subsystems/TODO.csv
./.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md
./.codex-tasks/20260929-multibody-evolution-p2-p5/PROGRESS.md
./.codex-tasks/20260929-multibody-evolution-p2-p5/SUBTASKS.csv
./.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p2-04-brake-drive/PROGRESS.md
./.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p2-04-brake-drive/SPEC.md
./.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p2-04-brake-drive/TODO.csv
```

这些都是**任务文档**（父 Epic、行表、以及本行的 SPEC/TODO/PROGRESS），不是代码消费点；`EPIC.md`/`SUBTASKS.csv` 按任务约束不得改，历史证据文件也不该改。

## 3. 一个对 SPEC 前提的更正（实测）

`SPEC.md:85` 与 `TODO.csv` 第 4 行都写「外部还有 `cases/vehicle_dynamic.py:579/582` 的契约表（归 p2-05）」。**实测这个说法不成立**：

```
$ sed -n '565,583p' packages/suspension_multibody/src/suspension_multibody/cases/vehicle_dynamic.py
        torque = prepared.wheel_torque.get(tire.name)
        if torque is not None:
            _append_table(blob, tables, torque, role="wheel_torque", tire=tire.name)
        brake = prepared.brake_torque.get(tire.name)
        if brake is not None:
            _append_table(blob, tables, brake, role="brake_torque", tire=tire.name)
```

`cases/vehicle_dynamic.py:577-582` 读的是 `prepared.wheel_torque` / `prepared.brake_torque`——即 `prepare_vehicle_run` 返回的 `VehicleRuntime`/prepared 对象上的**采样表**，由 `preparation/vehicle_dynamic.py::_build_wheel_torque_signals` 产出；它**不调用** `brake.wheel_torque_amplitudes` 或 `drive.wheel_torque_amplitudes`：

```
$ grep -rn "subsystems.brake\|subsystems.drive\|from .brake\|from .drive" \
    packages/suspension_multibody/src/suspension_multibody/cases packages/suspension_multibody/src/suspension_multibody/preparation
（无输出）
```

所以 `wheel_torque_amplitudes()` 的消费点**全部在 `brake.py`/`drive.py` 自身与两个子系统测试里**（13 处，逐条处置见上表），没有第三方代码消费点，`cases/**` 与 `preparation/**` 也不需要为这个函数做迁移。p2-05 要改的是那两个模块的产出通道（把采样表换成元素），与本行的函数删除无关——本行**未改** `cases/**` 与 `preparation/**`。

## 4. 产出力矩元的入口（改造后）

```python
# brake.py
brake.brake_amplitude(parameters, *, demand, share=1.0) -> float
brake.torque_parameters(parameters, *, wheel, demand, share=1.0, axis_a=None)
    -> RotationalTorqueParameters
brake.wheel_torque_element(parameters, *, wheel, own_body, report, ports,
                           demand, share=1.0, axis_a=None) -> ResolvedElement

# drive.py
drive.wheel_shares(driveline) -> dict[str, float]
drive.drive_amplitude(driveline, *, slots, wheel, drive) -> float
drive.torque_parameters(driveline, *, slots, wheel, drive, axis_a=None)
    -> RotationalTorqueParameters
drive.wheel_torque_element(driveline, *, slots, wheel, own_body, report, ports,
                           drive, axis_a=None) -> ResolvedElement
```

`ResolvedElement` 是 `subsystems/types.py` 的行类型，其 `kind` 为 `"rotational_torque"`；构造走 `subsystems/element_build.py` 的分派（p2-03 的 `:74-75`），测试用 `build_element(row)` 真构造一次（`test_brake_subsystem.py:255-266`、`test_drive_subsystem.py:398-406`）。

## 5. 未做到 / 不确定

- **组合层还没有调用点**：本行只交「子系统侧能产出力矩元」，`subsystems/composition.py`（归 p4-02）与 `preparation/vehicle_dynamic.py`（归 p2-05）尚未接线，因此**没有任何生产路径**在跑 `wheel_torque_element`。所以上面 13 处全部是「删除」或「测试迁移」，不存在「生产消费点迁移到新元素」这一步——**这是本行的实情，不是遗漏**；把元素接到装配上的那一行不在本行写范围内。
- 因为组合层未接线，`rotational_torque` 元素目前**没有**进入任何 `model_document`/契约文档；内核文档路由不接受 `rotational_torque` 的缺口（见 `raw/reaction_paths.md` 第 5 节）因此没有在本行被触发。
- `drive.drive_amplitude` 保留 live builder 的**带符号**值（与 `_build_wheel_torque_signals` 逐位一致），而元素本身承载幅值——这是内核力律的限制，登记在 `raw/reaction_paths.md` 第 4 节，不在这里重复。
