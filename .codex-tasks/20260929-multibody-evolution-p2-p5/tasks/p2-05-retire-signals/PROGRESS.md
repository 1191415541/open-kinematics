# PROGRESS：p2-05 废除离线预采样力矩与 front_brake_bias

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p2-05`
> 状态：**DONE**（2026-10-01）

## Session Start

- **Date**: 2026-10-01
- **Task name**: p2-05-retire-signals
- **Task dir**: `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p2-05-retire-signals/`
- **Spec**: 见 `SPEC.md`（判据 1/2 已按裁决 `ddc3f952` 修订）
- **Plan**: 见 `TODO.csv`（5 步，全部 DONE）
- **Environment**: Python 3.12 / uv / pytest

## 判据修订（裁决 `ddc3f952`，开工前落定）

原判据要求「`_build_wheel_torque_signals` 与 `front_brake_bias` 被删除且 grep 全仓零命中」。实测证明
**该判据与零回归硬门不可同时成立**：把默认路径也改走力矩元后，
`case_parity_check.py --family vehicle_dynamic` 立即报

```
NativeAxleError: case native-vehicle failed with status 5:
time integration failed at t=0.000000 s: Newton solve did not converge
```

那 8 个冻结用例的模型是同一个整车夹具，它给初速即不收敛（与 p2-11 的独立实测一致；既有 `xfail`
`tests/vehicle/test_native_vehicle.py:1698-1721` 记录同一结构性问题）。实验已回滚。

修订后的判据是**可判定的隔离**：

- `_build_wheel_torque_signals` 的**唯一调用点位于 `torque_demand == "none"` 分支内**，三种 opt-in 取值下调用次数为 0；
- `front_brake_bias` **保留字段**（非 exclude，删除会改 `model_dump(mode="json")` 与 `model_hash`），
  但 **opt-in 路径对它的读取次数为 0**，分配口径改由角色常量驱动。

## 改动清单

| 文件 | 改动 |
|---|---|
| `packages/suspension_multibody/src/suspension_multibody/preparation/vehicle_dynamic.py` | `prepare_vehicle_run` 的力矩段（原 `:295-312`）：读 `getattr(model.driveline, "torque_demand", "none")`；`none` 分支调 `_build_wheel_torque_signals` 并把两张需求表置空，非 `none` 分支把两张旧表置空并调 `_build_demand_signals` |
| `packages/suspension_multibody/src/suspension_multibody/subsystems/torque_elements.py` | 新增角色常量 `BRAKE_FRONT_SHARE = 0.6` 与 `brake_front_share()`；`brake_share()` 不再读 `model.driveline.front_brake_bias`，改读该常量 |

`none` 分支的源码与改动前**逐字相同**——唯一变化是在它外面加了一层 `if`，故冻结基线的逐位一致是
构造性的。未新增测试文件、未改既有断言、未新增 skip/xfail。

## 实测证据（落 `raw/`）

| 判据 | 证据文件 | 关键实测 |
|---|---|---|
| (a) 调用与数据流隔离 | `raw/retirement_grep.md` | `call sites of _build_wheel_torque_signals: 1`（`vehicle_dynamic.py:308`，位于 `none` 分支内）；`front_brake_bias attribute reads in torque_elements: 0`；四档声明 `none`→元素 0/旧表 4+4/新表 0+0，`brake`→4/0+0/0+4，`drive`→2/0+0/2+0，`both`→6/0+0/2+4 |
| (b) 力矩时程一致性 | `raw/torque_history_parity.md` | `case_parity_check.py --family vehicle_dynamic` → **PASS 8 cases, bit-identical**；差值 **0**（逐位，非容差内） |
| (c) 契约表 | `raw/contract_tables.md` | 默认路径 role 集合与表头逐项不变，`tests/cases/test_vehicle_dynamic_contract.py` **未改**；opt-in 三档 role/元素数对照 |
| (d) 动态基线登记 | `raw/dynamic_hash_registration.md` | combined sha256 `fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9` **逐字节**；`git status --short -- packages/suspension_multibody/tests/data/` 为空 |
| (e) 静止/倒车振荡失真断言 | `raw/stall_reverse_assertion.md` | `test_the_couple_the_kernel_applied_is_non_zero` 读内核 code-10 真实数值：末样本范数 `0.0`（静止对不产生反向加速）、首样本 `1.0 N·m`、逐样本等大反向 `atol=0`；倒车符号断言在内核侧 `test_rotational_torque.py` |

## 验证命令与退出码

```
$ uv run --no-sync pytest packages/suspension_multibody/tests/cases \
    packages/suspension_multibody/tests/vehicle \
    packages/suspension_multibody/tests/subsystems -q -p no:cacheprovider
383 passed, 1 xfailed in 81.31s                      # exit 0

$ uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check
artifacts hashed : 26
combined sha256  : fdfd5a6b...eebc9
OK: dynamic output matches the frozen baseline byte-for-byte   # exit 0

$ uv run --no-sync python packages/suspension_multibody/scripts/case_parity_check.py --family vehicle_dynamic
  vehicle_dynamic    PASS      8 cases, bit-identical to the frozen snapshot
OK: 1 families accepted                              # exit 0
```

## 未声称

- **不声称**整车 `vehicle_dynamic` 夹具上的非零力矩响应：该夹具要非零相对角速度就得给初速，
  而给初速即 `status 5` 不收敛；`static_equilibrium=True` 可收敛但要求零初速，故命中「静止对不施力偶」分支。
  这条硬判据由 **p2-11** 用专用两体装置独立验收（实测 code-10 首样本 `1.0 N·m`、等大反向 `atol=0`、
  末样本 `0.0`、对照组 0 行）。
- **不声称** `templates/builtin.py` 的 `BRAKE.property_slots` 已含「分配 share」的槽。按裁决 `ddc3f952`，
  分配口径由本行角色自己的常量 `BRAKE_FRONT_SHARE` 驱动；模板槽集是 p2-04 已交付的四项标准化物理参数，
  本行写范围不含模板。

## 未越界

- 未改 `preparation/vehicle_dynamic.py` 的转向段（归 p2-06）与轮胎拒绝段（归 p4-04）。
- 未改 `cases/vehicle_kc.py`（归 p2-06）、`templates/roles.py` 的角色表（归 p2-04）、内核（归 p2-02）。
- 未改 ABI 版本常量；未重录任何基线；未新增 skip/xfail。
