# PROGRESS：p3-05 报表通道按安装角色动态注册

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p3-05`

## Session Start

- **Date**: 2026-10-01
- **Task name**: p3-05-dynamic-channels
- **Task dir**: `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p3-05-dynamic-channels/`
- **Spec**: 见 `SPEC.md`
- **Plan**: 见 `TODO.csv`（5 步，全部 `DONE`）
- **Environment**: Python 3.12 / uv / pytest（`uv run --no-sync`）

## Context Recovery Block

- **Current milestone**: #5 — 分层门与 `outputs/builtin.py` 改动段落登记（收口）
- **Current status**: DONE
- **Last completed**: 五条验收命令全绿（见 `raw/run_log.md`）
- **Current artifact**: `raw/dynamic_channel_registration.md`、`raw/four_wheel_compat.md`、`raw/three_axle_channels.md`、`raw/metrics_test_changes.md`、`raw/run_log.md`、`raw/*.json`、`raw/three_axle_dump.txt`、`raw/channel_probe*.txt`
- **Key context**:
  - 父行 `depends_on = p3-04`；字段形状以 `tasks/p3-04-static-loads/raw/static_loads_contract.md` 为准，本行未重定义。p3-04 该文件第 49-52 行提示的「`result.summary` 只在四轮可用」正是本行解除的限制。
  - `outputs/builtin.py` 的派生输出声明归本行（`EPIC.md` 行 221），p5-03 须在本行之后串行改同一文件——改动段落见下面的「登记」节。
  - `report/` 分层约束：本行未 import native/kernel/solver/preparation，未自求力律（`legacy_surface_gate.py --check` 退出码 0，`findings: 0`）。
  - 验收命令口径：`packages/suspension_multibody/tests/report/` 目录今天仍不存在，本行**未**新建它，故命令用 `tests/metrics` 与 `tests/outputs`。
- **Known issues**:
  - 开工期间检测到**另一会话并发改 `subsystems/suspension.py`**（11:11 一次 `global_elements` 缺失导致的 4 个假失败，45 s 后重跑恢复）。该文件不在本行写范围；已记入 `raw/run_log.md`。
  - 三轴的**声明层**能力仍未打通（`RIG_OUTPUTS` / `minimum_unit_outputs` 只声明四角）——登记给主代理与 p5-03，不在本行写范围。
- **Next action**: 无（本行收口）。

## 登记（本行特有：`outputs/builtin.py` 改动段落与测试同步理由）

### 1. `outputs/builtin.py` 改动段落（供 p5-03 串行对接）

本行只改「**派生输出声明段**」，共 4 处，逐段登记（行号为改造后终态）：

| # | 段落 | 位置 | 改动内容 |
|---|---|---|---|
| 1 | `_LOAD_SIDES` 常量的新增 | `builtin.py:337` | 新增 `_LOAD_SIDES: tuple[str, str] = ("left", "right")`，供下列生成函数用 |
| 2 | `_loads` / `load_scalar` 表达式段（`# --- expressions ---` 段内） | `builtin.py:348-421` | 新增 `_load_placement`（`:348`）与 `_load_sum`（`:359`），新增 `_load_channel_table`（`:367`，由 `placement` 生成通道表）；`load_scalar`（`:416`）改为 `return _load_channel_table(_loads(view))[field]`；`_loads`（`:340`）仅改 docstring |
| 3 | `_load_outputs` 声明段（`# --- the restatement ---` 段内） | `builtin.py:683-724` | 新增 `_load_channel_names()`（`:683`，由 `_WHEELS` 生成 13 个声明名）；`_load_outputs()`（`:710`）改为迭代 `_load_channel_names()`，不再写死 11 个名字 |
| 4 | 无其它改动 | —— | `ASSEMBLY_OUTPUTS`（`:154`）、`RIG_OUTPUTS`（`:220`）、`DERIVED_OUTPUTS` 组装（`:760`）、`LEGACY_CLASSIFICATION`（`:774`）、`register_builtins`（`:927`）、`minimum_unit_outputs`（`:982`）**均未改** |

**净变化**：`git diff --stat` = `outputs/builtin.py | 134 ++++++++--`（105 插入 / 29 删除）。
**派生输出净增**：`report.metrics.vehicle.wheel_load_metrics` 名下的声明从 11 个变 13 个（新增 `normal_load_axle_front` / `normal_load_axle_rear`），顺序见 `tests/outputs/test_load_outputs.py::_DECLARED_LOAD_OUTPUTS`。

**给 p5-03 的两条边界**：

1. `RIG_OUTPUTS`（`:220`，**非本行段，未改**）仍只声明 `wheel_load_front_left/front_right/rear_left/rear_right`；`minimum_unit_outputs`（`:982`，**非本行段，未改**）的 `wheel_loads` 分支仍只读 `_WHEELS`。所以三轴的 `wheel_load_middle_*` 输入在本行结束时**没有声明**，`normal_load_axle_middle` 也**不是**静态声明的派生输出。要补这一段就得改上面两个非本行段 —— 需要主代理明确授权给 p5-03。
2. `LEGACY_CLASSIFICATION`（`:774`，**非本行段，未改**）里两行的通道名单仍是 11 个旧名，未含新的 `normal_load_axle_*`：
   - `report.metrics.case_specific._vehicle_dynamic_metrics`（`:848-854`）；
   - `report.metrics.vehicle.wheel_load_metrics`（`:904-911`）。
   这是**已登记的滞后**，不是遗漏：该段是分类表，不是「派生输出声明」段。`tests/metrics/test_outputs_match_legacy.py::test_every_legacy_function_is_classified_and_there_are_27` 仍然通过（它只比对函数名集合，不比通道名单）。

### 2. `tests/metrics/test_outputs_match_legacy.py` 同步理由

**必要**：既有用例 `test_the_wheel_load_metrics_are_restated_value_for_value` 的名字集合**取自被测函数自己**（`for name in legacy.items()`），因此它能发现「某个名字的值算错了」，但**不能**发现「某个历史名字消失了」——表从 11 项变 13 项后这一点更明显。兼容硬门（SPEC 判据 (b)）要求的正是后者。

**做法**：不改写原用例（只加两行说明注释），另加两条用例 + 一份**独立**的 11 项历史名单 `LEGACY_LOAD_CHANNELS`（`:104-116`）：

- `test_the_wheel_load_metrics_still_publish_every_historical_channel`：11 项名字与值逐项相等 + 两个新名与历史名同值 + 键集合等于「历史 11 + 新 2」；
- `test_the_declared_load_outputs_are_exactly_the_report_channel_table`：报表层键集 == 声明层名字集，并逐项比値（这份重复的防漂移锁）。

**断言未减弱**：原用例断言一字未动（`assert legacy` + 全键 `_compare`，现在覆盖 13 项）；其余 10 个用例未改。改动前后断言清单对照见 `raw/metrics_test_changes.md` 第 3 节。新增 0 个 skip、0 个 xfail。

### 3. 报表字段名集合变化登记（D5 口径）
| 文件 | 步骤 | 前 | 后 | 独立于结果字节的物理等价判据 |
|---|---|---|---|---|
| `report/wheel_loads.py` → `wheel_load_channels` | 4 轮 | 11 个通道 | 13 个 | 新增 2 项是既有 `normal_load_front_axle` / `normal_load_rear_axle` 的**同值别名**（`repr` 级相同），既有 11 项的**名字、相对顺序、值**逐位不变（`raw/channel_probe_compare.txt`：`moved channels: 0`） |
| `outputs/builtin.py` 派生输出声明 | 4 轮 | 11 个声明 | 13 个（+2 同值别名） | 同上；值由 `test_the_declared_load_outputs_are_exactly_the_report_channel_table` 逐项锁定 |
| 冻结基线（`kc_baseline/**`、`dynamic_hash_baseline.json`、`kc_perf_baseline_native.json`） | — | — | **逐字节未变** | 未重录；`dynamic_hash_sentinel.py --check` 退出码 0，sha256 = `fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9`（与冻结值一致） |

### 4. 行为放宽（唯一一处，登记）

`report/` 里原来「恰好四角，否则 `ValueError: wheel loads must contain exactly the four vehicle corners`」这条校验被替换为「至少一个轮端 + 每个名字必须声明 `<placement>_<side>` + 值有限」。这是本行目的（三轴/单轮本来拿不到任何通道），代价是一个更松的输入检查：一个拼错的名字（如 `normal_load_frontRight`）现在会被按「不声明侧」拒绝而不是「角数不符」，消息里点名该轮端。测试覆盖：`tests/metrics/test_placement_channels.py::test_a_name_that_states_no_side_is_refused_by_name`。

`outputs/builtin.py` 一侧的收紧**未变**：它仍只声明四个 `wheel_load_<corner>`，读不到就抛 `MissingOutputError`。

---

## Final Summary

把 `report/wheel_loads.py` 与 `report/metrics/vehicle.py` 里两处**重复**的硬编码前后轴字段（连同「强制恰好四角」校验）改为按安装角色（`placement`）动态生成：

- **真源**：`report/wheel_loads.py::wheel_load_channels`（`:137`）是发布通道表的唯一入口；放置名从轮端名 `f"{placement}_{side}"` 读出（`:59`）。
- **重复消灭**：`report/metrics/vehicle.py` 的 `_WHEELS` 与整张 11 项表删除，`wheel_load_metrics` 委派给真源（`vehicle.py:23`、`:28-30`）。
- **声明层保留一份生成式重复**：`outputs/builtin.py:683` 的 `_load_channel_names()` 从 `_WHEELS` 生成声明名，`load_scalar` 走 `_load_channel_table`（`:367`）；两份由 `tests/metrics/test_outputs_match_legacy.py` 的值级对照锁住不漂移。原因（`outputs/` 不得依赖 `report/`）写在 `builtin.py:371-377`。

**证据（全部实跑，见 `raw/run_log.md`）**：

| 判据 | 结果 |
|---|---|
| (a) 动态注册 | `raw/dynamic_channel_registration.md`：生成代码位置表 + 4 轮/3 轴/单轮三份通道清单 + 真源归属 |
| (b) 4 轮逐项一致 | `raw/four_wheel_compat.md`：`moved channels: 0` / `OK: every pre-existing channel kept its name, order and value`（`repr` 级，两个 4 轮用例） |
| (c) 3 轴三通道 + 改名实验 | `raw/three_axle_channels.md`：`normal_load_axle_front/middle/rear` 三者均出现；`bogie`/`trailer` 改名实验整表改名、输出无 `front`/`middle`/`rear` |
| (d) 测试同步 + 理由 | `raw/metrics_test_changes.md` + 上文登记节；断言未减弱（改动前后清单对照） |
| 分层门 | `legacy_surface_gate.py --check` 退出码 0，`findings: 0` |
| 基线 | `dynamic_hash_sentinel.py --check` 退出码 0，sha256 未变；未重录任何基线 |
| 静态门 | `ruff check .` 0；`ty check .` 0；`check_module_layering.py --strict --final` 0；`check_composable_release.py --skip-isolation` 0 |
| 测试 | `tests/metrics + tests/outputs`：61 → **75 passed**；快速集 **1182 passed / 1 xfailed**；kernel+contracts **73 passed**；无新增 skip/xfail |

**未完成（不在本行写范围，需主代理裁决）**：三轴的**声明层**能力。`RIG_OUTPUTS` / `minimum_unit_outputs`（`builtin.py:220` / `:982`）仍只声明四个轮端输入，因此 `normal_load_axle_middle` 目前只是运行期能力（`report/` 层已具备、已被 3 轴测试证明），不是静态声明的派生输出；补它必须改这两个非本行段。已用 `tests/outputs/test_load_outputs.py::test_a_placement_the_declaration_does_not_declare_is_not_declared` 把该边界固化成可判定的断言，供 p5-03 对接。`LEGACY_CLASSIFICATION` 两行的通道名单滞后同样已登记（见登记节第 1 节第 2 条）。
