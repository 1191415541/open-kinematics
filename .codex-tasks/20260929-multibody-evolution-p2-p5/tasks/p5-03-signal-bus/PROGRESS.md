# PROGRESS：p5-03 信号总线（测点与执行器）

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p5-03`

## Session Start

- **Date**: 2026-10-01
- **Task name**: p5-03-signal-bus
- **Task dir**: `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p5-03-signal-bus/`
- **Spec**: 见 `SPEC.md`
- **Plan**: 见 `TODO.csv`（5 步，全部 `DONE`）
- **Environment**: Python 3.12 / uv / pytest
- **Status**: **DONE**；仓库根 `c:/杂件/open-kinematics`
- **Depends on**: `p5-02`（`SUBTASKS.csv` 第 21 行），并受全局前置 `S1`（阶段一 Epic 01–07 全 `DONE`，`EPIC.md` 行 69/75）约束

## Context Recovery Block

- **Current milestone**: #1 — 信号总线模块骨架与 `outputs` 声明关系定义
- **Current status**: NOT_STARTED
- **Last completed**: 无（本子任务尚未开工）
- **Current artifact**: `SPEC.md` / `TODO.csv`（规划产物）
- **Key context**:
  - 今天**不存在** SignalBus/Sensor/Measurement 抽象（`EPIC.md` F19，全仓 `class SignalBus|class Measurement|class Sensor|signal_bus` 零命中）。
  - 现存两套互不相通：`outputs/` 静态声明（`outputs/builtin.py:154 ASSEMBLY_OUTPUTS`、`:220 RIG_OUTPUTS`、`outputs/declarations.py:1`，**未被 api.py 或 rigs 生产路径消费**，仅测试引用）与 `results/channels.py:22 ChannelRegistry`（只读，读 `adams/axle_channels.yaml:20`，是**冻结的 Adams 输出通道表**）。
  - 今天**无可变阻尼/CDC**（`EPIC.md` F20，`grep -i "damper_ratio|variable_damp"` 零命中）；既有 actuator 全是开环/预设的 `TimeSignal`。
  - 写范围按 `SUBTASKS.csv` 第 21 行 `notes`：新增信号总线模块 + `outputs/builtin.py` 的派生输出声明 + `api.py` 的暴露段。
  - **与 p3-05 共享 `outputs/builtin.py`，必须串行**（`EPIC.md` 行 221：该文件的派生输出声明归 p3-05）。
- **Known issues**:
  - 新增测点会**改变输出集合**——D5 口径（`EPIC.md` 行 64）已点名阶段五这种情形：必须逐项登记，**禁止重录**基线。
  - 总线**不得重算**测点值（`EPIC.md` 行 275(b)）；只能读既有结果文档。
  - D2（`EPIC.md` 行 61）：内核是批式 ABI（F23），本行只做 Python 侧总线与开环，闭环与内核单步需求交 p5-04 登记。
- **Next action**: 先读 `outputs/builtin.py:154/:220`、`outputs/declarations.py:1`、`results/channels.py:22` 与 `api.py` 的暴露段，确认「谁声明、谁提供值」，再按 `TODO.csv` 第 1 步起展开；动 `outputs/builtin.py` 前先确认 p3-05 已落地。

---

## 交付物

| 文件 | 性质 |
|---|---|
| `src/suspension_multibody/signal_bus.py` | **新增**：`MeasurementChannel` / `ActuatorChannel` / `SignalBus` / `open_bus` |
| `src/suspension_multibody/__init__.py` | 暴露 `open_bus`（`_PUBLIC_NAMES` 与 `__all__`，`__all__` 18 → 19） |
| `tests/api/test_signal_bus.py` | **新增**，13 用例 |

## 四条判据的落点

| `EPIC.md:275` 判据 | 证据 | 结果 |
|---|---|---|
| (a) 读测点（轮速 + 车身加速度各 ≥1）与写执行器（可变阻尼 + 电机力矩各 ≥1） | `raw/bus_io.md` | 3 个读通道 + 2 个写通道，各带断言 |
| (b) 测点与结果文档逐项对照、不重算 | `raw/point_parity.md` | 三对 `identical: True`，轮速那对**非零**（`[0,12,0]`） |
| (c) 与 `outputs/` 静态声明的关系 | `raw/outputs_relationship.md` | 声明集是**真源**，总线是**读者**；`ChannelRegistry` 与 `yaml` 未动 |
| (d) 双向读写断言 | `raw/bus_io.md` | 3 条读断言 + 2 条写断言 + 5 条拒绝路径，互不抵消 |

## 三处刻意的判断

1. **列索引不靠猜**：`body_state` 的行布局取自内核自己的写入器
   `kernel_output.cpp:24-35`（`position/quaternion/v/omega/a/alpha`）。
   首版我按印象写了 `column=13` / `column=20`，**实测对上之后才发现是错的**，
   改为 `omega=10:13`、`a=13:16`、轮胎 `4`（与 `results/kc_state.py` 同名常量一致）。
2. **读返回分量数组、不返回标量**：一个只有单分量的「通道」会让调用方以为量是标量，
   而这些量本质是向量。要单分量有一个单独的方法（`read_component`，越界即点名）。
3. **写入返回新文档**：绝不原地改调用方的 mapping——否则「我已提交的那次运行」与
   「我正在描述的那次运行」会变成同一个对象。

## 未做（明确记账）

- **未新增 `outputs/builtin.py` 的派生输出声明**。判据 (c) 要求的是**关系有定义**
  （谁是真源 + 未使用声明逐条处置），不是「必须加声明」。本行用
  `MeasurementChannel.declaration` 建立了可核账的对接，并给出
  `measurement_channels_without_declaration()`；**没有**为了「看起来接上了」而
  往声明表里塞条目——那会让声明集从「真源」退化成「跟着代码补的表」。
- **未改 `simulation/backend.py`**（内核提交唯一归属）；`signal_bus.py` 不 import
  kernel/solver/native，可由 `legacy_surface_gate` 与 `test_import_boundaries` 验证。
- **未跑** `tests/architecture` 整目录、`tests/adams`、`case_parity_check.py`、
  `kc_perf_gate.py`（归 Epic 收尾 p5-06）。未重录任何基线。
- **D2 的结论不由本行给出**：本行只做 Python 侧总线与开环读写；
  「是否需要内核单步接口」由 p5-04 实测登记。

---

## Final Summary

p5-03 **DONE**。新增 `signal_bus.py`：3 个读通道（`wheel_speed` / `body_acceleration` /
`tire_vertical_load`）与 2 个写通道（`variable_damping_L` / `motor_torque_FL`），
经 `open_bus(raw, case_document)` 打开。**读是同一数组的同一段切片**，
所以总线读数与结果文档逐位相等（实测三对 `identical: True`，轮速那对非零即 `[0,12,0]`）；
**写是文档编辑**，返回新文档、不动原件。声明集与总线的关系定义为
「声明集是真源、总线是读者」，并有 `measurement_channels_without_declaration()`
把「接了但没声明」变成可判定答案。`open_bus` 已从包根暴露，`__all__` 18 → 19。
