# 06 第 6 项：Python 本构的待删除登记（A1 修订口径）

本文件是子任务 06 第 6 项（「切换 api 元件报告到 native 事实并按可选通道状态处置 Python 本构」）的**处置登记**。结论：**保留，不删除**；本文件给出 file:line、阻断原因与解除条件。

## 1. 处置结论

第 6 项按 EPIC G3（A1 修订）与 06 SPEC 约束原文执行：

> 「在可选通道未启用期间，本构的删除**不构成本任务的完成条件**；……第 6 项按可选通道的实际状态执行：通道已启用则删除，未启用则登记为「待通道启用后删除」并明示理由。不得为删除本构而启用可选通道。」

可选通道（`element_wrench`，05 步骤 3 交付）当前状态：**默认关闭、未接线到生产路径**（05 步骤 5/7 已实测并阻断切换）。因此本项处置为**保留 + 登记**，不删除。

本次登记比「通道未启用」更强：05 步骤 7 的实测证明该通道**即使启用也不足以接管**（存在结构性缺失）。因此解除条件不是「启用通道」，而是第 4 节的契约裁决。

## 2. 待删除清单（file:line）

| 符号 | 定义位置 | 生产调用者 | 删除前置 |
|---|---|---|---|
| `LinearSpringElement.evaluate` | `elements/elastic.py:253-291` | 经 `evaluate_generalized_forces` | 见第 4 节 |
| `StaticDamperElement.evaluate` | `elements/elastic.py:310-335` | 同上 | 同上 |
| `BushingElement.evaluate` | `elements/elastic.py:443-494` | 同上 | 同上 |
| `PointWrenchElement.evaluate` | `elements/elastic.py:520-534` | 同上（注：该元件无生产调用者，见 06 步骤 1 矩阵） | 同上 |
| `VerticalTireElement.evaluate` | `elements/elastic.py:546-560` | 同上 | 同上 |
| `AntiRollBarElement.evaluate` | `elements/elastic.py:573-592` | 同上 | 同上 |
| `BumpStopElement.evaluate` | `elements/elastic.py:607-644` | 同上 | 同上 |
| `GravityElement.evaluate` | `elements/elastic.py:655-659` | 同上 | 同上 |
| `evaluate_generalized_forces`（力汇总） | `elements/assembly.py:24-54`（`__all__:21`） | **唯一生产入口 `api.py:47,742`** | 同上 |
| `ForceEvaluation` | `elements/base.py:10-19` | `elements/assembly.py:19,29,35,41,42`；各 `evaluate` 返回 | 同上 |

消费端（**不属**待删除清单，属生产者，随第 6 项一起不得改动取值来源）：

- `api._collect_element_results`：`api.py:737-777`；本构调用 `:742`；`ComponentLoad` 构造 `:750-758`；`BushingResult` 构造 `:762-777`；两个调用点 `api.py:453`（`_run_k`）与 `:559`（`_run_c`）。

## 3. 阻断原因（05 步骤 7 实测，file:line 证据）

| 类别 | 阻断内容 | 证据 |
|---|---|---|
| ① **固定体端结构性缺失（决定性）** | native 记录通道无法承载焊接/接地体端的力元反力：`add_force_on_body` 与 `add_torque_on_body` 在固定体上**直接早退**，早退发生在记录调用之前 | `packages/suspension_kernel/cpp/src/element/assembly_primitives.cpp:16`（`if (body < 0 || model.bodies[body].fixed) return {};`）、`:33`；实测 native 收到 `chassis` 的 16 行**全部** all-NaN，而同一样本 Python 有 8 条非零 `chassis` 力旋量 |
| ② **K 模式无元件事实** | K 模式模型文档不声明 `elements`，通道只有 external 行且力/力矩恒为 0，不可能提供 `ComponentLoad` 所需事实 | `cases/kc_quasi_static/contract.py:118-120`；实测 K 模式 shape `(2,10,13)`、仅 `{7:10}`、`force=[0,0,0]` |
| ③ **力矩参考点语义差** | native 力矩绕接收体原点，Python 力矩绕世界原点；直接接管会在车桥尺度产生 0.19–0.50 N·m 的力矩误差 | `cpp/include/mb_config/element_wrench.hpp:18-21` vs `elements/elastic.py:226-227`；实测未对齐时差 0.501792 N·m，对齐后差 2.195e-10 N·m |

完整登记与可复现探针：`../20260921-05-native-facts/raw/step7_law_difference_registration.md`、同目录 `step7_law_difference_probe.py` / `.log`；逐通道验收：同目录 `step6_channel_acceptance.md`。

**正向结论（重要）**：对齐参考点与单位后，两侧力差 `7.638e-10 N`、力矩差 `2.195e-10 N·m`——即**力律本身等价**，未发现第二套物理定义。阻断的原因是「事实通道不足以承载全部用户可见字段」，不是「力律不一致」。

## 4. 解除条件

下列任一事项须先被单独裁决并实施，在此之前不得删除本节第 2 项的清单，也不得把 `api.py` 的元件报告取值切到 native：

1. **固定体端事实口径**：裁决要么让记录通道记录「本应施加到固定体的反作用力旋量」（当前被 `assembly_primitives.cpp:16,33` 丢弃），要么在契约层明确声明「固定体端无载荷事实」并相应改变报告层的输出语义（`ComponentLoad.endpoint` 的含义）。EPIC 明令不得缺字段填零，故不能在报告层静默补零或静默丢弃。
2. **K 模式口径**：裁决 K 模式的元件报告是否本就不应存在（K 是纯运动学、无元件施力）。若是，须在报告层与文档中明确，而不是依赖一个结构上为空的可选通道。
3. **力矩参考点契约**：在契约中明确力矩参考点口径，并实现两侧一致（平移或声明），否则接管后力矩会以 `cross(r, f)` 量级出错。

## 5. 本项未做（不伪造达成）

- 未把 `api.py` 的元件报告取值来源切到 native。
- 未删除、未禁用任何 `evaluate` / 力汇总 / `ForceEvaluation`。
- 未为「删除本构」而启用可选通道（那会改变默认路径 artifact 字节）。
- 未在 report 侧复算本构（07 的边界，且本项未建 report）。
