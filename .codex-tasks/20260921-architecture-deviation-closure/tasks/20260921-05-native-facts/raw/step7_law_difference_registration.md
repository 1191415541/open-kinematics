# 05 步骤 7：力律差异登记与切换阻断

本文件是 05 步骤 7 的交付：Python 本构与 native 力元力旋量通道的逐项差异登记，以及**阻断切换**的结论与依据。可复现探针：`raw/step7_law_difference_probe.py`，原始输出 `raw/step7_law_difference_probe.log`。

> **2026-09-22 修订（独立复审触发）**：本文件初版把「承载端力矩差 0.5018 N·m」归因为**力律构造差异**。经 `code-reviewer d1a8db9f` 复审指出并**由主代理实测确认**，该差异实为**力矩参考点语义差**（native 写「绕接收体原点」，Python 报「绕世界原点」），不是力律差异。对齐参考点后该差值消失（见第 2 节）。初版的归因已作废，冲突结论（第 4 节「阻断切换」）不变——理由见第 4 节。同时修正了探针的两处方法学缺陷（样本索引不一致、缺少参考点对齐比较）。

## 1. 比对方法与可比性（修订）

探针在**同一状态**上评估两侧，排除收敛与位姿来源的伪差异：

- native 侧：环境变量 `SUSPENSION_KERNEL_ELEMENT_WRENCH_OUTPUT=1`，跑 K&C 工况，取 `element_wrench` 块的样本 0。
- Python 侧：用**同一次** native 运行的**同一**样本重建报告态（`api._rigid_state`，`api.py:591-609`）——初版探针取块样本 0 但用 `case_body_state(0)` 的默认末样本，二者不符；现已统一为同一 `sample`。
- 单位：native 为 m/N（力矩 N·m），Python 报告态为 mm/N（力矩 N·mm），故出力按 1 折算、力矩按 1/1000 折算。
- **参考点**（初版遗漏，是初版错判的根因）：native 的力矩列是**绕接收体自身原点**（`cpp/include/mb_config/element_wrench.hpp:18-21` 原文 "world moment about the receiving body's origin"；写入见 `cpp/src/element/assembly_primitives.cpp:24` 的 `arm = rotate(state.q[body], point_local)` 与 `:36`）；Python 的 `body_wrenches_global` 力矩是**绕世界原点**（`elements/elastic.py:226-227` 的 `_point_wrench` = `cross(世界点, f)`）。探针现在**两种参考系都比**，使参考点差异绝不会被误判为力律差异。
- 单位折算本身的相对误差约 1e-16，不构成下表差异的来源。

## 2. 差异分类（修订后的实测）

C 模式（C 是唯一装配衬套元件的模式，`model/front_axle.py:360-377`；K 模式模型文档 `elements` 为空，`cases/kc_quasi_static/contract.py:118-120`）：

| 类别 | 现象 | 实测数值 | 判定 |
|---|---|---|---|
| ① **力律等价** | 承载端力旋量在**对齐参考点后**两侧一致 | 力差 `7.6e-10 N`；力矩差 `2.2e-10 N·m`（相对 `1.3e-12`） | **等价**（浮点噪声量级） |
| ② ~~力矩构造差异~~ → **参考点语义差（非力律）** | 未对齐参考点时出现差值，对齐后消失 | 未对齐 `0.5018 N·m` → 对齐后 `2.2e-10 N·m`，**降低约 2.3e9 倍** | **表示层差异，非力律差异** |
| ③ **固定体端结构性缺失** | native 对固定体（`chassis`）端的行全为 NaN（力与力矩 6 列），Python 侧给出有限值 | native 接收到 `chassis` 的 16 行**全部** all-NaN；同一样本 Python 有 **8** 个衬套端给出非零 `chassis` 力旋量 | **不等价（结构性缺失）** |
| ④ **K 模式无元件事实** | K 模式通道只有 type=7 行且力与力矩均为 0.0 | shape `(2, 10, 13)`、force+moment 行 `{7: 10}`、`force=[0,0,0]`、`moment=[0,0,0]` | **无可用元件事实** |

**类别 ② 的归因更正依据**（实测，`raw/step7_law_difference_probe.log`）：

```text
element                    end body              |dF|max     |dM|world      |dM|body  |dM|b/|M|
upper_arm_0_L                0 upper_arm_L   7.63833e-10      0.501792   2.19501e-10   1.26e-12
lower_arm_0_L                0 lower_arm_L   3.73035e-10      0.188732   1.75476e-10      1e-12
```

未对齐参考点时的差值（`0.501792`、`0.188732` N·m）与初版报告一致，说明初版的观测是真实的；错的是**归因**。对齐后差值与力差同量级（1e-10），且初版注意到的「`upper_arm_0_L` 与 `upper_arm_1_L` 的 |dM| 完全相同」现象恰是参考点差的指纹（两衬套硬点关于 x=0 对称、载荷对称 → 同一接收体、同一 `cross(r, f)`），而非力律构造差异的特征。

**类别 ③ 的根因**（未变，且经复审确认为**必然结论**）：`cpp/src/element/assembly_primitives.cpp:16` 的 `add_force_on_body` 首行 `if (body < 0 || model.bodies[body].fixed) return {};`，`:33` 的 `add_torque_on_body` 同款早退——两者都发生在 `sink->add_*`（`:24`、`:36`）**之前**，且 `chassis` 确为固定体（`model/front_axle.py:637` `RigidBody("chassis", fixed=True)`）。因此固定体端行永远收不到力/力矩，这是代码结构的必然。

**类别 ④ 的根因**（未变）：`cases/kc_quasi_static/contract.py:118-120` 的 `model_document` 仅在 `drive_wheels=False`（C）时发出 `elements`；K 是纯运动学，力元不被施加。

**补充实测（初版未列）**：本探针的两个 K&C 工况中 `torque-only` 行数均为 0（C 模式 26 条 force+moment 行、16 条 opened-only 行、0 条纯力矩行）。但这**不**代表 native 不存在纯力矩记录——native 的 `drive_brake` 全部只调 `add_torque_on_body`（`cpp/src/element/drive_brake.cpp:64,75,103,117`），故其行必然是「力矩有限、力列 NaN」的纯力矩行，而步骤 5 的解码面当前会**丢弃**这类行（见第 5 节待修项）。

## 3. 与历史缺口登记一致

`VALIDATION.md:290-300` 的「报告契约缺口」表已独立登记名称/两端/坐标/作用点/符号/单位/能量/active 与凝聚身份等缺口；本步骤的实测为该表补充了「固定体端结构性缺失」与「力矩参考点语义差」两条量化证据。

## 4. 结论：阻断切换（不接线到生产路径）

按 EPIC G3（A1 修订）与 05 SPEC 的约束——「输出接管若揭示原 Python 与 native 力律差异，必须登记差异并阻断切换，在既有物理定义下消除，不得保留第二套力律」——本步骤结论如下：

1. **不切换**。`api._collect_element_results`（`api.py:737-777`）继续由 Python 本构（`:742`，`evaluate_generalized_forces`）产出 `ComponentLoad` / `BushingResult`；仅新增只读解码面（步骤 5）供显式启用时消费。
2. **阻断理由（修订后仍成立，且理由更准确）**：类别 ③ 是**结构性缺失**——固定体端（焊接/接地端）在 native 通路**没有事实来源**，而 `ComponentLoad.endpoint` / `global_load` / `local_load` 的用户可见语义包含该端（EPIC 明令不得缺字段填零），任一单独成立即足以阻断；类别 ④ 使 K 模式完全无 native 元件事实。
3. **类别 ② 不再是阻断理由**（已更正为参考点语义差），但它**仍然构成两侧输出不可直接互换的事实**：native 力矩绕接收体原点、Python 力矩绕世界原点，接管时必须做坐标平移并在契约中明确口径，否则力矩会以 `cross(r, f)` 的量级出错（实测车桥尺度下达 0.19–0.50 N·m）。
4. **力律本身未发现差异**：对齐参考点与单位后，力与力矩均在 1e-10 量级一致，说明两侧本构在当前工况下等价。这是对「不得保留第二套力律」的正向证据。
5. **不保留第二套生产力律**：native 通道保持**默认关闭且未接线**；生产路径仍只有 Python 一套力律。
6. **不通过调容差或重录基线掩盖**：未修改任何容差，未触碰 `dynamic_hash_baseline.json` / `kc_*_baseline` / `vehicle_dynamics_baseline`。
7. **对 06 的直接影响**：`elements/elastic.py` 的 `evaluate` 与 `elements/assembly.py` 的力汇总**不得删除**（删除条件未满足）；06 须按 A1 登记为「待条件满足后删除」，并写明本次实测的阻断证据。

## 5. 待修项（本步骤登记，不掩盖）

1. **解码面丢弃纯力矩行（缺陷）**：`results/element_wrench.py:103` 的 `_APPLIED_COLUMNS = slice(0, 7)` 配 `:231` 的 `np.isnan(...).any()` 判定，要求力列 0..2 有限；而 native 的纯力矩行力列恒为 NaN（`element_wrench.hpp:128-133` 的 `add_torque` 只写 3..5），于是 type 5（drive_brake）全部行与 steering 作动器行会被**静默丢弃**，与该模块 `:33-35` 自述「力、力矩、类型码**皆**为 NaN 才不是记录」相矛盾。判定应为「力矩或力任一列有限即为记录」，或按契约改为对力列与力矩列分别判 NaN。
2. 该缺陷的解释与修复归子任务 06/08 的 `results` 面（本步骤只登记，未改动步骤 5 已交付的代码）。

## 6. 本步骤未做（明确记录，不伪造达成）

- 未把 `api.py` 的元件报告取值来源切到 native（按第 4 节阻断）。
- 未删除、未禁用 Python 本构与力汇总（删除条件未满足）。
- 未修改任何容差、未重录任何基线、未新增 ABI 导出或契约字段。

## 7. 实测命令与退出码

```text
uv run --package suspension-multibody python .codex-tasks/20260921-architecture-deviation-closure/tasks/20260921-05-native-facts/raw/step7_law_difference_probe.py
```

退出码 `0`；输出归档 `raw/step7_law_difference_probe.log`。关键输出行：

```text
mode C: element_wrench shape: (2, 42, 13)   contract_version: 2
        records per type code (force rows + torque-only rows): {2: 16, 7: 10}
        force+moment rows=26  torque-only rows=0  opened-only (no wrench) rows=16
        worst force diff  = 7.63833e-10 N
        worst moment diff, world-origin reference = 0.501792 N*m
        worst moment diff, body-origin reference  = 2.19501e-10 N*m (max relative 1.26e-12)
        native rows receiving 'chassis': 16, of which all-NaN: 16
        python evaluations reporting a non-zero wrench on 'chassis': 8
mode K: element_wrench shape: (2, 10, 13)   contract_version: 2
        records per type code: {7: 10}   force=[0,0,0] moment=[0,0,0]
```
