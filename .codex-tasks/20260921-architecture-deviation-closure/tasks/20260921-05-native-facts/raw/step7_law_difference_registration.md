# 05 步骤 7：力律差异登记与切换阻断

本文件是 05 步骤 7 的交付：Python 本构与 native 力元力旋量通道的逐项差异登记，以及**阻断切换**的结论与依据。可复现探针：`raw/step7_law_difference_probe.py`，原始输出 `raw/step7_law_difference_probe.log`（两条命令均实测执行，见文末）。

## 1. 比对方法与可比性

探针在**同一状态**上评估两侧，排除收敛与位姿来源的伪差异：

- native 侧：`monkeypatch`/环境变量 `SUSPENSION_KERNEL_ELEMENT_WRENCH_OUTPUT=1`，跑 C 模式 K&C 工况，取 `element_wrench` 块的样本 0。
- Python 侧：用同一次 native 运行的样本 0 重建报告态（`api._rigid_state`，`api.py:591-609`），再调 `evaluate_generalized_forces`（`elements/assembly.py:24-54`）。
- 单位：native 为 m/N（力矩 N·m），Python 报告态为 mm/N（力矩 N·mm），故出力按 1 折算、力矩按 1/1000 折算。折算本身的相对误差约 1e-16，不构成下表差异的量级来源。
- 观测到的最小差异为 2.1e-15（浮点噪声量级），最大差异为 0.5018 N·m；两者相差 14 个数量级，说明下表差异是**结构性**而非数值噪声。

## 2. 差异四类（实测）

C 模式（C 是唯一装配衬套元件的模式，`model/front_axle.py:360-377`；K 模式模型文档的 `elements` 为空，`cases/kc_quasi_static/contract.py:118-120`）：

| 类别 | 现象 | 实测数值 | 判定 |
|---|---|---|---|
| ① 数值噪声 | 右侧（`_R` 侧）衬套在两端均为零载荷，两侧都算出零 | `|dM|` 2.1e-15 … 2.6e-15 | 等价（噪声量级） |
| ② 杠杆臂端的力矩差 | 左侧承载衬套的 body b 端力矩不一致 | `|dM|` 0.1887 … 0.5018 N·m，相对 **0.108% … 0.287%**；同端力差 1.8e-06 N | **不等价** |
| ③ 固定体端缺失 | native 对焊接/固定体（`chassis`）端的行全为 NaN（力与力矩 6 列），Python 侧给出有限值 | native 接收到 `chassis` 的 16 行**全部** all-NaN；同一样本 Python 有 **8** 个衬套端给出非零 `chassis` 力旋量 | **不等价（结构性缺失）** |
| ④ K 模式无元件事实 | K 模式通道只有 type=7 行且力与力矩均为 0.0 | shape `(2, 10, 13)`、finite rows `{7: 10}`、`force=[0,0,0]`、`moment=[0,0,0]` | **无可用元件事实** |

C 模式逐元素实测（节选，完整表见日志）：

```text
element                    end body              |dF|max     |dM|max   |dM|/|M|
upper_arm_0_L                0 upper_arm_L   1.32578e-06    0.501793    0.00287
lower_arm_1_L                0 lower_arm_L   1.80052e-06    0.188732    0.00108
uca_bushing_L_inner_front    0 upper_arm_L             0           0          0
upper_arm_0_R                0 upper_arm_R             0 2.61605e-15        116
```

（相对列在参考力矩接近零时被放大，属除零放大效应；判定以绝对差与承载端的相对差为准。）

## 3. 根因（file:line）

1. **固定体早退（类别 ③，决定性）**：`packages/suspension_kernel/cpp/src/element/assembly_primitives.cpp:16` —— `add_force_on_body` 首行即 `if (body < 0 || model.bodies[body].fixed) return {};`；`add_torque_on_body`（同文件 `:33`）同样早退。`element_wrench.hpp:103` 的契约注释也承认「固定体保留行但力列为 NaN，符合契约」。而 Python 侧无条件为两端构造力旋量（`elements/elastic.py:462-467`，`BushingElement.evaluate` 的 `wrenches` 字典含 `body_a` 与 `body_b` 两端）。因此**焊接/固定体端**的 `ComponentLoad.endpoint` / `global_load` / `local_load` 在 native 通道**没有事实来源**，不是精度问题而是结构性缺失。
2. **力矩构造差异（类别 ②）**：Python 的力矩由 `moment_global = pose_a.rotation @ generalized[3:]` 给出，其中 `generalized = -elastic + preload`、`elastic = stiffness @ deformation`、`deformation` 的第二分量来自 `rotational_deformation(relative.quaternion)`（`elements/elastic.py:446-467`，`rotational_deformation` 在 `:409`）。native 的力矩在 `add_force_on_body`/`add_torque_on_body`（`assembly_primitives.cpp:12-40`）按 `arm × f_world` 累积，其中 `arm = rotate(state.q[body], point_local)` 用的是**当前**位姿的杠杆臂。两者的旋转变形提取与杠杆处理不同，导致承载端力矩出现 0.1%–0.29% 的差异。
3. **K 模式无元件（类别 ④）**：`cases/kc_quasi_static/contract.py:64-151` 的 `model_document` 仅在 `drive_wheels=False`（C）时发出 `elements`（`:118-120`），K 模式模型的 `elements` 为空；K 本身是纯运动学求解，力元不被施加。因此该通道在 K 模式下**不可能**提供 `ComponentLoad` 所需事实。
4. **Ki 模式/固定体的历史缺口登记一致**：`VALIDATION.md:290-300` 的「报告契约缺口」表已独立登记名称/两端/坐标/作用点/符号/单位/能量/active 与凝聚身份等缺口；本步骤的实测为该表补充了「固定体端结构性缺失」与「力矩构造差异」两条量化证据。

## 4. 结论：阻断切换（不接线到生产路径）
4. **与历史缺口登记一致**：`VALIDATION.md:290-300` 的「报告契约缺口」表已独立登记名称/两端/坐标/作用点/符号/单位/能量/active 与凝聚身份等缺口；本步骤的实测为该表补充了「固定体端结构性缺失」与「力矩构造差异」两条量化证据。
按 EPIC G3（A1 修订）与 05 SPEC 步骤 3 的约束——「输出接管若揭示原 Python 与 native 力律差异，必须登记差异并阻断切换，在既有物理定义下消除，不得保留第二套力律」——本步骤结论如下：

1. **不切换**。`api._collect_element_results`（`api.py:737-777`）继续由 Python 本构（`:742`，`evaluate_generalized_forces`）产出 `ComponentLoad` / `BushingResult`。仅新增只读解码面（步骤 5）供显式启用时消费。
2. **阻断理由成立且已量化**：类别 ③ 是结构性缺失（无事实来源），类别 ② 是 0.1%–0.29% 的构造差异（远超通道级容差应承载的范围），类别 ④ 使 K 模式完全无 native 元件事实。任一单独成立即足以阻断。
3. **不保留第二套生产力律**：native 通道保持**默认关闭且未接线**（`element_wrench` 仅在显式设置 `SUSPENSION_KERNEL_ELEMENT_WRENCH_OUTPUT` 时存在）；生产路径仍只有 Python 一套力律，不构成「为了报告保留的第二套力律」。
4. **不通过调容差或重录基线掩盖**：未修改任何容差，未触碰 `dynamic_hash_baseline.json` / `kc_*_baseline` / `vehicle_dynamics_baseline`。
5. **对 06 的直接影响**：`elements/elastic.py` 的 `evaluate` 与 `elements/assembly.py` 的力汇总**不得删除**（删除条件未满足）；06 须按 A1 登记为「待条件满足后删除」，并写明本次实测的阻断证据。

## 5. 消除条件（解除阻断所需）

要让 native 结果接管元件事实，必须先在同一套物理定义下消除上述差异，至少包括：

1. **固定体端事实**：为固定体端的力元反力建立明确输出口径——要么让记录通道记录「本应施加到固定体的反作用力旋量」（当前被 `assembly_primitives.cpp:16,33` 的早退丢弃），要么在契约层明确声明「固定体端无载荷事实」并由报告层据此改变输出语义。二者都是契约决定，须显式裁决，不能在报告层填零或静默丢弃（EPIC 明令不得缺字段填零）。
2. **力矩构造对齐**：判定 `elastic.py:446-467` 的旋转变形/杠杆构造与 native `arm × f`（`assembly_primitives.cpp:12-40`）哪个是权威物理定义，然后消除另一侧；不得两侧并存。
3. **K 模式**：裁决 K 模式的元件报告是否本就不应存在（K 无元件施加力）——若是，须在报告层与文档中明确，而不是依赖一个结构上为空的可选通道。

在上述任一项被裁决并实施前，本子任务交付的是：**差异登记 + 只读解码面 + 默认关闭 + 生产路径不变**。

## 6. 本步骤未做（明确记录，不伪造达成）

- 未把 `api.py` 的元件报告取值来源切到 native（按上文阻断）。
- 未删除、未禁用 Python 本构与力汇总（删除条件未满足）。
- 未修改任何容差、未重录任何基线、未新增 ABI 导出或契约字段。

## 7. 实测命令与退出码

```text
uv run --package suspension-multibody python .codex-tasks/20260921-architecture-deviation-closure/tasks/20260921-05-native-facts/raw/step7_law_difference_probe.py
```

退出码 `0`；输出归档 `raw/step7_law_difference_probe.log`。关键输出行：

```text
mode C: element_wrench shape: (2, 42, 13)   contract_version: 2
        finite rows per type code: {2: 16, 7: 10}
        worst force diff  = 1.80052e-06 N
        worst moment diff = 0.501793 N*m (max relative 0.00287 on a loaded end)
        native rows receiving 'chassis': 16, of which all-NaN: 16
        python evaluations reporting a non-zero wrench on 'chassis': 8
mode K: element_wrench shape: (2, 10, 13)   contract_version: 2
        finite rows per type code: {7: 10}   force=[0,0,0] moment=[0,0,0]
```
