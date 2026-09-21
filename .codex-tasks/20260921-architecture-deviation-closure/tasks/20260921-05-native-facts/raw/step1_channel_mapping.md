# 05 步骤 1：Python 元件报告字段 ↔ native 通道逐项对照表（冻结）

本表是 05 后续步骤（凝聚、C++ 缺口补齐、decoder 接线）的前置契约。左侧为 Python 报告侧用户可见字段（`schema/result.py`），右侧为 native 输出通道（`cpp/include/mb_input/types.hpp`、`cpp/src/abi/kernel_contract_run.cpp`）的现状。

判定列含义：
- **覆盖**：native 已有等价事实，可直接读取或经无歧义换算得到。
- **缺口**：native 无对应输出，须按 EPIC「向后兼容扩展」补通道。
- **非 native 事实**：该字段本就不来自求解结果（作者侧输入或报告元数据），不属缺口。

## 1. ComponentLoad（元件载荷，`schema/result.py:68`）

| Python 字段 | 语义（用户可见） | native 现状 | 判定 |
|---|---|---|---|
| `state_id` | 该载荷属于哪个工况样本 | 每条样本一行；样本序号即 state_id 后缀 | 非 native 事实（报告元数据） |
| `component` | 元件名 | `AxleRunResult.spring_names` / `bushing_names` / `anti_roll_bar_names`（`result.py:258-262`） | 覆盖（名称通道已有） |
| `endpoint` | 元件作用的 body（一端） | **无任何按 body 索引的力元输出** | **缺口（关键）** |
| `global_load` | 该 body 上的 6 维世界系力旋量 | **无**。仅 `constraint_wrench` 按 body 输出世界力旋量，但那是理想约束（joint/driven），不是力元 | **缺口（关键）** |
| `local_load` | 投影到该 body 本体系 | **无**。bushing 有 body_b 端**局部**力旋量（`bushing_output[6:12]`），但仅限 bushing 且只有 b 端 | **缺口（关键）** |
| `utilization` / `over_limit` | 元件用量与超限标志 | 无 | 非 native 事实（报告派生，可由载荷+限值算） |

**结论（关键）**：`ComponentLoad` 的 `endpoint` / `global_load` / `local_load` 三项在 native 完全没有对应输出。现状由 Python 本构计算：`api.py:742` 调 `evaluate_generalized_forces`（`elements/assembly.py:28`）→ 逐元件 `evaluate`（`elements/elastic.py`）→ `body_wrenches_global` → `api.py:750-758` 构造。

## 2. BushingResult（衬套，`schema/result.py:78`）

| Python 字段 | 语义 | native 现状 | 判定 |
|---|---|---|---|
| `bushing` | 衬套名 | `bushing_names` | 覆盖 |
| `deformation` | 本体系 6 维变形（3 平移 m + 3 转动 rad） | `bushing_output[0:6]`（`bushing.cpp:103-112`），体 a 挂载的衬套局部系（`assembly_primitives.cpp:54,60-72`） | 覆盖（坐标系需核对，见 §6-R1） |
| `load` | body_b 端局部力旋量 | `bushing_output[6:12]`（`types.hpp:546`：local wrench on body_b） | 覆盖 |
| `strain_energy` | 应变能 0.5·dᵀK·d | 无单元件能量列；`energy` 块有 `bushing_elastic_energy_j` 聚合值（`result.py:183-184` ENERGY_COLUMNS） | 部分覆盖（聚合有、逐件无；可由 native 变形 + K 复算，但那是"报告复算本构"，须裁决） |
| `stiffness_id` | 刚度标识 | 无（作者侧数据） | 非 native 事实 |
| `zero_load_pose` | 零载位姿 | 无（作者侧数据） | 非 native 事实 |

现状：`api.py:762-777` 用 `element.deformation(state)`（Python 本构）+ `element.stiffness` 直接算。

## 3. 各元件的 native 标量通道对照

| Python 元件（`elements/elastic.py`） | 对应 native 通道 | 覆盖度 |
|---|---|---|
| `LinearSpringElement`（:231） | `spring_output` 7 列（`spring.cpp:169-180`） | 标量覆盖：length / length_rate / elastic / damping / stop×2 / total axial force。**但缺作用点与轴向量**，无法还原 `global_load` |
| `StaticDamperElement`（:293） | 并入 `spring_output` 的 damping 列（native spring 含阻尼） | 标量覆盖；缺力旋量 |
| `BushingElement`（:337） | `bushing_output` 12 列 | 覆盖（body_b 端）；**缺 body_a 端反作用力旋量** |
| `AntiRollBarElement`（:562） | `anti_roll_output` 3 列（`anti_roll.cpp:65-71`） | 标量覆盖（angle/rate/torque on body_b）；缺力旋量 |
| `BumpStopElement`（elastic.py，限位） | 并入 `spring_output` 的 compression/rebound stop 列（`spring.cpp:84-92,120-128`） | 标量覆盖；**缺独立元件身份**（native 把它并在 spring 通道里，Python 是独立元件） |
| `VerticalTireElement`（:536） | `tire_output` 41 列（`result.py:67-124`） | 覆盖（轮胎事实最完整） |
| `GravityElement` | `energy`/力总线内（无独立通道） | 非报告项 |
| `PointWrenchElement`（:496） | 无独立通道 | 无生产调用者（已由调用面测绘确认） |

**K/C 两模式差异**：C 模式才装配 `BushingElement`（`front_axle.py:360-377`，`mode == "C"`）；K 模式无衬套元件，故 `bushing_output` 在 K 模式下为空块。对照表覆盖两模式时，K 模式的元件事实仅含 spring/stop/tire。

## 4. 能量与 active 状态

| 项目 | Python 现状 | native 现状 | 判定 |
|---|---|---|---|
| 元件能量 | `ForceEvaluation.energy`（逐元件，`elements/base.py:14`） | `energy` 块仅聚合列：spring/stop/bushing/anti_roll/tire_normal/tire_brush（`result.py:183-189` ENERGY_COLUMNS） | 部分覆盖（聚合有、逐件无） |
| active 状态 | `ForceEvaluation.active`（`base.py:17`）、`event`（:18） | tire 有 `active` 列（`TIRE_OUTPUT_COLUMNS[0]`）；力元无 active 列 | 部分覆盖 |

## 5. 缺口汇总（须 05 步骤 3 处理）

按 EPIC「新增契约字段只在证明确有输出缺口时采用向后兼容扩展，记录版本策略；ABI 签名及现有导出不变」：

- **G1（关键）**：按 body 的力元力旋量输出通道（世界系 6 维）。覆盖 `ComponentLoad.global_load` / `endpoint`。
- **G2（关键）**：元件局部力旋量（本体系 6 维，涵盖 body_a 与 body_b 两端）。覆盖 `ComponentLoad.local_load` 与 bushing body_a 端反作用。
- **G3**：逐元件能量列（`BushingResult.strain_energy` 等）。
- **G4**：力元 active 状态列。
- **G5**：bushing 的 body_a 端力矩臂项（native 现只记 b 端，`bushing.cpp:74-78` 的反作用含臂项未输出）。

非缺口（不属 native 事实，保持 Python 侧）：`state_id`、`component`、`stiffness_id`、`zero_load_pose`、`utilization`、`over_limit`。

## 6. 需裁决的歧义

- **R1**：`BushingResult.deformation` 现有 Python 值来自 `BushingElement.deformation(state)`（本体系），与 native `bushing_output[0:6]` 的坐标系是否逐位一致，须在步骤 6 通道级容差验收中确认；若不一致须先登记差异再切换。
- **R2**：`strain_energy` 若要从 native 读，须新增逐件能量列（G3）；否则保持 Python 由 native 变形 + 作者侧 K 复算——后者属"report 复算本构"边界，须与 07 的 report 边界规则一致。
- **R3**：`BumpStopElement` 在 native 并入了 spring 通道，元件身份丢失（`component` 名称无法逐一对齐）。要么 native 保留独立身份（G1/G2 通道按元件类型分别命名），要么 Python 报告侧不再把限位报成独立元件。

## 7. 数值门约束（决定通道形态）

- `dynamic_hash_sentinel.py` 比较 `arrays.npz` 字节，覆盖 **axle** 合成模型 26 个 artifact（`run_axle_dynamics_acceptance.py:70` `build_axle_model`）。
- `case_parity_check.py:398-410` 的**整车门也是字节级 sha256**（`_VEHICLE_LEDGERS`）。
- 因此：native 新增通道若进入 `arrays.npz`（`io/artifacts.py:402-470` 的 `arrays` 字典即 npz 内容），两个门都会失败。

按用户裁决（方案 A：向后兼容可选扩展），新增通道设计为**默认关闭、显式启用**，使默认路径的 artifact 字节不变，两个门保持绿；Python 侧 native 事实通道仅在显式请求时启用。契约版本允许 1→2，ABI 七符号与 ABI 版本号（15/30/1）不动。

## 8. 凝聚迁移的现状与裁决（补充）

- 现状：Python `build_vehicle` 先凝聚（`model/vehicle.py:188` 调 `_condense_welded_bodies`），再把凝聚后模型送 native（`preparation/vehicle_dynamic.py:222`）。native 侧已支持 `kind="fixed"`（`preparation/vehicle_dynamic.py:629` → `build_model.cpp:89-123`，joint 表 `"fixed"`=6 行，`contract_registry.cpp:24`）。
- 存在现成开关 `SUSPENSION_MULTIBODY_CONDENSE_WELDS=0`（`vehicle.py:245`）可关闭凝聚。
- 实测（`SUSPENSION_MULTIBODY_CONDENSE_WELDS=0` 跑 tests/）：`tests/vehicle/test_native_vehicle.py:325` 断言 body 数 22 实际 23（差一个被凝聚掉的 body）。即"不凝聚"路径**当前不被测试支持**，且会改变 native 收到的 body 集合 → 改变 `states` 数组 → `_VEHICLE_LEDGERS` 字节门失败。
- 用户裁决（方案 A）：Python 不再凝聚，把 weld 作为 fixed 约束送 native，并验证与现凝聚结果等价。此裁决直接与上述字节门冲突，须在步骤 2 前先解决门策略（同 §7 的可选通道思路，或经用户再次确认后修订门基线）。
