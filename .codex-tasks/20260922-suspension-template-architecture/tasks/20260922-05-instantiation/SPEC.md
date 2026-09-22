# 子任务 05：模板实例化与 K/C 列激活

## 目标

让「同一个模板，K 模式与 C 模式只差激活列」成为事实，并支持装配后任意切换。

1. 建立**模板实例化**：`template.instantiate(model, mode="K"|"C", properties=...)` 产出子系统。
2. 实现 **K/C 列激活**：模板中每个 `ConnectionDefinition` 可同时带 joint 列与 bushing 列，`mode` 决定激活哪一列。**激活规则（按实测，不得按字面简化）**：某一列存在才激活该列；**只有 joint 列、没有 bushing 列的点（臂外点、拉杆两端、齿条导轨）在两个模式下都保留其 joint**——C 模式实测有 9 个 joint（外点 4 + 拉杆 4 + rack 导轨 1），必须保留，不得因「C 模式只激活衬套」而丢弃。同理只有 bushing 列的点在两模式下都保留其衬套。
3. 实现**装配后切换**：`subsystem.with_mode("C")` 返回切换后的子系统，几何、体身份、连接点位置、驱动坐标定义**完全不变**，只有激活列变；`with_mode` 对已是该模式的输入幂等。
4. 用模板属性取代 C 模式的零刚度占位衬套：C 模式激活的 `bushing` 列带真实刚度/阻尼（来自模板默认属性或属性引用），**不再是 `stiffness=np.zeros((6,6))`**。

## 非目标

- 不实现属性文件的加载与解析（06）——本步只消费模板自带的默认属性或已解析的属性对象。
- 不抽取四类子系统的边界（04 已做）；本步在 04 的子系统之上做实例化。
- 不实现输出求值（07）。
- 不改 K/C 之外的拓扑语义；`symmetric_proxy`/`explicit` 两条路径的几何生成规则不变。

## 约束

- **核心断言（防第二套实现）**：`template.instantiate(model, mode="C")` 与 `subsystem_with_mode_K.with_mode("C")` 必须**逐位一致**。若做不到，说明"切换"是第二套代码路径，本步不算达成。
- **本步会改变 kc 的 C 模式文档内容**（占位衬套从零刚度变为真实属性），因此 `kc_baseline/` 必然失效。这是 EPIC 中少数**预期需要重录**的步骤之一：
  - 重录前必须先建立对照证据：零刚度与真实属性的差异定量可解释（说明这是模型改变而非数值噪声）。
  - 重录必须在 `PROGRESS.md` 逐项登记：基线文件 + 导致重录的步骤 + 重录前后的值 + 判定依据。
  - 禁止"先改基线让门变绿"。
- **K 模式的行为必须逐字节不变**（K 列激活结果与现役 `build_front_axle(model, "K")` 一致）——K 模式是现有 `kc_baseline/k_states.json` 的基准，不应因此步而变。
- 不引入新依赖。
- `with_mode` 不得触碰 `preparation/assembly/types.py` 中与 04 冲突的部分（04 已完成后再动）。

## 范围与文件归属

- 可写：
  - `packages/suspension_multibody/src/suspension_multibody/templates/**`（实例化逻辑）
  - 新增 `packages/suspension_multibody/src/suspension_multibody/subsystems/**`（04 建立的子系统包，本步加实例化与切换）
  - `packages/suspension_multibody/src/suspension_multibody/preparation/assembly/front_axle.py`（接线：由模板驱动副/衬套产出）
  - 新增测试 `packages/suspension_multibody/tests/instantiation/**`
  - 基线文件 `packages/suspension_multibody/tests/data/kc_baseline/**`（仅 C 模式相关，且必须登记）
- 只读：`cases/**`、`schema/**`、02 的 `joints/`、父计划文件。
- 不写：`outputs/`（07）、`report/`（07）、`preparation/vehicle_dynamic.py` 的轮胎/质量部分（08、09）、父级计划文件（归主代理）。

## 依赖

- 前置：04（四类子系统已拆出，实例化有落点）。
- 后续：06（属性文件——本步用模板默认属性，06 把它换成外部文件）、09（study 合并）。

## 验收标准

1. 同一模板分别以 `mode="K"` 与 `mode="C"` 实例化，**几何与结构完全一致**：体集合与顺序、`points`、`hardpoints`、连接表、驱动坐标定义逐项相同；**只有** joint 与 bushing 集合不同。有测试逐项断言。
2. `with_mode("C")` 的结果与直接 `instantiate(mode="C")` **逐位一致**（关键断言）。
3. `with_mode` 幂等：`x.with_mode("K").with_mode("K")` 与 `x.with_mode("K")` 一致。
4. K 模式逐字节不变：K 列激活产出的约束集合与现役 `build_front_axle(model, "K")` 一致（13 约束、0 衬套）。
5. C 模式的衬套刚度**不再全为零**：有测试断言 C 模式激活的衬套刚度来自模板属性且非全零，并给出与旧零刚度版本的差异定量。
6. 基线重录有登记：`PROGRESS.md` 中记明 `kc_baseline/`（C 模式部分）的重录前后值与判定依据；K 模式基线不应变化。
7. 门禁：`dynamic_hash_sentinel`（axle 侧 26 artifact）若不受影响须保持绿；`case_parity_check.py` 8 family 需重新通过；`--strict --final` 保持绿。

## 验证协议

1. 实例化落地后：跑「K/C 几何一致」与「with_mode 等价于直接实例化」两个关键测试。
2. 接线后：跑 K 模式对照（须逐字节不变），确认未波及 K 基线。
3. C 模式属性落地后：先跑差异定量（零刚度 vs 真实属性），再重录 `kc_baseline/` 的 C 部分并登记。
4. 重录后：跑 `build_axle_native.py` + `kc_parity_check.py --check` + `case_parity_check.py` + `dynamic_hash_sentinel.py --check`。
5. 收尾：三套 pytest、ruff、ty、`--strict --final`、`git diff --check`。

**若 K 模式基线也发生变化，立即停止**——那说明本步误改了 K 列语义，不是预期的 C 模式重录范围。
