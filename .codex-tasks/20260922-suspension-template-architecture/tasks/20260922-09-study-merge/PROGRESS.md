- 任务：合并 kc_quasi_static 与 axle_dynamic 为准静态/动态 study
- 形态：single-full（Epic 子任务）
- 进度：10/10 步骤 DONE
- 当前：`studies/` 三层已落（声明 / 统一装配入口 / 两 schema 桥接）；kc 准备层改经统一入口；轴侧轮胎质量落点补齐。
- 文件：`.codex-tasks/20260922-suspension-template-architecture/tasks/20260922-09-study-merge/`
- 验证：`tests/studies` 23 passed、`tests/cases` 88 passed、全量 997 passed／1 skipped／1 xfailed；`kc_parity_check --check` 0；`case_parity_check` 8 families accepted；`dynamic_hash_sentinel --check` 26 artifact 逐字节一致；`--strict --final` 0；ruff/ty 全树通过；`tests/data` 与 `layering_baseline.json` **无 diff**。

## 用户裁决（本轮确认，决定了本步的口径）

SPEC 要求「kc 从无轮胎变为有轮胎（垂向激活）」并据此**重录 `kc_baseline`**。但 `scripts/kc_parity_check.py` 的 docstring 明确：C 快照由**已退役的 Python 求解器**产出并**冻结**，是判定内核实现的 oracle，且 **`--record` 已随该求解器移除**——重录等于用被测实现重新推导 oracle。用户裁决：

> **与 05 一致：统一装配入口 + study 选择 + 垂向退化能力落地并有测试，但 `kc_quasi_static` 家族的默认发射保持无轮胎，冻结 oracle 继续有效、不重录任何基线。**

因此 TODO 8（重录 kc_baseline 与 perf 基线）**按裁决不做**，并作为登记的口径偏离而非遗漏。全部基线文件零改动，三个数值门照旧通过。

## 交付物

- `studies/study.py`：`StudySpec` 声明两个 study 的 `time_semantics`（`independent_equilibria` / `integrated`）与 `tire_activation`（`vertical_only` / `full`）；`resolve_tire_activation` 对 `fiala`/`pac2002`/`native_brush` 三力律都接受，且**力律不改变激活**（D1：准静态是同一个力律的退化，不是第二条力律）。
- `studies/assembly.py`：`build_study_assembly(model, study=..., mode=...)`——**唯一的装配入口**。接受 `FrontAxleModel` 或**已构建的** `FrontAxleAssembly`；后者是关键：调用方可以装配一次、把同一个对象交给两个 study，于是「同一模型两 study」是 `is` 级别可断言的事实，而不是两次独立装配后的数值相似。mode（K/C）属于总成而非 study，所以两 study 都可用 K 或 C。
- `studies/bridge.py`：`FrontAxleAssembly` → `AxleDynamicsModel` 的**唯一**转换点。桥接前不存在这条通道（动态家族用独立的合成模型），这正是 SPEC 说的「两套装配路径」的实处。两条规则使其可复核：**派生而非重述**（只有总成里有的东西能进动态模型）；**无法表达的构造点名拒绝**（静默丢约束会得到一个能解但错的模型）。

## 垂向退化的实现与依据

- `bridge._vertical_tire` 把 `VerticalTireElement` 读成 SI 轮胎：垂向刚度 N/mm→N/m（实测 `200 → 200000`）、半径 mm→m（`300 → 0.3`）。
- 侧向/纵向摩擦与刷子项是**占位值**：内核要求它们非零，而在**静平衡零滑移**下它们不产生力。实测动态解的轮胎状态列显示，准静态路径下侧向/纵向相关列的量级为 `1e-11` 以下（数值零），而垂向列是 `5.1e3` 量级。**未新增任何内核力律**。
- 实测：桥接后 `tire_L`/`tire_R` 正确落到 `upright_L`/`upright_R`，`model_kind=native_brush`。

## 轴侧轮胎质量落点（审核 B4）

- `AxleTire` 新增 `mass_kg`（默认 0）与 `inertia_kg_m2`（默认 None）。
- **两个字段都是 `exclude=True`**。这不是风格选择：`dynamic_hash_sentinel` 哈希的是 `model.model_dump(mode="json")`，我最初没加 `exclude` 时**全部 26 个 artifact 立刻漂移**（连 `braking`/`driving` 这类与轮胎质量无关的用例都漂移）。加 `exclude=True` 后门禁恢复到逐字节一致。这是本步最重要的一个自我纠正——门禁抓到了它。
- `cases/axle_dynamic.py::_tire_entry` 仅在 `mass_kg > 0` 时发射 `mass`（并在有 `inertia_kg_m2` 时发射 `inertia`），所以**不声明质量的文档逐字节不变**，既有基线全部继续有效。

## 与 SPEC 的偏离

1. **不重录基线**（见上，用户裁决）。
2. **动态 study 不产出 K/C 契约文档**：`study_model_document` 对动态 study 显式报错并指向 `bridge.axle_dynamics_model`，因为两份文档（毫米 K/C 契约 vs SI 多体模型）不可互换。SPEC 的验收 4（两 study 结果可对照）由「同一总成对象 + 各自的结果层」满足，而非靠强行统一文档格式。

## 未闭合项

- **准静态 study 尚未端到端跑通带轮胎的 kc 求解**：kc 家族默认不发射轮胎（按裁决），所以「垂向退化的轮胎」目前在**桥接与模型层**被完整验证（含 mm→m 换算与体挂点），但没有一条 kc 求解路径消费它。解除条件：若将来需要 kc 真跑轮胎，必须先由用户裁定接受 oracle 重录的代价。
