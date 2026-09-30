# 03b 证据：形状规则数据化 + 契约放开 + 三轴装配

- 执行时间：2026-09-29
- 范围：`connections/policy.py`（形状规则）、`assembly.schema.json`（放置/轮数/整车 required）、`authoring/documents.py`（`PLACEMENT_ROLES`）、`schema/vehicle.py`（`WheelSpec.name`）、测试

## 1. 形状规则从「写死两轴」改为数据驱动

`connections/policy.py` 的 `full_vehicle` 规则今天读作：

```python
"full_vehicle": AssemblyRule(
    kind="full_vehicle",
    role_counts={"chassis": 1, "steering": 1, "brake": 1, "drive": 1},
    min_counts={"suspension": 1},
    at_most={},
    required_placements=frozenset(),
    forbidden_roles=frozenset(),
    wheels_complete=True,
),
```

即：车的**形状**仍固定（一个车身、一个转向、一套制动、一套驱动、至少一个悬架），而**轴数与放置**交给文件——`min_counts` 是下限、`required_placements` 不再点名 `front`/`rear`。新增两处判据替代原来的「恰好两个悬架、必须在前后」：

1. 同一 `(functional_role, placement_role)` 的悬架不得重复（重复＝同一根轴被写了两遍，读者无法判断某只轮来自哪个悬架）；
2. 每个**被声明的**轮端都必须被某个悬架覆盖（原来的四角常量 `WHEEL_ENDS` 只在 `any` 这个「我覆盖全部」的拼写里用到）。

实测（原始输出）：

```text
3 axles + 6 wheels: ACCEPTED
3 axles, middle wheels unclaimed: REFUSED -> ... declares wheel end(s) ['middle_left', 'middle_right'] that no suspension accounts for
duplicate placement: REFUSED -> ... places ['suspension at front'] more than once
2 axles: ACCEPTED
no suspension: REFUSED -> ... requires at least one suspension subsystem(s), found 0
single-side: ACCEPTED
```

## 2. 契约与文档段放开

- `assembly.schema.json`：`placement_role` 由四角枚举改为「非空字符串」（轴词汇属文件自己；形状规则仍保证「同一放置最多一次」与「轮端各有归属」）；`vehicle.wheels` 的 `minItems/maxItems: 4/4` 改为 `minItems: 1`；整车 `required` 去掉 `chassis`（拖挂需要两个车身侧体，见 03d）。`vehicle_part` 仍逐字段由模型类校验，`steering` 仍必填。
- `authoring/documents.py` 的 `PLACEMENT_ROLES` 增加 `middle`/`third` 及其左右，`front`/`rear` 与 `any` 保留——只改 schema 会在 `:139` 的文档读取层先被拒（这是第二轮复核点出的前置墙）。
- `schema/vehicle.py` 的 `WheelSpec.name` 由四角 `Literal` 放宽为非空字符串。**D1 保护的四角校验没有放松**：它写在 `VehicleModel._topology`（要求恰好四个角），本次只放开「一只轮怎么命名」，既有整车模型仍必须声明原样的四角。
- `authoring/documents.py` 里一条断言旧语义的测试改为新语义：`test_full_vehicle_rejects_two_suspensions_at_one_placement`（两个前轴 → 拒绝理由是「同一放置出现两次」而不是「缺后轴」）。

## 3. 三轴装配

新增 `tests/subsystems/test_three_axle_assembly.py`（7 条）：

- 三个条目（`front`/`middle`/`rear`）装配成功，每个轴保留自己的前缀；
- 三轴形状被接受、两轴形状仍被接受；
- 中轴的轮无人认领 → 拒绝并点名 `middle_left`；
- 同一放置两个悬架 → 拒绝并点名；
- 整车没有任何悬架 → 拒绝并点名；
- **条目自己的轴仍可跑 K/C 读数**（`build_study_assembly` + `study_model_document` 产出文档）；
- **边界显式登记**：三轴**装配体本身**今天还不能被 K/C 契约读取——`cases/kc_quasi_static/contract.py:698` 的轮心查找只看 `<stem>_<side>`，三轴会给出每侧三个候选而拒绝（原文 `side L declares more than one wheel centre (front_wheel_hub_L, middle_wheel_hub_L, rear_wheel_hub_L)`）。这一条断言把缺口摆明，而不是让绿色套件暗示三轴整车已经可以扫掠。放开该读者属 03c。

## 4. 门禁与数值门（真实退出码）

| 命令 | 结果 |
|---|---|
| `just check-fast` | **退出 0**：快速集 **1040 passed / 1 xfailed**（03a 为 1033，增量即本轮新增 7 条）；kernel 33、contracts 32；ruff/ty 全绿；三个架构门全绿 |
| `snapshot.py --check` | **`OK`，退出 0（零差异）** |
| `dynamic_hash_sentinel.py --check` | **退出 0**：`combined sha256` 与冻结基线一致（26 artifacts 逐字节）|

关于数值门输出里的 `acceptance exit : 1` 与 9 个 `failed cases`：这是脚本**文档化的预期**（`docs/axle_dynamics_results.md` 记录三个路面用例在 `time_convergence` 门失败、项目选择如实记录而不放宽），哨兵本身只按哈希漂移判定，本次未漂移。此处照录，避免把「预期非零」读成回归。
