# 新增 `anti_roll_bar` 角色的同步点全清单（p4-01 冻结证据 · F12）

实测命令：`grep -rn` 全仓（排除 `.codex-tasks/` 与 `*.pyc`）。下面的每一项都来自本行真实执行的 grep，不是抄父 Epic 文字。

## 六个角色名的全仓命中（真实命令）

命令：
```bash
grep -rn '"suspension", "steering", "wheel", "chassis", "brake", "drive"' --include=*.py --include=*.json . | grep -v "\.codex-tasks"
grep -rn '"chassis", "suspension", "steering"' --include=*.py --include=*.json . | grep -v "\.codex-tasks"
```
输出（按**代码**同步点，均含六角色元组的字面量）：
```
packages/suspension_contracts/src/suspension_contracts/contracts/assembly.schema.json:22
packages/suspension_contracts/src/suspension_contracts/contracts/subsystem.schema.json:8
packages/suspension_contracts/src/suspension_contracts/contracts/template.schema.json:11
packages/suspension_multibody/src/suspension_multibody/authoring/documents.py:60
packages/suspension_multibody/src/suspension_multibody/connections/policy.py:48
packages/suspension_multibody/src/suspension_multibody/subsystems/capabilities.py:39
packages/suspension_multibody/src/suspension_multibody/subsystems/composition.py:313
packages/suspension_multibody/src/suspension_multibody/subsystems/types.py:70
packages/suspension_multibody/src/suspension_multibody/subsystems/types.py:84
packages/suspension_multibody/src/suspension_multibody/templates/roles.py:158
```
（`roles.py:70` 的 `ROLES` 是**逐条 RoleSpec 定义**，不写成一行六元组，故不出现在上面的一行式 grep 里；见下方逐项。）

## 逐项同步点（file:line + 原文）

### 1. `templates/roles.py` — ROLES 真源 + import 期硬断言
- `roles.py:68-70`：
  ```python
  #: The six roles, keyed by name.  See `EPIC.md` G2 for the availability matrix
  ROLES: dict[str, RoleSpec] = {
  ```
  （`"suspension"` 条目 `:71-90`，`"brake"` `:122-139`，`"drive"` `:140-152`；新增 `"anti_roll_bar"` 要在此加条目。）
- `roles.py:156-162` **import 期硬断言**（集合不等于六角色即在导入时抛 `RoleSpecError`）：
  ```python
  def _check_roles() -> None:
      """Reject a role table that cannot be trusted, where it is written."""
      expected = {"suspension", "steering", "wheel", "chassis", "brake", "drive"}
      if set(ROLES) != expected:
          missing = sorted(expected - set(ROLES))
          extra = sorted(set(ROLES) - expected)
          raise RoleSpecError(f"role table mismatch; missing={missing} extra={extra}")
  ```
- `roles.py:194-196 role_names()`：`return tuple(sorted(ROLES))`（测试第 10 项依赖它）。

### 2. `authoring/documents.py` — FUNCTIONAL_ROLES
- `documents.py:59-61`：
  ```python
  FUNCTIONAL_ROLES = frozenset(
      {"suspension", "steering", "wheel", "chassis", "brake", "drive"}
  )
  ```
- 邻行 `:58` `TWIN_ENDED_ELEMENTS = frozenset({"spring", "damper", "bump_stop", "anti_roll_bar"})` 是**元素种类**集合（已含 `anti_roll_bar`），与角色无关，无需改——但要与角色同名区分，避免混淆。

### 3. `subsystems/types.py` — SUBSYSTEM_ROLES + 两个默认集合
- `types.py:69-71`（`:49` 在 `__all__` 导出）：
  ```python
  SUBSYSTEM_ROLES: frozenset[str] = frozenset(
      {"suspension", "steering", "wheel", "chassis", "brake", "drive"}
  )
  ```
- `types.py:76-78` 默认单轴集合：
  ```python
  DEFAULT_AXLE_SUBSYSTEMS: frozenset[str] = frozenset(
      {"suspension", "steering", "wheel"}
  )
  ```
- `types.py:83-85` 默认整车集合：
  ```python
  DEFAULT_VEHICLE_SUBSYSTEMS: frozenset[str] = frozenset(
      {"chassis", "suspension", "steering", "wheel", "brake", "drive"}
  )
  ```
- **附带**：`types.py:275-280 _ROLE_TEMPLATE_FIELD`（`suspension/steering/chassis/wheel` 四个映射）与新角色的文件载体相关，p4-02 若给 ARB 模板文件则需同步（`_role_template_field` 未命中会 `ValueError`，`:283-291`）。

### 4. `subsystems/capabilities.py` — ALL_SUBSYSTEMS
- `capabilities.py:36-40`：
  ```python
  #: The six subsystem role names an assembly may carry.  Deliberately `wheel`
  #: rather than `tire`, matching `templates.roles`.
  ALL_SUBSYSTEMS: frozenset[str] = frozenset(
      {"suspension", "steering", "wheel", "chassis", "brake", "drive"}
  )
  ```

### 5. `subsystems/composition.py` — SUBSYSTEM_ROLES（有序元组）+ 一处交集字面量
- `composition.py:57-70`：
  ```python
  #: The six roles, in the order the *recorded* body list has always used.
  SUBSYSTEM_ROLES: tuple[str, ...] = (
      "chassis",
      "suspension",
      "steering",
      "wheel",
      "brake",
      "drive",
  )
  ```
  （此处**顺序**影响记录产物，新增角色要决定其位置。）
- `composition.py:313`（**第二处字面量**，实测存在于六元组 grep 命中）：
  ```python
  subsystems=frozenset(roles) & {"suspension", "steering", "wheel", "chassis", "brake", "drive"},
  ```

### 6. `connections/policy.py` — ROLES
- `policy.py:47-48`：
  ```python
  #: The six subsystem roles, in the vocabulary the templates use.
  ROLES: tuple[str, ...] = ("suspension", "steering", "wheel", "chassis", "brake", "drive")
  ```

### 7-9. 三份契约 schema 的 `functional_role` enum
- `packages/suspension_contracts/src/suspension_contracts/contracts/template.schema.json:11`：
  ```json
  "functional_role": {"enum": ["suspension", "steering", "wheel", "chassis", "brake", "drive"]},
  ```
- `.../contracts/subsystem.schema.json:8`：
  ```json
  "functional_role": {"enum": ["suspension", "steering", "wheel", "chassis", "brake", "drive"]}, "placement_role": {...},
  ```
- `.../contracts/assembly.schema.json:22`：
  ```json
  "functional_role": {"enum": ["suspension", "steering", "wheel", "chassis", "brake", "drive"]},
  ```

### 10. 测试硬断言六角色名
- `packages/suspension_multibody/tests/templates/test_template_model.py:57-66`：
  ```python
  def test_six_roles_are_declared() -> None:
      assert role_names() == (
          "brake",
          "chassis",
          "drive",
          "steering",
          "suspension",
          "wheel",
      )
  ```
  （F12 称 `:57 test_six_roles_are_declared`，实测未漂移。）
- 同文件 `:69-71 test_only_brake_and_drive_carry_a_torque_channel`：`assert torque == {"brake", "drive"}`——新增角色若 `has_torque_channel=False` 则不受影响，但仍属角色表变化的关联断言。

## 项数统计

- **代码/契约同步点：10 处**（上面第 1~10 项），其中
  - 写角色集合/元组字面量的有 **9 处**（roles.py、documents.py、types.py×2、capabilities.py、composition.py×2、policy.py、三个 schema 中的 3 个算 3 处 → 实为 9 个文件位置；逐位置计见下）；
  - 测试硬断言 **1 处**。
- **逐位置计数（供核对，共 13 个「行位置」）**：
  1. `templates/roles.py:70`（ROLES 定义）
  2. `templates/roles.py:158`（expected 集合硬断言）
  3. `authoring/documents.py:60`（FUNCTIONAL_ROLES）
  4. `subsystems/types.py:70`（SUBSYSTEM_ROLES）
  5. `subsystems/types.py:77`（DEFAULT_AXLE_SUBSYSTEMS）
  6. `subsystems/types.py:84`（DEFAULT_VEHICLE_SUBSYSTEMS）
  7. `subsystems/capabilities.py:39`（ALL_SUBSYSTEMS）
  8. `subsystems/composition.py:63`（SUBSYSTEM_ROLES 元组）
  9. `subsystems/composition.py:313`（交集字面量）
  10. `connections/policy.py:48`（ROLES）
  11. `template.schema.json:11`（enum）
  12. `subsystem.schema.json:8`（enum）
  13. `assembly.schema.json:22`（enum）
  14. `tests/templates/test_template_model.py:57`（测试断言）
- **结论：F12 的「10+ 处」成立，实测 14 个行位置（13 个独立同步位置 + 1 处测试），无遗漏第 11 类文件。** 全仓六角色一行式字面量 grep 只有上述 10 个行命中；`types.py:84` 顺序不同（`chassis` 在前）故单列。除上述外，**未发现**其它文件把六个角色名写成集合/元组（`subsystems/explicit.py:273`、`si_assembly.py:277` 等是单角色 `roles.add("suspension")`，不是全表）。

## 附：与角色名同名但**不是**同步点的地方（避免误改）
- `subsystems/suspension.py:64 role = "suspension"`、`subsystems/wheel.py:54 role = "wheel"` 等是**子系统自身模块的 `role` 常量**，新增 `anti_roll_bar` 时需要新模块，但不属于「改现有表」。
- `authoring/solver.py:698/779` 等 `entry.functional_role == "suspension"` 是**按角色分流**，不是全表。
