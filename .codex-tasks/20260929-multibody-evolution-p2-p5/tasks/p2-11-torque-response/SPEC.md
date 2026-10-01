# SPEC：p2-11 rotational_torque 非零响应专用夹具验收

> 子任务规格。父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p2-11`。
> 形态：`single-full`。来源：`code-reviewer` 裁决 `8e86187b`（p2-09 收口口径）。

## Goal

把 `p2-09` 未达成的「两端非零力矩响应」这条硬判据**独立验收**，用一个能取到非零读数的
文档路由装置（而非当前整车夹具）。拆成 4 条可判定判据：

1. **非零且等于需求幅值**：opt-in 装置的**首个有效样本**，code-10 两端力矩向量范数 = 该块的
   `max_torque`（`min(stiffness * demand, max_torque)` 在单位需求下的值），`rel=1e-12`。
2. **严格等大反向**：每个样本的两端力矩列满足 `first == -second`，`atol=0`。
3. **这些行属于该元素**：对照组（同模型移除元素）的 code-10 行数为 **0**。
4. **力偶跟随状态**：末样本归零（相对速率会合后内核第三分支不施力偶），`abs=1e-12`——
   只断言「非零」的检查对「一直施满力偶的错律」同样通过。

并**如实记录**当前整车夹具为何取不到非零读数，**不得**把两体结果写成整车夹具通过。

## 写范围

- `packages/suspension_multibody/tests/cases/test_rotational_torque_document.py`（新增 `_raw`
  辅助与 1 条用例；既有 8 条不动）
- `packages/suspension_multibody/tests/subsystems/test_rotational_torque_element.py`（只读复用）
- `packages/suspension_multibody/tests/subsystems/test_torque_element_wiring.py`（只读复用）
- 本目录 `raw/`

**本行不改任何生产代码。**

## 禁止触碰

- 不改内核（`packages/suspension_kernel/cpp/**`）与 ABI 版本常量。
- 不改整车夹具 `tests/vehicle/test_native_vehicle.py` 与其既有 `xfail`（那是另一件事）。
- 不重录基线；不新增 skip/xfail；不删或放宽既有断言。
- 其它子任务目录、`EPIC.md`、`SUBTASKS.csv`（父表由主代理登记）、父 `PROGRESS.md`——不得改。

## 依赖与时机

- `depends_on = p2-07;p2-08`：前者让文档路由接受该族，后者让力律按需求求值。
- **不依赖 p2-09**（避免与 p2-09 的 `TODO` 形成环）；`p2-05` 与 `p5-04` 的 `depends_on` 加上本行。

## Done-When

- [x] 四条判据各有断言（见 Goal），实测读数落 `raw/non_zero_response.md`。
- [x] 本行 `validation_command` 三份测试文件 **30 passed**。
- [x] `dynamic_hash_sentinel.py --check` 逐字节一致（`fdfd5a6b…eebc9`）。
- [x] 未改生产代码、未改 ABI、未重录基线、未新增 skip/xfail。

## Final Validation Command

```bash
uv run --no-sync pytest packages/suspension_multibody/tests/cases/test_rotational_torque_document.py   packages/suspension_multibody/tests/subsystems/test_rotational_torque_element.py   packages/suspension_multibody/tests/subsystems/test_torque_element_wiring.py -q && uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check
```
