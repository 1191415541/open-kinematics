# 快照生成记录（01 第 1 行的证据）

本文件存放**不进快照载荷**的易变信息（时间、命令与实测输出）。快照本身只放产物，所以它能被两次运行逐字节复现。

## 生成

- 生成时间：2026-09-29 19:03–19:05（会话本地时间；口径修订后重新生成）
- 生成命令：

```bash
uv run --no-sync python .codex-tasks/20260929-assembly-layer-rework/tasks/20260929-01-freeze/snapshot.py
```

- 判定命令：

```bash
uv run --no-sync python .codex-tasks/20260929-assembly-layer-rework/tasks/20260929-01-freeze/snapshot.py --check
```

- 产物大小：`raw/assembly_snapshot.json` = 539,636 字节（SPEC 的风险项原估「< 200 KB」，实测更大：四个单轴产物 + 一个整车产物的点表与约束字段被完整记录，按实测登记）

## 可重复性（两次运行逐字节一致）

| 运行 | sha256 |
|---|---|
| 第 1 次 | `b58d35acd93560fd52046228856307ccc0cd97655b37b69dcaad86e852b24566` |
| 第 2 次 | `b58d35acd93560fd52046228856307ccc0cd97655b37b69dcaad86e852b24566` |

修掉的一处不确定性（实测发现并修复）：首版把 `frozenset`/`set` 交给 `str()` 兜底，而集合的迭代顺序随进程哈希种子变化，导致两次运行 sha256 不同（首次实测 `720d8437…` vs `d8f0ccb4…`，差异全在 `capabilities` 的 `frozenset` 字段）。改为「按 JSON 序列化排序后的列表」后两次一致。判据：**产物内容**的差异要看得见，迭代顺序的噪声不要。

## 7 个组合的覆盖与产物映射

`rigs/rig.py:114 RIGS` 恰有 7 项，枚举只以该注册表为来源。**供轮的两个试验台各自的产物单独成键**（`axle_<模式>@<试验台>`）：供轮试验台会把轮端夹具并进装配（`subsystems/si_assembly.py` 合并 rig link），夹具的刚体与约束就是这次运行要解的东西，用同一个键会掩盖「只有某个试验台变了」；其余 5 个整车试验台共用 `vehicle`——它们的装配只由模型构成（`compose_vehicle_runtime` 不接收试验台），试验台在更晚的读数层自行绑定，这一层不在本快照内，由 05 自己的运行时对照负责（`_meta.coverage_boundary` 与每个 rig 的 `bench_bound` 都写明这一点）。

| rig | route | bench_bound | 运行的装配产物 |
|---|---|---|---|
| `kc_quasi_static` | `kc_quasi_static` | True | `axle_K@kc_quasi_static`、`axle_C@kc_quasi_static` |
| `axle_dynamic` | `axle_dynamic` | True | `axle_K@axle_dynamic`、`axle_C@axle_dynamic` |
| `vehicle_kc` | `vehicle_kc` | False | `vehicle` |
| `vehicle_dynamic` | `vehicle_dynamic` | False | `vehicle` |
| `handling` | `handling` | False | `vehicle` |
| `ride_four_post` | `ride_four_post` | False | `vehicle` |
| `ride_random_road` | `ride_random_road` | False | `vehicle` |

实测计数（`bodies/points/constraints/ideal_constraints/bushings/elements/connections`）：

| 产物 | 计数 |
|---|---|
| `axle_K@kc_quasi_static` | 15 / 44 / 18 / 18 / 0 / **0** / 20 |
| `axle_C@kc_quasi_static` | 15 / 44 / 14 / 22 / 16 / 16 / 20 |
| `axle_K@axle_dynamic` | 15 / 44 / 18 / 18 / 0 / **0** / 20 |
| `axle_C@axle_dynamic` | 15 / 44 / 14 / 22 / 16 / 16 / 20 |
| `vehicle` | 29 / 79 / 36 / 32 / 0 / **0** / 40 |

**K 读数不含力元是既有分层，不是缺记录**：K 的受力列是理想副行（`constraints` / `ideal_constraints`），力元（`bushings` / `elements`）只在 C 读数出现（`EPIC.md` F9）。因此脚本的非空判据按「受力列整体」判定：`bodies`/`points`/`constraints` 必须非空，且 `constraints + ideal_constraints + elements` 必须大于 0；若强行要求 K 也有 `elements`，这条判据会把分层判成失败。

四个单轴产物的计数一致、且都带 `wheel_carrier_L/R`，说明两个供轮试验台在这次装配里的夹具结构相同；即便如此仍分开成键，因为**计数相同不等于内容相同**，判据要看内容。

## 登记口径（`raw/approved_deltas.json`）

`--check` 的差异必须被登记项**精确命中**：`product`、`pointer`、`before`、`after` 四项全等，**没有前缀覆盖规则**——一条登记只为它写的那一处差异负责，后面在同一位置发生的另一种变化不会被旧条目悄悄放行。登记项还必须带 `reason`、`registered_by`、`evidence`，且 `registered_by` 必须以 **05** 开头：只有 05 被允许移动既有产物。登记本身不合法（缺字段、归属不是 05）时 `--check` 直接失败（退出码 4），不当作「空登记」放过——那正是登记机制要防的失效模式。`registered_by` 是流程约束而不是证明，所以 07 要逐条审计登记与各行 `PROGRESS.md` 里记录的逐步变化是否对得上。

register 条目的字段与示例：

```json
[
  {
    "product": "axle_K@kc_quasi_static",
    "pointer": "/elements/2/wheel_body",
    "before": "wheel_hub_L",
    "after": "wheel_carrier_L",
    "reason": "试验台不再改写被测零件的所有者（D3）",
    "registered_by": "05 试验台非侵入",
    "evidence": "raw/rig_product_diff.md#tire-ownership"
  }
]
```

## `--check` 五条路径实测（负例 / 正例 / 登记守卫）

| # | 输入 | 实测输出 | 退出码 |
|---|---|---|---|
| 1 | 把 `axle_K@kc_quasi_static.body_names[0]` 改成 `upper_arm_L_X`，登记为空 | `differences: 1 (registered: 0)` + `[UNREGISTERED] …` | **1** |
| 2 | 同上，写入精确匹配的登记（`registered_by: 05 …`，四项全等） | `differences: 1 (registered: 1)` + `OK` | **0** |
| 3 | 同上登记但 `registered_by: 03 通用装配引擎` | `FAIL: register entry 0: registered_by … is not subtask 05` | **4** |
| 4 | 登记归属为 05 但 `before` 写成改动后的值（与差异不符） | `[UNREGISTERED] …` | **1** |
| 5 | 登记缺 `reason`/`registered_by`/`evidence` | `FAIL: register entry 0: missing reason, registered_by, evidence` + 归属检查 | **4** |

之后快照还原（sha256 回到 `b58d35ac…`）、登记复位为 `[]`，`--check` 再次退出 **0**。

## 口径修订记录（2026-09-29，本轮）

- 产物键从「3 个共用产物」改为「4 个按试验台的单轴产物 + 1 个整车产物」：原写法把两个供轮试验台都记成 `kc_quasi_static` 的产物，无法证明「每个试验台的挂接产物都未变」，也撑不起 05 的逐组合登记门。
- 登记从「产品名 + 指针前缀」改为「四项精确匹配 + 归属校验」：原写法下登记 `/elements` 就能放行其下任意变化。
- 快照因此重新生成（`f6d3c3f2…` → `b58d35ac…`）。**这是口径变化，不是产物变化**：`approved_deltas.json` 仍为空数组，没有任何既有产物被登记为「已改变」。
- 判据文字与实现不对齐之处已修：01 的 SPEC 与 `TODO.csv` 第 1 行原写「每个组合的 `elements` 非空」，与 K 读数的实际分层冲突，已改为「`bodies`/`points`/`constraints` 非空且受力列整体非空」。

## 与后续子任务的关系

- 02/03/04/06 在落地前后各跑一次 `--check`，预期**零差异**；出现差异即该行落地改变了既有产物（登记不能由它们写）。
- 05 的每个变化都要登记进 `raw/approved_deltas.json`（文件 + 步骤 + 前后值 + 独立于结果字节的物理等价判据），但**整车侧的试验台绑定不在本快照覆盖范围内**（见 `_meta.coverage_boundary`），05 对 5 个整车 rig 必须用自己那份「接入前后运行时逐项对照」的断言，不能只看本文件。
- 07 逐条审计该清单：登记项必须有物理等价判据，且不得被用来掩盖未声明的变化。
- **本快照不管数值**：数值层面的零回归由 `kc_baseline/` 与 `dynamic_hash_baseline.json` 负责（04 用两条 probe 生成 actual 再带 `--actual-dir` 比对）。
