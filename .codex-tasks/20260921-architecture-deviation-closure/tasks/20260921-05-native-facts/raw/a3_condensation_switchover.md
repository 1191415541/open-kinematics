# A3 实施记录：A1 关闭（生产路径不凝聚，weld 由 native fixed 承接）

用户裁决（2026-09-22）：**「A1 改为由 native 的 fixed 契约承接焊接语义，可以重录车辆基线」；「A2 先保持现状」**。本文档是该裁决的实施与证据记录。规划层修订见 `../../EPIC.md` 的 A3 修订记录。

## 1. 裁决解除了什么

原 A1（2026-09-21）的处置是「以『不重录基线、字节门保持绿』为约束上限，只采纳目标、保留手段」。A3 解除该上限，于是 A1 的**手段**可以实施：生产路径不再在 Python 侧融合焊接体，而把 weld 交给 native 的 `kind="fixed"` 契约。

## 2. 生产变更

`packages/suspension_multibody/src/suspension_multibody/preparation/assembly/vehicle.py`：

| 项目 | 变更前 | 变更后 |
|---|---|---|
| `_condense_welded_bodies` | 默认融合焊接体；`SUSPENSION_MULTIBODY_CONDENSE_WELDS=0` 关闭 | **默认不融合**（直接返回装配） |
| 融合实现 | 即 `_condense_welded_bodies` 本体 | 保留为 `_fuse_welded_bodies`，逻辑逐字未改 |
| 开关语义 | `=0` 关闭凝聚 | `=1` 启用凝聚（**语义反转**，回退路径） |
| weld 去向 | 被融合消去 | `WeldJoint` → `_build_joints` → `kind="fixed"`（`preparation/vehicle_dynamic.py` 的 `_build_joints`）交给 native |

契约侧无需改动：`contract_registry.cpp:24` 已声明 `{"fixed", 6}`（点重合 3 行 + 全相对旋转 3 行）。

## 3. 基线重录（唯一一项，经用户明确授权）

`packages/suspension_multibody/tests/data/vehicle_dynamics_baseline/sha256.json`：按**切换到 native fixed 后的 native 执行结果**重录 8 个 case，不与旧 Python 凝聚的数值做对比。

**未改动**的三项及其证据：

| 基线 | 状态 | 证据 |
|---|---|---|
| `tests/data/dynamic_hash_baseline.json` | 未改 | `dynamic_hash_sentinel.py --check` 组合哈希 `e7407656731ed556efc28fb89d8fc69881725b3bfb2f39066eb898986389d48e`，与 01 冻结值**逐位相同** |
| `tests/data/kc_baseline/` | 未改 | `kc_parity_check.py --check` 退出 0（within tolerance） |
| `tests/data/kc_perf_baseline*.json` | 未改 | 未触碰（性能门非本裁决范围） |

**为什么只有 vehicle 一项失效**：`dynamic_hash_sentinel` 与 `kc_parity_check` 比较的是 **axle** 模型（`build_axle_model` / `kc_native_probe`），**不经过 `build_vehicle`**，因此焊接体路径的变化触不到它们。整车门 `case_parity_check` 的 `vehicle_dynamic` family 经过 `build_vehicle`，是唯一受影响者。

## 4. 等价性证据（实测，非推断）

对 `vehicle_dynamic` 的**全部 8 个 case** 逐 case 对比两侧（新默认 = 不凝聚；回退 = 融合）：

| 物理量 | 差异类型 | 实测最大差异 | 判定 |
|---|---|---|---|
| 总质量 | 无 | 两侧均 `3080.0` | **一致** |
| 世界质心 | 无 | 两侧均 `[12.987013, 0.0, 160.390]` | **一致** |
| `states` | **形状** | `(2,23,19)` vs `(2,22,19)` | 表示差异（多 `rear_rack` body） |
| `constraint_wrench` | **形状** | `(2,30,6)` vs `(2,29,6)`；bushing case `22` vs `21` | 表示差异（weld 由 1 条 fixed 关节的 6 行表达，而非被消去） |
| `tire_output` | 数值 | `1.410e-09`（steering）、`1.149e-09`（nondefault）、`6.27e-12`（bushing）；其余 5 个 case 为 `0` | 求解器残差级 |
| `energy` | 数值 | `3.896e-15` … `3.109e-14`（3 个 case）；其余为 `0` | 浮点噪声级 |
| `bushing_output` | 数值 | `2.203e-13`（仅 bushing force curves case） | 浮点噪声级 |
| `spring_output` | 无 | 全部 `0` | 一致 |

**结论**：两条路径**不是逐位相同**。差异分两类，均**不是物理改变**：

1. **形状/布局差异**：不凝聚路径多一个 `rear_rack` body；weld 以 6 行 `fixed` 关节保留，而非被凝聚消去。这是同一物理的两种表述（自由度由约束消除 vs 由凝聚消除）。
2. **求解器残差级数值差异**：`tire_output` 最大 `1.4e-9` N、`energy` 最大 `3.9e-14` J、`bushing_output` `2.2e-13`。来源是两种表述下**约束求解的数值路径不同**，不是力律或模型参数的改变。**独立支撑**：世界系质量性质（总质量、质心）在全部 8 个 case 上完全一致；`spring_output` 全部逐位一致。

> **归因更正说明（须保留）**：本记录与提交信息 `90d6d23` 初版曾写「`tire_output` 与 `energy` 逐位相同」。该结论仅对 8 个 case 中逐位相同的那 5 个成立，**不可外推**。已按逐 case 实测数据更正为上表。这不改变切换决定（差异在求解器残差量级且质量性质一致），但结论强度必须如实表述。

**chassis 的常数偏移**：融合体的 body 原点被移到质量加权中心（chassis 的 CoM 从 `[0,0,0]` 变为 `[-107.692308, 0, 19.230769]`），故 chassis 的位姿行带常数偏移 `px=-0.1077, pz=+0.0192`（两样本相同）。这是**同一刚体的不同参考点约定**。

## 5. 测试变更

`packages/suspension_multibody/tests/vehicle/test_native_vehicle.py`：

- **新增** `test_native_fixed_joint_is_what_carries_a_weld`：钉住默认路径形态（独立 body + `fixed` 关节 + 无 `body_aliases`）**与** `=1` 回退形态（融合体 120 kg、无 `fixed` 关节、有 `body_aliases`）。
- **新增** `test_the_two_weld_routes_agree_on_the_world_mass_properties`：世界系总质量与质心必须相等（对全部体的质量加权求和）。
- **改写** `test_native_fixed_joint_matches_python_weld_condensation` → 上述第一项（原断言「凝聚后不得再有 fixed 关节」，在新默认下已反转）。
- **更新** `test_native_vehicle_runs_two_suspensions_and_four_wheels`：body 断言 `22 → 23`，并显式要求 `rear_rack` 在体集合内。

## 6. 门禁实测（切换后全集）

| 命令 | 退出码 | 摘要 |
|---|---|---|
| `build_axle_native.py` | 0 | DLL 产出 |
| `check_module_layering.py --strict --final` | 0 | 分层与旧模块缺席全绿 |
| `pytest suspension_kernel/tests -q` | 0 | 15 passed |
| `pytest suspension_contracts/tests -q` | 0 | 22 passed |
| `dynamic_hash_sentinel.py --check` | 0 | 26/26 逐位一致，组合哈希未变 |
| `kc_parity_check.py --check` | 0 | within tolerance |
| `case_parity_check.py` | 0 | **8 families accepted**（`vehicle_dynamic` 已按新基线通过） |
| `legacy_surface_gate.py --check` | 0 | no unregistered boundary violation |
| `pytest suspension_multibody/tests -q` | 0 | **737 passed** / 47 skipped / 1 xfailed |
| `ruff check .` / `ty check .` | 0 / 0 | All checks passed |
| `uv build` ×2 | 0 / 0 | 两包构建成功 |
| `git diff --check` | 0 | 干净 |

## 7. A2 状态（按同轮裁决保持现状）

`analysis/vehicle_physics.py` 的 `compute_static_wheel_loads` 继续保留在 Python，登记与解除条件不变（见 `step4_static_wheel_loads_registration.md`）。本轮未触碰该文件。

## 8. 变更文件清单

- `src/suspension_multibody/preparation/assembly/vehicle.py`（生产：默认不凝聚 + `_fuse_welded_bodies` 回退）
- `tests/vehicle/test_native_vehicle.py`（2 新增 + 1 改写 + 1 更新）
- `tests/data/vehicle_dynamics_baseline/sha256.json`（唯一重录项，经授权）
