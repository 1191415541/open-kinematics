# 05 步骤 2：凝聚等价性契约与体 ID 映射登记（A1 修订口径）

> **2026-09-22 更新（用户裁决 A3，A1 关闭）**：用户裁决「A1 改为由 native 的 `fixed` 契约承接焊接语义，可以重录车辆基线」，解除了原「凝聚保留在 Python 作者层、不重录基线」的约束上限。生产路径现已切换：`_condense_welded_bodies` 默认**不再融合**，weld 作为 `kind="fixed"` 交给 native；融合实现保留为 `_fuse_welded_bodies`（`SUSPENSION_MULTIBODY_CONDENSE_WELDS=1` 恢复），车辆字节基线按新路径的 native 结果重录。全文见 `EPIC.md` 的 A3 修订记录。

按用户裁决 A1（2026-09-21）：凝聚**保留在 Python 作者层**，不迁入 `mb_assembly`；native 已有的 `kind="fixed"` 关节作为**契约等价实现**，须有等价性测试与体 ID 映射登记。本文档是该登记，测试是证据。

## 1. 两侧各自的语义

| 侧 | 机制 | 位置 | 产出 |
|---|---|---|---|
| Python 作者层（**A3 后为回退路径**，默认不启用） | 焊接体精确凝聚：并查集分组 → 融合质量/质心/惯量（平行轴定理）→ 重定向点/轴/位姿 → 记录 `body_aliases` | `preparation/assembly/vehicle.py:272` `_fuse_welded_bodies`（原 `model/vehicle.py:243` `_condense_welded_bodies` 逐字搬运），由 `_condense_welded_bodies`（`:252`）在 `SUSPENSION_MULTIBODY_CONDENSE_WELDS=1` 时调用 | 融合体集合 + `body_aliases: dict[str,str]` |
| native 契约 | `kind="fixed"` 关节：两个 run，`{3,false}` 点重合 + `{3,true}` 全相对旋转 = 6 行 | `cpp/include/mb_joint/types.hpp:53`；残差 `cpp/src/joint/kernel_model_constraint.cpp:155` `joint_residual_fixed`；注册表 `cpp/src/contract/contract_registry.cpp:24` `{"fixed", 6}` | 6 个约束行，两侧 body 保持分离 |

**等价性声明**：两者表达同一个理想约束（两刚体相对位姿完全固定）。Python 路径把它**消去**（自由度减少），native 路径把它**保留为约束行**（自由度不变、由 6 行约束消除）。同一个物理问题的两种正确表述。

## 2. 体 ID → 凝聚体 ID 映射

- 映射载体：`VehicleAssembly.body_aliases`（`preparation/assembly/vehicle.py` 定义，`_fuse_welded_bodies` 内填充；原路径为 `model/vehicle.py:52,517`）。**A3 后该映射只在回退路径（`SUSPENSION_MULTIBODY_CONDENSE_WELDS=1`）产生；默认路径 `body_aliases` 为空，身份映射为空集。**
- 填充规则（`vehicle.py:295-301`）：每个成员数 >1 的分量，选 `component_root` 为根——含 `chassis` 时取 `chassis`，否则取 `weld.body_b` 中在 `body_order` 最靠前者，再否则取分量内最靠前者；其余成员 `alias[body] = root`。
- 消费方（已有生产调用者，均经 alias 解析后使用）：
  - `preparation/vehicle_dynamic.py:338` 静态旋转规（`add` 内 `assembly.body_aliases.get(body, body)`）
  - `:405-406` `body_aliases` 并入初值解析
  - `:446`、`:474` `_resolve_vehicle_body(..., body_aliases)`
  - `:1039-1042` 反查
- 消费方（结果/渲染侧）：Adams 渲染经同一 alias 表定位实体（`adams/full_vehicle_model.py:3165-3167` 走 `build_vehicle`）。

## 3. 等价性测试（本步骤交付）

新增于 `packages/suspension_multibody/tests/vehicle/test_native_vehicle.py`（**测试名与断言已按 A3 更新**）：

1. `test_native_fixed_joint_is_what_carries_a_weld`（:343）
   - 默认路径：`_build_joints` 产出中含 `kind == "fixed"` 关节，`body_aliases` 为空，关节 body 对与 `assembly.constraints` 一致；
   - `SUSPENSION_MULTIBODY_CONDENSE_WELDS=1` 回退路径：融合体质量 = 成员质量之和（本夹具 upright 100 kg + 轮 20 kg = 120 kg）、无 `fixed` 关节、`body_aliases` 非空（映射可追溯）。
2. `test_the_two_weld_routes_agree_on_the_world_mass_properties`（:396）
   - 对两条路径分别用 `assembly.state.point_world` 求世界系总质量与质心，断言相等（`atol=1e-9`）。
3. `test_native_vehicle_runs_two_suspensions_and_four_wheels`（:324）
   - body 断言由 22 改为 23，并显式要求 `rear_rack` 在体集合内。

**A3 前的旧测试已被改写/更名**（原文中的 `test_native_fixed_joint_matches_python_weld_condensation`、`test_native_fixed_joint_shape_matches_the_registry` 与「凝聚后不得再有 fixed 关节」的断言，均属 A1 口径、在新默认下已反转）。

既有覆盖（本步骤未改，作为回归基线）：`tests/model/test_vehicle.py:89`（凝聚消除 mount 约束）、`:110`（凝聚保持世界系质量/惯量性质）。

## 4. 该项的最终处置（2026-09-22 裁决 A3：已关闭）

用户第二问所选选项 A 的**手段**「Python 不再凝聚、把 weld 送 native 作 fixed 约束」，与原 A1 的「不重录基线、字节门保持绿」冲突：

- 证据：当时 `SUSPENSION_MULTIBODY_CONDENSE_WELDS=0` 下 native body 集合 22→23（`tests/vehicle/test_native_vehicle.py` 的既有断言实测），而整车门是字节级 sha256（`scripts/case_parity_check.py` 的 `_VEHICLE_LEDGERS`，基线 `tests/data/vehicle_dynamics_baseline/sha256.json`）。
- **原处置（已被取代）**：以 A1 为约束上限，只交付契约等价性（§3），生产路径维持 Python 凝聚，登记为未闭合项。
- **最终处置（裁决 A3）**：用户裁决「A1 改为由 native 的 `fixed` 契约承接焊接语义，可以重录车辆基线」，约束上限解除。生产路径已切换为**不凝聚 + weld 送 native fixed**，`vehicle_dynamics_baseline/sha256.json` 按新路径的 native 结果重录。**本项不再是未闭合项。**

**等价性证据（实测；已按全 8 case 逐 case 实测更正）**：两条路径**世界系质量性质完全一致**——总质量均为 `3080.0`、世界质心均为 `[12.987013, 0.0, 160.390]`（全部 8 case）。差异两类，均**非物理改变**：(a) 形状/布局——native 路径多出 `rear_rack` body（22→23）、weld 以 6 行 `fixed` 保留（约束行 29→30）、融合体原点移到质量加权中心致 chassis 位姿常数偏移 `px=-0.1077, pz=+0.0192`；(b) **求解器残差级数值差**——`tire_output` ≤`1.4e-9` N、`energy` ≤`3.9e-14` J、`bushing_output` `2.2e-13`（`spring_output` 全 0）。**初版「`tire_output` 与 `energy` 逐位相同」的表述只对 5 个 case 成立，已更正，不可外推**。测试：`test_native_fixed_joint_is_what_carries_a_weld`、`test_the_two_weld_routes_agree_on_the_world_mass_properties`。逐 case 数据见 `a3_condensation_switchover.md` §4。

## 5. 步骤 2 验证证据（raw/）

| 命令 | 退出码 | 证据文件 |
|---|---|---|
| `build_axle_native.py` | 0 | `step2_build.log` |
| `dynamic_hash_sentinel.py --check` | 0（26/26 逐位一致） | `step2_dynamic_hash.log` |
| `case_parity_check.py` | 0（8 families accepted） | `step2_case_parity.log` |
| `kc_parity_check.py --check` | 0 | `step2_kc_parity.log` |
| kernel tests | 0（15 passed） | `step2_kernel_tests.log` |
| contracts tests | 0（22 passed） | `step2_contracts_tests.log` |
| results+physics+vehicle+cases+architecture | 0（254 passed, 1 xfailed） | `step2_suite.log` |

**A3 前**未重录任何基线；**A3 后**按用户明确授权重录了唯一一项 `tests/data/vehicle_dynamics_baseline/sha256.json`（8 个 case，按新路径的 native 结果）；`dynamic_hash_baseline.json`、`kc_baseline/`、`kc_perf_baseline*.json` 未改动（组合哈希 `e7407656…8d48e` 仍逐位一致）。
