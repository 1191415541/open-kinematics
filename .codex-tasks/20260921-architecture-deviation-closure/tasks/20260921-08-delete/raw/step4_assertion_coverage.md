# 08 步骤 4：被删求解实现的物理断言 → native 契约测试（逐项登记）

本文件是 08 SPEC 验收 4 要求的覆盖关系登记：**原断言 → 新测试 file:line**。

被删的三个求解实现及其测试来源：

| 被删实现 | 原断言所在测试 | 原断言 |
|---|---|---|
| `core/rank.py`（`diagnose_rank`/`scale_jacobian`/`RankDiagnostic`） | `tests/core/test_reactions.py:17`（`test_rank_diagnostic_detects_under_and_over_constraint`） | `diagnose_rank(np.eye(2,6)).underconstrained` 为真；`diagnose_rank(堆叠重复行).overconstrained` 为真 |
| `core/reactions.py`（`recover_reactions`/`body_equilibrium_wrench`/`ReactionResult`） | `tests/core/test_reactions.py:24`（`test_reaction_wrench_is_balanced_between_ball_joint_bodies`） | 球铰两侧的约束反力力矩互相抵消：`body_equilibrium_wrench(...)` ≈ 0 |
| `model/mass.py`（`mass_matrix`/`body_mass_properties`/`spatial_bias_wrench`） | `tests/model/test_dynamic_mass.py:52`（`test_mass_matrix_rejects_zero_mass_movable_body`）、`:59`（`test_spatial_inertia_contains_mass_and_rotational_inertia`） | 质量矩阵只接受正质量动体；空间惯量对角线 = `[m,m,m,Ixx,Iyy,Izz]` |

`tests/core/test_reactions.py` 已随实现删除；`tests/model/test_dynamic_mass.py` 只保留与
schema→装配映射有关的一条测试（`:32`），两条质量矩阵断言移出。

新测试文件：`packages/suspension_multibody/tests/axle_dynamics/test_solver_invariants.py`
（走 `suspension_multibody.axle_dynamics` 的 native 契约路径：`AxleDynamicsModel` + `run_axle_dynamics`）。

## 1. 覆盖矩阵

| 原断言 | 新测试（file:line） | 覆盖方式与强度 |
|---|---|---|
| `diagnose_rank(...).overconstrained` | `tests/axle_dynamics/test_solver_invariants.py:115`（`test_a_rank_deficient_joint_set_is_refused_by_the_native_model_build`） | 在竖直滑块的导向上重复声明同一 prismatic 副，native `build_model` 直接拒绝：`NativeAxleError: model build: constraint Jacobian is rank deficient at the initial pose: rank 5 of 10 rows over 6 columns`。比原断言更强——旧实现只标记 `overconstrained`，native 是 fail-closed 拒绝 |
| `diagnose_rank(...).underconstrained` | 同文件 `:141`（`test_an_unconstrained_body_is_refused_as_an_equilibrium_and_left_free`） | 双向：无约束刚体在默认静力初始化下被拒（`static equilibrium initialization failed ... pinned_null_directions=5`），给出 consistent state 后按 `a_z = -9.80665` 自由下落（`state[0,13:16]` 精确、末样本 z 下降）。即 native 既不静默固定也不虚构约束 |
| `recover_reactions` + `body_equilibrium_wrench`（反力互相抵消/承载外载） | 同文件 `:177`（`test_a_revolute_reaction_recovers_the_gravity_load`） | 悬挂摆：`result.joint_wrench_on_body_b("hinge")` 的 z 向力 = `m*g`（98.0665 N，容差 1e-9），其余 5 个分量 ≈ 0，且 bob 静止（速度全 0）。约束反力通道 `constraint_wrench` 就是 native 的反力恢复出口 |
| 同上（“载荷确实进入求解器”） | `tests/cases/kc_quasi_static/test_k_body_wrench.py:58`（既有 native 契约测试，未改动） | K 工况下施加 body wrench：位姿逐位相同（`atol=1e-12`）而 `constraint_wrench` 块必须改变——载荷经约束反力进入求解器，而非被静默丢弃 |
| `body_mass_properties`/`mass_matrix`（空间惯量、质量矩阵） | `tests/axle_dynamics/test_solver_invariants.py:224`（`test_the_native_mass_matrix_uses_the_body_inertia_tensor`） | 防倾杆给自由体施加已知力矩，`body_state[...,18]`（角加速度 z）严格等于 `torque / I_zz`（本例 `I_zz=2.5` → `alpha=-84.0`，与 `torque=-210.0` 精确一致），证明 native 质量矩阵用的是刚体自身惯量张量 |
| `spatial_bias_wrench`（Newton–Euler 速度偏置） | 同文件 `:274`（`test_a_body_spinning_about_a_non_principal_axis_shows_the_bias_term`） | 自由刚体、`I=diag(1,2,3)`、`omega=(1,0,1)`：实测 `alpha[0]=(0,1,0)` 与 `I·dω/dt = -ω×(Iω)` 精确一致（容差 1e-12），转动动能 `0.5·ωᵀIω` 全程守恒（`atol=1e-9`），且 ω 相对初值确实变化 |
| 质量在载荷路径中的存在（原 `mass_matrix` 的正质量前提） | `tests/axle_dynamics/test_api.py:129`（既有 native 契约测试，未改动） | 竖直滑块静力平衡位置 `z = 0.25 - m·g/k` 与弹簧力 `m·g` 逐项断言，质量矩阵与重力装配一致 |

## 2. 未因删除而丢失的既有覆盖（登记备查）

| 原断言 | 保留位置 |
|---|---|
| `diagnose_rank` 的秩亏拒绝语义 | `tests/axle_dynamics/test_cpp_physics.py:197`、`tests/axle_dynamics/test_driven_coordinate.py:429`（native 既有的 “rank deficient” 拒绝测试） |
| `compute_static_wheel_loads` 自身的 `rank == 3`（A2 保留实现，非被删实现） | `tests/physics/test_vehicle_physics.py:14,77`（任务书要求该文件保留不动） |
| `core/constraints.py` 的残差/Jacobian 内核（保留实现，见 `raw/step3_deletion_record.md` 第 3 节） | `tests/core/test_constraints.py`（未改动） |

## 3. 门禁影响

`tests/` 目录本体保留，稳定验证命令不变；测试总数 `736 passed / 47 skipped / 1 xfailed`（删除前）
→ `737 passed / 47 skipped / 1 xfailed`（删除后）：新增 5 条 native 契约测试，删除 4 条
（`tests/core/test_reactions.py` 2 条 + `tests/model/test_dynamic_mass.py` 2 条），**无新增失败**。
