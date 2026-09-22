# 子任务 04：从 build_front_axle 拆出四类子系统

## 目标

把现役 `build_front_axle`（`packages/suspension_multibody/src/suspension_multibody/preparation/assembly/front_axle.py:625-926`）拆成四个可独立实例化的子系统，粒度由用户裁决：

用户原话：「子系统是左右悬架、转向、轮胎、车身，左右悬架加转向加轮胎加悬架实验台得到悬架实验总成，前后悬架加转向加车身加轮胎加整车 kc 实验台得到整车实验总成」。

1. 新建 `suspension_multibody/subsystems/` 包，定义四类子系统，各有明确输入与输出对象：
   - **左右悬架（suspension）**：body / point / constraint 的生成，含 `symmetric_proxy` 的 UCA/LCA/upright/tie rod 与镜像逻辑。现状落点：`front_axle.py:53-67`（`mirror_hardpoints` / `side_hardpoints`）、`:655-665`（每侧四体）、`:668-685`（挂点表 `mount_data`）、`:686-750`（UCA 内点，K 用 `RevoluteJoint` `:706-716`、C 用 `BallJoint` `:719-723` + 占位衬套 `:724-739`）、`:751-815`（LCA 内点，K `:771-781`、C `:784-788` + `:789-804`）、`:816-837`（外点 `BallJoint`）。同时归属左右悬架侧的弹性元件：springs `:525-541`、dampers `:542-560`、stops `:575-590`、anti-roll bar `:591-601`（**跨左右**：`left_body="upright_L"`、`right_body="upright_R"`，故只能归"左右悬架"这一含双侧的子系统）。
   - **转向（steering）**：rack 体（`:645-647`）、tie rod 两端连接（实测 `:838-873`：`rack_tie_joint_{side}` `:846-848`、`tie_upright_joint_{side}` `:849-851`）、rack 固定/导轨（`:874-903`：`model.rack_fixed_to_chassis` 为真时 `WeldJoint(rack_fixed_to_chassis)` `:881-888`，否则 `PrismaticJoint(rack_guide)` `:889-900`；仅当 `"rack_housing" not in bodies` 才追加 `:901-903`）。
   - **轮胎（tire）**：`VerticalTireElement`（实测 `:561-574`，来自 `model.tires`，`wheel_body=f"upright_{side}"`、`wheel_center_local` 取自 `wheel_center` 硬点）。
   - **车身（chassis）**：chassis 体与 `MassSpec`。实测：轴侧 `front_axle.py:644` 直接造 `RigidBody("chassis", fixed=True)`，**不消费** `MassSpec`（`schema/model.py:28-60`，字段在 `:165`；`front_axle.py` 只读 `RigidBodySpec.mass`，见 `:174`、`:191`）；整车侧 `preparation/assembly/vehicle.py:99` 的 `build_vehicle` 用 `_body_from_spec(model.chassis)` 造 chassis，再由 `:118-128` 把轴侧 chassis 重命名并入整车体表。
2. 四类子系统各自声明输入（模型 / 硬点 / 上游子系统产物）与输出（`RigidBody` 集合、点表 `dict[tuple[str,str], np.ndarray]`、`Connection`、`Constraint`、`Element`），互不隐含共享可变状态；单独构造不经过 `build_front_axle`。
3. `build_front_axle` 改为**由四类子系统组合**而成，组合顺序复现现役追加顺序；同时**保留为适配器**：签名 `build_front_axle(model, mode="K")` 与返回类型 `FrontAxleAssembly`（`:82-95`）不变，直到 11 之后再定是否删除。
4. 建立"组合路径 vs 现役实现"的逐位一致判据，以及逐项对照表（body 集合与顺序、`points`、`hardpoints`、`connections`、`constraints`、`ideal_constraints`、`bushings`、`elements` 名称序、驱动坐标定义），落 `raw/`。

## 非目标

- 不改 K/C 语义（05 做）：本步不把零刚度占位衬套改成模板属性，`front_axle.py:737`、`:802` 的 `stiffness=np.zeros((6,6))` 保持。
- 不接线模板（03 只建结构，05 接线）：本步不读 `templates/`。
- 不改几何生成规则、硬点别名表（`:114-155`）与镜像规则（`:53-67`、`:229-237`）。
- 不删除 `build_front_axle`。
- 不改 `symmetric_proxy` / `explicit` 两条拓扑的语义（`explicit` 走 `_build_explicit_axle` `:387`）。
- 不实现属性文件（06）与输出声明（07）。

## 约束

- **不得修改 `templates/**`**：那是 03 的写范围，本步只读其结构。
- 本步与 03 都触及 `preparation/assembly/types.py`，**必须串行**，不得并行。
- **产物逐位一致**：组合产物与现役实现必须是 `np.array_equal` 级别的一致（点表逐元素、集合逐顺序），不是"数值近似"。
- **元素顺序不得变**：`_runtime_elements`（`:515-622`）先 `for side in ("L","R")`（`:524`）逐侧追加 springs(`:525-541`) → dampers(`:542-560`) → tires(`:561-574`) → stops(`:575-590`)，循环后追加 ARB(`:591-601`)，再在 C 模式追加 `model.bushings`(`:602-622`)；`build_front_axle` 最后把 8 条衬套再接在尾部（`:910-911`）。轮胎子系统产出必须**插回该位置**，不得追加到末尾。
- **不得新增 legacy import**：`tests/architecture/legacy_surface_gate.py` 的 MODE_MIGRATION 只容忍注册表内的既有条目（`tests/architecture/legacy_surface_registry.json` 的 `entry[1]` = `preparation/assembly/front_axle.py` 导入 `elements`，`entry[2]` = `vehicle.py`）。新模块导入退役的 `elements` / `core` / `model` / `analysis` / `metrics` 会成为未注册 finding 并失败。因此新 `subsystems/` 包**不得直接导入 `elements`**：元素构造留在已注册的 `front_axle.py` / `vehicle.py`（子系统返回声明数据，元素在注册文件里构造），且 `legacy_surface_gate.py --check` 必须**在不新增注册条目**的前提下保持绿。
- 不引入新依赖。
- **不得重录任何基线**：`tests/data/kc_baseline/**`、`kc_perf_baseline.json`、`kc_perf_baseline_native.json`、`dynamic_hash_baseline.json`、`axle_dynamics_baseline/`、`vehicle_dynamics_baseline/`、`packages/suspension_kernel/layering_baseline.json` 全部保持原字节。

## 范围与文件归属

- 可写：
  - 新增 `packages/suspension_multibody/src/suspension_multibody/subsystems/**`
  - `packages/suspension_multibody/src/suspension_multibody/preparation/assembly/front_axle.py`（改为四类子系统组合；保留 `build_front_axle` 签名与 `FrontAxleAssembly` 字段）
  - `packages/suspension_multibody/src/suspension_multibody/preparation/assembly/vehicle.py`（chassis 侧接线；`build_vehicle` 行为不变）
  - `packages/suspension_multibody/src/suspension_multibody/preparation/assembly/__init__.py`（仅当需要导出新符号，且不得改变现有导出）
  - 新增测试 `packages/suspension_multibody/tests/subsystems/**`
  - 对照表与快照证据：`tasks/20260922-04-subsystems/raw/**`
- 只读：`templates/**`（03）、`schema/**`、`cases/**`、`elements/**`、`preparation/assembly/types.py`、父级 `EPIC.md` / `SUBTASKS.csv`。
- 不写：`templates/**`、`outputs/**`（07）、`report/**`（07）、`tests/data/**`（基线）、`packages/suspension_kernel/**`、父级计划文件（归主代理）。

## 依赖

- 前置：03（模板与双列连接点数据结构；本步只读其结构，不接线）。02 已给出统一副表与副编码。
- 后续：05（模板实例化与 K/C 列激活——在 04 的子系统之上做实例化）。

## 验收标准

1. 四类子系统可独立实例化：每类有自己的构造函数与明确的输入/输出对象，测试可在不经过 `build_front_axle` 的情况下单独构造成功。
2. **与现役产物逐项对照，`benchmark_axle.json` 基线上差异为零**。本 SPEC 制定时实跑的现役值（实施时先复核再对照）：
   - `bodies`：10 个，顺序 `['chassis','rack','upper_arm_L','lower_arm_L','upright_L','tie_rod_L','upper_arm_R','lower_arm_R','upright_R','tie_rod_R']`；
   - `points`：36 条；K 与 C 两模式的键序与数值完全一致；
   - `connections`：16 条，顺序 `['uca_mount_L_inner_front','uca_mount_L_inner_rear','lca_mount_L_inner_front','lca_mount_L_inner_rear','upper_arm_L_outer_joint','lower_arm_L_outer_joint','rack_tie_joint_L','tie_upright_joint_L']` 后接同序的 `_R` 组；
   - **K 模式**：`constraints=13` / `ideal_constraints=13` / `bushings=0` / `elements=0`；`constraints` 顺序为每侧 `[RevoluteJoint:uca_mount_{S}_inner_front, RevoluteJoint:lca_mount_{S}_inner_front, BallJoint:upper_arm_{S}_outer_joint, BallJoint:lower_arm_{S}_outer_joint, BallJoint:rack_tie_joint_{S}, BallJoint:tie_upright_joint_{S}]`（S=L 后 R），末尾 `PrismaticJoint:rack_guide`；
   - **C 模式**：`constraints=9` / `ideal_constraints=17` / `bushings=8` / `elements=8`；`ideal_constraints` 顺序为每侧 `[uca_mount_{S}_inner_front, uca_mount_{S}_inner_rear, lca_mount_{S}_inner_front, lca_mount_{S}_inner_rear, upper_arm_{S}_outer_joint, lower_arm_{S}_outer_joint, rack_tie_joint_{S}, tie_upright_joint_{S}]`，末尾 `PrismaticJoint:rack_guide`；`elements` 与 `bushings` 名称序为 `uca_bushing_{S}_inner_front, uca_bushing_{S}_inner_rear, lca_bushing_{S}_inner_front, lca_bushing_{S}_inner_rear`（L 后 R）；
   - `hardpoints`：模型硬点 + 每侧 `{name}__{side}` 副本（`:666-667`）+ `RACK_CENTER`（`:904-906`）。
   任何一项对不上：要么改实现，要么在 `raw/` 的对照表逐项登记理由（只允许"顺序由上游定义、数值不变"这类说明，不允许"近似即可"）。
3. **组合路径与现役实现逐位一致**（纯重构判据）：`build_front_axle(model, mode)` 在组合实现下与本步改动**之前**现场抓取并留在 `raw/` 的现役产物逐位一致；至少覆盖 K、C 两模式与 `rack_fixed_to_chassis` 真/假两个分支。
4. **驱动坐标定义不变**：kc 的驱动坐标由装配点表推导（`cases/kc_quasi_static/contract.py:177-222`，`:209` 取 `assembly.point("upright_{side}","wheel_center")`、`:219` 取 `assembly.point("rack","center")`），因此 `points` 的键与值逐位一致即证明驱动坐标定义一致；有测试断言。
5. `explicit` 拓扑路径行为不变：`_build_explicit_axle`（`:387`）与 `_runtime_elements_explicit`（`:314`）的产物逐位一致。
6. 现役门禁保持绿且**未重录任何基线**：`kc_parity_check.py --check`、`case_parity_check.py`、`dynamic_hash_sentinel.py --check`、`check_module_layering.py --strict --final`、`legacy_surface_gate.py --check`（不新增注册条目）、三套 pytest、ruff、ty、`git diff --check`。
7. **未消费事实如实登记**：`MassSpec` 在轴侧装配中从未被读取（`schema/model.py:165` 定义、`front_axle.py` 不引用），车身子系统必须保留这一差异并在 `raw/` 登记，不得顺手"修正"为消费 `MassSpec`。

## 验证协议

1. 子系统逐个落地，每落一类立刻跑一次 `tests/subsystems` 中该类测试（禁止攒到最后一起验）。
2. 每拆一类，立即与该类在现役实现中的产物逐项对照（body / 点 / 约束 / 连接 / 元素位置）。
3. 组合完成后跑逐位一致测试（K、C、`rack_fixed_to_chassis` 三组）。
4. 门禁顺序：`build_axle_native.py` → `kc_parity_check.py --check` → `case_parity_check.py` → `dynamic_hash_sentinel.py --check` → `legacy_surface_gate.py --check` → `check_module_layering.py --strict --final`。
5. 收尾：三套 pytest、`ruff check .`、`ty check .`、`git diff --check`，并确认 `tests/data/**` 与 `layering_baseline.json` 无 diff。

**本步是纯重构：任一基线发生变化即停止并上报**——那说明拆子系统时误改了产物，而不是"预期重录"。
