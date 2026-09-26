# 13 独立终局验收

## Recovery

- 任务：`13 独立完成A1至A10终局验收`。形态：single-full。依赖 12（DONE）。
- 状态见 `../../SUBTASKS.csv` 第 14 行；输入规格 `../../TASKS.md` 第 13 节。
- 主验收：`uv run --no-sync python packages/suspension_multibody/scripts/accept_composable_architecture.py --strict`。
- 本轮**不改实现**：新增的 `scripts/accept_composable_architecture.py` 是运行器本身，属本任务交付；其余改动只有两处指向旧路径的散文注释同步。

## 做法

运行器分两类检查，且刻意不复用子任务的测试文件作为证据：

- **probe**（A1-A10 各一个，共 10 个）：在本文件里直接构造场景并执行，自带断言。这样某个子任务测试写错时不会同时掩盖树的问题。probe 构造不出来（缺 native、缺自测可执行文件）一律**失败**，不 skip。
- **selection**（每个 A 项一份 pytest 选择）：跑该场景的测试文件集合。`--strict` 下选择里出现 skip/xfail 即失败。
- **cmd**（冻结命令集）：A9/A6/A7 相关的数值与分层门按 `tasks/01-baseline/COMMANDS.json` 原文执行。
- **native 前置**：先跑 `build_axle_native.py`，再核对两个镜像哈希一致。这是真实教训换来的：一次半成品重构建（内核镜像已写、multibody 副本因文件被占用没写成）会让 loader 的新鲜度门拒绝加载，随后 8 个互不相关的场景报同一条「镜像过期」——把这个真正的原因单独说一次，比让 8 个场景各自报症状有用。

`--strict` 的语义：拒绝缺失前置；拒绝选择里的 skip/xfail。退出码即结论。

## 关键实测

```text
uv run --no-sync python packages/suspension_multibody/scripts/accept_composable_architecture.py --strict
OK: 29 acceptance items passed        （退出码 0）
```

29 项 = native 前置 1 + selection 10 + probe 10 + 冻结命令 8。

| 项 | 实测证据 |
|---|---|
| native | 两个镜像同哈希（`9189e02c6181a7e0`，2200439 字节） |
| A1 | 4 种导入顺序 + 8 个低层模块**单独**导入；无一拉入上层（`templates`/`subsystems`/`rigs`/`connections`/`preparation`/`simulation`/`kernel`/`report`） |
| A2 | 拖曳臂 2 约束 vs 双横臂 13，joint kinds `{RevoluteJoint}`；准静态 3 states 全收敛、与独立推导旋转公式最大偏差 0.0218 mm；**动态实际求解** 1 case 3 samples 有限（轮心 z −0.144334 m） |
| A3 | 台声明 2 bodies/1 joint/2 forces；能力分支 wheel-supplying `[bench_frame, wheel_carrier_L, wheel_carrier_R]` vs vehicle-loading `[bench_frame]`；台弹簧进入求解并载荷 `(0, 0, 1250, −750000, 0, 0)`；9 个核心源文件在扩展前后哈希不变 |
| A4 | 硬点 +20 mm 精确传到连接点（450→470）；扰动改变求解响应（外倾角变化）；端口世界位姿与独立旋转矩阵合成一致（1e-9）；`solve_mount` 接受独立算出的位姿 |
| A5 | 两种 schema 同一物理输入 → 同一 SI 模型：10 bodies/13 joints/总质量 900.0 kg 一致；单轴总成两侧都不建轮体；来源链 `axle→simulation_assembly`、`chassis/steering/suspension/wheel→subsystem:*`；组合指纹 `a2f96c247eed58bb`；单轴点名拒绝 brake/drive；跨类别的 `ride_four_post` 点名拒绝；有转向时 3 个台驱动全保留 |
| A6 | 同一组合两种研究同一指纹；轮胎激活分别为 `vertical_only` 与 `full`；名为 `acceptance_bench` 的台路由到 `kc_quasi_static`；**垂向轮胎真在残差里**：同一 20 mm 行程下法向力 k=200→k=400 变化且驱动压缩量不变 |
| A7 | 三个 C++ 自测实跑：`mb_contract` 39 checks、`mb_cases` 41 checks、`mb_tire` 123 checks；`contract_case_family_in_protocol` 区分两种拒绝 |
| A8 | 无转向总成 7 bodies、无 rack/拉杆；结果通道 `['wheel_travel_left', 'wheel_travel_right']` 无 `rack_displacement`（不是填零）；有转向仍保留该通道；台接口收缩 `dropped=['rack_drive']`；brake/drive 点名拒绝 |
| A9 | K 经公共入口写盘并回读 manifest 一致；C 收敛；接触点驱动仍被拒（`ValueError`）；每个 state 带诊断记录 |
| A10 | 发布探针 4/4：迁移清单、3 个文档示例执行、文档根一致、3 wheel 构建+离线安装+源码路径外 native 收敛（残差 3.490e-07） |
| cmd:dynamic_hash | 逐位不变（`e7407656731ed556efc28fb89d8fc69881725b3bfb2f39066eb898986389d48e`） |
| cmd:kc_parity | 容差内无漂移 |
| cmd:case_parity | 8 families PASS |
| cmd:kc_perf | k-100 x0.668、c-66 x0.656，预算内 |
| cmd:kernel_layering | `--strict --final` 退出码 0 |
| cmd:legacy_surface_gate | migration 模式退出码 0 |
| cmd:ruff / cmd:ty | 均退出码 0 |

## 过程中发现并处理的问题

1. **A1 probe 最初写错判据**：它断言「任何导入顺序都不得加载 preparation 链」，等于要求「任何东西都不许导入任何东西」。改为只对**低层模块单独导入**断言不得触达上层——那才是依赖方向的定义。
2. **A6 probe 最初断言错对象**：它比较两次运行的**压缩量**，而压缩量在本场景是被驱动量，当然是同一个数（0.04 vs 0.04），于是报「轮胎不响应」。轮胎律移动的是**力**；改为断言法向力随刚度变化、并额外断言驱动压缩量确实不变（后者正是「力比较发生在同一几何下」的保证）。这个失败是我写错判据，不是缺陷。
3. **A7 probe 最初用错文档外壳**：`kind` 写成族名，内核读成「expected a multibody-case document」。修正为 `kind: "case"` 后，两种拒绝分明且可引用原文。
4. **A4 probe 最初用错约定**：`SE3` 的第二个字段是四元数，`rotation` 是派生属性；我按「旋转矩阵」传参导致 `quaternion must contain four finite values`。
5. **A10 selection 曾被 A10 probe 干扰**：probe 会构建并安装 wheel，与selection 并发时导致后者偶发失败（单独重跑 151 passed）。已把顺序改为**先 selection 后 probe**。
6. **`run_contract` 直连违反公共 API 边界门**：A7 probe 最初直接调内核入口，触发 `test_public_api_boundary_gate` 的严格 allowlist（该 allowlist 为空且模式为 strict）。改为经 `run_compiled` 走唯一被授权的提交归属者。

第 6 条值得记：`--strict` 运行器本身如果不遵守仓库自己的架构门，就等于用新的违规去验收架构。这也是为什么选择集里包含 `tests/architecture`。

## 未做的事（如实登记）

- 未把 `legacy_surface_gate --final` 记为通过。它仍要求 `elements` 消失，而该包尚有一个现役 import（`api.py` 的 `evaluate_generalized_forces`）；理由与解除条件在 `packages/suspension_multibody/README.md`。
- 未重录任何数值或性能基线。`dynamic_hash`/`case_parity`/`kc_perf` 全部按冻结值判定。
- 未修改实现来让任何一项通过。

## 后续变更（本任务验收之后，由 A1/A2 收尾带来）

以下都不是本任务做的，是之后 A1/A2 迁移的结果；记在这里是为了让 13 的结论与实际树保持一致。

- **`analysis/`（A2）已删除，其旧导入路径随之消失。** 两个构造迁到 `vehicle/static_loads.py` 与 `vehicle/roll_centers.py`（整车级派生量，只读装配，无需原先登记的扩 ABI）。`suspension_multibody.analysis` 从此 `ModuleNotFoundError`。
- **用户已明确接受旧导入路径消失**（`suspension_multibody.analysis`，以及 `elements` 下的 `LinearSpringElement`/`BushingElement` 等类：这些类迁至 `modeling/primitives/elements.py`，`elements` 包只剩 `evaluate_generalized_forces` 一个函数）。决定在 2026-09-25 由用户作出，属于 EPIC 要求的「公开 API 变更须先确认」，已确认。
- **A1 的固定体端阻塞点已处理**：`cpp/src/element/assembly_primitives.cpp` 不再对固定体早退，改为「求解累加器仍不写、但把该端记录进 element-wrench sink」。那是把事实记下来而不是改变求解，因此 `dynamic_hash` 重跑后仍逐位不变（`e7407656…`）。
- **A1 尚未收尾**：`elements/` 仍在，因为 KC 契约从不发射弹簧/减振器/横向稳定杆，native 手里没有这些事实；现在解码会静默丢掉报告一直有的行。这是新功能（内核 + 契约），不是清理。
- **`BASELINE.json` 已于本次重采**（内核 `.cpp` 改动 + native 重建所致）。`frozen_baselines` 七个键逐项相同，4 个冻结数值基线文件 `sha256sum -c` 全部 OK；只有 7 个描述性叶子键变化（库哈希/大小、内核源指纹、git status）。

## 验收中发现、但**不在本 EPIC 范围内**的一处缺陷（如实登记，未修）

`adapters/geometry_contract.py::front_axle_model_from_contract` 把合约角色映射成
`upper_front` / `lower_front` / `tierod_inner` 等键名，但
`subsystems/geometry.py::HARDPOINT_ALIASES` 的别名表**不含** `UPPER_FRONT`/`LOWER_FRONT`
（它列的是 `UPPER_INBOARD_FRONT`、`UPPER_INNER_FRONT`、`UCA_FRONT`、`UCA_INNER_FRONT`）。
后果：该公开适配器产出的模型**无法进入装配**——`build_front_axle` 报
`missing required front-axle hardpoint for upper_front`。

实测（本任务 A5 探路时命中，随后改用 benchmark 夹具走 A5 正式路径）：

```text
front_axle_model_from_contract(contract, mass=...)  →  assembly 时报
ValueError: missing required front-axle hardpoint for upper_front
```

为何不修：

- 该缺陷**先于本 EPIC 存在**，`git show HEAD:` 的适配器与别名表与本轮逐字相同，本轮未触及这两个文件；
- 修正方式是改适配器发出的键名或扩别名表，两者都会改变一个**公开适配器**的输入/输出契约；
  按 EPIC「不默认更改公开 API…必要变更必须先独立确认」，这属于须先向用户确认的事项，
  不能由一个验收任务顺手改掉；
- 现有测试只断言适配器**产出的键与坐标**（`tests/adapters/test_geometry_contract.py`），
  没有任何测试把它接到装配上，所以这个缺口没有被既有门禁覆盖。

建议的处置（留待用户裁决）：把 `_ROLE_TO_HARDPOINT` 的目标键改成别名表已接受的拼写
（`uca_front` 等），或给别名表补 `UPPER_FRONT`/`LOWER_FRONT`/`TIEROD_INNER`/`TIEROD_OUTER`；
并补一条「合约模型能装配并求解」的端到端测试，否则该适配器仍是一条无验收的死路。

## 后续变更（力元件三拆之后）

上面「适配器硬点别名缺口」一节的结论已过期：该缺口**已修复**，不再是遗留项。

处置与证据：

- `subsystems/geometry.py::HARDPOINT_ALIASES` 补入 `UPPER_FRONT`/`UPPER_REAR`/`LOWER_FRONT`/`LOWER_REAR`
  四个拼写（纯新增，未改动任何既有拼写，也未改适配器的输出契约）。
- 新增两条测试：`tests/adapters/test_geometry_contract.py::test_the_alias_table_resolves_every_role_the_assembly_asks_for`
  与 `::test_a_contract_derived_model_assembles_and_solves`。后者是原先缺失的端到端验收：
  合约模型 → 装配 → 原生求解。
- 可证伪性已实测：撤掉那四个别名 → 两条测试失败（`missing required front-axle hardpoint for upper_front`）；
  加回 → 4 passed。

同一批「力元件三拆」改动带来的验收面变化，一并登记：

| 项 | 变化 | 原因 |
|---|---|---|
| `kAxleKernelAbiVersion` | 15 → 16 | `AxleInput` 的融合弹簧字段组拆成三组 |
| `kVehicleKernelAbiVersion` | 30 → 31 | `VehicleInput` 按值内嵌 `AxleInput` |
| `kCoreKernelAbiVersion` | 1（不变） | 通用 core 面未触及 |
| `AxleInput` 字段数 | 106 → 129 | 同上；`VehicleInput` 保持 97 |
| `kElementBlockSize` | 176 → 216 | 追加 damper / bump_stop 两个家族 |
| `spring_output` | 宽 7 → 宽 4 | 加上新的 `damper_output`(4) / `bump_stop_output`(5) |
| `kElements` 表 | 9 → 10，去 `spring_damper` | `spring`/`damper`/`bump_stop` 三个名字 |
| `dynamic_hash_baseline.json` | 重录一次 | 已授权的哈希重录；`frozen_baselines` 其余 6 项逐字不变 |
| `case_parity` 快照 | 重录一次 | 同上；脚本新增 `--record`，且仅在全部家族通过时才写 |

物理未被改动的证据（这是本批改动唯一需要证明的事）：

1. `tests/axle_dynamics/test_three_laws_match_the_fused_record.py`（新增 4 条）把三条力律各自的
   报告值与融合记录的公式逐项对齐，并断言三者之和等于滑动块的 `ma`。
2. 静态算例 `static_equilibrium` 的 `states` 与冻结快照**逐位相同**（无积分放大环节）。
3. 粗网格动态算例对**末位扰动**（弹簧刚度 +1 ulp，相对 1e-16，小于本次拆分的 1e-13）
   的响应为 4.9e-5 ~ 5.4e-2，与拆分带来的位移同量级——即差异来自混沌放大而非力律改变。
4. 细网格（`native_refined_result`）与既有产物最大差 4.06e-10。
