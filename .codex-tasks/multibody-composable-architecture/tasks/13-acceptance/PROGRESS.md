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

- 未把 `legacy_surface_gate --final` 记为通过。它要求 `elements`/`analysis` 两个仍有现役生产调用、且 native 尚不能承载的包消失；理由与解除条件在 12 的现状文档里（`packages/suspension_multibody/README.md`）。
- 未重录任何数值或性能基线。`dynamic_hash`/`kc_parity`/`case_parity`/`kc_perf` 全部按冻结值判定。
- 未修改实现来让任何一项通过。

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
