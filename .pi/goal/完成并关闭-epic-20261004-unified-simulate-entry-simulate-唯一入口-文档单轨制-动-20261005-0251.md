# Goal

把 `.codex-tasks/20261004-unified-simulate-entry/` 这个 10 行 Epic 推到**该 Epic 自身规则允许的终态**，并给出终局验收证据：用户要求 `simulate` 成为唯一公共仿真入口、覆盖全部 7 个 bench；退役 `run_case`/`run_dynamic_case`/`run_axle_dynamics`/`run_vehicle_dynamics` 四个分散入口；v1 模型文件彻底废弃；SI 建模路线（`AxleDynamicsModel`）退出公共 API（单轨制）；动力学分层与 K&C 一致且建模数据无损进入运行时；允许重录 `gate-numeric` 动态基线，但必须以 01 冻结的迁移前快照逐位对照。

`SUBTASKS.csv` 是状态真源。每行落终态时必须与该行 `SPEC.md`/`TODO.csv` 的验收条件和 `EPIC.md` 的 Done-When A–H、G1–G8 对齐；**凡无法在 Epic 自身规则下达成的行，不得标 `DONE`，必须标为 blocked 并在 Epic `PROGRESS.md` 与 `tasks/10-final-acceptance/` 单列，Epic 相应不宣告完全关闭**。

# 起点实测（本会话亲自核实，非引用旧结论）

- 01–05 已 `DONE`；06 交付物齐备但状态为 PARTIAL/FAILED；07–10 未开工。
- `just check-fast` 退出码 0：快速集 **1377 passed, 1 xfailed**；kernel 47 passed；contracts 32 passed。skip 未出现，xfail 恒为 1（未增长）。
- `dynamic_hash_sentinel.py --check` 26 个 artifact 逐字节一致，combined sha256 = `fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9`，与 01 冻结值相同。
- `kc_native_probe.py` + `kc_native_c_probe.py` + `kc_parity_check.py --check --actual-dir` 实跑通过（worst ratio 1.66e-05 / 1.86e-04）。
- 工作区有 157 项未提交改动（含上一轮 Epic 遗留），属已知现状。

# 已识别的两个真实阻断项（须在终局记录中单列）

1. **06 的防倾杆（D1）**：装配侧 `AntiRollBarElement`（`modeling/primitives/elements.py:601`，垂向行程差 × 刚度）与 SI `AxleAntiRollBar`（`axle_dynamics/schema.py:921`，两体轴向扭转）是**不同物理模型**；后者全仓从未被生产代码构造。任何映射都需一个物理换算并会改动数值，与「数值逐位一致」冲突。
2. **06 的 PAC2002 轮胎（D2）**：`studies/bridge.py:_vertical_tire` 只给竖向刚度 + 占位摩擦/刷子刚度，装配侧无 PAC2002 参数存储通路；K/C 为准静态不受影响，动力学轮胎则不是模型描述的轮胎。

按用户「裁决交给 review 子代理，不用问我」的授权，我将据此在契约中落定裁决口径（见下），**不擅自改动物理**：判据是「已核实的缺口必须点名拒绝，不得静默丢弃」，而 `_refuse_unreadable_elements` 已满足该性质。

# Acceptance criteria（每条可客观判定）

1. **`simulate` 覆盖全部 7 个 bench**：`kc_quasi_static`、`axle_dynamic`、`vehicle_kc`、`vehicle_dynamic`、`handling`、`ride_four_post`、`ride_random_road` 逐个（不抽样）经 `api.simulate` 从**文件路线**与**内存路线**实跑；两路线的编译契约逐字段对照有实测证据。内存路线删除项目目录后仍可实跑。
2. **`.result` 不再恒为 `None`**：`simulate` 返回的 `SimulationRun.result` 在可解码的 family 上承载领域结果（`ResultBundle`/`AxleDynamicsResult`/`TimeSeriesResult`），并有测试断言；不可解码的组合必须**点名拒绝或显式说明**，不得静默返回 `None`。
3. **拒绝口径**：`simulate` 不再按 kind/family 字面量拒绝；不支持组合由注册表与策略判定并点名拒绝（含正例与负例两类测试）。
4. **提交边界唯一**：单次 `simulate` 调用只编译并提交一次契约（无重复编译/提交），有测试佐证。
5. **消费者迁移（08）**：`tests/axle_dynamics/` 的 91 个 test 按 R2=b 改为经文档对象与 `simulate` 驱动，**skip/xfail 不增长**，`test_cpp_physics`、`test_contact`、`test_energy`、`test_solver_invariants`、`test_integrator` 的原断言（积分精度、接触事件、能量守恒、不变量）有等价新测试承接或**按失败门回流并单列缺口**；`scripts/dynamic_hash_sentinel.py`、`scripts/case_parity_check.py`、`scripts/run_axle_dynamics_acceptance.py` 迁到文档路线；逐文件给出迁移前后 test 数与断言对照表。
6. **退役（09）**：`run_case`、`run_dynamic_case`、`run_axle_dynamics`、`run_vehicle_dynamics` 的生产定义、公开导出、注解与导入在 `axle_dynamics/` 之外零命中；`AxleDynamicsModel` 退出公共建模面（`__all__` 条目、`axle_dynamics/__init__.py` 显式 import 绑定、`load_axle_dynamics_model` **三条路径各有负例**）；退役 AST 门含注入负例与「解释性注释不误报」负例；`cli.py` 各命令按 01 的 `CLI-COMMANDS.csv` 逐条只经 `simulate` 或明确的非仿真工具路径，且有实跑证据。
7. **数值门与基线**：`gate-numeric` 三条全过。动态 SHA 若重录，必须先以 01 迁移前快照逐位对照、差异逐条判定（仅批准允许的动态差异），证据存 `tasks/09-retire-direct-entry/raw/` 与 `tasks/10-final-acceptance/raw/`；**`tests/data/kc_baseline/{k_states,c_states}.json` 禁止重录**，且 `git status --short -- packages/suspension_multibody/tests/data/kc_baseline/` 为空。
8. **全量回归**：`pytest packages/suspension_multibody/tests -q`（含 `tests/architecture`）全绿；`ruff check .`、`ty check .`、三个架构门脚本退出码均为 0；skip/xfail 不增长。
9. **终局验收（10）**：Done-When A–H 与 G1–G8 逐项有**独立可执行检查器**及其退出码（不得以 `echo` 或「子任务都 DONE」代替）；H 的覆盖登记核验为**失败门**。
10. **文档一致性**：`SUBTASKS.csv` 每行 `status`、`tasks/<id>/TODO.csv`、各 `PROGRESS.md`、`EPIC.md` 终态与实测数字一致；blocked 项在 Epic `PROGRESS.md` 与 `tasks/10-final-acceptance/raw/` 单列原因与证据。
11. **`plan_check.py` 结构校验**：11 列、ID/目录唯一、依赖无环、status 合法、7 bench 与 G1–G8 均有承接，退出码 0。

# Boundaries

- 不新增求解器、轮胎力律、family、bench、ABI 字段、第三方依赖；不放宽任何 bench 的物理能力边界。
- **不擅自做物理建模决策**：D1/D2 的映射若会改动物理数值则不实施；按「不可无损表达 → 点名拒绝 + 单列缺口」处置，并在终局记录中如实报告这一结论而非包装为已完成。
- 不重录 `tests/data/kc_baseline/**`；不重录 `kc_perf_baseline_native.json`（预算门须单独审批）；不新增 skip/xfail；不用 `-k`/`--deselect` 长期豁免失败；不用不带 `--actual-dir` 的 `kc_parity_check.py` 充当证据。
- 保持分层方向与既有架构门（`tests/architecture/test_no_production_bridge.py` 等）绿；`kernel/` 提交唯一归属不变；不重构 `kernel/` 内部算法与 `compilation/` 路由机制。
- 串行执行 06→07→08→09→10，不并发改同一文件；不改与本 Epic 无关的既有遗留改动。
- 每步以实测为准：做了什么、改了哪些文件、验证结果如实报告；未验证不声称已验证。

# 说明

本契约在我因工具集限制无法直接改文件的阶段提交。批准后我将按依赖顺序推进 06 收口→07→08→09→10，每步重跑门禁，直到全部可达成行落终态、Done-When 有实测证据，或出现真实阻断项（届时如实上报并把对应行标为 blocked，Epic 不宣告完全关闭）。