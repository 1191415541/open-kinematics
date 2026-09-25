# 06 子系统迁移与唯一仿真总成

## Recovery

- 任务：`06 收口子系统与SI仿真总成`。形态：single-full。依赖 05（DONE）。
- 状态见 `../../SUBTASKS.csv` 第 7 行；输入规格 `../../TASKS.md` 第 06 节。
- 主验收：`uv run --no-sync pytest packages/suspension_multibody/tests/subsystems packages/suspension_multibody/tests/studies packages/suspension_multibody/tests/vehicle_assembly packages/suspension_multibody/tests/tire_mass -q`

## 实测现状（进入本任务时）

- `subsystems/`（1,662 行）+ `preparation/assembly/`（1,556 行）：六类子系统是**函数**，按固定顺序改写共享可变的 `SubsystemContext`。顺序是承重的：底盘先建，悬架再建（转向的拉杆要够到 upright），弄错顺序得到的是半成品模型而不是错误。
- 装配只以「运行这些函数」的副作用形式存在，没有可检查的值。

## 做了什么

### 组合层（`subsystems/composition.py`）

- `SubsystemContribution`：一份贡献 = role + 既有 `SubsystemOutput`（**原样**）+ 它提供的端口 + 它的需求。刻意不重写子系统逻辑，这样新旧两条路径才可比。
- `compose_simulation_assembly`：先收齐全部贡献，**再**用 04 的匹配器解析需求。因此调用方列出子系统的顺序只影响记录顺序，不再决定成败。
- 重复 role 拒绝：一个角色一份贡献，实体来源因此可追。
- `body_order` 参数：契约文档按 `assembly.bodies` 的插入顺序输出，所以记录顺序必须显式复现（chassis → rack → 每侧四体），而不是听任合并顺序。列了不存在的刚体 → 报错。
- `fingerprint_assembly`：结构指纹，**含几何与质量**、**与顺序无关**。

### 装配级完整性（`modeling/assembly.py`）

- 修正了 02 引入的一条过严校验：fragment 的点/端口**允许**引用其他贡献拥有的刚体（悬架的点落在底盘与转向拉杆上，这是真实建模事实）。改为 `ModelFragment.unresolved_point_bodies()` / `unresolved_port_owners()` 报告，由 `SimulationAssembly` 在**合并后**统一检查。fragment 是部分视图，装配才是完整模型——检查放在正确的层级。

### SI 入口（`subsystems/si_assembly.py`）

`si_assembly_for_axle(model, request=...)`：调用既有六类子系统产出 contributions，再组合为 `SimulationAssembly`。本步**不**切换生产运行入口（那是 07），也不发出契约文档——避免「迁移一半、两条活路径」。

## 关键实测：新旧逐项一致

| 判据 | K 模式 | C 模式 |
|---|---|---|
| bodies 与记录顺序 | 一致 | 一致 |
| points 键集 | 一致 | 一致 |
| 约束数 | 13 = 13 | 9 = 9 |

无转向请求下：`rack` 与 `tie_rod_*` **整体消失**（不是退化体）——浮空的 rack 会污染能力判定，让试验台以为总成可转向。

## 修掉的两个真实缺陷

1. **指纹不含几何**：改硬点后指纹不变，会把两个不同模型报成同一个。已纳入 points（12 位有效数字）与 mass。
2. **指纹随收集顺序变化**：`walk()` 顺序受 contributions 顺序影响。改为按 level 名排序。
3. 附带：compose 内部传给指纹的 capabilities 与外部调用不一致，导致同一装配两个指纹。已统一为不带 capabilities——指纹是**模型**属性，谁问都该同值。

这三条都是测试先失败、再定位、再修，不是事后补写。

## 门禁设计（`tests/subsystems/test_si_composition.py`，16 项）

| 断言 | 说明 |
|---|---|
| K/C 两模式 bodies 顺序一致 | 顺序进入契约文档，集合相同不够 |
| points 键集一致 | |
| 约束数 (13, 9) 保持 | C 保留拉杆，不是丢弃关节 |
| 无转向时 rack/拉杆消失 | 不是退化体 |
| 角色报告正确 | 不含 brake/drive |
| 指纹结构且稳定 | 同模型同值；改硬点变值 |
| 指纹与收集顺序无关 | 顺序不是输入 |
| 重复角色/未知角色/空组合拒绝 | |
| body_order 含不存在刚体时拒绝 | |
| 结果是 SimulationAssembly，root_kind=axle | |
| **不经过旧路径也能组合** | 若暗中依赖旧路径，两条路径就是同一条的两个名字，07 无法切换 |

## 验证记录（实测退出码）

| 命令 | 退出码 | 结果 |
|---|---|---|
| `pytest tests/subsystems tests/studies tests/vehicle_assembly tests/tire_mass -q` | 0 | 主验收 97 passed（含 16 项新增） |
| `python scripts/kc_parity_check.py --check` | 0 | 冻结快照容差内 |
| `python scripts/case_parity_check.py` | 0 | 8 families 全 PASS |
| `ruff check .` | 0 | All checks passed |
| `ty check .` | 0 | All checks passed |
| `pytest tests -q`（全量） | 0 | `1234 passed, 1 skipped, 1 xfailed in 1890.11s`；较 05 的 1214 增 20（06 新增 18 + 02 语义更新 4 - 旧 2），skip/xfail 未增长 |

## 未覆盖与保留

- **生产运行入口未切换**：旧公共入口保持可用，新链路尚未接通编译器。切换与消除 K/C 转动态桥接属 07。
- 轮胎质量在总成中只累计一次的语义未改（既有 `tests/tire_mass` 全通过即证据）；质量守恒的逐位判据仍由该测试持有。
- 前后轴嵌套的整车侧总成复用 06 的组合 API，但整车装配的迁移由后续任务按需接入。

## 过程中发现并修正的一处自身回归

首轮全量回归暴露 2 项失败：02 写的
`test_points_must_reference_a_declared_body` 与 `test_port_owner_must_be_a_declared_body`
断言「点/端口必须引用本 fragment 的刚体」，而 06 有意放宽了该约束（跨贡献引用是真实建模事实）。

处理：这两项测试改为断言**修正后的语义**（fragment 报告 `unresolved_point_bodies()` /
`unresolved_port_owners()`），并新增 2 项装配级测试证明
`SimulationAssembly` 确实会对悬空引用报错、对完整模型放行。不是删测试，是把断言移到正确的层级。
