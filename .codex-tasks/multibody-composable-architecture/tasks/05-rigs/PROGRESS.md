# 05 内置物理试验台模板

## Recovery

- 任务：`05 迁移内置物理试验台模板`。形态：single-full。依赖 04（DONE）。
- 状态见 `../../SUBTASKS.csv` 第 6 行；输入规格 `../../TASKS.md` 第 05 节。
- 主验收：`uv run --no-sync pytest packages/suspension_multibody/tests/rigs -q`

## 实测现状（进入本任务时）

- `rigs/` 共 441 行：`rig.py`（190 行，7 个 `RigSpec` 声明）、`compose.py`（210 行，收缩与配对）、`__init__.py`（41 行）。
- `RigSpec` 只有 `drives`/`outputs`/`study`/`supplies_wheels`/`description` —— **不产出任何实体**。「试验台也是子系统，也由刚体、运动副、运动、力组成」此前只是文档里的一句话，内核从未见过。
- `_RIG_ASSEMBLIES` 把 7 个 rig 硬绑到 axle/vehicle；试验台差异靠**名字**表达，而非能力。

## 做了什么

### 试验台能力（`rigs/bench.py`）

- `BenchCapability` 两个取值：`wheel_supplying`（单轴台自有机轮）与 `vehicle_loading`（整车台只加载，不造车轮）。
- 能力**由 `RigSpec.supplies_wheels` 派生**（`_BENCH_CAPABILITY_BY_NAME`），不重复声明——一个台不能在一处声称有机轮、在另一处不声称。
- 分支依据是**能力**，不是总成名或悬架模板名。这正是 G3 要的「自适应」：新增总成类别不需要新增试验台代码。

### 真实实体产出

`build_rig_fragment` 实测（非声明，是实际构造）：

| 试验台 | 能力 | bodies | joints | forces | tires | drives |
|---|---|---|---|---|---|---|
| kc_quasi_static | wheel_supplying | 3 | 2 | 2 | 2 | 2 |
| axle_dynamic | wheel_supplying | 3 | 2 | 2 | 2 | 3 |
| vehicle_kc | vehicle_loading | 1 | 0 | 0 | 0 | 0 |
| vehicle_dynamic | vehicle_loading | 1 | 0 | 0 | 0 | 1 |
| handling | vehicle_loading | 1 | 0 | 0 | 0 | 2 |
| ride_four_post | vehicle_loading | 1 | 0 | 0 | 0 | 4 |
| ride_random_road | vehicle_loading | 1 | 0 | 0 | 0 | 1 |

轮供给型每侧构造：`wheel_carrier_{L,R}` 刚体、`contact` 点、`carrier_{side}` 平移副、`tire_force_{side}` 力元、`travel_{side}` 运动。单轴总成不建车轮（D9），所以若试验台不建就**根本没有车轮**——这不是可选项。

整车型只贡献 `bench_frame` 固定体与其自有运动（road_height / steering_wheel_angle）。**不造车轮**：整车已拥有车轮，再建一个会把质量与力路径算两遍，正是全局规则禁止的事。

### 端口

- 轮供给型提供 `wheel_centre_L` / `wheel_centre_R`，带 `L`/`R` 标签（防左绑右）与 `wheel`/`load` 能力。
- 整车型提供 `mount`（`body_mount`），它推车身而非接车轮。
- 两者的选择来自**同一个能力**，不会与实体产出不一致。

### 其它

- `build_rig_assembly` 产出 `Assembly`，**`root_kind` 留空**：全局规则只对被测体检查，给试验台一个 kind 会让它被判定两次（且单轴规则会错误地抱怨一个合法拥有车轮的台）。
- provenance 记录 `template=bench`、`revision=能力`、`instance=路径`，供 A5 追溯。
- `mode` 参数被接受但改变不了台的内容（台的几何与 K/C 无关），**传而不忽略**：调用方不能忘掉它而在将来拿到另一个台。

## 门禁设计（`tests/rigs/test_bench_template.py`，12 项）

| 断言 | 说明 |
|---|---|
| 每个出厂台都构出片段 | 不允许「只有名字」的注册 |
| 轮供给型真的造出轮/副/胎/力 | 逐项断言实体，非检查注册表 |
| 整车型不造第二个车轮 | 5 个整车型台逐个断言 tires 为空 |
| 能力来自 drives 声明 | 单一真源 |
| 轮供给型提供每侧 wheel_centre | 标签与能力正确 |
| 整车型提供 mount 而非车轮 | 语义正确 |
| 台的 assembly 无 root_kind | 避免被全局规则重复判定 |
| provenance 记录模板与能力 | 可追溯 |
| 自有输入成为台的运动 | 总成应提供的坐标**不**被台发射 |
| 未知台/未知模式点名拒绝 | 可诊断 |
| 两分支贡献确实不同 | 能力是承重的，不是装饰 |

## 验证记录（实测退出码）

| 命令 | 退出码 | 结果 |
|---|---|---|
| `pytest tests/rigs -q` | 0 | 主验收 55 passed（含 12 项新增） |
| `python scripts/kc_parity_check.py --check` | 0 | 冻结快照容差内，无漂移 |
| `python scripts/case_parity_check.py` | 0 | 8 families 全 PASS |
| `ruff check .` | 0 | All checks passed |
| `ty check .` | 0 | All checks passed |
| `pytest tests -q`（全量） | 0 | `1214 passed, 1 skipped, 1 xfailed in 2388.40s`；较 04 的 1172 增 42（04 后导入边界门 +30、本任务 bench +12），skip/xfail 未增长 |

## 未覆盖与保留

- 本任务让试验台**在模型层**真实产出实体；把 bench 片段接入编译与 native 求解属 07/11（TASKS 明确「生产求解穿透由 07/11 完成，本步不冒称已端到端通过」）。
- 现有 7 个 `RigSpec` 的 drives/outputs 声明保持不变，`compose` 收缩语义未动；旧的 7 种默认组合意图与单轴无转向能力保留。
- 新的物理试验台模板（供 11 的扩展性实证）由 11 新增，本任务只提供底座与两个内置分支。
