# Epic：组合式建模流程落地并退役旧装配路径

> 编号约定：本文提到的任务号一律指 `SUBTASKS.csv` 的 `id`，不再使用 S 号。

## Goal

让生产路径严格按下列环节工作，并删除旧装配路径：

```
选模板 → 建子系统 → 多子系统组合成总成 → 总成与试验台组合成试验总成
                                              → 试验总成 + case（含试验台驱动参数）→ 提交仿真
```

验收：

1. **按名选择模板**：用户可达入口用模板名选择子系统模板；换模板改变模型**实体**与**求解结果**；
2. **试验台是组合的一侧且进模型**：试验台声明的刚体进入被求解的试验总成，且与装配**physically 连接**；
3. **case 携带试验台驱动参数**：case 驱动段的坐标集合与取值来源由试验台声明与装配能力的交集决定；
4. **整车有与单轴同形的组合路径**；
5. `connections/policy.py::check_root` 是全局规则唯一裁决点；
6. `build_front_axle` / `FrontAxleAssembly` / `build_vehicle` / `VehicleAssembly` / `preparation/assembly/` 全部删除，`packages/` 下零引用；
7. 三条秒级架构门 + 快速测试集 + 数值门全绿。

## 用户决策（已确认）

### D1 试验台刚体真正进入求解模型

用户选择「B. 进模型」。**注意**：这要求试验台实体与装配之间有真实约束连接，
不是把刚体名并进 bodies 就完事。评审发现当前无此管线（见「已识别的设计缺口 G1」）。

### D2 显式拓扑纳入组合层

用户选择「A. 纳入组合层」，任务 4 承接，随后随任务 11 删除旧分支。

### D3 轮胎归属

单轴供轮型试验台（`supplies_wheels=True`）在场时，轮端刚体与轮胎力元归试验台；
整车（`supplies_wheels=False`）轮胎归装配。

理由：轴侧已发 `VerticalTireElement`（`subsystems/wheel.py:41-62`），供轮 bench 也发
`tire_force_{L,R}`（`rigs/bench.py:161-166`）；两者同时进模型会把同一轮胎计两次。
D9 的语义是「单轴不建车轮、由试验台供轮」，轮胎随轮端归试验台才自洽。

### D4 K/C 基线由新实现重新生成，并登记证据强度下降

> **2026-09-27 更正（实测推翻本决策的前提）**：D4 的前提是「D1 让 K/C 的刚体集
> **与力路径**随之改变，因此 K/C 族必然失败」。实测三个门**全部通过**：
> `kc_native_probe`（9 态，worst ratio 1.66e-05）、`kc_native_c_probe`
> （66 态，1.86e-04）、`kc_parity_check --check`（candidate matches the frozen
> snapshot within tolerance），动态门逐位一致，`case_parity_check` 8 族全 PASS。
>
> 原因：K/C 文档 `gravity = [0,0,0]`，且 carrier 经**共点 weld** 固定在 upright 上、
> **轮胎力的作用点世界坐标不变**（改归属只换 `wheel_body`，`wheel_center_local` 归零）。
> 所以刚体集变了，**受力路径没变**，解不变。
>
> **处置**：K/C 基线**不重录**——无漂移而重录只会把独立参考降级为回归快照，
> AGENTS.md 明确禁止。因此本决策的「代价」一段**不发生**。
> 证据：`artifacts/refactor/baseline-verdict.md`。
> 独立证据（Done-When 10 的 Adams 对标）仍按任务 13 执行——那是独立于本预测的正当要求。

`tests/data/kc_baseline/{k_states.json,c_states.json}` 的生产者（退役的 Python 准静态求解器）
不存在，`scripts/kc_parity_check.py:5-10` 明确 `--record` 已随求解器删除。

D1 让试验台刚体进入模型，K/C 的刚体集与力路径随之改变，因此
`kc_native_probe.py` / `kc_native_c_probe.py` / `case_parity_check.py` 的 `kc_quasi_static` 族
**必然失败**，处置是任务 12 从新实现重新生成。

**代价（必须留痕）**：重录后 `kc_baseline` 由「独立参考」退化为「回归冻结快照」，
`kc_parity_check.py:5-10` 自己就把这种重录描述为
"re-deriving the oracle from the implementation under test"。
该事实写入 `PROGRESS.md` 与 `packages/suspension_multibody/README.md`。

**独立证据的替代（D4 的必要补充）**：K/C 正确性改用 **Adams 外部参考**承担，
见 Done-When 10 与任务 13。同步失效的真值陈述：
`adams/strict_k.py:214`（"frozen separately"）、`templates/builtin.py:41-49`
（"frozen C snapshot cannot be regenerated"）。

### D5 `elements/` 一并删除

用户选择：先让 KC 契约发出弹簧/减振器/稳定杆声明，改 `api.py` 读内核的 element-wrench
通道，再删 `elements/`。任务 15 承接。

现状（已核实）：

- KC 文档的元素表只发 bushing（`cases/kc_quasi_static/contract.py:192`，且仅 C 模式）
  与轮胎（`contract.py:270-273`）；
  **弹簧、减振器、横向稳定杆从不发出** —— native 对它们没有任何事实；
- `api.py:42` 从 `elements` 导入 `evaluate_generalized_forces`，在 `_collect_element_results`
  （`api.py:955-960`）里于 Python 侧重算组件载荷；
- 通道两侧**都已存在**：Python `results/element_wrench.py`
  （开关 `SUSPENSION_KERNEL_ELEMENT_WRENCH_OUTPUT`，13 列，类型码含 spring/bushing/
  anti_roll/steering/drive_brake/tire/external/damper/bump_stop），
  C++ `cpp/include/mb_config/element_wrench.hpp`。缺口在**发射面**。

`README.md:85` 登记了**三条**阻断条件，任务 15 须逐条结清：

1. KC 契约不发弹簧/减振器/稳定杆；
2. 固定体端那一行被 `cpp/src/element/assembly_primitives.cpp:16` 的早退留在 NaN；
3. 力矩参考点口径不一致（native 对受力体原点，Python 对世界原点）。

删除 `elements/` 后，`check_composable_release.py:72-86` 的 `EXPECTED_BOUNDARY_FINDINGS`
与 `tests/architecture/legacy_surface_registry.json` 的最后一条登记随之清空，
`tests/elements/` 与 README 的保留表一并删除。

## 已识别的设计缺口

### G1 试验台实体与装配之间没有连接管线（评审发现，任务 14 承接）

现状事实：

- `rigs/bench.py:156-160` 为供轮 bench 产出 `joints["carrier_{side}"] =
  {"kind": "prismatic", "body": ..., "point": "contact"}` —— 声明字典，无 `body_b`；
- `cases/kc_quasi_static/contract.py:151-155` 只读真实 `Constraint` 对象；
  `subsystems/composition.py:114-120` 也只从 `SubsystemOutput.constraints` 取对象；
- rig 只声明 `contact` 点（`bench.py:155`），而驱动与 marker 按 `wheel_center` 标签查找
  （`contract.py:198,362-372,449`）；
- rig 的轮胎是 fragment 里的 `{"kind": "tire"}` 字典（`bench.py:161-166`），
  而 K/C 文档的轮胎只从 `assembly.elements` 的 `VerticalTireElement` 生成
  （`contract.py:270-273`）；`contract.py:244-266` 自述该路径必须进入残差。

后果：carrier 是无约束刚体，K 模式的规定运动可能作用在试验台而非悬架；
内核不报错，而是把多余自由度钉住
（`suspension_kernel/cpp/src/solve_static/kernel_static_contact.cpp:321` 的
`pinned_directions`，该量正是 `case_parity_check.py:71` 的对外诊断字段）——
即**静默改变物理**。

因此 D1 不能只靠任务 5 完成，必须由任务 14 补齐连接、标签对齐与轮胎通路。

### G2 rig 是否纳入指纹未定

`fingerprint_assembly` 只 walk `assembly`（`subsystems/composition.py:296`），
rig 不进指纹；`walk()`/`entity_ids()` 才含 rig（`modeling/assembly.py:208-222`）。
任务 5 必须决定并写明。

## 硬约束

### C1 顺序断言不可用重录消解

`tests/subsystems/test_assembly_matches_snapshot.py:55,107,124,173`、
`tests/cases/kc_quasi_static/test_contract_documents.py:47-59,78-80`、
`tests/cases/test_vehicle_dynamic_contract.py:127-129`、
`tests/cases/test_axle_dynamic_contract.py:192-197`。
失败时必须证明新顺序是刻意的并同步期望值。

### C2 可重录的基线

`dynamic_hash_baseline.json`、`kc_perf_baseline_native.json`、
`axle_dynamics_baseline/sha256.json`、`vehicle_dynamics_baseline/sha256.json`、
`kc_baseline/*`（按 D4）、`check_composable_baseline` 的 `BASELINE.json`。

### C3 接口必须先冻结

任务 1 冻结「组合运行时装载值」全部字段与构造入口，任务 14 冻结「试验台实体接入」协议；
其余任务只消费、不修改。这是并行不撞车的前提。

### C4 删除前必须逐层可验

任何旧实现的删除都排在「组合层对应能力已实现且有独立断言」之后。

## Non-Goals

- 不新增物理力律、不改内核算法、不改 ABI 版本。
- 不新增 case family、不改契约文档格式与版本。
- 不改 `suspension_kinematics` / `suspension_contracts` / `suspension_kernel`。
- `elements/`（A1）按 D5 一并删除，由任务 15 承接（见上方决策节）。

## 交付边界与单一写者归属

| 文件 | 唯一写者（任务 id） |
|---|---|
| `modeling/instance.py` | 1 |
| `subsystems/runtime.py`（新增） | 1 |
| `subsystems/si_assembly.py` | 1 |
| `subsystems/elements.py`（新增） | 2 |
| `preparation/assembly/front_axle.py`（元素构造器迁出） | 2 |
| `compilation/model_view.py` | 3 |
| `tests/simulation/test_orthogonal_requests.py` | 3 |
| `subsystems/explicit.py`（新增） | 4 |
| `subsystems/composition.py`、`rigs/bench.py`、`modeling/assembly.py` | 5 |
| `subsystems/rig_link.py`（新增，G1 连接管线） | 14 |
| `subsystems/vehicle_assembly.py`（新增） | 6 |
| `templates/`、`subsystems/{suspension,steering,wheel,chassis}.py` | 7 |
| `connections/policy.py` | 8 |
| `compilation/plan.py`、`compilation/compile.py`、`cases/` | 9 |
| `api.py` | 10（首次改写）→ 15（改读 element-wrench 通道），串行 |
| `preparation/`（除 assembly/）、`vehicle/`、`adams/` | 10 |
| 删除动作（`preparation/assembly`） | 11 |
| `scripts/check_composable_release.py`、`tests/architecture/legacy_surface_registry.json` | 11 → 15（结清 `elements` 的最后一条登记），串行 |
| `tests/data/`、`artifacts/kc-native-probe/` | 12 |
| `artifacts/acceptance/`、`PROGRESS.md` | 13 |
| `cases/kc_quasi_static/contract.py` | 9 → 14 → 15，串行 |
| `results/element_wrench.py`、`elements/`（删）、`tests/elements/`（迁出后删）、`README.md`、`tests/architecture/test_import_boundaries.py` | 15 |
| `rigs/bench.py`、`subsystems/wheel.py`、`subsystems/rig_link.py`（新增） | 14 |

## Done-When

每条都给出**可机器执行的命令**；`scripts/acceptance_composable_flow.py` 由任务 13 新建，
它内部逐条实现下列断言并在失败时非零退出。

1. **旧路径零引用**（两条都要满足；`from .elements` 不能作模式，它会误命中
   `modeling/primitives/elements.py` 与 `schema/elements.py` 这两个**幸存**模块）：
   - `git grep -n -e build_front_axle -e FrontAxleAssembly -e build_vehicle -e VehicleAssembly -e preparation.assembly -- packages` 输出为空；
   - 退役包以路径与符号两重判定：`test ! -d packages/suspension_multibody/src/suspension_multibody/elements`
     且 `git grep -n -e "suspension_multibody\.elements" -e "evaluate_generalized_forces" -- packages` 输出为空。
2. **按名选模板改变结构**：`python scripts/acceptance_composable_flow.py --check template-selection`。
   用模板名选中 trailing-arm 模板，断言 `bodies`、`constraints`、`elements` **三者名集都与默认模板不同**，
   且 K 工况求解结果不同。比较对象：默认模板同工况结果（脚本内冻结期望值，非手写）。
3. **试验台实体进模型且已连接**：`python scripts/acceptance_composable_flow.py --check rig-entities`。
   断言 rig 刚体名出现在模型文档 `bodies`、出现在结果的 `body_state`、且 carrier 与装配体之间存在
   真实约束行（不是零空间钉住）；关闭 `SUSPENSION_MULTIBODY_RIG_ENTITIES` 后同一 fixture
   回落到关闭前的值。
4. **快速门全绿**：`just check-fast` 退出码 0。
5. **数值门与基线归因**：`just gate-numeric` 退出码 0，且
   `python scripts/kc_native_probe.py`、`python scripts/kc_native_c_probe.py` 退出码 0；
   同一次运行产出 `artifacts/acceptance/baseline-attribution.md`，给出每个 case 的重录前后差异量，
   并把它归因到「试验台实体接入」这一项改动。
6. **case 驱动来自试验台**：`python scripts/acceptance_composable_flow.py --check rig-drives`。
   修改 `RigSpec.drives` 的一个坐标，断言生成的 case 文档驱动段随之改变，
   且被收缩的坐标缺席而非置零。
7. **显式拓扑经组合层**：`python scripts/acceptance_composable_flow.py --check explicit-topology`。
   断言在**不导入** `preparation.assembly` 的前提下，自由齿条与 `rack_fixed_to_chassis`
   两分支的 `PrismaticJoint`/`WeldJoint` 轴与点等于脚本内冻结的期望值。
8. **架构测试全绿**：`pytest packages/suspension_multibody/tests/architecture -q` 退出码 0。
9. **发布探针**：`python packages/suspension_multibody/scripts/check_composable_release.py`（不带 `--skip-isolation`）退出码 0。
10. **K/C 独立证据（D4 的替代）**：Adams strict-K 对标通过（`suspension-multibody validate-adams
    --strict-k --require-installed`，或在其不可用时明确报告为 BLOCKED 并说明原因）。
    该条不得用 `kc_baseline` 自身比对替代。

## 关键证据出处

| 事实 | 出处 |
|---|---|
| 组合层回指旧路径 | `subsystems/si_assembly.py:274` |
| rig 被写死为空 | `subsystems/composition.py:268` |
| 组合层丢弃 ideal_constraints/bushings | `subsystems/composition.py:114-147` |
| ModelFragment 字段面 | `modeling/instance.py:78-100` |
| 显式拓扑仅在旧路径 | `preparation/assembly/front_axle.py:677-678`、`:305-446` |
| G1：rig 关节是声明字典 | `rigs/bench.py:156-160` |
| G1：契约只读真实 Constraint | `cases/kc_quasi_static/contract.py:151-155` |
| G1：轮胎只从 assembly.elements 生成 | `cases/kc_quasi_static/contract.py:270-273` |
| G1：多余自由度被钉而非报错 | `suspension_kernel/cpp/src/solve_static/kernel_static_contact.cpp:321` |
| K/C 基线不可重录 | `scripts/kc_parity_check.py:5-10` |
| Adams 是外部参考 | `adams/strict_k.py:206-216` |
| 顺序断言清单 | `tests/subsystems/test_assembly_matches_snapshot.py:55,107,124,173` |
| bench 刚体当前不进文档 | `docs/composable_extension_examples.md:220-223` |
| 架构门展开 | `justfile:49,55-58,79-82` |
