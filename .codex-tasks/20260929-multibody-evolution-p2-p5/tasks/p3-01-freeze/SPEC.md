# SPEC：p3-01 冻结现状事实与判据（阶段三）

> 子任务规格。父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p3-01`。
> 形态：`single-full`。

## Goal

把 `SUBTASKS.csv` 中 `p3-01` 的 `acceptance_criteria` 拆成下面 4 条，逐条可判定：

1. **`vehicle/roll_centers.py` 与 `vehicle/static_loads.py` 的现状锚点全表复核**：两条链路各自的硬编码**全表**——`roll_centers.py` 的硬点别名表全部条目（含 `UPPER_*` / `LOWER_*` / `UCA_*` 三种写法）与 `static_loads.py` 的 `_WHEELS` 四元组全表。**另须逐条固化 `static_loads.py` 的「秩/残差」两处现状**（`EPIC.md` 行 249(a) 的复核清单）：**`rank < 3` 抛错段**（实测 `:97-98`）与 **`residual` 的计算口径**（实测 `:96` 的 `float(np.max(np.abs(matrix @ loads - rhs)))`，字段声明 `:39-40`）——这两处正是 p3-04 要改的点（判据口径由秩改为载荷相容性），本行须给出原文、行号与是否过期。锚点起点按 `EPIC.md` F7（行 126）与 F8（行 128）给出的 `file:line` 逐条复核，取原文；**锚点过期即记「已过期」并写明当前真实行号**，不得沿用过期锚点（`EPIC.md` 行 107 的口径）。
2. **四种构型最小几何模型的盘点实测**：双叉臂、5 连杆后悬、麦弗逊滑柱、扭梁四者**今天能否构造**——逐种实跑一次构造并记录结果：成功者记产物形状，失败者记**拒绝点原文**（异常/错误类型 + 消息全文 + 触发位置）。这是 `EPIC.md` 行 314「阶段三的引擎是新增能力，风险在『无用例保护』→ p3-01 先盘四种构型能否构造」的落点；`EPIC.md` 行 34 的配套要求声明四者「算法统一、零硬编码」，故四种构型的可构造性是 p3-02/p3-03 判据的前置事实。**四种构型今天能否构造按本行实测为准**，不得凭类型名推断。
3. **`tests/physics/test_vehicle_physics.py` 的 5 条断言原文**：该文件是 `compute_vehicle_roll_centers`（F7，`EPIC.md` 行 126）在全仓**唯一**的调用者，其 5 条断言是阶段三改造前唯一的回归网；逐条摘出断言原文与所断言的量。
4. **基线影响面复核（F10，`EPIC.md` 行 132）**：确认 `tests/data/kc_baseline/{k_states.json,c_states.json,manifest.json}` 与 `tests/data/dynamic_hash_baseline.json` **不含** roll center 字段；同时记录 F10 点名的两个「只有声明、无计算实现」的量（`roll_stiffness`、`track_change`）在当前代码中的出现位置与形态，作为 p3-02/p3-03 的取值面基线。

## 写范围（允许改的路径）

照 `SUBTASKS.csv` 的 `p3-01` `notes`：**写范围仅本行 `raw/` 与脚本与会话 scratch**。

- `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p3-01-freeze/raw/`（证据，**只存已执行的结果**）
- `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p3-01-freeze/` 下的临时脚本（若需要脚本，落本目录或会话 scratch）
- 会话 scratch：`$PI_SCRATCH_DIR`（临时中间产物）

`EPIC.md`「并行与写范围约束」（行 213–221）中与阶段三相关的三条，本行须一并对齐：

- **p3-03 与 p3-04 的调度关系是「硬串行（p3-04 在 p3-03 之后，不得并行）」**（`EPIC.md` 行 228 与行 253(e)）：生产写范围分别是 `vehicle/roll_centers.py` 与 `vehicle/static_loads.py`（**不相交**），但**测试写范围相交**（`tests/physics/test_vehicle_physics.py` 同时覆盖滚转中心与静平衡，归 p3-03）；`SUBTASKS.csv` 的 `p3-04.depends_on` 实测为 **`p3-03`**（不是 `p3-02`），两者都依赖 p3-02。本行是它们的共同前置，**必须把两条链路的现状锚点分别固化**，使 p3-02 能据此冻结引擎契约；同时**不得把「可并行」写成本行的口径**——测试文件已按文件级切分，但**文件级切分不是并行的理由**（`EPIC.md` 行 228 明写「两行硬串行……不得并行」）。
- **`outputs/builtin.py` 的派生输出声明归 p3-05**（`EPIC.md` 行 221）。本行不改该文件。
- **p3-05 在 p3-04 之后**（`EPIC.md` 行 220，`SUBTASKS.csv` 的 `p3-05.depends_on=p3-04`）。本行不得提前触及 `report/`。

本行是**冻结行**：只读生产代码，不写任何 `packages/**`。

## 禁止触碰

- `packages/**` 下任何文件——本行一行不改（`SUBTASKS.csv` 的 `p3-01` `notes` 只授权 `raw/` + 脚本 + scratch）。
- `EPIC.md`、`SUBTASKS.csv`、父 `PROGRESS.md`，以及 `tasks/` 下其它子任务目录（p2-*、p3-02 ~ p3-06、p4-*、p5-*）——**不得代写、不得预填**。
- `vehicle/roll_centers.py`、`vehicle/static_loads.py`（归 p3-02/p3-03/p3-04）、`report/wheel_loads.py`、`report/metrics/vehicle.py`、`outputs/builtin.py`（归 p3-05）——本行只**读**。
- 任何冻结基线文件：`tests/data/kc_baseline/**`、`tests/data/**/sha256.json`、`dynamic_hash_baseline.json`、`kc_perf_baseline_native.json`（`EPIC.md` 行 228 与 D5 行 64）——只读，**禁止重录**。
- `raw/` 中不得放未执行的内容（`EPIC.md` 行 355：`raw/` 只存**已执行**的证据，不存虚构结果）。

## 依赖与时机

- 父行 `depends_on` **为空**（`SUBTASKS.csv` 的 `p3-01`）：本行是**纯只读冻结行**（写范围只有本行 `raw/` 与会话 scratch，冻结的是**阶段三的现状**——`roll_centers.py` 与 `static_loads.py`，**不读前一阶段任何会被本 Epic 改写的产物**），故**本行不受 `S1` 阻塞、也不受本 Epic 前序阶段阻塞，可立即开工**（`EPIC.md` 行 77「只读冻结行的提前开工」）。`EPIC.md` 行 219 的跨阶段顺序（阶段三在第一阶段完成后开工）**只对实施行成立**；阶段三主线为 `p3-01 → p3-02 → p3-03 → p3-04 → p3-05 → p3-06`（p3-03 → p3-04 **硬串行**，`p3-04.depends_on = p3-03`，不得并行）。
- **阶段一 `S1`（01–07 全 `DONE`）的交付只影响实施行**（p3-02 ~ p3-06 与终局验收行），不影响本行长；本行**不写生产代码**（`SUBTASKS.csv` 的 `p3-01` `notes`）。
- 本行是阶段三的**第一行**，其产物是 p3-02 的输入：p3-02 要基于「装配运行时的约束集合与点表」做数值微分（D3，`EPIC.md` 行 62），故本行必须把**四种构型今天是否能构造**实测清楚（Goal 2），否则 p3-02 无从选择验证构型。
- 时机约束：p3-02 依赖本行；p3-03 与 p3-04 依赖 p3-02，且 **p3-04 硬串行在 p3-03 之后（不得并行）**（`EPIC.md` 行 228 与行 253(e)；`SUBTASKS.csv` 的 `p3-04.depends_on=p3-03`）——生产写范围不相交但测试写范围相交，测试文件已按文件级切分，但**文件级切分不是并行的理由**；p3-05 在 p3-04 之后；p3-06 依赖 p3-03 与 p3-05（`SUBTASKS.csv` 的 `p3-06.depends_on=p3-03;p3-05`）。
- **禁止依据旧任务 DONE 结论**（`EPIC.md` 行 320 的口径：本 Epic 起点须先实测）——所有事实以本行实跑为准。

## 判据与证据落点

逐条对应 `EPIC.md` 行 249 的 **(a)(b)(c)(d)**；每条写清「跑什么命令、看什么输出、证据落到哪个文件」。

1. **(a) 两条链路的完整现状锚点复核** → `raw/anchors_roll_centers_static_loads.md`
   - 跑什么：按 `EPIC.md` F7（行 126）与 F8（行 128）给出的 `file:line` 逐条打开并取原文：`roll_centers.py:59-67` 的硬点别名表**全部条目**、`:81-99` 的 `_instant_center`、`:96` 的 `_line_intersection`、`:42-47` 与 `:117-129` 的整车层二维交点；`static_loads.py:28` 的 `_WHEELS`、`:79-95` 的 3×4 平衡矩阵与 `np.linalg.lstsq`、**`:97-98` 的 `rank < 3` 抛错段**、**`:96` 的 `residual` 计算行与 `:39-40` 的字段声明**（后两处是 p3-04 要改的口径，必须逐字取原文）。
   - 看什么：每条给「锚点 = `文件:行` + 该行原文摘要 + 是否过期」；硬点别名表要**逐条目**列出（不是抽样），`_WHEELS` 要**逐元素**列出；**秩/残差两处要给：抛错条件的原文（实测 `rank < 3`）、`residual` 的算法原文与它今天的用途（仅记录、不参与抛错）**；过期项写明当前真实行号。
   - 落点：`raw/anchors_roll_centers_static_loads.md`。
2. **(b) 四种构型能否构造的实测** → `raw/config_refusals.md`
   - 跑什么：对双叉臂、5 连杆、麦弗逊、扭梁各写一份**最小几何模型声明**并实跑一次构造（用装配/构造入口或 `subsystems/geometry.py` 的硬点读取面，具体入口由本行实测确定并记录）；成功者记产物形状，失败者记拒绝点。
   - 看什么：四种构型**逐种**一行结论——「可构造 / 不可构造」+ 证据。不可构造的必须给**拒绝原文**（异常类型 + 消息全文 + 触发位置 `file:line`）。`EPIC.md` F7（行 126）指出今天的两条臂线求交只认 `[y,z]` 平面的上下臂硬点，故 5 连杆/麦弗逊/扭梁预期不能构造——**预期不能，按实测记录**。
   - 落点：`raw/config_refusals.md`（本行 `validation_command` 的 `test -s` 断言它非空）。
3. **(c) 既有 5 条断言原文** → `raw/physics_assertions.md`
   - 跑什么：读 `tests/physics/test_vehicle_physics.py`，摘出 5 条断言的原文；同时记该文件对 `compute_vehicle_roll_centers` 的调用位置（F7 行 126 给出的 `:5/55` 为起点锚点，按实测复核）。
   - 看什么：断言原文逐条列出，并注明每条断言的量名与判定方式（相等/范围/对称）。
   - 落点：`raw/physics_assertions.md`。
4. **(d) 基线影响面复核** → `raw/baseline_impact.md`
   - 跑什么：在 `tests/data/kc_baseline/` 与 `tests/data/dynamic_hash_baseline.json` 中检索 roll center 相关字段（`roll_center` / `roll_stiffness` / `track_change` 等），并核对 F10（行 132）点名的字段清单（`kc_baseline` 含 `left_wheel_center_y_mm` / `track_mm` / camber / toe；`dynamic_hash_baseline.json` 26 条目只有 `arrays_npz_sha256` / `manifest_sha256` / `status`）；另核 `roll_stiffness`（F10 给出的 `adams/vehicle_parameters.py:40`）与 `track_change`（`templates/builtin.py:393`、`templates/roles.py:85`）两处「只有常量/声明、无计算实现」的位置。
   - 看什么：命中数（roll center 类字段应为 0 命中）+ 各基线文件的实际字段清单 + 两个无计算实现量的原文与位置。
   - 落点：`raw/baseline_impact.md`（本行 `validation_command` 的 `test -s` 断言它非空）。

## Constraints（冻结约束）

与 `EPIC.md` 「冻结约束」（行 223–233）中与本行相关的条目：

- **不改动任何 K/C 读数**（`EPIC.md` 行 228，D5 行 64）：`tests/data/kc_baseline/` 与 `dynamic_hash_baseline.json` **逐字节不变是硬门**。本行只**读**并记起点值，**禁止重录**；`vehicle_dynamics_baseline/sha256.json` 与 `kc_perf_baseline_native.json` 本 Epic 期间不再动。
- **分层方向不可逆**（`EPIC.md` 行 226）：`modeling -> templates -> subsystems -> preparation/studies -> cases`；`report/` 不得 import native/kernel/solver，也不得自求力律（`legacy_surface_gate.py` 的 `report_native_import` / `report_constitutive_call` 规则）。本行不改生产代码，只在核对中把发现的跨层反向导入**记为事实**，**不在本行修**。
- **阶段三引入新依赖方向须先实测**（`EPIC.md` 行 315）：`vehicle/roll_centers.py` 今天依赖 `subsystems.geometry`（`side_hardpoints`）；新引擎若读装配运行时会加重 `vehicle → subsystems` 依赖。本行须把该依赖方向**记为事实基线**，供 p3-02 的 `test_import_boundaries.py` 判定使用。
- **不得新增 skip/xfail**（`EPIC.md` 行 229）：`tests/adams` 的环境 skip 是既有的，不得增长。本行记录起点 skip/xfail 计数，作为后续各行的对照。
- **每步落地后必须重跑**（`EPIC.md` 行 230）：本行落地的是证据文档，改动后重跑三条秒级架构门即可，不要求跑全量。
- **`kc_parity_check.py` 的口径**（`EPIC.md` 行 231）：不带 `--actual-dir` 时是拿冻结快照与自身比较（恒过），**不构成证据**；本行若跑它必须注明是否带 `--actual-dir`。

## 风险与回退

- **四种构型今天能否构造需实测**（`EPIC.md` 行 314 与行 34）：阶段三的配套要求声明双叉臂/5 连杆/麦弗逊/扭梁「算法统一、零硬编码」，但今天的两条臂线求交（F7，行 126）预期只支持双叉臂。回退/缓解：本行把四种构型**逐个实跑**并把拒绝原文落盘；若某种构型**连最小几何模型都无法声明**，记为事实并在 `raw/config_refusals.md` 中写明「p3-03 该构型的断言需先补最小模型声明」——**不得**把该构型从 p3-03 的判据里静默删掉（`EPIC.md` 行 83 的 G3 判据要求四种构型各有断言）。
- **阶段三无用例保护**（`EPIC.md` 行 314）：`compute_vehicle_roll_centers` 今天无生产调用者，唯一调用者是 `tests/physics/test_vehicle_physics.py`（F7，行 126）。缓解：本行的 5 条断言原文（Goal 3）是 p3-03 的对照基准；起点断言原文缺失时 p3-03 无对照，不得开工。
- **基线影响面小但不为零**（`EPIC.md` 行 132 与行 314）：F10 判定 `kc_baseline` / `dynamic_hash_baseline` 不含 roll center 字段，故通用引擎主要靠新增断言保护；但**仍必须守住**「不改变已有 K/C 读数」。缓解：本行把基线字段清单**完整落盘**，作为 p3-06 的零回归对照。
- **锚点过期风险**（`EPIC.md` 行 107 的口径）：F7/F8/F9/F10 给出的行号若与本行实测不符，记为事实并写明当前真实行号，不得静默沿用。
- **既有失败**（`EPIC.md` 行 320）：本 Epic 起点须先实测；本行记录既有失败清单并标注是否与阶段三相关。

## Done-When

- [ ] `raw/anchors_roll_centers_static_loads.md` 非空，含 `roll_centers.py` 硬点别名表**逐条目**、`_instant_center` / `_line_intersection` / 整车层二维交点，以及 `static_loads.py` 的 `_WHEELS` 逐元素、3×4 平衡矩阵、**`:97-98` 的 `rank < 3` 抛错段原文与 `:96` 的 `residual` 计算口径（含 `:39-40` 字段声明）**；每条带 `file:line` + 原文 + 是否过期。
- [ ] `raw/config_refusals.md` 非空，含双叉臂 / 5 连杆 / 麦弗逊 / 扭梁**四种构型各自**的「可构造 / 不可构造」结论；不可构造者带拒绝原文（异常类型 + 消息全文 + 触发位置）。
- [ ] `raw/physics_assertions.md` 非空，含 `tests/physics/test_vehicle_physics.py` 的 5 条断言原文与调用位置。
- [ ] `raw/baseline_impact.md` 非空，含 `kc_baseline` / `dynamic_hash_baseline.json` 的实际字段清单、roll center 类字段命中数为 0 的实测结论，以及 `roll_stiffness` / `track_change` 两处「无计算实现」的事实（`EPIC.md` 行 132）。
- [ ] `raw/start_state.md` 非空，含起点 skip/xfail 计数、既有失败清单（标注是否与阶段三相关）、`git status` / `git diff --stat -- packages/` 的自证输出。
- [ ] 未修改 `packages/**` 任何文件、未修改 `EPIC.md` / `SUBTASKS.csv` / 父 `PROGRESS.md`、未重录任何基线。
- [ ] p3-02 所需的「引擎输入面」证据齐备：四种构型的构造能力 + 两条链路的现状锚点（**含静态载荷侧的 `rank < 3` 抛错段与 `residual` 计算口径**）+ 基线字段清单；且本行记录的 p3-03 / p3-04 调度口径与父表一致（**硬串行，不得并行**，不写「可并行」）。

## Final Validation Command

```bash
uv run --no-sync pytest packages/suspension_multibody/tests/physics -q && test -s .codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p3-01-freeze/raw/config_refusals.md && test -s .codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p3-01-freeze/raw/baseline_impact.md
```
