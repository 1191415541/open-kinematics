# SPEC：01 冻结现状事实与判据

> 子任务规格。父 Epic：`.codex-tasks/20260929-assembly-layer-rework/EPIC.md`；父行：`SUBTASKS.csv` 的 `01`。

## Task Shape

- **Shape**: `single-full`

## Goals

为后续 02–07 提供**可复跑的对照基准**，把"现状"钉成文件：

1. 现有 7 个 (总成, 试验台) 组合的装配产物快照（bodies / points / constraints / ideal_constraints / elements / connections 的集合与名字），作为每步"逐项不变"的判据。
2. 5 条痛点（R1–R5）的代码锚点复核记录（file:line + 结论 + 与既有裁决的冲突）。
3. "3 轴"与"左右独立悬架文件"今天**在哪一层、以什么报错**被拒绝的原文。
4. 门禁与数值门的实测值与退出码（本 Epic 的起点）。

## Non-Goals

- 不写任何生产代码（本行只产出证据与判据）。
- 不改任何基线、不改任何测试。
- 不做 02–07 的改造设计（那属于各自 SPEC）。

## Constraints

- 写范围仅本子任务目录（`raw/`、脚本）与会话 scratch；不得改 `packages/**`。
- **禁止依据旧任务（20260922 / 20260921 Epic）的 DONE 结论**：所有事实必须本行实测。
- 快照必须**可重跑且稳定**（同一命令两次输出逐字节一致），否则它不能当判据。
- 证据只记**已执行**的结果；未执行的项留 `TODO`，不得先填结论。

## Environment

- **Project root**: `c:/杂件/open-kinematics`
- **Language/runtime**: Python 3.12（`uv run --no-sync`）
- **Package manager**: `uv`
- **Test framework**: `pytest`
- **Build command**: 无（纯 Python；内核 DLL 已构建，本行不动内核）
- **Existing test count**: 快速集 1015 passed / 1 xfailed；`tests/cases` 100；`tests/architecture` 147；`tests/adams` 160/47 skip；kernel+contracts 60

## Risk Assessment

- [ ] 7 个组合的枚举口径不确定 → 以 `rigs/` 注册表 + `subsystems/types.py` 的默认子系统集为准，枚举结果写进 `raw/assembly_snapshot.json` 的 `_meta` 段。

**7 个组合的确切口径（2026-09-29 实测）**：`rigs/rig.py:114` 的 `RIGS` 注册表恰有 7 项——`kc_quasi_static`、`axle_dynamic`、`vehicle_kc`、`vehicle_dynamic`、`handling`、`ride_four_post`、`ride_random_road`。因此 `snapshot.py` 的组合枚举以该注册表为唯一来源（不得另立名单），每个组合记录：rig 名、其 `RigSpec` 的 `family`/`study`/`supplies_wheels`、以及它运行的装配产物结构；单轴侧用 `tests/benchmark_fixture.benchmark_model()`、整车侧用 `tests/vehicle/test_native_vehicle.py::_vehicle()` 的既有夹具作为模型输入（夹具口径写进 `_meta`）。
- [ ] 快照口径（集合 vs 逐值）不足以当"零回归"判据 → 快照存**集合 + 名字 + 约束端点/类型**；数值层面的零回归由 04 的 `kc_baseline` 逐位比对负责。
- [x] 大文件生成 → 实测 `raw/assembly_snapshot.json` = 539,636 字节（原估「< 200 KB」偏小：四个单轴产物 + 一个整车产物的点表与约束字段被完整记录）。文件大小不影响判据，按实测登记。
- [x] 载荷里混进易变字段 → 首版把 `frozenset` 交给 `str()` 兜底，集合迭代顺序随进程哈希种子变化，两次运行 sha256 不同（`720d8437…` vs `d8f0ccb4…`）；改为「排序后的列表」后两次一致。第二轮审核后又按试验台拆开产物键、把登记收紧为四项精确匹配，当前快照 sha256 = `b58d35ac…`（两次运行一致）。

## Deliverables

- `raw/approved_deltas.json` — 已登记差异清单（初始为空数组 `[]`；登记项必须**四项精确匹配**（`product`/`pointer`/`before`/`after`）并带 `reason`/`registered_by`/`evidence`，`registered_by` 必须是 **05**——只有 05 被允许改变既有产物；缺字段或归属不符即 `--check` 退出 4）
- `raw/painpoint_anchors.md` — 5 条痛点的锚点复核表（已完成）
- `raw/triaxle_refusal.md` — 3 轴/不对称的负例原文（已完成）
- `raw/baseline_commands.md` — 门禁与数值门实测值（已完成）
- `snapshot.py` — 生成快照与判定差异的可重跑脚本（放本子任务目录，不放 scratch）

## Done-When

- [x] 五份 `raw/` 证据齐备，且每份都注明生成命令与时间：`painpoint_anchors.md`、`triaxle_refusal.md`、`baseline_commands.md` 已完成；`assembly_snapshot.json`、`approved_deltas.json`、`snapshot_notes.md` 本次落盘。
- [x] `snapshot.py` 连续两次运行的输出逐字节一致（`sha256` 相同）；载荷里不含生成时间等易变字段。
- [x] 快照覆盖 7 个组合，且每个组合的 `bodies`/`points`/`constraints` 非空、受力列整体（`constraints + ideal_constraints + elements`）非空。原写「每个组合的 `elements` 非空」与 K 读数的实际分层冲突（K 的受力列是 `ideal_constraints`，力元只在 C 出现，F9），已按实测改正；证据见 `raw/snapshot_notes.md`。
- [x] 供轮的两个试验台（`kc_quasi_static`、`axle_dynamic`）**各自成键**的产物都记录（`axle_<模式>@<试验台>`），整车侧 5 个 rig 共用 `vehicle` 并在 `_meta.coverage_boundary` 与各 rig 的 `bench_bound` 里写明「试验台绑定发生在读数层、不在本快照内」。
- [x] `raw/triaxle_refusal.md` 记录的是**报错原文**，不是转述。
- [x] `snapshot.py --check` 采用「未变化部分逐项相等 + 已登记差异」口径：差异必须命中 `raw/approved_deltas.json` 的某条登记，否则非零退出（负例已实测，见 `raw/snapshot_notes.md`）。

## Final Validation Command

```bash
cd /c/杂件/open-kinematics && \
uv run --no-sync python .codex-tasks/20260929-assembly-layer-rework/tasks/20260929-01-freeze/snapshot.py --check && \
test -f .codex-tasks/20260929-assembly-layer-rework/tasks/20260929-01-freeze/raw/assembly_snapshot.json && \
test -f .codex-tasks/20260929-assembly-layer-rework/tasks/20260929-01-freeze/raw/approved_deltas.json && \
test -f .codex-tasks/20260929-assembly-layer-rework/tasks/20260929-01-freeze/raw/triaxle_refusal.md && \
uv run --no-sync pytest packages/suspension_multibody/tests -q
```

## Demo Flow

1. `uv run --no-sync python .../snapshot.py` → 生成 `raw/assembly_snapshot.json`
2. 再跑一次并比对 `sha256` → 逐字节一致（载荷里没有生成时间等易变字段）
3. `uv run --no-sync python .../snapshot.py --check` → 与已落盘快照比对：无差异、或差异全部命中 `raw/approved_deltas.json` 时退出 0
4. 后续子任务（02–07）在落地前后各跑一次 `--check`，差异即"产物是否变化"的证据。
