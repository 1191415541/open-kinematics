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
- [ ] 大文件生成 → 快照预计 < 200 KB，无风险。

## Deliverables

- `raw/assembly_snapshot.json` — 7 组合的装配产物快照（含 `_meta`：枚举口径、生成命令、生成时间）
- `raw/painpoint_anchors.md` — 5 条痛点的锚点复核表（已完成）
- `raw/triaxle_refusal.md` — 3 轴/不对称的负例原文（已完成）
- `raw/baseline_commands.md` — 门禁与数值门实测值（已完成）
- `snapshot.py` — 生成快照的可重跑脚本（放本子任务目录，不放 scratch）

## Done-When

- [ ] 四份 `raw/` 证据齐备，且每份都注明生成命令与时间。
- [ ] `snapshot.py` 连续两次运行的输出逐字节一致（`sha256` 相同）。
- [ ] 快照覆盖 7 个组合，且每个组合的 `bodies/points/constraints/elements` 非空（空集会被当作"没有判据"）。
- [ ] `raw/triaxle_refusal.md` 记录的是**报错原文**，不是转述。

## Final Validation Command

```bash
cd /c/杂件/open-kinematics && \
uv run --no-sync python .codex-tasks/20260929-assembly-layer-rework/tasks/20260929-01-freeze/snapshot.py --check && \
test -f .codex-tasks/20260929-assembly-layer-rework/tasks/20260929-01-freeze/raw/assembly_snapshot.json && \
test -f .codex-tasks/20260929-assembly-layer-rework/tasks/20260929-01-freeze/raw/triaxle_refusal.md && \
uv run --no-sync pytest packages/suspension_multibody/tests -q
```

## Demo Flow

1. `uv run --no-sync python .../snapshot.py` → 生成 `raw/assembly_snapshot.json`
2. `uv run --no-sync python .../snapshot.py --check` → 与已落盘快照比对，逐字节一致则退出 0
3. 后续子任务（02–07）在落地前后各跑一次 `--check`，差异即"产物是否变化"的证据。
