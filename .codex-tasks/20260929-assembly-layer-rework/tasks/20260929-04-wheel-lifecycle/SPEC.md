# SPEC：04 轮端生命周期统一

> 子任务规格。父 Epic：`.codex-tasks/20260929-assembly-layer-rework/EPIC.md`；父行：`SUBTASKS.csv` 的 `04`。

## Task Shape

- **Shape**: `single-full`

## Goals

把轮端（车轮刚体 + 轮胎声明）的生命周期统一到**同一份 wheel 子系统文件**，并让"哪种受力生效"由**读数**决定而不是由装配阶段增删零件决定；**单轴侧**的凝结必须逐位等价。判据逐条来自父行 `04` 的 `acceptance_criteria` 与 `validation_command`，并按 `EPIC.md` 的「验证协议 / **04（统一轮端）**」展开：

1. **文件读取链打通**：单轴总成与整车总成都**读入同一份** wheel 子系统文件，两侧都由它产出车轮刚体 + 轮胎声明。今天单轴侧不产出任何 body、轮胎声明挂在"声明 `wheel_center` 点的那个 body"上（`subsystems/wheel.py:33-41`、`:87-111`，F5），`subsystems/wheel.py:44-59` 只读内置模板（`EPIC.md` 04 验证协议 (a)）。
2. **装配阶段不再删建轮端**：`subsystems/vehicle_assembly.py` 的 `isinstance(element, VerticalTireElement)` 类型过滤（`:233-240`，F5）被删除，`grep` 无命中；该过滤的删除归属本行。
3. **凝结硬门（D2）**：**单轴侧**（试验台/请求声明 `RigSpec.supplies_wheels=True` 的读数：K/C 与轴动态）把 wheel 子系统产出的车轮刚体与轮毂刚体在内存中**刚性凝结（condensation）**；整车侧不凝结。凝结后**实体集合、约束行数、自由度不变**，且 **`kc_baseline` 逐位不变**、**`dynamic_hash_baseline` 的 13 个轴侧用例逐字节不变**。**判据不得再用「车轮子系统本次不产出车轮刚体」**（文件层统一后该判据已失效），也不得用不带 `--actual-dir` 的 `kc_parity_check.py` 自比较（那恒过、不构成证据）；必须：`scripts/kc_native_probe.py` + `scripts/kc_native_c_probe.py` 生成 actual，再 `scripts/kc_parity_check.py --check --actual-dir artifacts/kc-native-probe`，并跑 `scripts/dynamic_hash_sentinel.py --check`（26 个 artifact 逐字节一致）。
4. **复用既有凝结机制**：凝聚走 `subsystems/vehicle_parts.py` 的 `_merge_fixed_wheel`（`:572-610`；`wheel.mount_joint_kind == "fixed"` 时按复合质量属性把车轮质量/惯量并入 mount body 且不建独立车轮体，分支 `:482-508`）与同文件的 `_fuse_welded_bodies`，不新造机制（F5b）；并在 `PROGRESS.md` 里点名与 **20260921 Epic 的 A3**（整车侧生产路径改为"不凝聚、weld 交内核 `fixed` 关节"）的关系，避免两处"凝聚"语义混淆。
5. **读数决定受力**：K 台（轮心推压）下轮胎不参与受力、C 垫板机下轮胎弹性生效，**两侧各有断言**；沿用 F9 的分层（装配体保留 `RevoluteJoint`、由**读数文档**决定激活哪种受力，先例见 `cases/kc_quasi_static/contract.py` 的 `TIRE_MODES` / `ELASTIC_MODES`）。
6. **不得迁移轮胎质量所有权**：质量/质心/惯量的口径不变——凝结只改求解拓扑表示，不把轮胎质量搬到别的实体上。

## Non-Goals

- 不反转既有裁决 **D9**（单轴侧车轮归谁）：本行按 **D2** 走"文件层统一 + 装配期刚性凝结"，不把车轮体收进悬架试验台。
- **严禁重录 `kc_baseline` 与 `dynamic_hash_baseline`**：凝结必须逐位等价（含轴侧动态用例逐字节一致），否则退回并先证明等价；禁止先改基线让门变绿。
- 不改轮胎本构与求解器数值路径（父 Epic Non-Goals）。
- 不做整车侧的凝聚取舍（20260921 Epic A3 的结论不动）。
- 不改试验台契约与 `subsystems/rig_link.py`、`rigs/`（05 独占）。
- 不做可选对称（06 的范围）：`_SIDES` 的收口不在本行。

## Constraints

- **写范围（逐条照 `SUBTASKS.csv` `04` 的 `notes`）**：
  - `subsystems/wheel.py`
  - `subsystems/element_build.py`
  - `subsystems/vehicle_parts.py`（仅**车轮与挂接段**：`_add_wheel`、`:482-508`、`:510-515`、`:525-541`、`:572-610`）
  - `subsystems/vehicle_assembly.py`（仅**轮胎过滤段** `:233-240`）
  - `subsystems/si_assembly.py`
  - `cases/kc_quasi_static/contract.py`
  - `preparation/vehicle_dynamic.py`（**仅轮端内容段**：`:900-903` 的 `VerticalTireElement` 拒收、`:1009-1025` 的轮胎构建；**模型访问段（`:188`、`:207`、`:210`、`:341`、`:368-369`）与 `:373` 的命名规则段归 03，且必须在 03 之后**）
- 本行依赖 `03`（`03` 与 `04` 都触及 `subsystems/vehicle_assembly.py` 与 `subsystems/vehicle_parts.py`，必须串行）；`04` 与 `06` 都触及 `subsystems/si_assembly.py` 与 `subsystems/suspension.py`，故 `06` 的 `depends_on` 为 `05`，不得并行。
- `subsystems/si_assembly.py` 在 04 与 05 的写范围里都出现：05 只动 **bench 接入段**，本行不动该段；两行串行执行、不得并发写同一文件。
- 旧有物理表示差异按 F5 的修正理解：真正的差异不是"轮胎力元被删又建"，而是**同一种物理在两条路径上用两种表示**（垂向力元 vs native 轮胎）；本行的落法是统一**实体**（车轮刚体 + 轮胎声明由 wheel 子系统产出），把"哪种受力激活"交给读数。
- **产物是否变化的判据**：用 `tasks/20260929-01-freeze/snapshot.py --check`，口径「未变化部分逐项相等 + 已登记差异」，登记处 `tasks/20260929-01-freeze/raw/approved_deltas.json`（**待 01 的 `#1` 完成后跑**）；**04 不得产生差异，差异非空即失败**（只有 05 被允许改变既有产物并登记），且 `kc_baseline` 与 `dynamic_hash_baseline` 轴侧逐位不变。
- 每步落地后必须重跑 `just check-fast`（ruff / ty / 三个架构门 / 快速集 / 另两包）与数值门三项；结构改动后必须重跑 `tests/architecture`。
- 不得新增 skip/xfail。
- 只改本行写范围；`raw/` 只存**已执行**的证据，不存虚构结果。

## Environment

- **Project root**: `c:/杂件/open-kinematics`
- **Language/runtime**: Python 3.12（`uv run --no-sync`）
- **Package manager**: `uv`
- **Test framework**: `pytest`
- **Build command**: 无（纯 Python；内核 DLL 已构建，本行不动内核）
- **Existing test count**: 快速集 1015 passed / 1 xfailed；`tests/cases` 100；`tests/architecture` 147；`tests/adams` 160/47 skip；kernel+contracts 60

## Risk Assessment

- [ ] **凝结不等价（本行唯一硬门）**：若凝结后 `kc_baseline` 有任何一位变化、或 `dynamic_hash_baseline` 的 13 个轴侧用例有任何字节变化，即凝结不等价 → 立即退回，先给出独立于结果字节的等价判据（自由度与约束行数、惯量、轮心与接触点几何、轮胎力路径）再重试。
- [ ] **凝结作用域的判据不确定** → 判据改用**试验台/请求声明**：`RigSpec.supplies_wheels=True` 的读数（K/C 与轴动态）即单轴侧、必须凝结；整车侧不凝结。**判据不得再用「车轮子系统本次不产出车轮刚体」**（文件层统一后已失效）；判据须写进代码与测试，不靠命名猜测。
- [ ] **同一份 wheel 子系统文件两侧语义不同**（单轴不产 body、整车产 body）导致文件层统一后产物漂移 → 先跑 `snapshot.py --check` 确认差异清单；**04 不得产生差异，差异非空即失败**（只有 05 被允许改变既有产物并登记），`kc_baseline` 与 `dynamic_hash_baseline` 轴侧不接受任何差异。
- [ ] **K 台/C 台断言可能只是"测试写的假设"** → 两侧断言必须落在实际受力路径上（K 台轮胎不参与、C 垫板机轮胎弹性生效），并复用 F9 的读数分层。
- [ ] **与 20260921 Epic A3 的"凝聚"语义混淆** → 在 `PROGRESS.md` 里明确点名两者关系（整车侧不凝聚、单轴 K/C 侧凝结）。
- [ ] **质量口径漂移**：凝结若把轮胎质量并入轮毂，质量/质心/惯量口径会变 → 本行禁止迁移轮胎质量所有权，需有断言或证据。

## Deliverables

- 写范围内各路径的改造（`subsystems/wheel.py`、`subsystems/element_build.py`、`subsystems/vehicle_parts.py` 车轮与挂接段、`subsystems/vehicle_assembly.py` 轮胎过滤段、`subsystems/si_assembly.py`、`cases/kc_quasi_static/contract.py`、`preparation/vehicle_dynamic.py` **仅轮端内容段 `:900-903`、`:1009-1025`**）。
- 覆盖 Goals 1–6 的断言（测试落在 `packages/suspension_multibody/tests/` 现有对应目录，含"同一份 wheel 文件两侧加载""无 `VerticalTireElement` 过滤""凝结后实体/约束行数/自由度不变""K 台与 C 台两条受力路径""质量口径不变"）。
- `raw/kc_condensation_parity.md` — 凝结等价证据：两条 probe 的调用命令、`artifacts/kc-native-probe` 的 actual 清单、`kc_parity_check --check --actual-dir` 的退出码、`kc_baseline` 逐位比对结果与 `dynamic_hash_sentinel.py --check` 的 26 个 artifact 逐字节结果（本行开工后生成）。
- `raw/wheel_file_unification.md` — 文件层统一的产物差异清单（`snapshot.py --check` 的前后结果，口径「未变化部分逐项相等 + 已登记差异」、登记处 `tasks/20260929-01-freeze/raw/approved_deltas.json`；**04 不得产生差异，差异非空即失败**）。
- `raw/mass_ownership.md` — 质量/质心/惯量口径不变的证据（本行开工后生成）。

## Done-When

- [ ] 单轴总成与整车总成都从**同一份** wheel 子系统文件产出车轮刚体 + 轮胎声明（文件读取链打通），两侧各有断言。
- [ ] `grep -n "VerticalTireElement"` 在 `subsystems/vehicle_assembly.py` 无命中，且该过滤的删除已登记在本行。
- [ ] **单轴侧**（`RigSpec.supplies_wheels=True` 的读数：K/C 与轴动态）把 wheel 子系统产出的车轮刚体与轮毂刚体刚性凝结进轮毂；装配后**实体集合、约束行数、自由度不变**；整车侧不凝结。
- [ ] `kc_native_probe.py` + `kc_native_c_probe.py` 生成 actual 后，`kc_parity_check.py --check --actual-dir artifacts/kc-native-probe` 通过，且 **`kc_baseline` 逐位不变**、**`dynamic_hash_baseline` 的 13 个轴侧用例逐字节不变**、`dynamic_hash_sentinel.py --check` 的 26 个 artifact 逐字节一致（均未重录）。
- [ ] `_merge_fixed_wheel` / `_fuse_welded_bodies` 被复用（未新造凝结机制），`PROGRESS.md` 已点名与 20260921 Epic A3 的关系。
- [ ] K 台下轮胎不参与受力、C 垫板机下轮胎弹性生效，**两侧各有断言**，分层沿用 F9。
- [ ] 轮胎质量所有权未迁移（质量/质心/惯量口径不变，有证据）。
- [ ] `just check-fast` 与数值门三项全绿；无新增 skip/xfail。
- [ ] `tasks/20260929-01-freeze/snapshot.py --check` 通过且**无差异**（口径「未变化部分逐项相等 + 已登记差异」、登记处 `raw/approved_deltas.json`；**04 不得产生差异，差异非空即失败**）（待 01 的 `#1` 完成后跑）。

## Final Validation Command

```bash
cd /c/杂件/open-kinematics && \
uv run --no-sync pytest packages/suspension_multibody/tests/subsystems packages/suspension_multibody/tests/vehicle packages/suspension_multibody/tests/simulation packages/suspension_multibody/tests/authoring -q && \
uv run --no-sync python packages/suspension_multibody/scripts/kc_native_probe.py && \
uv run --no-sync python packages/suspension_multibody/scripts/kc_native_c_probe.py && \
uv run --no-sync python packages/suspension_multibody/scripts/kc_parity_check.py --check --actual-dir artifacts/kc-native-probe && \
uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check && \
uv run --no-sync python .codex-tasks/20260929-assembly-layer-rework/tasks/20260929-01-freeze/snapshot.py --check
```

其中倒数第二行（`dynamic_hash_sentinel.py --check`）为动态基线硬门：26 个 artifact（含 13 个轴侧用例）逐字节一致；最后一行（`snapshot.py --check`，作为「产物是否变化」的判据，口径「未变化部分逐项相等 + 已登记差异」、登记处 `tasks/20260929-01-freeze/raw/approved_deltas.json`，**04 不得产生差异，差异非空即失败**）**待 01 的 `#1` 完成后跑**。

## Demo Flow

1. 单轴总成与整车总成各引用同一份 wheel 子系统文件 → 打印两侧产出的车轮刚体与轮胎声明清单 → 断言两侧同源。
2. 单轴侧装配（`RigSpec.supplies_wheels=True` 的读数）→ 打印凝结前后的实体集合/约束行数/自由度 → 断言三者不变；整车侧确认不凝结。
3. `kc_native_probe.py` + `kc_native_c_probe.py` → `artifacts/kc-native-probe`；`kc_parity_check.py --check --actual-dir artifacts/kc-native-probe` → 退出 0，`kc_baseline` 逐位未变、`dynamic_hash_sentinel.py --check` 的 26 个 artifact 逐字节未变。
4. K 台与 C 垫板机各跑一次读数 → 分别断言"轮胎不参与受力"与"轮胎弹性生效"。
5. 落地前后各跑一次 `snapshot.py --check`（待 01 的 `#1` 完成后）→ 其差异即"产物是否变化"的证据；**04 不得产生差异，差异非空即失败**。
