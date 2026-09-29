# SPEC：07 终局独立验收

> 子任务规格。父 Epic：`.codex-tasks/20260929-assembly-layer-rework/EPIC.md`；父行：`SUBTASKS.csv` 的 `07`。

## Task Shape

- **Shape**: `single-full`

## Goals

本行是**独立终局验收**：逐条实跑 `EPIC.md` 的 Done-When 端到端清单 (a)-(g)，逐条记录退出码与产物差异，作为「Epic 达成」的唯一终局证据。

1. **独立于子任务自证**：**不得采信任何子任务的自报结论**（02–06 的 `PROGRESS.md`、`TODO.csv` 的 `DONE`、`raw/` 内的转述都只作线索）。每条判据都由本行亲自跑命令、亲自读输出。`EPIC.md` 明确「每行 DONE 不代替这些条件」。
2. **逐条实跑 (a)-(g)**（原文见 `EPIC.md` 的 Done-When 代码块）：
   - **(a) 3 轴整车总成**（3 个悬架子系统 + 车身 + 转向 + 车轮 + 制动 + 驱动）装配并跑通一次；**实体清单与文件清单一一对应**；装配路径 `grep` 无 `front_axle`/`rear_axle`。注意 `EPIC.md` **F2** 指出拒绝发生在更早两层（`connections/policy.py` 的 `role_counts`/`required_placements` 与 `assembly.schema.json` 的 placement 枚举与轮数限制），故 07 必须确认这三处前置墙都已打开，而不只是装配函数能跑。
   - **(b) 镜像 vs 左右独立文件**：同一份总成文件，悬架条目分别写「镜像」与「左右独立文件」，两次装配产物**逐项比较点坐标与约束端点**后一致（只比名字集合不构成证据）。
   - **(c) 单轴与整车共用同一份 wheel 子系统文件**；装配阶段无「删轮胎再建车轮」；单轴 K/C 装配期刚性凝结车轮与轮毂，**自由度拓扑与 `kc_baseline` 逐位不变**（D2）。凝结机制必须是 `EPIC.md` **F5b** 的既有先例（`_merge_fixed_wheel` / `_fuse_welded_bodies`），且与 20260921 Epic A3（整车侧不凝聚）的关系已在 04 的 PROGRESS 里登记。
   - **(d) 试验台非侵入**：接入前后被测 runtime **逐项比较所有权、参数与几何值**后一致；差异只在外加约束/载荷。只比实体名集合不算通过（`_reown_tires` 正是「名字不变、所有权变」）。
   - **(e) 显式配对**：文件写出悬架→车身挂点配对即按配对连；车身刚体改名 `subframe` 后同一文件仍装配。
   - **(f) 零回归**：7 个既有组合的文档与数值门全绿；**所有基线重录都有登记**；无新增 skip/xfail。
   - **(g) 拖挂铰接**：同一总成内两个车身侧体（牵引车与挂车）经**现有副类型**的铰接副连接并跑通一次；若实测确实需要新的运动副类型，才登记为内核范围、本轮不做（D6：拖挂铰接为**必验**用例）。
3. **G1–G6 逐条确认**（`EPIC.md` 的 Done-When 第 1–6 条），每条给出命令、退出码与产物差异。
4. **终局命令清单**照 `SUBTASKS.csv` 的 `07` 行 `validation_command` 逐条实跑并记录退出码（全量 pytest、架构目录、契约与内核包、ruff、ty、三个数值门）。
5. **K/C 对标口径**：**不得用不带 `--actual-dir` 的 `kc_parity_check`**——那时它拿冻结快照与自身比较，恒过，不构成证据（`EPIC.md`「数值门为独立项」）。需要 K/C 等价对标时必须先跑 `kc_native_probe.py` + `kc_native_c_probe.py`，再带 `--actual-dir artifacts/kc-native-probe` 判定。
6. **读数分层未被打穿**：K/C 与动态的差异由**读数**决定而非装配阶段增删零件，沿用 `EPIC.md` **F9** 的现成范式（装配体保留 `RevoluteJoint`、读数文档把它写成刚性连接）。

## Non-Goals

- 不修改 02–06 的实现代码（本行只做验收；发现问题**报回**，不顺手改）。
- 不重录基线来让门变绿（`EPIC.md` 冻结约束与 D7）。
- 不新增测试以外的交付物；验收脚本与日志写本子任务目录与会话 scratch。
- 不改 C++ 内核、`check_module_layering.py --strict --final` 必须保持绿。

## Constraints

- **写范围**：仅本子任务目录（`raw/`、验收脚本）与会话 scratch；**不改任何生产代码与测试**（除本行自己的验收用例）。
- **起点对照值是 F8**：快速集 1015 passed / 1 xfailed；`tests/cases` 100；`tests/architecture` 147；`tests/adams` 160/47 skip；kernel+contracts 60；`ruff`/`ty`/三个架构门全绿；数值门 3/3 绿。其中 2026-09-29 **经用户授权**重录的两项（车辆动态快照 `vehicle_dynamics_baseline/sha256.json`、`kc_perf_baseline_native.json`）在验收时作为**已登记**的既有变化，不算新增重录。
- **`kc_baseline` 逐位不变是硬门**（D2/D7）：用 `kc_native_probe.py` + `kc_native_c_probe.py` 生成 actual 再与冻结快照比对；**`kc_parity_check.py` 不带 `--actual-dir` 时的通过不算证据**。
- **产物是否变化的判据**：用 `tasks/20260929-01-freeze/snapshot.py --check` 作为「产物是否变化」的判据（**待 01 的 `#1` 完成后跑**；01 的 7 组合快照尚未生成）。本行的默认路径与零回归判定依附于它。
- **不得新增 skip/xfail**；`tests/adams` 的 47 个环境 skip 是既有的，不得增长。
- **凡产物变化必须有登记**，且先给出**独立于结果字节的物理等价判据**（自由度与约束行数、惯量、轮心与接触点几何、轮胎力路径）；质量与质心相同不足以证明等价。
- **全量回归成本高**，本行作为 Epic 收尾必须跑（`EPIC.md` 验证协议 07 要求 `pytest packages/suspension_multibody/tests -q`）；不相关既有失败独立列明，任何新增失败阻断完成。
- 证据只记**已执行**的结果；未执行的项留 `TODO`，不得先填结论。

## Environment

- **Project root**: `c:/杂件/open-kinematics`
- **Language/runtime**: Python 3.12（`uv run --no-sync`）
- **Package manager**: `uv`
- **Test framework**: `pytest`
- **Build command**: 无（纯 Python；内核 DLL 已构建，本行不动内核）
- **Existing test count**: 快速集 1015 passed / 1 xfailed；`tests/cases` 100；`tests/architecture` 147；`tests/adams` 160/47 skip；kernel+contracts 60

## Risk Assessment

- [ ] **采信子任务自报**：本行最大的失效模式 → 每条判据独立实跑，子任务报告只用于定位，不作证据。
- [ ] **弱判据冒充证据**：`kc_parity_check` 不带 `--actual-dir` 恒过；「实体名集合一致」漏掉所有权变化；「只比几何不比分枝」漏掉读数层被打穿 → 每条判据列出可比对的字段（所有权、参数、点坐标、约束端点与类型）。
- [ ] **全量回归耗时（约 33 分钟）** → 本行为 Epic 收尾，必须跑；中途失败保留原始输出，不重录基线。
- [ ] **拖挂/3 轴若需新副类型**：属内核范围（ABI 与版本常量），本 Epic 明确不做 → 以登记形式收口，不擅自扩范围（D6）。
- [ ] **01 快照尚未生成**（`#1` 仍为 `TODO`）→ 「默认路径产物未变」与「零回归」在快照落盘前不可判定；落盘后必须补跑并记录。
- [ ] **既有环境 skip 被误判为新增**：`tests/adams` 的 47 个 skip 全是「Adams 参考 artifacts 不在本副本」的环境跳过，作为既有基线核对，不得增长。
- [ ] **验收脚本本身写错**（枚举组合、比较口径）→ 脚本放本子任务目录并保留原始输出，便于复核。

## Deliverables

- `raw/acceptance_report.md` — (a)-(g) 与 G1–G6 的逐条结论：命令、退出码、原始输出关键段、产物差异（若有）。
- `raw/command_log.md` — 终局命令清单的逐条退出码与耗时。
- 本子任务目录内的验收脚本（生成 actual 产物并比较，不放 scratch）。
- 基线与产物差异登记表（含 2026-09-29 两项已授权重录的对照值）。
- 未完成/未复现项的明确清单（不得用「基本通过」之类的转述替代）。

## Done-When

- [ ] Done-When (a)-(g) 逐条实跑，每条有命令、退出码与产物差异记录。
- [ ] G1–G6 逐条确认，各附对应证据。
- [ ] 终局命令清单（照 `SUBTASKS.csv` 的 `07` 行）逐条退出码已记录；`kc_baseline` 逐位未变有 actual 产物比对证据（带 `--actual-dir`）。
- [ ] 无新增 skip/xfail；`ruff` / `ty` / 三个架构门退出码 0。
- [ ] 所有基线变化有登记，且附独立于结果字节的物理等价判据；`kc_baseline` 无重录。
- [ ] 01 快照落盘后默认路径 `snapshot.py --check` 通过。

## Final Validation Command

```bash
cd /c/杂件/open-kinematics && \
uv run --no-sync pytest packages/suspension_multibody/tests -q && \
uv run --no-sync pytest packages/suspension_multibody/tests/architecture -q && \
uv run --no-sync pytest packages/suspension_contracts/tests packages/suspension_kernel/tests -q && \
uv run --no-sync ruff check . && \
uv run --no-sync ty check . && \
uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check && \
uv run --no-sync python packages/suspension_multibody/scripts/case_parity_check.py && \
uv run --no-sync python packages/suspension_multibody/scripts/kc_perf_gate.py --check
```

> 以上照 `SUBTASKS.csv` 的 `07` 行 `validation_command` 逐条实跑并记录退出码。
> 附加判据（非命令清单的一部分，但必须记录）：
> - 三个架构门脚本：`legacy_surface_gate.py --check`、`check_module_layering.py --strict --final`、`check_composable_release.py --skip-isolation`。
> - `kc_baseline` 逐位不变：`kc_native_probe.py` + `kc_native_c_probe.py` → `kc_parity_check.py --check --actual-dir artifacts/kc-native-probe`。**不得用不带 `--actual-dir` 的 `kc_parity_check`（自比较恒过，不构成证据）。**
> - 产物是否变化：`python .codex-tasks/20260929-assembly-layer-rework/tasks/20260929-01-freeze/snapshot.py --check`（待 01 的 `#1` 完成后跑）。

## Demo Flow

1. **前置**：确认 02–06 均已 `DONE`，且 01 的 `#1` 快照已落盘；否则先记录不可判定项。
2. **(a)-(g) 逐条实跑**：每条先跑命令、保存原始输出，再写结论（退出码 + 产物差异），严禁引用子任务报告。
3. **(f) 零回归**：7 组合文档全绿 + 数值门三项 + `kc_baseline` 逐位比对 + skip/xfail 计数与 F8 对齐。
4. **基线与差异登记**：逐项列出「文件 + 步骤 + 前后值 + 独立于结果字节的等价判据」。
5. **收口**：写 `raw/acceptance_report.md` 与 `raw/command_log.md`；未通过项明确列为阻断，不做「基本通过」式表述。
