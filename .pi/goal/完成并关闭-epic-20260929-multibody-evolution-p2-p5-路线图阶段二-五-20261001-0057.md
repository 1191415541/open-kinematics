# Goal

完成 `.codex-tasks/20260929-multibody-evolution-p2-p5/` 全部 23 个子任务（阶段二 p2-01~p2-06、阶段三 p3-01~p3-06、阶段四 p4-01~p4-05、阶段五 p5-01~p5-06），逐条实跑 Epic 的 Done-When (a)–(j)，把该 Epic **彻底关闭**：`SUBTASKS.csv` 各行与其 `TODO.csv` 全部落终态，Epic 的 `EPIC.md`/`PROGRESS.md` 记录真实结论。

## 已核实的起点（本会话实测，非引用旧结论）

- **前置 `S1` 已满足**：阶段一 Epic `20260929-assembly-layer-rework` 的 01/02/03/04/04b/05/06/07 全部 `DONE`；全量 1531 passed / 1 skipped / 1 xfailed；`kc_baseline` 逐位未变；该 Epic 已提交（`8d8c5c0`）。
- **内核可重建**：`uv run python packages/suspension_kernel/scripts/build_suspension_kernel.py` 退出码 **0**，产出 `.../native/suspension_kernel.dll`（该 dll 与 `packages/suspension_kernel/build/` 均被 gitignore，重建不脏工作树）。故 Epic 最大风险项（内核新增元素类型）工程上可行。
- **工作区干净**，唯一未跟踪文件是上游路线图本身 `packages/suspension_multibody/docs/multibody_architecture_evolution.md`。
- **F4 锚点已过期（须登记）**：EPIC 写的 `preparation/vehicle_dynamic.py:205-211` 已是 `_select_assembly_mode`；后轮转向限制现位于 `_validate_steering_topology`（约 `:216-234`，报错原文 `native vehicle dynamics actuates the rack of {steered!r} only; rack_fixed_to_chassis must be true on ...`），第二层仍在 `authoring/vehicle.py:112-119`。p2-01 按代码现状记录，不沿用过期行号。

## Acceptance criteria（每条都可客观判定）

1. **子任务落终态**：`SUBTASKS.csv` 23 行的 `status` 均为 `DONE`；每行对应 `tasks/<id>/TODO.csv` 全部 `DONE`；每行 `PROGRESS.md` 记有已执行的证据路径。若有任何行确实无法达成，该行**不得**标 `DONE`，必须写为 blocked 并在 Epic `PROGRESS.md` 单列，Epic 相应不宣告关闭。
2. **阶段收尾门**：p3-06、p4-05、p5-06 各实跑「快速集 + `tests/architecture` + 数值门三项」，四项退出码均为 0；数值门三项的确切调用为 `dynamic_hash_sentinel.py --check`、`case_parity_check.py`（**无参数**，该脚本不存在 `--check`）、`kc_perf_gate.py --check`。
3. **Epic 收尾门**：`uv run --no-sync pytest packages/suspension_multibody/tests -q` 全绿；`packages/suspension_kernel/tests` 与 `packages/suspension_contracts/tests` 全绿；`ruff check .`、`ty check .` 退出码 0；三个架构门脚本退出码 0。
4. **零回归硬门**：`git status --short -- packages/suspension_multibody/tests/data/` 为空（`kc_baseline`/`dynamic_hash_baseline` 未重录）；`dynamic_hash_sentinel.py --check` 的 26 个 artifact 逐字节一致，combined sha256 = `fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9`；skip/xfail 不增长（基线 1 skipped / 1 xfailed）。
5. **Done-When (a)–(j) 逐条实跑**，命令与退出码记录在 `tasks/p5-06-acceptance/raw/`；不得以「子任务都 DONE」替代。
6. **ABI 治理**：内核 ABI 版本常量的单一真源门（`tests/architecture/test_kernel_abi_version_single_source.py`）保持绿；每一次 ABI 变更逐项登记（改了哪几个常量、改前改后值、依据）。
7. **判据不得弱化**：不删除或放宽既有断言；被反转的既有契约（例如 `tests/cases/test_handling.py` 的「只表达开环工况」）必须同步更新并写明理由与作用线/力路径对照。
8. **收尾交付**：全部改动提交（按阶段或 Epic 收尾提交均可），提交信息说明已完成的行与任何偏差；`EPIC.md` 状态改为终态并附最终实测数字。

## 裁决口径（用户授权我做决定；本轮 `code-reviewer` 委派工具不可用，故由我在契约中落定并写进 EPIC 的 D 表）

- **D1 内核 ABI 变更**：**条件采纳**。先由 p2-02 实测「不改 ABI 是否也能达成 G1」（复用既有 `driven_signals` / `ELEMENT_DAMPER` 通道按实时 ω 求值的可行性）；可行且满足 G1 的三态断言（静止/倒车/抱死）则**优先不改 ABI**；确需新增元素类型时按「单点提交 + 两处真源同步（`mb_config/version.hpp` 与 `kernel/native.py`）+ 逐项登记」执行。
- **D2 闭环所需内核接口**：**先实测再定**。若批式 ABI（`suspension_kernel_run`/`mb_core_run`，无 step/state 入口）确实无法在一次运行内完成「状态→控制→执行器→状态」，则**新增一行专属子任务**做单步接口（含版本常量归属、依赖 p2-02、自身验收），**不得**只登记缺口就放行；闭环目标本身（G8 的 ABS 或 ESC）必须交付，不得降级为开环回放。
- **D3 雅可比来源**：**采纳 Python 侧数值微分、不改 ABI**；但 p3-02 必须先用已知解析解构型标定步长与截断误差，证明精度足以支撑 p3-03 判据 (e) 的独立数值判据；精度不足则回到 D1 的授权范围内评估内核 `constraint_jacobian` 过 ABI。
- **D4 FMI 版本与范围**：**FMU 2.0 Co-Simulation**，只导出模型 + 输入/输出变量，不含 Python 侧求值，不做硬件在环。
- **D5 基线重录口径**：**采纳**（沿用阶段一 D7 并收紧）：`kc_baseline/` 与 `dynamic_hash_baseline.json` 逐字节不变；`vehicle_dynamics_baseline/sha256.json` 与 `kc_perf_baseline_native.json` 本 Epic 期间不动；确需变化的产物逐项登记「文件 + 步骤 + 前后值 + 独立于结果字节的物理等价判据」。
- **D6 新依赖**：**条件允许**——仅 p5-05 的 FMI 库，且引入前须先说明再落地。若最终无法引入，p5-05 交付「可联合仿真的接口契约 + 仓库外独立校验脚本」并**登记为未闭合项**，Epic 相应不宣告完全关闭（不掩盖）。

## Boundaries

- **不得重录任何冻结基线**（`tests/data/kc_baseline/**`、`dynamic_hash_baseline.json`、`vehicle_dynamics_baseline/sha256.json`、`kc_perf_baseline_native.json`）；不得新增 skip/xfail；不得用 `-k`/`--deselect` 长期豁免失败；不得用不带 `--actual-dir` 的 `kc_parity_check.py` 充当证据（自比较恒过）。
- **保持分层方向**：`modeling → templates → subsystems → preparation/studies → cases`；`report/` 不得 import/调用 native/kernel/solver，也不得自求力律；内核提交唯一归属仍只有 `simulation/backend.py`。
- **不改 `FrontAxleModel` / `VehicleModel` 的字段形状**（`api.py` 用 `model_dump(mode="json")` 算 `model_hash`）；两者只降级为适配器，**不删除**。
- **不改轮胎本构**（PAC2002/Fiala）；**不做车型级物理**（4WS/多连杆只交付机制 + 最小端到端用例）。
- **不回退阶段一已交付的结论**（试验台非侵入、轮端凝结、条目清单装配引擎）。
- **不擅自扩范围**：若 D2 需要新增子任务，先在 `SUBTASKS.csv` 登记该行（含写范围、验收、依赖）并说明来源，再实施。
- `packages/suspension_multibody/docs/multibody_architecture_evolution.md`（上游路线图，当前未跟踪）**保持不动**。
- 每个子任务只改其 `SPEC.md` 声明的写范围；跨行共享文件按 EPIC 的分段归属与串行约束执行，不多写者并发改同一文件。
- 交付时如实报告：做了什么、改了哪些文件、验证结果；有真实阻断项就直说，不包装成后续选项，也不声称未执行的验证。

## 说明

本契约在我因工具集限制无法直接改文件时提交；批准后我将按 `SUBTASKS.csv` 的 `depends_on` 顺序（四个只读冻结行可先行）逐行实施，每步重跑门禁，直到 23 行落终态、Done-When (a)–(j) 有实测证据，或出现真实阻断项为止。