# PROGRESS：p4-05 终局独立验收（阶段四）

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p4-05`

## Session Start

- **Date**: 2026-10-01
- **Task name**: p4-05-acceptance
- **Task dir**: `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p4-05-acceptance/`
- **Spec**: 见 `SPEC.md`
- **Plan**: 见 `TODO.csv`（6 步，全部 `DONE`）
- **Environment**: Python 3.12 / uv / pytest
- **Status**: **DONE**

## Context Recovery Block

- **Current milestone**: #1 — 起点值与既有失败实测
- **Current status**: NOT_STARTED
- **Last completed**: 无（本子任务尚未开工）
- **Current artifact**: `SPEC.md` / `TODO.csv`（规划产物）
- **Key context**:
  - `depends_on = p4-04`；阶段四串行主线 p4-01 → p4-02 → p4-03 → p4-04 → p4-05（`EPIC.md:207`）。
  - **独立于子任务自证**（父行 `notes` 逐字）：G5（`EPIC.md:87`）与 G6（`EPIC.md:89`）的每条判据都要有独立命令直接读产物或跑既有路径，不得只跑 p4-02/p4-03/p4-04 自己新增的用例（`EPIC.md:285` 同口径）。
  - **基线硬门**：`kc_baseline/` 与 `dynamic_hash_baseline.json` 逐字节不变（`EPIC.md:228`、D5 见 `EPIC.md:64`）；F17 的「无 ARB 基线」不得用来放松（`EPIC.md:150`）。
  - **数值门口径**（`EPIC.md:230` 与 `EPIC.md:234`）：三项且缺一不算全绿——`dynamic_hash_sentinel.py --check`、**`case_parity_check.py`（无参数）**、`kc_perf_gate.py --check`；`case_parity_check.py` 的参数面实测只有 `--family` / `--allow-partial` / `--record`（`case_parity_check.py:1142-1161`），**没有 `--check`**；`kc_parity_check.py` 不带 `--actual-dir` 恒过，不构成证据（`EPIC.md:231`）。
  - **不得新增 skip/xfail**（`EPIC.md:229`）；不得用 `-k` / `--deselect` 长期豁免失败（`AGENTS.md` 第 7 节）。
  - 全局前置 `S1`：阶段一 01–07 全部 `DONE`；**未完成之前不得置 `IN_PROGRESS`**（`EPIC.md:75`）。
- **Known issues**:
  - 尚未开工，故起点值（快速集与 architecture 的 passed/skipped/xfailed 计数、`tests/adams` 的环境 skip 计数、既有失败清单）**尚未实测**；第 1 步必须先测。
  - 本行只跑不改 `packages/**`；任何判据失败退回对应行（p4-02 / p4-03 / p4-04 或前置行）修复后重跑本行。
  - 本行结论只覆盖阶段四范围；父 Epic 的完成还要看 p5-06（`EPIC.md:281`）。
- **Next action**: 先实测起点值（快速集、`tests/architecture` 的计数与既有失败清单）并落 `raw/baseline_start.md`，然后逐条独立实跑 G5（`EPIC.md:87`）与 G6（`EPIC.md:89`）的判据，最后跑快速集、`tests/architecture` 与**数值门三项**（`dynamic_hash_sentinel.py --check`、`case_parity_check.py` 无参数、`kc_perf_gate.py --check`）并比对 skip/xfail 增量。

---

## Final Summary

p4-05 **DONE**（阶段四终局独立验收）。

| 步 | 结果 |
|---|---|
| 1 起点 | 快速集 `1188 passed, 1 xfailed`；skipped 0；**无既有失败**（`raw/baseline_start.md`） |
| 2 G5 | `grep "upright_L"\|"upright_R"` 在 `subsystems/` **exit=0**；`tests/subsystems`+`tests/vehicle_assembly` **200 passed**；防倾杆 4 端口与悬架 `arb_mount_L/R` 配对已验证（`raw/g5_g6_rerun.md`） |
| 3 G6 | 父行命令 `! grep -rn VerticalTireElement …/assembler.py` **exit=0**；`grep -c isinstance assembler.py` = **0**；单轴与整车同一份 wheel 子系统（`raw/g5_g6_rerun.md`） |
| 4 逐位 | sentinel sha256 `fdfd5a6b…eebc9` 未变；`kc_baseline` 三文件哈希本行自算；`git status tests/data/` **空**（`raw/baseline_bitwise.md`） |
| 5 门禁 | 快速集 / `tests/architecture`(147) / 数值门三项 **全部 exit=0**（`raw/gates.md`） |
| 6 skip/xfail | 增量 **0**（`raw/skip_xfail.md`） |

**本行独立验收发现并如实记录的一处**：把 G6 的 grep 范围从父行指定的
`assembler.py` 放宽到 `subsystems/` + `preparation/` 后，仍有三处 `VerticalTireElement`
命中（`element_build.py` 的生产者、`rig_link.py` 的读、`vehicle_dynamic.py` 的刻意拒绝）。
用 `git show HEAD:` 逐条比对确认**三处都在阶段一提交 `8d8c5c0` 就已存在**，
即本 Epic 未引入任何新的类型过滤。**本行不自行修**（父行 notes 明令发现越界即退回）。

**未闭合项（不在本行范围，登记给 Epic 收尾）**：
- 防倾杆子系统尚未接入 `si_assembly.py` 的贡献派发链，故装配产物里还没有防倾杆自己的体
  （端口与配对已就位）。
- **未做 K/C 对标**：未运行不带 `--actual-dir` 的 `kc_parity_check.py`（自比较恒过，禁作证据）。
- **未跑** `tests/adams`（不触及轮胎力律）、全量回归（Epic 收尾 p5-06）。
- 未重录任何基线。
