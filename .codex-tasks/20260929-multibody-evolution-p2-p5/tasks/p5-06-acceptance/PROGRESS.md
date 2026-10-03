# PROGRESS：p5-06 终局独立验收（阶段五）

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p5-06`

## Session Start

- **Date**: 2026-10-02 起，2026-10-03 收口
- **Task name**: p5-06-acceptance
- **Task dir**: `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p5-06-acceptance/`
- **Spec**: 见 `SPEC.md`
- **Plan**: 见 `TODO.csv`（5 步）
- **Environment**: Python 3.12 / uv / pytest；仓库根 `E:/杂件/open-kinematics`；本机 Adams 2025.1.1（`G:\MSC.Software\Adams\2025_1_1`，license 通过）
- **Depends on**: `p5-05`（`SUBTASKS.csv` 第 24 行），并受全局前置 `S1`（阶段一 Epic 01–07 全 `DONE`，`EPIC.md:69`、`EPIC.md:75`）约束

## Context Recovery Block

- **Current milestone**: #5 — 四类交付证据重跑与 Done-When 逐条（**已完成，Epic 收口**）
- **Current status**: **DONE**
- **Last completed**: 全部 5 步；(a)–(j) **十条全部满足**
- **Current artifact**: `raw/done_when_a_to_j.md`（终态逐条记录）、`raw/p504_convergence.md`（p5-04 收敛判据达成过程）、`raw/acceptance.md`、`raw/*.txt`（各门原文）
- **Key context**:
  - 本行是**Epic 收口**：`EPIC.md:287-314` 的 Done-When (a)-(j) **逐条实跑并逐条记录自己的退出码**（不是只记录测试退出码），且「端到端独立验收（**不依赖子任务自证**）」（`EPIC.md:289`）。
  - 判据真源：G7 在 `EPIC.md:95`，G8 在 `EPIC.md:97`，阶段五验证协议在 `EPIC.md:285`，Done-When 在 `EPIC.md:287-314`。
  - **(f) 轮端统一按「独立复验阶段一 04 的实际交付」判**（`EPIC.md:75`、`EPIC.md:93` G6、`EPIC.md:271`）：判据要求「比对**读到的文件路径与内容指纹**」，故证据是**运行期实探**（两侧 `SubsystemDocument.load` 打开同一文档 + 同一 sha256），不是结构性论证。
  - **(h) 总线与闭环按「ABS/ESC 实际反馈闭环 + 同次运行三段数值记录」判**（`EPIC.md:97` G8、`EPIC.md:308`）：状态 → 控制 → 执行器 → 状态四段都有可读数值，**开环回放不算**。
  - 全量基线（`AGENTS.md` 第 3 节）：**1297 passed, 1 skipped, 1 xfailed**，skip 与 xfail **不得增长**（`EPIC.md:233`）。
  - 本行原规格是**只读验收轮**（不改生产代码、不改测试）；实际执行中因用户指令「完成 Epic 直到关闭、需要决策处问 code-reviewer 代理」，发现的具体回归由本行就地修复并逐条登记（见下）。
- **Known issues**:
  - **数值门命令口径（已修正）**：`case_parity_check.py` **只接受 `--family` / `--allow-partial` / `--record`**（实测 `packages/suspension_multibody/scripts/case_parity_check.py:1142-1161`），**没有 `--check`**（`EPIC.md:234` 审核阻断项 5）——本行一律用**无参数**调用。
  - **两次 pytest 调用必须分开**：`suspension_kernel/tests` 与 `suspension_contracts/tests` 并进 multibody 目录会改 `rootdir`，使 `tests.benchmark_fixture` 解析失败（`SUBTASKS.csv` 第 24 行 `notes` 与 `AGENTS.md` 第 1 节）。
  - **`kc_parity_check.py` 不带 `--actual-dir` 时是自比较（恒过），不构成证据**（`EPIC.md:235`）。
  - 全量回归约 30 分钟（`AGENTS.md` 第 3 节），需预留时长并保留原始输出。
  - 禁止用 `-k` / `--deselect` 豁免失败（`AGENTS.md` 第 7 节）。
- **Next action**: 无（本行已完成，Epic 28/28 收口）

---

## Final Summary（2026-10-03 收口）

Done-When (a)–(j) **十条全部满足**，逐条命令、退出码与依据落 `raw/done_when_a_to_j.md`。

### 收口前必须闭合的三项（裁决 `8ab15196` → `003e00a0`）

复核 `8ab15196` 判此前「9/10 满足」偏宽，认定 (a) 不满足、(f) 未证实、(h) 未达成；`003e00a0`
进一步认定 p5-03/p5-05 的阻断可解除、但 **p5-04 的现有数字不通过**（不接受仅以 ON 误差小于
OFF 代替收敛）。三项逐一闭合：

1. **(a) 调用隔离**：旧 helper `_build_wheel_torque_signals` 原先在 `prepare_vehicle_run` 内
   **无条件**调用、事后 `pop` 丢弃——那不是退役。改为真正的**条件调用**：新函数
   `_build_torque_and_demand_signals` 只在 `declared == "none"` 分支调旧 helper（
   `preparation/vehicle_dynamic.py:2071-2075`），opt-in 分支**根本不构造**旧表并对会被声明
   舍弃的显式 `wheel_drive_torque`/`wheel_brake_torque` 按名拒绝。新增 3 条测试锁定，
   **鉴别力已验证**（还原旧实现即失败）。另更正 p2-05 证据文件里一段**从未存在**的
   `declared_demand` 分支（`grep -c declared_demand` = 0，`git log --all -S` 为空）。
2. **(f) 轮端统一**：改用运行期实探，两侧打开同一文档路径 + 同一 sha256
   `7510feade8…4d7f`，probe 体名到达两侧装配产物。证据 `raw/dw_f_wheel_document.txt`。
3. **(h) 三个子项各自修完并验证**（提交 `bd8d84c`）：
   - **总线写执行器**：原先 `variable_damping_L` 直接 `BusError`（`elements` 是数组，path 写的是
     嵌套 mapping）；`motor_torque_FL` 写入成功但 `max|state delta| = 0.0`。改为
     `ActuatorChannel` 声明 `document` + 二选一定位（模型文档 `elements` 按名，或 case 文档
     `blobs` 字节区间）。新增 **2 条轨迹对照**测试，鉴别力已验证。
   - **p5-04 收敛**：见下节。
   - **FMU 输入时间因果性**：原先 `fmi2SetReal` 把新值写入**全部样本**，追溯改写输入历史。
     改为从**首个严格晚于时钟的节点**起写，保留时钟前的历史。新增因果性测试，鉴别力已验证。

### p5-04 收敛判据的达成（`raw/p504_convergence.md`）

`003e00a0` 的预置线：target 固定 `0.30`、尾段 `[120:201]`、`sx` 读 `tire_output[:,0,10]`、
ON `max(e)<=0.05` 且 `mean(e)<=0.03`、OFF `mean(e)>=0.10`、ON `<= 0.8×OFF`。

实测（`tests/cases/test_abs_closed_loop.py`，**7 passed**）：

| 运行 | 尾段 `mean|sx|` | `ptp` | `mean(e)` | `max(e)` |
|---|---|---|---|---|
| OFF | 0.5132 | 0.140221 | **0.2132** | 0.2741 |
| ON（gain=30） | 0.3180 | **0.000002** | **0.0180** | **0.0180** |

四条预置线全部满足（ratio `0.0843`）。达成靠修两个**真缺陷**，不是事后改 target：

- **轮胎接触帧挂在自转的车轮上**（`kernel_model_accessors.cpp:33-35` 在 `tire.frame_body < 0`
  时回退到 `tire.body`），`forward` 随之翻滚、滑移每半圈变号。轴族此前未声明 `frame_body`。
- **`authority` 在 `|slip| <= target` 时恒为 1**（`element/anti_roll.cpp:243-251`），控制器
  只能削减；开环平衡若落在 target 以下，ON 与 OFF **逐位相同**。

### 事故登记（缺陷 C，如实保留）

本行在实施 FMU 缺陷修复时，首版用 `Path()`（即 `.`，恒为真值）当「无暂存目录」哨兵，
`finally: shutil.rmtree(staged)` 因此删除了仓库根目录。`.git/objects` 幸存
（271 commit / 3115 tree / 4288 blob），据此重建了 `1fe7544` 的全部 1535 个跟踪文件。
`artifacts/`（gitignored、从未跟踪、rmtree 不经回收站）不可恢复——**后由本机 Adams 2025.1.1
重建**（提交 `a19c2a5`），skip 由 47 归零。哨兵已改为 `None`
（`grep -c "staged = Path()"` = 0）。

### 各门实跑

- 全量：**2026-10-03 收口后重跑 `1755 passed, 0 skipped, 1 xfailed, 0 failed, 0 errors`**
  （1443.82 s；此前一轮 1750 的读数为收口前状态，测试载体 +5 全部为有意新增）；
  `tests/adams` 单独 **213 passed / 0 skipped**
- `tests/architecture` **147 passed**；contracts + kernel **79 passed**
- `ruff check .` / `ty check .` 全仓 exit 0
- 三架构门全绿（legacy_surface findings 0、module_layering 0 环、composable_release 3 PASS）
- 数值门三项全绿（sentinel 26 artifact 逐字节一致，combined sha256
  `fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9` 等于冻结值；
  `case_parity_check.py` 无参数、8 families；`kc_perf_gate.py --check` 在预算内）
- `git status --short -- packages/suspension_multibody/tests/data/` 为空；ABI 仍 **17/32/1**

### 父级真值已同步

`SUBTASKS.csv` 的 `p5-06` → `DONE`，`EPIC.md` 记 Epic **28/28 DONE**。
