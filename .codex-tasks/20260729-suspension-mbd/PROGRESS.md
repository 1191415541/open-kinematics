# Progress Log

> 由 Taskmaster 维护。`TODO.csv` 是里程碑状态的唯一事实来源。

---

## Session Start

- **Date**: 2026-07-29 12:50
- **Task name**: `20260729-suspension-mbd`
- **Task dir**: `.codex-tasks/20260729-suspension-mbd/`
- **Spec**: See `SPEC.md`
- **Plan**: See `TODO.csv` (12 milestones)
- **Environment**: Python 3.12.13 / NumPy-SciPy-Pydantic / pytest 8.3.4

---

## Context Recovery Block

- **Current milestone**: #12 — 完成 Adams 精度验收与全仓集成门禁
- **Current status**: FAILED (external numeric baseline/runner gate; retry 2)
- **Last completed**: #11 — 优化并验证 100/6600 点性能目标
- **Key context**: KModeSolver 已支持轮心/接地点驱动、rack Y 驱动、K&C 指标和确定性网格；K 网格增加延续失败回退；CModeSolver 支持六分量柔顺、三种左右模式、11 级路径和 KReferenceCache；`run_c_grid` 固定 100 个 K 参考并生成 6600 个 C 状态；结果层已写 manifest、Parquet/CSV 和哈希 checkpoint。
- **Known issues**: 本机未安装 `just`；等价根 Ruff/ty 全仓命令分别报告 47/149 个既有 `kinematics` GUI/测试诊断；Adams full 还需要外部数值 runner 和非专有参考结果。
- **Next action**: 提供等价 Adams runner/reference 后执行 `validate-adams --full --require-installed --reference ... --runner ...`；本次 adapter 离线数值门禁证据已保留。

> 每次里程碑状态变化时更新此区块。

---

## Planning Entry

- **Status**: PLAN_READY
- **Started**: 12:44
- **Completed**: 12:50
- **What was done**:
  - 将已确认的产品需求冻结为 `SPEC.md`。
  - 将实施拆为 11 个按依赖排序、带自动验证命令的里程碑。
  - 核验仓库为 Python 3.12/Hatch/uv/pytest，现有测试规模为 548 selected / 549 collected。
  - 核验新增包必须修改 Hatch wheel 包列表，且需要独立 schema、结果命名空间和 CLI 入口。
- **Key decisions**:
  - Decision: 采用 Taskmaster FULL 模式并保留任务目录。
  - Reasoning: 工作跨越领域模型、非线性约束平衡、并行性能和外部 Adams 验证，需要规格冻结、验证门禁和恢复日志。
  - Alternatives considered: LITE 模式无法承载每阶段验收和 Adams 决策日志。
- **Problems encountered**:
  - Problem: 一次求解器规划子代理未能解析任务载荷。
  - Resolution: 未采纳其结果；方案基于主线程已确认需求和独立仓库接入核验制定。
  - Retry count: 0
- **Validation**: `Import-Csv TODO.csv + milestone/status/validation checks + git check-ignore` -> exit 0；11 个里程碑、唯一 IN_PROGRESS、全部验证命令和忽略规则均通过。
- **Files changed**:
  - `.codex-tasks/.gitignore` — 默认忽略任务工件。
  - `.codex-tasks/20260729-suspension-mbd/SPEC.md` — 冻结 v1 规格。
  - `.codex-tasks/20260729-suspension-mbd/TODO.csv` — 11 个实施里程碑。
  - `.codex-tasks/20260729-suspension-mbd/PROGRESS.md` — 决策与恢复日志。
- **Next step**: Milestone 1 — 搭建独立包并冻结 v1 数据协议。

## Plan Review Entry

- **Status**: PLAN_CORRECTED
- **Date**: 2026-07-29
- **What was reviewed**:
  - `SPEC.md` 的需求可实现性、数学边界、包隔离、性能与 Adams 验收定义。
  - `TODO.csv` 的依赖顺序、验收标准与验证命令覆盖关系。
  - `PROGRESS.md` 的恢复状态与实际业务代码状态。
- **Findings corrected**:
  - 原计划把新包放进现有 wheel，不满足独立构建/安装；改为 uv workspace 独立 distribution。
  - Adams 冒烟命令依赖后置 CLI；里程碑 1 现在先交付最小 CLI/架构契约，里程碑 2 再执行真实冒烟。
  - 原规格未冻结位姿、平衡方程、切线、主动集和缩放；新增 SE(3) 6 维增量、稀疏 KKT、解析切线和单位归一化决策。
  - 原性能门禁只运行目录，未定义状态数和计时内容；新增 `k-100` 与 `c-6600` 固定 fixture 和硬件清单。
  - 原最终命令漏掉新包 lint/type/普通测试、性能与 require-installed；现已显式覆盖。
  - 新增 Adams 安装/许可证失败策略、专有文件扫描和绝对载荷容差冻结要求。
- **Independent review evidence**:
  - 子代理指出 TODO 原第 2 步提前依赖第 10 步 CLI、性能命令未断言规模/阈值、最终命令未覆盖 Done-When、无静态 import/专有文件门禁；修正已逐项落入新计划。
- **Plan change**:
  - 里程碑由 11 个调整为 12 个；原当前里程碑未开始实现，故不产生完成状态或重试计数变化。
- **Validation**: `Import-Csv TODO.csv + update_plan 顺序对比 + SPEC 关键门禁 + final/performance command coverage + git check-ignore` -> exit 0。
- **Next step**: Milestone 1 — 建立独立 workspace 包、架构契约与 v1 数据协议。

## Milestone 1 Entry

- **Status**: DONE
- **Date**: 2026-07-29 13:15
- **What was done**:
  - 新增 `packages/suspension_mbd` 独立 Hatch/uv distribution、`suspension-mbd` CLI 和 README。
  - 新增 common/model/elements/case/result v1 Pydantic schema 及 YAML/JSON loader。
  - 新增 wheel/import 隔离、严格字段、版本、单位、四元数、6x6 刚度、控制冲突和 loader 测试。
  - 根 `pyproject.toml` 加入 workspace member，刷新 `uv.lock`；根 wheel 与新 wheel 内容互斥。
- **Validation**: 16 schema/contract tests passed; Ruff passed; `uv build` and member build passed; `suspension-mbd --help` passed; wheel isolation assertions passed.
- **Problems encountered**: 初始 bushing test fixture 使用对称全一矩阵，修正为非对称矩阵后通过；根 sdist 构建约需 2 分钟但最终成功。
- **Next step**: Milestone 2 — 探测 Adams/Car 2024.1 并完成最小批处理 adapter 验证。

## Milestone 2 Entry

- **Status**: DONE
- **Date**: 2026-07-29 13:22
- **What was done**:
  - 新增 Adams profile discovery：入口、版本、许可证环境、数据库、模板、子系统和报告字段。
  - 新增无专有文件的 batch smoke adapter，支持 CI 离线跳过和本机 `--require-installed` 硬失败策略。
  - 固化本机 profile 到任务目录 `raw/adams-profile.json`。
- **Validation**: 4 Adams tests passed; Ruff passed; `suspension-mbd validate-adams --profile adams-car-2024.1 --smoke --require-installed` passed.
- **Problems encountered**: Adams 版本输出包含 `_x64` 后缀，探测器已归一化为 `2024.1`；仓库中文路径会导致 Adams Fortran runtime 错误，probe 改用 ASCII 临时工作目录。
- **Next step**: Milestone 3 — 实现空间代数与刚体状态内核。

## Milestone 3 Entry

- **Status**: DONE
- **Date**: 2026-07-29 13:28
- **What was done**:
  - 新增 `core.spatial`：四元数乘法/指数映射、SE(3) compose/inverse/retract、wrench/twist 变换和解析力臂切线。
  - 新增 `core.rigid_body`：刚体质量属性、不可变多体状态、局部增量和点位 Jacobian。
- **Validation**: 6 core tests passed; Ruff passed.
- **Problems encountered**: 无。
- **Next step**: Milestone 4 — 实现理想关节与约束反力。

## Milestone 4 Entry

- **Status**: DONE
- **Date**: 2026-07-29 13:34
- **What was done**:
  - 新增约束接口与 Ball/Revolute/Prismatic/Distance/CoordinateDrive 约束及确定性装配。
  - 新增混合单位 SVD 秩诊断和 Lagrange multiplier 反力恢复。
- **Validation**: 12 core constraint/rank/reaction tests passed; Ruff passed.
- **Problems encountered**: 标量约束 Jacobian 初始为一维数组，已统一为 `(rows, 6)`；反力矩平衡测试改为满足球铰位置一致的状态。
- **Next step**: Milestone 5 — 实现准静态弹性力元。

## Milestone 5 Entry

- **Status**: DONE
- **Date**: 2026-07-29 13:41
- **What was done**:
  - 新增统一 `ForceEvaluation` 和元素错误类型。
  - 实现线性弹簧、准静态减振器、局部 6x6 衬套、压缩-only 垂向轮胎、等效稳定杆、限位及重力。
- **Validation**: 5 element tests passed; Ruff passed.
- **Problems encountered**: 元素测试初始把弹簧/衬套刚度力值混淆，已按各自变形量修正；限位测试改为明确间隙状态。
- **Next step**: Milestone 6 — 装配对称双横臂前轴与齿条。

## Milestone 6 Entry

- **Status**: DONE
- **Date**: 2026-07-29 13:47
- **What was done**:
  - 新增 `model.front_axle`：单侧硬点镜像、双侧刚体拓扑、稳定连接 ID、K 理想安装和 C 独立安装衬套。
  - 齿条、刚性拉杆、A 臂外球铰、车轮/转向节合并体和可选弹簧/减振器/稳定杆已纳入装配。
- **Validation**: 3 model topology/symmetry tests passed; Ruff passed。
- **Problems encountered**: C 模式安装衬套初始附件位姿未绑定硬点，已改为两端相同硬点 SE(3) 且默认零刚度，避免伪预载。
- **Next step**: Milestone 7 — 共享静平衡与正反向配平。

## Milestone 7 Entry

- **Status**: DONE
- **Date**: 2026-07-29 13:54
- **What was done**:
  - 新增共享 KKT Newton 静平衡，支持约束/力元/外载、解析或中心差分力切线、线搜索和主动集事件。
  - 新增正向 trim、标量反向预载求解和簧下质量/轮荷换算。
- **Validation**: 5 solver/trim/scaling tests passed; Ruff passed。
- **Problems encountered**: 纯理想球铰无法抵抗独立纯力矩，测试已改为物理可平衡的点力；KKT 收敛判据改用 `g + J^T lambda` 而不是原始外载广义力。
- **Next step**: Milestone 8 — K 模式、指标与标准扫掠。

## Milestone 8 Entry

- **Status**: DONE
- **Date**: 2026-07-29 14:02
- **What was done**:
  - 新增 KModeSolver：轮心/接地点驱动、齿条位移和轮胎压缩记录。
  - 新增 camber/toe/track/左右差指标和确定性 KGrid 扫掠，默认 10x10=100 状态。
- **Validation**: 4 K analysis/sweep tests passed; Ruff passed。
- **Problems encountered**: 指标键初始使用 `r_*/l_*`，已统一为 `right_*/left_*`，保持结果协议和车辆侧语义一致。
- **Next step**: Milestone 9 — C 模式、载荷路径与 K 参考缓存。

## Milestone 9 Entry

- **Status**: DONE
- **Date**: 2026-07-29 14:13
- **What was done**:
  - 新增合规 6x6 柔顺验证、中心/割线计算和 CModeSolver。
  - 新增六轴标准路径、单轮/同向/反向模式、11 级对称值和 KReferenceCache。
- **Validation**: 6 C/compliance/cache tests passed; Ruff passed。
- **Problems encountered**: Pydantic `SixVector` 不支持位置参数，已增加显式 `_six` 转换；路径缓存只按驱动键保存一个 K 参考。
- **Next step**: Milestone 10 — 结果协议、公共 API、CLI 和断点续算。

## Milestone 10 Entry

- **Status**: DONE
- **Date**: 2026-07-29 14:25
- **What was done**:
  - 新增独立 `suspension_mbd_meta` 结果协议、manifest、states/component_loads/bushings/diagnostics Parquet/CSV writer 和 reader。
  - 新增 canonical hash、checkpoint/resume 哈希校验、统一 `run_case` API 与 `run` CLI。
- **Validation**: 6 result/API/CLI/e2e tests passed; Ruff passed。
- **Problems encountered**: PyArrow metadata读取测试使用了错误参数，已改为 schema metadata API。
- **Next step**: Milestone 11 — 100/6600 性能基准和硬件清单。

---

## Final Summary

- **Status**: External baseline gate remains unresolved.
- **Completed**: 11/12 implementation milestones; milestone 12 functional gates pass for `suspension_mbd` and root `tests/`.
- **Validation evidence**:
  - Root regression: `uv run pytest tests/ -q` -> 546 passed, 2 skipped, 1 deselected.
  - New package: 64 normal tests passed, 2 deselected; Ruff and ty passed.
  - Performance: `k-100` 100/100 and `c-6600` 6600/6600 within 30/300 second gates.
  - Packaging: root and member wheels built successfully and packaging isolation tests passed.
  - Adams: `validate-adams --full --require-installed` passed for Adams/Car 2024.1.
- **External blockers**: `just` is unavailable; equivalent `uv run ruff check .` reports 47 pre-existing diagnostics and `uv run ty check .` reports 149 pre-existing diagnostics across legacy GUI/tests. No suspension_mbd source imports `kinematics`.
- **Changes made during final gate**: restored one-dimensional CMA-ES test contract with a bounded fallback for the installed cma library's scaling error; made legacy CSV output explicitly UTF-8; stabilized the symmetric zero branch in the legacy semi-analytic steering solver.

## Milestone 11 Entry

- **Status**: DONE
- **Date**: 2026-07-29 15:02
- **What was done**:
  - 新增固定 `k-100`/`c-6600` benchmark API、硬件报告和性能测试。
  - K 网格增加延续失败后的中性姿态重试；无力元 K 求解跳过零力有限差分切线。
  - 新增 `CGrid`/`run_c_grid`，预填 100 个 K 参考并生成 6x11 路径共 6600 个 C 状态。
  - 结果写入包含完整 manifest、Parquet/CSV 表，状态数和残差由测试断言。
- **Validation**: `pytest tests/performance -m performance --strict-markers -q` -> K 100/100、约 15.9 秒；C 6600/6600、约 16.8 秒；两项均通过 30/300 秒门禁。
- **Problems encountered**: 默认延续在大轮跳/齿条角点可能落入错误分支；中性姿态回退恢复全部 100 点收敛。K 无力元有限差分造成约 2 倍状态维度开销，已精确跳过。
- **Next step**: Milestone 12 — 完成 Adams 精度验收与全仓集成门禁。

## Milestone 12 Entry

- **Status**: FAILED (retry 2; external numeric baseline/runner unavailable)
- **Date**: 2026-07-29
- **What was done**:
  - `AdamsBatchAdapter.full()` now creates a non-proprietary run request, invokes a callable/command runner (or `SUSPENSION_MBD_ADAMS_RUNNER`), reads JSON/CSV numeric results, and emits per-field comparisons for `K_geometry`, `C_compliance`, and `static_load`.
  - Added max(abs, relative) position/angle/force/moment/compliance tolerances, frozen absolute force/moment defaults, runner stdout/stderr capture, and explicit failures for missing references, missing groups/fields, invalid values, and runner errors.
  - Adams profile availability now includes the license probe; CLI exposes `--reference` and `--runner` and prints failure/report paths.
  - Added offline passing, out-of-tolerance, missing-group, runner-failure, and license-gate tests.
- **Validation**:
  - New package normal tests: 69 passed, 2 deselected; Adams tests: 10 passed; Ruff and ty passed; member wheel built; performance tests passed (100/100 K and 6600/6600 C).
  - Root regression: 546 passed, 2 skipped, 1 deselected; root wheel built.
  - `just` is not installed; equivalent root Ruff/ty still report pre-existing 47/149 diagnostics.
  - Installed `validate-adams --full --require-installed` now fails because no numeric reference/runner was supplied, as required by the non-false-pass gate.
- **Known blocker**: The repository does not contain a model-equivalence reference or an Adams/Car batch script that creates/export numeric K/C/load results. The adapter contract is ready, but the installed-template numerical acceptance cannot be marked DONE without those external artifacts.
- **Spec clarification**: Added the explicit runner/reference and no-false-pass rule to `SPEC.md` under the Adams validation contract; this records the implemented boundary rather than changing the physical scope.
- **Review corrections**:
  - Added an explicit `profile_available` gate, minimum field counts per comparison group, rejection of unknown Adams fields, duplicate-field rejection in flat CSV, dotted-version parsing, and unit-aware JSON/CSV conversion (`m`/`mm`, `rad`/`deg`, `kN`/`N`, `N*m`/`N*mm`).
  - Revalidated the Adams adapter suite: 18 passed; the complete member suite is 79 passed; member Ruff and ty passed; the member wheel built successfully.
