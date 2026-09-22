# Epic 进度：20260921-architecture-deviation-closure

## 恢复信息

形态：epic。
进度：**9/9 子任务 DONE**（01 8/8、02 7/7、03 7/7、04 7/7、05 8/8、06 8/8、07 7/7、08 8/8、09 7/7）。
当前：**Epic 完成**。C++ 侧：模块按职责拆分、旧模块与旧 include 清零（`--strict --final` 退出 0）。Python 侧：`report` 建立、作者层归位 `preparation/`、`kernel/capabilities`、旧模块中已无生产调用者的部分删除（`core/`、`model/`、`metrics/`、顶层 `pac2002_scope.py`、`analysis/` 7 文件）；`elements/`(A1) 与 `analysis/vehicle_physics.py`(A2) 按裁决保留并逐项登记。终局验收 G1–G4 全部达成。
下一步：无。Epic 9/9 DONE，终局验收已独立核对 G1–G4（`tasks/20260921-09-acceptance/raw/acceptance_record.md`）。A1 已按 2026-09-22 裁决关闭（生产路径切到 native fixed，车辆基线经授权重录）；A2 按同轮裁决保持现状。
## 证据

- 两路 explorer 完成 C++ 文件/函数与 Python 符号/调用者清单；补充报告确认 external_force_vector/directional 总线及 Python 元件 evaluate 力律。
- 主线程 Read elements/elastic.py:253-288，确认力、切线、能量本构实现，不属于纯报告搬运。
- Python CSV 自检通过：9 行，字段完整，全部 TODO，依赖按序且无环。
- 子任务 01 已实际执行构建、ABI/help、全量测试、符号矩阵、动态哈希、K/C/family/性能门、报告缺口校验和收尾回归；动态哨兵兼容修复后连续两次退出 0，原始证据与既有失败/限制均归档于 `tasks/20260921-01-baseline/raw/`，父级细节见 `VALIDATION.md`。
 - 子任务 01 已实际执行构建、ABI/help、全量测试、符号矩阵、动态哈希、K/C/family/性能门、报告缺口校验和收尾回归；动态哨兵兼容修复后连续两次退出 0，原始证据与既有失败/限制均归档于 `tasks/20260921-01-baseline/raw/`，父级细节见 `VALIDATION.md`。

## 审查

首审 b749492f-4797-47cc-8923-3ca7bca3c9ff：2项阻断（03分层验证、05契约/ABI/数值验证不足），已修订。
复审 f7591a8e-fc60-4878-b708-5ab8ca46ec0d：通过，两阻断清零；剩余非阻断04命令与正文不一致已补 build+strict+architecture。
可选意见处理：G1列入mb_input；01记录实际数值命令输出；pac2002删除附历史架构出处；03/04按模块拆子步验证。
上一轮只交付父级三个规划文件，未创建 Full Single 子任务文件；本轮依据 taskmaster v5 修正。构建与数值测试仍属实施01的实际基线门，不声称已运行。

## 2026-09-21 命名与结构修正

- 用户原话：“计划的命名方式和结构不对，你重新阅读skill修正一下”。
- 已重新加载 taskmaster v5；采用 Epic 父真源与 Full Single 子真源，子 TODO 仅包含叶子步骤。
- Epic 采用日期前缀 20260921-architecture-deviation-closure；子任务采用 20260921-NN-name。日期前缀依据技能示例及本仓库惯例。
- 父目标、验收及依赖保持不变；九个子任务补 SPEC.md、TODO.csv、PROGRESS.md、raw/；所有实施状态 TODO。
- 旧位置迁移到新位置，不保留两份活动真源。
- 本轮结构验证与独立复核：已通过，具体结果见下方记录。

### 本轮验证结果

- fixer交付：九个子任务各SPEC/TODO/PROGRESS及空raw，共27个子文件，父子合计30个文件；拆分66个叶子步骤。
- 主线程结构脚本 `python "$PI_SCRATCH_DIR/check_task_structure.py"` 两次退出0：9子任务、66叶子、30文件、9raw；CSV字段、唯一ID、依赖无环、路径和恢复计数均通过；旧目录不存在。
- 主线程修正04/05凝聚职责交接，05实现顺序与SPEC一致；03/04/05每行明确完整门禁，专项命令不得代替全集。
- 独立首审2eb4fecb-5789-4662-9e3b-905960fb482e：命名结构合格，两条父子步骤/门禁阻断；均已修正。
- 独立复审67d9e751-445b-4bee-b6c4-2b4403752a37：通过，阻断清零，无新增问题。
- 本轮只修正规划，未运行生产构建或数值测试，未把任何实施行标记完成。

## 2026-09-21 基线闭合与实施启动

- 子任务 01 的步骤 1-8 已全部 DONE；动态数组 bytes、兼容 canonical manifest 和组合哈希均与冻结值一致，未重录 baseline。
- 父级 `SUBTASKS.csv` 已将 01 保持为 `DONE`；子任务 02 进入 `IN_PROGRESS`，依赖已满足。
- 子任务 02 当前只允许修改门禁脚本、layering baseline 与架构测试；不修改生产模块源码、API 或包初始化文件。
- 报告契约缺口、真实 Adams 限制和 acceptance 既有失败继续保留为独立状态，不因基线闭合而宣称消除。

## 2026-09-21 子任务 02 完成

- 门禁实现经 code-reviewer 只读审查后发现 5 个缺口并全部修复：`cpp/src/**` 私有头逃逸扫描、动态 import 常量参数盲区、self/aggregate include 计数比较弱化、`extern "C"` 声明被丢弃、registry `occurrences` 未校验。修复各有负例测试。
- 关键验证：扫描扩展后与既有 baseline 的边集/环/互斥边/证据键集逐项一致，未重录 baseline；`--strict` 迁移模式退出 0，`--strict --final` 退出 1（35 条终局 finding 为 03-08 的预期阻断）。
- 验证全集（证据在 `tasks/20260921-02-gates/raw/`）：架构测试 86 passed、kernel 15 passed、legacy_surface `--check` 退出 0 / `--final` 退出 1（预期）、动态 sentinel `--check` 退出 0（26/26 哈希匹配）、ruff/ty/`git diff --check` 通过。
- 未修改生产模块源码（`artifacts.py` 的 diff 属子任务 01 已验证的 NPZ 兼容修复）；C++、CMake、`api.py`、包 `__init__.py` 均未触碰。
- 03-09 调用契约已冻结在 `tasks/20260921-02-gates/PROGRESS.md`；03 可启动。

## 2026-09-21 子任务 03 完成

- mb_base 按职责三拆：mb_config（26 个 env/版本/诊断函数）、mb_numeric（57 个向量/四元数/旋转/曲线函数）、mb_dual（对偶代数与对偶几何三 TU）；聚合声明头 functions.hpp 按符号表拆为三个模块头，39 个 TU 的 include 逐一按符号使用改写。
- mb_linalg→mb_linear、mb_constraint→mb_joint 重命名；关键职责修正：constraint_rows 从 model accessor 迁到 mb_joint，消除 mb_joint<->mb_model 环与一条 mutual 边。
- mb_integrator→mb_solve_dynamic、mb_static→mb_solve_static；共享输入采样（interpolate_input/next_prescribed_input_breakpoint）迁到新模块 mb_input，静态层不再依赖动态层实现。
- 每步完整门通过：构建 0 错误、--strict 退出 0、动态哈希 26/26 逐位一致、K/C parity、8 family parity、kernel 15 passed、arch 86 passed、ABI 七符号 15/30/1/1 ctypes 实测。
- 四份清单同步（CMake 20 目标、MODULES.md 重写、layering_baseline 175 条审核台账、build.py 与 3 个测试硬编码）；未改 Python 生产代码、未改容差、未重录数值基线。
- 证据在 tasks/20260921-03-foundation/raw/（step1-7 全套日志，真实退出码）。

## 2026-09-21 子任务 04 完成

- 上一会话已把 mb_vehicle/mb_suspension 拆解入库（随 HEAD 7ef77a0，提交信息误标 01-03），但收口未完成：assembly_primitives.cpp 未登记 CMake 导致链接失败、三组模块环未消除、baseline/MODULES.md 停在 03 冻结态（门禁 135 条 findings）。本会话完成收口。
- 修复与修正：assembly_primitives.cpp 补进 CMake（MB_ELEMENT_SOURCES）；删 8 处注释提及的无效 include 断开 mb_element↔mb_force↔mb_tire 与 mb_solve_dynamic↔mb_tire_state 环（mutual 3→0、SCC 1→0）；audit_constraint_system 迁 mb_joint、static_rotation_gauge_for_pivot 迁 mb_model，消除 SPEC 禁止的 assembly→solve_static 反向边（诊断文本与失败时机不变）。
- 同步四清单：CMake 20 目标不变（assembly/element/force 已在内）、MODULES.md 重写为 23 模块、layering_baseline 重录并登记 35 条新边 reviewed 台账（189→224）、架构测试两处"迁移期阻断"预期改为终局绿。
- 验证：构建 0 错误；--strict 与 --strict --final 均退出 0；动态哈希 26/26 逐位一致；K/C 与 8 family parity 通过；kernel 15 passed；arch 86 passed；ABI 七符号 15/30/1/1；ruff/ty 通过。证据在 tasks/20260921-04-ownership/raw/。
- 凝聚算法与身份映射按 SPEC 只登记交接（raw/step1_symbol_ownership.md），实现归 05；05 可启动。

## 2026-09-21 计划修订（用户裁决 A1，第 3 级修改）

- **改了什么**：EPIC Goal G2/G3 收窄、Done-When 增「数值门为独立项」、凝聚条款改写、Python 迁移矩阵凝聚行改写；05 SPEC 目标 1 与验收 2/3/5/6/7、05 TODO 第 2/7 行、06 SPEC 约束与验收 5、08 SPEC 目标 1/约束/验收 3、09 TODO 第 3/4 行。
- **为什么**：05 实施前侦查证明原 G2/G3 两项要求在「不得重录数值基线」下不可同时成立。证据：native 无按 body 的力元力旋量通道，`ComponentLoad.global_load/local_load/endpoint` 全由 Python 本构（`api.py:742`）产生；把凝聚迁入 native 会改变 native 收到的 body 集合（实测 `SUSPENSION_MULTIBODY_CONDENSE_WELDS=0` 下 body 22→23），而整车门为字节级 sha256（`case_parity_check.py:398-410`）。详见 `tasks/20260921-05-native-facts/raw/step1_channel_mapping.md` §5-§8。
- **影响子任务**：05（目标与验收 2/3/5/6/7；TODO 2/7）、06（约束、验收 5）、08（目标、约束、验收 3）、09（TODO 3/4）。01-04 不受影响（01-04 已 DONE，且其验收与字节门一致）。
- **未修改**：G1、G4、「不得重录数值基线」、01 冻结的 VALIDATION.md 及其容差。
- **改动级别**：第 3 级（改 Goal/Non-Goals/Done-When），按 taskmaster 规则须重新独立审核，通过前不得把新行置为 IN_PROGRESS。
- **本轮只改规划文件**，未运行生产构建或数值测试，未把任何实施行标记为完成。05 步骤 1 的对照表作为只读交付已产出（`raw/step1_channel_mapping.md`）。
- **已知未闭合项（须显式登记，不判达成）**：用户第二问所选「Python 不再凝聚、weld 送 native 作 fixed 约束」的**手段**，与第一问所选 A1 的「不重录基线、字节门保持绿」冲突（不凝聚使 native body 集合 22→23，整车门字节 sha256 失败）。本次以 A1 为约束上限，只采纳该选项的目标（native fixed 关节作契约等价实现 + 等价性测试），生产路径暂留 Python 作者层。解除条件：单独裁决数值门策略（是否允许重录车辆基线）。

## 2026-09-21 计划修订的两轮独立审核与修订补全

- 首审 `ee519507`：5 项阻断，均为「改了子任务 SPEC 但兄弟真源文件未同步」的同源问题——09 SPEC、SUBTASKS 06 行、06 TODO 第 6 行、08 TODO 第 3 行、EPIC 迁移矩阵 elements 行。已逐条修复。
- 复审 `5a2a503c`：5 项原阻断全部清零；新发现 1 项同源遗漏（三处未限定的「旧目录缺席」：09 TODO 第 6 行、09 SPEC:12、EPIC:136）＋2 项建议（Done-When 补凝聚等价性、矩阵闭合补第三态）＋2 处笔误（09 SPEC 验收编号、EPIC:104 状态）。均已修复。
- 终审 `5851fee5`：确认上述修复。
- **本轮额外修复的两处真源结构损坏**（由我早先的编辑事故造成，已修）：
  - `SUBTASKS.csv`：一次编辑误删 07 行并产生重复 08 行，已重写 06-10 行恢复为 9 行 11 字段、依赖链 01→09 完整。
  - `tasks/20260921-04-ownership/TODO.csv`：第 1-7 行缺 retry_count 字段（导致 notes 被解析为空、列错位），已补 `0` 并修正含逗号 notes 的引号。
- 现状（已实测校验）：SUBTASKS.csv 9 行/11 字段；各子任务 TODO.csv 7-8 行/8 字段，无空字段；9 个子任务目录均含 SPEC.md、TODO.md 三件套（05-09 的 `raw/` 待实施时创建）。
- **两处未采纳的审核建议（低危，登记不阻断）**：05 SPEC 与 06 SPEC 的「待删除登记载体」未点名具体文件（实施时归 06/08 的 PROGRESS）；09 TODO 第 1 行的终局命令用单条 `&&` 链而非逐条记录（notes 已要求逐条留退出码，实施时按 notes 执行）。
- 本轮仍未运行生产构建或数值测试，未把任何实施行标记为完成。计划修订已闭环，05 可启动实施。

## 2026-09-22 计划修订（用户裁决 A2，第 3 级修改）与 05 实施进度

- **进度定位**（本轮开工时实测）：Epic 9 子任务/66 叶子步骤中，01-04 DONE；05 已完成步骤 1-3（步骤 3 已入库 `ecbca0f`，但父级状态未回填）。剩余 34 个叶子步骤：05 步骤 4-8、06（8 步）、07（7 步）、08（7 步）、09（7 步）。
- **基线复现**（未改动任何生产代码前实测）：`build_axle_native.py` 退出 0；`dynamic_hash_sentinel.py --check` 退出 0（26/26 artifact 逐位一致，组合哈希 `e7407656731ed556efc28fb89d8fc69881725b3bfb2f39066eb898986389d48e`，与 01 冻结值相同）；`check_module_layering.py --strict` 退出 0；全量 `tests` 为 `712 passed, 47 skipped, 1 xfailed`。
- **改了什么**：EPIC 的 A2 修订记录与偏差登记口径两节、状态段、G3、事实与修正（vehicle_physics 行）、Python 迁移矩阵（compute_static_wheel_loads 行）、验证协议 05 段、Done-When；05 SPEC 目标 2/范围/验收 4/验证协议末段；05 TODO 第 4 行；09 SPEC 验收 3/4 与 TODO 第 4 行；`SUBTASKS.csv` 第 05 行（状态回填 IN_PROGRESS + A2 口径）；本文件进度/当前/下一步三行。
- **为什么**：05 步骤 4 实施前核实，`compute_static_wheel_loads` 无法在既有冻结约束下迁入 native。证据：`cpp/src/abi/kernel_abi.cpp:854-858` 明示库只暴露一个 run 入口；`mb_solve_static/functions.hpp:61` 的 `solve_static_least_squares` 是方形方程且无 `extern "C"`；实现 `src/solve_static/kernel_static_projection.cpp:212-213` 强制 `matrix.size()==dimension*dimension`；该函数为 `np.linalg.lstsq` 的 3×4 最小范数（`vehicle_physics.py:115-131`），输入是 `build_vehicle(mode="K")`（:110）这一与动态整车算例不同的装配；唯一生产调用者 `vehicle/service.py:30,40`；静轮荷不参与字节级门（`case_parity_check.py:413-421` 的 `_VEHICLE_LEDGERS` 不含该字段）。
- **A2 处置**：05 步骤 4 由「迁入 native」改为「按 A1 同类偏差登记」——保留 Python 最小范数算法与 service 语义，归属在 `results` 映射层收口，登记 file:line + 阻断原因 + 解除条件，**不判为已迁 native**。解除条件：先单独裁决「是否允许扩展 ABI 导出面」或「是否允许在既有契约下新增默认关闭的静力输出块」。
- **A2 第 2 问（后续偏差处置口径）**：06-09 若再遇到「子任务 SPEC 字面要求与 EPIC 冻结约束不可同时成立」，一律按 A1 模式：采纳目标意图、保留生产路径、逐项登记偏差与解除条件，绝不伪造达成。
- **独立审核**：`code-reviewer 3d76f3a3` 只读复审本轮修订。A1-A5 事实前提全部核验成立（含 `kernel_abi.cpp:854-858` 原文、方形矩阵强制、3×4 lstsq、唯一调用者、两门不含静轮荷）；B7「不伪造达成」成立。B6 指出 G3/Done-When 的「反力求解」禁令与新口径外显冲突，B8 列出 8 处兄弟真源未同步——**均已逐条修复**（见「改了什么」）。
- **未修改**：G1、G2、G4、「不得重录任何数值基线」、01 冻结的 `VALIDATION.md` 及其容差、ABI 七符号与版本常量。
- **本轮未运行**生产构建或数值测试以外的任何改动验证；未把任何实施行标记为完成。
- **已知未闭合项（须显式登记，不判达成）**：
  1. 用户此前第二问所选「Python 不再凝聚、weld 送 native 作 fixed 约束」的**手段**与 A1「不重录基线、字节门保持绿」冲突（不凝聚使 native body 集合 22→23，整车门字节 sha256 失败）。本次以 A1 为约束上限，只采纳目标（native fixed 关节作契约等价实现 + 等价性测试），生产路径暂留 Python 作者层。解除条件：单独裁决数值门策略（是否允许重录车辆基线）。
  2. **（A2 新增）** 静轮荷最小范数辅助求解保留在 Python：native 无静力求解 ABI 入口，且 ABI 导出面冻结，故无法迁入。归属已收口到 `results` 映射层并登记保留理由；解除条件同 A2 修订记录。

## 2026-09-22 子任务 05 完成

- **8/8 步骤 DONE**。提交：步骤 1-2 `87b7445`、步骤 3 `ecbca0f`（上一会话产物）、步骤 4 `5700e72`、步骤 5-8 `81e4d57`、父级回填 `0f26cd6`。
- **步骤 4（静轮荷归属收口，A2）**：按裁决保留 Python 最小范数算法本体与 service 调用语义，归属收口于 `VehicleDynamicsResult.static_wheel_loads`；登记 `raw/step4_static_wheel_loads_registration.md`。新增**真判别器**测试 `test_static_wheel_loads_are_the_minimum_norm_solution`：实测平衡矩阵秩 3、零空间维 1（方向 `[1,-1,-1,1]`，`A·v=0` 残差 0）、均匀解 2-范数 7161300 为族内最小；族内其它平衡解满足余额断言却在此失败。
- **步骤 5（decoder 接线）**：新增 `results/element_wrench.py` 只读事实解码面（13 列契约、类型码 1-7、`ElementWrenchRecord`、`decode_element_wrench`、`element_wrench_block`、`rows_per_element`），接入 `results` 包级导出；新增 7 个测试含两条真实 native 运行。开关开：shape `(2,42,13)`、52 条记录（bushing 32 / external 20）、`contract_version=2`；开关关：无块、空 tuple、`contract_version=1`。**`api.py` 生产取值来源未改**。
- **步骤 6（逐通道容差验收）**：按 01 冻结的 K/C 容差与两字节级门逐通道核验，五类元件**无一通过等价性验收**；登记 `raw/step6_channel_acceptance.md`。
- **步骤 7（力律差异登记与阻断切换）**：可复现探针 `raw/step7_law_difference_probe.py` + 输出 `raw/step7_law_difference_probe.log`，实测三类**结构性**差异（非数值噪声；最小差 2.1e-15 与最大差 0.5018 N·m 相差 14 个数量级）：
  1. **固定体端结构性缺失**：`assembly_primitives.cpp:16,33` 对固定体直接早退，native 收到 `chassis` 的 16 行**全部** all-NaN，而同一样本 Python 有 **8** 条非零 `chassis` 力旋量 → 该端在 native 通路无事实来源。
  2. **承载端力矩差**：0.1887–0.5018 N·m（相对 0.108%–0.287%）；根因是 Python `moment = pose_a.rotation @ generalized[3:]`（`elastic.py:446-467`）与 native `arm × f`（`assembly_primitives.cpp:12-40`）的构造不同。
  3. **K 模式无元件事实**：K 模型文档不发 `elements`（`cases/kc_quasi_static/contract.py:118-120`），通道仅 external 行且力/力矩恒为 0。
- **结论**：**阻断切换**——不把 `api.py` 的元件报告取到 native；不删除 Python 本构与力汇总（删除条件未满足）；不保留第二套生产力律（通道默认关闭且未接线）；未改任何容差、未重录任何基线、未新增 ABI 导出。此结论直接约束 06 第 6 项（按 A1 处置为「保留 + 待删除登记」）。
- **门禁实测**（本步全集，证据在 `tasks/20260921-05-native-facts/raw/step5_*.log`、`step4_*.log`）：构建 0；动态哈希 26/26 逐位一致（组合哈希 `e7407656…8d48e` 未变）；K/C parity 0；8 family 0；kernel 15；contracts 22；套件 263 passed / 1 xfailed；ruff 0；ty 0。
- **顺带修复**：既有 ruff I001 违规（`tests/vehicle/test_native_vehicle.py:409` 导入乱序，由本 Epic 步骤 1-2 引入，上一会话遗留）已修；`test_vehicle_physics.py` 新增 docstring 的 D213 已修。
- **06 可启动**：06 步骤 1 的迁移矩阵已冻结于 `tasks/20260921-06-authoring/raw/step1_migration_matrix.md`（逐符号 file:line + 生产调用者 + 新归属 + 未闭合项）。

## 2026-09-22 Epic 完成（9/9）与终局验收

- **子任务 05-09 在本会话完成**，提交序列：05 步骤 4 `5700e72`、05 步骤 5-8 `81e4d57`、05 复审修正 `fb2ea8d`、父级回填 `0f26cd6`、06 `a734038`+`393ee46`、07 `550566e`、08 步骤 3-6 `f7c1f61`、08 收尾 `89ca113`、G3 残余 `core/` 闭合 `8b81c56`、09 终局验收（本提交）。
- **终局 12 条命令 + 2 条构建命令全部退出 0**：构建 0；`--strict` 0；kernel 15；contracts 22；multibody 736 passed / 47 skipped / 1 xfailed（基线 712，增量全为各子任务新增测试，**新增失败 0**）；ruff 0；ty 0；动态哈希 26/26 逐位一致（组合哈希 `e7407656…8d48e` 未变）；K/C parity 0；8 family 0；`legacy_surface --check` 0；两包 `uv build` 0；`git diff --check` 0。
- **G1–G4 逐条达成**（证据见 `tasks/20260921-09-acceptance/raw/acceptance_record.md`）：G1 由 `--strict --final` 退出 0（target missing 0 / legacy present 0 / mutual 0 / cycles 0）证明；G2 由 `report` 建立 + import 面实测干净 + 已无生产调用者旧模块为零证明；G3 由 `core/` 整包删除（关节残差/Jacobian 与反力求解）+ 保留项登记 + 通道逐项证据 + 字节门未动证明；G4 由隔离 wheel 端到端（import/CLI/七符号 15-30-1-1/native 真实运行/artifact 往返）+ 8 family parity 证明。
- **隔离 wheel 验证**：会话 scratch 新建 venv，装三个本地 wheel（contracts/kernel/multibody）；15 个已删模块全部 `ModuleNotFoundError`，`report`/`elements`/`analysis.vehicle_physics` 在位；`suspension_kernel_free` 不存在（未新增导出）。
- **数值门为独立项且未重录**：全程未修改 `dynamic_hash_baseline.json`、`kc_*_baseline*`、`vehicle_dynamics_baseline/`；`report` 切换后字节级门保持绿。
- **本会话新增的独立复审与修正**（未掩盖）：`code-reviewer 3d76f3a3` 审 A2 修订（事实前提成立，G3 口径冲突与兄弟真源同步已修）；`code-reviewer d1a8db9f` 审 05 交付，指出并**经主代理实测确认**两项——力矩差实为参考点语义差（对齐后 2.195e-10 N·m，力律等价）而非力律差异、解码器丢弃纯力矩行（已修 + 补回归测试）。两处原归因与缺陷均已在登记文档中据实更正。
- **三处口径裁决**（08 上报，主代理裁决并登记）：`legacy_surface --check --final` 退 1 系 A1 保留项与门禁自测断言的必然（不放宽门禁、不改测试，判据取 `--check` 退 0）；`core/constraints.py` 的 `residual`/`jacobian` 与其唯一消费者 `ConstraintSystem` 一并删除（只删前者才是半删）；`ty check .` 原退 1 因本 Epic 目录下 05 的证据探针按设计引用已退役模块，已在 `pyproject.toml` 的 `[tool.ty.src] exclude` 加 `.codex-tasks/` 修复。
- **G3 的终局闭合**：08 曾把 `core/{constraints,rigid_body,spatial}` 作为「A1 连带保留」。主代理核查发现它们分别是「仅剩 residual/Jacobian 实现的文件」与自称「re-export 后待 08 删除的转发壳」，且唯一消费者是 `tests/core/test_constraints.py`、生产路径零调用，故整体删除 `core/` 并把断言改造为契约/声明测试（合并进 `tests/core/{test_constraints,test_rigid_body,test_spatial}.py` 与既有的 `tests/axle_dynamics/test_solver_invariants.py`）。删除后扫描 30 → 15 条。
- **既有失败与限制（与 01 一致，不阻断）**：47 skipped / 1 xfailed；真实 Adams 执行缺许可，**未做整车数值等价声明**；动态 acceptance 9 个 case 的既有失败（字节级门本身为绿）；未运行 frozen median-of-N performance protocol。
- **两条未闭合项（登记，不判达成）**：见下一小节。

## 未闭合项（Epic 完成后的裁决结果）

**A1：已关闭（2026-09-22 裁决 A3）。** 用户裁决「A1 改为由 native 的 `fixed` 契约承接焊接语义，可以重录车辆基线」，解除了原「不重录基线、字节门保持绿」的约束上限。生产路径已切换：`preparation/assembly/vehicle.py` 的 `_condense_welded_bodies` 默认**不再融合**焊接体，weld 作为 `WeldJoint` → `kind="fixed"` 交给 native（6 行）；融合实现保留为 `_fuse_welded_bodies`，由 `SUSPENSION_MULTIBODY_CONDENSE_WELDS=1` 恢复（回退有测试覆盖）。这是本 Epic **唯一**被重录的数值基线：`tests/data/vehicle_dynamics_baseline/sha256.json`（8 个 case，按新路径的 native 结果）。`dynamic_hash_baseline.json`、`kc_baseline/`、`kc_perf_baseline*.json` 未改动（组合哈希 `e7407656…8d48e` 仍逐位一致）。

**等价性证据（实测；已按全 8 case 逐 case 实测更正）**：世界系质量性质完全一致——总质量 `3080.0`、世界质心 `[12.987013, 0.0, 160.390]`（全部 8 case）。差异两类、均非物理改变：(a) 布局——多出 `rear_rack` body（22→23）、weld 以 6 行 `fixed` 保留（`constraint_wrench` 29→30）、融合体原点移动导致 chassis 位姿常数偏移；(b) 求解器残差级数值差——`tire_output` ≤`1.4e-9` N、`energy` ≤`3.9e-14` J、`bushing_output` `2.2e-13`（`spring_output` 全 0）。测试：`test_native_fixed_joint_is_what_carries_a_weld`、`test_the_two_weld_routes_agree_on_the_world_mass_properties`。详见 `tasks/20260921-05-native-facts/raw/a3_condensation_switchover.md`。

**A2：保持现状（用户同轮裁决）。** `analysis/vehicle_physics.py` 的 `compute_static_wheel_loads` 继续保留在 Python：native 无静力求解 ABI 入口（`kernel_abi.cpp:854-858` 明示只有 `suspension_kernel_run`），且 ABI 导出面冻结；输入是 `build_vehicle(mode="K")` 装配，与动态整车算例不是同一套。登记 file:line + 阻断原因 + 解除条件见 `tasks/20260921-05-native-facts/raw/step4_static_wheel_loads_registration.md`。解除条件（未变）：单独裁决「是否允许扩展 ABI 导出面」或「是否允许在既有契约下新增默认关闭的静力输出块」。
