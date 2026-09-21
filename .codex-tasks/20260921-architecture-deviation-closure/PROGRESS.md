# Epic 进度：20260921-architecture-deviation-closure

## 恢复信息

任务：消除 C++ 模块职责与 Python report/旧模块两处偏差。
形态：epic。
进度：4/9 子任务 DONE（01 8/8、02 7/7、03 7/7、04 7/7）；05-09 TODO。
当前：C++ 侧已完成——01 冻结基线与工具链；02 建立分层与删除门禁；03 拆分基础层（mb_config/mb_numeric/mb_dual/mb_linear/mb_joint/mb_input/mb_solve_dynamic/mb_solve_static）；04 拆解 mb_vehicle（mb_assembly/mb_element/mb_force/mb_tire 落地，模块环与 mutual 边清零，`--strict --final` 全绿）。Python 侧未动——`report`/`preparation` 归位、元件事实通道、旧模块删除、终局验收均待 05-09。
文件：本目录 EPIC.md、SUBTASKS.csv、VALIDATION.md；当前子任务目录 tasks/20260921-05-native-facts/（步骤 1 对照表已交付于其 raw/）。
下一步：执行子任务 05 —— 步骤 2 凝聚等价性登记、步骤 3 按缺口补默认关闭的可选 native 输出通道、步骤 4 静轮荷迁 native、步骤 5 decoder 接线；每步跑 SPEC 验证协议全集与字节级数值门，且不得重录基线（A1 修订口径见 EPIC 修订记录）。

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
