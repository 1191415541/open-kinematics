# Epic 进度：20260921-architecture-deviation-closure

## 恢复信息

任务：消除 C++ 模块职责与 Python report/旧模块两处偏差。
形态：epic。
进度：实施 3/9 子任务完成；子任务 01 DONE（8/8）、02 DONE（7/7）、03 DONE（7/7）；父级叶子完成 22/66。
当前：C++ 基础层迁移完成。mb_base/mb_linalg/mb_constraint/mb_integrator/mb_static 全部消失，mb_config/mb_numeric/mb_dual/mb_linear/mb_joint/mb_input/mb_solve_dynamic/mb_solve_static 落地；constraint_rows 职责修正消除 joint<->model 环。终局缺口 35->12，全部归 04（mb_vehicle/mb_suspension 拆解 + mb_element/mb_assembly/mb_force 落地）。Python 侧缺口不变。
文件：.codex-tasks/20260921-architecture-deviation-closure/EPIC.md、SUBTASKS.csv、VALIDATION.md；下一子任务目录为 tasks/20260921-03-foundation/。
下一步：启动子任务 04，拆解 mb_vehicle（注册/build_model 归 mb_assembly，本构归 mb_element/mb_tire，力总线归 mb_force）并清零 mb_suspension；每步保持迁移模式 --strict 退出 0 并执行冻结数值门。

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
