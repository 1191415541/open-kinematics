# Epic：消除多体架构两处偏差

- 任务编号：20260921-architecture-deviation-closure
- 创建日期：2026-09-21
- 形态：epic
- 状态：**DONE**（子任务 01-09 全部 DONE，9/9；G1–G4 逐条验收达成，见 `tasks/20260921-09-acceptance/raw/acceptance_record.md`）。两条按 A1/A2 保留的未闭合项显式登记、未判达成，见下方修订记录与父 PROGRESS「未闭合项」。
- 真源：本目录 SUBTASKS.csv；子任务相对路径均相对此 Epic 目录解析。

## 状态与原始需求
状态（2026-09-21 更新）：子任务 01-04 DONE——01 冻结基线与工具链；02 建立终局分层与删除门禁；03 拆分 C++ 基础层；04 拆解 mb_vehicle 并清零终局缺口（C++ 侧 DAG 与 `--strict --final` 全绿）。05-09 待实施（Python 侧：元件事实通道、作者层归位、report、删除、终局验收）；计划已按用户裁决 A1 修订（见下方修订记录与 PROGRESS.md）。

用户原话：“针对上述存在的两处偏差，制定全新计划，消除这两处偏差”；后续：“继续”。

两处偏差：C++ 未按已确认的模块职责拆分，尤其 mb_base、mb_vehicle 仍存在；Python 未建立 report，core/elements/model/analysis 中仍有混合职责。

本次交付计划，不代表授权立即删除核心文件或实施重构。

## Goal

G1. C++ 按职责形成 numeric/dual/config、model/contract/input、joint/element/tire/energy、assembly/force、linear/solve_static/solve_dynamic、cases/output/abi；对应模块使用 mb_ 前缀（abi 除外）。删除 mb_base、mb_vehicle 等被替代模块与旧 include 路径。不能靠重命名掩盖职责混合。
G2. Python 建立 report，报告和派生指标从 analysis/metrics 收敛于此；core/elements/model/analysis 的现役职责各有明确新归属，并删除其中**已无生产调用者**的旧模块；仍有现役生产调用的模块按 G3 的修订保留，不设「必须删完」的强制项，不保留转发壳。
G3. Python 不再保留关节残差/Jacobian 与反力求解实现；力元本构按「05 通道证据 → 06 切换」推进，但允许在 native 力旋量通道尚未启用期间继续存在，其删除以 05 的可选通道证据为前提。原有元件载荷报告保持用户可见字段、单位、方向、参考点和失败证据。**本条「反力求解」指关节约束反力恢复（`core/reactions.py`）；`analysis/vehicle_physics.py:88` 的静轮荷最小范数辅助求解不属本条范围，按 2026-09-22 裁决 A2 登记保留（见 A2 修订记录），未解除前不判达成。**
G4. 保留最新统一 preparation、simulation、results、service 和 artifact 生命周期，不退回历史“仅七个目录”的草案；保持现有 API、CLI、七个 family、Adams 输入输出和历史 artifact 读取功能。

### 修订记录（2026-09-21，用户裁决 A1）

修订原因：05 实施前的侦查证明，原 G2/G3 的两项要求在「不得重录数值基线」约束下不可同时成立，证据见 `tasks/20260921-05-native-facts/raw/step1_channel_mapping.md` §7-§8：

- native 尚无按 body 的力元力旋量输出通道；`ComponentLoad.global_load/local_load/endpoint` 现全部由 Python 本构（`api.py:742`）产生。
- 把凝聚迁入 native 会改变 native 收到的 body 集合（实测 `SUSPENSION_MULTIBODY_CONDENSE_WELDS=0` 下 body 数 22→23），而 `case_parity_check.py:398-410` 的整车门是字节级 sha256。

本次修订为第 3 级计划修改（改 Goal/Non-Goals/Done-When），修订内容：

1. G2 收窄：旧目录删除只覆盖**已无生产调用者**的部分；保留仍有现役生产调用的模块，不设「必须删完」的强制项。
2. G3 收窄：力元本构的删除以 05 的可选 native 通道证据为前提，通道未启用期间允许保留；关节残差/Jacobian 与反力求解的删除保持不变。
3. 凝聚：**已被 2026-09-22 裁决 A3 取代**——原口径为「保留在 Python 作者层；native 的 `kind="fixed"` 关节作为契约等价实现，须有等价性测试，不再要求迁入 mb_assembly」。A3 后生产路径改为**不凝聚 + weld 送 native fixed**，车辆基线经授权重录；详见下方 A3 修订记录。
   - **[已被 A3 取代]** 与用户第二问答复的关系：该选项的目标是「由 native 的 fixed 契约承接焊接语义」，当时只采纳目标、保留手段，因为手段（Python 不再凝聚）与 A1 的「不重录基线」冲突。A3 解除该上限后，手段亦已实施。
4. 元件报告新通道：按向后兼容可选扩展实现，**默认关闭**，默认路径的 artifact 字节不变，两个字节级门保持绿。

未修订：G1（C++ 模块职责与 DAG）、G4（API/CLI/family/Adams/历史读取）保持原文；「不得重录数值基线」保持原文。
### 修订记录（2026-09-22，用户裁决 A2）

修订原因：05 步骤 4 实施前核实，`analysis/vehicle_physics.py:88` 的 `compute_static_wheel_loads` **无法**在既有冻结约束下迁入 native：

- native 侧没有静力求解 ABI 入口，`cpp/src/abi/kernel_abi.cpp:854-858` 明示「the shared library exposes exactly one run entry point: `suspension_kernel_run`」；`mb_solve_static/functions.hpp:61` 的 `solve_static_least_squares` 是**方形**方程求解且无 `extern "C"` 声明，不是 3×4 最小范数。
- EPIC 冻结「ABI 签名及现有导出不变」，故不得新增静态求解导出符号。
- 该函数的输入是 `build_vehicle(vehicle, mode="K")`（`vehicle_physics.py:110`）的 K 模式装配，与动态整车算例（`preparation/vehicle_dynamic.py`）不是同一套装配；唯一生产调用者是 `vehicle/service.py:40`，且静轮荷**不参与**两个字节级门（`case_parity_check.py:413-421` 的 `_VEHICLE_LEDGERS` 与 `dynamic_hash_sentinel.py` 的 26 个 axle artifact 均不含该字段）。

本次修订为第 3 级计划修改（改验收口径），修订内容：**05 步骤 4 由「迁入 native」改为「按 A1 同类偏差登记」**——保留 Python 最小范数算法本体与 service 调用语义不变，只在 `results` 映射层收口归属，逐项登记 file:line + 阻断原因 + 解除条件，**不判该步为「已迁 native」**。

解除条件：先单独裁决「是否允许扩展 ABI 导出面」或「是否允许在既有契约下新增默认关闭的静力输出块」；在此之前不得以登记代替交付。

未修订：G1、G2、G3、G4、「不得重录数值基线」、ABI 七符号冻结。

### 偏差登记口径（2026-09-22，用户裁决 A2 第 2 问）

06-09 实施中若再遇到「子任务 SPEC 字面要求与 EPIC 冻结约束（不重录基线／字节门保持绿／ABI 不变）不可同时成立」，一律按 A1 模式处置：**采纳目标的意图、保留生产路径、逐项登记偏差与解除条件，绝不伪造达成**。集中登记于父 `PROGRESS.md` 的「未闭合项」小节。

### 修订记录（2026-09-22，用户裁决 A3：解除 A1 的字节门上限）

修订原因：用户就 A1 开放项给出裁决——**A1 改为由 native 的 `fixed` 契约承接焊接语义，可以重录车辆基线**；A2 保持现状。这解除了 A1 原先「不重录基线、字节门保持绿」这一约束上限，使 A1 的**手段**（生产路径不再凝聚）得以实施。

本次修订为第 3 级计划修改（改 Done-When 口径），修订内容：

1. **A1 关闭**：`preparation/assembly/vehicle.py` 的 `_condense_welded_bodies` 默认**不再融合**——焊接体保持为独立 body，weld 以 `WeldJoint` → `kind="fixed"` 交给 native（6 行：点重合 + 全相对旋转）。融合实现保留为 `_fuse_welded_bodies`，由 `SUSPENSION_MULTIBODY_CONDENSE_WELDS=1` 恢复（回退开关有测试覆盖）。原「凝聚保留在 Python 作者层」的第 3 条随之**被本次修订取代**。
2. **基线重录（经用户明确授权）**：`tests/data/vehicle_dynamics_baseline/sha256.json` 按**切换到 native fixed 后的 native 执行结果**重录（8 个 case），不与旧 Python 凝聚的数值做对比。这是本 Epic 唯一被改动的数值基线；`dynamic_hash_baseline.json`、`kc_baseline/`、`kc_perf_baseline*.json` **未改动**（实测 axle 侧 26 artifact 组合哈希仍为 `e7407656…8d48e`，kc parity 仍在容差内）。
3. **既有字节门口径更新**：整车门 `case_parity_check` 的 `vehicle_dynamic` 快照随本次重录更新；该门的性质不变（仍是字节级 bit-identity，只是基准改为新生产路径的结果）。`dynamic_hash_sentinel` 与 `kc_parity_check` 维持原冻结基线不动。

**等价性证据（实测，非推断）**：两条路径在同一工况下的**物理量一致**——总质量均为 `3080.0`、世界质心均为 `[12.987013, 0.0, 160.390]`、`tire_output` 与 `energy` 逐位相同。差异仅在**表示层**：融合体的 body 原点被移到质量加权中心（chassis 的 states 呈常数偏移 `px=-0.1077, pz=+0.0192`），且 native 路径多出 `rear_rack` 一个 body（22→23）、`constraint_wrench` 29→30。新增测试 `test_the_two_weld_routes_agree_on_the_world_mass_properties` 钉住「同一物理、两种表示」这一等价性；`test_native_fixed_joint_is_what_carries_a_weld` 钉住两条路径的形态与回退开关。

**A2 未变**：静轮荷最小范数求解仍按 A2 保留在 Python（native 无静力 ABI 入口且导出面冻结），登记与解除条件不变。

**未修订**：G1、G2、G3（除第 3 条凝聚项被本次取代）、G4；「不得重录数值基线」收窄为「除本次经用户授权的 vehicle 基线外不得重录」。

## 事实与修正

- C++ 清单依据 packages/suspension_kernel/MODULES.md 与 CMakeLists.txt；当前分层 baseline 是 20 个模块、86 条头文件边，原检查器只扫描头文件，不能证明 cpp 实现的依赖也正确。
- build_model 位于 cpp/src/abi/kernel_model_build.cpp:23；车辆注册在 src/vehicle/kernel_registration.cpp；标量总线为 external_force_vector，对偶总线为 external_force_directional。
- Python elements/elastic.py:253 的 evaluate 含弹簧力律；api.py:742 在内核返回的状态上调用 evaluate_generalized_forces。因此先前“Python 无本构”的结论不成立，不能把它整体搬到 report。
- core/constraints.py 的对象被 preparation 使用，但 residual/jacobian 不应随数据对象保留。
- analysis/vehicle_physics.py:88 的 compute_static_wheel_loads 用最小二乘求支反力，属于求解；若仍保留该现役功能，应迁到内核而非 report。**2026-09-22 裁决 A2 改判：native 无静力 ABI 入口且导出面冻结，该功能按 A2 修订记录登记保留，不迁内核；本行结论仅「不得迁 report」部分继续有效。**
- 实测 DLL 是 7 个导出符号，不是历史架构草案中的三个函数；本次冻结实际符号面与调用签名，不新增 suspension_kernel_free。

## Non-Goals / Constraints

不改变积分器、K/C 稳态算法、轮胎物理、Adams 等价范围；不新增图表/UI/第三方依赖；不把未实现的整车 Adams 对标标为通过；不为凑目录数删除最新架构模块；不强制把轮胎各实现合并成一个静态库，mb_tire 作为职责边界保留内部子目标。

固定编译器、浮点选项、线程、后端和计算顺序。纯结构阶段要求现有动态数组逐位一致；Python 报告来源切换单独使用冻结的通道级容差验收。不得重录数值基线使失败消失。

旧内部 import 路径属于本次删除范围；包级公开 API 和历史结果读取保持。删除前须完成调用方迁移与前置门禁。新增契约字段只在证明确有输出缺口时采用向后兼容扩展，记录版本策略；ABI 签名及现有导出不变。

## C++ 迁移矩阵

| 原职责 | 新归属 | 必须处理的接缝 |
|---|---|---|
| mb_base 向量/四元数/旋转/普通曲线 | mb_numeric | kernel_base.cpp 按函数拆；对偶曲线函数归 dual，避免 numeric→dual 环 |
| dual.hpp/dual_geometry 与对偶代数 | mb_dual | 仅向 numeric/中性数据依赖 |
| env/版本/运行时诊断配置 | mb_config | 版本字面量仍单一来源，标准库 prelude 不制造反向边 |
| mb_linalg / mb_constraint | mb_linear / mb_joint | 线性算法与关节残差不混入装配 |
| mb_suspension + steering/drive_brake/aerodynamic | mb_element | 标量与方向导数实现一起迁移 |
| 车辆注册 11 个函数、build_model、element reader | mb_assembly | 装配只读中性输入；不得反向依赖 abi 或求解器 |
| external_force_vector/directional、外力重力、广义力汇总 | mb_force | 本构留 element/tire，标量与对偶总线保持次序 |
| kernel_directional.cpp 的轮胎语义 | mb_tire | 与轮胎接触/内部状态共同维护；总线不得残留轮胎力律 |
| mb_integrator / mb_static | mb_solve_dynamic / mb_solve_static | 把共享输入采样移到 mb_input，去掉静态层对动态实现的依赖 |
| build_model 末尾 audit_constraint_system | joint 审计或 ABI 装配后校验 | 禁止 assembly→solve_static 反向边；保留原有失败时机与诊断 |
| mb_model / mb_input / mb_energy | 中性数据与输入采样 | 保持纯数据边界；质量矩阵装配与线性分解分属 assembly/linear |
| mb_cases / mb_output / abi | 工况展开/输出/入口编排 | cases 不承担求解实现；入口调用装配和求解 |

装配范围须核对 Python build_vehicle 的焊接体凝聚。**2026-09-22 用户裁决 A3**：生产路径**不凝聚**——焊接体保持独立 body，weld 由 native 的 `kind="fixed"` 关节（`contract_registry.cpp:24`，6 行）承接，须有等价性测试与体 ID 映射/回退开关登记；`vehicle_dynamics_baseline` 按新路径的 native 结果重录。（原 2026-09-21 裁决 A1「凝聚保留在 Python 作者层」已被 A3 取代。）原文「不能因为当前 C++ 没有同名函数就判为无需迁移」对凝聚一项不再适用，理由与证据见本文件 A3 修订记录及 `tasks/20260921-05-native-facts/raw/step1_channel_mapping.md` §8。

## Python 迁移矩阵

| 旧模块或职责 | 新归属/处置 |
|---|---|
| model/front_axle、model/vehicle 的硬点镜像、命名、schema→声明转换 | preparation/assembly/ 下的 front_axle、vehicle；只作者侧转换 |
| core 的关节/刚体/元素数据字段 | preparation/assembly/types.py；不携带 residual/jacobian/evaluate |
| DOF、约束系统、质量矩阵、反力求解 | C++ assembly/joint/solve；Python 旧实现删除，物理测试转成 native 契约测试 |
| 焊接体凝聚 | **修订（A3）**：生产路径不凝聚，weld 作为 `WeldJoint` → `kind="fixed"` 交给 native（6 行）；融合实现保留为 `_fuse_welded_bodies`（`SUSPENSION_MULTIBODY_CONDENSE_WELDS=1` 回退）；车辆字节基线经授权重录。（原 A1 的「保留在 Python 作者层」已被取代。） |
| core/spatial 的现役输入坐标转换 | preparation/geometry.py；结果侧坐标转换置 results/geometry.py，复核调用方向，避免 report→preparation |
| elements/elastic、assembly 的本构和力汇总 | **修订（A1）**：删除以 05 的可选 native 力旋量通道**实际启用**为前提（先完成通道与 results 解码，再切换 api 元件报告，再删除）。通道未启用期间保留，但须有 file:line 待删除登记（阻断原因 + 启用条件）；不得为删除而启用可选通道，不得在 report 侧复算本构。 |
| metrics、analysis 的轮几何/柔度/统计 | report/metrics/、report/geometry.py、report/compliance.py；只消费结果及只读说明数据 |
| analysis/time_signals | preparation/signals.py；属于输入信号采样，不是报告 |
| VehicleKCTimeDomainSolver 规定运动 replay | simulation/replay.py 编排 + results/timeseries.py 结果聚合；保留“无积分”的原语义 |
| compute_static_wheel_loads | **修订（A2）**：native 无静力求解 ABI 入口且 ABI 导出面冻结，算法本体与 service 调用语义保持原样并登记保留（file:line + 阻断原因 + 解除条件）；不迁 report。详见 A2 修订记录。 |
| analysis/benchmarks | tests/data 的声明式夹具；脚本通过明确夹具路径读取，不导入测试包 |
| core/rank、reactions、model/mass 等无生产调用的求解实现 | 删除实现；保留有价值的物理断言，转 native 测试，逐项登记覆盖关系 |
| analysis/time_domain_physics 等仅导出功能 | 逐符号判定：纯诊断迁 report；求解迁 native；不以“无内部调用”擅自删除公开能力 |
| pac2002_scope | kernel/capabilities.py（惰性读取）、schema 的校验辅助、Adams 证据说明归 adams；删除旧顶层模块 |

pac2002_scope 的搬迁只承接旧架构明确列出的边界清理，不扩展支持模式。results 负责事实通道解码，report 不调用 native、不执行 preparation、不复算本构；io.artifacts 保持唯一读写归属。

pac2002_scope 删除来源：`.codex-tasks/20260917-native-multibody-takeover/ARCHITECTURE.md:155` 的 Python 删除清单；这里只迁移能力读取与校验职责。03/04 的 single-full 在实施前的子级 TODO 中按模块拆步，每步单独构建和跑数值/架构门，禁止一次积累全任务 diff 后才验证。

## 执行顺序与共享文件

子任务见 SUBTASKS.csv（01-04 DONE，05-09 TODO）。采用串行主线：冻结基线→门禁→C++ 基础→C++ 职责→native 输出接管→Python 作者层→report/replay→删除→终局验收。

CMakeLists.txt、layering_baseline.json、api.py、包 __init__.py、compiler/runner/decoder、schema 和共享测试门禁均顺序修改，不并行写。每个任务有独立 task_dir，不能据此假设代码写范围互斥。

九个 single-full 子任务在 tasks/ 下各自持有 SPEC.md、TODO.csv、PROGRESS.md 与 raw/。SPEC 固定交付范围，TODO 仅保存叶子步骤，PROGRESS 保存验证记录和恢复信息。实施前先读取对应 TODO.csv；完整符号清单和数值命令由01实测冻结。迁移矩阵未闭合的符号不允许删除；不允许把未决归属留到最后一次全量测试。

## 验证协议

01 实测现状，不依据旧 DONE：运行现有全量测试、动态哈希、K/C parity、family parity，记录失败/skip 原因。运行产生的中间证据置会话 scratch；需长期保留的冻结夹具与清单是任务交付物。

02 建立新门禁：扫描头和 cpp 的真实依赖、验证目标模块集合及旧模块缺席；显式允许依赖 DAG，fixture 注入反向边/旧导入/绕过 report-native 边界必须失败。重命名阶段先按迁移映射解释旧边，再审核语义新增边，禁止盲目 --record-baseline。

03/04/05 每步构建并同步 DLL，运行动态字节门、K/C/family parity、ABI 七符号与版本门。03/04 同步运行 cpp+header 分层与 architecture 门禁。05 涉及新增输出或契约字段时额外执行 kernel/contracts 测试与版本兼容门；现有输入和输出通道顺序/单位不变。

05 冻结 Python 元件报告的 name/ID、两端、坐标系、作用点、符号、单位、能量、active 状态与 native 输出逐项对照。覆盖弹簧、阻尼、衬套、防倾杆、限位、垂向轮胎及 K/C 两模式；静轮荷按 **A2 登记保留**，其 Python 最小范数算法单独有测试（该测试验证保留语义，不是迁移证据）。**2026-09-21 修订（A1）**：缺口按「向后兼容可选扩展、默认关闭」补 C++ 输出，默认路径 artifact 字节不得变化；不得缺字段填零；不得为让可选通道生效而重录数值基线。凝聚按修订记录第 3 条处理（保留 Python 作者层 + native fixed 关节等价性测试）。

06/07 保留 API、CLI、Adams source rendering、七 family preparation/document bypass、结果异常/partial、历史读取；report 的计算以冻结结果验证，replay 时间与聚合协议不变。

08 删除前后全仓源码/脚本/测试/配置/当前文档扫描（排除历史任务记录和生成目录）；禁止旧路径转发壳、重复 native 提交、第二统一 decoder。扫描器带相对 import、别名和动态 import 测试。

09 独立终局命令：
- uv run python packages/suspension_multibody/scripts/build_axle_native.py
- uv run python packages/suspension_kernel/scripts/check_module_layering.py --strict
- uv run --package suspension-kernel pytest packages/suspension_kernel/tests -q
- uv run --package suspension-contracts pytest packages/suspension_contracts/tests -q
- uv run --package suspension-multibody pytest packages/suspension_multibody/tests -q
- uv run --all-packages ruff check .
- uv run --all-packages ty check .
- uv build --package suspension-kernel
- uv build --package suspension-multibody
- git diff --check

动态哈希脚本、K/C parity、family parity、性能门及 C++ selftest 的具体参数/产物路径在 01 通过 --help 和现有构建配置核实后冻结到 VALIDATION.md；未冻结或不能执行不得开始结构迁移。构建 wheel 后须在隔离环境检查 import/CLI、七符号、native 执行与**已无生产调用者**的旧模块缺席（按 A1 保留的部分不要求缺席），不能只看构建退出码。

## Done-When

独立逐条确认 G1–G4：目标模块职责和 cpp+header DAG 实测通过；**已无生产调用者**的旧模块与旧导入、wheel 残留为零（仍有现役生产调用的模块按 G3 修订保留，须逐项列明其保留理由）；Python 无关节残差/Jacobian 与反力求解实现（反力求解指关节约束反力恢复；`analysis/vehicle_physics.py:88` 的静轮荷最小范数辅助求解按 A2 登记保留，须有保留理由与解除条件），且 native 输出有逐通道证据（力元本构的删除以 05 可选通道证据为前提）；**凝聚等价性有实测证据**：native `kind="fixed"` 关节与 Python 凝聚的等价性测试通过，body ID→凝聚体 ID 映射有登记；公开 API/CLI、七 family、Adams 渲染、历史读取与 success/partial/failed artifact 端到端通过。**数值门为独立项**：`dynamic_hash_sentinel` 与 `case_parity_check` 的字节级门必须保持绿，且未重录任何基线；若某 Goal 与字节门冲突，以字节门为准并将该 Goal 退回修订，不得改基线使其通过。每行 DONE 不代替这些条件。

## 风险与回退

纯结构哈希漂移立即停止当前步，检查编译选项、LTO、运算顺序；不得改容差或重新记录基线。分步骤保留可逆 diff，回退仅本次任务改动。输出接管可能揭示原 Python 与 native 力律差异：登记差异并阻断切换，在既有物理定义下消除，不以“报告用途”保留第二套力律。

真实 Adams 执行需现有安装/许可和相应授权；默认使用已冻结证据与渲染测试，缺少真实执行明确记录，不能声称整车数值等价。最终仍有不相关既有失败则独立列明，任何新增失败阻断完成。

## 目录与命名

按 taskmaster v5 的 Epic + Full Single 结构组织；日期前缀遵循技能示例及仓库既有命名惯例。

```text
.codex-tasks/20260921-architecture-deviation-closure/
├── EPIC.md
├── SUBTASKS.csv
├── PROGRESS.md
└── tasks/
    ├── 20260921-01-baseline/
    ├── 20260921-02-gates/
    ├── 20260921-03-foundation/
    ├── 20260921-04-ownership/
    ├── 20260921-05-native-facts/
    ├── 20260921-06-authoring/
    ├── 20260921-07-report/
    ├── 20260921-08-delete/
    └── 20260921-09-acceptance/
```

每个子目录均含 SPEC.md、TODO.csv、PROGRESS.md、raw/。临时脚本和中间日志仍写会话 scratch；raw/ 为可归档交付证据预留，不存虚构或未执行结果。父 SUBTASKS.csv 管子任务状态，子 TODO.csv 管具体步骤，禁止相互替代。规划交付不将任何实施任务置为 DONE。
用户本轮修正原话：“计划的命名方式和结构不对，你重新阅读skill修正一下”。本轮仅修正规划组织形式，G1–G4、范围和实施顺序不变。
