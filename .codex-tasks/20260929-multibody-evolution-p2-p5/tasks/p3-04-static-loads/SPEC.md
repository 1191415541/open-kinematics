# SPEC：p3-04 N 点接触面广义静平衡求解

> 子任务规格。父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p3-04`。
> 形态：`single-full`。

## Goal

把 `SUBTASKS.csv` 中 `p3-04` 的 `acceptance_criteria` 拆成下面 4 条，逐条可判定：

1. **`_WHEELS` 删除，静平衡按实际接触点集合构造方程；解的存在性由载荷相容性判定**：`vehicle/static_loads.py:28 _WHEELS = ("front_left","front_right","rear_left","rear_right")` 从文件中消失；静平衡方程由**实际接触点集合**构造（接触点从哪里来由本行实测确定并登记，不得从四轮名字常量来）。**可判定边界（2026-09-29 复审修订：解的存在性与唯一性是两件事，必须分开判）**：设接触点 `N` 个（未知量 `N` 个）、平衡方程 3 条，矩阵 `A` 为 3×N——**解是否存在由载荷相容性判定**：`lstsq` 后的残差 `‖A x − b‖` 在容差内即可解；**解是否唯一由 `rank(A) == N` 判定**：`rank(A) < N` 时解不唯一，取**最小范数解**并**标记「解不唯一」**（4 轮工况 `rank = 3 < N = 4`，是常态）；**`rank(A) < 3` 只表示三条平衡方程不独立（约束能力不足），与解的存在性、唯一性无关，不得据此报错**；**载荷不相容（残差超容差）才报错点名**，异常消息须含**残差实测值与容差**（**不是秩**）与实际接触点数量 `N`，不是无信息的 `ValueError`。**不得一律对单轮（N = 1）报错**——单轮在**接触点位于质心正下方且无侧向/纵向加速度**（方程相容）时必须给出可解结果，此时 `rank(A) = 1 = N`，**解唯一，不得标成「解不唯一」**。同一单轮几何的矩阵秩**不随载荷改变**，因此**秩无法区分「可解例」与「不可解例」**——两例的差别来自**载荷与接触点几何的相容性**，不来自点数阈值（`EPIC.md` 行 89 的 G4）。`EPIC.md` F8 给出的今天实现是 3×4 平衡矩阵 `[ones(4), x偏移, y偏移]` 的 `np.linalg.lstsq` 最小范数解（`:79-95`）与四点不张成时的 `ValueError`（`:98`；实测今天的判据就是 `rank < 3`，见 `static_loads.py:95-98`），锚点起点按实测复核。
2. **3 轴（6 点接触）与单轮（1 点接触）各有断言，且单轮必须给两个例子**：3 轴构型（6 点接触）跑通静平衡并有断言（解存在、力矩平衡残差在容差内）；**单轮（N = 1）必须给「可解数值例」与「不可解报错例」各一个**——可解例是**接触点位于质心正下方且无侧向/纵向加速度**（或等价的载荷相容条件成立）这类**载荷相容输入**，此时**必须可解**，且 `rank(A) = 1 = N`，**解唯一（不得标记「解不唯一」）**（这是路线图 `docs/multibody_architecture_evolution.md:188` 行「支持 N 点接触面」的字面要求，也是 `EPIC.md` 行 89 的 G4 判据）；不可解例是**载荷与接触点几何不相容**（残差超容差，例如质心投影远离接触点且有侧向/纵向加速度）的输入，此时**报错点名（异常类型 + 消息含残差实测值与容差，不是秩）**。两例的差别**来自载荷与接触点几何的相容性，不来自点数阈值**——同一单轮几何的矩阵秩不随载荷改变，用秩无法区分两例。`EPIC.md` 行 259(b) 明确要求二者各有断言，且**不得把单轮一律写成报错、也不得把单轮标成「解不唯一」**。
3. **4 轮情形与改造前逐位一致（零回归硬门）**：4 点接触（今天的两轴四轮）下，本行改造后的结果与改造前**逐位一致**——这是硬门，因为唯一生产调用点是 `vehicle/service.py:40`（dynamic 服务算静态轮荷，F8 `EPIC.md` 行 128）；`SUBTASKS.csv` 的 `p3-04` `notes` 与 `EPIC.md` 行 255(c) 均明写「4 轮逐位一致是本行硬门」。
4. **力矩平衡残差在容差内有断言，且容差就是解存在性的判据**：求解结果代回平衡方程后的残差（力与力矩两组）有数值断言，容差取值与理由写明；该断言对 3 轴（6 点）与 4 轮两种情形都跑。**该容差同时是「解是否存在」的判据**：残差在容差内即可解（包括 `rank(A) < N` 的解不唯一情形，取最小范数解并标记「解不唯一」），超容差才报错，且报错消息含残差实测值与容差。

## 写范围（允许改的路径）

照 `SUBTASKS.csv` 的 `p3-04` `notes`：**生产写范围与 p3-03 不相交，但测试写范围相交（`EPIC.md` 行 228）——`tests/physics/test_vehicle_physics.py` 归 p3-03，本行只写它新建的 `tests/physics/test_static_loads.py`。两行**硬串行**（`SUBTASKS.csv` 的 `p3-04.depends_on = p3-03`，**不得并行**；测试文件已按文件级切分，但**文件级切分不是并行的理由**）。4 轮逐位一致是本行硬门；p3-05 依赖本行（报表要消费广义静平衡字段）**。

- `packages/suspension_multibody/src/suspension_multibody/vehicle/static_loads.py`（**本行独占**）
- `packages/suspension_multibody/tests/physics/test_static_loads.py`（**本行新建并独占**：静平衡断言全部写这里，见 `EPIC.md` 行 228）与 `packages/suspension_multibody/tests/vehicle/` 下本行**新建**的静平衡断言文件
- 本目录 `raw/`（证据）、临时脚本或会话 scratch

`EPIC.md`「并行与写范围约束」（行 213–221）中与本行相关的条目：

- **p3-03 与 p3-04 生产写范围不相交，但测试写范围相交（`EPIC.md` 行 228）**：本行写 `vehicle/static_loads.py`，p3-03 写 `vehicle/roll_centers.py`，**生产侧不重叠**；但现有 `packages/suspension_multibody/tests/physics/test_vehicle_physics.py:5-10` **同时覆盖**滚转中心与静平衡——该文件**归 p3-03**，**本行不改它**，静平衡断言写入**新建**的 `tests/physics/test_static_loads.py`，`tests/vehicle/` 下同理各写各的新建文件。**两行硬串行（本行在 p3-03 之后，不得并行）**——`SUBTASKS.csv` 的 `p3-04.depends_on = p3-03`，测试文件已按文件级切分，但**文件级切分不是并行的理由**（`EPIC.md` 行 228）。**本行与 p3-03 都依赖 p3-02**；共同前置 p3-02 的引擎契约已冻结；**本行与 p3-03 之间没有接口协商空间**——本行对外暴露的字段形状就是 p3-05 的消费面（见下条）。
- **p3-05 在 p3-04 之后**（`EPIC.md` 行 220）：`p3-05` 依赖本行，因为它要消费广义静平衡的字段（`SUBTASKS.csv` 的 `p3-05.depends_on=p3-04` 与 `notes`）。因此本行必须在 `raw/static_loads_contract.md` 中**冻结输出字段的形状与命名**，供 p3-05 直接对接。
- **`outputs/builtin.py` 的派生输出声明归 p3-05**（`EPIC.md` 行 221）；本行不改该文件，也不改 `report/`（`report/wheel_loads.py`、`report/metrics/vehicle.py` 归 p3-05）。

## 禁止触碰

- `packages/suspension_multibody/src/suspension_multibody/vehicle/roll_centers.py`（归 p3-02/p3-03，本行只读不改）、`report/wheel_loads.py`、`report/metrics/vehicle.py`、`outputs/builtin.py`（归 p3-05）。
- `packages/suspension_multibody/tests/physics/test_vehicle_physics.py`（**归 p3-03**：它同时覆盖滚转中心与静平衡，`EPIC.md` 行 228）——本行**不得改**该文件，也不得改 p3-03 在 `tests/vehicle/` 下自己的断言文件。
- p3-02 交付的**引擎模块本体**——本行若复用其设施，只调用公开接口，不改内部。
- `vehicle/service.py`——它是本行的生产消费点（`:40`，F8 `EPIC.md` 行 128），**本行不改它**；若改它会扩大影响面，需要另行裁量。若实测发现必须改（例如调用签名变化），**登记为范围外并回退给主代理裁决**，不得就地改。
- `packages/suspension_kernel/**`、`mb_config/version.hpp`、`kernel/native.py` 的 ABI 版本常量（ABI 单点提交归 p2-02，`EPIC.md` 行 225）。
- `subsystems/element_build.py`、`templates/roles.py`、`templates/builtin.py`（共享注册文件，`EPIC.md` 行 215 与行 216）。
- `packages/suspension_kinematics/**`（另一包，两解算器产品不得互相导入，F9 `EPIC.md` 行 130）。
- `EPIC.md`、`SUBTASKS.csv`、父 `PROGRESS.md`，以及 `tasks/` 下其它子任务目录——不得代写、不得预填。
- 任何冻结基线文件：`tests/data/kc_baseline/**`、`tests/data/**/sha256.json`、`dynamic_hash_baseline.json`、`kc_perf_baseline_native.json`（`EPIC.md` 行 228 与 D5 行 64）——**禁止重录**。
- `raw/` 中不得放未执行的内容（`EPIC.md` 行 355）。

## 依赖与时机

- 父行 `depends_on = p3-03`（`SUBTASKS.csv` 的 `p3-04`；两者都依赖 p3-02）。跨阶段顺序为「阶段三在第一阶段完成后开工」（`EPIC.md` 行 211）；阶段三主线见行 206（`p3-01 → p3-02 → p3-03 → p3-04 → p3-05 → p3-06`，**p3-04 硬串行在 p3-03 之后，不得并行**）。
- **前置 S1**：本行以阶段一 Epic `.codex-tasks/20260929-assembly-layer-rework/` 的 01–07 全部 `DONE` 为前置（`EPIC.md` 前置一节：**19 行实施行与终局验收行**受此约束；四个只读冻结行不受限）。阶段一未完成之前本行不得置 `IN_PROGRESS`。
- **生产写范围与 p3-03 不相交，但测试写范围相交**（`EPIC.md` 行 228）：本行写 `vehicle/static_loads.py`，p3-03 写 `vehicle/roll_centers.py`；测试侧文件级切分见下条。两者都依赖 p3-02。
- **本行硬串行在 p3-03 之后（不得并行）**（`SUBTASKS.csv` 的 `p3-04.depends_on = p3-03`；`EPIC.md` 行 228 明写「两行硬串行——`SUBTASKS.csv` 的 p3-04 `depends_on = p3-03`，不得并行」）：生产写范围不相交，但**测试写范围相交**（`tests/physics/test_vehicle_physics.py` 归 p3-03、本行只写新建的 `tests/physics/test_static_loads.py`）。**测试文件已按文件级切分，但这不是并行的理由**；**集成验证由主代理在本行与 p3-03 全部落地之后串行跑**——两侧各自测过不等于合起来能跑。
- **本行的下游是 p3-05**（`SUBTASKS.csv` 的 `p3-05.depends_on=p3-04`）：p3-05 要消费本行的广义静平衡字段，故本行必须冻结输出字段形状（`raw/static_loads_contract.md`）。
- **p3-06 依赖 p3-03 与 p3-05**（`SUBTASKS.csv` 的 `p3-06.depends_on=p3-03;p3-05`）。
- **禁止依据旧任务 DONE 结论**（`EPIC.md` 行 320 的口径）——所有事实以本行实跑为准。

## 判据与证据落点

逐条对应 `EPIC.md` 行 255 的 **(a)(b)(c)(d)**；每条写清「跑什么命令、看什么输出、证据落到哪个文件」。

1. **(a) `_WHEELS` 删除 + 按实际接触点集合构造方程** → `raw/wheels_const_removed.md` 与 `raw/static_loads_contract.md`
   - 跑什么：本行 `validation_command` 的第二段 grep（见 Final Validation Command 的逐字原文）；另跑一次实际接触点来源的实跑调用，输出接触点数量 `N`。
   - 看什么：`_WHEELS` 的 grep 零命中；**载荷相容输入求解一次（唯一性由 `rank(A) == N` 判定：`rank(A) < N` 一律取最小范数解并标记「解不唯一」——4 轮与 3 轴都是这种情形；只有 `rank(A) == N` 才解唯一，单轮相容时正是 `1 == 1`）、载荷不相容输入报错一次**；**报错消息必须点名**（含**残差实测值与容差**、实际接触点数量 `N`，**不是秩**）。输出字段形状（字段名、含义、单位约定）写入 `raw/static_loads_contract.md` 供 p3-05 对接。
   - 落点：`raw/wheels_const_removed.md`、`raw/static_loads_contract.md`。
2. **(b) 3 轴（6 点）与单轮（1 点）各有断言** → 新增测试 + `raw/n_point_cases.md`
   - **写哪里**：新增断言全部落在**新建**文件 `packages/suspension_multibody/tests/physics/test_static_loads.py`（`tests/vehicle/` 下同理为新建文件）；**不得改 p3-03 的 `tests/physics/test_vehicle_physics.py`**（`EPIC.md` 行 228）。
   - 跑什么：3 轴构型（6 点接触）跑静平衡并断言解存在 + 残差在容差内；单轮构型（N = 1）**跑两个例子**——**可解数值例**（接触点位于质心正下方且无侧向/纵向加速度，断言解存在、残差在容差内、且 `rank(A) = 1 = N` 故**解唯一**）与**不可解报错例**（载荷与接触点几何不相容、残差超容差，断言 `pytest.raises` 且消息点名含**残差实测值与容差**）。
   - 看什么：三条断言的原文与实跑通过输出；3 轴情形的接触点清单（6 个）；单轮可解例的解与残差数值；单轮不可解例的异常消息全文（含残差与容差）。**不得把单轮一律写成报错**，**也不得用秩作判据**（同一几何的秩不随载荷改变）（`EPIC.md` 行 89 的 G4 与路线图 `:188`）。
   - 落点：新增测试断言 + `raw/n_point_cases.md`（pytest 输出原文与退出码）。
3. **(c) 4 轮逐位一致的零回归硬门** → `raw/four_wheel_bitwise.md`
   - 跑什么：4 点接触（今天的两轴四轮）下，改造前（今日 `static_loads.py:28/:79-95/:98` 路径，锚点见 F8 `EPIC.md` 行 128，按实测复核）与改造后的结果逐位比较；同时跑生产消费点 `vehicle/service.py:40` 的路径。
   - 看什么：**逐位比较的机器可判定输出**（`repr` 级或 `hex`/`struct` 级比较，或 `np.array_equal` 为 True 且逐元素 `repr` 相同），不是「数值接近」。差异非空即失败——**这是零回归硬门**（`EPIC.md` 行 255(c) 与行 316；`SUBTASKS.csv` 的 `p3-04` `notes`）。
   - 落点：`raw/four_wheel_bitwise.md`。
4. **(d) 力矩平衡残差容差断言** → 新增测试 + 并入 `raw/n_point_cases.md`
   - 跑什么：对 3 轴（6 点）与 4 轮两种情形，把解代回平衡方程（力平衡 + 力矩平衡）算残差并断言在容差内；容差同时用于判定解是否存在（残差在容差内即判可解，`rank(A) < N` 时取最小范数解并标记「解不唯一」）。
   - 看什么：容差取值与理由（为何该量级）、残差实测值；断言原文；报错路径的异常消息含残差实测值与容差。
   - 落点：新增测试断言 + `raw/n_point_cases.md`。

## Constraints（冻结约束）

与 `EPIC.md` 「冻结约束」（行 223–233）中与本行相关的条目：

- **不改动任何 K/C 读数**：`tests/data/kc_baseline/` 与 `dynamic_hash_baseline.json` **逐字节不变是硬门**（`EPIC.md` 行 228 与 D5 行 64）；本行**不得重录任何基线**。本行比 p3-03 更重：`static_loads.py` 有**真实生产调用者** `vehicle/service.py:40`（F8 `EPIC.md` 行 128），故「4 轮逐位一致」是本行的硬门（`EPIC.md` 行 255(c)），必须**实跑逐位比较**，不接受「数值接近」。
- **分层方向不可逆**（`EPIC.md` 行 226）：`modeling -> templates -> subsystems -> preparation/studies -> cases`；**`report/` 不得 import native/kernel/solver，也不得自求力律**（`legacy_surface_gate.py` 的 `report_native_import` / `report_constitutive_call` 规则）。本行不写 `report/`。
- **不得引入新依赖**（D6，`EPIC.md` 行 65）：除 p5-05 的 FMI 库外不新增依赖；本行用现有 numpy。
- **不得新增 skip/xfail**（`EPIC.md` 行 229）；`tests/adams` 的环境 skip 是既有的，不得增长。**载荷不相容输入（含单轮的不可解例）必须用 `pytest.raises` 断言**，不得用 skip 表达；**载荷相容输入（含单轮的可解例、以及 `rank(A) < N` 的解不唯一情形）必须实跑求解**，不得预先按 N 的大小或按秩一律抛出；`rank(A) < 3` 只表示三条平衡方程不独立，**不得据此报错**，单轮方程相容时 `rank(A) = 1 = N`、解唯一，**不得标成「解不唯一」**。
- **每步落地后必须重跑**（`EPIC.md` 行 230）：`just check-fast`；改结构后加跑 `tests/architecture`；触及求解路径加跑 `just gate-numeric`。
- **过渡期兼容口径**：本行删除 `_WHEELS` 属于「消灭硬编码」，但**不得**用「N == 3 特判」之类的新硬编码替换旧硬编码；方程构造必须由接触点集合的**数据**驱动（同 `EPIC.md` 行 233 的精神：不得按名字猜身份）。

## 风险与回退

- **4 轮情形向后兼容逐位一致（本行硬门）**：`static_loads.py` 是**在用的生产链路**（`vehicle/service.py:40`，F8 `EPIC.md` 行 128），今天的结果由 3×4 矩阵 + `lstsq` 最小范数解给出（`:79-95`）。回退/缓解：改造**前**先落盘 4 轮的逐位基准（`raw/four_wheel_bitwise.md` 的前半）；改造后逐位比较，**差异非空即回退该步**；若 `lstsq` 的最小范数解选择在 N>4 时与新构造的方程不一致，**只允许在 N>4 的情形改变求解口径**，N=4 必须保持今天的解（必要时对 N=4 保留等价的最小范数路径并说明等价性，说明须**独立于结果字节**——自由度、约束行数、接触点几何、力路径，`EPIC.md` 行 228）。
- **报错行为反转风险**：今天 `:97-98` 的 `ValueError` 判据是 `rank < 3`（实测 `static_loads.py:95-98`），语义是「四点不张成力/力矩平衡」（F8 `EPIC.md` 行 128）；`rank` 与 `residual` 都已在 `StaticWheelLoadResult`（`:39-40`）里，但**报错只用了 `rank`**。新的报错语义是「**载荷不相容**：残差超容差」，判据是**残差**而不是矩阵的秩、也不是接触点数与 3 的大小——**同一单轮几何的 3×1 矩阵秩恒为 1，不随载荷改变，因此用秩无法区分「可解例」与「不可解例」**；按点数判同样与路线图 `:188`「支持 N 点接触面」和 `EPIC.md` 行 89 的 G4 冲突（单轮会被误判为一律报错）。缓解：既有测试若断言了旧 `ValueError` 的消息，须检查语义是否真的等价；**语义不同就同步更新测试并登记理由**，不得静默改消息；改测试时只改本行新建的 `tests/physics/test_static_loads.py` 或本行新建的 `tests/vehicle/` 文件，**不得改 p3-03 的 `tests/physics/test_vehicle_physics.py`**。报告消息须含**残差实测值与容差**，便于把「载荷不相容（不可解）」与「解不唯一（`rank(A) < N`）」两种情形分开；`rank(A) < 3` 只表示三条平衡方程不独立，不参与报错判定。
- **依赖方向**：本行若复用 p3-02 的引擎设施，可能加重 `vehicle → subsystems` 依赖。缓解：先跑 `tests/architecture/test_import_boundaries.py` 与 `check_module_layering.py --strict --final`（`EPIC.md` 行 315 的同口径）。
- **接触点来源未定**：实际接触点集合从哪个既有事实面取（装配运行时的点表、轮心表、还是调用方传入）**须实测确定并登记**；若需要改 `vehicle/service.py` 才能传接触点，登记为范围外并回退给主代理裁决（`SPEC.md` 禁止触碰节）。
- **既有失败**（`EPIC.md` 行 320）：起点值取自 p3-01 的 `raw/start_state.md`；任何新增失败阻断完成，不相关的既有失败独立列明。

## Done-When

- [ ] `raw/wheels_const_removed.md` 非空：本行 Final Validation Command 的 grep 段退出码 0（零命中）。
- [ ] `raw/static_loads_contract.md` 非空：输出字段的形状与命名冻结，p3-05 可据此对接。
- [ ] 3 轴（6 点接触）与单轮（N = 1）**各有断言**：前者解存在 + 残差在容差内；单轮**可解数值例**（接触点位于质心正下方且无侧向/纵向加速度）实跑可解且 `rank(A) = 1 = N`、**解唯一** + **不可解报错例**（载荷不相容、残差超容差）报错且消息点名含**残差实测值与容差**（`EPIC.md` 行 89 与行 259(b)、路线图 `:188`）；pytest 通过，输出落在 `raw/n_point_cases.md`。静平衡断言全部写在**新建**的 `tests/physics/test_static_loads.py`，未改 `tests/physics/test_vehicle_physics.py`。
- [ ] `raw/four_wheel_bitwise.md` 非空：4 轮的改造前基准与改造后结果**逐位一致**（机器可判定输出），差异为空。
- [ ] 力矩平衡残差（力与力矩两组）在容差内有断言，容差取值与理由已写明；该容差同时是解存在性的判据（残差在容差内即判可解；解是否唯一另由 `rank(A) == N` 判定，`rank(A) < 3` 只表示方程不独立）。
- [ ] `uv run --no-sync pytest packages/suspension_multibody/tests/physics packages/suspension_multibody/tests/vehicle -q` 通过。
- [ ] `kc_baseline` 与 `dynamic_hash_baseline` **逐字节未变**，未重录任何基线；无新增 skip/xfail。
- [ ] `check_module_layering.py --strict --final` 0 环、`test_import_boundaries.py` 通过、`legacy_surface_gate.py --check` 退出码 0。

## Final Validation Command

```bash
uv run --no-sync pytest packages/suspension_multibody/tests/physics packages/suspension_multibody/tests/vehicle -q && bash -c '! grep -rn -e \_WHEELS = (\\\"front_left\\\"\" packages/suspension_multibody/src/suspension_multibody/vehicle'"
```
