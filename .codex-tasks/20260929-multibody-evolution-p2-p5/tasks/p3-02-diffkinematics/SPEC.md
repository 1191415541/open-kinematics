# SPEC：p3-02 微分运动学引擎（速度旋量与空间瞬轴）

> 子任务规格。父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p3-02`。
> 形态：`single-full`。

## Goal

把 `SUBTASKS.csv` 中 `p3-02` 的 `acceptance_criteria` 拆成下面 5 条，逐条可判定：

1. **引擎的输入面是装配运行时的约束集合与点表**：新引擎的公开入口接收「本次装配的约束集合」（`constraints` / `ideal_constraints`）与「点表」（`points`），输入契约在签名与文档字符串中写明；引擎**不得**从名称字符串或模型对象反查几何。锚点起点按 `EPIC.md` F9（行 130）给出的可复用设施：`subsystems/assembler.py:163/283` 的 `constraints` / `ideal_constraints`、`:281` 的 `points` 表、`:242-243` 的 `wheel_centers`。
2. **输出是速度旋量（`omega_rel`, `v_rel`）与空间瞬轴**：引擎对给定构型与给定广义坐标扰动返回相对角速度、相对线速度与**空间瞬轴**（轴线上一点 + 方向 + 螺距，或等价的最小表示——具体表示由本行确定并在文档字符串中写明）；返回值是有限数值且形状固定。
3. **瞬轴算法在已知解析解的构型上验证**：至少一条断言——**单个旋转副的瞬轴就是它的轴线**（铰链轴方向与轴上一点与声明一致）；`EPIC.md` 行 251(b) 明确要求此断言存在。判据可判定：断言比较方向（容差内）与轴上一点到轴线的距离（容差内），不是「返回非空」。
4. **数值微分的步长与截断误差有说明，且与今日 `_instant_center` 可对照**：文档字符串或模块注释写明数值微分使用的步长与截断/舍入误差量级；在双叉臂构型上，新引擎给出的滚转中心（或等价的横向瞬心）与今天的 `_instant_center` 结果**可对照**——**不要求逐位一致**，但任何差异必须给出**已解释的来源**（例如：今天的实现把上/下臂内外点**平均**后在 `[y,z]` 平面求两条臂线交点，F7 `EPIC.md` 行 126；新引擎走约束雅可比的数值微分）。
5. **引擎路径无硬点名称嗅探，分层方向不被破坏**：在**引擎新模块**的路径上 `grep -n "UPPER_\|LOWER_\|UCA_\|_BODY_ALIASES"` **零命中**；`check_module_layering.py --strict --final` 保持 0 环（`EPIC.md` 行 226 与行 226 引用的分层方向），且 `test_import_boundaries.py` 绿（`EPIC.md` 行 315：新引擎若读装配运行时会加重 `vehicle → subsystems` 依赖，必须先跑通再落地）。

## 写范围（允许改的路径）

照 `SUBTASKS.csv` 的 `p3-02` `notes`：**写范围新增引擎模块与 `vehicle/roll_centers.py` 的调用段（`roll_centers` 主体重构归 p3-03，但同一文件，故 p3-03 依赖本行串行）**。

- `packages/suspension_multibody/src/suspension_multibody/vehicle/` 下**新增的引擎模块**（具体模块名与位置由本行确定并登记；`EPIC.md` 行 226 的分层方向不可逆，落点必须使 `vehicle/` 既有依赖方向不反向）
- `packages/suspension_multibody/src/suspension_multibody/vehicle/roll_centers.py`——**仅调用段**（把 `compute_vehicle_roll_centers` 的入口改为可路由到新引擎；主体重构与硬点别名表删除归 p3-03，同一文件不得并行）
- 对应测试：`packages/suspension_multibody/tests/physics/`、`packages/suspension_multibody/tests/vehicle/`，以及本行新增引擎的专属测试文件
- 本目录 `raw/`（证据）、临时脚本或会话 scratch

`EPIC.md`「并行与写范围约束」（行 213–221）中与本行相关的条目：

- **p3-03 与 p3-04 都依赖 p3-02，且二者之间硬串行**（`SUBTASKS.csv` 的 `p3-04.depends_on = p3-03`；`EPIC.md` 的并行与写范围约束条）：写范围分别是 `vehicle/roll_centers.py` 与 `vehicle/static_loads.py`（生产侧不相交），但测试写范围相交且调度上 **p3-04 在 p3-03 之后、不得并行**。本行是二者的**共同前置**，因此本行的**引擎公开接口（输入/输出契约）必须在开工时就冻结并写进本节**——p3-03 与 p3-04 都不与对方协商接口，只对着本行冻结的签名写。
- **`outputs/builtin.py` 的派生输出声明归 p3-05**（`EPIC.md` 行 221）；**p3-05 在 p3-04 之后**（行 220）。本行不碰 `report/` 与 `outputs/`。
- **`subsystems/element_build.py` 被多行触及且必须串行**（`EPIC.md` 行 216）：本行若需要读构造后的约束集合则**只读**，不得改 `element_build.py`。
- 本行与 p2-03 分属不同阶段，`element_build.py` 的**构造分派段**归 p2-03（`EPIC.md` 行 216）——本行不写该段。

## 禁止触碰

- `packages/suspension_multibody/src/suspension_multibody/vehicle/static_loads.py`（归 p3-04）与 `report/`、`outputs/builtin.py`（归 p3-05）。
- `vehicle/roll_centers.py` 的**硬点别名表与 `_line_intersection` 二维交点路径**——删除归 p3-03（`SUBTASKS.csv` 的 `p3-03`；本行只改调用段）。
- `packages/suspension_kernel/**` 与 `mb_config/version.hpp`、`kernel/native.py` 的 ABI 版本常量（**D3 已建议不改 ABI**，`EPIC.md` 行 62；ABI 单点提交归 p2-02，`EPIC.md` 行 225）。
- `packages/suspension_kinematics/src/suspension_kinematics/jacobians.py`——另一包（SymPy 生成），**未被 multibody 引用**，两解算器产品不得互相导入（`EPIC.md` F9，行 130）。
- `subsystems/element_build.py`、`templates/roles.py`、`templates/builtin.py`（共享注册文件，`EPIC.md` 行 215 与行 216）。
- `EPIC.md`、`SUBTASKS.csv`、父 `PROGRESS.md`，以及 `tasks/` 下其它子任务目录——不得代写、不得预填。
- 任何冻结基线文件：`tests/data/kc_baseline/**`、`tests/data/**/sha256.json`、`dynamic_hash_baseline.json`、`kc_perf_baseline_native.json`（`EPIC.md` 行 228 与 D5 行 64）——**禁止重录**。
- `raw/` 中不得放未执行的内容（`EPIC.md` 行 355）。

## 依赖与时机

- 父行 `depends_on = p3-01`（`SUBTASKS.csv` 的 `p3-02`）。本行是阶段三主线的**第二行**（`EPIC.md` 行 206）。
- **前置 S1**：本行以阶段一 Epic `.codex-tasks/20260929-assembly-layer-rework/` 的 01–07 全部 `DONE` 为前置（`EPIC.md` 前置一节：**19 行实施行与终局验收行**受此约束；四个只读冻结行 p2-01/p3-01/p4-01/p5-01 不受限、可立即开工）；阶段一未完成之前本行不得置 `IN_PROGRESS`。
- **与 p3-03 / p3-04 的时机**：二者都依赖本行（`SUBTASKS.csv` 的 `p3-03.depends_on=p3-02`、`p3-04.depends_on=p3-02`，`EPIC.md` 行 220）；本行落地后二者**硬串行**（`p3-04.depends_on = p3-03`，不得并行）；p3-05 在 p3-04 之后；p3-06 依赖 p3-03 与 p3-05。
- **本行的接口冻结义务**：由于 p3-03 与 p3-04 要并行，本行必须在 `raw/engine_contract.md` 中落盘**引擎的公开签名与返回值形状**（含瞬轴表示与容差约定），使两个后续行可以各自独立对接，不需要彼此协商。
- **禁止依据旧任务 DONE 结论**（`EPIC.md` 行 320 的口径）——所有事实以本行实跑为准。

## 判据与证据落点

逐条对应 `EPIC.md` 行 251 的 **(a)(b)(c)(d)(e)**；每条写清「跑什么命令、看什么输出、证据落到哪个文件」。

1. **(a) 引擎输入 / 输出契约** → `raw/engine_contract.md`
   - 跑什么：把引擎的公开签名与返回值形状落盘，并附一次最小调用的实跑输出（约束集合与点表来自装配运行时；锚点起点为 `subsystems/assembler.py:163/283`、`:281`、`:242-243`，F9 `EPIC.md` 行 130，按实测复核）。
   - 看什么：输入是约束集合 + 点表（不是名称字符串）；输出是 `omega_rel` / `v_rel` / 瞬轴三元（点、方向、螺距或等价表示），形状固定、可复现。**这份契约就是 p3-03 / p3-04 的对接面**。
   - 落点：`raw/engine_contract.md`。
2. **(b) 已知解析解构型的瞬轴验证** → 新增测试 + `raw/axis_assertions.md`
   - 跑什么：对**单个旋转副**构造最小构型，跑引擎并断言瞬轴等于该副的轴线（方向与轴上一点）。
   - 看什么：断言原文与实跑通过输出；容差取值与理由。
   - 落点：新增测试文件的断言 + `raw/axis_assertions.md`（pytest 输出原文与退出码）。
3. **(c) 数值微分步长与截断误差说明 + 与今日 `_instant_center` 的对照** → `raw/numerics_and_comparison.md`
   - 跑什么：在双叉臂构型上分别跑今日实现（`vehicle/roll_centers.py:81-99 _instant_center`，F7 `EPIC.md` 行 126）与新引擎，把两侧结果并列；另记录所选步长下扰动量与残差的量级。
   - 看什么：**并列对照表**（不是「一致/不一致」一句）+ 差异来源的**逐条解释**（今天的上/下臂点平均后在 `[y,z]` 平面求线交 vs 新引擎的约束雅可比数值微分）；步长与截断误差量级有数字。
   - 落点：`raw/numerics_and_comparison.md`。
4. **(d) 无硬点名称嗅探** → `raw/no_name_sniffing.md`
   - 跑什么：`bash -c '! grep -rn -e UPPER_ -e LOWER_ -e UCA_ -e _BODY_ALIASES <本行新增引擎模块路径>'`（路径按本行实际落点写死）。
   - 看什么：命中数为 0 的实测输出（命令 + 退出码 + 输出原文）；同时记 `_BODY_ALIASES`（`subsystems/geometry.py:226`，F16 修正项 `EPIC.md` 行 148）在本行的取用与否。
   - 落点：`raw/no_name_sniffing.md`。
5. **(e) 分层与导入边界** → `raw/layering.md`
   - 跑什么：`check_module_layering.py --strict --final`（0 环）与 `tests/architecture/test_import_boundaries.py`。
   - 看什么：两条命令的退出码与关键输出；若本行新增 `vehicle → subsystems` 依赖，须先跑通再落地（`EPIC.md` 行 315）。
   - 落点：`raw/layering.md`。

## Constraints（冻结约束）

与 `EPIC.md` 「冻结约束」（行 223–233）中与本行相关的条目：

- **不改动任何 K/C 读数**：`tests/data/kc_baseline/` 与 `dynamic_hash_baseline.json` **逐字节不变是硬门**（`EPIC.md` 行 228 与 D5 行 64）；本行**不得重录任何基线**。F10（`EPIC.md` 行 132）判定两条基线不含 roll center 字段，且 `compute_vehicle_roll_centers` 在生产代码里**没有任何调用者**（唯一调用者是 `tests/physics/test_vehicle_physics.py`）——正因为这样，本行**必须主动证明**「本行改动不影响任何 K/C 读数」，而不是靠「反正没被调用」推断。
- **分层方向不可逆**（`EPIC.md` 行 226）：`modeling -> templates -> subsystems -> preparation/studies -> cases`。**`report/` 不得 import native/kernel/solver，也不得自求力律**（`legacy_surface_gate.py` 的 `report_native_import` / `report_constitutive_call` 规则）——这是本行 `validation_command` 第二段要跑该门的原因；若本行的引擎被 `report/` 侧消费，消费点必须留在 `vendor` 之外的合法层，且不得引入力律计算。
- **D3 口径**：先走 **Python 侧数值微分，不改 ABI**（`EPIC.md` 行 62）。内核 `mb_joint/functions.hpp:49-66` 的 `constraint_jacobian` 等**未过 ABI**（Python 绑定层无雅可比 API），把它暴露成 ABI 是**可选改进**并属 D1/D2 类的 ABI 变更，**本行不得做**（`EPIC.md` 行 225「本 Epic 最多两次 ABI 变更，只有 p2-02 与视 D2 的 p5-04」）。
- **不得引入新依赖**（D6，`EPIC.md` 行 65）：除 p5-05 的 FMI 库外不新增依赖；本行用现有 numpy / 几何设施。
- **不得新增 skip/xfail**（`EPIC.md` 行 229）；`tests/adams` 的环境 skip 是既有的，不得增长。
- **每步落地后必须重跑**（`EPIC.md` 行 230）：`just check-fast`；改结构后加跑 `tests/architecture`；触及求解路径加跑 `just gate-numeric`。本行新增引擎属结构改动，`check_module_layering.py --strict --final` 必须保持 0 环。
- **装配层不得出现按名字猜身份的规则**（`EPIC.md` 行 233）：任何新增跨子系统连接必须经形式化的 `Port` / `Connection` 契约——这是 G3 的判据来源。

## 风险与回退

- **四种构型今天能否构造需实测**（`EPIC.md` 行 314 与行 34）：本行依赖 p3-01 的 `raw/config_refusals.md`。回退/缓解：若某构型**连最小几何模型都无法声明**，本行的解析解验证**先落在可构造的构型**（单旋转副 + 双叉臂），并把该构型登记为「p3-03 前需先补最小模型声明」；**不得**把 G3 要求的四种构型从后续判据里静默删掉（`EPIC.md` 行 83）。开工前必须先读 p3-01 的四种构型实测结论，确认哪些构型可用于解析解验证。
- **数值微分的精度与步长选择**：约束雅可比由数值微分得到，步长过大会有截断误差、过小会有舍入噪声。回退/缓解：本行必须给出步长的**取值与理由**与误差量级实测（Goal 4），并以「单旋转副瞬轴 = 其轴线」这条**解析解断言**作为精度下限的证明；若双叉臂对照差异无法解释，**不得**用放宽容差掩盖，须先定位来源（`EPIC.md` 行 251(c) 要求差异有已解释来源）。
- **`vehicle → subsystems` 依赖加重**（`EPIC.md` 行 315）：新引擎若读装配运行时，会加重该方向的依赖。缓解：先跑 `tests/architecture/test_import_boundaries.py`（子进程逐入口检查）确认可落地，再写引擎；不通过则把引擎的输入改为**调用方注入的约束集合与点表**（而非引擎自己去取），把依赖留在调用侧。
- **与 p3-03 同文件串行**（`SUBTASKS.csv` 的 `p3-02` `notes`）：本行只改 `roll_centers.py` 的**调用段**，主体重构与硬点别名表删除归 p3-03；本行**不得**在 p3-03 之前删别名表（否则 p3-03 的「改造前可对照」基准丢失）。
- **既有失败**（`EPIC.md` 行 320）：起点值取自 p3-01 的 `raw/start_state.md`；任何新增失败阻断完成，不相关的既有失败独立列明。

## Done-When

- [ ] `raw/engine_contract.md` 非空，含引擎公开签名、输入（约束集合 + 点表）与输出（`omega_rel` / `v_rel` / 瞬轴）的形状与容差约定，附一次最小调用的实跑输出；该契约可被 p3-03 与 p3-04 **各自独立**对接（二者并行）。
- [ ] 存在断言：**单旋转副的瞬轴等于其轴线**（方向与轴上一点均带容差判定），pytest 通过。
- [ ] `raw/numerics_and_comparison.md` 非空，含数值微分步长与截断误差量级、双叉臂上与今日 `_instant_center` 的**并列对照表**与**逐条差异来源解释**（`EPIC.md` 行 251(c)）。
- [ ] `raw/no_name_sniffing.md` 非空，引擎模块路径上 `grep UPPER_|LOWER_|UCA_|_BODY_ALIASES` 退出码 1（零命中）。
- [ ] `raw/layering.md` 非空，`check_module_layering.py --strict --final` 退出码 0（0 环）且 `tests/architecture/test_import_boundaries.py` 通过。
- [ ] `packages/suspension_multibody/tests` 快速集通过（除 `adams/` 与 `cases/`），`legacy_surface_gate.py --check` 退出码 0。
- [ ] `kc_baseline` 与 `dynamic_hash_baseline` **逐字节未变**，未重录任何基线；无新增 skip/xfail。

## Final Validation Command

```bash
uv run --no-sync pytest packages/suspension_multibody/tests -q --ignore=packages/suspension_multibody/tests/adams --ignore=packages/suspension_multibody/tests/cases && uv run --no-sync python packages/suspension_multibody/tests/architecture/legacy_surface_gate.py --check
```
