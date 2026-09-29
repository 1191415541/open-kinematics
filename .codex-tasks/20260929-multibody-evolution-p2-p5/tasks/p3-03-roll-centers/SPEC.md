# SPEC：p3-03 通用滚转中心（摆脱硬点名称嗅探）

> 子任务规格。父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p3-03`。
> 形态：`single-full`。

## Goal

把 `SUBTASKS.csv` 中 `p3-03` 的 `acceptance_criteria` 拆成下面 5 条，逐条可判定：

1. **`compute_vehicle_roll_centers` 改为调用新引擎，且双叉臂结果与改造前可对照**：函数体改为走 p3-02 交付的微分运动学引擎（契约见 `tasks/p3-02-diffkinematics/raw/engine_contract.md`）；双叉臂构型的结果与改造前（今日 `vehicle/roll_centers.py:81-99 _instant_center` 路径）**并列对照**，每处差异给出已解释来源。判据可判定：对照表存在 + 差异逐条有解释 + 未解释差异为零；**不要求逐位一致**（`EPIC.md` 行 253(a)）。
2. **5 连杆、麦弗逊、扭梁三种构型各有断言**：三种构型**各自**有可判定的性质断言——有限值 + 对称性或单调性之类（例如左右对称构型的滚转中心 `y` 分量对称、随某几何参数单调变化）。`EPIC.md` 行 83 的 G3 判据要求「同一套算法对双叉臂、5 连杆、麦弗逊、扭梁四种构型各给出有限且可对照的滚转中心（四种构型各有断言）」——双叉臂由第 1 条承接，其余三种由本条承接。判据可判定：断言比较的是具体数值关系，不是「返回非空」。
3. **硬点别名表与 `_line_intersection` 二维交点路径删除**：`vehicle/roll_centers.py:59-67` 的硬点别名表（`UPPER_*` / `LOWER_*` / `UCA_*` 等）与 `:96 _line_intersection`（及依赖它的 `:81-99 _instant_center` 二维求交路径）**从文件中消失**；`grep` 在 `vehicle/` 目录无命中（即本行 `validation_command` 第二段：`! grep -rn -e UPPER_INBOARD -e LOWER_INBOARD -e UCA_ packages/suspension_multibody/src/suspension_multibody/vehicle`）。
4. **报表侧若暴露滚转中心，通道按安装角色命名**：若本行在报表侧新增/暴露滚转中心通道，通道名必须由**安装角色**（`placement`）动态生成，不得是 `front`/`rear` 之类硬编码字面量；若本行**不**暴露滚转中心通道，须在 `raw/channel_naming.md` 中记「未暴露」+ 该决定的依据（`EPIC.md` 行 220 的 `report/` 相关写范围约束与行 253(d)）。
5. **滚转中心高必须由侧倾反力虚功导数矩阵解算（路线图 2.6 节的指定口径）**：滚转中心高**不得只求几何连线交点**（含新引擎的瞬心连线求交），必须按路线图 `packages/suspension_multibody/docs/multibody_architecture_evolution.md:136-139`（`EPIC.md` 行 34 的阶段三配套要求、行 87 的 G3、行 257(e)）的指定口径——**由车轮横向力对车身产生的侧倾反力虚功导数矩阵**直接解算。三条子判据缺一不可：(i) 给出该矩阵的**构造方式**（广义坐标选取、虚位移摄动口径、矩阵元素含义与单位）；(ii) **与瞬心法的对照**——两者数值一致，或在已解释容差内一致，差异逐条给出来源；(iii) **一条独立数值判据**——给定车轮横向力增量 `ΔF_y` 时，「侧倾力矩导数 `∂M_x/∂φ` 与滚转中心高 `h` 的乘积关系」在数值上成立，容差取值与理由写明。判据可判定：矩阵构造方式有原文 + 对照表差异逐条有解释 + 独立数值判据有实测数值与断言。

## 写范围（允许改的路径）

照 `SUBTASKS.csv` 的 `p3-03` `notes`：**写范围 `vehicle/roll_centers.py`**。

- `packages/suspension_multibody/src/suspension_multibody/vehicle/roll_centers.py`（**本行独占**；p3-02 只改过它的调用段，本行做主体重构与硬点别名表 / `_line_intersection` 删除）
- `packages/suspension_multibody/tests/physics/test_vehicle_physics.py`（**本行独占**：它同时覆盖滚转中心与静平衡，见 `EPIC.md` 行 224）与 `packages/suspension_multibody/tests/vehicle/` 下本行自己的断言文件（与 p3-04 各写各的**新建**文件）
- 本目录 `raw/`（证据）、临时脚本或会话 scratch

`EPIC.md`「并行与写范围约束」（行 213–221）中与本行相关的条目：

- **p3-03 与 p3-04 生产写范围不相交，但测试写范围相交（`EPIC.md` 行 228）**：本行写 `vehicle/roll_centers.py`，p3-04 写 `vehicle/static_loads.py`，**生产侧不重叠**；但现有 `packages/suspension_multibody/tests/physics/test_vehicle_physics.py:5-10` **同时覆盖**滚转中心与静平衡——该文件**归本行**，p3-04 的静平衡断言写入它**新建**的 `tests/physics/test_static_loads.py`，`tests/vehicle/` 下同理各写各的新建文件。**两行硬串行**（`SUBTASKS.csv` 的 `p3-04.depends_on = p3-03`），**不得并行**——文件级切分不是并行的理由（第三轮复审核实：原「若确要并行」的逃逸口与硬依赖冲突，已废）。**共同前置 p3-02**（`p3-03.depends_on=p3-02`）的引擎契约已冻结，本行**对着该契约写**，不与 p3-04 协商接口。
- **p3-05 在 p3-04 之后**（`EPIC.md` 行 220）：**`outputs/builtin.py` 的派生输出声明归 p3-05**（行 221）。本行若在报表侧暴露滚转中心通道，只落在本行写范围内且不改 `outputs/builtin.py` 的声明段。
- **本行写范围内的 `report/` 改动须守分层方向**（`EPIC.md` 行 226）：`report/` 不得 import native/kernel/solver，也不得自求力律（`legacy_surface_gate.py` 的 `report_native_import` / `report_constitutive_call` 规则）。

## 禁止触碰

- `packages/suspension_multibody/src/suspension_multibody/vehicle/static_loads.py`（归 p3-04）与 `report/wheel_loads.py`、`report/metrics/vehicle.py`、`outputs/builtin.py`（归 p3-05）。
- `packages/suspension_multibody/tests/physics/test_static_loads.py` 与 p3-04 在 `tests/physics/`、`tests/vehicle/` 下**新建**的静平衡断言文件（归 p3-04）；`tests/physics/test_vehicle_physics.py` 相反是**本行独占**，p3-04 不得改（`EPIC.md` 行 224）。
- p3-02 交付的**引擎模块本体**——本行只调用它的公开接口，不修改引擎内部（引擎的精度/步长由 p3-02 负责）；若发现引擎缺口，登记在本行 `raw/` 并回退给 p3-02，不得就地改引擎。
- `packages/suspension_kernel/**`、`mb_config/version.hpp`、`kernel/native.py` 的 ABI 版本常量（ABI 单点提交归 p2-02，`EPIC.md` 行 225）。
- `subsystems/element_build.py`、`templates/roles.py`、`templates/builtin.py`（共享注册文件，`EPIC.md` 行 215 与行 216）。
- `packages/suspension_kinematics/**`（另一包，两解算器产品不得互相导入，F9 `EPIC.md` 行 130）。
- `EPIC.md`、`SUBTASKS.csv`、父 `PROGRESS.md`，以及 `tasks/` 下其它子任务目录——不得代写、不得预填。
- 任何冻结基线文件：`tests/data/kc_baseline/**`、`tests/data/**/sha256.json`、`dynamic_hash_baseline.json`、`kc_perf_baseline_native.json`（`EPIC.md` 行 228 与 D5 行 64）——**禁止重录**。
- `raw/` 中不得放未执行的内容（`EPIC.md` 行 355）。

## 依赖与时机

- 父行 `depends_on = p3-02`（`SUBTASKS.csv` 的 `p3-03`）。p3-02 未落地、引擎契约未冻结之前不得开工（`EPIC.md` 的阶段三主线 `p3-01 → p3-02 → p3-03 → p3-04 → p3-05 → p3-06`；p3-03 → p3-04 **硬串行**，不得并行）。
- **前置 S1**：本行的**实施部分**以阶段一 Epic `.codex-tasks/20260929-assembly-layer-rework/` 的 01–07 全部 `DONE` 为前置（`EPIC.md` 前置一节：**19 行实施行与终局验收行**受此约束；四个只读冻结行不受限）。阶段一未完成之前本行不得置 `IN_PROGRESS`。
- **与 p3-04 硬串行**（`EPIC.md` 行 228）：生产写范围不相交，但**测试写范围相交**（`tests/physics/test_vehicle_physics.py` 归本行、p3-04 只写它新建的 `tests/physics/test_static_loads.py`）。`SUBTASKS.csv` 的 `p3-04.depends_on = p3-03`，**不得并行**。**集成验证在合并之后由主代理跑**——两侧各自测过不等于合起来能跑。
- **与 p3-02 同一文件串行**：`SUBTASKS.csv` 的 `p3-02` `notes` 明确「`roll_centers` 主体重构归 p3-03，但同一文件，故 p3-03 依赖本行串行」。本行必须在 p3-02 之后改 `roll_centers.py`。
- **p3-06 依赖本行与 p3-05**（`SUBTASKS.csv` 的 `p3-06.depends_on=p3-03;p3-05`）。
- **禁止依据旧任务 DONE 结论**（`EPIC.md` 行 320 的口径）——所有事实以本行实跑为准。

## 判据与证据落点

逐条对应 `EPIC.md` 行 253 的 **(a)(b)(c)(d)(e)**；每条写清「跑什么命令、看什么输出、证据落到哪个文件」。

1. **(a) 改造前后双叉臂对照** → `raw/before_after_double_wishbone.md`
   - 跑什么：双叉臂构型下，改造前的今日实现（p3-01 的 `raw/anchors_roll_centers_static_loads.md` 给出的锚点）与改造后的 `compute_vehicle_roll_centers` 各跑一次，并列结果；`EPIC.md` F7（行 126）给出的 `:59-67`、`:81-99`、`:96`、`:42-47`、`:117-129` 为改造前锚点起点，按实测复核。
   - 看什么：**并列对照表** + 每处差异的逐条解释（`EPIC.md` 行 253(a)：差异须有解释）；未解释差异为零。
   - 落点：`raw/before_after_double_wishbone.md`。
2. **(b) 三种构型的性质断言** → 新增测试 + `raw/three_topologies.md`
   - 跑什么：对 5 连杆、麦弗逊、扭梁各构造最小几何模型（能力与拒绝点先看 p3-01 的 `raw/config_refusals.md`），各跑一次并断言可判定性质（有限值 + 对称性/单调性）。
   - 看什么：三种构型**逐种**的断言原文 + 实跑通过输出；若某构型需先补最小模型声明，须写明补了什么（`EPIC.md` 行 83 的 G3 要求四种构型各有断言）。
   - 落点：新增测试断言 + `raw/three_topologies.md`（pytest 输出原文与退出码）。
3. **(c) 硬点别名表与二维交点路径删除** → `raw/name_sniffing_removed.md`
   - 跑什么：本行 `validation_command` 的第二段（`! grep -rn -e UPPER_INBOARD -e LOWER_INBOARD -e UCA_ .../vehicle`），另加 `grep -rn -e _line_intersection -e _instant_center .../roll_centers.py` 证明路径已删。
   - 看什么：两条 grep 的退出码与输出原文；`_instant_center` / `_line_intersection` 若被新引擎的同名概念替代，须在不同模块或改名后出现，**不得留在 `vehicle/` 的 grep 命中里**。
   - 落点：`raw/name_sniffing_removed.md`。
4. **(d) 报表通道命名（若暴露）** → `raw/channel_naming.md`
   - 跑什么：检查本行改动是否在报表侧新增了滚转中心通道；若有，检查通道名的生成方式是否由安装角色（`placement`）驱动。
   - 看什么：通道名清单 + 生成代码位置；若是硬编码字面量则**不算达成**；若未暴露，记「未暴露」与该决定的依据。
   - 落点：`raw/channel_naming.md`。
5. **(e) 侧倾反力虚功导数判据** → `raw/roll_center_virtual_work.md`
   - 跑什么：按路线图 `docs/multibody_architecture_evolution.md:136-139`（`EPIC.md` 行 87 的 G3 与行 257(e)）构造「车轮横向力对车身产生的侧倾反力虚功导数矩阵」，实跑解出滚转中心高；同时跑瞬心法结果并列对照；再跑一条**独立数值判据**（给定横向力增量 `ΔF_y` 时，侧倾力矩导数与滚转中心高的乘积关系成立）。
   - 看什么：矩阵的构造方式原文（广义坐标、虚位移摄动口径、元素含义与单位）；与瞬心法的对照表 + 逐条差异解释（一致或在已解释容差内一致）；独立数值判据的实测数值 + 容差与理由 + 断言原文。
   - 落点：`raw/roll_center_virtual_work.md`。

## Constraints（冻结约束）

与 `EPIC.md` 「冻结约束」（行 223–233）中与本行相关的条目：

- **不改动任何 K/C 读数（本行口径）**：`tests/data/kc_baseline/` 与 `dynamic_hash_baseline.json` **逐字节不变是硬门**（`EPIC.md` 行 228 与 D5 行 64）；`SUBTASKS.csv` 的 `p3-03` `notes` 已明写「本行不改变任何 K/C 读数（`kc_baseline` 逐位不变）」。本行**不得重录任何基线**。F10（`EPIC.md` 行 132）判定两条基线不含 roll center 字段，且 `compute_vehicle_roll_centers` 在生产代码里**没有任何调用者**——但本行仍须**主动实跑**证明 K/C 读数逐位未变，而不是靠「未被调用」推断。
- **分层方向不可逆**（`EPIC.md` 行 226）：`modeling -> templates -> subsystems -> preparation/studies -> cases`；**`report/` 不得 import native/kernel/solver，也不得自求力律**——这是 `legacy_surface_gate.py` 的 `report_native_import` / `report_constitutive_call` 规则。本行若改 `report/` 侧的滚转中心暴露，必须保持这两条规则绿；`report/` 侧要用的量只能来自已算好的结果，不得在 `report/` 里重新求力律。
- **装配层不得出现按名字猜身份的规则**（`EPIC.md` 行 233）：任何新增跨子系统连接必须经形式化的 `Port` / `Connection` 契约——这也是 G3「零硬点名称嗅探」判据的来源；本行删除别名表即该约束在滚转中心链路上的落地。
- **与 p3-02 的接口不重复定义**：引擎的输入/输出形状与容差以 p3-02 的 `raw/engine_contract.md` 为准；本行**不重定义**它，只消费。
- **不得新增 skip/xfail**（`EPIC.md` 行 229）；`tests/adams` 的环境 skip 是既有的，不得增长。若某构型的断言无法成立，**登记原因**，不得用 skip 掩盖（`AGENTS.md` 第 7 节：失败要么修，要么按登记说明原因）。
- **每步落地后必须重跑**（`EPIC.md` 行 230）：`just check-fast`；改结构后加跑 `tests/architecture`；触及求解路径加跑 `just gate-numeric`。

## 风险与回退

- **四种构型今天能否构造需实测**（`EPIC.md` 行 314 与行 34）：本行依赖 p3-01 的 `raw/config_refusals.md` 与 p3-02 的解析解结论。回退/缓解：某构型若连最小几何模型都无法声明，本行须**先补最小模型声明**（写范围允许，落在本行测试侧）再断言；**不得**把该构型从四条判据里静默删掉（`EPIC.md` 行 83 的 G3 要求四种构型各有断言）。若补模型需要改**生产代码**而非测试数据，登记为范围外并回退给主代理裁决。
- **`vehicle → subsystems` 依赖方向的连锁**（`EPIC.md` 行 315）：本行删除 `subsystems.geometry`（`side_hardpoints`）的使用点会**减轻**该方向的依赖，但新引擎的输入面可能重新引入它。缓解：落地前后各跑一次 `tests/architecture/test_import_boundaries.py` 并对照；`check_module_layering.py --strict --final` 必须 0 环。
- **改造前基准丢失**：p3-02 已把 `roll_centers.py` 的调用段改写。缓解：本行做对照时，改造前的实现以 **p3-01 落盘的锚点原文 + 一个新引擎之外的可复现调用**为准（p3-02 **不得**在 p3-03 之前删别名表，见 `tasks/p3-02-diffkinematics/SPEC.md`）；若别名表已被提前删除导致无法复现改造前结果，登记为阻断项并回退给主代理。
- **既有失败**（`EPIC.md` 行 320）：起点值取自 p3-01 的 `raw/start_state.md`；任何新增失败阻断完成，不相关的既有失败独立列明。

## Done-When

- [ ] `raw/before_after_double_wishbone.md` 非空，含双叉臂改造前后的**并列对照表**与逐条差异解释；未解释差异为零（`EPIC.md` 行 253(a)）。
- [ ] `raw/roll_center_virtual_work.md` 非空：滚转中心高由**侧倾反力虚功导数矩阵**解算（矩阵构造方式、与瞬心法的对照、独立数值判据三者齐备），**不是**只求几何连线交点（路线图 `:136-139`、`EPIC.md` 行 87 与行 257(e)）。
- [ ] 5 连杆、麦弗逊、扭梁**三种构型各有断言**（有限值 + 对称性/单调性之类可判定性质），pytest 通过；断言原文与退出码落在 `raw/three_topologies.md`。
- [ ] `raw/name_sniffing_removed.md` 非空：`grep -e UPPER_INBOARD -e LOWER_INBOARD -e UCA_` 在 `.../vehicle` 退出码 1（零命中），且 `_instant_center` / `_line_intersection` 的旧路径已从 `roll_centers.py` 删除。
- [ ] `raw/channel_naming.md` 非空：报表侧要么按安装角色命名（有生成代码位置），要么记「未暴露」与依据（`EPIC.md` 行 253(d)）。
- [ ] `uv run --no-sync pytest packages/suspension_multibody/tests/physics packages/suspension_multibody/tests/vehicle -q` 通过。
- [ ] `kc_baseline` 与 `dynamic_hash_baseline` **逐字节未变**（实跑证明，不靠「未被调用」推断），未重录任何基线；无新增 skip/xfail。
- [ ] `check_module_layering.py --strict --final` 0 环、`test_import_boundaries.py` 通过、`legacy_surface_gate.py --check` 退出码 0。

## Final Validation Command

```bash
uv run --no-sync pytest packages/suspension_multibody/tests/physics packages/suspension_multibody/tests/vehicle -q && bash -c '! grep -rn -e UPPER_INBOARD -e LOWER_INBOARD -e UCA_ packages/suspension_multibody/src/suspension_multibody/vehicle'
```
