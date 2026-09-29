# SPEC：p3-05 报表通道按安装角色动态注册

> 子任务规格。父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p3-05`。
> 形态：`single-full`。

## Goal

把 `SUBTASKS.csv` 中 `p3-05` 的 `acceptance_criteria` 拆成下面 4 条，逐条可判定：

1. **重复硬编码字段改为按安装角色动态生成**：`report/wheel_loads.py:17 _WHEELS`、`:26-31` 的 `front_axle` / `rear_axle` / `front_rear_delta` / `left_side` / `right_side` / `right_left_delta`、`:36-38` 的「强制恰好四角」校验，与 `report/metrics/vehicle.py:21-43` 中**重复定义**的同一批字段（`normal_load_front_axle`、`load_transfer_front_minus_rear` 等）改为**按安装角色（`placement`）动态生成**，通道名形如 `normal_load_axle_{placement}`。锚点起点见 `EPIC.md` F8（行 128），按实测复核。
2. **4 轮情形既有字段名与值逐项一致（向后兼容硬门）**：4 轮（今天的 front/rear 两轴）下，既有字段名与数值**逐项一致**；新增的 `normal_load_axle_{placement}` 命名与之并存（不得只改名导致既有消费者失效）。判据可判定：`tests/metrics/test_outputs_match_legacy.py` 的对照通过 + 逐项对比表差异为空（`EPIC.md` 行 257(b) 的「向后兼容硬门」）。
3. **3 轴情形生成 `normal_load_axle_front/middle/rear`**：3 轴（front/middle/rear 放置）情形下生成三个按角色命名的通道；放置名来自总成声明的 `placement_role`，**不是**写死的 `front`/`middle`/`rear` 三元组（`EPIC.md` 行 257(c)）。
4. **`tests/metrics/test_outputs_match_legacy.py` 同步且理由登记**：该测试若因命名生成方式变化而需要更新，必须同步修改并**在 `PROGRESS.md` 中登记理由**（`EPIC.md` 行 257(d) 的口径：同步且理由登记）；断言不得减弱（`AGENTS.md` 第 7 节：不为了跑得更快删测试或把断言改弱）。

## 写范围（允许改的路径）

照 `SUBTASKS.csv` 的 `p3-05` `notes`：**写范围 `report/wheel_loads.py` 与 `report/metrics/vehicle.py` 与 `outputs/builtin.py` 的派生输出声明。`legacy_surface_gate` 的 `report_native_import` 与 `report_constitutive_call` 规则必须保持绿（`report` 不得调 native 也不得自求力律）**。

- `packages/suspension_multibody/src/suspension_multibody/report/wheel_loads.py`
- `packages/suspension_multibody/src/suspension_multibody/report/metrics/vehicle.py`
- `packages/suspension_multibody/src/suspension_multibody/outputs/builtin.py`（**仅派生输出声明段**，`EPIC.md` 行 221）
- 对应测试：`packages/suspension_multibody/tests/metrics/`、`packages/suspension_multibody/tests/outputs/`（**实测提醒**：`packages/suspension_multibody/tests/report/` 目录今天**不存在**，故验收命令用 `tests/metrics` 与 `tests/outputs`；若本行新建 `tests/report/`，则命令与本 SPEC 的 notes 同步更新）
- 本目录 `raw/`（证据）、临时脚本或会话 scratch

`EPIC.md`「并行与写范围约束」（行 213–221）中与本行相关的条目：

- **p3-05 在 p3-04 之后**（`EPIC.md` 行 220）：本行**消费 p3-04 的广义静平衡字段**（`SUBTASKS.csv` 的 `p3-05.depends_on=p3-04` 与 `notes`：「报表要消费广义静平衡字段」），字段形状以 p3-04 的 `raw/static_loads_contract.md` 为准，本行**不重定义**。
- **`outputs/builtin.py` 的派生输出声明归本行**（`EPIC.md` 行 221）：**p5-03 若需新增测点声明，必须与本行串行**（同一文件）——故本行必须在 `PROGRESS.md` 中把 `outputs/builtin.py` 的改动段落**精确登记**（改了哪一段的哪些声明），供 p5-03 对接。
- **`report/` 的分层约束**（`EPIC.md` 行 226）：`report/` 不得 import native/kernel/solver，也不得自求力律（`legacy_surface_gate.py` 的 `report_native_import` / `report_constitutive_call` 规则）——本行 `validation_command` 第二段就是要跑该门。

## 禁止触碰

- `packages/suspension_multibody/src/suspension_multibody/vehicle/**`（`roll_centers.py` 归 p3-02/p3-03、`static_loads.py` 归 p3-04、`service.py` 未授权）。
- p3-04 交付的**静平衡实现本体**——本行只消费它暴露的字段，不改 `static_loads.py`；若字段形状不足，登记给主代理裁决，不得就地改 p3-04 的写范围。
- `outputs/builtin.py` 中**非本行段**的声明（`EPIC.md` 行 221 指定的是「派生输出声明」段；其它段若需改动，登记范围外并回退裁决）。
- `packages/suspension_kernel/**`、`mb_config/version.hpp`、`kernel/native.py` 的 ABI 版本常量（ABI 单点提交归 p2-02，`EPIC.md` 行 225）。
- `subsystems/element_build.py`、`templates/roles.py`、`templates/builtin.py`（共享注册文件，`EPIC.md` 行 215 与行 216）。
- `packages/suspension_kinematics/**`（另一包，两解算器产品不得互相导入，F9 `EPIC.md` 行 130）。
- `EPIC.md`、`SUBTASKS.csv`、父 `PROGRESS.md`，以及 `tasks/` 下其它子任务目录——不得代写、不得预填。
- 任何冻结基线文件：`tests/data/kc_baseline/**`、`tests/data/**/sha256.json`、`dynamic_hash_baseline.json`、`kc_perf_baseline_native.json`（`EPIC.md` 行 228 与 D5 行 64）——**禁止重录**。报表字段名与值的兼容是**本行自证的硬门**，不得靠重录基线让门变绿。
- `raw/` 中不得放未执行的内容（`EPIC.md` 行 355）。

## 依赖与时机

- 父行 `depends_on = p3-04`（`SUBTASKS.csv` 的 `p3-05`）。p3-04 未落地、`raw/static_loads_contract.md` 未冻结之前不得开工。
- **前置 S1**：本行以阶段一 Epic `.codex-tasks/20260929-assembly-layer-rework/` 的 01–07 全部 `DONE` 为前置（`EPIC.md` 前置一节：**19 行实施行与终局验收行**受此约束；四个只读冻结行 p2-01/p3-01/p4-01/p5-01 不受限、可立即开工）；阶段一未完成之前本行不得置 `IN_PROGRESS`。
- **本行是阶段三的收口行之一**（`EPIC.md` 行 206）：`p3-06` 依赖 p3-03 与 p3-05（`SUBTASKS.csv` 的 `p3-06.depends_on=p3-03;p3-05`），故本行的交付物是 p3-06 终局验收的输入。
- **与 p5-03 串行同一文件**（`EPIC.md` 行 221）：`outputs/builtin.py` 的派生输出声明归本行；p5-03 若需新增测点声明必须在本行之后，且本行须把改动段落登记清楚。
- **禁止依据旧任务 DONE 结论**（`EPIC.md` 行 320 的口径）——所有事实以本行实跑为准。

## 判据与证据落点

逐条对应 `EPIC.md` 行 257 的 **(a)(b)(c)(d)**；每条写清「跑什么命令、看什么输出、证据落到哪个文件」。

1. **(a) 动态注册实现** → `raw/dynamic_channel_registration.md`
   - **验收命令口径**（`SUBTASKS.csv` 的 `p3-05` `notes`）：`uv run --no-sync pytest packages/suspension_multibody/tests/metrics packages/suspension_multibody/tests/outputs -q`——**实测 `packages/suspension_multibody/tests/report/` 目录今天不存在**，原命令必失败；若本行新建 `tests/report/`，则命令与 notes 同步更新。
   - 跑什么：改造后跑一次 4 轮与一次 3 轴，导出报表通道清单；对照改造前的字段定义位置（`report/wheel_loads.py:17/:26-31/:36-38`、`report/metrics/vehicle.py:21-43`，F8 `EPIC.md` 行 128，按实测复核）。
   - 看什么：通道名的生成代码位置（由 `placement` 驱动）+ 生成结果清单；两处**重复定义**是否已合并到单一来源（同一批字段今天在 `wheel_loads.py` 与 `metrics/vehicle.py` **重复定义**，`EPIC.md` 行 128 与行 257(a)）——若仍有两份，须说明真源归属。
   - 落点：`raw/dynamic_channel_registration.md`。
2. **(b) 4 轮向后兼容逐项一致** → `raw/four_wheel_compat.md`
   - 跑什么：4 轮情形下改造前后的**完整字段名与值**逐项对比；跑 `packages/suspension_multibody/tests/metrics/test_outputs_match_legacy.py`。
   - 看什么：逐项对比表（字段名 + 值，机器可判定输出如 `repr` 级比较）**差异为空**；既有消费者字段名一个都不少。
   - 落点：`raw/four_wheel_compat.md`（pytest 输出原文与退出码 + 对比表）。
3. **(c) 3 轴通道生成** → `raw/three_axle_channels.md`
   - 跑什么：3 轴（front/middle/rear 放置）跑一次并导出通道清单。
   - 看什么：`normal_load_axle_front` / `normal_load_axle_middle` / `normal_load_axle_rear` 三者均出现；放置名来自声明（做一次「非 front/rear 的放置名」的负例或改名实验，证明不是写死三元组——**这条证明必须有**，否则「动态」无法判定）。
   - 落点：`raw/three_axle_channels.md`。
4. **(d) 测试同步与理由登记** → `PROGRESS.md` 的登记节 + `raw/metrics_test_changes.md`
   - 跑什么：`packages/suspension_multibody/tests/metrics/` 与 `packages/suspension_multibody/tests/outputs/` 全跑（**不是** `tests/report/`——该目录今天不存在；若本行新建它，命令与 notes 同步更新）。
   - 看什么：`tests/metrics/test_outputs_match_legacy.py` 的改动 diff（若有）+ 理由登记；断言**未减弱**（对比改动前后的断言清单）。
   - 落点：`raw/metrics_test_changes.md` + `PROGRESS.md`。

## Constraints（冻结约束）

与 `EPIC.md` 「冻结约束」（行 223–233）中与本行相关的条目：

- **不改动任何 K/C 读数**：`tests/data/kc_baseline/` 与 `dynamic_hash_baseline.json` **逐字节不变是硬门**（`EPIC.md` 行 228 与 D5 行 64）；本行**不得重录任何基线**。本行改的是**报表字段的生成方式**，属「输出集合」层面的改动——若报表字段名集合发生变化，须按 D5 口径**逐项登记**「文件 + 步骤 + 前后值 + 独立于结果字节的物理等价判据」（`EPIC.md` 行 64 的 D5 要求）。4 轮情形的兼容硬门（`EPIC.md` 行 257(b)）意味着**既有字段名与值一个都不能少、不能变**。
- **分层方向不可逆**（`EPIC.md` 行 226）：`report/` **不得 import native/kernel/solver**（`legacy_surface_gate.py` 的 `report_native_import`），也**不得自求力律**（`report_constitutive_call`）。本行的 `validation_command` 第二段就是跑该门，必须退出码 0。报表要用的量只能来自**已算好的结果**，不得在 `report/` 里重新按轮胎本构求一次力。
- **`outputs/builtin.py` 归属**（`EPIC.md` 行 221）：本行只改**派生输出声明段**，且必须在 `PROGRESS.md` 中登记改动段落，因为 p5-03 要在此之后串行改同一文件。
- **不得引入新依赖**（D6，`EPIC.md` 行 65）：除 p5-05 的 FMI 库外不新增依赖。
- **不得新增 skip/xfail**（`EPIC.md` 行 229）；`tests/adams` 的环境 skip 是既有的，不得增长。
- **每步落地后必须重跑**（`EPIC.md` 行 230）：`just check-fast`；改结构后加跑 `tests/architecture`。

## 风险与回退

- **4 轮情形向后兼容逐位一致（本行硬门）**：`report/wheel_loads.py` 与 `report/metrics/vehicle.py` 的字段今天是**重复定义**的硬编码集合（F8 `EPIC.md` 行 128），下游消费者（含 `tests/metrics/test_outputs_match_legacy.py` 固化的既有契约）按旧名字取值。回退/缓解：改造**前**先把 4 轮的完整字段名与值落盘（`raw/four_wheel_compat.md` 前半）；改造后逐项比较，**差异非空即回退该步**；`normal_load_axle_{placement}` 是**新增**命名，必须与既有名并存（不得只改名）。
- **「动态」可能退化为换一种硬编码**（`EPIC.md` 行 257(a) 与行 233 的精神）：把三处 `front/rear` 字面量换成 `front/middle/rear` 三元组，仍是硬编码。缓解：`raw/three_axle_channels.md` 的**改名实验**（用非 `front`/`middle`/`rear` 的放置名证明生成逻辑由声明驱动）是这条的判据；缺该证据即视为未达成。
- **分层门在第一段就被拒**：`report/` 若为了拿放置信息而 import 装配/内核侧，会命中 `report_native_import`。缓解：放置信息必须由**已算好的结果文档**带进来（字段级），不在 `report/` 内回查装配运行；每步跑 `legacy_surface_gate.py --check`。
- **`outputs/builtin.py` 与 p5-03 的写冲突**（`EPIC.md` 行 221）：同一文件必须串行。缓解：本行把改动段落**精确登记**在 `PROGRESS.md`（段落名 + 改动内容），p5-03 只在其后追加。
- **既有失败**（`EPIC.md` 行 320）：起点值取自 p3-01 的 `raw/start_state.md`；任何新增失败阻断完成，不相关的既有失败独立列明。

## Done-When

- [ ] `raw/dynamic_channel_registration.md` 非空：给出由 `placement` 驱动生成通道名的**代码位置**与生成结果清单；两处重复定义的真源归属已说明。
- [ ] `raw/four_wheel_compat.md` 非空：4 轮改造前后的字段名与值**逐项一致**（机器可判定输出，差异为空）；`tests/metrics/test_outputs_match_legacy.py` 通过。
- [ ] `raw/three_axle_channels.md` 非空：3 轴情形生成 `normal_load_axle_front` / `normal_load_axle_middle` / `normal_load_axle_rear`，且**改名实验**证明生成逻辑由声明驱动（不是写死三元组）。
- [ ] `raw/metrics_test_changes.md` + `PROGRESS.md` 登记非空：测试同步的 diff 与理由；断言未减弱（改动前后断言清单对照）。
- [ ] `uv run --no-sync pytest packages/suspension_multibody/tests/metrics packages/suspension_multibody/tests/outputs -q` 通过（**不是** `tests/report`——该目录今天不存在；若本行新建它，命令与 notes 同步更新）。
- [ ] `legacy_surface_gate.py --check` 退出码 0：`report_native_import` 与 `report_constitutive_call` 两条规则未破（`report/` 不调 native、不自求力律）。
- [ ] `kc_baseline` 与 `dynamic_hash_baseline` **逐字节未变**；报表字段名集合若有变化，逐项按 D5 登记；未重录任何基线；无新增 skip/xfail。

## Final Validation Command

```bash
uv run --no-sync pytest packages/suspension_multibody/tests/metrics packages/suspension_multibody/tests/outputs -q && uv run --no-sync python packages/suspension_multibody/tests/architecture/legacy_surface_gate.py --check
```
