# 子任务 12：执行独立终局验收并逐条核对 G1-G9

## 目标

独立判定 EPIC `20260922-suspension-template-architecture` 的 Goal 是否达成。

1. **逐条核对 G1–G9**：每条结论指向独立证据（命令退出码或文件），不以子任务自证代替。
2. **执行端到端独立验收**（EPIC Done-When 规定，不依赖子任务完成度）：用**同一个模板**完成九件事，逐条按 EPIC Done-When 的 (a)–(i) 执行——
   - (a) 同一个轴的模板，K 模式跑一次、C 模式跑一次 → 只差激活列；
   - (b) 同一个轴模板 + 同一属性文件，准静态 study 跑一次、动态 study 跑一次；
   - (c) 换一份属性文件重跑 (b)，刚度/阻尼按属性变化而模板与几何不变；
   - (d) 总成 × 试验台矩阵任取两个组合跑通，其中一个此前不存在；并用最小单位输出 + 自定义表达式算出一个新指标；
   - (e) 整车实验总成（前后悬架+转向+车身+车轮+制动+驱动+整车 KC 试验台）跑通一次，并证明它与悬架实验总成共享同一套子系统定义；
   - (f) 轮胎质量归属：轴侧与整车侧各验证一次质量来自 tire 而非轮端 body；
   - (g) 无转向的悬架实验总成：同一模板组装一个**不含转向子系统**的单轴总成 + 悬架 KC 试验台，跑通一次；证明 rack 驱动轴与 rack 相关输出整体消失、K 网格降维为纯轮跳，且同一试验台在含转向总成上行为不变（不是报错绕过）；
   - (h) 制动/驱动的简化→复杂可替换性（需求 20 / D11）：用同一 role 下的**复杂模板**替换简化模板，装配层与试验台代码零改动即可跑通；且简化模板下整车总成的轮端力矩与 Adams 简单版逐参数对标；
   - (i) 可用性矩阵（需求 17 / D8）：悬架实验总成不含制动/驱动、整车实验总成含制动/驱动，两侧都有测试锁定。
3. **核对基线重录台账**：逐项确认每次重录都有登记（文件 + 步骤 + 前后值 + 等价性判定），且无"先改基线让门变绿"的痕迹。

## 非目标

- 不修代码、不改测试、不改文档；发现缺口退回对应子任务。
- 不把未实现的整车 Adams 对标标为通过。
- 不以"所有子任务 DONE"代替 Goal 判定。
- 不重录任何基线来让本步通过（本步是验收，不是修复）。

## 约束

- **本任务只读**生产代码与既有交付物。
- 终局命令集必须**逐条记录退出码与摘要**，缺项即未完成；不得只跑其中一条 pytest。
- 既有失败必须与 01 基线的 `baseline_values.md` 对照；`skip` 数的差异（如本机 Adams 工件就绪导致 skip 减少）必须定性为环境差异而非进展。
- 端到端九件事必须**实际执行**，不得以"子任务已测过"推断通过。
- 若发现某 Goal 与某项冻结约束冲突，**退回 EPIC 修订**，不得改基线或改验收标准使其通过。

## 范围与文件归属

- 可写：本任务目录下的 `PROGRESS.md`、`raw/**`（验收记录、命令输出、探针脚本）。
- 只读：仓库全部生产代码与测试、父 `EPIC.md`、`SUBTASKS.csv`、全部子任务目录。
- 不写：任何生产代码、测试、基线文件；父级计划文件（归主代理）。

## 依赖

- 前置：07 与 10（依赖列表中已声明）；实际前置为**全部 01-11 完成**。
- 后续：无。本任务为 EPIC 终局门禁。

## 验收标准

1. **终局命令全集逐条执行**（EPIC 验证协议的命令），每条有实测退出码与摘要，缺项为零。
2. **G1–G9 逐条结论**，每条附独立证据（命令退出码或文件路径），且明确区分"达成"/"未达成"/"登记保留"。
3. **端到端九件事 (a)–(i) 全部通过**，每件附命令与输出证据。这是 Goal 达成的终局判据，独立于子任务完成度；(g) 须同时给出「无转向跑通」与「含转向行为不变」两侧证据；(h) 须给出「装配层/试验台 diff 为空」的证据；(i) 须给出两侧锁定测试。
4. **既有失败独立列明**：与 01 基线对照；新增失败为零；未把未实现的整车 Adams 对标标为通过。
5. **未闭合项显式登记**（若有）：不判为达成，附解除条件。

## 验证协议

### 终局命令集（逐条记录退出码）

```text
uv run python packages/suspension_multibody/scripts/build_axle_native.py
uv run python packages/suspension_kernel/scripts/check_module_layering.py --strict --final
uv run --package suspension-kernel pytest packages/suspension_kernel/tests -q
uv run --package suspension-contracts pytest packages/suspension_contracts/tests -q
uv run --package suspension-multibody pytest packages/suspension_multibody/tests -q
uv run --all-packages ruff check .
uv run --all-packages ty check .
uv run python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check
uv run python packages/suspension_multibody/scripts/kc_parity_check.py --check
uv run python packages/suspension_multibody/scripts/case_parity_check.py
uv run python packages/suspension_multibody/tests/architecture/legacy_surface_gate.py --check
uv build --package suspension-kernel
uv build --package suspension-multibody
git diff --check
```

### 隔离 wheel 验证

在会话 scratch 新建 venv，安装三个本地 wheel（contracts / kernel / multibody），检查：

- import 与 CLI 可用；
- 七个导出符号与 01 冻结清单一致（`suspension_kernel_{run,capabilities,contract_version}` + `axle/vehicle/mb_core_abi_version` + `mb_core_run`），无新增导出；
- 版本常量 15/30/1/1（或 08 的 append-only 裁决所记录的值）；
- native 可执行，且端到端 artifact 往返 `status=success`。

### 端到端九件事（Goal 判据）

严格按 EPIC Done-When 的 (a)–(i) 执行，每件记录：命令、输出摘要、与预期的对照。(g) 的证据必须包含**同一试验台**在两种总成上的两次运行，且不含转向侧**没有** `StopIteration`/`KeyError` 等内部异常——不许把内部异常当"收缩生效"的证据。(h) 须用复杂桩模板实际替换并给出装配层与试验台的 diff 为空。

### 既有失败对照

与 01 的 `raw/baseline_values.md` 逐项对照：

- skip/xfail 数量与原因；
- Adams 相关 BLOCKED；
- 任何此前记录在案的既有失败。

差异必须定性（环境差异 / 新增失败），新增失败即阻断。

## 本步最容易犯的错（须自检）

1. **以 DONE 代替达成**：所有子任务 DONE 不等于 Goal 达成。端到端九件事 (a)–(i) 必须实际跑；(h) 与 (i) 是第三轮新增的判据，不得省略。
2. **把 skip 减少当成进展**：本机 Adams 工件就绪会让 46 项 skip 转为实跑，那是环境差异，不是本 Epic 的成果。
3. **漏核基线重录台账**：D4 授权重录，但"授权"不等于"免登记"。每个重录都必须在子任务 PROGRESS 中有前后值与判定依据。
4. **把重录当验收通过**：若某门是靠重录基线才变绿，必须额外提供等价性论证，否则该 Goal 不判达成。
