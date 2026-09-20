# Epic 进度：统一 preparation 生命周期与整车旧模块删除

- **任务**：统一多体 preparation 入口并删除整车旧模块
- **形态**：epic
- **状态**：DONE
- **进度**：5/5 子任务完成
- **当前**：01-05 全部 DONE；05 预收口首轮 exit1 已保留失败记录并收敛证据字段，r2 `--final-preclose` exit0，状态同步后 `--final` 独立执行 exit0，无未关闭阻断项
- **文件**：`.codex-tasks/20260920-unified-preparation-cutover/`

## 恢复入口

恢复时依次读取：

1. `EPIC.md`
2. `SUBTASKS.csv`
3. 本文件
4. 当前子任务目录下的 `SPEC.md`、`TODO.csv`、`PROGRESS.md`

子任务必须按 `01 → 02 → 03 → 04 → 05` 串行推进；源码实现前规划审核必须为 `PASS`。

## 当前阻断项

- 第三轮独立审核的四个阻断项已修订：`SUBTASKS.csv` 第 4 行改为 tasks/04 SPEC 删除后完整检查并加 `--progress-records 04-pre-delete`/`04-post-delete` 记录检查；任务 03 明确拥有 `results/decoder.py` 接线与旧导入清理；任务 01 改为可注入替身验证延迟加载七 key、adapter 协议/包装/优先级/匹配/stale 并保持独立可过；`PREPARATION_MATRIX.md` 固定 `_vehicle_axle_result` 私有名。同轮两条一致性问题也已修正：矩阵把 `prepared_simulation` 复用与 document bypass 分开；`SUBTASKS.csv` 第 3 行验证加入 artifact 回归和 `architecture_contract_scan.py`。
- 第四轮修订（本轮）：`planning_contract_scan.py` 改为逐条原子命令核对成功记录、阶段标签统一并核对成功扫描记录的真实证据文件、`--final` 改为事后全量核对、新增 `--final-preclose` 预收口；`SUBTASKS.csv` 第 4 行与 tasks/04 TODO 第 4/5 行的命令链移除 `--progress-records` 审计（避免“要求执行后才记录”的循环）；tasks/05 TODO 第 5 行末步改为 `--final-preclose && git diff --check`；新增 `tasks/05-final-validation/test_planning_contract_scan.py` 门禁回归测试。
- 定向独立复审（06ae7cf1-71bf-4255-beff-c6d12c9981bb）确认四项阻断关闭，规划 PASS。非阻断注意项：记录审计豁免子串匹配过宽；终局须实际运行门禁回归测试；删除前后须分别重跑完整验证并保留各阶段独立证据；实现记录不得填“证据文件：无”。按现有规格执行，不以脚本薄弱点替代真实验证。

## 子任务状态

| id | task | status | latest evidence |
|---|---|---|---|
| 1 | 统一 preparation 协议与 runner 生命周期 | DONE | 独立审查收敛；simulation 53 + architecture 44 passed；ruff/ty通过 |
| 2 | 迁移六个非 vehicle_dynamic family 的装配前处理 | DONE | 独立审查后修复axle文档校验；117受影响测试通过 |
| 3 | 拆分 vehicle_dynamics 职责并接入 vehicle_dynamic preparation | DONE | 两轮独立复核PASS；影响范围回归260 passed/1 xfailed；父证据审计退出码0；旧模块保留待04删除 |
| 4 | 迁移调用方并删除旧模块 | DONE | 调用方迁移与历史读取兼容已验收；pre/post-delete 七条门禁逐条 exit0；post-delete 功能 364 passed/1 skipped/1 xfailed（533.42s）；阶段记录审计 exit0；旧模块已删除 |
| 5 | 终局回归与 Epic 收口 | DONE | 预收口 r2 与 `--final` 均 exit0；专项全量 714 passed/1 skipped/1 xfailed；六项 Goal 核验与 5ef24a10 复审无阻断 |

## 验证记录格式

每次执行子任务或终局门禁后，在本文件追加一条 `### 验证记录`，必须使用以下字段：

### 验证记录：<子任务或门禁>
- **时间**：执行时间；
- **子任务/门禁**：对应 `SUBTASKS.csv` 行或删除前/删除后阶段；
- **命令**：完整命令，不用“同上”代替；
- **退出码**：实际退出码；
- **摘要**：通过数量、关键断言或失败首因；
- **证据文件**：指向原始输出文件，可为工作区相对路径或绝对路径（含 `$PI_SCRATCH_DIR` 下的临时日志）；无独立文件时写明 `无`。

## 修订验证

- `SUBTASKS.csv` CSV 结构校验：退出码 0，6 行且每行 11 列。
- 规划目录 Markdown 代码块校验：退出码 0，13 个 Markdown 文件无未闭合代码块。
- `legacy_reference_scan.py` 语法校验：退出码 0。

只有实际执行并记录验证结果后，才能更新对应子任务状态；所有子任务为 `DONE` 也不能替代终局验收记录。

## 下一步

1. 修订规划文件并确保扫描、职责映射、验收命令和证据位置一致。
2. 重新执行独立规划审核。
3. 审核通过后执行子任务 01，并在本文件同步状态和验证摘要。
## 最新修订验证

- `python .codex-tasks/20260920-unified-preparation-cutover/tasks/04-delete-legacy/architecture_contract_scan.py`：退出码 0；`decode_result=results/decoder.py`，native submission 唯一位于 `simulation/backend.py:24`。
- `python -m py_compile .codex-tasks/20260920-unified-preparation-cutover/tasks/04-delete-legacy/legacy_reference_scan.py .codex-tasks/20260920-unified-preparation-cutover/tasks/04-delete-legacy/architecture_contract_scan.py`：退出码 0。
- `python -c "import csv; from pathlib import Path; root=Path('.codex-tasks/20260920-unified-preparation-cutover'); files=sorted(root.rglob('*.csv')); report=[]; bad=[]; [report.append((p.as_posix(),len(list(csv.reader(p.open(encoding='utf-8',newline='')))),sorted({len(r) for r in csv.reader(p.open(encoding='utf-8',newline=''))}))) for p in files]; [bad.append(item) for item in report if set(item[2]) not in ({8},{11})]; assert not bad,bad; print('csv_files=%d'%len(files)); [print(item) for item in report]"`：退出码 0；6 个 CSV 文件均为固定列数，`SUBTASKS.csv` 为 11 列，5 个子任务 `TODO.csv` 为 8 列。
- `python -c "from pathlib import Path; root=Path('.codex-tasks/20260920-unified-preparation-cutover'); files=sorted(root.rglob('*.md')); bad=[(p.as_posix(),sum(1 for line in p.read_text(encoding='utf-8').splitlines() if line.strip().startswith(chr(96)*3))) for p in files if sum(1 for line in p.read_text(encoding='utf-8').splitlines() if line.strip().startswith(chr(96)*3))%2]; assert not bad,bad; print('markdown_files=%d unclosed=0'%len(files))"`：退出码 0；13 个规划 Markdown 文件无未闭合代码块。

上述检查只验证规划和门禁工具，未改变任何子任务状态；当前仍等待独立规划三审。
## 二审修订后的验证

- `python -c "import csv; from pathlib import Path; root=Path('.codex-tasks/20260920-unified-preparation-cutover'); files=sorted(root.rglob('*.csv')); bad=[]; [bad.append((p.as_posix(),sorted({len(row) for row in csv.reader(p.open(encoding='utf-8',newline=''))}))) for p in files if {len(row) for row in csv.reader(p.open(encoding='utf-8',newline=''))} not in ({8},{11})]; assert not bad,bad; print('csv_files=%d valid=1'%len(files))"`：退出码 0；6 个 CSV 文件列数分别为 `SUBTASKS.csv` 11 列、5 个子任务 `TODO.csv` 8 列。
- `python -c "from pathlib import Path; root=Path('.codex-tasks/20260920-unified-preparation-cutover'); paths=[p for p in root.rglob('*') if p.is_file() and p.name!='PROGRESS.md' and p.suffix in {'.md','.csv'}]; text='\\n'.join(p.read_text(encoding='utf-8') for p in paths); bad=('PRE_'+'DELETE'+':','POST_'+'DELETE'+':','DELETE'+': remove'); assert all(pattern not in text for pattern in bad); csv_text='\\n'.join(p.read_text(encoding='utf-8') for p in root.rglob('*.csv')); assert ('run_'+'contract(') not in csv_text; print('planning_bad_patterns=0')"`：退出码 0；排除当前验证日志自身后，规划真源无非法删除阶段标签或文本 native 调用断言。
- `python -m py_compile .codex-tasks/20260920-unified-preparation-cutover/tasks/04-delete-legacy/legacy_reference_scan.py .codex-tasks/20260920-unified-preparation-cutover/tasks/04-delete-legacy/architecture_contract_scan.py && python .codex-tasks/20260920-unified-preparation-cutover/tasks/04-delete-legacy/architecture_contract_scan.py`：退出码 0；唯一 `decode_result` 和唯一 native submission 断言通过。
### 验证记录：规划门禁 01-05 / CSV 结构
- **时间**：2026-09-20
- **子任务/门禁**：规划门禁 01-05 / CSV 结构
- **命令**：`python -c "import csv; from pathlib import Path; root=Path('.codex-tasks/20260920-unified-preparation-cutover'); files=sorted(root.rglob('*.csv')); bad=[]; [bad.append((p.as_posix(),sorted({len(row) for row in csv.reader(p.open(encoding='utf-8',newline=''))}))) for p in files if {len(row) for row in csv.reader(p.open(encoding='utf-8',newline=''))} not in ({8},{11})]; assert not bad,bad; print('csv_files=%d valid=1'%len(files))"`
- **退出码**：0
- **摘要**：6 个 CSV 文件列数有效；父级 11 列，5 个子任务 8 列。
- **证据文件**：无

### 验证记录：规划门禁 01-05 / Python 语法
- **时间**：2026-09-20
- **子任务/门禁**：规划门禁 01-05 / Python 语法
- **命令**：`python -m py_compile .codex-tasks/20260920-unified-preparation-cutover/tasks/04-delete-legacy/legacy_reference_scan.py .codex-tasks/20260920-unified-preparation-cutover/tasks/04-delete-legacy/architecture_contract_scan.py .codex-tasks/20260920-unified-preparation-cutover/tasks/05-final-validation/planning_contract_scan.py`
- **退出码**：0
- **摘要**：三个规划/门禁脚本编译通过。
- **证据文件**：无

### 验证记录：规划门禁 01-05 / 规划结构
- **时间**：2026-09-20
- **子任务/门禁**：规划门禁 01-05 / 规划结构
- **命令**：`python .codex-tasks/20260920-unified-preparation-cutover/tasks/05-final-validation/planning_contract_scan.py --planning`
- **退出码**：0
- **摘要**：五个子任务、七个矩阵 key、TODO 状态和矩阵列结构通过。
- **证据文件**：无

### 验证记录：规划门禁 01-05 / 非整车矩阵
- **时间**：2026-09-20
- **子任务/门禁**：规划门禁 01-05 / 非整车矩阵
- **命令**：`python .codex-tasks/20260920-unified-preparation-cutover/tasks/05-final-validation/planning_contract_scan.py --non-vehicle-matrix`
- **退出码**：0
- **摘要**：六个非 vehicle_dynamic family 的矩阵行和字段通过。
- **证据文件**：无

### 验证记录：规划门禁 01-05 / PROGRESS 模板
- **时间**：2026-09-20
- **子任务/门禁**：规划门禁 01-05 / PROGRESS 模板
- **命令**：`python .codex-tasks/20260920-unified-preparation-cutover/tasks/05-final-validation/planning_contract_scan.py --progress-template`
- **退出码**：0
- **摘要**：验证记录模板包含时间、任务、命令、退出码、摘要和证据文件字段。
- **证据文件**：无

### 验证记录：规划门禁 01-05 / 架构扫描
- **时间**：2026-09-20
- **子任务/门禁**：规划门禁 01-05 / 架构扫描
- **命令**：`python .codex-tasks/20260920-unified-preparation-cutover/tasks/04-delete-legacy/architecture_contract_scan.py`
- **退出码**：0
- **摘要**：唯一统一 decode_result 位于 results/decoder.py；唯一 native submission 位于 simulation/backend.py:24。
- **证据文件**：无

### 验证记录：规划门禁 01-05 / Markdown 与坏模式
- **时间**：2026-09-20
- **子任务/门禁**：规划门禁 01-05 / Markdown 与坏模式
- **命令**：`python -c "from pathlib import Path; root=Path('.codex-tasks/20260920-unified-preparation-cutover'); files=sorted(root.rglob('*.md')); bad=[(p.as_posix(),sum(1 for line in p.read_text(encoding='utf-8').splitlines() if line.strip().startswith(chr(96)*3))) for p in files if sum(1 for line in p.read_text(encoding='utf-8').splitlines() if line.strip().startswith(chr(96)*3))%2]; assert not bad,bad; print('markdown_files=%d unclosed=0'%len(files))"`；`python -c "from pathlib import Path; root=Path('.codex-tasks/20260920-unified-preparation-cutover'); paths=[p for p in root.rglob('*') if p.is_file() and p.name!='PROGRESS.md' and p.suffix in {'.md','.csv'}]; text='\\n'.join(p.read_text(encoding='utf-8') for p in paths); bad=('PRE_'+'DELETE'+':','POST_'+'DELETE'+':','DELETE'+': remove'); assert all(pattern not in text for pattern in bad); csv_text='\\n'.join(p.read_text(encoding='utf-8') for p in root.rglob('*.csv')); assert ('run_'+'contract(') not in csv_text; print('planning_bad_patterns=0')"`
- **退出码**：0
- **摘要**：13 个 Markdown 无未闭合代码块；规划真源无非法删除阶段标签或文本 native 调用断言。
- **证据文件**：无
### 验证记录：规划门禁 01-05 / 记录格式
- **时间**：2026-09-20
- **子任务/门禁**：规划门禁 01-05 / 记录格式
- **命令**：`python .codex-tasks/20260920-unified-preparation-cutover/tasks/05-final-validation/planning_contract_scan.py --progress-records 01`
- **退出码**：0
- **摘要**：所有已记录验证块的时间、任务、完整命令、数值退出码、摘要和证据字段均非空。
- **证据文件**：无
> 说明：该 `--progress-records 01` 记录是旧版本格式检查，只验证记录字段格式，不能作为当前实现或当前门禁语义的执行证据；其退出码按历史原样保留，不追改。

### 验证记录：规划门禁 01-05 / Diff 检查
- **时间**：2026-09-20
- **子任务/门禁**：规划门禁 01-05 / Diff 检查
- **命令**：`git diff --check`
- **退出码**：0
- **摘要**：无 diff whitespace 错误；仅报告工作区既有 LF/CRLF 转换警告。
- **证据文件**：无

### 验证记录：第三轮修订 / 规划结构
- **时间**：2026-09-20
- **子任务/门禁**：规划修订 03 / 规划结构
- **命令**：`python .codex-tasks/20260920-unified-preparation-cutover/tasks/05-final-validation/planning_contract_scan.py --planning`
- **退出码**：0
- **摘要**：5 个子任务保持 TODO、7 个矩阵 key 与矩阵列结构通过；矩阵已把 `prepared_simulation` 复用与 document bypass 分开，`_vehicle_axle_result` 固定私有名，阶段验证策略已写入。
- **证据文件**：无

### 验证记录：第三轮修订 / 矩阵检查
- **时间**：2026-09-20
- **子任务/门禁**：规划修订 03 / 矩阵检查
- **命令**：`python .codex-tasks/20260920-unified-preparation-cutover/tasks/05-final-validation/planning_contract_scan.py --matrix` 和 `python .codex-tasks/20260920-unified-preparation-cutover/tasks/05-final-validation/planning_contract_scan.py --non-vehicle-matrix`
- **退出码**：0
- **摘要**：七个 registry key 行、六个非整车 family 行及 context/consumer/owner 列检查通过。
- **证据文件**：无

### 验证记录：第三轮修订 / 删除阶段记录门禁对照
- **时间**：2026-09-20
- **子任务/门禁**：规划修订 03 / 删除阶段记录门禁
- **命令**：`python "$PI_SCRATCH_DIR/epic_sim2/tasks/05-final-validation/planning_contract_scan.py" --progress-records 04-pre-delete` 与 `--progress-records 04-post-delete`（在规划目录副本上分别注入有效记录、退出码 1 记录、缺少 `--pre-delete` 标记记录）
- **退出码**：0
- **摘要**：有效 `04-pre-delete`/`04-post-delete` 记录均通过（exit 0）；无记录、记录退出码为 1、命令缺少 `legacy_reference_scan.py --pre-delete` 标记三种反向用例均 exit 1，正向与反向对照成立。
- **证据文件**：无

### 验证记录：第三轮修订 / 真实仓库阶段记录缺失（预期失败）
- **时间**：2026-09-20
- **子任务/门禁**：规划修订 03 / 04-pre-delete 记录缺失
- **命令**：`python .codex-tasks/20260920-unified-preparation-cutover/tasks/05-final-validation/planning_contract_scan.py --progress-records 04-pre-delete`
- **退出码**：1
- **摘要**：删除动作尚未执行，父级 PROGRESS 尚无 `子任务 04-pre-delete` 记录，断言报 `missing progress record for 04-pre-delete`；这是预期失败，记录由子任务 04 删除前写入。
- **证据文件**：无

### 验证记录：第三轮修订 / CSV 结构
- **时间**：2026-09-20
- **子任务/门禁**：规划修订 03 / CSV 结构
- **命令**：`python -c "import csv; from pathlib import Path; root=Path('.codex-tasks/20260920-unified-preparation-cutover'); bad=[]; [bad.append((p.as_posix(),sorted({len(r) for r in csv.reader(p.read_text(encoding='utf-8').splitlines(True))}))) for p in sorted(root.rglob('*.csv')) if sorted({len(r) for r in csv.reader(p.read_text(encoding='utf-8').splitlines(True))}) not in ([8],[11])]; assert not bad,bad; print('csv_files=6 valid=1')"`
- **退出码**：0
- **摘要**：6 个 CSV 均有效；`SUBTASKS.csv` 11 列，5 个子任务 `TODO.csv` 8 列。
- **证据文件**：无

### 验证记录：第三轮修订 / 门禁脚本语法
- **时间**：2026-09-20
- **子任务/门禁**：规划修订 03 / 门禁脚本语法
- **命令**：`python -m py_compile .codex-tasks/20260920-unified-preparation-cutover/tasks/05-final-validation/planning_contract_scan.py .codex-tasks/20260920-unified-preparation-cutover/tasks/04-delete-legacy/legacy_reference_scan.py .codex-tasks/20260920-unified-preparation-cutover/tasks/04-delete-legacy/architecture_contract_scan.py`
- **退出码**：0
- **摘要**：三个规划/门禁脚本编译通过，`planning_contract_scan.py` 新增 `04-pre-delete`/`04-post-delete` 记录检查分支。
- **证据文件**：无

### 验证记录：第三轮修订 / Diff 检查
- **时间**：2026-09-20
- **子任务/门禁**：规划修订 03 / Diff 检查
- **命令**：`git diff --check`
- **退出码**：0
- **摘要**：无 diff whitespace 错误；仅报告工作区既有 LF/CRLF 转换警告。
- **证据文件**：无

### 验证记录：第四轮修订 / 门禁回归测试
- **时间**：2026-09-20
- **子任务/门禁**：规划修订 04 / 门禁回归测试
- **命令**：`python .codex-tasks/20260920-unified-preparation-cutover/tasks/05-final-validation/test_planning_contract_scan.py`
- **退出码**：0
- **摘要**：Ran 23 tests OK。覆盖引号内 `&&` 不拆分、逐条成功记录通过、缺记录/退出码 1/缺证据/仅整链记录拒绝、记录审计命令无需证据、绝对与相对证据路径、阶段标签可识别且 pre/post 不混用、`--final-preclose` 无需本次自证、`--final` 拒绝功能证据缺失与未完成子任务。
- **证据文件**：无

### 验证记录：第四轮修订 / 规划结构
- **时间**：2026-09-20
- **子任务/门禁**：规划修订 04 / 规划结构
- **命令**：`python .codex-tasks/20260920-unified-preparation-cutover/tasks/05-final-validation/planning_contract_scan.py --planning`
- **退出码**：0
- **摘要**：5 个子任务保持 TODO、7 个矩阵 key 与矩阵列结构通过；修订后 CSV 结构仍为 11 列/8 列。
- **证据文件**：无

### 验证记录：第四轮修订 / 记录格式
- **时间**：2026-09-20
- **子任务/门禁**：规划修订 04 / 记录格式
- **命令**：`python .codex-tasks/20260920-unified-preparation-cutover/tasks/05-final-validation/planning_contract_scan.py --progress-records planning`
- **退出码**：0
- **摘要**：所有已记录验证块的时间、任务、完整命令、数值退出码、摘要和证据字段均非空；旧版 `--progress-records 01` 记录保留原退出码并已标注不能作为当前实现证据。
- **证据文件**：无

### 验证记录：第四轮修订 / 门禁脚本语法
- **时间**：2026-09-20
- **子任务/门禁**：规划修订 04 / 门禁脚本语法
- **命令**：`python -m py_compile .codex-tasks/20260920-unified-preparation-cutover/tasks/05-final-validation/planning_contract_scan.py .codex-tasks/20260920-unified-preparation-cutover/tasks/05-final-validation/test_planning_contract_scan.py .codex-tasks/20260920-unified-preparation-cutover/tasks/04-delete-legacy/legacy_reference_scan.py .codex-tasks/20260920-unified-preparation-cutover/tasks/04-delete-legacy/architecture_contract_scan.py`
- **退出码**：0
- **摘要**：门禁脚本与新增回归测试脚本均编译通过。
- **证据文件**：无

### 验证记录：第四轮修订 / 删除前阶段记录缺失（预期失败）
- **时间**：2026-09-20
- **子任务/门禁**：规划修订 04 / 04-pre-delete 记录缺失
- **命令**：`python .codex-tasks/20260920-unified-preparation-cutover/tasks/05-final-validation/planning_contract_scan.py --progress-records 04-pre-delete`
- **退出码**：1
- **摘要**：删除动作尚未执行，父级 PROGRESS 尚无 `子任务 04-pre-delete` 记录，断言报 `missing progress record for 04-pre-delete`；这是预期失败，记录由子任务 04 删除前逐条写入。
- **证据文件**：无

### 验证记录：第四轮修订 / Diff 检查
- **时间**：2026-09-20
- **子任务/门禁**：规划修订 04 / Diff 检查
- **命令**：`git diff --check`
- **退出码**：0
- **摘要**：无 diff whitespace 错误；仅报告工作区既有 LF/CRLF 转换警告。
- **证据文件**：无

## 本轮恢复与审核结论

- 恢复时工作区干净，五个源码子任务均 TODO；当前已完成规划冲突修复及独立复审，开始任务 01。
- 主线程复跑门禁回归：23 tests OK；planning 结构检查和 git diff --check 均退出码 0。
- test-runner 基线：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests/simulation packages/suspension_multibody/tests/architecture -q`，退出码 0，63 passed in 8.14s；输出位于会话 scratch 的 baseline.log。
- 任务 01 fixer：19e44750-671b-4421-86b6-5238f8420e66；只写中央运行时/协议测试及子任务 01 真源。完成后主线程审查、集成验证、父级证据核对，再推进 02。

## 本会话接管

- 用户确认由本会话接管现有改动；原 fixer/审查句柄已失效，不据此声明完成。
- 独立规划审核 7ae302d5-0ee4-4933-9a11-9621c875853b 返回 PASS；运行时代码仍需独立审查及修复后验收。

### 验证记录：子任务 01 / 矩阵
- **时间**：2026-09-20
- **子任务/门禁**：子任务 01
- **命令**：`python .codex-tasks/20260920-unified-preparation-cutover/tasks/05-final-validation/planning_contract_scan.py --matrix`
- **退出码**：0
- **摘要**：七个 family 的矩阵结构及映射检查通过。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/01-matrix.log

### 验证记录：子任务 01 / 默认注册表
- **时间**：2026-09-20
- **子任务/门禁**：子任务 01
- **命令**：`uv run --package suspension-multibody python -c "from suspension_multibody.simulation.preparation import default_preparation_registry; required={('axle','axle_dynamic'),('axle','kc_quasi_static'),('vehicle','vehicle_kc'),('vehicle','handling'),('vehicle','ride_four_post'),('vehicle','ride_random_road'),('vehicle','vehicle_dynamic')}; keys=set(default_preparation_registry().keys()); assert keys==required, keys^required"`
- **退出码**：0
- **摘要**：默认延迟注册表恰含七个规范化 key。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/01-registry.log

### 验证记录：子任务 01 / 接管阶段测试
- **时间**：2026-09-20
- **子任务/门禁**：子任务 01
- **命令**：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests/simulation packages/suspension_multibody/tests/architecture -q`
- **退出码**：0
- **摘要**：test-runner 实际执行，97 passed in 5.90s；尚待独立代码审查收敛。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/pytest_sim_arch.log

### 验证记录：子任务 01 / scoped ruff
- **时间**：2026-09-20
- **子任务/门禁**：子任务 01
- **命令**：`uv run --all-packages ruff check packages/suspension_multibody/src/suspension_multibody/simulation packages/suspension_multibody/tests/simulation`
- **退出码**：0
- **摘要**：All checks passed。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/ruff_check.log

## 运行时独立审查处置

- code-reviewer `b608f382-fd4a-474c-981a-aa48f43aa65c` 未发现运行时功能缺陷；保留上下文合并的复用包装，修正矩阵“直接返回同一实例”的冲突措辞，未改变复用条件、准备次数或提交次数。
- 明确所有 family 导出 `prepare_request(request) -> PreparedSimulation`，任务 03 承接 runner 改读 `vehicle_dynamic_prepared`；EPIC、矩阵和相关 SPEC 已同步。
- 全仓 ty 首次失败来自在途新增门禁测试的列表默认值、动态模块属性和 kwargs 转发类型；修复后全仓通过，未屏蔽检查规则。失败原始日志保留于 scratch 的 01-takeover-ty.log、01-takeover-ty-fixed.log、01-ty-final.log。

### 验证记录：子任务 01 / simulation
- **时间**：2026-09-20
- **子任务/门禁**：子任务 01
- **命令**：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests/simulation -q`
- **退出码**：0
- **摘要**：53 passed in 0.09s。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/01-simulation.log

### 验证记录：子任务 01 / architecture
- **时间**：2026-09-20
- **子任务/门禁**：子任务 01
- **命令**：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests/architecture -q`
- **退出码**：0
- **摘要**：44 passed in 5.59s。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/01-architecture.log

### 验证记录：子任务 01 / 全仓类型检查
- **时间**：2026-09-20
- **子任务/门禁**：子任务 01
- **命令**：`uv run --all-packages ty check .`
- **退出码**：0
- **摘要**：修复规划门禁测试类型声明后全仓通过。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/01-ty-passed.log

### 验证记录：子任务 01 / 规划门禁回归
- **时间**：2026-09-20
- **子任务/门禁**：子任务 01
- **命令**：`python .codex-tasks/20260920-unified-preparation-cutover/tasks/05-final-validation/test_planning_contract_scan.py`
- **退出码**：0
- **摘要**：标准库 unittest 门禁回归通过。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/01-planning-tests.log

### 验证记录：子任务 01 / 差异检查
- **时间**：2026-09-20
- **子任务/门禁**：子任务 01
- **命令**：`git diff --check`
- **退出码**：0
- **摘要**：无 whitespace 错误。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/01-diff.log

### 验证记录：子任务 01 / 记录审计
- **时间**：2026-09-20
- **子任务/门禁**：子任务 01
- **命令**：`python .codex-tasks/20260920-unified-preparation-cutover/tasks/05-final-validation/planning_contract_scan.py --progress-records 01`
- **退出码**：0
- **摘要**：TODO 中每条功能验收命令均有成功记录与存在的证据文件。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/01-records.log

## 当前执行恢复点

- 01 已 DONE，四步与父级证据核对完成。
- 02 fixer `2d10dfad-f254-4dfa-bc17-260cce662aff` 负责六个 preparation、cases/compiler 指定 section、family tests 和子任务02真源；不得并发写它的范围。
- 下一步：收敛02报告，主线程抽查新增接口和文档/领域路径，独立代码审查与集成回归后更新父级；再按03→04→05推进。
- 原始用户需求仍是迁移后删除旧模块；当前未删除，未声称终局完成。

- 门禁测试类型修复独立复核 `5a0d304c-4113-45c4-939c-9eb74778180c`：PASS；Sequence 默认值、动态模块 cast 和显式参数转发均不改变运行行为；逐命令成功证据、pre/post 阶段与终局校验要求未削弱。

## 子任务02交付接管

- 原 fixer 已在子任务02 PROGRESS 写入实现与验证报告；当前只读复审 aa1e1a09-c557-465c-b589-cd52d37ff9c9 进行中，尚未将02标记DONE。

### 验证记录：子任务 02 / 六个 family 回归
- **时间**：2026-09-20
- **子任务/门禁**：子任务 02
- **命令**：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests/cases/test_axle_dynamic_contract.py packages/suspension_multibody/tests/cases/kc_quasi_static packages/suspension_multibody/tests/cases/test_vehicle_kc.py packages/suspension_multibody/tests/cases/test_handling.py packages/suspension_multibody/tests/cases/test_ride_four_post.py packages/suspension_multibody/tests/cases/test_ride_random_road.py packages/suspension_multibody/tests/axle_dynamics/test_api.py -q`
- **退出码**：0
- **摘要**：fixer执行63 passed，主线程已读取原始日志核对。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/02-family-tests.log

### 验证记录：子任务 02 / architecture
- **时间**：2026-09-20
- **子任务/门禁**：子任务 02
- **命令**：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests/architecture -q`
- **退出码**：0
- **摘要**：fixer执行44 passed。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/02-architecture.log

### 验证记录：子任务 02 / axle-KC-vehicleKC
- **时间**：2026-09-20
- **子任务/门禁**：子任务 02
- **命令**：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests/cases/test_axle_dynamic_contract.py packages/suspension_multibody/tests/cases/kc_quasi_static packages/suspension_multibody/tests/cases/test_vehicle_kc.py packages/suspension_multibody/tests/axle_dynamics/test_api.py -q`
- **退出码**：0
- **摘要**：fixer执行47 passed。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/02-row2-pytest.log

### 验证记录：子任务 02 / handling-four-post-random-road
- **时间**：2026-09-20
- **子任务/门禁**：子任务 02
- **命令**：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests/cases/test_handling.py packages/suspension_multibody/tests/cases/test_ride_four_post.py packages/suspension_multibody/tests/cases/test_ride_random_road.py -q`
- **退出码**：0
- **摘要**：fixer执行16 passed。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/02-row3-pytest.log

### 验证记录：子任务 02 / non-vehicle-matrix
- **时间**：2026-09-20
- **子任务/门禁**：子任务 02
- **命令**：`python .codex-tasks/20260920-unified-preparation-cutover/tasks/05-final-validation/planning_contract_scan.py --non-vehicle-matrix`
- **退出码**：0
- **摘要**：六个非整车family矩阵检查通过。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/02-non-vehicle-matrix.log

### 验证记录：子任务 02 / registry key
- **时间**：2026-09-20
- **子任务/门禁**：子任务 02
- **命令**：`uv run --package suspension-multibody python -c "from suspension_multibody.simulation.preparation import default_preparation_registry; required={('axle','axle_dynamic'),('axle','kc_quasi_static'),('vehicle','vehicle_kc'),('vehicle','handling'),('vehicle','ride_four_post'),('vehicle','ride_random_road')}; keys=set(default_preparation_registry().keys()); assert required <= keys, (required-keys, keys)"`
- **退出码**：0
- **摘要**：六个规范化key均已登记。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/02-registry.log

## 子任务02验收与03恢复点

- 02独立审查 aa1e1a09-c557-465c-b589-cd52d37ff9c9 确認迁移成立；发现axle document校验缺口。主线程最小复现输出WRONG_FAMILY_ACCEPTED handling，随后启用DocumentPairCompiler.validate_contract_family并新增7种错误文档回归。
- 02父级证据已补全，记录审计退出码0；02四步DONE。
- 03 fixer dbf703e6-6ad9-4da3-a5fe-e2e16ecddfae 正在迁移整车准备/结果/service与runner新键，旧模块只改临时转发薄壳，04门禁前不删除。另承接02四个vehicle准备模块与对应family测试的旧prepare_vehicle_run导入迁移。
- 下一步收敛03报告、独立审查并集成验收，之后04调用方迁移与删除前后门禁、05终局全量验证。

### 验证记录：子任务 02 / 审查修复回归
- **时间**：2026-09-20
- **子任务/门禁**：子任务 02
- **命令**：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests/cases/test_axle_dynamic_contract.py packages/suspension_multibody/tests/simulation packages/suspension_multibody/tests/architecture -q`
- **退出码**：0
- **摘要**：117 passed in 10.97s；覆盖7种非法contract身份的document bypass拒绝路径。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/02-fix-tests.log

### 验证记录：子任务 02 / 修复后ruff
- **时间**：2026-09-20
- **子任务/门禁**：子任务 02
- **命令**：`uv run --all-packages ruff check packages/suspension_multibody/src packages/suspension_multibody/tests packages/suspension_multibody/scripts`
- **退出码**：0
- **摘要**：All checks passed。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/02-fix-ruff.log

### 验证记录：子任务 02 / 修复后ty
- **时间**：2026-09-20
- **子任务/门禁**：子任务 02
- **命令**：`uv run --all-packages ty check .`
- **退出码**：0
- **摘要**：All checks passed。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/02-fix-ty.log

### 验证记录：子任务 02 / 父记录审计
- **时间**：2026-09-20
- **子任务/门禁**：子任务 02
- **命令**：`python -B .codex-tasks/20260920-unified-preparation-cutover/tasks/05-final-validation/planning_contract_scan.py --progress-records 02`
- **退出码**：0
- **摘要**：planning_contract_scan=pass。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/02-parent-records.log

### 验证记录：子任务 02 / 差异检查
- **时间**：2026-09-20
- **子任务/门禁**：子任务 02
- **命令**：`git diff --check`
- **退出码**：0
- **摘要**：无whitespace错误。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/02-parent-diff.log

## 第四项历史读取验收预检

- 读取真实 `schema/loader.py` 和 `schema/dynamic.py` 后确认版本位置不一致：`_read` 强制顶层 `schema_version`，而 `DynamicResultBundle` 仅允许 `manifest/samples/diagnostics`，版本在 manifest。
- 已执行 scratch 最小复现：合法 `DynamicResultBundle.model_dump_json()` 经 `load_dynamic_result` 报 `unsupported schema_version None; expected 1`（退出码1）；增加顶层版本后报 `extra_forbidden`（退出码1）。输入保存在 session scratch 的 `legacy-bundle-probe.json` 和 `legacy-bundle-root-version-probe.json`。
- 子任务04必须先新增真实 loader 失败回归，再修正 loader 的历史 bundle 版本读取边界；不能用 model_validate 代替公开加载函数来制造兼容验收通过。其它 model/case 的顶层版本校验保持原语义。此修复承接既定 Goal 6 的历史读取兼容验收，不扩展结果格式。

- 历史读取修复规划复审：64040997 首审确认在Goal内并指出归属/验收两处缺口；已同步EPIC:153、SUBTASKS id4和04 SPEC验收5-7。ac9aecb7 定向复核PASS，两阻断关闭。保持严格schema：bundle顶层额外版本不接受、不静默丢弃；其它五个loader顶层版本检查不变。

### 验证记录：整车完整职责归属
- **时间**：2026-09-20
- **子任务/门禁**：子任务 03
- **命令**：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests/vehicle/test_service_contract.py -q -k test_vehicle_legacy_definition_map`
- **退出码**：0
- **摘要**：1 passed，12 deselected；46项职责归属及临时薄壳同对象重导出断言通过。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/03-main-definition-map.log

### 验证记录：整车compiler与preparation协议
- **时间**：2026-09-20
- **子任务/门禁**：子任务 03
- **命令**：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests/cases/test_vehicle_dynamic_contract.py packages/suspension_multibody/tests/simulation -q`
- **退出码**：0
- **摘要**：61 passed。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/03-main-compiler.log

### 验证记录：整车拆分SPEC回归
- **时间**：2026-09-20
- **子任务/门禁**：子任务 03
- **命令**：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests/vehicle/test_native_vehicle.py packages/suspension_multibody/tests/vehicle/test_service_contract.py packages/suspension_multibody/tests/results packages/suspension_multibody/tests/cases/test_vehicle_dynamic_contract.py packages/suspension_multibody/tests/simulation packages/suspension_multibody/tests/io/test_artifacts_unified.py -q`
- **退出码**：0
- **摘要**：fixer执行并回报125 passed、1 xfailed；主线程已核对原始日志。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/03_spec_validation.log

### 验证记录：整车拆分架构回归
- **时间**：2026-09-20
- **子任务/门禁**：子任务 03
- **命令**：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests/architecture -q`
- **退出码**：0
- **摘要**：fixer执行并回报44 passed；主线程已核对原始日志。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/03_architecture.log

### 验证记录：整车拆分包级全量预检
- **时间**：2026-09-20
- **子任务/门禁**：子任务 03
- **命令**：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests -q`
- **退出码**：0
- **摘要**：fixer执行并回报679 passed、1 skipped、1 xfailed，750.29s；主线程已核对原始日志。不能替代04删除前后与05终局验证。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/03_full_suite.log

### 验证记录：整车结果层与service
- **时间**：2026-09-20
- **子任务/门禁**：子任务 03
- **命令**：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests/results packages/suspension_multibody/tests/vehicle/test_service_contract.py -q`
- **退出码**：0
- **摘要**：25 passed。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/03-main-results.log

### 验证记录：整车service与artifact
- **时间**：2026-09-20
- **子任务/门禁**：子任务 03
- **命令**：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests/vehicle packages/suspension_multibody/tests/io/test_artifacts_unified.py -q`
- **退出码**：0
- **摘要**：fixer执行并回报68 passed、1 xfailed。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/03_vehicle_io.log

### 验证记录：整车统一解码和native唯一入口
- **时间**：2026-09-20
- **子任务/门禁**：子任务 03
- **命令**：`uv run --package suspension-multibody python .codex-tasks/20260920-unified-preparation-cutover/tasks/04-delete-legacy/architecture_contract_scan.py`
- **退出码**：0
- **摘要**：fixer执行并回报统一解码唯一位于results/decoder.py；native提交唯一位于simulation/backend.py:24。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/03_arch_contract_scan.log

### 验证记录：整车新归属导入检查
- **时间**：2026-09-20
- **子任务/门禁**：子任务 03
- **命令**：`uv run --package suspension-multibody python -c "import importlib, inspect; from suspension_multibody.simulation.preparation import default_preparation_registry; prep=importlib.import_module('suspension_multibody.preparation.vehicle_dynamic'); result=importlib.import_module('suspension_multibody.results.vehicle'); service=importlib.import_module('suspension_multibody.vehicle.service'); decoder=importlib.import_module('suspension_multibody.results.decoder'); assert ('vehicle','vehicle_dynamic') in default_preparation_registry().keys(); assert all(hasattr(prep,name) for name in ('PreparedVehicleRun','prepare_vehicle_run','adapt_legacy_prepared_request')); assert all(hasattr(result,name) for name in ('VehicleDynamicsResult','_vehicle_axle_result','_contract_constraint_names','decode_vehicle_result')); assert hasattr(service,'run_vehicle_dynamics'); assert 'vehicle_dynamics' not in inspect.getsource(decoder)"`
- **退出码**：0
- **摘要**：fixer执行并回报新符号归属、registry key和decoder无旧导入检查通过。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/03_quickcheck.log

### 验证记录：整车decoder唯一性检查
- **时间**：2026-09-20
- **子任务/门禁**：子任务 03
- **命令**：`uv run --package suspension-multibody python -c "import inspect; from suspension_multibody.results import decoder, vehicle; assert hasattr(vehicle,'decode_vehicle_result'); src=inspect.getsource(decoder); assert 'vehicle_dynamics' not in src; assert src.count('def decode_result')==1"`
- **退出码**：0
- **摘要**：fixer执行并回报family adapter存在、decoder无旧路径且唯一定义检查通过。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/03_todo3_quick.log

### 验证记录：整车拆分收口回归
- **时间**：2026-09-20
- **子任务/门禁**：子任务 03
- **命令**：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests/vehicle packages/suspension_multibody/tests/results packages/suspension_multibody/tests/cases/test_vehicle_dynamic_contract.py packages/suspension_multibody/tests/simulation packages/suspension_multibody/tests/io/test_artifacts_unified.py -q`
- **退出码**：0
- **摘要**：141 passed、1 xfailed，6.39s。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/03-main-close-tests.log

### 验证记录：整车拆分父记录审计
- **时间**：2026-09-20
- **子任务/门禁**：子任务 03
- **命令**：`python .codex-tasks/20260920-unified-preparation-cutover/tasks/05-final-validation/planning_contract_scan.py --progress-records 03`
- **退出码**：0
- **摘要**：planning_contract_scan=pass；子任务03全部原子验收命令已有成功记录和真实日志。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/03-main-evidence-audit.log

## 第三项独立复审与document优先级补修

- 4aa65071只读复审确认整车职责、单一类型、legacy adapter、partial证据无阻断；指出prepared与document并存时compiler优先读prepared。主线程按EPIC:109文档优先契约判定必须修复，而非可选建议。
- 整车新增4例先复现重新author错误，再改VehicleDynamicCompiler使用既有 `_is_document_request` 优先分派；顺查发现axle同源问题，给已有非法文档测试加prepared并存组合，7例先复现TypeError，再在 `_axle_dynamic_prepared` 对document请求返回None。这是02/03同一冻结契约的主线程补修，不迁移物理逻辑或扩大接口。
- tests/simulation/test_compilers.py单行旧键到新键迁移已验收为03必需接线。
- 其它非阻断建议（常量模块级依赖、service额外计数）未扩展实现。
- 长回归首次工具timeout60，scratch `03-document-priority-all-green.log`仅为未完成输出，不计通过；已交test-runner 4d69b3d3执行同一命令并单独存新日志。

### 验证记录：整车文档优先级失败复现
- **时间**：2026-09-20
- **子任务/门禁**：子任务 03
- **命令**：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests/cases/test_vehicle_dynamic_contract.py -q -k documents_take_priority --tb=short`
- **退出码**：1
- **摘要**：4 failed；document与prepared并存时错误重新author。前一版测试常量导入错误已修正，本日志为目标缺陷真实复现。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/03-document-priority-reproduced.log

### 验证记录：车轴文档优先级失败复现
- **时间**：2026-09-20
- **子任务/门禁**：子任务 02
- **命令**：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests/cases/test_axle_dynamic_contract.py -q -k document_bypass_rejects_invalid_contract_identity --tb=line`
- **退出码**：1
- **摘要**：7 failed、7 passed；prepared类型检查抢在文档身份检查前。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/02-document-priority-red.log

### 验证记录：整车文档优先级修复回归
- **时间**：2026-09-20
- **子任务/门禁**：子任务 03
- **命令**：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests/cases/test_vehicle_dynamic_contract.py packages/suspension_multibody/tests/simulation packages/suspension_multibody/tests/architecture packages/suspension_multibody/tests/vehicle/test_service_contract.py -q`
- **退出码**：0
- **摘要**：122 passed，6.65s。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/03-document-priority-green.log

### 验证记录：文档优先级修复ruff
- **时间**：2026-09-20
- **子任务/门禁**：子任务 03
- **命令**：`uv run --all-packages ruff check packages/suspension_multibody/src packages/suspension_multibody/tests packages/suspension_multibody/scripts`
- **退出码**：0
- **摘要**：All checks passed。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/03-priority-ruff.log

### 验证记录：文档优先级修复全仓ty
- **时间**：2026-09-20
- **子任务/门禁**：子任务 03
- **命令**：`uv run --all-packages ty check .`
- **退出码**：0
- **摘要**：All checks passed。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/03-priority-ty.log

- 7fc8ca12对两处document优先级修复定向复核PASS；文档分支优先且领域请求既有检查不变。等待4d69b3d3长回归完成后关闭03，随后进入04；禁止把03首次全包679通过视为优先级修复后的结果。

### 验证记录：文档优先级修复diff检查
- **时间**：2026-09-20
- **子任务/门禁**：子任务 03
- **命令**：`git diff --check`
- **退出码**：0
- **摘要**：无whitespace错误；仅已有文件CRLF提示。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/03-priority-diff.log

### 验证记录：文档优先级修复完整影响范围回归
- **时间**：2026-09-20
- **子任务/门禁**：子任务 03
- **命令**：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests/cases packages/suspension_multibody/tests/simulation packages/suspension_multibody/tests/vehicle packages/suspension_multibody/tests/results packages/suspension_multibody/tests/io/test_artifacts_unified.py packages/suspension_multibody/tests/architecture -q`
- **退出码**：0
- **摘要**：test-runner 4d69b3d3 回报260 passed、1 xfailed，164.03s；无残留pytest进程，未触发1800秒超时。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/03-document-priority-regression.log

## 第三项关闭与 04 启动

- 03 实现审查 `4aa65071` 与 document 优先级定向审查 `7fc8ca12` 均 PASS；修复后影响范围回归由 test-runner `4d69b3d3-58f1-4fd0-a919-ce782c62964f` 完成（260 passed、1 xfailed、164.03s），父级记录见上。
- 03 关闭：`tasks/03-vehicle-split/TODO.csv` 第 5 步与 `SUBTASKS.csv` 第 3 行置 DONE；`vehicle_dynamics.py` 仍是临时重导出薄壳，未删除。
- 04 启动：调用方迁移（顶层 `__init__.py`、CLI、Adams、scripts、docs）、历史 `DynamicResultBundle` 读取兼容测试与 loader 最小修复、以及删除前后门禁。删除动作在 pre-delete 门禁逐条通过并记录前不执行。

### 验证记录：子任务03关闭 / 父级证据审计
- **时间**：2026-09-20
- **子任务/门禁**：子任务 03
- **命令**：`python .codex-tasks/20260920-unified-preparation-cutover/tasks/05-final-validation/planning_contract_scan.py --progress-records 03`
- **退出码**：0
- **摘要**：planning_contract_scan=pass；子任务 03 全部原子验收命令均有退出码 0 记录且证据文件存在，记录字段完整。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/03-final-evidence-audit.log

## 04 调用方迁移与历史读取兼容

- 调用方迁移已完成：顶层 `__init__.py`、`cli.py`、`tests/adams/test_full_vehicle_model.py`、`scripts/case_parity_check.py`、`scripts/diagnose_native_*.py`、`scripts/measure_native_fiala_solver_time.py`、`scripts/run_full_native_three_model_comparison.py`、`scripts/run_native_tire_rig.py` 注释、`scripts/kc_legacy_path_check.py` 的 native 路径清单和 `docs/axle_dynamics_architecture.md` 均改指 `preparation.vehicle_dynamic` / `results.vehicle` / `vehicle.service`。`legacy_reference_scan.py --pre-delete` 已无命中。
- 历史读取兼容：新增 `tests/schema/test_dynamic_result_compat.py` 先复现失败，再最小修复 `schema/loader.py` 的 bundle 版本读取边界；bundle 顶层 `schema_version` 仍按未知字段拒绝，其它五个 loader 的顶层版本检查不变。

### 验证记录：历史读取兼容失败复现
- **时间**：2026-09-20
- **子任务/门禁**：子任务 04-pre-delete
- **命令**：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests/schema/test_dynamic_result_compat.py -q`
- **退出码**：1
- **摘要**：9 failed、9 passed；修复前真实复现合法 `DynamicResultBundle` 经公开 `load_dynamic_result` 报 `unsupported schema_version None; expected 1`，根因是 `_read` 要求顶层版本而 bundle schema 只允许 manifest 版本。失败证据不计作删除门禁通过。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/04-dynamic-result-compat-red.log

### 验证记录：历史读取兼容修复后回归
- **时间**：2026-09-20
- **子任务/门禁**：子任务 04-pre-delete
- **命令**：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests/schema/test_dynamic_result_compat.py -q`
- **退出码**：0
- **摘要**：18 passed；`load_dynamic_result` 读取生产写出与 schema 合法样例的 JSON/YAML 成功，manifest 版本缺失/非1、bundle/manifest/sample 未知字段、bundle 顶层 `schema_version` 均被拒；五个输入 loader 的顶层版本检查不变；adams 历史 adapter 的 body 过滤、时间排序、缺 body/metrics 失败与统一 `TimeSeriesResult` 分支均覆盖。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/04-dynamic-result-compat-green.log

### 验证记录：历史读取与统一 artifact 定向回归
- **时间**：2026-09-20
- **子任务/门禁**：子任务 04-pre-delete
- **命令**：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests/schema/test_dynamic_result_compat.py packages/suspension_multibody/tests/io/test_artifacts_unified.py -q`
- **退出码**：0
- **摘要**：24 passed；历史 bundle 读取与统一 artifact 写出/读取（含 `vehicle_dynamics_result` artifact 类型标识）同时通过。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/04-history-artifact.log

### 验证记录：CLI 迁移后回归
- **时间**：2026-09-20
- **子任务/门禁**：子任务 04-pre-delete
- **命令**：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests/cli -q`
- **退出码**：0
- **摘要**：5 passed；`run-vehicle-dynamics` / `validate-vehicle-dynamics` 命令在顶层导入改指 `results.vehicle` 与 `vehicle.service` 后仍可用。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/04-cli.log

### 验证记录：调用方迁移后 scoped ruff
- **时间**：2026-09-20
- **子任务/门禁**：子任务 04-pre-delete
- **命令**：`uv run --all-packages ruff check packages/suspension_multibody/src packages/suspension_multibody/tests packages/suspension_multibody/scripts`
- **退出码**：0
- **摘要**：All checks passed。首轮在同一范围报出 8 条 I001/D213（迁移后的 import 顺序与新写 docstring 摘要位置），已按 isort 顺序重排并改写 docstring，未放宽或忽略任何规则。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/04-ruff.log

### 验证记录：迁移文件编译检查
- **时间**：2026-09-20
- **子任务/门禁**：子任务 04-pre-delete
- **命令**：`uv run --all-packages python -m compileall -q packages/suspension_multibody/src/suspension_multibody packages/suspension_multibody/scripts packages/suspension_multibody/tests/schema/test_dynamic_result_compat.py`
- **退出码**：0
- **摘要**：迁移涉及的 src 包、scripts 与新测试文件全部编译通过（无输出）。完整 `compileall` 门禁由主线程执行。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/04-compileall-scoped.log

### 验证记录：顶层公开导出同一性
- **时间**：2026-09-20
- **子任务/门禁**：子任务 04-pre-delete
- **命令**：`uv run --package suspension-multibody python -c "import suspension_multibody as pkg; from suspension_multibody.results.vehicle import VehicleDynamicsResult; from suspension_multibody.vehicle.service import run_vehicle_dynamics; assert pkg.VehicleDynamicsResult is VehicleDynamicsResult, 'result identity'; assert pkg.run_vehicle_dynamics is run_vehicle_dynamics, 'service identity'; assert pkg.__all__.count('run_vehicle_dynamics') == 1; assert pkg.__all__.count('VehicleDynamicsResult') == 1; print('public-api-identity=ok')"`
- **退出码**：0
- **摘要**：`public-api-identity=ok`；顶层导出改指 `results.vehicle`/`vehicle.service` 后仍是同一对象，`__all__` 无重复项。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/04-public-api-identity.log

### 验证记录：native 路径检查脚本迁移后运行
- **时间**：2026-09-20
- **子任务/门禁**：子任务 04-pre-delete
- **命令**：`uv run --package suspension-multibody python packages/suspension_multibody/scripts/kc_legacy_path_check.py`
- **退出码**：0
- **摘要**：`native-path references to the Python solver : 0`；`legacy consumers still using the solver : 1`（既有 `api.py:49`，与本次迁移无关）；脚本的 native 路径清单已由旧 `vehicle_dynamics.py` 改为 `preparation/vehicle_dynamic.py`、`vehicle/service.py`、`axle_dynamics`，删除后仍可失败。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/04-kc-legacy-path-check.log

### 验证记录：架构门禁回归
- **时间**：2026-09-20
- **子任务/门禁**：子任务 04-pre-delete
- **命令**：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests/architecture -q`
- **退出码**：0
- **摘要**：44 passed；公开 API 边界门禁（含空 LEGACY_ALLOWLIST 的严格比对）在新顶层导出与新增测试文件下无新违规。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/04-architecture.log

### 验证记录：Adams 整车测试迁移后回归
- **时间**：2026-09-20
- **子任务/门禁**：子任务 04-pre-delete
- **命令**：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests/adams/test_full_vehicle_model.py -q`
- **退出码**：0
- **摘要**：37 passed、1 skipped；`run_vehicle_dynamics` 导入改指 `vehicle.service` 后行为不变。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/04-adams-full-vehicle.log

### 验证记录：迁移后全仓类型检查
- **时间**：2026-09-20
- **子任务/门禁**：子任务 04-pre-delete
- **命令**：`uv run --all-packages ty check .`
- **退出码**：0
- **摘要**：All checks passed；`schema/loader.py` 的 `_read(version_path=...)`/`_version_at` 与迁移后的调用方通过全仓类型检查。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/04-ty.log

### 验证记录：ruff 修复后历史读取复跑
- **时间**：2026-09-20
- **子任务/门禁**：子任务 04-pre-delete
- **命令**：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests/schema/test_dynamic_result_compat.py packages/suspension_multibody/tests/io/test_artifacts_unified.py -q`
- **退出码**：0
- **摘要**：24 passed；在 ruff 重排 import 与改写 docstring 之后复跑确认历史读取与 artifact 回归仍为绿色（同命令前次记录见上，日志已按本次运行覆盖）。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/04-history-artifact.log

### 验证记录：调用方迁移后删除前引用扫描
- **时间**：2026-09-20
- **子任务/门禁**：子任务 04-pre-delete
- **命令**：`uv run --package suspension-multibody python .codex-tasks/20260920-unified-preparation-cutover/tasks/04-delete-legacy/legacy_reference_scan.py --pre-delete`
- **退出码**：0
- **摘要**：无输出；`packages`、`docs`、`.github`、根 `scripts`、`README.md`、`pyproject.toml`、`justfile` 中已无旧模块 import/路径引用，目标文件仍存在（pre-delete 断言通过）。迁移前同一命令退出码 1、命中 9 个文件（日志 `04-predelete-baseline.log`，不计通过）。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/04-predelete-migration.log

### 验证记录：迁移后唯一解码与 native 提交点扫描
- **时间**：2026-09-20
- **子任务/门禁**：子任务 04-pre-delete
- **命令**：`uv run --package suspension-multibody python .codex-tasks/20260920-unified-preparation-cutover/tasks/04-delete-legacy/architecture_contract_scan.py`
- **退出码**：0
- **摘要**：`decode_result=results/decoder.py`、`native_submission=simulation/backend.py:24`；调用方迁移未引入第二个统一解码定义或第二个 native 提交点。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/04-architecture-contract-scan.log

### 验证记录：Adams/vehicle/cases 调用方迁移回归
- **时间**：2026-09-20
- **子任务/门禁**：子任务 04-pre-delete
- **命令**：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests/adams packages/suspension_multibody/tests/vehicle packages/suspension_multibody/tests/cases -q`
- **退出码**：0
- **摘要**：350 passed、1 skipped、1 xfailed，686.02s；本次进程以 `timeout 1700` 包裹同一命令启动（仅外层超时包装，pytest 参数与上面记录的命令一致），日志为该次运行的原始输出。覆盖 04 TODO 第 2 行的验收命令，是迁移后最宽的一次定向回归。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/04-adams-vehicle-cases.log

### 验证记录：04-pre-delete 阶段记录审计
- **时间**：2026-09-20
- **子任务/门禁**：子任务 04-pre-delete
- **命令**：`python .codex-tasks/20260920-unified-preparation-cutover/tasks/05-final-validation/planning_contract_scan.py --progress-records 04-pre-delete`
- **退出码**：0
- **摘要**：planning_contract_scan=pass；`子任务 04-pre-delete` 记录可识别、未混入 `04-post-delete` 标记，含一条退出码 0 且带存在证据文件的 `legacy_reference_scan.py --pre-delete` 记录。本审计不替代正式 pre-delete 门禁。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/04-pre-delete-records-audit.log

### 验证记录：正式删除前功能门禁
- **时间**：2026-09-20
- **子任务/门禁**：子任务 04-pre-delete
- **命令**：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests/vehicle packages/suspension_multibody/tests/results packages/suspension_multibody/tests/cases/test_vehicle_dynamic_contract.py packages/suspension_multibody/tests/adams packages/suspension_multibody/tests/cli packages/suspension_multibody/tests/architecture packages/suspension_multibody/tests/schema/test_dynamic_result_compat.py packages/suspension_multibody/tests/io/test_artifacts_unified.py -q`
- **退出码**：0
- **摘要**：test-runner b5e01f2f 回报364 passed、1 skipped、1 xfailed，541.58s；退出码另存04-pre-gate-pytest.exit，未超时重启。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/04-pre-gate-pytest.log

## 04 独立复审收敛

- e0a6b676只读复审PASS：公开导出、CLI异常/partial/artifact分支、全部迁移符号和严格历史loader均无阻断。主线程核对loader与新增兼容测试，接受其结论。
- 两处与本次迁移直接相关的收尾由主线程处理：cases/vehicle_dynamic.py的旧路径文档指引改为新preparation路径；kc_legacy_path_check.py补入results/vehicle.py，保持旧文件结果职责迁出后的扫描覆盖。前者只改docstring，后者只改静态门禁目标清单，不改变已通过功能回归的运行逻辑；以定向脚本检查和随后完整静态门禁验证。
- reviewer建议的既有schema/test_loader.py回归将在05全包验收覆盖；无真实历史bundle fixture，兼容结论仍限于schema合法样例往返。

### 验证记录：正式删除前ruff
- **时间**：2026-09-20
- **子任务/门禁**：子任务 04-pre-delete
- **命令**：`uv run --all-packages ruff check packages/suspension_multibody/src packages/suspension_multibody/tests packages/suspension_multibody/scripts`
- **退出码**：0
- **摘要**：完整范围ruff通过，包含复审后的文档和扫描清单修正。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/04-pre-gate-ruff.log

### 验证记录：正式删除前compileall
- **时间**：2026-09-20
- **子任务/门禁**：子任务 04-pre-delete
- **命令**：`uv run --all-packages python -m compileall -q packages/suspension_multibody/src packages/suspension_multibody/tests packages/suspension_multibody/scripts`
- **退出码**：0
- **摘要**：src/tests/scripts完整编译检查通过。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/04-pre-gate-compileall.log

### 验证记录：正式删除前ty
- **时间**：2026-09-20
- **子任务/门禁**：子任务 04-pre-delete
- **命令**：`uv run --all-packages ty check .`
- **退出码**：0
- **摘要**：全仓类型检查通过。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/04-pre-gate-ty.log

### 验证记录：正式删除前diff check
- **时间**：2026-09-20
- **子任务/门禁**：子任务 04-pre-delete
- **命令**：`git diff --check`
- **退出码**：0
- **摘要**：diff check通过；无新增空白错误，保留既有CRLF提示。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/04-pre-gate-diff.log

### 验证记录：正式删除前引用扫描
- **时间**：2026-09-20
- **子任务/门禁**：子任务 04-pre-delete
- **命令**：`uv run --package suspension-multibody python .codex-tasks/20260920-unified-preparation-cutover/tasks/04-delete-legacy/legacy_reference_scan.py --pre-delete`
- **退出码**：0
- **摘要**：旧文件存在，交付目录旧模块import/path引用清零。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/04-pre-gate-references.log

### 验证记录：正式删除前架构契约扫描
- **时间**：2026-09-20
- **子任务/门禁**：子任务 04-pre-delete
- **命令**：`uv run --package suspension-multibody python .codex-tasks/20260920-unified-preparation-cutover/tasks/04-delete-legacy/architecture_contract_scan.py`
- **退出码**：0
- **摘要**：唯一统一decode_result位于results/decoder.py，唯一native submission位于simulation/backend.py。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/04-pre-gate-architecture.log

### 验证记录：复审后solver扫描覆盖检查
- **时间**：2026-09-20
- **子任务/门禁**：子任务 04-pre-delete
- **命令**：`uv run --package suspension-multibody python packages/suspension_multibody/scripts/kc_legacy_path_check.py`
- **退出码**：0
- **摘要**：新增results/vehicle.py扫描范围后脚本通过，native路径无旧Python solver引用。cases文档指引已改到preparation.vehicle_dynamic.prepare_vehicle_run；无运行逻辑改动。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/04-pre-gate-solver-scan.log

### 验证记录：正式删除前阶段审计
- **时间**：2026-09-20
- **子任务/门禁**：子任务 04-pre-delete
- **命令**：`python .codex-tasks/20260920-unified-preparation-cutover/tasks/05-final-validation/planning_contract_scan.py --progress-records 04-pre-delete`
- **退出码**：0
- **摘要**：阶段记录审计通过。主线程另逐条核对SPEC七条正式门禁：pytest、ruff、compileall、ty、diff、pre-delete scan、architecture scan均已真实执行且退出码0，有各自04-pre-gate日志与父记录。完整符号映射已由364项功能门禁和独立复审覆盖；目标仍是118行纯重导出薄壳，下一动作单独删除。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/04-pre-gate-audit.log

## 04 已关闭：当前恢复入口

- 历史：主线程在全部 pre-delete 门禁通过并即时记录后，以独立 Edit REM 删除 packages/suspension_multibody/src/suspension_multibody/vehicle_dynamics.py；读取确认其只有 118 行重导出，无自有实现。
- 删除后功能门禁由 test-runner 7ddac9f6-9e79-452c-ad0d-6f243ef0d853 执行，364 passed/1 skipped/1 xfailed（533.42s），日志 scratch/04-post-gate-pytest.log、退出码 04-post-gate-pytest.exit；删除后 ruff、compileall、ty、diff、architecture scan、post-delete reference scan 逐条 exit0 并已即时写入 子任务 04-post-delete 记录。
- 04 收口：TODO 第 5 步 DONE、child PROGRESS 5/5、SUBTASKS 第 4 行 DONE；--progress-records 04-post-delete 与 04 均 exit 0；Epic/父 PROGRESS 恢复信息 4/5，Epic 未标 DONE。
- 05需跑TODO/SPEC/SUBTASKS内所有不同原子命令并记子任务05；包括TODO第1项额外vehicle/results范围、SPEC独立architecture/cli/simulation、Adams时间域+artifact、历史loader+artifact、全包pytest、ruff/compileall/ty/diff、post-delete/architecture扫描、SUBTASKS的AST唯一decode_result命令。每个原子命令必须实际独立执行后立即记录，不把前阶段日志冒充终局结果。
- 最后05前四步DONE后执行--final-preclose；通过才同步所有DONE并运行--final，失败回退。没有Git提交。

### 验证记录：审计脚本自身回归
- **时间**：2026-09-20
- **子任务/门禁**：规划门禁 05 / 审计脚本回归
- **命令**：`uv run --package suspension-multibody pytest .codex-tasks/20260920-unified-preparation-cutover/tasks/05-final-validation/test_planning_contract_scan.py -q`
- **退出码**：0
- **摘要**：23 passed，0.58s；验证记录缺失/失败/日志缺失、pre/post阶段和预收口/终局状态检查。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/05-planning-gate-tests.log

### 验证记录：正式删除后ruff
- **时间**：2026-09-20
- **子任务/门禁**：子任务 04-post-delete
- **命令**：`uv run --all-packages ruff check packages/suspension_multibody/src packages/suspension_multibody/tests packages/suspension_multibody/scripts`
- **退出码**：0
- **摘要**：删除旧模块后的完整范围ruff通过。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/04-post-gate-ruff.log

### 验证记录：正式删除后compileall
- **时间**：2026-09-20
- **子任务/门禁**：子任务 04-post-delete
- **命令**：`uv run --all-packages python -m compileall -q packages/suspension_multibody/src packages/suspension_multibody/tests packages/suspension_multibody/scripts`
- **退出码**：0
- **摘要**：删除旧模块后 src/tests/scripts 完整编译检查通过，无输出。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/04-post-gate-compileall.log

### 验证记录：正式删除后ty
- **时间**：2026-09-20
- **子任务/门禁**：子任务 04-post-delete
- **命令**：`uv run --all-packages ty check .`
- **退出码**：0
- **摘要**：全仓类型检查通过（`All checks passed!`），旧模块删除后无残留类型引用。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/04-post-gate-ty.log

### 验证记录：正式删除后diff check
- **时间**：2026-09-20
- **子任务/门禁**：子任务 04-post-delete
- **命令**：`git diff --check`
- **退出码**：0
- **摘要**：diff check 通过，无空白错误；仅保留既有的 `LF will be replaced by CRLF` 提示（18 个已跟踪文件的换行提示，与本次删除无关）。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/04-post-gate-diff.log

### 验证记录：正式删除后架构契约扫描
- **时间**：2026-09-20
- **子任务/门禁**：子任务 04-post-delete
- **命令**：`uv run --package suspension-multibody python .codex-tasks/20260920-unified-preparation-cutover/tasks/04-delete-legacy/architecture_contract_scan.py`
- **退出码**：0
- **摘要**：删除旧模块后仍为唯一统一解码定义 `decode_result=results/decoder.py`、唯一 native 提交点 `native_submission=simulation/backend.py:24`。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/04-post-gate-architecture.log

### 验证记录：正式删除后引用扫描
- **时间**：2026-09-20
- **子任务/门禁**：子任务 04-post-delete
- **命令**：`uv run --package suspension-multibody python .codex-tasks/20260920-unified-preparation-cutover/tasks/04-delete-legacy/legacy_reference_scan.py --post-delete`
- **退出码**：0
- **摘要**：无输出；post-delete 断言目标文件不存在通过，且 `packages`、`docs`、`.github`、根 `scripts`、`README.md`、`pyproject.toml`、`justfile` 中已无旧模块 import/路径引用。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/04-post-gate-references.log

### 验证记录：正式删除后功能门禁
- **时间**：2026-09-20
- **子任务/门禁**：子任务 04-post-delete
- **命令**：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests/vehicle packages/suspension_multibody/tests/results packages/suspension_multibody/tests/cases/test_vehicle_dynamic_contract.py packages/suspension_multibody/tests/adams packages/suspension_multibody/tests/cli packages/suspension_multibody/tests/architecture packages/suspension_multibody/tests/schema/test_dynamic_result_compat.py packages/suspension_multibody/tests/io/test_artifacts_unified.py -q`
- **退出码**：0
- **摘要**：test-runner 7ddac9f6回报364 passed、1 skipped、1 xfailed，533.42s；独立删除后运行，退出码副本04-post-gate-pytest.exit。静态门禁代理写入结束后立即合并此记录，无并发改写。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/04-post-gate-pytest.log

### 验证记录：正式删除后阶段记录审计
- **时间**：2026-09-20
- **子任务/门禁**：子任务 04-post-delete
- **命令**：`python .codex-tasks/20260920-unified-preparation-cutover/tasks/05-final-validation/planning_contract_scan.py --progress-records 04-post-delete`
- **退出码**：0
- **摘要**：planning_contract_scan=pass；`子任务 04-post-delete` 记录可识别、未混入 `04-pre-delete` 标记，含一条退出码 0 且带存在证据文件的 `legacy_reference_scan.py --post-delete` 记录。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/04-post-delete-records-audit.log

### 验证记录：04 全任务记录审计
- **时间**：2026-09-20
- **子任务/门禁**：子任务 04
- **命令**：`python .codex-tasks/20260920-unified-preparation-cutover/tasks/05-final-validation/planning_contract_scan.py --progress-records 04`
- **退出码**：0
- **摘要**：planning_contract_scan=pass；tasks/04 TODO 第1-5行每个原子命令（cli、adams+vehicle+cases、architecture、删除前后完整门禁链）均有子任务 04 成功记录且证据文件存在。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/04-records-audit.log

## 05 文档运行缺陷修复与红绿证据

- 05 前序 fixer（0b9812a5）被主动停止；静态独立审查 cb5812cf 确认 6 个目标但发现 axle 文档运行级缺陷，主线程真实复现后判定不能收口。
- 根因：03 解码接线暴露的现有缺口——`results/decoder.py` 对 `axle`/`axle_dynamic` 缺少模型/case 实例校验，document（raw）请求被错误分派到 typed 解码路径。
- 最小修复（主线程，代码冻结）：`results/decoder.py` 分派加入 `AxleDynamicsModel`/`AxleDynamicsCase` 实例校验，否则保持原有 raw fallback；`tests/cases/test_axle_dynamic_contract.py` 末尾新增运行级回归 `test_document_request_runs_without_domain_decoding`（doc both/model/case × facade/staged 共 6 例：真实 native 仅 1 次、无重复准备、raw 非空有限、typed 为 None）。回退到 03 责任边界补最小修复，不恢复旧文件、不改物理。
- 原 `05-final-full-pytest.log` 为被中止/修复前运行（仅 20% 即中断），不计为通过，旧日志保留；已确认系统无旧 pytest 进程。

### 验证记录：子任务 05 / 文档运行失败复现（probe）
- **时间**：2026-09-20
- **子任务/门禁**：子任务 05
- **命令**：`uv run --package suspension-multibody python "C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/05-document-run-probe.py"`
- **退出码**：1
- **摘要**：native 运行成功后 typed 解码报错 `AttributeError: 'dict' object has no attribute 'tires'`（`axle_dynamics/contract_run.py:209`），document 请求被错误分派到 typed 解码路径。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/05-document-run-red.log

### 验证记录：子任务 05 / 文档运行回归失败复现（测试）
- **时间**：2026-09-20
- **子任务/门禁**：子任务 05
- **命令**：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests/cases/test_axle_dynamic_contract.py -k document_request_runs_without_domain_decoding -q`
- **退出码**：1
- **摘要**：6 failed、26 deselected（1.36s）；其中 4 个初始 mixed 用例 context 未给显式 model_document/case_document，在 `simulation/compiler.py:36` 报 `TypeError: model_document must be a document or (document, payload) pair`，如实保留。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/05-document-run-tests-red.log

### 验证记录：子任务 05 / 文档运行回归失败复现（补齐 context 后）
- **时间**：2026-09-20
- **子任务/门禁**：子任务 05
- **命令**：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests/cases/test_axle_dynamic_contract.py -k document_request_runs_without_domain_decoding -q`
- **退出码**：1
- **摘要**：补齐显式 context `model_document`/`case_document` 后同命令仍 6 failed、26 deselected（2.22s）：4 例为 `AttributeError: 'dict' object has no attribute 'tires'`（原始缺口），2 例为 `run.result is None` 断言失败（raw fallback 前返回了 typed `AxleDynamicsResult`）。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/05-document-run-tests-reproduced.log

### 验证记录：子任务 05 / 文档运行回归修复后（定向）
- **时间**：2026-09-20
- **子任务/门禁**：子任务 05
- **命令**：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests/cases/test_axle_dynamic_contract.py packages/suspension_multibody/tests/results -q`
- **退出码**：0
- **摘要**：44 passed（7.36s），含 6 个新增运行级回归。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/05-document-run-green.log

### 验证记录：子任务 05 / 文档运行修复后（probe 绿）
- **时间**：2026-09-20
- **子任务/门禁**：子任务 05
- **命令**：`uv run --package suspension-multibody python "C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/05-document-run-probe.py"`
- **退出码**：0
- **摘要**：`document_run=pass success`；raw 非空且 typed 结果为 None。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/05-document-run-probe-green.log

### 验证记录：子任务 05 / 修复后全仓类型检查
- **时间**：2026-09-20
- **子任务/门禁**：子任务 05
- **命令**：`uv run --all-packages ty check .`
- **退出码**：0
- **摘要**：`All checks passed!`（decoder.py 实例校验与新增测试无类型问题）。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/05-document-run-ty.log

### 验证记录：子任务 05 / 专项全量回归
- **时间**：2026-09-20
- **子任务/门禁**：子任务 05
- **命令**：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests -q`
- **退出码**：0
- **摘要**：714 passed、1 skipped、1 xfailed（677.06s / 11:17）；退出码副本 05-final-r2-full-package.exit（内容 0），由 05 收口步骤单独核对。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/05-final-r2-full-package.log

### 验证记录：子任务 05 / 核心架构回归（TODO 第1步）
- **时间**：2026-09-20
- **子任务/门禁**：子任务 05
- **命令**：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests/architecture packages/suspension_multibody/tests/cli packages/suspension_multibody/tests/simulation packages/suspension_multibody/tests/vehicle packages/suspension_multibody/tests/results -q`
- **退出码**：0
- **摘要**：177 passed、1 xfailed（11.06s）；architecture/CLI/simulation/vehicle/results 关键路径通过；退出码副本 05-final-r2-todo-core.exit（内容 0）。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/05-final-r2-todo-core.log

### 验证记录：子任务 05 / SPEC 架构-CLI-simulation 回归
- **时间**：2026-09-20
- **子任务/门禁**：子任务 05
- **命令**：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests/architecture packages/suspension_multibody/tests/cli packages/suspension_multibody/tests/simulation -q`
- **退出码**：0
- **摘要**：103 passed（5.52s）；退出码副本 05-final-r2-spec-arch-cli-sim.exit（内容 0）。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/05-final-r2-spec-arch-cli-sim.log

### 验证记录：子任务 05 / SPEC artifact 与 Adams 时间域回归
- **时间**：2026-09-20
- **子任务/门禁**：子任务 05
- **命令**：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests/io/test_artifacts_unified.py packages/suspension_multibody/tests/adams/test_time_domain_axle.py packages/suspension_multibody/tests/adams/test_time_domain_vehicle_kc.py -q`
- **退出码**：0
- **摘要**：12 passed（1.20s）；统一 artifact 与 Adams axle/vehicle-KC 时间域读取回归通过；退出码副本 05-final-r2-spec-artifact-adams.exit（内容 0）。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/05-final-r2-spec-artifact-adams.log

### 验证记录：子任务 05 / 历史 DynamicResultBundle 与统一 artifact 回归
- **时间**：2026-09-20
- **子任务/门禁**：子任务 05
- **命令**：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests/schema/test_dynamic_result_compat.py packages/suspension_multibody/tests/io/test_artifacts_unified.py -q`
- **退出码**：0
- **摘要**：24 passed（0.40s）；历史读取兼容与统一 artifact 回归通过（无真实历史 fixture，兼容限于 schema 合法样例）；退出码副本 05-final-r2-history-artifact.exit（内容 0）。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/05-final-r2-history-artifact.log

### 验证记录：子任务 05 / scoped ruff
- **时间**：2026-09-20
- **子任务/门禁**：子任务 05
- **命令**：`uv run --all-packages ruff check packages/suspension_multibody/src packages/suspension_multibody/tests packages/suspension_multibody/scripts`
- **退出码**：0
- **摘要**：`All checks passed!`；退出码副本 05-final-r2-ruff.exit（内容 0）。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/05-final-r2-ruff.log

### 验证记录：子任务 05 / compileall
- **时间**：2026-09-20
- **子任务/门禁**：子任务 05
- **命令**：`uv run --all-packages python -m compileall -q packages/suspension_multibody/src packages/suspension_multibody/tests packages/suspension_multibody/scripts`
- **退出码**：0
- **摘要**：src/tests/scripts 编译检查通过，无输出（日志 0 字节）；退出码副本 05-final-r2-compileall.exit（内容 0）。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/05-final-r2-compileall.log

### 验证记录：子任务 05 / 全仓 ty（r2 重跑）
- **时间**：2026-09-20
- **子任务/门禁**：子任务 05
- **命令**：`uv run --all-packages ty check .`
- **退出码**：0
- **摘要**：`All checks passed!`（修复后重新执行）；退出码副本 05-final-r2-ty.exit（内容 0）。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/05-final-r2-ty.log

### 验证记录：子任务 05 / diff check
- **时间**：2026-09-20
- **子任务/门禁**：子任务 05
- **命令**：`git diff --check`
- **退出码**：0
- **摘要**：无空白错误；仅保留既有 `LF will be replaced by CRLF` 换行提示（19 条，与本次验证无关）；退出码副本 05-final-r2-diff.exit（内容 0）。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/05-final-r2-diff.log

### 验证记录：子任务 05 / 删除后引用扫描
- **时间**：2026-09-20
- **子任务/门禁**：子任务 05
- **命令**：`uv run --package suspension-multibody python .codex-tasks/20260920-unified-preparation-cutover/tasks/04-delete-legacy/legacy_reference_scan.py --post-delete`
- **退出码**：0
- **摘要**：无输出；post-delete 断言目标文件不存在且交付目录无旧模块 import/路径引用（日志 0 字节）；退出码副本 05-final-r2-post-delete-scan.exit（内容 0）。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/05-final-r2-post-delete-scan.log

### 验证记录：子任务 05 / 架构契约扫描
- **时间**：2026-09-20
- **子任务/门禁**：子任务 05
- **命令**：`uv run --package suspension-multibody python .codex-tasks/20260920-unified-preparation-cutover/tasks/04-delete-legacy/architecture_contract_scan.py`
- **退出码**：0
- **摘要**：`decode_result=results/decoder.py`、`native_submission=simulation/backend.py:24`；统一解码定义唯一、native 提交点唯一；退出码副本 05-final-r2-architecture-scan.exit（内容 0）。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/05-final-r2-architecture-scan.log

### 验证记录：子任务 05 / 唯一 decode_result AST 检查（SUBTASKS.csv 原子命令）
- **时间**：2026-09-20
- **子任务/门禁**：子任务 05
- **命令**：`python -c "import ast; from pathlib import Path; root=Path('packages/suspension_multibody/src/suspension_multibody'); defs=[]; [defs.append(str(p.relative_to(root)).replace('\\\\','/')) for p in root.rglob('*.py') for n in ast.parse(p.read_text(encoding='utf-8')).body if isinstance(n,(ast.FunctionDef,ast.AsyncFunctionDef)) and n.name=='decode_result']; assert defs==['results/decoder.py'], defs"`
- **退出码**：0
- **摘要**：CSV 原样命令（保留转义）经 bash 执行 exit 0、无输出，断言 `defs==['results/decoder.py']` 成立；补充 sanity 打印 `defs= ['results/decoder.py']`，确认全仓恰有一个 `decode_result` 定义；日志 0 字节，退出码副本 05-final-r2-ast-decode-result.exit（内容 0）；命令脚本 05-final-r2-ast-command.sh，sanity 输出 05-final-r2-ast-sanity.log。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/05-final-r2-ast-decode-result.log

## 05 收口完成：当前恢复入口（5/5 DONE）

- 05 前四步（TODO 1-4）全部不同原子命令已逐条独立执行并即时写入 子任务 05 记录：专项全量 714 passed/1 skipped/1 xfailed（677.06s）；核心范围 177 passed/1 xfailed；SPEC architecture/cli/simulation 103 passed；artifact+Adams 12 passed；历史兼容+artifact 24 passed；ruff/compileall/ty/diff 全 exit0；post-delete 引用扫描与 architecture 契约扫描 exit0；SUBTASKS AST 唯一 `decode_result` 命令 exit0（`defs=['results/decoder.py']`）。
- 文档运行缺陷最小修复（主线程，代码冻结）与红绿证据（A-E）已入本文件「05 文档运行缺陷修复与红绿证据」段；原 `05-final-full-pytest.log` 为修复前被中止运行（仅 20%），不计通过。
- 收口：05 第 5 步与 SUBTASKS 第 5 行已置 DONE（含 completed_at/notes，TODO 第 5 步 retry_count=1），05 child PROGRESS 5/5，Epic 状态 DONE、恢复信息 5/5；`--final` 与 `git diff --check` 独立执行结果见本文件末尾 05 收口记录；历史失败/中断记录（被中止的 `05-final-full-pytest.log`、首轮 `--final-preclose` exit1）全部保留，无 git 提交。

## 05 收口预检（首轮 exit 1）与目标核验收敛

- 主线程在代码/测试冻结后执行 `python .codex-tasks/20260920-unified-preparation-cutover/tasks/05-final-validation/planning_contract_scan.py --final-preclose`，退出码 1。首因：`_assert_preclose_state()` → `_assert_task_records(5, rows=done_rows)` → `_assert_evidence()`（planning_contract_scan.py:190）把 12 条 r2 记录「证据文件」字段里附带的「（退出码副本 ….exit）」说明当成路径的一部分，`Path.is_file()` 为假。失败命令、原因与原始日志完整保留，该次运行不计为预收口通过。
- 修复：仅把「证据文件」收敛为真实存在的单个日志路径，退出码副本/命令脚本/sanity 说明移入同一条记录的「摘要」；未新增或删除记录，未改动审计脚本、审计规则与任何退出码，未改写历史失败/中断记录（含被中止的 `05-final-full-pytest.log` 与首轮 `--final-preclose` 失败）。
- 目标核验与复审收敛：六项 Epic Goal 的代码/测试证据逐项列于主线程核验文档 `$PI_SCRATCH_DIR/05-goal-review.md`（统一 preparation 协议/registry；staged/facade 同顺序、单次提交；七个 family 真实集成；旧整车职责完整归属；调用方迁移与旧模块删除；ABI/单位/异常/partial/artifact/历史兼容）。独立核验代理 cb5812cf 对六项目标 PASS；窄范围复审 5ef24a10 确认 `results/decoder.py` 最小类型判断修复无阻断，原有 typed 路径与 raw 回退语义保持。当前专项全包 714 passed/1 skipped/1 xfailed，ruff、compileall、全仓 ty、git diff --check、post-delete 引用扫描、architecture 契约扫描与 AST 唯一 `decode_result` 检查全部 exit 0，无未关闭阻断项。
- 已知限制（非阻断）：仓库无真实历史 artifact fixture，历史 `DynamicResultBundle` 兼容结论限于 schema 合法 JSON/YAML 样例经公开 loader 往返及非法版本/字段严格拒绝，未以跳过或用例替身掩盖。

### 验证记录：子任务 05 / 终局预收口首轮失败（exit 1）
- **时间**：2026-09-20
- **子任务/门禁**：子任务 05
- **命令**：`python .codex-tasks/20260920-unified-preparation-cutover/tasks/05-final-validation/planning_contract_scan.py --final-preclose`
- **退出码**：1
- **摘要**：`_assert_evidence` 把证据路径后的「（退出码副本 05-final-r2-full-package.exit）」当文件名，`Path.is_file()` 断言失败（planning_contract_scan.py:190，经 314→268 调用）；审计规则、退出码与失败日志均未改动，本失败不计为通过。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/05-final-preclose.log

### 验证记录：子任务 05 / 目标核验与复审收敛
- **时间**：2026-09-20
- **子任务/门禁**：子任务 05
- **命令**：`python "C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/05-goal-review-check.py"`
- **退出码**：0
- **摘要**：`goal_review_rows=6 reviewers=2 pass=1 historical_fixture_limit=noted`；六项 Goal 证据行齐备，cb5812cf/5ef24a10 复审 PASS 结论与「无真实历史 fixture」限制均在该文档中，当前全包与全部门禁成功、无未关闭阻断。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/05-goal-review-check.log

### 验证记录：子任务 05 / 终局预收口重跑（r2 通过）
- **时间**：2026-09-20
- **子任务/门禁**：子任务 05
- **命令**：`python .codex-tasks/20260920-unified-preparation-cutover/tasks/05-final-validation/planning_contract_scan.py --final-preclose`
- **退出码**：0
- **摘要**：`planning_contract_scan=pass`；01-04 DONE、05 前四步 DONE 且有真实证据，矩阵、阶段记录（04-pre-delete/04-post-delete）与逐原子命令成功记录检查全部通过；证据字段收敛后独立重跑，退出码副本 05-final-preclose-r2.exit（内容 0）。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/05-final-preclose-r2.log

### 验证记录：子任务 05 / 终局全量状态与证据核对（--final）
- **时间**：2026-09-20
- **子任务/门禁**：子任务 05
- **命令**：`python .codex-tasks/20260920-unified-preparation-cutover/tasks/05-final-validation/planning_contract_scan.py --final`
- **退出码**：0
- **摘要**：`planning_contract_scan=pass`；SUBTASKS 五行全 DONE 且 completed_at/notes 齐备、EPIC 状态为 DONE、05 TODO 五行 DONE，01-05 逐原子命令成功记录与证据文件、矩阵及 04-pre-delete/04-post-delete 阶段记录全部通过；退出码副本 05-final-audit.exit（内容 0）。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/05-final-audit.log

### 验证记录：子任务 05 / 收口后 diff check
- **时间**：2026-09-20
- **子任务/门禁**：子任务 05
- **命令**：`git diff --check`
- **退出码**：0
- **摘要**：无 whitespace error；输出 20 行全部为既有 `LF will be replaced by CRLF` 换行提示，本次记录编辑未引入空白问题；退出码副本 05-final-close-diff.exit（内容 0）。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/05-final-close-diff.log

### 验证记录：子任务 05 / 收口记录写入后复核（--final 复跑）
- **时间**：2026-09-20
- **子任务/门禁**：子任务 05
- **命令**：`python .codex-tasks/20260920-unified-preparation-cutover/tasks/05-final-validation/planning_contract_scan.py --final`
- **退出码**：0
- **摘要**：`planning_contract_scan=pass`；在 `--final` 与 diff check 记录写入后复跑同一终局核对，确认新增记录未破坏状态与证据契约；退出码副本 05-final-audit-r2.exit（内容 0）。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/05-final-audit-r2.log

### 验证记录：子任务 05 / 收口记录全部写入后终局复核（--final 三跑）
- **时间**：2026-09-20
- **子任务/门禁**：子任务 05
- **命令**：`python .codex-tasks/20260920-unified-preparation-cutover/tasks/05-final-validation/planning_contract_scan.py --final`
- **退出码**：0
- **摘要**：`planning_contract_scan=pass`；在首轮预收口失败记录、目标核验收敛记录、`--final` 记录、diff check 记录与复跑记录全部写入后再运行同一终局核对，确认记录集合完整且状态/证据契约仍成立；退出码副本 05-final-audit-r3.exit（内容 0）。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/05-final-audit-r3.log

### 验证记录：子任务 05 / 收口后 diff check（复跑，末次）
- **时间**：2026-09-20
- **子任务/门禁**：子任务 05
- **命令**：`git diff --check`
- **退出码**：0
- **摘要**：在全部收口记录写入后复跑，无 whitespace error；20 行输出全部为既有 `LF will be replaced by CRLF` 换行提示，记录编辑未引入空白问题；退出码副本 05-final-close-diff-r2.exit（内容 0）。
- **证据文件**：C:/Users/zzy11/.pi-desktop/scratch/5d217c5d-208c-4ebd-a11c-39e7f374b599/05-final-close-diff-r2.log
