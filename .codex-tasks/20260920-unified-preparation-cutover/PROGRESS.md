# Epic 进度：统一 preparation 生命周期与整车旧模块删除

- **任务**：统一多体 preparation 入口并删除整车旧模块
- **形态**：epic
- **状态**：TODO
- **进度**：0/5 子任务完成
- **当前**：规划结构化验收和门禁脚本修订完成，等待最终独立规划审核
- **文件**：`.codex-tasks/20260920-unified-preparation-cutover/`

## 恢复入口

恢复时依次读取：

1. `EPIC.md`
2. `SUBTASKS.csv`
3. 本文件
4. 当前子任务目录下的 `SPEC.md`、`TODO.csv`、`PROGRESS.md`

子任务必须按 `01 → 02 → 03 → 04 → 05` 串行推进；源码实现前规划审核必须为 `PASS`。

## 当前阻断项

- 最新独立规划审核曾因验收命令弱校验返回 `FAIL`；已补充 `planning_contract_scan.py`，逐行验证矩阵、CSV 状态、PROGRESS 记录字段、数值退出码和终局证据，当前等待最终独立规划审核。
- 规划二审通过前不得把任何子任务改为 `IN_PROGRESS`。

## 子任务状态

| id | task | status | latest evidence |
|---|---|---|---|
| 1 | 统一 preparation 协议与 runner 生命周期 | TODO | 规划审核未通过 |
| 2 | 迁移六个非 vehicle_dynamic family 的装配前处理 | TODO | 依赖任务 01 |
| 3 | 拆分 vehicle_dynamics 职责并接入 vehicle_dynamic preparation | TODO | 依赖任务 02 |
| 4 | 迁移调用方并删除旧模块 | TODO | 依赖任务 03 |
| 5 | 终局回归与 Epic 收口 | TODO | 依赖任务 04 |

## 验证记录格式

每次执行子任务或终局门禁后，在本文件追加一条 `### 验证记录`，必须使用以下字段：

### 验证记录：<子任务或门禁>
- **时间**：执行时间；
- **子任务/门禁**：对应 `SUBTASKS.csv` 行或删除前/删除后阶段；
- **命令**：完整命令，不用“同上”代替；
- **退出码**：实际退出码；
- **摘要**：通过数量、关键断言或失败首因；
- **证据文件**：指向 `tasks/<id>/raw/` 中的原始输出，若无独立文件写明 `无`。

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

### 验证记录：规划门禁 01-05 / Diff 检查
- **时间**：2026-09-20
- **子任务/门禁**：规划门禁 01-05 / Diff 检查
- **命令**：`git diff --check`
- **退出码**：0
- **摘要**：无 diff whitespace 错误；仅报告工作区既有 LF/CRLF 转换警告。
- **证据文件**：无
