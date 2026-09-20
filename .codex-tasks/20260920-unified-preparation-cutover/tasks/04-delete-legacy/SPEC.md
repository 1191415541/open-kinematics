# 子任务 04：迁移调用方并删除旧模块

## 目标

在新 preparation、results 和 vehicle service 归属稳定、所有生产和测试调用方迁移后，先通过删除前门禁，再删除 `vehicle_dynamics.py` 并执行删除后扫描。

## 交付范围

- 迁移顶层 `__init__.py`、`api.py`、CLI、Adams、scripts、docs、CI/配置和 architecture tests 中的旧模块导入或路径引用；不接管 family preparation/compiler/results 文件。
- 新增 `packages/suspension_multibody/tests/schema/test_dynamic_result_compat.py`，明确验证历史 `DynamicResultBundle` 读取边界；不删除历史 artifact 兼容。
- 历史读取预检已复现 `schema/loader.py::_read` 要求顶层版本、`DynamicResultBundle` 却只允许 manifest 版本的矛盾；本子任务负责 loader 的最小修复。先用合法历史 JSON 经 `load_dynamic_result` 形成失败回归，再读取 manifest 版本完成兼容，保持其它 model/case 的顶层版本检查、严格 schema 和结果格式不变。
- 维护 `.codex-tasks/20260920-unified-preparation-cutover/tasks/04-delete-legacy/legacy_reference_scan.py`，统一覆盖 packages、docs、`.github`、根目录 scripts、README、配置和常见脚本/文档后缀；显式排除 `.codex-tasks`、缓存和构建产物，覆盖点号模块路径、Unix/Windows 文件路径、相对/包属性 import 形式，并提供可失败的 pre-delete/post-delete 模式。
- 删除 `packages/suspension_multibody/src/suspension_multibody/vehicle_dynamics.py`。
- 删除前后都运行同一 helper 和职责清单核对，确认不存在旧模块 import、路径引用、失效文档命令或未迁移的旧职责；删除前 helper 必须证明目标存在并写入 `子任务 04-pre-delete` 记录，删除后必须证明目标不存在并写入 `子任务 04-post-delete` 记录。

## 约束

- 不删除历史 artifact 读取兼容，不删除仍有用户价值且已迁移到新归属的公开结果/异常语义。
- 不修改 family preparation/compiler/results 的实现和 native 内核；本任务只迁移调用方、公共边界、文档、门禁和删除旧文件。
- 删除必须发生在删除前门禁全部通过之后；门禁失败时保留旧文件并修复调用方。

## 验收标准

1. 删除前门禁中的功能、历史兼容和静态检查全部通过，并在父级 `PROGRESS.md` 的 `子任务 04-pre-delete` 记录中写明每条命令的完整命令、退出码和摘要。
2. 目标文件已删除；交付目录中无旧模块 import、路径引用或失效文档命令，post-delete helper 以非零退出暴露任一命中。
3. `PREPARATION_MATRIX.md` 的旧模块完整职责清单均已核对；顶层导出、API、CLI、Adams、scripts 和 architecture tests 均指向新归属与统一 preparation/runner 生命周期。
4. 删除后完整校验（核心回归、ruff、compileall、ty、`git diff --check`、architecture scan、post-delete 扫描）、历史 `DynamicResultBundle` 读取通过，并写入父级 `PROGRESS.md` 的 `子任务 04-post-delete` 记录。
5. 合法历史 schema 的 JSON/YAML 经公开 `load_dynamic_result` 读取成功，manifest 版本缺失/非1及未知字段被拒；bundle 顶层 `schema_version` 仍作为未知字段拒绝，不新增 schema 字段或静默丢弃字段。无真实历史fixture时只声明 schema 合法样例往返兼容。
6. `load_model`、`load_case`、`load_dynamic_case`、`load_vehicle_model`、`load_vehicle_dynamic_case` 对缺失/非1顶层版本仍报 `unsupported schema_version`；新增回归覆盖这些反向约束。
7. 先执行新增真实 loader 回归并在父级 `子任务 04-pre-delete` 记录保留失败退出码和日志，再修复并记录成功结果，失败证据不计作删除门禁通过。
## 删除前门禁

调用方迁移完成后、删除文件前必须依次运行以下每条命令，逐条记录真实退出码和证据：每条命令单独执行后立即在父级 `PROGRESS.md` 写入一条 `子任务 04-pre-delete` 验证记录（完整命令、退出码、摘要和存在的证据文件；证据文件允许绝对 scratch 路径或工作区相对路径）；删除动作是这些命令全部退出码为 0 后的单独文件操作，不能嵌入删除前门禁命令，pre-delete 记录缺失或未通过时不得删除。阶段记录的审计不嵌入命令链，由子任务 05 的 `--final-preclose`/`--final` 单独执行。

```bash
uv run --package suspension-multibody pytest packages/suspension_multibody/tests/vehicle packages/suspension_multibody/tests/results packages/suspension_multibody/tests/cases/test_vehicle_dynamic_contract.py packages/suspension_multibody/tests/adams packages/suspension_multibody/tests/cli packages/suspension_multibody/tests/architecture packages/suspension_multibody/tests/schema/test_dynamic_result_compat.py packages/suspension_multibody/tests/io/test_artifacts_unified.py -q
uv run --all-packages ruff check packages/suspension_multibody/src packages/suspension_multibody/tests packages/suspension_multibody/scripts
uv run --all-packages python -m compileall -q packages/suspension_multibody/src packages/suspension_multibody/tests packages/suspension_multibody/scripts
uv run --all-packages ty check .
git diff --check
uv run --package suspension-multibody python .codex-tasks/20260920-unified-preparation-cutover/tasks/04-delete-legacy/legacy_reference_scan.py --pre-delete
uv run --package suspension-multibody python .codex-tasks/20260920-unified-preparation-cutover/tasks/04-delete-legacy/architecture_contract_scan.py
```

## 删除后验证

删除目标文件后必须逐条运行以下完整校验，每条命令单独执行后立即在父级 `PROGRESS.md` 写入一条 `子任务 04-post-delete` 验证记录（完整命令、退出码、摘要和存在的证据文件；证据文件允许绝对 scratch 路径或工作区相对路径）；该完整检查与 `SUBTASKS.csv` 第 4 行 `validation_command` 一致，扫描必须额外断言目标文件不存在：

```bash
uv run --package suspension-multibody pytest packages/suspension_multibody/tests/vehicle packages/suspension_multibody/tests/results packages/suspension_multibody/tests/cases/test_vehicle_dynamic_contract.py packages/suspension_multibody/tests/adams packages/suspension_multibody/tests/cli packages/suspension_multibody/tests/architecture packages/suspension_multibody/tests/schema/test_dynamic_result_compat.py packages/suspension_multibody/tests/io/test_artifacts_unified.py -q
uv run --all-packages ruff check packages/suspension_multibody/src packages/suspension_multibody/tests packages/suspension_multibody/scripts
uv run --all-packages python -m compileall -q packages/suspension_multibody/src packages/suspension_multibody/tests packages/suspension_multibody/scripts
uv run --all-packages ty check .
git diff --check
uv run --package suspension-multibody python .codex-tasks/20260920-unified-preparation-cutover/tasks/04-delete-legacy/architecture_contract_scan.py
uv run --package suspension-multibody python .codex-tasks/20260920-unified-preparation-cutover/tasks/04-delete-legacy/legacy_reference_scan.py --post-delete
```

`--final-preclose` 断言 01-04 已 DONE、05 前 4 步 DONE 且有真实证据，并核对 `子任务 04-pre-delete`/`子任务 04-post-delete` 记录：两个阶段标签必须可识别且不得混用，每个阶段至少有一条退出码为 0、命令包含对应 `legacy_reference_scan.py --pre-delete`/`--post-delete` 标记且带真实证据文件的记录。删除后命令链自身不包含 `--progress-records` 审计，避免“要求执行后才记录”的循环；记录审计由子任务 05 的 `--final-preclose`/`--final` 单独执行。
