# 子任务 04：迁移调用方并删除旧模块

## 目标

在新 preparation、results 和 vehicle service 归属稳定、所有生产和测试调用方迁移后，先通过删除前门禁，再删除 `vehicle_dynamics.py` 并执行删除后扫描。

## 交付范围

- 迁移顶层 `__init__.py`、`api.py`、CLI、Adams、scripts、docs、CI/配置和 architecture tests 中的旧模块导入或路径引用；不接管 family preparation/compiler/results 文件。
- 新增 `packages/suspension_multibody/tests/schema/test_dynamic_result_compat.py`，明确验证历史 `DynamicResultBundle` 读取边界；不删除历史 artifact 兼容。
- 维护 `.codex-tasks/20260920-unified-preparation-cutover/tasks/04-delete-legacy/legacy_reference_scan.py`，统一覆盖 packages、docs、`.github`、根目录 scripts、README、配置和常见脚本/文档后缀；显式排除 `.codex-tasks`、缓存和构建产物，覆盖点号模块路径、Unix/Windows 文件路径、相对/包属性 import 形式，并提供可失败的 pre-delete/post-delete 模式。
- 删除 `packages/suspension_multibody/src/suspension_multibody/vehicle_dynamics.py`。
- 删除前后都运行同一 helper 和职责清单核对，确认不存在旧模块 import、路径引用、失效文档命令或未迁移的旧职责；删除前 helper 必须证明目标存在，删除后必须证明目标不存在。

## 约束

- 不删除历史 artifact 读取兼容，不删除仍有用户价值且已迁移到新归属的公开结果/异常语义。
- 不修改 family preparation/compiler/results 的实现和 native 内核；本任务只迁移调用方、公共边界、文档、门禁和删除旧文件。
- 删除必须发生在删除前门禁全部通过之后；门禁失败时保留旧文件并修复调用方。

## 验收标准

1. 删除前门禁中的功能、历史兼容和静态检查全部通过，并在父级 `PROGRESS.md` 记录每条命令的退出码和摘要。
2. 目标文件已删除；交付目录中无旧模块 import、路径引用或失效文档命令，post-delete helper 以非零退出暴露任一命中。
3. `PREPARATION_MATRIX.md` 的旧模块完整职责清单均已核对；顶层导出、API、CLI、Adams、scripts 和 architecture tests 均指向新归属与统一 preparation/runner 生命周期。
4. 删除后扫描、历史 `DynamicResultBundle` 读取和 `git diff --check` 通过。
## 删除前门禁

调用方迁移完成后，删除文件前必须依次通过以下每条独立命令；删除动作是这些命令全部成功后的单独文件操作，不能嵌入删除前门禁命令。

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

删除目标文件后重新执行以下门禁；扫描必须额外断言目标文件不存在：

```bash
uv run --package suspension-multibody pytest packages/suspension_multibody/tests/vehicle packages/suspension_multibody/tests/results packages/suspension_multibody/tests/cases/test_vehicle_dynamic_contract.py packages/suspension_multibody/tests/adams packages/suspension_multibody/tests/cli packages/suspension_multibody/tests/architecture packages/suspension_multibody/tests/schema/test_dynamic_result_compat.py packages/suspension_multibody/tests/io/test_artifacts_unified.py -q
uv run --package suspension-multibody python .codex-tasks/20260920-unified-preparation-cutover/tasks/04-delete-legacy/architecture_contract_scan.py
uv run --package suspension-multibody python .codex-tasks/20260920-unified-preparation-cutover/tasks/04-delete-legacy/legacy_reference_scan.py --post-delete
git diff --check
```
