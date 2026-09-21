# 子任务 08：迁移剩余调用方并删除旧模块、同步文档与打包

## 目标

在 06/07 已把生产调用方切走之后，删除旧归属并完成收尾：

1. **（2026-09-21 用户裁决 A1 修订）** 删除 `core`/`elements`/`model`/`analysis`/`metrics` 旧目录与旧顶层 `pac2002_scope.py` 中**已无生产调用者**的部分，不留旧路径转发壳、不重复 native 提交、不留第二统一 decoder。仍有现役生产调用的模块（按 EPIC G3 修订保留的力元本构等）**不删**，但须在删除记录中逐项列明保留理由与解除条件。原「必须删完」不再作为完成条件。
2. 删除 `core/rank.py`、`core/reactions.py`、`model/mass.py` 等无生产调用的求解实现；保留其中有价值的物理断言并转成 native 测试，逐项登记覆盖关系。
3. 旧物理测试改造成 native 契约测试；测试覆盖迁移矩阵逐项闭合。
4. 删除前后各做一次全仓扫描（源码、脚本、测试、配置、当前文档；排除历史任务记录与生成目录），扫描器支持相对 import、别名 import 与动态 import。
5. 同步当前文档与打包：`packages/suspension_kernel/MODULES.md`、根 `CONTEXT-MAP.md`、`README.md`、`packages/suspension_multibody/CONTEXT.md`、迁移说明；不改历史任务记录。

## 非目标

- 不新增功能、不重构 06/07 已定稿的 `preparation`/`report`/`results` 边界。
- 不删除测试目录本身：`tests/metrics`、`tests/analysis`、`tests/results`、`tests/cli` 等稳定验证命令保留。
- 不改历史任务记录（`.codex-tasks/20260917-*`、`20260919-*`、`20260920-*`、`architecture-deviation-closure` 旧目录）。
- 不为了让扫描通过而放宽扫描范围或加豁免名单。

## 约束

- 删除前必须先完成扫描与迁移矩阵闭合核对；未闭合符号禁止删除。
- 删除后必须复跑扫描与门禁；**已无生产调用者**的旧模块、旧导入、转发壳残留数为零才算完成；仍有现役生产调用的模块按第 1 条保留并逐项登记理由。
- 扫描器是本任务交付物，放在 `tasks/20260921-08-delete/` 下，排除历史任务记录与生成目录（`build/`、`dist/`、`__pycache__/`、`.venv/`）。
- wheel 打包检查只看内容归属（文件清单与元数据不含旧模块）；安装后的端到端隔离验证属 09。

## 范围与文件归属

- 可写：`packages/suspension_multibody/src/suspension_multibody/**`（删除旧目录与残留调用面）、`packages/suspension_multibody/tests/**`（改造与归属调整）、`packages/suspension_multibody/scripts/**`、`packages/suspension_kernel/{MODULES.md,README.md}`、根 `CONTEXT-MAP.md`、`README.md`、`docs/**`、本任务目录下的扫描器脚本。
- 只读：`.codex-tasks/20260917-*`、`.codex-tasks/20260919-*`、`.codex-tasks/20260920-*`、`.codex-tasks/architecture-deviation-closure`（旧目录，只读不改）、父 `EPIC.md`、`VALIDATION.md`、`tasks/20260921-01-baseline/SYMBOL_MATRIX.csv`。
- 不写：父 `EPIC.md`、`SUBTASKS.csv`、`PROGRESS.md`。

## 依赖

- 前置：01（`SYMBOL_MATRIX.csv` 与 `VALIDATION.md`）、02（删除门禁与扫描器模式）、03–07（全部迁移完成，旧调用方清零）。
- 后续：09 依赖本任务的删除后零残留证据。

## 验收标准

1. `SYMBOL_MATRIX.csv` 每行三态之一闭合**（A1 修订，原为两态）**：①已迁移并指向新归属；②有明确的删除依据；③**保留**（按 A1 仍有现役生产调用，须记 file:line + 阻断原因 + 解除条件）。无未闭合行。
2. 删除前与删除后两次全仓扫描结果留档；扫描器覆盖相对 import、别名 import、动态 import，且排除历史任务记录与生成目录。
3. **（A1 修订）** `core`/`elements`/`model`/`analysis`/`metrics` 与 `pac2002_scope.py` 中**已无生产调用者**的部分不存在，无转发壳、无重复 native 提交、无第二统一 decoder；仍有现役生产调用的部分保留且有登记。
4. 被删除求解实现的物理断言已转为 native 契约测试，并逐项登记覆盖关系（原断言→新测试）。
5. `tests/architecture`、`tests/contract`、`tests/io`、`tests/schema` 全绿；无新增失败。
6. `MODULES.md`、`CONTEXT-MAP.md`、`README.md`、`CONTEXT.md` 与迁移说明反映最终模块与目录结构；历史任务记录未被修改。
7. wheel/sdist 文件清单与元数据不含旧模块归属；打包成功且不依赖旧目录。

## 验证协议

```bash
uv run --package suspension-multibody pytest packages/suspension_multibody/tests/architecture packages/suspension_multibody/tests/contract packages/suspension_multibody/tests/io packages/suspension_multibody/tests/schema -q
uv run --package suspension-kernel pytest packages/suspension_kernel/tests -q
uv run python packages/suspension_kernel/scripts/check_module_layering.py --strict
uv build --package suspension-kernel
uv build --package suspension-multibody
git diff --check
```

删除前与删除后都必须运行同一扫描命令并留档；删除后残留数不为零则本任务未完成。
