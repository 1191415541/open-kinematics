- 任务：迁移剩余调用方并删除旧模块、同步文档与打包
- 形态：single-full（Epic 子任务）
- 进度：0/7 步骤，TODO
- 当前：未执行（规划已建立，实施未开始）
- 文件：`.codex-tasks/20260921-architecture-deviation-closure/tasks/20260921-08-delete/`
- 验证：未运行

## 恢复信息

前置：01 的 `SYMBOL_MATRIX.csv` 与 `VALIDATION.md` 已冻结；02 的删除门禁与扫描器模式就绪；03–07 已完成全部迁移且旧调用方清零。旧调用方未清零时不得开工本任务。

本任务删除 `core`/`elements`/`model`/`analysis`/`metrics` 旧目录与旧顶层 `pac2002_scope.py`，改造旧物理测试为 native 契约测试，同步当前文档与打包，并在本任务目录交付扫描器脚本 `legacy_reference_scan.py`。

下一步：步骤 1 核对迁移矩阵闭合，步骤 2 删除前全仓扫描，步骤 3 删除旧目录与转发壳及无生产调用的求解实现，步骤 4 改造旧物理测试，步骤 5 同步文档与迁移说明，步骤 6 删除后扫描复跑与打包内容检查，步骤 7 汇总回填。

删除前后扫描必须留档；`.codex-tasks` 历史任务目录只读不改。raw/ 规划阶段为空；实施期可归档的证据放 raw/，中间日志放会话 scratch。
