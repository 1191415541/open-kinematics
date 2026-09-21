- 任务：执行独立终局验收并逐条核对 G1–G4
- 形态：single-full（Epic 子任务）
- 进度：0/7 步骤，TODO
- 当前：未执行（规划已建立，实施未开始）
- 文件：`.codex-tasks/20260921-architecture-deviation-closure/tasks/20260921-09-acceptance/`
- 验证：未运行

## 恢复信息

前置：01–08 全部完成且各自门禁通过；`VALIDATION.md`、`tasks/20260921-01-baseline/SYMBOL_MATRIX.csv`、`tasks/20260921-08-delete/legacy_reference_scan.py` 与删除后零残留证据齐备。

本任务只读生产代码与既有交付物，不修代码、不改测试与文档；发现缺口退回对应子任务。

下一步：步骤 1 执行父 EPIC 十条终局命令并逐条记录退出码，步骤 2–5 分别核对 G1–G4，步骤 6 在会话 scratch 的隔离环境验证 wheel，步骤 7 列明既有失败并回填父级状态。

结论必须区分「Goal 达成」与「子任务 DONE」；数值门、ABI 七符号与通道级容差按 01 的 `VALIDATION.md` 原文执行。raw/ 规划阶段为空；实施期可归档的证据放 raw/，中间日志放会话 scratch。
