- 任务：拆解 mb_vehicle 并落地 assembly/force/element/tire 的职责归属
- 形态：single-full（Epic 子任务）
- 进度：0/7 步骤，TODO
- 当前：未执行（规划已建立，实施未开始）
- 文件：`.codex-tasks/20260921-architecture-deviation-closure/tasks/20260921-04-ownership/`
- 验证：未运行

## 恢复信息

前置：01 的父级 `VALIDATION.md` 已冻结并实测通过；02 门禁就绪；03 已完成基础层拆分并冻结 `CMakeLists.txt`、`MODULES.md`、`layering_baseline.json`（三个共享文件必须串行，本任务不得与 03 并行）。

本任务写 `cpp/include/{mb_assembly,mb_force,mb_element,mb_model,mb_tire,mb_joint}` 与 `cpp/src/{vehicle,suspension,tire,model,abi}`，并同步构建清单、模块表、分层基线与测试硬编码；Python 生产代码本任务不改。

下一步：步骤 1 冻结 `mb_vehicle` 逐符号归属表，步骤 2–3 建立 `mb_assembly` 并迁移注册函数、element reader、`build_model` 与 `audit_constraint_system`，步骤 4 建立 `mb_element`，步骤 5 建立 `mb_force`，步骤 6 把轮胎语义归 `mb_tire`，步骤 7 完成 `mb_model` 纯数据化与凝聚归属并清空 `mb_vehicle`。

每步必须"构建 + 架构门 + 数值门"三过再进下一步；数值漂移立即停止该步。raw/ 规划阶段为空；实施期可归档的证据放 raw/，中间日志放会话 scratch。
