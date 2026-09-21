- 任务：接管 Python 元件报告事实、静轮荷辅助求解与计算性凝聚
- 形态：single-full（Epic 子任务）
- 进度：0/8 步骤，TODO
- 当前：未执行（规划已建立，实施未开始）
- 文件：`.codex-tasks/20260921-architecture-deviation-closure/tasks/20260921-05-native-facts/`
- 验证：未运行

## 恢复信息

开工前必须核验：01父级VALIDATION.md已冻结，02门禁就绪，03/04模块职责拆分已验收。目前这些前置任务均TODO，尚未完成。

本任务写 `results/**`、`api.py` 的元件报告取值来源、`analysis/vehicle_physics.py` 的静轮荷迁出，以及必要的 C++ 输出通道与向后兼容契约字段；不建 `report`、不改 `preparation`。

三步顺序固定：① 凝聚与模型映射 ② native 输出与静轮荷 ③ decoder 接线；每步单独构建并跑通道级容差与数值门，禁止一次积累全任务 diff 后再验证。

下一步：先核验前置，再执行步骤1冻结字段表、步骤2完成凝聚映射、步骤3补输出字段、步骤4迁静轮荷、步骤5接decoder、步骤6通道验收、步骤7处理差异、步骤8汇总。每行除专项命令外必须执行SPEC验证协议全集及父VALIDATION.md冻结门禁。

旧 Python 路径（`elements/elastic.py`、`elements/assembly.py`）保留到 06 切换，本任务不得提前删除，也不得让两套力律同时通过验收。raw/ 规划阶段为空；实施期可归档的证据放 raw/，中间日志放会话 scratch。
