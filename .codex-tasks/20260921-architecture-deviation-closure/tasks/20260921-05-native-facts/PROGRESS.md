- 任务：接管 Python 元件报告事实、静轮荷辅助求解与凝聚等价性登记（A1 修订：原「计算性凝聚」口径已改）
- 形态：single-full（Epic 子任务）
- 进度：2/8 步骤 DONE（步骤 1 对照表、步骤 2 凝聚等价性登记）
- 当前：步骤 3（按缺口补默认关闭的可选 native 输出通道）
- 文件：`.codex-tasks/20260921-architecture-deviation-closure/tasks/20260921-05-native-facts/`
- 验证：步骤 1-2 全集通过（见 raw/step2_condensation_equivalence.md 的证据表）

## 恢复信息

开工前必须核验：01 父级 VALIDATION.md 已冻结（DONE）、02 门禁就绪（DONE）、03/04 模块职责拆分已验收（DONE）。**前置全部满足，05 可启动。**

本任务写 `results/**`、`api.py` 的元件报告取值来源、`analysis/vehicle_physics.py` 的静轮荷迁出，以及必要的 C++ 输出通道与向后兼容契约字段；不建 `report`、不改 `preparation`。

三步顺序固定：① 凝聚等价性登记（A1 修订：凝聚保留 Python 作者层，native fixed 关节作契约等价实现 + body ID 映射登记，**不迁入 mb_assembly**）② native 输出与静轮荷 ③ decoder 接线；每步单独构建并跑通道级容差与数值门，禁止一次积累全任务 diff 后再验证。

下一步：前置已全部满足，可执行步骤 1（已交付字段对照表）→步骤 2 凝聚等价性登记→步骤 3 按缺口补可选输出通道→步骤 4 迁静轮荷→步骤 5 接 decoder→步骤 6 通道验收→步骤 7 差异处理→步骤 8 汇总。每行除专项命令外必须执行 SPEC 验证协议全集及父 VALIDATION.md 冻结门禁。

旧 Python 路径（`elements/elastic.py`、`elements/assembly.py`）按 EPIC G3 修订在可选通道未启用期间保留，且不得为删除本构而启用可选通道；不得让两套力律同时通过验收。raw/ 已有步骤 1 交付物 `step1_channel_mapping.md`（字段↔通道逐项对照表）；其余实施期证据放 raw/，中间日志放会话 scratch。
**A1 相关口径**（详见父 EPIC 修订记录与父 PROGRESS）：本任务新增的 native 力旋量通道为**默认关闭的可选扩展**，默认路径 artifact 字节不得变化；`dynamic_hash_sentinel` 与 `case_parity_check` 的字节级门必须保持绿，且不得重录任何基线。契约版本允许 1→2，ABI 七符号与 ABI 版本号（15/30/1）不变。
