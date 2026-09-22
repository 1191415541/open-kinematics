- 任务：接管 Python 元件报告事实、静轮荷辅助求解与凝聚等价性登记（A1 修订：原「计算性凝聚」口径已改）
- 形态：single-full（Epic 子任务）
进度：8/8 步骤 DONE（步骤 1 对照表、2 凝聚等价性登记、3 可选 native 通道、4 静轮荷保留登记、5 decoder 只读解码面、6 逐通道验收、7 力律差异登记、8 汇总）。
当前：子任务完成。native 通道与 Python 事实在三类上不等价（固定体端结构性缺失、承载端力矩差 0.108%–0.287%、K 模式无元件事实），据此**阻断切换**，生产路径保持 Python 本构不变；`report` 未创建、`preparation` 未改动。
文件：`.codex-tasks/20260921-architecture-deviation-closure/tasks/20260921-05-native-facts/`
验证：步骤 1-8 全集通过——构建 0；动态哈希 26/26 逐位一致（组合哈希 `e7407656…8d48e` 未变）；K/C parity 0；8 family 0；kernel 15；contracts 22；套件 263 passed / 1 xfailed；ruff 0；ty 0（见 raw/step5_*.log、step4_*.log 与各步登记文档）。

## 恢复信息

开工前必须核验：01 父级 VALIDATION.md 已冻结（DONE）、02 门禁就绪（DONE）、03/04 模块职责拆分已验收（DONE）。**前置全部满足，05 可启动。**

本任务写 `results/**`、`api.py` 的元件报告取值来源、`analysis/vehicle_physics.py` 的静轮荷迁出，以及必要的 C++ 输出通道与向后兼容契约字段；不建 `report`、不改 `preparation`。

三步顺序固定：① 凝聚等价性登记（A1 修订：凝聚保留 Python 作者层，native fixed 关节作契约等价实现 + body ID 映射登记，**不迁入 mb_assembly**）② native 输出与静轮荷 ③ decoder 接线；每步单独构建并跑通道级容差与数值门，禁止一次积累全任务 diff 后再验证。

下一步：无（本子任务 8/8 DONE）。06 可启动；其第 6 项按 A1 处置为「保留 + 待删除登记」，登记依据见 `raw/step7_law_difference_registration.md` 与 `raw/step6_channel_acceptance.md`。

旧 Python 路径（`elements/elastic.py`、`elements/assembly.py`）按 EPIC G3 修订在可选通道未启用期间保留，且不得为删除本构而启用可选通道；不得让两套力律同时通过验收。raw/ 已有步骤 1 交付物 `step1_channel_mapping.md`（字段↔通道逐项对照表）；其余实施期证据放 raw/，中间日志放会话 scratch。
**A1/A2 相关口径**（详见父 EPIC 修订记录与父 PROGRESS）：本任务新增的 native 力旋量通道为**默认关闭的可选扩展**，默认路径 artifact 字节未变化；`dynamic_hash_sentinel` 与 `case_parity_check` 的字节级门保持绿，未重录任何基线。契约版本允许 1→2（实测关 1 开 2），ABI 七符号与 ABI 版本号（15/30/1）未变。A2 修订：静轮荷求解不迁 native，保留算法本体并登记（`raw/step4_static_wheel_loads_registration.md`）。
