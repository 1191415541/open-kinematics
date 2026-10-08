# 通用关节坐标与车轮自转边界

- 形态：single-full；父任务：../../EPIC.md；依赖：3;4。
- 责任范围：contracts joint-coordinate/motion schema、authoring 坐标解析、native constraint/init/directional/output、Rig/Case 边界测试。
- 交付与验证：以 TODO.csv 各叶级步骤为准；新测试属于本子任务交付，路径清单见 planning/validation.md。
- 约束：继承 EPIC、设计方案和 AGENTS；无新依赖，无兼容生产fallback，无隐式业务实体创建，不重录基线。
- 本子任务实施前保存相关当前代码与证据；准备删除前须有真实消费者清单和物理等价验证。
- 代码验证附加：ruff/ty、结构门；触及求解按 validation.md 运行数值门。与 native/contracts 的 pytest 分开调用。
- DONE：全部叶步骤验证通过，记录退出码、断言、失败/修复及适用门；父 CSV 同步，不把计划审核当作实施完成。
