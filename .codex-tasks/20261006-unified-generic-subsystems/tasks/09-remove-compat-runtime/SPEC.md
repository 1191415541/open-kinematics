# 单次生产切换与兼容执行退役

- 形态：single-full；父任务：../../EPIC.md；依赖：2;3;4;5;6;7;8。
- 责任范围：api/CLI、presets.legacy、subsystems 专用装配/rig_link、preparation、Adams导入/脚本/公开导出；AST退役门及离线迁移器。
- 步骤 2 的库存核对、数值/性能和单一路径退出门必须全部通过才允许步骤 3 切换删除。未通过时记录 FAILED 并修复，不能移除模块或旁路断言。有效通用定义先搬到已存在的低层职责模块并验证调用迁移。
- 交付与验证：以 TODO.csv 各叶级步骤为准；新测试属于本子任务交付，路径清单见 planning/validation.md。
- 约束：继承 EPIC、设计方案和 AGENTS；无新依赖，无兼容生产fallback，无隐式业务实体创建，不重录基线。
- 本子任务实施前保存相关当前代码与证据；准备删除前须有真实消费者清单和物理等价验证。
- 代码验证附加：ruff/ty、结构门；触及求解按 validation.md 运行数值门。与 native/contracts 的 pytest 分开调用。
- DONE：全部叶步骤验证通过，记录退出码、断言、失败/修复及适用门；父 CSV 同步，不把计划审核当作实施完成。
