# 领域契约与唯一解析后模型

- 形态：single-full；父任务：../../EPIC.md；依赖：无。
- 责任范围：modeling/、compilation/model_view.py、contracts schema；新增迁移清单脚本和 IR 测试。
- 步骤 1 先实现只读库存脚本并采集当前证据，再允许步骤 2 修改产品源码。输入、输出与失败记录契约见 ../../planning/interfaces.md 第 1 节。
- 交付与验证：以 TODO.csv 各叶级步骤为准；新测试属于本子任务交付，路径清单见 planning/validation.md。
- 约束：继承 EPIC、设计方案和 AGENTS；无新依赖，无兼容生产fallback，无隐式业务实体创建，不重录基线。
- 本子任务实施前保存相关当前代码与证据；准备删除前须有真实消费者清单和物理等价验证。
- 代码验证附加：ruff/ty、结构门；触及求解按 validation.md 运行数值门。与 native/contracts 的 pytest 分开调用。
- DONE：全部叶步骤验证通过，记录退出码、断言、失败/修复及适用门；父 CSV 同步，不把计划审核当作实施完成。
