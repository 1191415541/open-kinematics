# 本轮：计划及全部子任务SPEC落盘

- 形态：single-full，Epic规划活动，不是产品实施子任务。
- 目标：按用户指定完整设计编写可恢复、可执行、可验证的Epic及全部SPEC/TODO/PROGRESS。
- 输入：用户请求、AGENTS、taskmaster v5、设计全文与定点源码证据。
- 范围：只写本Epic目录，不改设计/产品代码/测试/依赖/旧任务。
- 交付：EPIC/SUBTASKS/PROGRESS、9子任务三件套、REVIEW、证据索引与规划记录。
- 验收：设计覆盖、依赖DAG无环、P4b/P7不成为核心前置，子SPEC完整，CSV叶子/恢复/链接一致，独立审查阻断清零，产品状态全TODO。
- 验证：CSV/依赖/链接/来源指纹自检、独立只读计划审查、git diff --check与仓库check-fast。本轮不触物理求解，不跑数值或慢测试。
- 完成规划不等于完成产品任务，也不授权商业服务/数据生成。
