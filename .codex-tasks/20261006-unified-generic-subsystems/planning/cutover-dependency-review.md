# 切换验收依赖复审

原阻断：任务7要求无assembly_kind提前分流，任务8要求删除业务解码分派；任务9依赖7/8，但interfaces.md:18禁止任务9退出门前切换删除。原验收文字产生循环。

修订：任务7/8验收限定为显式新路径就绪且已验证。旧生产分流、业务解码分派及专用实体构建由任务9在全部消费者和物理等价退出门通过后删除。任务9和Epic最终目标不变。

只读探子plan_cutover_dependency_review判定BLOCKED，修订后的plan_cutover_dependency_recheck判定PASS：SUBTASKS.csv:8-10依赖单向；任务7/8不再提前要求9的生产删除；tasks/09-remove-compat-runtime/TODO.csv:4禁止feature flag和双路fallback的最终验收保留。
