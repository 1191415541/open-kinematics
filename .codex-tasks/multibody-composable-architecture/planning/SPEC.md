# 方案冻结任务

## Goal
将本轮关于多体内核、多体模块、可组合子系统与物理试验台的讨论冻结为仓库内可执行方案；完成需求追踪、职责和接口设计、任务依赖、客观验收与独立审查。

## Boundary
本轮只修改方案、术语与导航文档，不实施源码、不新增测试实现、不重录数值基线、不运行 Adams 或付费资源。未来实施由父 EPIC 的 SUBTASKS.csv 管理，均保持 TODO。

## Deliverables
- ../EPIC.md：原始需求、裁决、Goal、Non-Goals、Done-When。
- ../DESIGN.md：目标结构、接口契约、模块迁移、兼容性与风险。
- ../TASKS.md 与 ../SUBTASKS.csv：13 个实施子任务的范围、依赖、验收与命令。
- ../REVIEW.md：独立计划审查及处理结果，不伪造审查结论。
- ../PROGRESS.md：父任务状态、实施恢复入口与本轮验证记录。
- packages/suspension_multibody/CONTEXT.md：只补领域术语。
- CONTEXT-MAP.md：增加目标方案导航，不把规划描述成已实现。

## Done-When
用户三项裁决准确进入方案；每条 Goal 都有实施任务和独立终局验收；CSV 依赖无环、任务目录唯一、命令对应明确验收；独立审查无未处理阻断；本轮仅有文档变更，实施任务仍全为 TODO。

## Protocol
规划阶段的叶级步骤由本目录 TODO.csv 管理；父 SUBTASKS.csv 只记录未来实施。执行每个实施子任务前创建该任务的 SPEC.md/TODO.csv/PROGRESS.md，不生成一批空完成记录。临时校验脚本与审查中间输出写会话 scratch，只有归档结论进入本目录。

## 版本管理
沿用仓库既有惯例，仅在根 .gitignore 增加本方案目录的两条放行规则，使正式方案可随仓库提交。其他任务、临时日志和构建产物仍保持忽略，不实施运行时配置变更。
