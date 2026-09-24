# EPIC 进度与恢复入口

## Recovery
- 任务：冻结可组合多体架构；未来按本方案实施。
- 形态：epic。
- 进度：规划 3/3 完成；实施 0/13，全部 TODO。
- 当前：正式方案已冻结，独立计划审查完成；实施未授权。
- 文件：EPIC.md、DESIGN.md、TASKS.md、SUBTASKS.csv；规划记录见 planning/。
- 下一步：取得实施指令后重新读取上述真源，创建 tasks/01-baseline 的 Full Single 记录并实测基线；本轮不启动。

## 真源与优先级
用户本轮 D1-D3 > EPIC Goal/Non-Goals > DESIGN 冻结契约 > TASKS 子任务规格；状态以 SUBTASKS.csv 为唯一实施真源。发现矛盾先修正文档，不由实施者私自选较宽解释。

## 依赖主线
```text
01 -> 02 -> 03 -> 04 -> 05 -> 06 --+
01 -> 08 ------------------------+-> 07 --+
      08 -> 09 --------------------------+-> 10 -> 11 -> 12 -> 13
```
07 必须等待08，编号不是执行顺序。共享源码、schema、CMake、注册表与 native 镜像不得多写者并发。06先验证SI装配，07切换运行，10收敛API，12删除无调用旧路径。

## 本轮裁决
- 只冻结方案，不实施代码。
- 未来允许合成新拓扑和新试验台做实际求解验收，不作为工程产品交付。
- 全局限制不放宽：单轴禁制动/驱动、车轮归试验台；整车必需转向/制动/驱动；新模板和嵌套配方不可绕过。

## 交付与修改边界
新增本方案目录（EPIC/DESIGN/TASKS/SUBTASKS/REVIEW/PROGRESS及规划记录），补充 multibody 领域术语与根导航；按既有规则在 .gitignore 中仅放行本方案目录，确保文件可随仓库提交。
未修改既有10个已修改Python文件及1个未跟踪测试，未实施代码、未改旧任务完成记录、未重录基线、未调用外部服务。

## 验证记录（2026-09-24）
- code-reviewer `8d57d44c-6796-49b3-a5b7-a5e4fce7d201`：通过，无阻断；1项中等与2项低优先级意见已处理，详见 REVIEW.md。修订由主线程复核，不冒称第二轮独立审查。
- `uv run --no-sync python "$PI_SCRATCH_DIR/validate_multibody_plan.py"`：退出码0；13行全TODO，ID/目录唯一，依赖无环；TASKS与CSV的命令/依赖一致；9目标覆盖13任务；A1-A10唯一且G5独立场景齐全；11个本地链接有效，文档无编辑标记/空白问题。
- `git diff --check`：退出码0；仅Git的LF/CRLF提示。
- `git status --short -- .gitignore CONTEXT-MAP.md packages/suspension_multibody/CONTEXT.md .codex-tasks/multibody-composable-architecture`：方案目录可见，导航/术语/忽略例外均可提交。
- 已核对现有动态哈希、K/C parity、case parity与性能脚本参数；性能门默认check，不触发重录。
- 本轮没有运行新架构的实现测试；TASKS所有实现命令是未来验收规范，不是已通过声明。

## 实施时复核与提问
旧任务登记的准静态轮胎与rack输出缺口由01复测并分别归07/10，目标相关缺口不得豁免。偏心轮胎质量、制动参数来源、真实Adams等价不因本方案自动获得支持。
若协议/ABI/API需变更、必须修改物理或数值基线、引入依赖/付费资源或突破总成规则，暂停相关任务并向用户提问。当前无未决的业务选择。
