- 任务：冻结迁移清单、行为基线与可执行验证命令
- 形态：single-full（Epic 子任务）
进度：8/8 步骤 DONE
当前：动态哈希兼容性已闭合；统一 artifact writer 恢复冻结 NPZ 成员顺序，sentinel 对统一 manifest 使用旧动态字段的 canonical projection。26/26 artifact 的数组 bytes、兼容 canonical manifest 和组合哈希均匹配冻结基线；02-09 可按依赖启动。
文件：`.codex-tasks/20260921-architecture-deviation-closure/tasks/20260921-01-baseline/`
验证：动态哨兵 `--check` 连续两次退出 0，组合哈希 `e7407656731ed556efc28fb89d8fc69881725b3bfb2f39066eb898389d48e`；收尾回归 `668 passed, 47 skipped, 1 xfailed`，ruff/ty 均退出 0。原始日志为 `raw/dynamic_hash_check_compat_fix.log`、`raw/dynamic_hash_check_compat_fix_repeat.log`、`raw/step8_closeout_compat_fix_final.log`。

## 恢复信息

前置：步骤 1-5、7 已冻结并完成；步骤 6/8 曾因 artifact bytes/schema 兼容漂移失败，现已通过最小兼容修复恢复。修复未改变 solver、ABI、单位、数值基线或统一 artifact 的真实字段。

交付物路径：父级 `.codex-tasks/20260921-architecture-deviation-closure/VALIDATION.md`（02-09 的只读前置真源）；本任务 `SYMBOL_MATRIX.csv`（08 删除门禁的逐项闭合依据）。

已完成：步骤 1-4 冻结工具链、ABI、帮助参数、全量测试与静态检查基线；步骤 5 生成并校验 379 行 `SYMBOL_MATRIX.csv`；步骤 6 完成动态/K/C/family/性能门及不可执行项记录；步骤 7 完成报告契约缺口表；步骤 8 完成回归和父子状态回填。

兼容修复证据：统一 writer 的 native arrays 顺序恢复为旧 writer 的 `diagnostics` 在 `tire_output` 前；sentinel 保留统一 manifest 的 `metrics`、`failure_evidence`、`partial_evidence` 等当前契约字段，只在 hash 比较时投影旧字段。新增测试锁定 NPZ 成员顺序。

仍未完成：真实 Adams accuracy 受安装、参考工件和许可证限制；报告契约缺口仍由任务 05-07 负责 native 事实补齐、decoder/API 切换和 report 消费。两者不能声明为已通过。

下一步：进入子任务 02，先实现源码/头文件分层和 Python 职责及删除门禁；后续每个子任务继续执行 `VALIDATION.md` 的冻结 ABI、动态 hash、K/C、family 和性能门。

关键证据：步骤 6 新日志在本任务 `raw/`；旧失败日志仍保留为 `raw/dynamic_hash_check.log`、`raw/dynamic_hash_check_repeat.log`、`raw/dynamic_hash_diff.log`，用于追溯原始漂移裁定。

