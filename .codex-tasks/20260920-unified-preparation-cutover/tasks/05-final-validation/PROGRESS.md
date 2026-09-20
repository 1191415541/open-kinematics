- 任务：终局回归与 Epic 收口
- 形态：single-full（Epic 子任务）
- 进度：5/5 步骤完成；前四步全部原子命令已逐条独立执行 exit0 并即时写入父级 PROGRESS 的 子任务 05 记录
- 当前：`--final-preclose` 首轮 exit1（证据文件字段把「退出码副本」说明并入路径）已保留失败记录并收敛为单一真实路径，r2 exit0；状态写入后 `--final` 独立执行 exit0；Epic 收口完成
- 验证：专项全量 714 passed/1 skipped/1 xfailed（677.06s）；核心范围 177 passed/1 xfailed；SPEC architecture/cli/simulation 103 passed；artifact+Adams 12 passed；历史兼容+artifact 24 passed；ruff/compileall/ty/diff/post-delete scan/architecture scan/AST 唯一 decode_result 全部 exit0
- 文件：`.codex-tasks/20260920-unified-preparation-cutover/tasks/05-final-validation/`

## 恢复信息

05 前序 fixer（0b9812a5）被主动停止；静态独立审查 cb5812cf 确认 6 个目标但发现 axle 文档运行级缺陷，主线程真实复现后回退到 03 责任边界做最小修复（`results/decoder.py` 分派加入 `AxleDynamicsModel`/`AxleDynamicsCase` 实例校验、`test_axle_dynamic_contract.py` 末尾新增 6 个运行级回归），代码已冻结。红绿证据（A-E）与 r2 全部 12 条不同原子命令均已入父级 PROGRESS 记录。

第 5 步已完成：`--final-preclose` 首轮 exit1 的失败命令、原因与日志保留在父级记录（TODO 第 5 步 retry_count=1），「证据文件」字段收敛为真实单一路径后 r2 exit0，随后写入 DONE 状态（含 completed_at/notes）并独立运行 `--final`；无 git 提交。原 `05-final-full-pytest.log` 为修复前被中止运行，不计为通过。

终局任务不负责修复未知架构问题；若验证发现缺口，应退回对应子任务并记录失败证据，不得直接把状态改为 DONE。
