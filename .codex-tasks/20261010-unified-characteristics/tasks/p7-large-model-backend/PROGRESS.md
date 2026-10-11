# P7 实施进度

## 恢复入口

- 日期：2026-10-10（Asia/Shanghai）。
- 任务：在同物理定义、同目标样本下独立验证大型模型的无矩阵方向作用/预条件Krylov策略，只有正确性与实测收益成立才推荐该场景。
- 形态：single-full；状态：IN_PROGRESS，步骤 1 因 P4a 直接基准缺失而阻断。
- 进度：0/5；当前步骤：1 — 冻结目标规模与直接基准成本（FAILED，retry_count=1；P4a native 直接响应未提供）。
- 真源：[TODO.csv](TODO.csv)；范围：[SPEC.md](SPEC.md)；父真源：[SUBTASKS.csv](../../SUBTASKS.csv)。
- 已完成：阻断核验并保存 `raw/direct-benchmark.json`；没有用未知规模或静态自测冒充大模型收益。
- 启动依赖：P2;P4a；证据缺口以SPEC为准。
- 下一步：P4a native/local_response_v1 与同物理问题直接基准可重放后重试步骤 1；每新步骤先重读 TODO。
- 最新实施验证：当前 direct benchmark 状态为 blocked；证据：[raw/direct-benchmark.json](raw/direct-benchmark.json)。

## 证据与同步规则

实施在raw/保存实际命令/退出码、oracle/误差、预算测量/调用迁移证据。未通过不DONE；同类3次失败暂停重评，保留FAILED/retry_count。每步同步本恢复块；子任务完成后回填父SUBTASKS，completed_at记实际时间。
