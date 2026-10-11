# P6 实施进度

## 恢复入口

- 日期：2026-10-10（Asia/Shanghai）。
- 任务：完成统一消费/查询/JSON-NPZ-CSV-Parquet真实往返、R1–R9唯一归属和核心四组合发布门；明确核心发布与整个Epic完成的区别。
- 形态：single-full；状态：IN_PROGRESS，步骤 2、3 已完成独立可交付部分。
- 进度：2/6；当前步骤：1 — 收口查询与全部消费者。
- 真源：[TODO.csv](TODO.csv)；范围：[SPEC.md](SPEC.md)；父真源：[SUBTASKS.csv](../../SUBTASKS.csv)。
- 已完成：步骤 2（真实 CSV/Parquet 写表）与步骤 3（冷读/部分结果/样本身份往返）已行为验收；步骤 1、4、5、6 仍 TODO。
- 启动依赖：P1;P2;P3;P4a;P5；证据缺口以SPEC为准。
- 下一步：P4a/P5 条件满足后再收口统一消费者、R1–R9 与核心发布门；每新步骤先重读 TODO。
- 最新实施验证：35 个 artifact/查询测试通过；完整 P6 仍被 P4a/P5 前置阻断。证据：[raw/table-export-evidence.md](raw/table-export-evidence.md)。

## 证据与同步规则

实施在raw/保存实际命令/退出码、oracle/误差、预算测量/调用迁移证据。未通过不DONE；同类3次失败暂停重评，保留FAILED/retry_count。每步同步本恢复块；子任务完成后回填父SUBTASKS，completed_at记实际时间。
