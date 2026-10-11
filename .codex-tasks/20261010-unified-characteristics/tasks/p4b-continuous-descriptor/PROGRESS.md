# P4b 实施进度

## 恢复入口

- 日期：2026-10-11（Asia/Shanghai）。
- 任务：独立交付 continuous_descriptor 的连续导数和测量映射，分别验证约束一致状态/载荷扰动，不把它作为 P4a/P5/P6前置。
- 形态：single-full；状态：**IN_PROGRESS**。
- 进度：2/5；当前步骤：2 — 实现连续描述符请求方向与测量映射（仍为 TODO）。步骤 4 已按“未实现则明确无时间响应能力”完成范围锁定。
- 真源：[TODO.csv](TODO.csv)；范围：[SPEC.md](SPEC.md)；父真源：[SUBTASKS.csv](../../SUBTASKS.csv)。
- 已完成：步骤 1 审计与范围锁定；步骤 4 明确无有限时域响应支持；未新增产品能力。
- 启动依赖：P0;P3；两项均在父 `SUBTASKS.csv` 标记 DONE，并有真实验收证据。
- 下一步：先按步骤 2 重新核对 ABI/capability 与 F_xdot/F_x/F_u 结果形状；当前没有足够的 native 结果通道，未实施前不扩写产品代码。
- 最新实施验证：步骤 1 源码审计、17 行 CSV 及 41 个证据文件路径检查通过；步骤 4 范围证据基于 native capability/result descriptor 与 Python sampling/trajectory 源码复核，并实测 `require_characteristics_answerable('continuous_descriptor')` fail-closed，计划校验通过。全仓 ruff/ty 与架构门在本轮 P4a 收尾时通过；步骤 2/3 行为门禁尚未执行。证据见 raw/continuous-audit.md、raw/time-response-scope.md。

## 证据与同步规则

实施在raw/保存实际命令/退出码、oracle/误差、预算测量/调用迁移证据。未通过不DONE；同类3次失败暂停重评，保留FAILED/retry_count。每步同步本恢复块；子任务完成后回填父SUBTASKS，completed_at记实际时间。
