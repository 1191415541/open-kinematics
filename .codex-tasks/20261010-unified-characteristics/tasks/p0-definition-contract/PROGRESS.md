# P0 实施进度

## 恢复入口

- 日期：2026-10-10（Asia/Shanghai）。
- 任务：冻结核心四组合、指标/目标力律、边界及冻结协议；让特性请求穿过唯一编译/native reader/结果契约，未知内容逐层拒绝。
- 形态：single-full；状态：**DONE**；产品实施已完成并行为验收。
- 进度：**6/6**；全部步骤 DONE（2026-10-10）。
- 真源：[TODO.csv](TODO.csv)；范围：[SPEC.md](SPEC.md)；父真源：[SUBTASKS.csv](../../SUBTASKS.csv)。
- 已完成：步骤 1–5 的 raw 交付物 + 步骤 6 的 [raw/validation.md](raw/validation.md)。
  产品代码：`suspension_contracts/characteristics.py`、native `case_characteristics.cpp`、
  `solve_plan`/`multibody_case` schema、`capabilities` 的读取/应答能力分离。
  测试：`suspension_multibody/tests/characteristics/test_request_contract.py`(28) +
  `suspension_kernel/tests/test_characteristics_request.py`(15)。
- 启动依赖：无。
- 下一步：进入 P1（准静态测量、运动学与兼容迁移）。注意 P5/P6 因 Adams 2024.1 缺失被阻断。
- 最新实施验证：快速集 **1581 passed / 1 xfailed**（skip 0，未增长）；
  kernel+contracts **176 passed**；ruff/ty/三架构门/dynamic_hash_sentinel 全 0。
  详见 [raw/validation.md](raw/validation.md)。
- 阻断项：Adams Car 2024.1 未安装（本机仅 `G:\MSC.Software\Adams\2025_1_1`），
  G1–G6 保持 `pending_definition`。按 EPIC §36 与 SPEC §41–42 **不阻断 P0 DONE**，
  但**阻断 P5 与 P6 的 Adams 兼容发布**。

## 证据与同步规则

实施在raw/保存实际命令/退出码、oracle/误差、预算测量/调用迁移证据。未通过不DONE；同类3次失败暂停重评，保留FAILED/retry_count。每步同步本恢复块；子任务完成后回填父SUBTASKS，completed_at记实际时间。
