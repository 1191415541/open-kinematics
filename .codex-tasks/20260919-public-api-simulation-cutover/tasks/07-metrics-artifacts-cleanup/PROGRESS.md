# PROGRESS

- 任务：完成 metrics 与统一 artifact IO 收口
- 形态：single-full（Epic 子任务）
- 进度：6/6 步骤完成
- 当前：任务 07 已完成；任务 08 为下一个可执行依赖
- 文件：`.codex-tasks/20260919-public-api-simulation-cutover/tasks/07-metrics-artifacts-cleanup/`
- 下一步：执行兼容代码删除与最终 strict 门禁

## 已完成

- [x] common、axle、vehicle、case-specific metrics 已从正式结果对象进入生产路径
- [x] K&C 指标已迁移到 `metrics.case_specific` 注册实现，`analysis.metrics` 保留兼容转发
- [x] vehicle physics 保留在 `analysis.vehicle_physics`，wheel-load 派生指标迁移到 `metrics.vehicle`
- [x] `io.artifacts` 已统一成功、partial、failed artifact manifest、status 和 evidence
- [x] arrays、layout、diagnostics、performance、metrics 重读协议已闭环
- [x] 新生产路径停止创建 `DynamicResultBundle` 和调用领域旧 writer
- [x] `vehicle_kc_dynamic` 明确返回 `not_applicable` evidence；partial malformed payload 不覆盖原生失败
- [x] vehicle static wheel-load evidence 已进入成功和 partial metrics 路径

## 验证

- `uv run --all-packages pytest packages/suspension_multibody/tests/metrics packages/suspension_multibody/tests/io packages/suspension_multibody/tests/analysis/test_axle_dynamic.py -q`：19 passed
- 任务 07 全量验证：204 passed，1 skipped，1 xfailed，0 failed
- architecture：41 passed；source/tests/scripts `compileall` 通过；`git diff --check` 通过

## 恢复信息

保持结果事实与派生指标分层，旧 writer 仅保留历史读取或兼容转发；任务 07 的生产路径、统一 artifact IO、指标注册和回归验证均已完成。
