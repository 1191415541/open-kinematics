# 子任务 04：整轴 service 入口迁移

## 目标
把整轴内部执行 service 切换到统一 runner、正式结果对象和指标层；拆除 `axle_dynamics/contract_run.py` 中混合的执行、错误处理和结果解码职责，但不在本任务切换所有外部调用方或新 artifact 写出。

## 范围
- `axle_dynamics/contract_run.py`
- `axle_dynamics/result.py`
- `results/axle.py` 的调用接口配合
- `metrics/axle.py`
- `metrics/common.py`
- 整轴 service、整轴 contract wrapper 和内部测试
- 任务 01 allowlist 中归属于整轴 service 的条目

## 必须完成
1. 整轴输入转换由对应 compiler/service 完成。
2. native 执行只经 `simulation.run_request()`。
3. 事实结果由 `results.decoder` 生成 `AxleResult`，兼容层按需包装为 `AxleDynamicsResult`。
4. 整轴指标由 `metrics.common`、`metrics.axle` 和明确注册的 case-specific metrics 计算。
5. 保留异常类型、失败诊断、partial 状态、通道顺序、单位和时间语义。
6. `run_axle_dynamics()` 在迁移期只作为薄兼容 wrapper/service facade，不直接调用 native、不内联解码。
7. `axle_dynamics.io.write_axle_dynamics_artifact()` 不在本任务产生新 artifact；统一 writer 由任务 06 建立并接管调用方。
8. `axle_dynamics.native` 只在删除门槛前保留必要兼容，不得被新整轴 service 引用。

## 约束
- 保留 `AxleDynamicsResult` 的公开字段和兼容访问行为。
- 不把整轴事实数据复制到 metrics。
- 不用零值掩盖 unavailable 或 failed 通道。
- 不修改 `api.py`、CLI、Adams 外部调用方的切换归属；这些由任务 06 负责。

## 验收
- 整轴 service 的 native 调用链完整经过统一 runner。
- 生产路径实际使用 `AxleResult` 和 axle metrics。
- 旧 contract_run 只保留薄兼容或被安全删除。
- 整轴成功、失败、partial、异常和结果兼容回归通过；新 artifact 不在本任务验收。

## 验证
`uv run --all-packages pytest packages/suspension_multibody/tests/axle_dynamics packages/suspension_multibody/tests/adams/test_time_domain_axle.py packages/suspension_multibody/tests/adams/test_axle_dynamics_equivalence.py -q && uv run --all-packages pytest packages/suspension_multibody/tests/architecture -q`
