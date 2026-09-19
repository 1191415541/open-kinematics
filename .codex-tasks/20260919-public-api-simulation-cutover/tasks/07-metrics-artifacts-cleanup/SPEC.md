# 子任务 07：metrics 与统一 artifact IO 收口

## 目标
让 `metrics/` 真正进入生产链路，完成 K&C/vehicle physics 派生指标注册，并验证统一 artifact IO 对成功、partial、failed 三类结果的落盘与重读。

## 范围
- `metrics/common.py`
- `metrics/axle.py`
- `metrics/vehicle.py`
- `metrics/case_specific.py`
- `metrics/__init__.py`
- `analysis/metrics.py` 的 K&C 指标迁移
- `analysis/vehicle_physics.py` 的 wheel-load 派生指标迁移
- 各工况族 case-specific metric 注册实现；没有专用派生量的工况使用显式 `not_applicable`
- `io/artifacts.py` 的 metrics/manifest/重读完善
- `io/results.py`、`axle_dynamics/io.py`、`vehicle_dynamics.py` 的旧 writer 停产边界
- 成功、partial、failed fixture 和回归测试

## 必须完成
1. common、axle、vehicle、case-specific metrics 均从正式结果对象计算，并由生产 service 实际调用。
2. 将 `compute_k_metrics` 的 K&C 派生逻辑迁移到 `metrics.case_specific` 的明确注册实现；保留必要兼容转发但不再由 Public API 直接导入 `analysis.metrics`。
3. 将 vehicle physics 中的 wheel-load 派生指标纳入 `metrics.vehicle`，物理求解本身仍留在 analysis/physics 层。
4. 指标输出与事实结果分层保存，不复制 native block、时间序列或对象维度事实数据。
5. `io.artifacts` 写出 model/case document、hash、channel registry、layout、arrays、time grid、status、diagnostics、performance 和 failure/partial evidence。
6. 成功、partial、failed 使用同构 manifest 和可追溯目录结构；缺失数据保持 unavailable/not-applicable 语义。
7. 旧 `schema.DynamicResultBundle`、`io/results.py`、`axle_dynamics/io.py`、`vehicle_dynamics.py` writer 不再产生新的非统一格式。
8. 旧 artifact 读取兼容若保留，必须明确标注历史格式，并与新写出路径隔离。
9. 对 artifact 写出后重读建立 manifest、数组、诊断、性能、metrics 和失败证据回归。

## 约束
- 不把 K&C、操稳、平顺专用指标塞入 common metrics。
- 不改变通道名、顺序、单位和 manifest hash。
- 不用 fallback 零值替代缺失或失败结果。
- 不为迁移引入与当前范围无关的新存储格式。

## 责任边界
- 任务 06 负责 `io.artifacts` 的 owner、入口调用和基础协议。
- 任务 07 负责 metrics 注册、旧派生逻辑迁移、三态 artifact 完整性和重读回归。
- 任务 08 负责删除不再需要的兼容 facade 和重复实现。

## 验收
- 生产路径实际调用四类 metrics，且每个工况族都有明确注册或 not-applicable 证据。
- 成功、partial、failed artifact 均可写出、重读和追溯。
- 新生产代码不再创建旧 bundle 或调用重复 writer。
- 统一 IO 的错误处理、诊断、性能和失败证据信息完整保留。

## 验证
`uv run --all-packages pytest packages/suspension_multibody/tests/metrics packages/suspension_multibody/tests/io packages/suspension_multibody/tests/analysis -q && uv run --all-packages pytest packages/suspension_multibody/tests/architecture -q`
