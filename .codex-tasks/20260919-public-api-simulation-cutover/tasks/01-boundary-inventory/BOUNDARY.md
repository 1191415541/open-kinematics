# 统一仿真边界与调用图

## 目标链路

```text
Public API / CLI / Adams / scripts
        ↓
SimulationRequest
        ↓
cases/<family> compiler
        ↓
simulation.run_request()
        ↓
NativeContractBackend
        ↓
ContractRun
        ↓
RawContractResult
        ↓
results.decoder
        ↓
AxleResult / VehicleResult
        ↓
metrics
        ↓
io.artifacts
```

时域/replay 在单样本链路上生成 `TimeSeriesResult(times_s, samples, diagnostics, metrics, provenance, status)`。

## 分层职责

| 层 | 允许职责 | 明确禁止 |
|---|---|---|
| Public API / CLI / Adams / scripts | 输入校验、构造 request、调用 service、展示结果、请求 artifact 写出 | 直接 native、直接 case runner、直接 decoder、手工读取 block、领域旧 writer |
| `cases/<family>` compiler | 编译 model/case document、payload、layout、metadata | DLL/ABI、native 执行、结果解码、指标、artifact |
| `simulation` | request、compiler registry、runner、backend 编排 | 第二套 solver、理解工况专用物理语义 |
| `NativeContractBackend` | 唯一 native contract 提交点 | 组装领域结果、计算指标、写 artifact |
| `results` | raw 解码、assembly 结果、时域聚合、状态和证据 | 计算派生指标、写业务 artifact |
| `metrics` | 从正式结果对象计算派生量 | 复制 native block、事实时间序列或结果对象 |
| `io.artifacts` | 统一 manifest、arrays、diagnostics、performance、metrics 和 failure evidence 读写 | 重新执行仿真、重复定义结果 schema |

## 工况族边界

| 工况族 | 独立 compiler | 单样本结果 | 时域聚合 | case-specific metrics |
|---|---|---|---|---|
| `kc_quasi_static` | 保留 K&C load path / contract 语义 | `AxleResult` | replay → `TimeSeriesResult` | K/C derived metrics |
| `axle_dynamic` | 保留整轴动态输入语义 | `AxleResult` | dynamic samples → `TimeSeriesResult` | axle/dynamic metrics |
| `vehicle_kc` | 保留整车 K&C 语义 | `VehicleResult` | replay → `TimeSeriesResult` | vehicle K/C metrics |
| `vehicle_dynamic` | 保留显式多体整车语义 | `VehicleResult` | dynamic samples → `TimeSeriesResult` | vehicle wheel-load metrics |
| `handling` | 保留操稳输入与输出语义 | `VehicleResult` | scenario samples → `TimeSeriesResult` | explicit not-applicable or registered metrics |
| `ride_four_post` | 保留四柱平顺语义 | `VehicleResult` | time-domain → `TimeSeriesResult` | explicit not-applicable or registered metrics |
| `ride_random_road` | 保留随机路面语义 | `VehicleResult` | time-domain → `TimeSeriesResult` | explicit not-applicable or registered metrics |

## 迁移阶段

1. **边界盘点**：锁定入口、旁路、重复 decoder、旧 writer 和兼容 facade。
2. **compiler/runner**：所有工况族进入 `SimulationRequest → run_request()`。
3. **results**：统一 raw、decoder、单样本结果和 `TimeSeriesResult`；清零新 bundle 创建。
4. **axle / vehicle service**：内部 service 转发到 simulation，结果进入 results，指标进入 metrics。
5. **API / CLI / Adams / IO**：统一 artifact owner 和调用方。
6. **metrics / artifact 收口**：接通四类 metrics，完成三态 artifact 重读。
7. **删除门禁**：清零 allowlist，切 strict mode，删除无调用方兼容代码。

## 架构门禁语义

- 任务 01 起使用精确到文件和符号的 `LEGACY_ALLOWLIST.toml`，只允许已知既有旁路。
- 新增旁路、目录级豁免、通配符豁免和未登记的直接 native/decoder/writer 调用必须失败。
- 任务 02–07 每完成一项迁移，删除对应 allowlist 条目并记录验证证据。
- 任务 08 切 strict mode，仅保留 `NativeContractBackend` 的 native 调用和明确的历史读取兼容。
