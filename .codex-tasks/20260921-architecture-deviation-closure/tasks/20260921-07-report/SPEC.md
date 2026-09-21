# 子任务 07：建立 report 并迁移报告指标、replay 编排与声明式基准夹具

## 目标

建立独立的 `report` 包，使报告与派生指标从 `analysis`/`metrics` 收敛于此，且 `report` 只消费结果与只读说明数据：

1. `metrics/`（`axle.py`、`case_specific.py`、`common.py`、`vehicle.py`）→ `report/metrics/`。
2. `analysis` 的轮几何、柔度与统计 → `report/geometry.py`、`report/compliance.py`。
3. `analysis/time_domain_physics.py` 等仅导出功能逐符号判定：纯诊断迁 `report`，求解迁 native，不以"无内部调用"擅自删除公开能力。
4. `VehicleKCTimeDomainSolver` 的规定运动 replay → `simulation/replay.py` 编排 + `results/timeseries.py` 结果聚合，保留"无积分"的原语义与时间/聚合协议。
5. `analysis/benchmarks` → `tests/data` 的声明式夹具；脚本通过明确夹具路径读取，不导入测试包。
6. 边界：`report` 不调用 native、不执行 preparation、不复算本构；`io.artifacts` 保持唯一读写归属。

## 非目标

- 不删除 `analysis`/`metrics` 目录本体与文件（08 负责），本任务只把生产调用方切到 `report`。
- 不改变 replay 的时间协议与聚合口径，不改变 metrics 的统计定义与阈值。
- 不改公开 API/CLI、不改 Adams 渲染、不新增图表/UI/第三方依赖。
- 不把求解实现搬进 `report`：求解类功能归 native。

## 约束

- 每一步结束都要跑对应测试目录；`tests/metrics`、`tests/analysis`、`tests/results`、`tests/cli` 是稳定验证命令，目录可保留。
- `report` 的边界必须由负例门禁保证：注入 `report`→native、`report`→preparation、`report` 内复算本构三类调用必须失败（复用 02 的扫描器）。
- 基准脚本只按显式夹具路径读取 `tests/data`，禁止 import 测试包；夹具为声明式数据，不写隐藏全局状态。
- `results/timeseries.py` 与 `simulation/replay.py` 是共享写面，本任务串行修改，不得与 06/08 并行写同一文件。

## 范围与文件归属

- 可写：`packages/suspension_multibody/src/suspension_multibody/report/**`（新建）、`simulation/replay.py`、`results/timeseries.py`、`analysis/**` 中被迁走后的残留调用面、`packages/suspension_multibody/scripts/*.py` 中读取基准夹具的脚本、`packages/suspension_multibody/tests/{metrics,analysis,results,cli,data,architecture}/**`。
- 只读：`packages/suspension_multibody/src/suspension_multibody/{results,io,preparation}/**`（05/06 已定稿）、父 `EPIC.md`、`VALIDATION.md`。
- 不写：`api.py`、`schema/**`（06 已定稿）、父 `EPIC.md`、`SUBTASKS.csv`、`PROGRESS.md`。

## 依赖

- 前置：01（`VALIDATION.md` 冻结结果通道与容差）、02（职责边界与负例门禁）、05（results 解码边界）、06（作者层与 native 事实切换完成）。
- 后续：08 依赖本任务把 `analysis`/`metrics` 生产调用方清零后才能删除目录。

## 验收标准

1. `report` 包存在且只消费结果与只读说明数据：无 native 调用、无 preparation 执行、无本构复算，三类负例均被门禁检出。
2. 生产 `metrics` 全部迁入 `report/metrics/`；`report/geometry.py`、`report/compliance.py` 承接轮几何、柔度与统计，数值与迁移前一致。
3. `analysis/time_domain_physics` 等仅导出符号逐条有判定结论（迁 report / 迁 native / 保留公开能力并说明），无"因无内部调用而删除"的情况。
4. replay 由 `simulation/replay.py` 编排、`results/timeseries.py` 聚合；"无积分"语义、时间与聚合协议不变，且有测试。
5. 基准夹具在 `tests/data` 且为声明式；脚本按明确路径读取，不 import 测试包。
6. `tests/metrics`、`tests/analysis`、`tests/results`、`tests/cli` 全绿，无新增失败。
7. `io.artifacts` 仍是唯一读写归属；未删除 `analysis`/`metrics` 目录本体。

## 验证协议

```bash
uv run --package suspension-multibody pytest packages/suspension_multibody/tests/metrics packages/suspension_multibody/tests/analysis packages/suspension_multibody/tests/results packages/suspension_multibody/tests/cli -q
uv run --package suspension-multibody pytest packages/suspension_multibody/tests/architecture -q
uv run python packages/suspension_multibody/scripts/case_parity_check.py
uv run --all-packages ruff check . && uv run --all-packages ty check .
```

`report` 的边界负例与 `tests/architecture` 一起验收：只跑正例不算通过。
