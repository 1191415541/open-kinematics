- 任务：建立 report 并迁移报告指标、replay 编排与声明式基准夹具
- 形态：single-full（Epic 子任务）
- 进度：7/7 步骤 DONE
- 当前：子任务完成。`report` 包 10 模块建立；replay 编排迁 `simulation/replay.py`；基准改 `tests/data` 声明式夹具；三类边界负例齐备。未删除 `analysis`/`metrics` 目录本体（属 08）。
- 文件：`.codex-tasks/20260921-architecture-deviation-closure/tasks/20260921-07-report/`
- 验证：metrics/analysis/results/cli 42 passed；architecture 91 passed；全量 736 passed / 47 skipped / 1 xfailed（HEAD 基线 724，+12 为新增测试，新增失败 0）；动态哈希 26/26 逐位一致；`--strict` 0；8 family 0；ruff 0；ty 0。证据在 `raw/step7_verification.md`。

## 恢复信息

前置：01 的 `VALIDATION.md` 已冻结；02 职责边界与负例门禁就绪；05 的 results 解码边界与 06 的作者层切换完成。

本任务新建 `report/**`，并改 `simulation/replay.py`、`results/timeseries.py`、`analysis/**` 的残留调用面、读取基准夹具的脚本与相关测试；不写 `api.py`、`schema/**`（06 已定稿），不删除 `analysis`/`metrics` 目录本体（属 08）。

下一步：无（7/7 DONE）。

实施结果（提交 `550566e`）：

- 新增 `report/` 10 模块：`report/metrics/{__init__,axle,common,case_specific,vehicle}.py`（`metrics/**` 逐字搬运，注册机制与默认 family 逐项不变）、`report/{geometry,compliance,wheel_loads,time_domain_physics}.py`、`report/__init__.py`。
- `report` 边界实测干净：import 面只有 `__future__`、标准库、`numpy` 与 `report.*` 内部；无 native/kernel/solver/axle_dynamics、无 preparation。
- `simulation/replay.py` 承接 `VehicleKCTimeDomainSolver` 的规定运动 replay；`results/timeseries.py` 新增 `aggregate_replay_samples`；无积分语义、时间网格、样本键与 provenance 不变（`tests/simulation/test_replay.py` 9 条）。
- 基准夹具改 `tests/data/benchmark_axle.json` + `tests/benchmark_fixture.py`；3 个脚本按显式常量路径读取，不 import 测试包。
- 门禁：`legacy_surface_gate` 新增 report→preparation 与 report 内复算本构两类负例（report→native 为既有），`test_legacy_surface_gate` 加真实 report 树正例；registry 14 → 5。
- 已删除或改归属的调用方：`api.py`（3 处 import 来源）、`axle_dynamics/contract_run.py`、`vehicle/service.py`、`adams/reference.py`、`cases/kc_quasi_static/workflow.py`、`analysis/metrics.py`、`analysis/vehicle_physics.py`、3 个脚本。

两处交 08 的已知遗留（未伪装完成）：
1. `api.py` 的 `preparation.assembly` 须在包级 import 块之前装载（`# isort: skip` 附理由）。已实测：还原字母序后导入因 `elements` 半初始化而失败；循环源系 06 的 `front_axle`→`elements` 边，08 删除 `elements` 后消失（08 已按此删除 `core/`，该约束的最后一环即 `elements` 本身）。
2. `analysis/vehicle_physics.py:compute_vehicle_roll_centers` 无生产调用者但调用 `build_vehicle`（执行 preparation），迁 `report` 会违反 report 边界；已原样保留，08 按 A2 模式登记（见 `../20260921-08-delete/raw/step3_deletion_record.md`）。
