- 任务：输出声明与衍生输出（request 机制）
- 形态：single-full（Epic 子任务）
- 进度：9/9 步骤 DONE
- 当前：`outputs/` 包已落（`declarations.py` 声明与合并、`derived.py` 衍生输出求值器、`builtin.py` 27 个 legacy 指标的重述与适配器）；`report/metrics/**` 未改（非目标：旧函数保留）。
- 文件：`.codex-tasks/20260922-suspension-template-architecture/tasks/20260922-07-outputs/`
- 验证：`tests/outputs` + `tests/metrics` 共 59 passed；`legacy_surface_gate --check` 退出 0（注册条目仍 8 条）；`ruff`/`ty` 0；`tests/data/**` 与 `layering_baseline.json` 无 diff。

## 恢复信息

**本步已实施完成。** 前置 02 已给出统一副表；本步与其写范围（`outputs/**`、`report/metrics/**`）不相交。父 `EPIC.md` 的 G6 与 Done-When 中与输出相关的条目是终局判据。

## 用户裁决的落点（本步是其实现）

用户原话：「所有仿真的输出由总成+试验台定义，总成带有它所定义的输出，试验台也带有它独有的输出，这些输出是最小单位输出，后续的衍生结果都是使用这些输出做运算得到的（比如 KC 指标、操稳指标等等），可以引入类似 adams car 的 request 功能，即可以通过这些基本输出自定义任何输出」——本步把这句话落成三层：声明归属（总成/试验台）、最小单位输出、衍生输出表达式。

## 交付物

- `src/suspension_multibody/outputs/declarations.py`：`OutputDeclaration`（name/unit/dimension/domain/shape/description，JSON 可往返）、`DeclarationSet`、`merge_declarations`。
- `src/suspension_multibody/outputs/derived.py`：`DerivedOutput`（`reads` 声明依赖、`legacy` 记录被重述的旧函数）、`ExpressionRegistry`、`MinimumUnitOutputs`（受限视图）、`MissingOutputError`/`UndeclaredOutputError`/`UnknownOutputError`。
- `src/suspension_multibody/outputs/builtin.py`：27 个 legacy 函数的重述、内置最小单位输出声明（`ASSEMBLY_OUTPUTS`/`RIG_OUTPUTS`）、`LEGACY_CLASSIFICATION`、`minimum_unit_outputs` 适配器。
- `tests/outputs/{test_declarations,test_merge,test_derived,test_bypass}.py`、`tests/metrics/test_outputs_match_legacy.py`。

## 27 个 legacy 函数的重述对照

`resolve_properties` 式的「模块.函数名」登记在 `builtin.LEGACY_CLASSIFICATION`，逐条如下（27/27，由 `test_every_legacy_function_is_classified_and_there_are_27` 与一份独立写死的名字列表核对）：

| 分组 | 函数 | 处置 |
|---|---|---|
| axle (3) | `_tire_column` | 私有助手，列的选取改为声明的最小单位输出 `tire_normal_force`/`tire_longitudinal_force`/`tire_lateral_force` |
| | `compute_axle_metrics` | 重述为 6 个力聚合（`maximum_*`/`rms_*` × normal/longitudinal/lateral）+ 组合 time/convergence/contact |
| | `axle_metrics` | 别名，同上 6 个输出 |
| case_specific (12) | `register_case_metric` / `case_metric_for` / `registered_case_metric_families` / `_register_defaults` | 注册表 API，不产数值 |
| | `compute_case_metrics` / `_kc_quasi_static_metrics` | 分派器：按 family 取实现并组合；replay 汇总的键名来自运行期 sample metrics，无固定名输出可重述（已登记） |
| | `not_applicable_metrics` | 占位，明确不伪造数值键 |
| | `wheel_metrics` | 重述为 10 个 per-side 输出（左右各 3 中心 + camber + toe），姿态输入按侧声明 |
| | `compute_k_metrics` | 重述为 5 个 axle 级输出（track_mm / z_mean / z_difference / camber_deg_difference / toe_deg_difference） |
| | `_summarize_series_values` | 助手，统计定义复用 |
| | `_axle_dynamic_metrics` / `_vehicle_dynamic_metrics` | family 钩子，委托到 `compute_axle_metrics`/`compute_vehicle_metrics`，同输出 |
| common (8) | `_finite_array` / `peak` / `rms` / `residual_norm` | 共享统计原语，本地同语义重实现并注册为表达式 |
| | `time_metrics` | 重述为 4 个（sample_count/start_s/end_s/duration_s） |
| | `convergence_metrics` | 重述为 5 个，含可用性规则 |
| | `contact_metrics` | 重述为 3 个，含可用性规则 |
| | `compute_common_metrics` | 组合 time/convergence/contact + `status` |
| vehicle (4) | `_embedded_wheel_loads` | 助手，轮荷改为声明的最小单位输出 |
| | `wheel_load_metrics` | 重述为 11 个轮荷输出 |
| | `compute_vehicle_metrics` | 组合 `axle_` 前缀一族 + 轮荷 + steering 2 个 + 壁时 |
| | `vehicle_metrics` | 别名，同输出 |

## 逐值一致（本步最关键判据）

`tests/metrics/test_outputs_match_legacy.py` 在同一输入集上逐键对照新（衍生输出）与旧（legacy 函数）：

| 输入 | 对照键数 | 结果 |
|---|---|---|
| `FakeResult`（轮胎力 + 时间网格，无诊断） | 6 力聚合 + 4 时间 + 2 可用性 = 12 | 逐值一致 |
| `FakeDiagnosedResult`（属性形状的诊断） | 8（convergence 5 + contact 3） | 逐值一致，含可用性规则 |
| `wheel_load_metrics` | 11 | 逐值一致 |
| `compute_vehicle_metrics` | 10 | 逐值一致 |
| steering 含 NaN | 2 | 逐值一致（有限值过滤） |
| `wheel_metrics` + `compute_k_metrics`（真实装配 `benchmark_model` + `build_front_axle`） | 15 | 逐值一致 |

浮点用 `pytest.approx`：两侧走的是同一批输入与同一套算术，差异只可能来自浮点结合顺序，`approx` 的相对容差即为此。**注意**：这 15 项里有 12 项在基准装配下恰好为 0.0，逐值一致证据因此偏弱——为此另加了两条**不依赖 legacy 对比**的符号测试：按文档公式独立算出期望值核对左右两侧的 camber 符号（左右 outward 相反，同一旋转矩阵应给出相反符号），以及「翻转矩阵所读元素则 camber 反号」，避开"两侧一起错也一致"的盲区。

## 两处与 SPEC 的偏差（已在代码 docstring 登记）

1. **键存在性改为声明**：legacy `compute_vehicle_metrics` 在结果无 steering 输出时**省略** `maximum_steering_output` 键；本步的输出始终被声明，运行期未产出该输入则拒绝求值。方向是有意的（声明集不依赖某一结果恰好填了哪些字段），适配器 `minimum_unit_outputs` 通过「不产出该输入」保留 legacy 行为。
2. **`status`/`reason`/`available` 标志不重述**：它们是「这次运行是否可用」的事实与分派决策，不是对输出的算术运算。其数值化的部分（`convergence_available`/`contact_available`）已重述，其余保留在读取运行证据的地方。

## 本步的放行 gate（全部通过）

1. **旁路必须失败**：`tests/outputs/test_bypass.py` 用 `ast` 扫描 `outputs/**` 的 import 与调用名（`kernel`/`native`/`solver`/`preparation`/`elements`/`core`/`analysis` 与解算类调用），并在 `tmp_path` 里写入一个违规模块与一个干净模块**双向自证扫描器可失败**；`legacy_surface_gate.py --check` 退出 0。
2. **求值器只吃最小单位输出**：非 `Mapping` 输入抛 `TypeError`；读未声明名字抛 `UndeclaredOutputError` 并点名；运行期未产出所声明的输入抛 `MissingOutputError` 并点名。
3. **未重录任何基线**：`git status --porcelain packages/suspension_multibody/tests/data packages/suspension_kernel/layering_baseline.json` 为空。

## 门禁实测（2026-09-23）

| 命令 | 结果 |
|---|---|
| `pytest tests/outputs tests/metrics` | `59 passed` |
| `legacy_surface_gate.py --check` | 退出 0，`legacy_module_import: 8`（未新增） |
| `ruff check`（outputs + tests/outputs + tests/metrics） | All checks passed |
| `ty check`（`outputs/`） | All checks passed |
| `tests/data/**`、`layering_baseline.json` | 无 diff |

## 未完成 / 边界

- **未删除 27 个旧函数**（SPEC 非目标）：本步只做并存与逐值对照，删除留到 11 之后再定。
- `_summarize_series_values` 与 `_kc_quasi_static_metrics` 的 replay 汇总**没有**对应衍生输出：它们的键名来自运行期 sample metrics 的并集，是一组动态命名的汇总而非固定名输出。已在 `LEGACY_CLASSIFICATION` 与代码 docstring 中如实登记理由，未强行造名。
- 12 个「共享统计原语/注册表 API/分派器」类函数按定义不产固定名数值输出，登记为「无输出」，不算漏项（由 27/27 计数测试锁定）。

## 下一步

09（study 合并）与 10（试验台正交）可启动。本步为 12 的端到端判据 (d)「用最小单位输出 + 自定义表达式算出一个新指标」提供了实现：`ExpressionRegistry.register_expression` + `DerivedOutput(reads=...)` 即为该机制。
