# P3 动态采样 —— 子任务验收证据

> SPEC §57 要求 `raw/validation.md` 记录**真实证据**后才可更新父状态。
> 本文件是 P3 的退出验收；逐步证据见 `raw/` 下各步骤文件。

## 1. 验收结论

P3 全部 6 个步骤 **DONE**，行为验收通过。子任务状态 `TODO` → **`DONE`**。

| 步骤 | 验收条件 | 结论 | 证据 |
|---|---|---|---|
| 1 | 只读核实状态/导数/模式/内部变量同步的不可替代证据 | 通过 | [sample-state-audit.md](sample-state-audit.md) |
| 2 | `initial`/`accepted`/`output` 与 `indices` 的真实样本身份 | 通过（`accepted` 如实拒绝） | [sample-selection-evidence.md](sample-selection-evidence.md) |
| 3 | 移动 frame 与停车/零行进速度正确，不含伪时间 | 通过 | [trajectory-rates-evidence.md](trajectory-rates-evidence.md) |
| 4 | 模板/端点/力元覆盖，全覆盖与事件/可微性 | 通过 | [sampled-difference-evidence.md](sampled-difference-evidence.md) |
| 5 | 有/无请求原状态/能量/事件/步序一致且覆盖目标网格 | **分层通过**（见 §3） | [runtime-comparison.md](runtime-comparison.md) |
| 6 | 共享片段精确记录；离散参数未进入物理服务；专项与回归通过 | 通过 | [retirement-evidence.md](retirement-evidence.md) |

## 2. 交付物

### 2.1 生产代码（纯 Python 读侧）

| 文件 | 内容 |
|---|---|
| `results/sampling.py` | `SAMPLE_KINDS`、`SampleSelection`、`select_samples`、`require_selection` |
| `results/trajectory.py` | `DYNAMIC_GEOMETRIES`、`GeometryRate`、`pose_rates`、`geometry_rate_request` |
| `results/differences.py` | `DIFFERENCE_TEMPLATES`、`SampledDifference`、`sampled_time_difference` |
| `characteristics/values.py` | `SAMPLE_KINDS`、`SampleIdentity.sample_kind` |
| `results/__init__.py` | 上述导出 |

### 2.2 专项测试

| 文件 | 条数 |
|---|---|
| `tests/characteristics/test_sample_selection.py` | 19 |
| `tests/characteristics/test_trajectory_rates.py` | 26 |
| `tests/characteristics/test_sampled_time_difference.py` | 34 |
| `tests/characteristics/test_runtime_invariance.py` | 16 |
| **合计新增** | **95** |

### 2.3 C++ 改动（步骤 6）

`mb_model/functions.hpp` 新增 `body_world_inertia`；`kernel_model_accessors.cpp` 定义；
**7 处调用点**迁移（`force/layout.cpp`、`joint/kernel_directional_constraint.cpp`×2、
`output/kernel_output.cpp`、`solve_dynamic/kernel_integrator.cpp`、
`solve_dynamic/kernel_integrator_newton.cpp`×2、`solve_dynamic/kernel_integrator_residual.cpp`）。

## 3. 步骤 5 的**分层**验收（不得简化为"通过"）

SPEC §56 原文要求"开启功能也不改变原积分/事件/步序"。实测**不能笼统判为通过**：

| 子主张 | 实测 | 判定 |
|---|---|---|
| 无请求基线不变 | 逐位精确一致 | **通过** |
| 端点状态不随请求改变 | 逐位一致 | **通过** |
| 内部共享时刻状态不变 | 仅到 1–4 ULP | **部分** |
| 事件不变 | 本 rig **无事件可验** | **未覆盖** |
| **步序不变** | **不成立**（接受步数 44/46/46/46/50） | **不通过** |

**"步序不变"不通过的成因**：native 采样循环按 sample 迭代
（`kernel_abi.cpp:347`），步长被 `remaining` 与 `next_prescribed_input_breakpoint`
（`kernel_input.cpp:169-184`）双重截断——**请求的输出网格即积分步长的硬上界**。

**归属判定**：该机制位于 native 采样循环内。P3 是读侧子任务，既无权也无需修改。
**本步如实登记该缺陷面，未修改、未弱化、未掩盖，也未以"旧测试全过"替代新能力验收。**

**移交建议（不属 P3 范围）**：若全局口径确需严格步序不变，应**单独立项**，
且须先与 SPEC §39"超全接受步预算报告，不降采样"的口径对齐——两者存在张力：
既要求不改变步序、又要求超预算时报告而非降采样，需明确以何者优先。

## 4. 验证命令与实测结果

### 4.1 数值门（步骤 6 触及 native，故必跑）

| 命令 | 结果 |
|---|---|
| `uv run --no-sync python packages/suspension_multibody/scripts/build_axle_native.py` | 重建成功，产出 `suspension_kernel.dll` |
| `dynamic_hash_sentinel.py --check` | **OK: matches the frozen baseline byte-for-byte** |
| `case_parity_check.py` | **OK: 8 families accepted** |
| `kc_perf_gate.py` | **OK: within budget**（k-100 ×0.935、c-66 ×0.893） |
| `mb_linear_batch_selftest.exe` | **exit 0** |

**sentinel 的非零 acceptance exit code 是既有事实、非本步回归**：该脚本自述非零退出
**属于预期**（三个 road case 已知在 `fixture.force_z` 单通道上不过 `time_convergence` 门，
项目如实记录而非放宽用例），sentinel **只在漂移时失败**。实测失败清单与冻结基线
`dynamic_hash_baseline.json` 的 `failed_cases` 字段**逐项相同（9 个 case 名，MATCH: True）**，
未新增、未消失。

### 4.2 快速门（AGENTS.md §1）

| 命令 | 结果 |
|---|---|
| `ruff check .` | 0 |
| `ty check .` | 0 |
| `legacy_surface_gate.py --check` | 0 |
| `check_module_layering.py --strict --final` | 0（0 环，无新增边） |
| `check_composable_release.py --skip-isolation` | 0 |
| 快速集（排除 adams/architecture/cases） | **1958 passed, 1 xfailed** |
| `pytest packages/suspension_kernel/tests packages/suspension_contracts/tests` | **176 passed** |
| `pytest .../tests/characteristics` | **401 passed** |
| `git diff --check` | 干净 |

**skip 0、xfail 1（未增长）**，与基线一致。**未重录任何冻结基线。**

### 4.3 架构门的分层代价

`mb_model → mb_numeric` 是 `layering_baseline.json` 中**已登记**的 source edge，
故新增 `body_world_inertia` 到 `mb_model` **未产生新边、未修改基线**；
`check_module_layering.py --strict --final` 独立复核通过（0 环、无未登记边）。

## 5. 破坏验证汇总

各步自设破坏的捕获情况（详见各步骤证据文件）：

| 步骤 | 捕获 | 未捕获 | 未捕获项性质 |
|---|---|---|---|
| 2 | 3/3（修测试后） | — | — |
| 3 | 12/12（修测试后） | — | — |
| 4 | 8/8（修测试后） | — | — |
| 5 | 2/3 | `_DIAGNOSTIC_TAIL_ROWS: 2→0` | **单 case 下语义等价**（非测试缺陷） |
| 6 | 数值门逐位比对 | — | 抽取类改动由 sentinel 逐位覆盖 |

步骤 5 的未捕获项经查明是**语义等价**（`_diagnostic_samples` 逐 case 切
`[offset : offset+count]`，单 case、offset=0 时两种常数都给同样行数），须多 case 计划
才能观测——与步骤 2/3/4 三次"测试缺陷"性质不同，已如实区分记录。

## 6. 复用既有实现（D6）核对

| 复用目标 | 用途 |
|---|---|
| `compliance.py:RATE_FLOOR`/`RateMetric`/`_refused` | 阈值与拒绝语义同构 |
| `measurements.py:measurement_derivative` | 确认差分只属测试 oracle，已引其禁令 |
| `envelope.py:ContactEvent` | 事件通道（时间→样本映射仍缺，见 §7） |
| `raw.py:_PERFORMANCE_FIELDS`/ledger 列约定 | 预算计量 |
| `mb_model:body_effective_inertia_body` | 步骤 6 共享单元的基础访问器 |

## 7. 缺口清单（不声称已解决）

1. **`accepted` 采样选择不可回答**（SPEC §84 点名风险，实测成立）：逐接受步状态不落在
   任何输出数组，只有计数没有身份。以**带具体数字的拒绝**回应，未缩小该缺口。
2. **步序不随请求不变**（§3）：native 采样循环内，P3 无权修改，如实登记。
3. **事件不变未覆盖**：本 rig 无接触事件。`ContactEvent` → 样本下标的映射**尚未实现**，
   调用方须自行提供 `blocks`。这是步骤 4 与步骤 5 的**同一缺口**。
4. **`raw.performance` 全零**：本路径 `available: False`、计数器全 0，预算数字只能取自
   ledger。当前 `sampling.py` 已正确绕开，但该陷阱**未在任何地方拦截**。
5. **`trajectory.py` 与 `differences.py` 无生产消费者**（R9 缺口）：
   两模块只在手工构造状态上验证，未接真实 `body_state` 输出。
6. **SPEC §38 完整偏导矩阵未实现**：只做到可区分、可拒绝。
7. **`curvature_scale` 含 `span²` 故跨网格不可比**；三维模板看不到 `f'''`，误差量级不可得。
8. **活动模式切换无对应事件通道**（事件可来自 `contact_events`，模式切换尚无）。
9. **多 case 计划下的 `_DIAGNOSTIC_TAIL_ROWS` 未覆盖**（步骤 5）。
10. **性能门禁无方差界**，<10% 回归不可排除（步骤 6）。
11. **`kernel_integrator_newton.cpp:114` 的 per-body 重算**是既有冗余，未修（范围外）。

## 8. 复审记录

| 步骤 | 复审发现 | 处置 |
|---|---|---|
| 2 | 3 个真实缺陷 | 全部修复 |
| 3 | 0 实现缺陷 + 3 组覆盖漏洞 | 全部补测 |
| 4 | 1 真实缺陷 + 4 测试缺口 + 1 命名过度声称 | 全部处置；**1 条复审主张被独立反驳** |
| 5 | 8 项意见全部成立 | 全部处置；复审另漏 1 点由自查发现 |
| 6 | 0 缺陷 + 1 死重载 + 1 证据不足项 | 死重载已删除；证据不足项如实登记 |

**累积教训（已连续八次验证）**：自设破坏的覆盖面受限于设计者自己的想象，
只有独立复审能突破它。步骤 6 复审即指出我添加的四元数重载**零调用者**——
这是"我没想到要破坏"的维度（投机 API）。
