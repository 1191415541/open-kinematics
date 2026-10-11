# P1 步骤 5 验证证据：最小查询与原生工件往返

对应 `SPEC.md` §44–51 步骤 5；验收条件：**每准静态样本值/原因有身份，冷读取查询不重新求解**。

## 0 结论

| 项 | 结论 |
|---|---|
| 样本身份 | 每个样本按 `(case_index, output_sample_id, time_s)` 标识，不用数组下标 |
| 缺失通道 | 报查询错误并列出可用通道；**绝不**零填充或回退重算 |
| 冷读不重解 | 以「把 `run_compiled` 换成抛错函数 + 计数为 0」证明，非靠函数名推断 |
| 工件往返 | 通道、时间栅格、模型文档、`model_fingerprint` 逐项存回；可重建 envelope 并重答同一测量 |
| 向量量 | `roll_center` 是 `(samples, 2)`；压成一维会谎称两分量共用一个单位，已改 `VectorValue` |
| 独立性复审 | 交付后经只读子代理复审，发现 2 个真实缺陷（多 case 时间身份、dtype 强制转换），均已修 |
| 门 | ruff/ty 全仓 0；三条架构门 exit 0；快速集 1715 passed / 1 xfailed |

## 1 新增查询层 `results/characteristics.py`

设计 §5.1 把「校验/解码通道及采样身份」归 `results/characteristics.py`。本步落地六个函数：

| 函数 | 职责 |
|---|---|
| `sample_identities(result)` | 按 native manifest 的 case 跨度给每个扁平样本 `(index, case_index, output_sample_id, time_s)` |
| `require_channel(result, name, ...)` | 返回已存 native 通道；缺失则报错并列出可用通道 |
| `measurement_from_channel(...)` | 一维→`MeasurementValue`；`(samples,n)`→`VectorValue`；非有限值须有原因 |
| `characteristic(result, request_id, ...)` | 把已持有通道组装成应答，校验 domain 合法、样本数一致 |
| `envelope_from_artifact(payload)` | 从工件重建可查询 envelope（不重解、不重编译） |
| `measurement_query(result, name, ...)` | 对一个已声明测量作答的最小查询 |

`ResultEnvelope.characteristic(name)` 为其委托入口，`measure` 契约未改动。

### 为什么「读请求」与「应答请求」要分开

P0 已把这两件事拆成两个能力（`require_characteristics_answerable`）：能力文档声明
`characteristics_request_versions: [1]` 而 `characteristics_response_kinds: []`，因此**接受请求不等于能算出应答**。
查询层据此设计——未请求的通道就是**缺失**，不是零。返回零数组会让调用方把
「内核没被要求算」读成「答案是零」，而两者在表里长得一模一样。

## 2 实测发现（四点，均已修）

### ① `roll_center` 是向量量，`ravel` 会伪装单位

首轮实现用 `np.asarray(values).ravel()` 统一处理，测试报
`response 'roll' has 4 samples but the request selected 2`——因为 `measure("roll").values` 形状是
`(2, 2)`（两个样本 × 横向/垂向两分量），`ravel` 把 4 个数字摊平，既谎称两分量共用一个单位 token，
又让「每样本原因」无处存放。改为：一维→`MeasurementValue`，二维→`VectorValue`（每分量一个单位）。

### ② 工件存的是 native 通道，不是派生测量值

实测 `arrays.npz` 的键为
`body_state / constraint_jacobian / constraint_multiplier / constraint_reaction / constraint_wrench / diagnostics / element_wrench / energy / times_s`，
`manifest["tables"]` 为空。即 `write_artifact` 冻结的是**原生通道 + 模型文档**，而不是某个公式的输出。
这是对的设计（否则换一个中心算法就得重写全部历史工件），但意味着：
**工件的可查询性来自随行的 `model` 文档**，而不是存了一个 `roll` 数组。
据此把 round-trip 测试改为「重建 envelope 后重答同一测量并逐值比对原始答案」，
这比「从 npz 里取名为 roll 的数组」诚实得多，也真的验证了往返。

### ③ `VectorValue` 缺 `valid_mask`，与 `MeasurementValue` 不对称

`MeasurementValue` 有 `valid_mask`/`all_valid`，`VectorValue` 只有 `__len__`。
补 `valid_mask`（整行可用才算可用：缺一个分量的一行不是读数，只报有限分量会看起来像缺失分量为零）
与 `all_valid`。

### ④ 原拟的「key 与 measurement_id 不一致」守卫是同义反复

`measurement_from_channel(name, ...)` 的 `measurement_id` 由 `name` 派生，
故「两者不一致」永不触发。已删除该守卫，改为测**真实**的拒绝：响应长度与请求选中样本数不一致
（`CharacteristicResult` 处两者才同时已知）。

## 3 冷读不重解的证据方式

「有没有重解」在答案确定时无法从返回的数字看出，因此用两道独立的锁：

1. 把 `suspension_multibody.simulation.runner.run_compiled` 换成抛 `AssertionError` 的函数；
2. 加计数器，断言查询三次后计数为 **0**。

第二道锁是第一道锁有意义的前提——若那段代码根本不被咨询，单靠 monkeypatch 两种实现都会通过。
另有 `test_the_read_path_does_not_reload_modelling_resources`：把 `result.model` 换成
**任何属性访问都抛错**的对象后，已算出的应答仍可读出（证明应答来自已解码结果，不是重新编译）。

## 4 验收覆盖

`tests/characteristics/test_query_artifact.py`，23 个用例，六组：

| 组 | 用例 | 断言要点 |
|---|---|---|
| 身份 | `test_every_sample_carries_its_own_identity` | 每样本 `index/case_index/output_sample_id/time_s` 齐备 |
| 身份 | `test_a_query_attaches_identity_to_every_answered_value` | 应答样本数 == `len(times_s)` |
| 身份 | `test_the_answer_records_the_provenance_it_came_from` | request/domain/boundary/operator 版本 + 模型指纹 |
| 缺失 | `test_a_channel_that_was_never_requested_is_a_query_error` | 报缺失通道名并列出可用通道 |
| 缺失 | `test_a_channel_query_returns_the_stored_values_unchanged` | 返回存储值，不做变换 |
| 缺失 | `test_an_unavailable_sample_needs_a_reason_and_is_not_a_zero` | 无原因即拒绝；有原因则状态为不可用且值为 NaN |
| 缺失 | `test_a_valid_sample_may_not_claim_a_reason` | 有效样本不得携带原因 |
| 冷读 | `test_a_cold_read_does_not_invoke_the_kernel` | 计数 0 |
| 冷读 | `test_the_read_path_does_not_reload_modelling_resources` | 毒化 `model` 后应答仍可读 |
| 往返 | `test_native_artifact_preserves_identity_units_and_reasons` | 逐通道 + 时间栅格 + 指纹 |
| 往返 | `test_a_measurement_can_be_re_answered_from_the_artifact_alone` | 重建 envelope 后逐值比对 |
| 往返 | `test_a_written_artifact_can_be_read_back_without_the_original_result` | 丢弃原 envelope 后仍可读 |
| 往返 | `test_a_multi_case_result_keeps_each_cases_own_grid` | 多 case 各用自己栅格，无样本丢时刻 |
| 往返 | `test_case_spans_that_disagree_with_the_grid_are_refused` | 栅格过短报错，不留空时刻 |
| 往返 | `test_the_stored_dtypes_survive_the_rebuild` | `int64` 块回读仍为 `int64` |
| 往返 | `test_the_manifest_declares_a_layout_for_every_stored_channel` | 每通道有 shape 布局 |
| 往返 | `test_artifact_tables_are_csv_parseable_with_stable_columns` | 声明表列宽一致 |
| 向量 | `test_a_vector_measurement_keeps_its_components_and_units` | 形状 `(samples,2)`、两单位、逐值等 |
| 向量 | `test_a_vector_row_with_one_missing_component_is_not_a_reading` | 缺一分量整行不可用 |
| 向量 | `test_a_vector_response_refuses_one_reason_per_component` | 拒绝按分量给原因 |
| 拒绝 | `test_an_unknown_domain_is_refused_rather_than_defaulted` | 未知 domain 拒绝 |
| 拒绝 | `test_a_response_whose_length_disagrees_with_the_samples_is_refused` | 样本数不一致拒绝 |
| 拒绝 | `test_an_unknown_measurement_is_refused` | 未声明测量拒绝 |

## 5 门与基线

| 命令 | 结果 |
|---|---|
| `ruff check .` | All checks passed |
| `ty check .` | All checks passed |
| `legacy_surface_gate.py --check` | findings 0，exit 0 |
| `check_module_layering.py --strict --final` | 0 环，exit 0 |
| `check_composable_release.py --skip-isolation` | 3 checks passed，exit 0 |
| `tests/characteristics + tests/results + tests/vehicle` | 273 passed, 1 xfailed |
| 快速集（`--ignore=adams/architecture/cases`） | **1715 passed, 1 xfailed** |
| `packages/suspension_kernel/tests packages/suspension_contracts/tests` | 176 passed |

快速集从步骤 4 的 1692 增至 1715（+23 = 本步新测试，其中 3 个来自复审修复），
**skip/xfail 未增长**（仍 1 xfailed），未重录任何冻结基线。

## 5 交付后只读复审发现（2 个真实缺陷，均已修）

按 D2 对交付代码做独立只读复审，发现两处**当前测试全绿但语义错**的问题：

### ⑤ 多 case 结果的时间身份错误（本轮最重要的一处）

`RawContractResult.times_s`（`results/raw.py:122-134`）的语义不是「一条连续时间栅格」，
而是**逐 case 取 `base[:count]` 再首尾相接**。实测：`base=[10,11,20,21,22,23]`、
两个 case 各 `count=2` 与 `count=3` 时，返回 `[10, 11, 10, 11, 20]`。

我原先用 `flat = sample_offset + within` 去索引它，后果有二：

1. case 1 的样本会**配到 case 0 的时间**（`times[2] == 10.0` 而非 `20.0`）；
2. 越过数组末尾时 `time_s` 静默变 `None`——而 `None` 是 `SampleIdentity.time_s`
   的**合法取值**（表示该样本本就无时刻），于是「offset 记错」被伪装成「合法无时刻」。

已改为**按 case 逐个走各自的栅格**（`cursor += count`），并在栅格短于 case 跨度时
**报错**而非留空。新增 `test_a_multi_case_result_keeps_each_cases_own_grid`
与 `test_case_spans_that_disagree_with_the_grid_are_refused`。

诚实说明：`raw.times_s` 的逐 case 切片语义是**既有行为**（本步未改 `raw.py`），
上述两点是本层建立在它之上的索引假设错误；护栏测试同时钉住了 `raw` 自己的拒绝路径
（`exceeds decoded time grid`），因此该测试若因 `raw` 语义变化而失效会被发现。

### ⑥ `envelope_from_artifact` 无条件 `dtype=float` 强转

原实现对除 `times_s` 外的每个块做 `np.asarray(value, dtype=float)`。今天所有 native 块
实测都是 `float64`（`body_state/constraint_wrench/diagnostics/energy/element_wrench/`
`constraint_multiplier/constraint_reaction/constraint_jacobian`），所以**现有测试全绿**；
但若将来加入整型索引块或接触事件块，回读会静默改 dtype 或在字符串上抛 `ValueError`。
已改为保留存储 dtype，新增 `test_the_stored_dtypes_survive_the_rebuild`（用 `int64` 块断言）。

### 复审同时指出的「测试弱于 docstring」

原 `test_native_artifact_preserves_identity_units_and_reasons` 的 docstring 宣称验证
「units 与无效原因」往返，而工件存的是 native 通道、根本没有 units/reasons 字段——
该测试已按发现②改写为「重建 envelope 后重答测量」，docstring 同步更正，不再声称验证不存在的东西。

## 6 诚实的空白

- 本步的「工件往返」验证的是 `io.artifacts` 既有写入器；设计 §225 指出
  **`write_artifact` 的 `formats` 参数尚未驱动 CSV/Parquet 写表**（实测 `manifest["tables"]` 为空），
  即**导出能力不等于读取能力**。实际写表行为按设计 §225/§227 归 P6（步骤 6 之后的导出收口），
  本步不声称已完成导出。
- `envelope_from_artifact` 重建的 `document` 只带 `contract_version/manifest/status`，
  未还原 `case_identity`、`contract`、`kind` 等原生文档字段——本步只需要它们支撑测量查询与身份；
  若后续有消费者需要完整原生文档，应补从这里还原。
- 查询层只覆盖 `kinematic` 域的**已存测量**；`equilibrium`/`trajectory`/`dynamic` 域的应答
  依赖 P2/P3 的共享物理切线与接受态调度，本步不涉及。
- 冷读测试用 monkeypatch 打桩 `run_compiled`，属于**同一进程内**的证明；
  跨进程的「不重解」由工件读取路径本身不含 kernel 调用保证（本步未开子进程复核）。
