# P2 步骤 5 证据：兼容请求共享缓存与计量

> 交付物：`packages/suspension_multibody/src/suspension_multibody/results/cache.py`
> 测试：`packages/suspension_multibody/tests/characteristics/test_response_cache.py`（57 用例）
> 授权：SPEC §51（步骤 5）· 设计 §5.3（303-312 行）· 设计 §8 / §8.1（495-515 行）

## 0. 结论摘要

| 项目 | 结果 |
|---|---|
| 同一样本同 A 的装配/分解次数 | **1 次**（跨指标、跨载荷模式共享），装配由 `assemble` 可调用对象计量证明 |
| 结构复用与数值复用的区分 | **可区分且互斥**：值变而模式不变 → `structural_reuse`；模式变 → `reassembled` |
| 跨样本 / 跨域 / 跨边界复用 | **拒绝**，且拒绝文本逐项列出差异字段 |
| 响应缓存与分解缓存的层级 | 响应缓存**在上**：不同指标 ID 不阻断同一 A 的分解共享 |
| 返回值别名 | `tangent` 与 `held_tangent` **均返回副本**，调用方改写不可污染缓存 |
| 计量诚实性 | `assembly_count`/`factorization_count` 只记真实发生的次数，复用不增加计数 |
| 独立复审 | D2 只读复审发现 **3 项真实缺陷 + 6 项契约/命名/覆盖问题**，全部修复并各自以蓄意破坏验证 |
| 蓄意破坏 | **14/14 全部被捕获**，模块恢复后 57 用例全绿 |

## 1. 键的构成：什么会改变 A，什么不会

`FactorizationKey` 的字段与设计 §8.1（515 行）逐项对应：

| 设计要求的键成分 | 字段 |
|---|---|
| 模型 / 计划 | `model_fingerprint`、`plan_fingerprint` |
| 样本 | `sample_index` |
| 响应域 / 版本 | `domain`、`definition_version` |
| 边界 profile | `boundary_profile`、`boundary_version` |
| 冻结协议 | `frozen_protocol`、`frozen_version` |
| 活动模式 | `active_modes`（元组，**有序**） |
| 会改变 A 的其他参数 | 观测到的非零模式（见下） |

**活动模式用元组而非集合**：活动集的顺序是装配的一部分，成员相同而顺序不同的两个样本不是同一个 A。

**`frozen_version` 是复审补入的字段**。此前键里只有 `frozen_protocol`，于是
`frozen_inertial_load_v1` 的 version 1 与 version 2 产生同一个键。二者在 §6.5 下是实质
不同的规则（body 固定分量 / 固定世界作用点 / 随扰动重算惯性），会改变 A，却共享了一个
分解。同包的 `CharacteristicResult`（`characteristics/values.py:317`）与契约
`_FROZEN_FIELDS = {"protocol", "version"}` 早已是两字段表示，本模块此前与它们不一致。

三类身份的关系：

- `structural` —— 条件身份（不含样本、不含值、**不含模式**）
- `with_pattern(observed)` —— 条件身份 + 观测模式，供结构复用比较
- `numeric` —— 条件身份 + 样本序号，供数值复用比较

## 2. 模式由矩阵观测，不由调用方声明

这是本步骤最重要的一处修正，也是复审发现的**最严重缺陷**。

**原缺陷**：键上有 `pattern_digest: str = ""` 可选字段，且仅在非空时才校验。于是
`tangent(key(), 稠密)` 后再 `tangent(key(), 稀疏)`（一个非零变零）——两者 `structural`
完全相同（模式字段都是 `""`），`_find_structural` 命中，决策记录为
`structural_reuse`，理由写着"非零模式未变，故复用符号分析"。

而模式**已经变了**。这正是设计 §5.3（312 行）明令必须使结构缓存失效的情形，却被记成
它的反面，并发放了"可以复用符号分析"的许可证。后果分两层：

- 审计层：`CacheDecision.source`/`reason` 错，`structural_reuse` / `numeric_invalidations`
  / `structural_invalidations` 三个计数全部错位；
- 正确性层：本模块只存矩阵、不存符号分析，上层据此复用上一模式的
  fill-reducing ordering / 消元树，在模式变稀之后就是**按错误的稀疏结构分解**。

**为什么 42 个用例没抓到**：旧测试本身传了显式 `pattern=matrix_pattern_digest(...)`，
唯独覆盖不到默认空 digest 的路径——而空 digest 恰恰是测试辅助函数 `key()` 的默认值，
即绝大多数"结构 vs 数值"正例都是在空 digest 下跑的。**断言的是部署，不是行为。**

**修法**：键不再接受模式（`FactorizationKey` 无 `pattern_digest` 字段），模式一律由
`matrix_pattern_digest(matrix)` 从矩阵观测。测试
`test_the_pattern_is_observed_from_the_matrix_not_declared` 直接断言字段集，使"删掉观测"
无法通过；`test_a_dense_to_sparse_change_is_not_reported_as_a_structural_reuse` 与
`test_a_newly_nonzero_entry_is_a_pattern_change` 分别锁死两个方向。

`array != 0.0` 而非幅值阈值：结构上不存在的项在这里是精确零，引入容差会让
`1e-30` 这种真实新非零被吞掉（`test_the_pattern_digest_sees_structure_not_values`）。

## 3. 结构复用与数值复用必须可分

| 情形 | 决策 | 复用符号分析 | 复用数值分解 |
|---|---|---|---|
| 同一条件、同值 | `numeric_reuse` | 是 | **是** |
| 同一样本、模式不变、值变 | `structural_reuse` | **是** | 否 |
| 模式改变 | `reassembled` | 否 | 否 |
| 不同样本 / 不同域 / 不同边界 | `reassembled` | 否 | 否 |

`_find_structural` 把 `sample_index` 纳入匹配，而不只是比结构摘要。理由：在**另一个
样本**上命中的结构复用会被记成"复用了上一点的分析"，而这正是设计禁止的复用。结构摘要
真正的用途是在**同一样本内**区分"同模式值刷新"（真结构复用）与"模式改变"（符号分析失效）。

把复用记成复用的**理由**必须如实：`CacheDecision.__post_init__` 拒绝
`source="numeric_reuse"` 却 `numeric_reused=False` 的记录，也拒绝未知 source。审计轨迹
谎报复用，预算检查就会读到反的事实。

## 4. 响应缓存在上，分解缓存在下

设计 515 行原文要求"**不能因指标 ID 不同阻断同一 A 的分解共享**"。因此：

- **分解键不含任何指标/测量/请求 ID**，只含会改变 A 的条件；
- **响应键**由载荷模式、测量投影集合、求解策略构成，不含 A 的成分。

`test_a_second_metric_does_not_re_factor_the_same_matrix` 用装配计数器证明：换了指标之后
装配次数仍是 1。若把指标折进分解键，第二个指标就会静默重新分解——模块存在的意义全部
落空，而每一个测试仍会通过。

`response_key` 对 `measurements` 排序、对 `modes` 不排序：测量集合是无序的"问哪些量"，
载荷模式顺序是装配的一部分。两者语义不同，处理也就不能相同。

## 5. 返回值别名：两条路径都必须复制

`tangent` 与其后更名的 `held_tangent` **都返回副本**。存进去的数组是下一次请求作答的
依据，交出去让调用方原地改写，就会出现"缓存被改了、而键里的值摘要仍记录着原值"的局面：
下一次同值请求照样走数值复用分支，把被改写过的矩阵当作"同一矩阵"发出去。

`held_tangent` 这条路径比 `tangent` 更隐蔽——`_Entry.matrix_digest` 不会跟着更新，所以
篡改**永远不被察觉**。两张回归测试分别锁死：

- `test_the_returned_tangent_is_a_copy_the_caller_cannot_poison`
- `test_a_held_tangent_is_a_copy_the_caller_cannot_poison`

`held_tangent` 由 `factorize` 改名而来：本层**从不做任何分解**（分解在响应层由
`numpy.linalg.solve` 完成，批量回代归 C++ 的 `solve_many`）。把一个矩阵以 "factorize"
之名交出去，会让调用方以为自己拿到了分解句柄。测试
`assert not hasattr(cache, "factorize")` 锁死这个名字不再回来。

## 6. 计量的诚实性

设计 499 行要求统计独立右端、测量投影、分解、峰值内存与输出体积，并把组装、分解、回代、
序列化单独计量。本模块的计量原则是**每个字段都记"真实发生过的工作事实"**，且两个方向
必须可分：

- `assembly_count` / `factorization_count`：只记真实发生的次数；
- `assembly_reuse`（隐含于 `numeric_reuse`/`structural_reuse`）：记省下的次数；
- `solve_count` / `independent_rhs` / `projection_count`：由 `record_solve` /
  `record_projection` 记录，由真正做这件事的层调用。

两处复审发现的计量缺陷：

1. **`factorization_count` 在复用路径自增，冷路径反而不加**——数字随工作量下降而上升，
   预算检查读到的是与事实相反的方向。改为 `record_factorization(dimension, seconds)`，
   由真正分解的一方登记。
2. **`assemble=None`（调用方已自己付过账）时仍计 `assembly_count`**——凭空报告本缓存
   没做过的装配。改为仅在真正调用 `assemble` 时计数。

`record_solve(right_hand_sides=0)`、`record_projection(count=0)` 被拒绝：零列的求解不是
一条"工作事实"。`record_factorization` 顺带把 `peak_matrix_bytes` 抬到 `dim²·8`。

`merge` 中 `peak_matrix_bytes` 取 **max** 而非求和：峰值是水位数，两份峰值相加会凭空
造出一个从未装配过的更大的矩阵，预算检查据此报出根本没发生的超限。

## 7. 复审与修复清单

D2 只读对抗性复审（独立子代理）发现的真实缺陷，逐项修复并以蓄意破坏复核：

| 编号 | 缺陷 | 后果 | 修法与锁定用例 |
|---|---|---|---|
| D1 | 空 `pattern_digest` 下稠密→稀疏被记成 `structural_reuse` | 发放失效的符号分析复用许可；三个计数错位 | 模式改由矩阵观测；`test_a_dense_to_sparse_change_...`、`test_a_newly_nonzero_entry_...` |
| D2 | `factorize` 返回缓存内部数组 | 篡改被当作合法数值复用且永不被察觉 | 改名 `held_tangent` + 返回副本；两条 copy 用例 + 名字断言 |
| D3 | 键缺 `frozen_version`，且 `frozen_protocol` 默认 `""` 不校验 | 冻结协议两个版本共享一个分解 | 补字段；`test_every_condition_that_changes_a_changes_the_key` 参数化 9 项 |
| N1 | 复用路径自增 `factorization_count`，冷路径不自增 | 成本计数与事实反向 | 改 `record_factorization`；`test_a_pure_reuse_does_not_increase_...` |
| N2 | `assemble=None` 仍计装配 | 报告未发生的装配 | 仅真调用时计数；`test_a_caller_supplied_matrix_is_not_counted_...` |
| C1 | `factorize` 名不副实（从不分解） | 调用方误以为持有分解句柄 | 改名 `held_tangent`；`assert not hasattr(cache, "factorize")` |
| C2 | 校验失败清单硬编码字段名，与 dataclass 不同步 | 新增字段可静默逃过守卫 | `assert_no_condition_reuse` 改为遍历 `__dataclass_fields__`；`test_the_condition_guard_compares_every_key_field` 断言字段集 |
| C3 | `_Entry.created_order` / `self._order` 为死状态 | 误导后续读者 | 删除 |
| C4 | `pattern_digest` 键字段与观测模式重复 | 两者不一致时以谁为准不明 | 合并为观测值，键不再声明 |

复审同时判定为**非问题**（记录以免误"修"）：`array != 0.0` 无容差正确；
`-0.0` 与 `0.0` 摘要不同是保守方向（宁可多算）；`tangent` 无条件算摘要只是 O(n²) 哈希，
不是重复装配；驱逐淘汰旧条目只会退化为多算一次，方向安全；`response` 命中/未命中记账与
返回值一致。

## 8. 蓄意破坏：14/14 全部被捕获

对模块逐一注入缺陷，要求测试集失败；每次注入后复原，最终恢复原始文本并复跑全绿。

| 注入 | 结果 |
|---|---|
| A：`tangent` 返回存储数组本身 | 捕获（1 failed） |
| D1：不观测模式 | 捕获（2 failed） |
| D1b：观测了但不进入结构身份 | 捕获（2 failed） |
| D2：`held_tangent` 返回存储数组 | 捕获（1 failed） |
| B：结构查找忽略样本 | 捕获（1 failed） |
| C：数值命中忽略值 | 捕获（3 failed） |
| D：模式摘要忽略位置 | 捕获（4 failed） |
| F：条件守卫永不拒绝 | 捕获（10 failed） |
| N1：复用自增分解计数 | 捕获（1 failed） |
| N2：调用方矩阵计为缓存装配 | 捕获（1 failed） |
| K：键丢弃 `frozen_protocol` | 捕获（2 failed） |
| L：键丢弃 `frozen_version` | 捕获（2 failed） |
| G：峰值内存求和而非取大 | 捕获（1 failed） |
| H：未命中同时计为命中 | 捕获（1 failed） |

**教训（与步骤 4 同）**：自测的蓄意破坏不足以覆盖盲区。D1/D2/D3 三项真实缺陷自测 8 项
破坏一个都没碰到——因为它们都落在"我没想到要破坏"的语义维度上（可选字段的省略、
第二条返回值路径、字段表示与同包既有契约的一致性）。独立只读复审是必需的第二次通过。

## 9. 命令与退出码

| 命令 | 退出码 | 关键输出 |
|---|---|---|
| `pytest .../tests/characteristics -q` | 0 | 273 passed |
| `ruff check .` | 0 | All checks passed |
| `ty check .` | 0 | All checks passed |
| `legacy_surface_gate.py --check` | 0 | no unregistered Python boundary violation |
| `check_module_layering.py --strict --final` | 0 | layering matches the recorded baseline |
| `check_composable_release.py --skip-isolation` | 0 | 3 release checks passed |
| 快速测试集（multibody，排除 adams/architecture/cases） | 0 | **1830 passed, 1 xfailed** |
| `pytest packages/suspension_kernel/tests packages/suspension_contracts/tests` | 0 | 176 passed |
| `dynamic_hash_sentinel.py --check` | 0 | 逐位匹配冻结基线 |
| `case_parity_check.py` | 0 | 8 families accepted |
| `kc_perf_gate.py` | 0 | within the recorded budget |
| `git diff --check` | 0 | 干净 |

xlaid 基线未增长（仍 1 xfailed）；未重录任何冻结基线。

## 10. 遗留缺口

1. **本模块尚无生产消费者。** 响应层尚未调用 `SampleTangentCache`；`equilibrium_response`
   自报 `audit["factorizations"]` 并自行求解，与本缓存尚未接线。设计 §8.1 要求的
   "开启功能不改变积分/事件/接受拒绝次数"与"无请求保持原布局与数值基线"需在接线后由
   P4a/P4b 按预算验证。本步骤交付的是**缓存机制本身及其计量语义**，不含接线。
2. **输出体积未计量。** 设计 499 行要求的"输出体积"需要序列化层参与，本模块只记矩阵
   与求解侧；留待接线时补。
3. **32 位十六进制摘要（128 bit）足以实用**：碰撞概率按生日界在 10⁹ 级键下约 10⁻²⁰ 量级，
   远低于任何数值容差的影响；且摘要只用于相等比较，不参与数值判定。
4. **`frozen_protocol=""` 与"无冻结协议"仍共用一个字面值**，未强制区分。当前不构成
   复用风险（空协议与真实协议产生不同键，已验证），但若未来需要在审计中区分二者，
   需改为 `None` 与空串两态。
