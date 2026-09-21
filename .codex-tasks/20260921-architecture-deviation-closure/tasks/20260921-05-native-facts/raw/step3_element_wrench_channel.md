# 05 步骤 3：按缺口补 native 元件力旋量输出通道（默认关闭的可选扩展）

按 EPIC「向后兼容可选扩展」与用户裁决 A1：缺口由 C++ 输出补字段解决，但**默认关闭**，默认路径 artifact 字节不得变化，两个字节级门保持绿。

## 1. 缺口回顾（来自步骤 1 对照表）

步骤 1 识别出 G1/G2 为关键缺口：native **完全没有**按 body 的力元力旋量输出，`ComponentLoad.endpoint` / `global_load` / `local_load` 三项现全部由 Python 本构（`api.py:742`）产生。本步骤补上该通道。

## 2. 契约（已实现）

| 项目 | 值 |
|---|---|
| 常量 | `kElementWrenchOutputWidth = 13`（`cpp/include/mb_config/constants.hpp`） |
| 块名 | `element_wrench` |
| 形状 | `{total_samples, record_count, 13}` |
| 列布局 | `[0..2]` 世界系力 N；`[3..5]` 世界系力矩 N·m；`[6]` 类型码；`[7..9]` 作用点世界坐标 m；`[10]` body a；`[11]` body b；`[12]` 该行所属 body |
| 类型码 | 1=spring（含止点项）2=bushing 3=anti_roll 4=steering 5=drive_brake 6=tire 7=external/gravity |
| 开关 | `element_wrench_output_enabled()`（声明 `cpp/include/mb_config/env.hpp`，实现 `cpp/src/config/kernel_config.cpp`）；环境变量 `SUSPENSION_KERNEL_ELEMENT_WRENCH_OUTPUT`；未设置/空/"0" 均为关闭 |
| 行数 | `record_count = 2×(springs+bushings+anti_rolls+steering) + 5×tires + bodies + drags` |
| 实测形状例 | K 工况 9 样本 → `(81, 10, 13)`；弹簧+重力 3 样本 → `(3, 4, 13)` |
| 未施加力的行 | 保持 NaN，**不填零**（契约要求） |

## 3. 实现要点

- 记录点在**力装配原语层**：`add_force_on_body` / `add_torque_on_body` 增加末位默认参数 `ElementWrenchSink* sink = nullptr`（`cpp/include/mb_element/functions.hpp:76-82`，实现 `cpp/src/element/assembly_primitives.cpp:12-40`）。记录在既有累加**之后**执行，并复用已算好的 `arm` 与 `state.r[body]`；null sink 只多一次指针判断，不改变任何既有表达式。
- 力的最终加入点全部经过这两个原语（步骤 1 已核：`element/{spring,bushing,anti_roll,steering,drive_brake}.cpp`、`force/layout.cpp`、`tire/{assemble,kernel_tire_assembly}.cpp`），因此这条通道覆盖全部力元而不必逐个改函数签名。
- 开关关闭时**零分配、零描述、零追加**：块只在开关打开时分配，`push_block` 对空块直接跳过（`kernel_contract_run.cpp:839`），blob 追加 0 个 double（`:889`）。
- `contract_version`：三处（capability `:216`、case identity `:836`、顶层 `:889`）统一走 `document_contract_version()` → 开关关闭 1、打开 2。
- 新头 `cpp/include/mb_config/element_wrench.hpp` 放在 `mb_config`：该模块被各层广泛 include 且依赖为空，因此不新增任何模块依赖边（`--strict` 分层门实测通过）。

## 4. 与规格的两处实现选择（已登记）

1. 开关实现落在 `src/config/kernel_config.cpp`（`src/config/env.cpp` 不存在；内核全部开关都在 kernel_config.cpp）。
2. 块传输不经 `AxleOutput` 结构体（加字段会改变 `sizeof`，迫使 ctypes 镜像同步改，且违反「不改既有字段偏移」），改为 ABI 分配块 + 每样本观察者开记录窗口。

## 5. 验证证据（raw/）

| 命令 | 退出码 | 证据 |
|---|---|---|
| `build_axle_native.py` | 0 | `step3_build.log` |
| `dynamic_hash_sentinel.py --check` | 0（26/26 逐位一致，组合哈希 `e7407656…8d48e` 与 01 冻结值相同） | `step3_dynamic_hash.log` |
| `case_parity_check.py`（开关默认关） | 0（8 families accepted） | `step3_case_parity.log` |
| `case_parity_check.py`（`…=1`） | 0 | `step3_case_parity_switch_on.log` |
| `kc_parity_check.py --check` | 0 | `step3_kc_parity.log` |
| kernel tests | 0（15 passed） | `step3_kernel_tests.log` |
| contracts tests | 0（22 passed） | `step3_contracts_tests.log` |
| `check_module_layering.py --strict` | 0 | `step3_layering.log` |
| results+physics+vehicle+cases+architecture | 0（254 passed, 1 xfailed） | `step3_suite.log` |

ABI 实测（我方独立复核）：七符号全在、`axle=15`、`vehicle=30`、`core=1`、`contract_version=1`（默认关闭）。
`include/abi/version.hpp` 未改动（`git diff` 为空）。

## 6. 未完成项（转步骤 5）

通道已存在且可开关，但**Python 侧尚未读取**——`ComponentLoad` 仍由 Python 本构产生。接线属步骤 5（decoder 接线）；届时按 A1 保持默认关闭，只有在显式请求时才走 native 事实。

## 7. 独立复审与修复（code-reviewer `ab37bad2`）

复审结论「需修复后可提交」，7 项中 6 项通过、1 项实质性缺陷，已修复：

- **缺陷（已修）**：`element_wrench_sink()` 是进程级单例，其 `block_` 指向 `suspension_kernel_run` 栈上的局部 vector。原实现从不 `clear()`（`clear()` 成为死代码），因此该函数返回后 sink 持有悬垂指针；同进程内若随后调用 `mb_core_run`（该路径**从不** configure sink）且开关打开、且记录数恰好相等，`begin_sample` 会算出落入已释放内存的行指针 → use-after-free。
  **修复**（`cpp/src/abi/kernel_contract_run.cpp:264-275`）：在 `suspension_kernel_run` 入口加 RAII 守卫 `ElementWrenchGuard`，析构时 `if (sink) sink->clear()`，覆盖**所有**返回路径（含 `return 11` 的扩容重试与各 early `fail` 返回）。开关关闭时 `element_wrench_sink()` 返回 nullptr，守卫为空操作。
  修复后验证：构建 0；动态哈希 26/26 逐位一致（组合哈希仍为 `e7407656…8d48e`）；`case_parity_check` 8 families；kernel 15、contracts 22、套件 254 passed；`--strict` 分层门通过；**开关打开时**跑真实 native 的 `tests/e2e` + `tests/api` 4 passed（该组合需多次 `run` 调用且同进程，正是缺陷的触发路径，修复后无崩溃）。
- **复审通过项**：默认路径零影响（块不分配/不描述/不追加，`contract_version` 关闭为 1）；既有浮点结果不变（记录均在原累加之后，`layout.cpp` range-for→索引循环语义等价）；越界与 NaN 契约（`row_at` 三重检查、未触碰列保持 NaN 不填零）；`add_force_on_body` 早退语义（固定体保留行但力列为 NaN，符合契约）；模块分层无新边；`build.py` 登记单处且风格一致。
- **复审记录的非阻断风险**：两线程并发调用 `suspension_kernel_run`（开关打开）会共享同一无锁单例 sink。当前 Python 绑定为单线程串行调用，不构成现实缺陷；作为已知单线程假设登记。
- **复审记录的契约灰色地带（转步骤 5 确认）**：`drive_brake` 行对每个轮胎无条件 `add_torque_on_body`，零驱/零制动矩时行值为 0.0 而非 NaN（`add_slot` 首写语义，头文件注释明示「真正施加的零保持为零」）。步骤 5 接线时须确认消费方（`ComponentLoad`/`BushingResult`）对「施加的零」与「未施力」的区分预期。
