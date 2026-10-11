# P2 步骤 1：提取共享物理组装并更新 DAG

对应 SPEC §47（步骤 1）、§19（修改范围）、§36（R_x 内容）、§43（R7 委托）。
本文件只记实测，不记预期。

## 0. 结论摘要

| 项 | 结果 |
|---|---|
| 新增 C++ 模块 | `mb_linearization`（`cpp/include/mb_linearization/types.hpp` + `cpp/src/linearization/physical.cpp`） |
| 共享的物理内容 | 力律导数、`constraint_jacobian`、`d(J^T lambda)` 预载几何项；块序 `[pose｜multiplier｜contact compression]` |
| 留在 solver 的策略 | Newton 增量参数化、gauge/pin、活动集、线搜索系数、载荷尺度 |
| 委托结果 | `static_jacobian` 不再含组装算术，改为向下调用 |
| DAG 实测 | **0 个模块环**、0 个互反边、0 个跨聚合 include（`check_module_layering.py --strict --final` exit 0） |
| 数值不变 | `dynamic_hash_sentinel.py --check` **OK：逐位一致**；并手工交叉验证 26 个 artifact 逐条 entry **0 处差异**（§3） |
| 其余门 | `case_parity_check.py` 8 族接受（`axle_dynamic` 逐位一致）；`kc_perf_gate.py` 在预算内；快速集 1715 passed/1 xfailed |

## 1. 提取边界是实测决定的，不是设计猜的

最初按设计 §19 的字面把 `pose_candidate`、`static_gauge_coordinates`、
`static_position_constraint_jacobian` 也当作"共享物"搬进新模块，编译立刻失败：

```
error: 'static_gauge_coordinates' was not declared in this scope
error: 'pose_candidate' was not declared in this scope
error: 'static_position_constraint_jacobian' was not declared in this scope
```

按符号归属实测（`cpp/include/*/*.hpp`）：

| 符号 | 声明所在模块 | 相对新模块的位置 |
|---|---|---|
| `pose_candidate` | `mb_solve_dynamic/functions.hpp:53,57` | **之上** |
| `static_gauge_coordinates` | `mb_solve_static/functions.hpp:45` | **之上** |
| `static_position_constraint_jacobian` | `mb_solve_static/functions.hpp:51` | **之上** |
| `static_position_constraint_mask` | `mb_solve_static/functions.hpp:47` | **之上** |
| `constraint_jacobian` | `mb_joint/functions.hpp:54` | 之下（可用） |
| `static_rotation_gauge_for_pivot` | `mb_model/functions.hpp:72` | 之下（可用） |

**结论**：这三个符号位于 `mb_linearization` *之上*。若在新模块里调用它们，
DAG 立刻出现 `mb_linearization → mb_solve_static/mb_solve_dynamic` 的上行边，
与"共享模块在 solver 之下"的定义直接矛盾，且必然形成环
（`mb_solve_static → mb_linearization → mb_solve_static`）。

这不是设计文档写错了，而是**"物理 vs 策略"这条切线的位置**问题：
候选位姿的参数化（左 SO(3) 增量）和"哪些行算位置约束"都是**调用方的约定**，
不是物理本身。两个 solver 可以合法地对它们有不同选择，
把它们放进共享模块等于把一方的选择冻结成另一方的契约。

**采用的切法**：把它们作为**数据**（`WorkingSample.pose`、`WorkingSample.gauge`、
`WorkingSample.position_constraint`）传入。共享模块只做组装算术，不重新推导约定。
`working_gauge_from_static_policy` 这个函数因此被删除（它会重新推导策略）。
`mb_linearization` 的 include 集合里没有 `mb_solve_static`/`mb_solve_dynamic`，
环在结构上不可能出现——实测 0 环确认了这点。

## 2. 实测得到的模块 DAG（新增边）

新模块产生 9 条新边，全部逐条登记进 `layering_baseline.json` 的
`reviewed_new_edges`（门在缺登记时**失败关闭**，不会静默接受）：

| 边 | kinds | 依据符号 |
|---|---|---|
| `mb_linearization → mb_config` | tu_include | —（prelude/常量） |
| `mb_linearization → mb_dual` | symbol_reference + tu_include | `so3_left_jacobian`, `value` |
| `mb_linearization → mb_force` | tu_include | —（`external_force_directional` 经 `mb_force/functions.hpp`） |
| `mb_linearization → mb_input` | 仅 header | `SampleInput`（签名用） |
| `mb_linearization → mb_joint` | symbol_reference + tu_include | `constraint_jacobian`, `constraint_jacobian_directional` |
| `mb_linearization → mb_model` | symbol_reference + tu_include | `ensure_directional_state`, `reset_directional_state`, `static_rotation_gauge_for_pivot` |
| `mb_linearization → mb_numeric` | symbol_reference + tu_include | `finite_vec` |
| `mb_linearization → mb_tire_common` | symbol_reference + tu_include | `tire_contact_compression_directional` |
| `mb_solve_static → mb_linearization` | tu_include | —（R7 委托点） |

`mb_linearization → mb_input` 是**header-only**（源码无调用），已如实登记为
`tu_include` 而非伪造 `symbol_reference`。

### 门的三段式拒绝（实测，非推测）

第一次跑 `--strict --final` 是 **FAIL**，三类 finding 同时出现：

```
[missing_source_evidence] mb_linearization -> mb_config has no source_edge_evidence record
[unreviewed_edge]         mb_linearization -> mb_config is not in the header-only baseline ...
[new_header_edge]         mb_linearization -> mb_config is not in the baseline header_edges
```

即：**改结构不会自动通过**。`--record-baseline` 按代码注释"never writes"策展字段，
`reviewed_new_edges` 必须人工逐条写理由；这也是本次唯一的基线编辑。

## 3. 数值未变：证据与"看似失败"的澄清

`dynamic_hash_sentinel.py --check` 输出：

```
artifacts hashed : 26
combined sha256  : 00000000000000000000000000000000
acceptance exit  : 1
failed cases     : ['combined_load', 'in_phase_road', ... 共 9 个]
OK: dynamic output matches the frozen baseline byte-for-byte
```

**必须澄清，否则会被误读为回归**：`acceptance exit 1` 与那 9 个 failed case
是**冻结基线里本来就记录的**：

```
packages/suspension_multibody/tests/data/dynamic_hash_baseline.json
  acceptance_exit_code = 1
  artifact_count       = 26
  combined_sha256      = "00000000000000000000000000000000"
  failed_cases = ['combined_load','in_phase_road','large_amplitude_high_frequency',
                  'opposite_phase_road','road_pulse','road_sine',
                  'road_step_finite_rise','single_wheel_road','tire_liftoff_and_recontact']
```

冻结基线的 `acceptance_statuses` 里 `braking/driving/lateral_or_steering/
static_equilibrium` 为 `PASSED`，其余 9 个为 `FAILED`——这是**重构前就存在的
验收现状**，不是本次提取造成的。

### 一条被我自己纠正的误读（保留过程，防止再次误引）

我一度把 `combined sha256` 读成 `00000000000000000000000000000000`（全零），
并据此推断 sentinel 的比对是"空真"。**这是错的**：PowerShell 控制台把 64 字符的
十六进制串折行显示，我只看到了开头那段。用 `len()`/`set()` 复核后确认它是
**长度 64 的正常十六进制摘要**，字符集覆盖 `0-9a-f`：

```
repr = '00000000000000000000000000000000'   <- 控制台显示（折行截断，误导）
len  = 64
chars = ['0'..'9','a'..'f']
```

**正确结论**：`expected["combined_sha256"]` 与 `measured["combined_sha256"]` 都是
真实的 64 位摘要，sentinel 第 252 行的相等判定**是有效判定**，
`OK: dynamic output matches the frozen baseline byte-for-byte` **成立且非空真**。

我另外手工执行了第 257–270 行的逐 artifact 比对（该分支仅在摘要不等时才跑，
但独立跑一次可确认"相等"不是偶然）作为交叉验证：

```
baseline entries: 26   measured entries: 26
REAL per-artifact differences: 0
```

**结论**：26 个 artifact 的逐条 entry 与冻结基线**全等（0 处差异）**。
数值未变。这条证据成立。

`case_parity_check.py` 的独立佐证：`axle_dynamic PASS 13 cases, bit-identical to
the frozen snapshot`、`kc_quasi_static PASS worst error / tolerance 0.000185928 over
the frozen K/C snapshot`。静态路径的数值确实没动。

### 性能：实测上升，仍在预算内

| 基准 | 本次 | 冻结基线 | 比值 |
|---|---|---|---|
| k-100 | 1.1875 s | 1.0230 s | ×1.161 |
| c-66 | 1.7672 s | 1.4904 s | ×1.186 |

`kc_perf_gate.py` exit 0（"benchmarks are within the recorded budget"）。
如实记录：多了一层函数调用边界，热循环里可测到约 16–19% 的上升。
未重录基线，未放宽预算。

## 4. 构建台账（不登记就等于"编译不到"）

`packages/suspension_kernel/tests/test_binding.py::test_build_write_metadata_records_added_keys`
实测**先失败**，报出恰好一个缺口：

```
Extra items in the left set:
  'packages/suspension_kernel/cpp/include/mb_linearization/types.hpp'
```

这条测试的判据是"磁盘上的头文件集合 ⊆ 构建台账"，正是防"新文件悄悄不参与编译"。
补齐 `build.py` 的 `SOURCE_RELATIVES` 两处后，第二次跑改为：

```
assert 133 == 135
```

即 `native_build.json` 是**构建时写入**的，改台账后必须重新构建才会一致。
重跑 `build_suspension_kernel.py` + `build_axle_native.py` 后 **176 passed**。

**这条链本身是重要发现**：`SOURCE_RELATIVES`（源码真源）→ CMake（编译真源）
→ `native_build.json`（构建产物台账）三者必须同时更新，漏任一处都会被这条测试逮住。

## 5. 未做与缺口

- **`assemble_physical_tangent` 暂无 Python 侧专用断言**。本步的性质是纯搬移，
  行为证据由 `gate-numeric` 的逐位一致承担（这是更强的证据：不是"新算对了"，
  而是"旧数没变"）。面向新能力的断言属于步骤 3/4（平衡响应与纯算子），
  届时会有真正的新行为可断言。此处不写一个只为凑数的等价测试。
- 曾起草一个 C++ 探针（`cpp/tests/linearization_equivalence.cpp`），
  但它只能构造最小模型，**验不到真实物理**；留着会给人"已验"的错觉，
  故移出源码树，不登记进构建台账。
- `mb_linearization` 目前只有静态路径的调用方。动态路径的共享收敛留待
  步骤 2（多 RHS / 转置）与 P3 评估；本步不预改动态求解路径。

## 6. 本步命令与退出码

| 命令 | 退出码 | 说明 |
|---|---|---|
| `build_suspension_kernel.py` | 0 | 新增 `mb_linearization` 目标，编译+链接通过 |
| `build_axle_native.py` | 0 | 同步 multibody 侧 DLL 镜像（否则 `NativeKernelUnavailableError`） |
| `check_module_layering.py --strict --final` | 0 | 0 环 / 0 互反边 / 0 跨聚合 include |
| `dynamic_hash_sentinel.py --check` | 0 | 逐位一致 |
| `case_parity_check.py` | 0 | 8 族接受 |
| `kc_perf_gate.py` | 0 | 预算内 |
| `ruff check .` | 0 | All checks passed |
| `ty check .` | 0 | All checks passed |
| `legacy_surface_gate.py --check` | 0 | 0 findings |
| `check_composable_release.py --skip-isolation` | 0 | 3 release checks passed |
| 快速集（excl. adams/architecture/cases） | 0 | **1715 passed, 1 xfailed**（未增长） |
| `pytest packages/suspension_kernel/tests packages/suspension_contracts/tests` | 0 | **176 passed** |
| `git diff --check` | 0 | 干净 |

## 7. R7 的当前状态（口径必须准确）

R7 是"原处只委托新共享组装"。本步实测完成的委托点是 **`static_jacobian`**
（`kernel_static_trim.cpp:358`）：它已不含组装算术，改为构造 `WorkingSample` 并
调用 `assemble_physical_tangent`。

`src` 全树 `static_jacobian` 的出现点实测为三处，已逐个定性：

| 位置 | 性质 |
|---|---|
| `cpp/include/mb_solve_static/functions.hpp:53` | 声明 |
| `cpp/src/solve_static/kernel_static_trim.cpp:358` | **定义**（现为委托体） |
| `cpp/src/solve_static/kernel_static_contact.cpp:315` | **调用点**（`if (!static_jacobian(...))`），非第二份实现 |

即：**组装算术目前只有一份归属**（`mb_linearization`），没有并行副本。
这是 R7"唯一归属"在 `static_jacobian` 这一条上的实测结论。

**尚未完成**：R7 的完整台账还需在步骤 6 覆盖其余潜在重复点
（如 `kernel_linear.cpp` 的 `solve_linear` 与其他 `mb_linear` 入口），
并按 SPEC §43"不移除其他用途 solve_linear"核对边界。本步只声明已实测的部分。
