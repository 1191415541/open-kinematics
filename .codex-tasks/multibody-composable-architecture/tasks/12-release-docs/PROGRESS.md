# 12 清理、文档与打包

## Recovery

- 任务：`12 清理内部重复路径并验证文档与产物`。形态：single-full。依赖 11（DONE）。
- 状态见 `../../SUBTASKS.csv` 第 13 行；输入规格 `../../TASKS.md` 第 12 节。
- 主验收：`uv run --no-sync python packages/suspension_multibody/scripts/check_composable_release.py`；补充 ruff/ty 与两侧分层门。

## 实测现状（进入本任务时）

- 边界门 `legacy_surface_gate.py --check` 退出码 0（8 项已登记）；`--check --final` 退出码 1。
- 02 留下两个**转发壳**，真身已迁至 `modeling/primitives/`：
  - `preparation/geometry.py` → `modeling/primitives/spatial.py`
  - `preparation/assembly/types.py` → `modeling/primitives/joints.py`
  它们有 5 处生产调用者与 14 处测试调用者。
- `core/`、`model/`、`metrics/` 三个旧包目录只剩 `__pycache__`，git 不跟踪；`--final` 仍把它们报为「legacy package still present」（它扫目录）。
- 文档停留在 08 的结论：根 `CONTEXT-MAP.md` 仍写「实施未启动」，`packages/suspension_multibody/README.md` 仍把 `core/` 列为 A1 保留项，`packages/suspension_kernel/README.md` 仍说 `mb_suspension`/`mb_static` 存在，`MODULES.md` 的 `mb_cases` 一行早于 08 的单表收敛，`CONTEXT.md` 的术语表没有 family/plan/view/emitter。
- 没有任何脚本做过「构建三包 → 隔离安装 → 离开源码路径跑一次 native」。

## 做了什么

### 1. 删除已无生产调用者的内部重复路径

把全部调用者从两个转发壳切到 `modeling/primitives/`：5 处生产（`preparation/assembly/{front_axle,vehicle}.py`、`preparation/vehicle_dynamic.py`、`preparation/assembly/__init__.py`、`scripts/case_parity_check.py`、`scripts/kc_native_c_probe.py`）与 14 处测试。随后删除两个文件。同步修正 6 处指向旧路径的散文注释（`joints/table.py`、`joints/__init__.py`、`results/geometry.py`、3 个 `tests/core/*`）。

删除 `core/`、`model/`、`metrics/` 三个只剩 `__pycache__` 的目录。`core/` 的删除没有连带影响保留项：`elements/` 的本构依赖在 08 已切到 `modeling/primitives/`（见 `elements/{assembly,elastic}.py` 的 import），不是 `core/`。

**删除范围严格限于本次改动造成、且已无调用者的路径。** `elements/`、`analysis/` 有现役生产调用者，保留并登记理由与解除条件（见下）。

### 2. 现状文档

| 文件 | 改动 |
|---|---|
| `packages/suspension_multibody/README.md` | 「Python 模块结构」整节重写：按依赖方向分层，列出 `modeling/`（含 primitives）、`templates/`、`connections/`、`rigs/`、`subsystems/`、`compilation/`、`studies/`、`schema/`、`results/`、`outputs/`、`report/`、`io/`、`simulation/`、`cases/`、`preparation/` 与两个保留项；新增「仍保留 elements/ 与 analysis/ 的理由」表（现役 import 点 / 阻断原因 / 解除条件）与「扩展示例」小节 |
| `CONTEXT-MAP.md` | `suspension_multibody` 条目改写为分层现状；「Composable Multibody Architecture Plan」从「planning deliverable, not implemented」改为「已实施，迁移记录在 EPIC/SUBTASKS」 |
| `packages/suspension_kernel/README.md` | 元素/求解层改名为 `mb_element`、`mb_tire*`、`mb_solve_*`；显式说明 `mb_suspension`/`mb_static`/`mb_vehicle` 已不存在 |
| `packages/suspension_kernel/MODULES.md` | `mb_cases` 一行补 08 的单表事实（`family_table.hpp` 的 `FamilyRow` = 名称 + 协议已知 + expander）；扩展点一节补「已知未实现 vs 未知」两种拒绝 |
| `packages/suspension_multibody/CONTEXT.md` | 术语表补 `Family`、`Solve Plan`、`Model View`、`Emitter` 四条 |

### 3. 可执行扩展示例

新增 `packages/suspension_multibody/docs/composable_extension_examples.md`：E-1 新增子系统模板（builder 产出可变构件数）、E-2 新增试验台并跑通既有拓扑（真实 native 求解）、E-3 硬点更新传到求解。文档声明了 `python runnable` 围栏约定，由发布探针抽取执行。

### 4. 发布探针

新增 `packages/suspension_multibody/scripts/check_composable_release.py`（四项检查）与 `packages/suspension_multibody/tests/architecture/isolated_native_probe.py`（离开源码路径跑的那次 native K）。

关键设计：**不以 wheel 存在为通过**——构建 → 隔离安装 → 真正运行；安装用 `--offline`，缺依赖按阻断报告而不是下载；迁移登记表双向核对（新增发现失败，登记项消失也失败，因此只能收缩）。

## 关键实测

| 判据 | 实测 |
|---|---|
| 发布探针（全量） | 4/4 PASS，退出码 0 |
| ├ 迁移清单 | 8 项保留导入（5 生产 + 3 测试），3 个旧包已消失 |
| ├ 文档示例 | E-1/E-2/E-3 三块在独立解释器全部执行成功 |
| ├ 文档根 | 17 个记载模块存在、3 个记载为已退役的模块不存在 |
| └ 隔离安装 | 3 个 wheel 构建 + 离线安装 + 源码路径外一次 native K 收敛（残差 3.490e-07，`repo_on_path` 为空） |
| 边界门 `--check` | 退出码 0 |
| 边界门 `--check --final` | 退出码 1，仅剩 5 项生产 legacy import（`elements` ×4、`analysis` ×1）与 2 个 A1/A2 保留包，全部已登记并写出解除条件 |
| 内核分层门 `--strict --final` | 退出码 0 |
| ruff / ty | 均退出码 0 |
| 迁移后受影响测试树 | 420 passed, 1 xfailed |
| kc_parity | 通过，无漂移 |
| case_parity | 8 families PASS |
| dynamic_hash | 逐位不变（`e7407656731ed556efc28fb89d8fc69881725b3bfb2f39066eb898986389d48e`） |
| kc_perf_gate | k-100 x0.668、c-66 x0.656，预算内 |

## 未闭合项（如实登记，不伪装成通过）

### `legacy_surface_gate --final` 不能通过

`--final` 要求 5 个旧包全部消失、且零 legacy import。本任务后仍有两项能力**有现役生产调用、且 native 尚不能承载**：

| 保留项 | 阻断原因 | 解除条件 |
|---|---|---|
| `elements/`（A1） | native `element_wrench` 通道对固定体端早退（`cpp/src/element/assembly_primitives.cpp`），承载不了固定端反力；作者层因此必须自建元件 | 固定端事实口径裁定 + K 模式力元件声明 + 力矩参考点契约 |
| `analysis/`（A2） | native ABI 只导出 `suspension_kernel_run`，无静力求解入口，且导出面冻结 | 已解除：两个构造都是整车级派生量（静力轮荷是四未知量对三方程的欠定最小范数解，native 只分解方阵；侧倾中心是几何作图），迁入 `vehicle/` 后不再需要 ABI，目录已删 |

两者都属于 01 登记的 A1/A2 裁决，**不属于本任务的删除范围**（TASKS 第 12 节：删除仅限本次改动造成且已无生产调用者的内部重复路径）。因此本任务的通过标准是「登记表与实测发现双向一致」，而不是 `--final` 退出 0；`00` 的 EPIC 把 `--final` 通过列为 12 的目标，此处如实说明它不可达，理由与解除条件已写进现状文档。

### `check_composable_baseline.py --check` 失败（残留，非本任务引入）

三项：两个 native 库哈希与记录不符、5 个内核源文件（08/09 改的 `case_dispatch.cpp`、`family_table.hpp`、`kc_quasi_static.cpp`、`contract_registry.cpp`、`mb_cases/functions.hpp`）自基线采集后变动。

本任务**未改任何源码**，这三项在进入本任务前就已存在（08/09 的合法内核改动 + native 重构建）。另外实测：同一份源码连续两次构建得到的 DLL 哈希不同，**native 构建本身不是逐位可复现的**，所以 01 记录的库哈希无法在任何重构建后匹配——这是 01 基线的记录口径问题，不是回归。

判据上真正有约束力的是逐位数值门，它们全部通过：dynamic_hash 逐位不变、kc_parity 无漂移、case_parity 8 families PASS、kc_perf_gate 预算内。**没有重录任何基线。**
