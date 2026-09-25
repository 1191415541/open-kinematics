# 01 基线、风险与可执行门禁

## Recovery

- 任务：`01 冻结现状与可执行基线`。形态：single-full。
- 父任务：`.codex-tasks/multibody-composable-architecture`。
- 状态：见 `../../SUBTASKS.csv` 第 2 行（唯一实施真源）。
- 输入：`../../TASKS.md` 第 01 节 + 「基线与终局公共命令集」。
- 本目录文件：`SPEC.md`、`TODO.csv`、`PROGRESS.md`、`BASELINE.json`（自动采集）、`COMMANDS.json`、`TOLERANCES.json`、`FINDINGS.json`、`collect_baseline.py`。
- 主验收：`uv run --no-sync python packages/suspension_multibody/scripts/check_composable_baseline.py --check`。

## 初始状态（实测）

- HEAD：`ed63dc5c0f0c4428b7b9f7fc96f656c67a35b28e`（`ed63dc5`，2026-09-24T17:03:23+08:00）。
- `git status --porcelain`：开工时为空——**无既有未提交修改**，因此本轮不存在"把用户改动误算成新回归"的风险。
- 环境：Windows-11-10.0.26200-SP0，Python 3.12.13，原生库 ABI 15 / core 1 / vehicle 30，Release，`-O3 -DNDEBUG -std=c++17 -flto=auto`，`-fno-fast-math`。
- 原生库镜像：kernel 侧与 multibody 侧两份 `suspension_kernel.dll`，实测内容一致，sha256 `1fdd241d2265807d3a588ee8d84135c9b78835fdd5235b7cf067b4a32b99d092`。
- 内核 C++ 源码指纹：132 个文件，combined sha256 `f15a8718228e451b…`（完整值见 `BASELINE.json`）。

## 命令集执行结果（实测）

全部 16 条命令在 HEAD `ed63dc5` 实跑，逐条退出码见 `COMMANDS.json`。摘要：

| 命令 | 退出码 | 结果 |
|---|---|---|
| build_axle_native | 0 | 通过 |
| kernel_layering（`--strict --final`） | 0 | 通过（0 环、0 反向边、0 legacy 模块） |
| kernel_tests | 0 | `21 passed` |
| contracts_tests | 0 | `27 passed` |
| multibody_tests | 0 | 全量通过（精确汇总行以实测为准） |
| dynamic_hash（`--check`） | 0 | 字节级一致，combined `e7407656731e…` |
| kc_parity（`--check`） | 0 | 在冻结快照容差内 |
| case_parity | 0 | 8 families 全 PASS |
| kc_perf | 0 | k-100 ×0.806、c-66 ×0.949，预算 1.25 |
| legacy_surface_gate（`--check`） | 0 | migration 模式通过（8 项已登记） |
| ruff / ty / 三个 uv build / git diff --check | 0 | 全部通过 |

参数核实：`kc_perf_gate.py` 的默认子命令即 `--check`；`legacy_surface_gate.py` 的 `--check` 为 migration 模式。TASKS 命令集写法与实际 CLI 一致，无需修正文档或 CSV。

## 本轮发现与处置

### 一次由我引入、已修复的环境故障

首次执行 `build_axle_native.py` 时，多体全量测试正在并发加载原生库，导致 `shutil.copy2` 报 `WinError 32`（文件被占用）；同时 kernel 侧的 `suspension_kernel.dll` 已被重写为新哈希而 multibody 镜像仍是旧哈希，触发 `kernel/native.py:84` 的 `require_fresh_mirror`，使约 68 个用例以 `NativeKernelUnavailableError` 报错。

处置：按错误信息指引重跑 `uv run python packages/suspension_multibody/scripts/build_axle_native.py`，两份镜像恢复一致。**受测路径无漂移**：重编译后 `dynamic_hash_sentinel --check` 的 combined sha256 仍为 `e7407656731e…`，与冻结基线逐位一致；`kc_parity --check` 与 `case_parity` 亦不变。这支持「已受测路径未观察到行为变化」，不构成对全部行为不变的证明。此前失败属环境状态，非代码回归。

教训已写入 `FINDINGS.json` LIM-7：构建原生库时不得有测试进程并发加载它；两份镜像必须内容一致。

### 两个目标相关缺口仍然存在（均属阻断项，已给归属）

- **GAP-1 → 07**：准静态路径的垂向轮胎未端到端消费。`api.py:754` 的 `_tire_compression` 文档原文说明它只回显零压缩（`CaseSpec` 无路面高度与轮胎半径），准静态装配与 workflow 全文无轮胎引用，实测 `run_case` 结果通道无任何垂向轮胎力项。对应 A6。
- **GAP-2 → 10**：无转向总成时 rack 接口未收缩。`cases/kc_quasi_static/contract.py:359` 用 `next(...)` 取第一个 `rack_*` 驱动名，无转向时抛 `StopIteration` 而非点名报错；`:288` 无条件建立 rack 平移驱动；实测 K 模式 drives 恒定含 `rack_displacement`。对应 A8。

### 非目标历史限制（保留，不豁免）

`FINDINGS.json` 登记 7 条：9 个动态工况 `time_convergence` 失败（仅 `fixture.force_z`，文档已如实记录）、`legacy_surface_gate --final` 未通过（5 个旧包 + 8 项 legacy 导入，归 12）、Adams 资源缺失导致的 skip、制动常数 0.1 来源未核实、偏心轮胎质量保持拒绝、一个 degenerate 制动夹具的 strict xfail、原生库镜像新鲜度门。

### 容差标准

`TOLERANCES.json` 登记 6 个权威来源（K/C parity 容差公式、case parity 精确/容差分级、动态哈希字节级 tier、性能 BUDGET_FACTOR 1.25、工况门限、严格 Adams 门）并覆盖 A1-A10 的 6 类量纲。其中质量/质心/惯量与状态导数两类由 04/06 与 08/09 建立，已在门禁中作为具名延期接受，不允许留空。

## 门禁设计

`check_composable_baseline.py --check` 校验**证据的完整性、自洽性与时效性**，不重复执行昂贵命令集（重跑归各行所有与 13）。它非零退出的情形，除「文件缺失」外全部经实测构造用例验证：

| 构造的弱证据 | 实测结果 |
|---|---|
| 两份 `suspension_kernel.dll` 不一致 | 拒绝（并给出 `build_axle_native` 修复指引） |
| 记录的镜像哈希与磁盘不符 | 拒绝 |
| 内核源码指纹漂移 | 拒绝 |
| 记录中列出已删除的源文件 | 拒绝 |
| 必跑命令缺失 | 拒绝 |
| 命令 `result` 非 `pass`（如 `missing`/`skipped`） | 拒绝 |
| **任意**非零退出码（含曾想放行的 `dynamic_hash`） | 拒绝 |
| stub 条目（`command` 或 `observes` 为空） | 拒绝 |
| 测试命令缺数值汇总行 | 拒绝 |
| 容差权威名不存在（含 `kc_parity_tolerance-not-a-rule` 这类连字符后缀） | 拒绝 |
| 权威声明缺 `rule` | 拒绝 |
| GAP 缺 `still_present` | 拒绝 |
| 缺 GAP-1 或 GAP-2 | 拒绝 |
| GAP 归属写错（如把 GAP-2 写成 12） | 拒绝 |
| `non_goal_limits` 出现非字典条目 | 拒绝 |
| 冻结快照计数变化或文件摘要漂移 | 拒绝 |

评审后的两处收紧：非零退出码不再有按命令 id 的放行白名单（实测 16 条命令退出码全为 0，无需例外；`dynamic_hash_sentinel` 内部的 acceptance 退出码 1 记在 `observes` 文字里，不作为命令退出码混入）；容差权威匹配从子串改为标识符边界（连字符不再构成边界）。

## 验证记录（均为实测退出码）

| 命令 | 退出码 | 结果 |
|---|---|---|
| `uv run --no-sync python packages/suspension_multibody/scripts/check_composable_baseline.py --check` | 0 | 主验收通过 |
| `uv run --no-sync ruff check .` | 0 | All checks passed |
| `uv run --no-sync ty check .` | 0 | All checks passed |
| `uv run --no-sync python .codex-tasks/.../01-baseline/collect_baseline.py` | 0 | 证据重新采集，指纹一致 |
| `uv run --no-sync pytest packages/suspension_multibody/tests -q` | 0 | `1052 passed, 1 skipped, 1 xfailed in 1150.85s` |
| `git diff --stat` | 0 | 空——未修改任何已跟踪文件 |

## 独立复审与处理

code-reviewer 子代理（delegation `91dc5f2c-3c47-4e0e-9536-2c856e9ab1c4`）只读复审，结论为「不能在不放宽标准的前提下标为 DONE」，提出 5 项阻断、4 项中等、2 项低。全部处理如下：

| 意见 | 处置 |
|---|---|
| 非零退出码按 id 放行，无法区分内部 acceptance 码与命令自身失败 | 已移除白名单；任意非零退出码一律失败。实测确认 |
| 只要求退出码是整数，stub 条目可冒充通过 | 已加 `command`/`observes` 非空校验与 `result == "pass"` 要求；实测确认 |
| 源指纹不覆盖夹具与 Python 实现，冻结基线未与磁盘比对 | 已新增 4 个冻结文件的磁盘摘要校验（K/C 快照、动态哈希、性能基线），实测确认漂移被拒 |
| 质量/惯量与状态导数只有具名延期，没有误差值 | 已补齐两个权威：质量守恒取现有测试的**逐位一致**判据；状态导数取相对误差 ≤1e-6。6 类量纲现全部 `status=defined` |
| `TODO.csv` 步骤 5-8 仍 TODO，`PROGRESS.md` 指向不存在的「最终验证」节 | TODO 全部据实更新为 DONE；PROGRESS 补入真实退出码表，删除悬空引用 |
| 容差子串匹配可被 `-not-a-rule` 绕过 | 已改为标识符边界匹配，连字符纳入 token。实测确认 |
| 删除 GAP 的 `still_present` 即静默通过 | 已要求 `still_present` 必须是布尔、GAP-1/GAP-2 必须存在、归属须匹配 07/10。实测确认 |
| GAP-2 引用的 `StopIteration` 不是当前无转向路径的首个失败点 | **复审正确**。实测：删去 `rack_center` 后 `run_case(K)` 失败于 `subsystems/geometry.py:102` 的 `ValueError: missing required front-axle hardpoint for rack_center`。FINDINGS.json 已按实测更正，并保留旧任务原文作对照 |
| 「重编译无行为漂移」超出证据范围 | 已改为限定表述：受测路径（动态哈希、K/C parity、case parity）未观察到漂移，不声称全部行为不变 |
| `TODO.csv` 步骤 3/4 的 evidence 指向错文件 | 已改为 `COMMANDS.json` / `FINDINGS.json`，并注明具体节名 |

复审同时确认：`api.py:754` 零压缩回显与 `:532` 调用、`contract.py:287-293` 无条件建 rack 驱动均属实。

## 未覆盖与保留

- 复审的只读工具无法执行 shell，故写范围与实际退出码由主线程自查（`git diff --stat` 为空、主验收退出码 0）。
- 本轮未运行 02-13 的任何实现测试；`TASKS.md` 的实现命令是未来验收规范，不是已通过声明。
- `legacy_surface_gate --final` 当前失败（5 个旧包 + 8 项 legacy 导入）属 12 的目标，已登记。
