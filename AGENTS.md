# AGENTS.md

本文件是本仓库的开发与验证约定。核心取舍：**日常迭代不跑慢测试**，用秒级的静态门和一套快速测试集保证质量；慢测试只在大改动收尾时跑。

背景：全量 `pytest packages/suspension_multibody/tests` 约 33 分钟，但其中约 28 分钟花在与代码正确性无关的重复计算上（`adams/` 与 Adams 参考做数值对标、`architecture/` 里一个开 506 个新进程的用例）。真正买到质量的部分约 6 分钟。

## 1. 日常迭代：必须跑的四件事（合计约 1 分钟）

一条命令即可（实测退出码 0、**52 秒**）：

```bash
just check-fast
```

展开为下面四步，逐步单独跑也可以：

```bash
# 1) 静态检查，秒级。任何改动都要先过。
uv run --no-sync ruff check .
uv run --no-sync ty check .

# 2) 秒级架构门（不经 pytest，直接跑脚本；各 1~3 秒）
uv run --no-sync python packages/suspension_multibody/tests/architecture/legacy_surface_gate.py --check
uv run --no-sync python packages/suspension_kernel/scripts/check_module_layering.py --strict --final
uv run --no-sync python packages/suspension_multibody/scripts/check_composable_release.py --skip-isolation

# 3) 快速测试集：除 adams/、architecture/、cases/ 外的全部目录
#    实测 854 passed / 1 xfailed，约 25 秒
uv run --no-sync pytest packages/suspension_multibody/tests \
    --ignore=packages/suspension_multibody/tests/adams \
    --ignore=packages/suspension_multibody/tests/architecture \
    --ignore=packages/suspension_multibody/tests/cases -q -p no:cacheprovider

# 4) 另两个包，实测 59 passed，约 11 秒。必须单独调用：把 kernel/contracts
#    目录并进上面的命令会改变 rootdir，使 suspension_multibody 测试里的
#    `from tests.benchmark_fixture import ...` 解析失败。
uv run --no-sync pytest packages/suspension_kernel/tests packages/suspension_contracts/tests \
    -q -p no:cacheprovider
```

用 `--ignore` 而不是逐个列目录，这样**新增的测试目录默认会被包含**，不会被静默跳过。

日常改动的验证到此为止。**不要**默认跑全量。

### 关于 `tests/architecture/`

`tests/architecture/` **不在日常测试集里**（上面的命令已排除它，跑一次要 10 分钟，其中 8.8 分钟是那一个开 506 个进程的用例）。

但架构约束本身不能失去保护，所以第 2 步保留的是**独立门禁脚本**而不是 pytest 目录——同一个检查，各 1~3 秒：

| 独立脚本 | 对应的 pytest 目录检查 |
|---|---|
| `tests/architecture/legacy_surface_gate.py --check` | `test_legacy_surface_gate.py` |
| `packages/suspension_kernel/scripts/check_module_layering.py --strict --final` | `test_module_layering_gate.py` |
| `scripts/check_composable_release.py --skip-isolation` | 迁移清单与文档现状 |

这三条是**本次重构的全部结构证据**（分层方向、模块 DAG、公共 API 提交唯一归属、退役面、ABI 单一真源）。改动结构、搬模块、删文件后**必须重跑**，否则架构漂移无人发现。

`tests/architecture/` 整个目录（149 个用例）只在**大重构收尾**时跑一次：

```bash
uv run --no-sync pytest packages/suspension_multibody/tests/architecture -q -p no:cacheprovider
```

## 2. 数值门：改动触及求解路径时跑（约 3.5 分钟）

```bash
just gate-numeric
```

展开即：

```bash
uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check   # 25 s，逐位
uv run --no-sync python packages/suspension_multibody/scripts/case_parity_check.py               # 162 s，8 个族
uv run --no-sync python packages/suspension_multibody/scripts/kc_perf_gate.py                    #   9 s，性能预算
```

这三条是「物理与性能没有变」的唯一证据，改动触及 `axle_dynamics/`、`kernel/`、`cases/`、轮胎/力元/求解路径时必须跑。

**不包含 `kc_parity_check.py`。** 它不带 `--actual-dir` 时是拿冻结快照与自身比较（0.4 s、恒过），不构成证据。若确实需要 K/C 等价对标，先跑生产者再判定（耗时较长，按需）：

```bash
uv run --no-sync python packages/suspension_multibody/scripts/kc_native_probe.py
uv run --no-sync python packages/suspension_multibody/scripts/kc_native_c_probe.py
uv run --no-sync python packages/suspension_multibody/scripts/kc_parity_check.py \
    --check --actual-dir artifacts/kc-native-probe
```

## 3. 收尾与提交前：跑全量（约 33 分钟）

```bash
just test-all
```

（即 `uv run --no-sync pytest packages/suspension_multibody/tests -q`。）只在准备提交、或改动跨越多个子系统时跑。当前基线：**1297 passed, 1 skipped, 1 xfailed**。skip 与 xfail 都有登记原因，**不得增长**。

`adams/`（约 15 分钟）验的是 PAC2002/Fiala 与 Adams 参考的数值等价，与架构/重构无关；`cases/`（约 5 分钟）验整车 K/C 网格语义。这两块按需跑：

```bash
uv run --no-sync pytest packages/suspension_multibody/tests/adams -q      # 只在改轮胎力律/对标时
uv run --no-sync pytest packages/suspension_multibody/tests/cases -q      # 只在改 cases/ 或 preparation 时
```

## 4. 质量由这几层保证（不要绕过）

| 层 | 成本 | 抓什么 |
|---|---|---|
| `ruff` + `ty` | 秒级 | 语法、死代码、未用导入、类型与接口漂移 |
| AST 静态门 | 每个 1~3 s | 分层依赖方向、内核模块 DAG、公共 API 提交唯一归属、退役面、ABI 单一真源 |
| 快速测试集（850 个） | 25 s | 契约、装配、编译、结果、API 的运行时行为 |
| 数值门 | 3.5 min | 物理结果逐位不变、性能预算 |
| 全量回归 | 33 min | 上述全部 + Adams 对标 + 整车 K/C |

**架构约束靠秒级 AST 门守，不靠跑程序猜。** 本仓库最关键的结构性质——低层 `modeling/` 不得反向导入作者层、内核提交只有一个归属者、ABI 版本单一真源、退役模块不得复活——全部由第 1 节里的脚本在几秒内判定。改了结构就必须重跑它们。

## 5. 提交前检查清单

- [ ] `ruff check .` 与 `ty check .` 退出码 0
- [ ] 第 1 节的三个架构门脚本退出码 0（`just gate-architecture`）
- [ ] 快速测试集全通过，且 skip/xfail 未增长
- [ ] 改动触及求解路径 → 第 2 节数值门全通过（`just gate-numeric`）
- [ ] 未重录任何数值/性能基线（除非用户明确授权；重录必须证明冻结数值逐位不变）
- [ ] `git diff --check` 干净

## 6. 已知的测试性能问题（可优化，尚未做）

以下是纯实现开销，**优化后覆盖率不降**，需要时按此修：

1. `tests/architecture/test_import_boundaries.py::test_every_ordered_pair_of_entry_points_imports` 为 23×22=506 个顺序组合各开一个**新解释器**，占 528 s。改为单进程内重放（清 `sys.modules` 里的产品模块）实测 **506 个顺序全跑完仅 76 s**，顺序数一个不少。改动前**必须先验证它仍能抓到真实的循环依赖缺陷**（此验证尚未完成，两次构造实验均未复现出真环）。
2. `tests/adams/test_pac2002_adams_correlation_gate.py` 内含 8 个 module 级 fixture，各自 `mktemp` 重算同一份 native 对比结果；`tests/adams/` 另有 3 个文件调用同一脚本。改为 session 级共享缓存（按输入哈希复用）约省 12 分钟，**断言一条不删**。

## 7. 不要做的事

- 不要为了「跑得更快」删测试或把断言改弱。能砍的是**重复计算**，不是判定。
- 不要重录 `kc_baseline/`、`dynamic_hash_baseline.json`、`kc_perf_baseline_native.json` 等冻结基线来掩盖回归。
- 不要用 `-k`、`--deselect` 长期豁免某个失败用例；失败要么修，要么按第 6 节说明登记原因。
- 不要在会话外并发改同一批文件：本仓库出现过两个写入者同时改 `tests/` 与 `src/`，导致测试报告中间态假失败。
