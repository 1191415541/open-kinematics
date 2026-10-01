# p5-01 冻结现状 · 三个公共 API 门禁的规则原文与现状

以下原文均本行实读；实跑命令与退出码见 `run_log.md`。

## 1. `tests/architecture/legacy_surface_gate.py`（565 行）

### 1.1 守什么

文件头（`:1-11`）：

```
1:"""
2:Python responsibility and deletion gate for ``suspension_multibody``.
...
9:The scanner is AST based and deliberately never imports the scanned module: it
10:reads and parses text only, so it can run on the production tree, on a script,
11:or on a fixture in a temporary directory.
```

### 1.2 规则名清单（模块 docstring `:13-34` + 代码实现）

| 规则名 | docstring 行 | 实现行 | 守什么 |
|---|---|---|---|
| `legacy_module_import` | `:16-19` | `:290` | 退役包外任何文件（绝对/相对/别名）导入 `core`/`elements`/`model`/`analysis`/`metrics` |
| `legacy_module_dynamic_import` | `:20-21` | `:290` | 同上，走 `importlib.import_module` / `__import__`（`:311-317`） |
| `report_native_import` | `:22-25` | `:295` | `report` 内导入 native/kernel/solver/axle_dynamics 或 `suspension_kernel` |
| `report_native_call` | `:22-25` | `:326` | `report` 内调用求解/原生入口（`NATIVE_TOKENS` 命中） |
| `report_preparation_import` | `:26-28` | `:297` | `report` 内导入 `preparation` |
| `report_preparation_call` | `:26-28` | `:332` | `report` 内调用 `PREPARE_NAMES`/preparation 接收者 |
| `report_constitutive_call` | `:29-31` | `:334` | `report` 内求值元素力律（`CONSTITUTIVE_NAMES`，`:94-105`） |
| `legacy_forwarding_shell` | `:32-34` | `:409` | 非包模块，整体只是 import + `__all__` + 再导出，且位于退役路径或转发退役包 |

关键常量原文：
```
65:LEGACY_PACKAGES = ("core", "elements", "model", "analysis", "metrics")
68:NATIVE_TOKENS = ("native", "kernel", "solver", "axle_dynamics")
69:SOLVE_NAMES = (
70:    "run_contract",
71:    "solve",
72:    "solve_static",
73:    "solve_dynamic",
74:    "run_solver",
75:)
77:PREPARATION_TOKENS = ("preparation",)
94:CONSTITUTIVE_NAMES = (
95:    "apply_element",
96:    "bushing_force",
97:    "constitutive_force",
98:    "damper_force",
99:    "element_forces",
100:    "evaluate_element",
101:    "evaluate_generalized_forces",
102:    "generalized_forces",
103:    "spring_force",
104:    "tire_force",
105:)
```

（SPEC.md:11 列的四条规则是 `legacy_module_import` / `report_native_import` / `report_constitutive_call` / `legacy_forwarding_shell`，均为上表子集，锚点成立。）

### 1.3 两种 mode 原文（`:36-53`，常量定义在 `:107-108`）

```
39:``MODE_MIGRATION``
40:    Registered legacy imports are tolerated while their owners migrate, but
41:    every live finding must be registered: an unregistered finding fails, and a
42:    registration without a finding fails as stale, so the registry can only
43:    shrink.  Findings in tests are reported, not enforced, because 08 removes
44:    them together with the packages.
45:``MODE_FINAL``
46:    Nothing is tolerated: a retired package that still exists, any import of it
47:    in any scope, a forwarding shell, and a native call from ``report`` all
48:    fail.
49:
50:The mode is an explicit CLI flag; there is no environment override.
51:
52:    python legacy_surface_gate.py --check              # migration mode
53:    python legacy_surface_gate.py --check --final      # end state
```
```
107:MODE_MIGRATION = "migration"
108:MODE_FINAL = "final"
```
（SPEC.md:11 写「两模式定义在 `:45`」对得上 docstring 里 `MODE_FINAL` 那段；**常量本身在 `:107-108`**。）

`--final` 决定 mode：
```
524:    parser.add_argument("--final", action="store_true")
538:    mode = MODE_FINAL if args.final else MODE_MIGRATION
```

### 1.4 现状（两种模式各实跑一次）

```bash
$ uv run --no-sync python packages/suspension_multibody/tests/architecture/legacy_surface_gate.py --check
mode      : migration
findings  : 0

OK: no unregistered Python boundary violation
exitcode=0

$ uv run --no-sync python packages/suspension_multibody/tests/architecture/legacy_surface_gate.py --check --final
mode      : final
findings  : 0

OK: no unregistered Python boundary violation
exitcode=0
```

**当前用哪种**：AGENTS.md 第 1 节的 `just check-fast` 用 `--check`（**migration**）；`check_composable_release.py` 同时跑两者（`run_boundary_gate(final=False)` 与 `(final=True)`，`:139-152`）。**两种模式都绿（findings 0）**。

## 2. `packages/suspension_multibody/scripts/check_composable_release.py`（550 行）

### 2.1 守什么

```
2:The release probe: whether the product works once it has left this repository.
```
机制（`:4-23`）：其它门只读源码树，因而可以在包安装后不可用时全绿；本探针问树内门**结构上问不到**的问题。

### 2.2 四项检查（docstring `:8-23` + `main` 列表 `:492-497`）

```
492:    checks = [
493:        "migration list -- the boundary findings are the registered ones",
494:        "documentation examples -- every `python runnable` block executes",
495:        "documentation roots -- the current-state docs match the tree",
496:        "isolated install -- three wheels build, install offline and solve natively",
497:    ]
```

1. **`migration list`**（docstring `:8-12`；实现 `:189-220`）：Python 边界门以 **`--final`** 跑（`run_boundary_gate(final=True)` `:139-152`），findings 与其 release condition 一并报告；登记表 `EXPECTED_BOUNDARY_FINDINGS`（`:72`，**当前为空元组**）——新 finding 失败、登记的却消失也失败（`:195-205`）；并核对 `RETIRED_PACKAGES`（`:88 = ("core","model","metrics","analysis")`）不得仍存在（`:206-212`）。
2. **`documentation examples`**（docstring `:13-15`；实现 `:249-281`）：抽取 `docs/composable_extension_examples.md` 里 ` ```python runnable ` 段（`FENCE` `:64`、`EXAMPLES` `:59`），写盘后在**全新解释器**里跑；跑不通即失败。
3. **`documentation roots`**（docstring `:16-17`；实现 `:298-336`）：`DOCUMENTED_PACKAGES`（`:91-108`，16 个模块目录）必须都在、`RETIRED_PACKAGES` 必须都不在；并核对 `README.md`/`CONTEXT.md`/`CONTEXT-MAP.md`/`docs/composable_extension_examples.md` 存在（`:321-330`）。
4. **`isolated install`**（docstring `:18-23`；实现 `:360-451`）：三个 wheel（`WHEEL_PACKAGES` `:111-115` = `suspension-contracts` / `suspension-kernel` / `suspension-multibody`）构建到 scratch，`uv pip install --offline` 装进 scratch venv，然后**在仓库外目录**跑一次真实 native K 解（`ISOLATED_RUN` `:61` = `tests/architecture/isolated_native_probe.py`），断言 `repo_on_path` 为空且 `converged`/`finite`。

明写不做的三件事（`:25-34`）：「不把 wheel 文件存在当证据」「不下载任何东西」「不重录数值基线」。
开关：`--list`（`:470-474`）、`--skip-isolation`（`:475-479`）、`--skip-native-rebuild`（`:480-484`）、`--scratch`（`:485-489`）。

**本行未实跑它**：第 4 项会构建 wheel 并装 venv（写产物、耗时长），超出只读冻结轮的判据；本行的 `validation_command`（SPEC.md:90）不含它。**状态：未跑**（不是红）。

## 3. `tests/architecture/test_public_api_boundary_gate.py`（334 行）

### 3.1 扫描范围与 allowlist 路径

```
 9:ROOT = Path(__file__).parents[4]
10:SOURCE_ROOT = ROOT / "packages" / "suspension_multibody" / "src" / "suspension_multibody"
11:SCRIPT_ROOT = ROOT / "packages" / "suspension_multibody" / "scripts"
12:TEST_ROOT = ROOT / "packages" / "suspension_multibody" / "tests"
13:ALLOWLIST_PATH = (
14:    ROOT
15:    / ".codex-tasks"
16:    / "20260919-public-api-simulation-cutover"
17:    / "tasks"
18:    / "01-boundary-inventory"
19:    / "LEGACY_ALLOWLIST.toml"
20:)
```
即 `.codex-tasks/20260919-public-api-simulation-cutover/tasks/01-boundary-inventory/LEGACY_ALLOWLIST.toml`，与 SPEC.md:11 写的路径逐字一致。
`_scan_repository()` `:216-223`：生产源码树 + 脚本树用**全规则**，测试树只用 `_TEST_RULES`。

### 3.2 规则名清单

| 规则名 | 触发点行 | 守什么 |
|---|---|---|
| `axle_native_facade_import` | `:148-149` | 导入 `axle_dynamics` 的 `native` 门面（`_is_native_facade_import` `:68-71`） |
| `direct_kernel_run_contract` | `:150-152`, `:173-177`, `:188-190` | 导入/调用 `run_contract`，除非是 backend 的 `run` |
| `direct_case_contract_import` | `:153-157` | 从 `cases*` 导入 `run_*_contract`（`_is_case_contract_name` `:52-53`） |
| `direct_case_contract_call` | `:191-194` | 调用 `run_*_contract`，除非调用点在 `src/.../cases/` |
| `direct_raw_decoder` | `:158-159`, `:195-196` | 导入/调用 `decode_contract_run`，除非是 `results/raw.py` / `results/decoder.py` |
| `legacy_artifact_writer` | `:160-166`, `:197-203` | 导入/调用 `write_bundle` / `write_dynamic_bundle` / `write_axle_dynamics_artifact` / `write_vehicle_dynamics_artifact` |
| `dynamic_bundle_reference` / `dynamic_bundle_creation` | `:167-168`, `:204-205` | 引用/构造 `DynamicResultBundle`，除非是历史兼容所有者 |

`_TEST_RULES`（`:98-111`）——**测试树也扫这五条**：
```
 98:_TEST_RULES = frozenset(
 99:    {
...
105:        "axle_native_facade_import",
106:        "direct_case_contract_call",
107:        "direct_case_contract_import",
108:        "direct_kernel_run_contract",
109:        "direct_raw_decoder",
110:    }
111:)
```
（SPEC.md:11 写的 `direct_kernel_run_contract` / `direct_raw_decoder` / `axle_native_facade_import` 三条均在列，锚点 `:13` 成立；另两条是同族 case-contract 规则。测试用例见 `:285-296`、`:299-312`。）

### 3.3 归属判定原文

SPEC.md:11/F24 写的 `_is_owner` 在 `:74` —— 实测该行是 **`_is_backend_owner`**（名字不同、位置对）： 
```
74:def _is_backend_owner(path: Path, scope: str, function_scope: str) -> bool:
75:    return (
76:        scope == "production"
77:        and path.as_posix().endswith("/simulation/backend.py")
78:        and function_scope == "run"
79:    )
```
另两个归属判定：
```
82:def _is_result_decoder_owner(path: Path) -> bool:
83:    path_text = path.as_posix()
84:    return path_text.endswith("/results/raw.py") or path_text.endswith(
85:        "/results/decoder.py"
86:    )
89:def _is_history_compat_owner(path: Path) -> bool:
90:    """Identify the historical-read boundary owners."""
91:    path_text = path.as_posix()
92:    return (
93:        path_text.endswith("/schema/loader.py")
94:        or path_text.endswith("/schema/__init__.py")
95:        or path_text.endswith("/adams/time_domain.py")
96:    )
```
唯一归属的独立断言：`test_backend_is_the_only_direct_kernel_submission_owner`（`:315-334`）。

### 3.4 allowlist 原文（全文 15 行）

```toml
 1:version = 2
 2:mode = "strict"
 3:source = "task 01 repository scan; cleared by task 08 after every listed bypass was deleted"
 4:root = "."
 5:
 6:# The gate is exact to path, scope, rule, symbol, and line, and in strict mode it
 7:# allows no entries at all: task 08 deleted the last compatibility facade, the
 8:# duplicate writers, the dead raw decoder export and the legacy K&C contract
 9:# runners, so the repository scan finds nothing.  Any new entry here is a
10:# regression rather than a record.
11:#
12:# Kept out of the scanner by construction instead of by an entry:
13:#   * `schema/__init__.py`, `schema/loader.py` and `adams/time_domain.py` are the
14:#     historical `DynamicResultBundle` read boundary -- they may name the type but
15:#     no new production path may create it (see `_is_history_compat_owner`).
```
- `mode = "strict"` ✅（`:2`）
- **条目数 0**：全文**没有任何 `[[entry]]` 段**（只有 `:1` 的 `version`、`:2-4` 的标量与注释）。测试断言同一件事：
```
258:    assert data["version"] == 2
259:    assert data["mode"] == "strict"
270:    assert len(entries) == len(_allowlist_keys()) == len(_scan_repository())
```
（strict 下「allowlist 条目数 == 扫描到的 violation 数」，前者 0 ⇒ 后者必须 0。）

### 3.5 `legacy_surface_registry.json` 的 `entry`

文件 `packages/suspension_multibody/tests/architecture/legacy_surface_registry.json`（28 行），键 `version=1` / `mode="migration"` / `notes` / `successors` / `entry`。末行原文：
```
27:  "entry": []
```
实跑复核：
```bash
$ uv run --no-sync python -c "import json; d=json.load(open('packages/suspension_multibody/tests/architecture/legacy_surface_registry.json')); print(list(d)); print(d['entry']); print(len(d['entry']))"
['version', 'mode', 'notes', 'successors', 'entry']
[]
0
```
**`entry` 为 `[]`，条目数 0** ✅（与 F24 一致）。`check_composable_release.py:72` 的 `EXPECTED_BOUNDARY_FINDINGS: tuple[tuple[str, str], ...] = ()` 同步为空。

## 4. 起点门禁实测（本行 `validation_command` 的 pytest 段）

```bash
$ uv run --no-sync pytest packages/suspension_multibody/tests/api packages/suspension_multibody/tests/architecture -q -p no:cacheprovider
........................................................................ [ 41%]
........................................................................ [ 82%]
...............................                                          [100%]
175 passed in 632.58s (0:10:32)
EXITCODE=0
```
**绿：175 passed，0 failed，0 skipped，0 xfailed**（日志里 `skip` 出现 0 次）。起点值供 p5-06 对照（EPIC.md 行 320）。
