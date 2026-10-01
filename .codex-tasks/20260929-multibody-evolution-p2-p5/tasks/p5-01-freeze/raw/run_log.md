# p5-01 冻结现状 · 实际执行的命令与退出码

会话 scratch：`$PI_SCRATCH_DIR` = `C:\Users\zzy11\.pi-desktop\scratch\ba123231-caeb-4061-b2dc-c970cf431e16`。
本行**只写** `tasks/p5-01-freeze/raw/**` 与会话 scratch；`packages/**` 与 `tests/**` 未产生任何 diff（见 §E）。

---

## A. 门禁实跑

| # | 命令 | 退出码 | 输出末尾关键行 |
|---|---|---|---|
| 1 | `uv run --no-sync python packages/suspension_multibody/tests/architecture/legacy_surface_gate.py --check` | **0** | `mode : migration` / `findings : 0` / `OK: no unregistered Python boundary violation` |
| 2 | `uv run --no-sync python packages/suspension_multibody/tests/architecture/legacy_surface_gate.py --check --final` | **0** | `mode : final` / `findings : 0` / `OK: no unregistered Python boundary violation` |
| 3 | `uv run --no-sync pytest packages/suspension_multibody/tests/api packages/suspension_multibody/tests/architecture -q -p no:cacheprovider` | **0** | `175 passed in 632.58s (0:10:32)`（0 failed / 0 skipped / 0 xfailed；日志中 `skip` 出现 0 次） |

日志：`$PI_SCRATCH_DIR/p501/pytest_api_arch.log`。

## B. 静态事实采集（grep 原文见各证据文件）

| # | 命令 | 结果 |
|---|---|---|
| 4 | `grep -rn "def simulate" packages src tests scripts docs` | 命中 **0**，exit 1 |
| 5 | `grep -rn --exclude-dir={.git,.venv,.venv-build-gui,.mindfs,build,dist,.codex-tasks,node_modules,.pi} "def simulate" .` | 命中 **0**，exit 1 |
| 6 | `grep -rn --include=*.py "FrontAxleModel" packages/*/src src` | 82 行 / 25 文件 |
| 7 | `grep -rl --include=*.py "FrontAxleModel" <4 个 tests 根>` | 53 文件 |
| 8 | `grep -rl --include=test_*.py "FrontAxleModel" packages/suspension_multibody/tests` | **48 文件**（与 F18 一致） |
| 9 | `grep -rn --include=*.py "\brun_case\b" packages src tests scripts` | 54 行 / 17 文件 |
| 10 | `grep -rn --include=*.py "\brun_dynamic_case\b" packages src tests scripts` | 21 行 / 10 文件 |
| 11 | `grep -rn --include=*.py "\brun_contract\b" packages src tests scripts` | 定义 1 + 生产调用 1（`simulation/backend.py:24`） |
| 12 | `grep -rnE "class SignalBus\|class Measurement\|class Sensor\|signal_bus\|SignalBus" …` | **0**，exit 1 |
| 13 | `grep -rniE "damper_ratio\|variable_damp" …` | **0**，exit 1 |
| 14 | `grep -rnE "\babs\b" --include=*.py packages src tests scripts` | 457 行，**无一条指 ABS**（全是 `abs()`/`np.abs`/`abs=`/图例文本） |
| 15 | `grep -rniE "\besc\b" …` / `grep -rnE "\bPID\b" …` | **0** / **0**（`\bpid\b` 50 处全是 point-id 变量名） |
| 16 | `grep -rniE "controller" …` | 4 行，全部是积分器 local-error 步长控制器 |
| 17 | `grep -rniE "fmi\|fmu\|cosim\|co-simulation\|co_simulation" …` | 全仓含 artifacts 518；排除 artifacts **5**；其中代码/文档仅 3 处，全在路线图 md |
| 18 | `grep -rniE "kernel_step\|run_step\|step_once\|read_state\|get_state\|set_state\|advance_one" <abi 头 + python kernel>` | **0**，exit 1 |
| 19 | `grep -rn "ASSEMBLY_OUTPUTS" --include=*.py .` / `grep -rn "RIG_OUTPUTS" …` | 各 2 行，均在 `outputs/builtin.py` 内（定义 + `__all__`），**零外部消费者** |
| 20 | `grep -rn "ChannelRegistry" --include=*.py .` | 6 行（定义 2、包内再导出 2、测试 2） |
| 21 | `grep -rn "declared_names" --include=*.py .` | 1 行（只有定义，零消费者） |
| 22 | `cat -n .codex-tasks/20260919-public-api-simulation-cutover/tasks/01-boundary-inventory/LEGACY_ALLOWLIST.toml` | 15 行，`mode = "strict"`，**无 `[[entry]]` 段** |
| 23 | `uv run --no-sync python -c "import json; …legacy_surface_registry.json…"` | `entry` = `[]`，长度 0，exit 0 |

## C. 形状与哈希实测（脚本写会话 scratch，不入仓库）

| # | 命令 | 退出码 | 输出 |
|---|---|---|---|
| 24 | `uv run --no-sync python "$PI_SCRATCH_DIR/p501/hash_probe.py"` | **0** | `model_dump keys` 19 个；`model_dump bytes 1007`；`canonical_hash 72bd9f30330cce22fe672fc63c4248df09261928942443e2aedd98102b38c60b` |

脚本内容（`$PI_SCRATCH_DIR/p501/hash_probe.py`，只读夹具）：
```python
from suspension_multibody.schema import FrontAxleModel
from suspension_multibody.io import canonical_hash
payload = json.loads(Path("packages/suspension_multibody/tests/data/benchmark_axle.json").read_text())
model = FrontAxleModel.model_validate(payload["model"])
print(canonical_hash(model.model_dump(mode="json")))
```

| # | 命令 | 退出码 | 输出 |
|---|---|---|---|
| 25 | `uv run --no-sync python -c "import yaml; …axle_channels.yaml…"` | **0** | `schema_version 1` / `channels 33` / `required_role_bindings 12` / `total lines 119` |

**一条失败过的尝试（如实记录）**：`uv run --no-sync python -c "from suspension_multibody.adams.axle_contract import load_axle_channel_contract; …"`
→ **exit 1**，`KernelAbiMismatchError: abi_version reports ABI 16, expected 17; rebuild the kernel or install the matching wheel`。
原因：该模块 import 链会经 `kernel/capabilities.py:58` 触发 `load_library()`；而本行运行期间，工作区的
`mb_config/version.hpp` 与 `kernel/native.py` 的版本常量被**另一个并发写入者**从 `16/31` 改成了 `17/32`，
已构建的 native 镜像（`native_build.json`）却仍是 `16/31`。改用 `yaml.safe_load` 直接读 YAML（#25）绕开加载器，事实不变。

## D. 并发写入告警（重要，供主代理处置）

本行（只读）运行期间，工作区出现**不属于本行**的改动：

```
$ git status --porcelain -- packages/ src/ tests/ scripts/ docs/
 M packages/suspension_kernel/cpp/include/mb_config/element_wrench.hpp
 M packages/suspension_kernel/cpp/include/mb_config/version.hpp
 M packages/suspension_kernel/cpp/include/mb_element/functions.hpp
 M packages/suspension_kernel/cpp/include/mb_input/types.hpp
 M packages/suspension_kernel/cpp/include/mb_model/types.hpp
 M packages/suspension_kernel/cpp/src/abi/kernel_contract_run.cpp
 M packages/suspension_kernel/cpp/src/assembly/element_reader.cpp
 M packages/suspension_kernel/cpp/src/config/kernel_config.cpp
 M packages/suspension_kernel/cpp/src/element/anti_roll.cpp
 M packages/suspension_kernel/cpp/src/element/directional.cpp
 M packages/suspension_kernel/cpp/src/force/external_vector.cpp
 M packages/suspension_kernel/cpp/src/output/kernel_output.cpp
 M packages/suspension_multibody/src/suspension_multibody/kernel/native.py
?? packages/suspension_multibody/docs/multibody_architecture_evolution.md
```
```diff
$ git diff -- packages/suspension_multibody/src/suspension_multibody/kernel/native.py
-_NATIVE_KERNEL_ABI_VERSION = 16
-_NATIVE_VEHICLE_KERNEL_ABI_VERSION = 31
+_NATIVE_KERNEL_ABI_VERSION = 17
+_NATIVE_VEHICLE_KERNEL_ABI_VERSION = 32
```
```
$ grep -n "kAxleKernelAbiVersion\|kVehicleKernelAbiVersion" packages/suspension_kernel/cpp/include/mb_config/version.hpp
33:inline constexpr int kAxleKernelAbiVersion = 17;
40:inline constexpr int kVehicleKernelAbiVersion = 32;

$ git show HEAD:packages/suspension_kernel/cpp/include/mb_config/version.hpp | grep -n kAxleKernelAbiVersion
27:inline constexpr int kAxleKernelAbiVersion = 16;
```
文件 mtime：`version.hpp` 02:41:54、`native.py` 02:42:39 —— **正好落在本行 pytest 运行（02:33 起、02:43:10 结束）的中段**。
本行从未编辑这两个文件（本行工具只做 read/grep/sed/cat/mkdir 与 raw/ 与 scratch 下的写）。
**含义**：这是 EPIC $D1$ 授权的一次 ABI 变更（`EPIC.md:60/233`），正在被另一并发执行体（p2-02 方向）落地；
但已构建的 native 镜像尚未同步（`native_build.json` 仍 `abi_version=16`/`vehicle_abi_version=31`，mtime 01:08），
所以此刻**任何触发 `load_library()` 的调用都会失败**。这会影响本 Epic 后续所有子任务的运行期门禁
（`just gate-numeric`、`just test-all`、`pytest tests/architecture` 里依赖 native 的用例）。
本行落盘的证据全部是**源码文本级**（grep / AST / 文件读取 / 纯 schema 哈希），**不受该漂移影响**；
唯一受影响的是 §C 里如实记录的那条失败尝试。

## E. 本行写范围自检

```bash
$ git status --porcelain -- packages/ src/ tests/ scripts/ docs/
（输出见 §D —— 全部是并发写入者的改动，无一条来自本行）
$ git status --porcelain -- .codex-tasks/20260929-multibody-evolution-p2-p5/
 M .codex-tasks/.../EPIC.md                       ← 本行开始前即已 M（见下）
 M .codex-tasks/.../SUBTASKS.csv                  ← 同上
 M .codex-tasks/.../tasks/p2-01-freeze/PROGRESS.md ← 并发
 M .codex-tasks/.../tasks/p2-01-freeze/TODO.csv    ← 并发
 M .codex-tasks/.../tasks/p3-01-freeze/PROGRESS.md ← 并发
 M .codex-tasks/.../tasks/p3-01-freeze/TODO.csv    ← 并发
?? .codex-tasks/.../tasks/p2-01-freeze/raw/       ← 并发
?? .codex-tasks/.../tasks/p3-01-freeze/raw/       ← 并发
?? .codex-tasks/.../tasks/p4-01-freeze/raw/       ← 并发
?? .codex-tasks/.../tasks/p5-01-freeze/raw/       ← 本行（唯一）
```
**本行对 `EPIC.md` / `SUBTASKS.csv` / 任何 `tasks/*/TODO.csv` / `PROGRESS.md` / `SPEC.md` 零写入**；
本行唯一的写入是 `tasks/p5-01-freeze/raw/{public_surface,callers,gates,absent_features,outputs_vs_channels,run_log}.md`
与会话 scratch 的 `$PI_SCRATCH_DIR/p501/`。

## F. 未执行项（如实登记）

- `packages/suspension_multibody/scripts/check_composable_release.py`：**未跑**。第 4 项会构建 wheel 并安装 scratch venv
  （写产物、耗时长），超出只读冻结轮判据；本行的 `validation_command`（SPEC.md:90）不含它。**状态「未跑」，不是红。**
- 全量测试、`just gate-numeric`、`tests/architecture` 整目录：按任务约束**不跑**。
- 未修改任何冻结基线（`tests/data/kc_baseline/`、`dynamic_hash_baseline.json`、`vehicle_dynamics_baseline/sha256.json`、
  `kc_perf_baseline_native.json`）；未新增 skip/xfail；未将任何其它子任务置为 DONE 或 IN_PROGRESS。
