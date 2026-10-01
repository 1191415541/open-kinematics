# PROGRESS：p5-01 冻结现状事实与判据（阶段五）

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p5-01`

## 状态

`DONE`（2026-10-01）。五条判据全部实测，证据落 `raw/`。本行是**纯只读冻结行**：未写任何生产代码，
未改 `packages/**`、`EPIC.md`、`SUBTASKS.csv`。

## 交付物

| 判据 | 文件 | 内容 |
|---|---|---|
| (1) 公共面与调用者 | `raw/public_surface.md`（188）、`raw/callers.md`（208） | `__all__` 17 名与 `_PUBLIC_NAMES` 16 条逐名带行号、`run_case`/`run_dynamic_case` 签名原文、`def simulate` 零命中、`FrontAxleModel` 19 字段、调用者逐 file:line 与两级计数 |
| (2) 三个门禁 | `raw/gates.md`（272） | 8 条规则名 + 两模式常量 + 关键常量原文；allowlist 与 registry 原文（strict + 零条目）；实跑 `findings : 0` |
| (3) 字段形状与哈希口径 | `raw/public_surface.md` §5 | 19 字段清单、`canonical_hash` 定义原文、实测哈希值与键集合（p5-02 的对照基准） |
| (4) 六项不存在事实 | `raw/absent_features.md`（297）、`raw/outputs_vs_channels.md`（268） | F19–F23 每类的真实 grep 命令与命中数 |
| (5) 起点门禁与自证 | `raw/run_log.md`（140） | 每条命令 + 退出码 + 关键输出；未跑项与并发告警 |

## 实测要点（供 p5-02 ~ p5-05 使用）

- **`__all__` = 17 名，`_PUBLIC_NAMES` = 16 条**。`__version__` 在 `__init__.py:38` 直接定义、
  不经 `__getattr__`，所以 `_PUBLIC_NAMES`（`:43-60`）比 `__all__`（`:63-79`）少一个。
  F24 与 SPEC 写「`_PUBLIC_NAMES` 与 `__all__` 有 17 个名字」——**只在 `__all__` 侧成立**，本行按实测记。
- **`model_hash` 口径与基准值**（p5-02 的硬门）：
  `model_hash = canonical_hash(model.model_dump(mode="json"))`（`api.py:116`、`:290-291`）；
  `canonical_hash` = `sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), default=str).encode("utf-8"))`
  （定义处 `:16-21`）。对既有夹具实测 **`72bd9f30330cce22fe672fc63c4248df09261928942443e2aedd98102b38c60b`**，键集合 **19**。
- **公共 API 门禁全绿且零容忍**：`legacy_surface_gate.py` 实测 **8 条规则**（`legacy_module_import`、
  `legacy_module_dynamic_import`、`report_native_import`、`report_native_call`、`report_preparation_import`、
  `report_preparation_call`、`report_constitutive_call`、`legacy_forwarding_shell`；两模式 `MODE_MIGRATION`/
  `MODE_FINAL` 在 `:107-108`）。`--check` 与 `--check --final` 各实跑一次均 `findings : 0`、退出码 0。
  allowlist `mode = "strict"` 且**零 `[[entry]]`**；`legacy_surface_registry.json` 的 `entry` = `[]`。
  内核提交唯一归属仍是 `simulation/backend.py:24`（生产侧唯一 `run_contract`）。
- **起点门禁实跑**：`pytest tests/api tests/architecture -q -p no:cacheprovider` → 退出码 **0**，
  **175 passed**，**0 failed / 0 skipped / 0 xfailed**。
- **F23 是 D2 的硬约束（本行实测确认）**：全仓 `kernel_step|run_step|step_once|read_state|get_state|set_state|
  advance_one` **零命中**；`kernel/native.py` 只绑 `suspension_kernel_run` / `_contract_version` / `_capabilities`
  三个符号；Python 侧唯一同步入口是 `kernel/__init__.py:186 run_contract`，状态经 `:174 _read_blocks` 从返回文档
  `blocks["body_state"]` 反序列化（布局 `:14-16` 与 `:32`）。→ **批式 ABI 无单步入口，实时闭环在当前 ABI 下无法实现**。
  p5-04 必须按 D2 的裁决处置（若需单步接口，先新增专属子任务，不得只登记缺口）。
- **F19 成立且比 EPIC 描述更明确**：`outputs/builtin.py` 的 `ASSEMBLY_OUTPUTS`（`:154`）与 `RIG_OUTPUTS`（`:220`）
  全部命中**都在该文件自身**（定义 + `__all__`），**零外部消费者**；`results/channels.py` 的 `ChannelRegistry`
  是冻结的 Adams 通道表。p5-03 必须定义「谁是真源」。

## 未做（本行边界，已记入 `raw/run_log.md`）

- **未跑 `check_composable_release.py`**：其第 4 项检查要构建三个 wheel 并在仓库外装 venv 跑一次真实 K 解，
  超出只读冻结行的写范围预算。**这不是红**——`p5-02` 与 `p5-06` 的验收命令会跑它。
- 未跑全量测试、`tests/architecture` 整目录、`just gate-numeric`（均按只读行约束）。

## 并发告警（必须随行传递）

本行执行期间，另一执行体把 ABI 版本常量从 `16/31` 改成 **`17/32`**（p2-02 方向），而已构建镜像
`native_build.json` 仍是 `16/31`（未重建），导致**该时段内任何触发 `load_library()` 的调用抛
`KernelAbiMismatchError`**。这是 p2-02 的中间态，重建内核后应恢复。本行证据全部是源码文本级与
AST 级读取，**不受该中间态影响**；本行也确实未修改任何 `packages/**` 文件。
