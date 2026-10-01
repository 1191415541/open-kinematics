# p5-01 冻结现状 · `outputs/` 静态声明 vs `results/channels.py` 的通道注册表

今天**两套互不相通**的东西：一套是 `outputs/` 的静态「声明」（谁产出什么），一套是 Adams 对标侧的冻结通道表。
以下均为本行实读/实跑。

---

## 1. `outputs/` 静态声明（定义处原文）

### 1.1 `outputs/declarations.py`（`declarations.py:1-16` 文件头原文）

```
1:"""
2:What a run declares it produces, before anything computes it.
3:
4:An output has an owner.  An assembly declares the outputs its subsystems produce;
5:a rig declares the ones its own driving and instrumentation add.  Neither knows
6:the other's list, and a run is only well defined once the two have been merged.
7:
8:"Minimum-unit" is the load-bearing phrase: these are the *smallest* things a run
9:produces -- a wheel centre, a tire force, a rack displacement -- not the numbers
10:a report wants.  Everything a report wants (camber, toe, KC gradients, stability
11:indices) is a derived output computed from these, which is what makes a new
12:report metric a declaration rather than a new code path.
13:
14:Declarations are data, so they serialise.  A rig author and an assembly author
15:have to be able to compare two lists on paper before a run exists.
16:"""
```
（SPEC/F19 写的锚点 `outputs/declarations.py:1` **成立**：文件首行就是这句定义性声明。）

关键类型/函数：
```
24:__all__ = [
25:    "DOMAINS",
26:    "OUTPUT_NAME_PATTERN",
27:    "DeclarationSet",
28:    "DeclarationError",
29:    "OutputDeclaration",
30:    "merge_declarations",
31:    ...
38:Domain = Literal["assembly", "rig", "derived", "merged"]
40:DOMAINS: tuple[str, ...] = ("assembly", "rig", "derived", "merged")
44:OUTPUT_NAME_PATTERN = r"^[a-z][a-z0-9_]*$"
47:class DeclarationError(ValueError):
52:class OutputDeclaration:
124:class DeclarationSet:          # 含 from_json(:186) / loads(:197)
202:def merge_declarations(*sets: DeclarationSet) -> DeclarationSet:
```

### 1.2 `outputs/builtin.py` 的两套静态声明

`ASSEMBLY_OUTPUTS`（定义处 `outputs/builtin.py:154`，注释在 `:151-153`）：
```
151:#: What the assembly's subsystems produce.  The tire force columns are the three
152:#: the legacy reader indexed out of `tire_output`; the poses are what the K&C
153:#: metrics read; the rest are the run-level facts the report passes through.
154:ASSEMBLY_OUTPUTS = DeclarationSet(
155:    domain="assembly",
156:    outputs=(
157:        _declaration("time_s", "s", "time", (None,), "the accepted sample time grid"),
158:        *(_declaration(name, "N", "force", (None, None), f"tire {name} column")
159:          for name in _TIRE_FORCES),
160:        *(_declaration(name, "-", "orientation", (3, 3), f"upright {side} pose rotation")
161:          for side, name in _UPRIGHT_ROTATION.items()),
162:        *(_declaration(name, "mm", "length", (3,), f"upright {side} pose translation")
163:          for side, name in _UPRIGHT_TRANSLATION.items()),
164:        *(_declaration(name, "mm", "length", (3,), f"wheel centre on the {side} upright")
165:          for side, name in _WHEEL_CENTER_LOCAL.items()),
166:        _declaration("diagnostic_accepted", "bool", "count", (None,), ...),
167:        _declaration("diagnostic_rejected_attempts", "count", "count", (None,), ...),
168:        _declaration("diagnostic_newton_iterations", "count", "count", (None,), ...),
169:        _declaration("diagnostic_active_contacts", "count", "count", (None,), ...),
170:        _declaration("diagnostic_contact_events", "count", "count", (None,), ...),
171:    ),
172:)
```
（上面把原文的多行 `_declaration(...)` 折叠为一行展示；原文行号为 `:154-216`，逐条 `_declaration` 分别在 `:157/159/164/170/176/180/187/194/201/208`。）
（SPEC/F19 写的锚点 `outputs/builtin.py:154` **成立**。）

`RIG_OUTPUTS`（定义处 `outputs/builtin.py:220`，注释在 `:218-219`）：
```
218:#: What the rig's own driving and instrumentation adds.  A wheel load and a
219:#: steering output are measured at the rig, not produced by the suspension.
220:RIG_OUTPUTS = DeclarationSet(
221:    domain="rig",
222:    outputs=(
223:        _declaration("steering_output", "rad", "angle", (None, None),
228:                     "steering angle samples", domain="rig"),
229:        *(_declaration(f"wheel_load_{wheel}", "N", "force", (),
237:                       f"{wheel} normal load", domain="rig")
238:          for wheel in _WHEELS),
242:        _declaration("native_kernel_wall_time_s", "s", "time", (), ...),
250:        _declaration("diagnostics_available", "bool", "count", (), ...),
258:    ),
259:)
```
（SPEC/F19 写的锚点 `outputs/builtin.py:220` **成立**。）

`outputs/builtin.py:40-52` 的 `__all__` 同时导出两者（`:41 ASSEMBLY_OUTPUTS`、`:44 RIG_OUTPUTS`）。

---

## 2. 是否被 `api.py` 或 rigs 生产路径消费（真实 grep）

### 2.1 `ASSEMBLY_OUTPUTS`

```bash
$ grep -rn "ASSEMBLY_OUTPUTS" --include=*.py .
./packages/suspension_multibody/src/suspension_multibody/outputs/builtin.py:41:    "ASSEMBLY_OUTPUTS",
./packages/suspension_multibody/src/suspension_multibody/outputs/builtin.py:154:ASSEMBLY_OUTPUTS = DeclarationSet(
```
**命中 2 行，全在同一文件内（定义处 + 自己模块的 `__all__`）。全仓没有任何外部消费者，测试里也没有。**

### 2.2 `RIG_OUTPUTS`

```bash
$ grep -rn "RIG_OUTPUTS" --include=*.py .
./packages/suspension_multibody/src/suspension_multibody/outputs/builtin.py:44:    "RIG_OUTPUTS",
./packages/suspension_multibody/src/suspension_multibody/outputs/builtin.py:220:RIG_OUTPUTS = DeclarationSet(
```
**同样只有定义 + `__all__`，零外部消费者，测试里也是零。**

### 2.3 `api.py` 是否消费 `outputs/`

```bash
$ grep -n "outputs" packages/suspension_multibody/src/suspension_multibody/api.py
399:    its outputs together -- a run that carried a rack axis of zeros would look
```
**`api.py` 里唯一出现 "outputs" 的地方是 `:399` 的一句散文注释**，没有任何 `outputs` 包/声明表的 import 或读取。
`api.py` 的 import 段（`:35-81`）里没有 `from .outputs import ...`。

### 2.4 rigs 生产路径是否消费 `outputs/`

```bash
$ grep -rn "outputs" --include=*.py packages/suspension_multibody/src/suspension_multibody/rigs/*.py \
    packages/suspension_multibody/src/suspension_multibody/simulation/*.py \
    packages/suspension_multibody/src/suspension_multibody/compilation/*.py
```
命中的全部是**同名但不同物**的 rig 自身字段 / `SimulationRequest.outputs`，**没有一条读 `outputs/` 包里的 `ASSEMBLY_OUTPUTS`/`RIG_OUTPUTS`/`DeclarationSet`**：
```
rigs/rig.py:80-81    #: Minimum-unit outputs only this bench produces.   outputs: tuple[str, ...] = ()
rigs/rig.py:124/136/150/157/167/179/186    outputs=("wheel_load", ...)   # rig 自己的声明
rigs/compose.py:58-59/124   Composition.outputs
rigs/bench.py:194/274       spec.outputs
simulation/request.py:74-77  #: What the run is asked to produce ... outputs: tuple[str, ...] = ()
simulation/request.py:124    object.__setattr__(self, "outputs", tuple(self.outputs))
simulation/runner.py:129     outputs=plan.outputs
compilation/plan.py:124/179/229  outputs
```
`SimulationRequest.outputs` 的注释（`request.py:74-76`）说的是**「空即取装配与试验台声明合并后的集合」**，但实测没有任何代码路径去调 `outputs.builtin.ASSEMBLY_OUTPUTS`/`RIG_OUTPUTS` 或 `merge_declarations` 来兑现它（见下）。

### 2.5 谁在 import `outputs/`

```bash
$ grep -rn "outputs.builtin\|from .outputs\|from ..outputs\|from ...outputs" --include=*.py \
    packages/suspension_multibody/src packages/suspension_multibody/tests
packages/suspension_multibody/src/suspension_multibody/outputs/derived.py:417:#: The built-in registry, filled by `outputs.builtin`.
packages/suspension_multibody/tests/metrics/test_outputs_match_legacy.py:22:from suspension_multibody.outputs.builtin import LEGACY_CLASSIFICATION
```
加上包内相对导入（`outputs/__init__.py:22-43`）与测试侧的：
```
tests/metrics/test_outputs_match_legacy.py:21-22   from suspension_multibody.outputs import BUILTIN, builtin
tests/outputs/test_declarations.py:14              from suspension_multibody.outputs import (...)
tests/outputs/test_derived.py:16
tests/outputs/test_merge.py:15
tests/architecture/test_import_boundaries.py:81    "suspension_multibody.outputs"（import 边界名单）
```
`declared_names`（`outputs/builtin.py:975`）与 `merge_declarations` 的消费者复核：
```bash
$ grep -rn "declared_names" --include=*.py .
./packages/suspension_multibody/src/suspension_multibody/outputs/builtin.py:975:def declared_names(...)
```
**`declared_names` 零消费者**（只有定义）。`merge_declarations` 的调用者只出现在测试里：
```
tests/outputs/test_merge.py:19/45/57/66/78/89/100/102/118/122/126
```

**结论：`outputs/` 的静态声明（`ASSEMBLY_OUTPUTS` / `RIG_OUTPUTS` / `declarations.py` 的整套 `DeclarationSet` /
`merge_declarations` / `declared_names`）在今天的生产路径里完全没有消费者**——`api.py` 不读、rigs 不读、
simulation/compilation 也不读；唯一引用者是 `tests/outputs/`、`tests/metrics/` 与 `tests/architecture/` 的测试。
（与 F19 写的「未被 api.py 或 rigs 生产路径消费（仅测试引用）」一致。）

---

## 3. `results/channels.py` 与 `adams/axle_channels.yaml` 的关系

`results/channels.py:1-2`（文件头）：
```
1:"""Read-only registry for the frozen Adams axle channel contract."""
```
`results/channels.py:21-32`：
```
21:@dataclass(frozen=True)
22:class ChannelRegistry:
23:    """Minimal read-only view over the single frozen axle channel table."""
24:
25:    _contract: Mapping[str, Any]
26:
27:    @classmethod
28:    def load(cls) -> "ChannelRegistry":
29:        """Load the packaged ``axle_channels.yaml`` contract."""
30:        from ..adams.axle_contract import load_axle_channel_contract
31:
32:        return cls(_contract=_freeze(load_axle_channel_contract()))
```
（SPEC/F19 写的锚点 `results/channels.py:22 class ChannelRegistry` **成立**。）

绑定关系：
- `adams/axle_contract.py:32` — `_CHANNELS_PATH = Path(__file__).with_name("axle_channels.yaml")`
- `adams/axle_contract.py:93-95` — `def load_axle_channel_contract() -> dict[str, Any]: return _load_yaml_mapping(_CHANNELS_PATH)`
- 即 `ChannelRegistry.load()` → `load_axle_channel_contract()` → 读
  `packages/suspension_multibody/src/suspension_multibody/adams/axle_channels.yaml`，
  再经 `_freeze`（`channels.py:13-18`：`dict` → `MappingProxyType`、`list` → `tuple`）冻结。

`adams/axle_channels.yaml` 规模（解析实测）：`schema_version=1`、`channels=33`、`required_role_bindings=12`、全文 119 行。
文件头（`axle_channels.yaml:1-23`）：
```
1:schema_version: 1
2:world_axes:
3:  x: vehicle_rearward
4:  y: vehicle_rightward
5:  z: upward
6:rotation_sign: right_hand_rule
7:required_role_bindings:
8:  - sprung_body
9:  - fixture_reference_marker
10:  - left_wheel_center_marker
11:  - right_wheel_center_marker
12:  - left_wheel_spin_joint
13:  - right_wheel_spin_joint
14:  - left_spring
15:  - right_spring
16:  - left_damper
17:  - right_damper
18:  - left_tire
19:  - right_tire
20:channels:
21:  sprung_body.heave:
22:    unit: m
23:    formula: dot(world_z,com_position-com_position_at_trim)
```
（SPEC.md:13 写的锚点 `adams/axle_channels.yaml:20` 落在 `channels:` 段首，**成立**。）

消费者：
```bash
$ grep -rn "ChannelRegistry" --include=*.py .
results/channels.py:22 / :28          （定义 + classmethod）
results/__init__.py:4 / :27           （包内再导出）
tests/results/test_neutral_results.py:13 / :72
```
**它是一张冻结的 Adams 输出通道表（只读注册表），不是运行时信号总线**：没有写入面、没有按时间步读写的接口，
生产路径零消费者，唯一使用者是 `tests/results/test_neutral_results.py`。

---

## 4. 两套东西的关系（结论）

| | `outputs/` 静态声明 | `results/channels.py` + `adams/axle_channels.yaml` |
|---|---|---|
| 是什么 | 「一次运行**声明**产出什么」的声明数据（四域 assembly/rig/derived/merged） | Adams 对标侧**冻结的输出通道表**（通道名 → 单位 + 公式） |
| 生产者 | `outputs/builtin.py:154/220`、`outputs/declarations.py` | `adams/axle_channels.yaml`（33 通道） |
| 消费者 | **只有测试**（生产零引用） | **只有 `tests/results/test_neutral_results.py:72`**（生产零引用） |
| 有写入面吗 | 无 | 无（`frozen=True` + `MappingProxyType`） |
| 是信号总线吗 | 否 | 否 |
| 互通吗 | **不互通**：两者互不引用、互不消费，都没有运行时读写语义 | |

**结论：今天没有任何运行时信号总线；`outputs/` 静态声明与 Adams 冻结通道表是两套互不相通、且在生产路径上都不被消费的静态数据。**（F19 成立。）
