# p5-02 证据 · `FrontAxleModel` / `run_case` / `run_dynamic_case` 的适配器现状与 `model_hash` 逐位对照

本文件只记本行**实跑**结果。`model_dump(mode="json")` 的键集合、字节数与 `canonical_hash`
均与 p5-01 `raw/public_surface.md` §4/§5.1 的冻结基准逐位对照。

## 1. 绞杀者模式：删了什么、留了什么

**删除：无。**

| 名字 | 位置 | 本行后状态 |
|---|---|---|
| `FrontAxleModel` | `schema/model.py:157`（**未改**） | 仍在 `schema/vehicle.py:252-253` 被 `VehicleModel.front_axle/rear_axle` 使用；仍在 `__init__.py` 的 `_PUBLIC_NAMES`（16→17 条之一）与 `__all__`（17→18 之一） |
| `VehicleModel` | `schema/vehicle.py`（**未改**） | 未触及 |
| `run_case` | `api.py:323`（签名未改） | 仍可用；`tests/api`、`scripts/accept_composable_architecture.py`、`tests/architecture/isolated_native_probe.py` 的既有调用全部通过 |
| `run_dynamic_case` | `api.py`（签名未改） | 同上 |
| `_run_axle_quasi_static` | `api.py`（未改） | 未触及：它走的 `run_request(...).raw` 没有开 `element_wrench` 通道，打开会改结果文档的 `contract_version`，进而扰动 `dynamic_hash_sentinel` 冻结字节 |

`git diff --stat`（本行写范围内）：

```
 packages/suspension_multibody/src/suspension_multibody/api.py         |  ...
 packages/suspension_multibody/src/suspension_multibody/__init__.py    |  ...
 packages/suspension_multibody/tests/api/test_simulate.py              |  (新增)
```

`api.py` 内被改动的既有函数只有两个：

1. `run_case` —— docstring 里的一处笔误修正 + 补回被误删的一行
   `assembly = _kc_assembly(model, case.mode, case.subsystems)`（行为与改动前一致）；
2. `_compile_plan_run` —— 元素通道的 `try/finally` 换成 `with _element_wrench_facts()`，
   **行为等价**：同一个环境变量、同一对 set/restore、同一处只包住 `run_compiled(compiled).raw`。

## 2. 与 SPEC Done-When 第二条的偏差（必须登记）

SPEC.md:88 / EPIC G7 行 91 的原话是 `run_case` / `run_dynamic_case`「**内部改走 `simulate`**」。
本行**没有**把它们字面改成调用 `simulate`，理由是实测得到的、不是省事：

1. 这两个入口的输入是 `FrontAxleModel` + `CaseSpec`/`DynamicCaseSpec`，**不是**装配文档。
   要让它们调 `simulate`，就得在 `api` 里凭空写出一份总成/子系统/rig 文件（或在内存里造等价的
   `AssemblyDocument`）——这正是 `SPEC.md:80` 明令禁止的「`simulate` 只做文档 → 既有装配器 →
   既有运行入口的薄封装；**若发现需要复制装配逻辑，先登记并停下确认**」。
2. 它们今天提交的 K/C 契约文档字节由 `tests/data/kc_baseline/` 与
   `tests/data/dynamic_hash_baseline.json` 冻结。换成「先造文档再交给 `simulate`」会改掉
   `drive_mode`、`drives` 轴映射或 model 文档的装配来源，改动量远超本行写范围
   （`cases/**`、`compilation/**`、`simulation/**` 全部禁改）。

实际共享的东西是**可验证**的三条：

| 共享点 | 证据 |
|---|---|
| 同一个提交环境 | 两处提交都包在 `api.py:817-844` 的 `_element_wrench_facts()` 里；实跑两条路线的结果都带 `element_wrench` 块（见 `simulate_entrypoint.md` §4.1/4.2） |
| 同一个编译层 | `run_case` → `_compile_plan_run` → `compilation.compile_plan`；`simulate` → `run_request` → `simulation.compiler.compile_request` → `DocumentPairCompiler.compile` → `compile_document_pair`。两者都产出 `CompiledSimulation` + `pack_container(...)` 载荷 |
| 同一个内核提交 | 两者最终都只经 `simulation/backend.py:24` 的 `run_contract`（`test_public_api_boundary_gate.py` 的 `direct_kernel_run_contract` 规则绿，见 `gates.txt`） |

**因此这一条按「未按字面完成、以等价共享替代并登记」记，不记为静默降级。**

## 3. `model_dump(mode="json")` 形状与 `model_hash` 逐位对照

口径：`canonical_hash(model.model_dump(mode="json"))`，`canonical_hash` =
`sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), default=str).encode("utf-8"))`
（`io/results.py:16-21`）。`api.py` 的调用点是 `run_case` 的 `model_hash = ...` 一行与
`_run_axle_quasi_static` 内部的 `Provenance(model_hash=...)`；本行未触碰这两行，
下面两次实测证明其产物未变。

### 3.1 p5-01 §5.1 基准夹具（`tests/data/benchmark_axle.json` 的 `"model"` 段）

| 项 | p5-01 冻结基准（§5.1） | 本行改动后实测 | 逐位 |
|---|---|---|---|
| 键集合 | `['schema_version','name','units','coordinate_system','hardpoints','bodies','mass','springs','dampers','bushings','tires','anti_roll_bars','stops','side','topology','joints','rack_axis','rack_fixed_to_chassis','road']` | 完全相同的 19 项、同序 | ✅ |
| 键数 | 19 | 19 | ✅ |
| `model_dump` JSON 字节数 | 1007 | 1007 | ✅ |
| `canonical_hash` | `72bd9f30330cce22fe672fc63c4248df09261928942443e2aedd98102b38c60b` | `72bd9f30330cce22fe672fc63c4248df09261928942443e2aedd98102b38c60b` | ✅ |

本行实跑输出原文：

```
benchmark fixture keys : ['schema_version', 'name', 'units', 'coordinate_system', 'hardpoints', 'bodies', 'mass', 'springs', 'dampers', 'bushings', 'tires', 'anti_roll_bars', 'stops', 'side', 'topology', 'joints', 'rack_axis', 'rack_fixed_to_chassis', 'road']
benchmark key count    : 19
benchmark dump bytes   : 1007
benchmark hash         : 72bd9f30330cce22fe672fc63c4248df09261928942443e2aedd98102b38c60b
```

### 3.2 `tests/conftest.py::full_vehicle_model`（`VehicleModel`）

| 项 | 改动前（本行开工时实测，会话 scratch `hash_before.txt`） | 改动后实测 | 逐位 |
|---|---|---|---|
| 键集合 | `['schema_version','name','units','coordinate_system','chassis','front_axle','rear_axle','wheels','steering','driveline','coordinate_couplers','aerodynamic_drag']` | 完全相同、同序 | ✅ |
| 键数 | 12 | 12 | ✅ |
| JSON 字节数 | 11835 | 11835 | ✅ |
| `canonical_hash` | `75cb96f55a7b120eff172b0760c378d016a99e7caeb7320e925ff500d9903fd9` | `75cb96f55a7b120eff172b0760c378d016a99e7caeb7320e925ff500d9903fd9` | ✅ |
| 形状 digest（键→类型树） | `dd803cec2309d1c668ee2e9dad71f0c18a467485eefc60d2170bca29f847aca5` | `dd803cec2309d1c668ee2e9dad71f0c18a467485eefc60d2170bca29f847aca5` | ✅ |

改动后实跑输出原文：

```
model_dump keys   : ['schema_version', 'name', 'units', 'coordinate_system', 'chassis', 'front_axle', 'rear_axle', 'wheels', 'steering', 'driveline', 'coordinate_couplers', 'aerodynamic_drag']
model_dump keycnt : 12
model_dump bytes  : 11835
canonical_hash    : 75cb96f55a7b120eff172b0760c378d016a99e7caeb7320e925ff500d9903fd9
shape_digest      : dd803cec2309d1c668ee2e9dad71f0c18a467485eefc60d2170bca29f847aca5
```

改动前实测（同一脚本，本行开工时）：

```
model_dump keys   : ['schema_version', 'name', 'units', 'coordinate_system', 'chassis', 'front_axle', 'rear_axle', 'wheels', 'steering', 'driveline', 'coordinate_couplers', 'aerodynamic_drag']
model_dump keycnt : 12
model_dump bytes  : 11835
canonical_hash    : 75cb96f55a7b120eff172b0760c378d016a99e7caeb7320e925ff500d9903fd9
shape_digest      : dd803cec2309d1c668ee2e9dad71f0c18a467485eefc60d2170bca29f847aca5
```

**四个值全部逐位相同；形状无增删、无顺序变化。**

### 3.3 与 p5-01 §5 锚点的差异说明

p5-01 §5 写 `api.py` 的两个 `model_hash` 调用点是 `:116` 与 `:290`。本行改了 `api.py` 的行数，
改动后实测位置为：

```
$ grep -n "canonical_hash(model.model_dump" packages/suspension_multibody/src/suspension_multibody/api.py
345:    model_hash = canonical_hash(model.model_dump(mode="json"))
```

（第二处仍在 `_run_axle_quasi_static` 的 `Provenance(...)` 构造里。）这是**行号位移**，
不是调用点增删：调用次数仍为 2，参数与口径未变，产物已在上表逐位对照。

## 4. `__all__` / `_PUBLIC_NAMES` 计数变化（如实登记）

| | p5-01 实测 | 本行实测 | 差异 |
|---|---|---|---|
| `_PUBLIC_NAMES` 条目 | 16 | **17** | +1（`simulate`） |
| `__all__` 名字 | 17 | **18** | +1（`simulate`） |

`FrontAxleModel` 在两处都**仍在**。`p5-01` §1.3 已登记「`_PUBLIC_NAMES` 是 16 条而 SPEC 写 17」
的旧差异；本行把 SPEC 写的 17 变成了 `_PUBLIC_NAMES` 的真实数字，属巧合而非刻意对齐。
