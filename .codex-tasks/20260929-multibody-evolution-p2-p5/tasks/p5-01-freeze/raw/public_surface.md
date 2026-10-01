# p5-01 冻结现状 · 公共面清单与 FrontAxleModel 形状/哈希口径

全部为 2026-10-01 本行**实跑/实读**结果（命令与输出见同目录 `run_log.md`）。
被读文件：`packages/suspension_multibody/src/suspension_multibody/__init__.py`（100 行）。

## 1. 公共面清单（`__init__.py`）

### 1.1 `_PUBLIC_NAMES`（`__init__.py:43-60`）

`dict[str, tuple[str, str]]`，**16 条**（不是 SPEC/EPIC 说写的 17 条，见 1.3）。

| # | 行号 | 名字 | 目标模块 | 目标属性 |
|---|---|---|---|---|
| 1 | `__init__.py:44` | `AxleDynamicsResult` | `.axle_dynamics` | `AxleDynamicsResult` |
| 2 | `:45` | `CaseSpec` | `.schema` | `CaseSpec` |
| 3 | `:46` | `FrontAxleModel` | `.schema` | `FrontAxleModel` |
| 4 | `:47` | `Manifest` | `.schema` | `Manifest` |
| 5 | `:48` | `ResultBundle` | `.schema` | `ResultBundle` |
| 6 | `:49` | `SchemaVersion` | `.schema` | `SchemaVersion` |
| 7 | `:50` | `VehicleDynamicsResult` | `.results.vehicle` | `VehicleDynamicsResult` |
| 8 | `:51` | `load_case` | `.schema` | `load_case` |
| 9 | `:52` | `load_model` | `.schema` | `load_model` |
| 10 | `:53` | `load_vehicle_dynamic_case` | `.schema` | `load_vehicle_dynamic_case` |
| 11 | `:54` | `load_vehicle_model` | `.schema` | `load_vehicle_model` |
| 12 | `:55` | `read_artifact` | `.io.artifacts` | `read_artifact` |
| 13 | `:56` | `run_case` | `.api` | `run_case` |
| 14 | `:57` | `run_dynamic_case` | `.api` | `run_dynamic_case` |
| 15 | `:58` | `run_vehicle_dynamics` | `.vehicle.service` | `run_vehicle_dynamics` |
| 16 | `:59` | `write_artifact` | `.io.artifacts` | `write_artifact` |

原文（`__init__.py:43`）：
```
43:_PUBLIC_NAMES: dict[str, tuple[str, str]] = {
```

### 1.2 `__all__`（`__init__.py:62-80`）

**17 条**（`_PUBLIC_NAMES` 的 16 条 + `"__version__"`，`__version__` 在 `:38` 直接定义、不经 `__getattr__`）。

| # | 行号 | 名字 |
|---|---|---|
| 1 | `__init__.py:63` | `AxleDynamicsResult` |
| 2 | `:64` | `CaseSpec` |
| 3 | `:65` | `FrontAxleModel` |
| 4 | `:66` | `Manifest` |
| 5 | `:67` | `ResultBundle` |
| 6 | `:68` | `SchemaVersion` |
| 7 | `:69` | `VehicleDynamicsResult` |
| 8 | `:70` | `__version__` |
| 9 | `:71` | `load_case` |
| 10 | `:72` | `load_model` |
| 11 | `:73` | `load_vehicle_dynamic_case` |
| 12 | `:74` | `load_vehicle_model` |
| 13 | `:75` | `read_artifact` |
| 14 | `:76` | `run_case` |
| 15 | `:77` | `run_dynamic_case` |
| 16 | `:78` | `run_vehicle_dynamics` |
| 17 | `:79` | `write_artifact` |

**总数：17**（与 SPEC.md:9 判据「清单恰好 17 条」一致）。

### 1.3 与 SPEC/EPIC 的差异（必须登记）

- SPEC.md:9 与 EPIC.md F24（`:174`）写「`__init__.py:43-80` 的 `_PUBLIC_NAMES`/`__all__` 有 17 个名字」。**实测**：`__all__` 17 条（判据成立），`_PUBLIC_NAMES` **16 条**——`__version__` 是 `__init__.py:38` 的模块级常量，不经过 `_PUBLIC_NAMES`/`__getattr__`。两处锚点 `:43-80` 正确。
- `__getattr__` 在 `:83-95`，`__dir__` 在 `:98-100`。

## 2. `api.py` 公开入口签名原文

`run_case`（`api.py:100-106`）：
```
100:def run_case(
101:    model: FrontAxleModel,
102:    case: CaseSpec,
103:    output_dir: str | Path | None = None,
104:    *,
105:    inputs: Mapping[str, Any] | None = None,
106:) -> ResultBundle:
```

`run_dynamic_case`（`api.py:154-158`）：
```
154:def run_dynamic_case(
155:    model: FrontAxleModel,
156:    case: DynamicCaseSpec,
157:    output_dir: str | Path | None = None,
158:) -> TimeSeriesResult:
```

与 EPIC.md F18（`:162`）一致（F18 写的 `:100`/`:154` 正确）。

## 3. `def simulate` 全仓核查（真实 grep）

命令：
```bash
grep -rn "def simulate" packages src tests scripts docs
grep -rn --exclude-dir=.git --exclude-dir=.venv --exclude-dir=.venv-build-gui \
  --exclude-dir=.mindfs --exclude-dir=build --exclude-dir=dist \
  --exclude-dir=.codex-tasks --exclude-dir=node_modules --exclude-dir=.pi \
  "def simulate" .
```
结果：**产品代码（packages/src/tests/scripts/docs）命中 0（exit 1）**；全仓同口径（排除 .venv/.mindfs/.codex-tasks 等）**命中 0（exit 1）**。
唯一的字面命中出现在 `.venv/Lib/site-packages/numba/...`、`scipy/.../test_kdtree.py`（`simulate_periodic_box`）与 `.codex-tasks/**` 的规划文本里，均非本仓库产品代码。
**结论：`simulate(assembly_document, case_document)` 不存在**（F18 成立）。

## 4. `FrontAxleModel` 字段清单

定义：`packages/suspension_multibody/src/suspension_multibody/schema/model.py:157 class FrontAxleModel(StrictModel)`。

字段（`schema/model.py:160-183`，逐个，原文顺序）：

| 行号 | 字段 | 类型 / 默认 |
|---|---|---|
| `:160` | `schema_version` | `SchemaVersion = 1` |
| `:161` | `name` | `str = "front_double_wishbone"` |
| `:162` | `units` | `UnitSystem = UnitSystem.ENGINEERING` |
| `:163` | `coordinate_system` | `CoordinateSystem = CoordinateSystem.VEHICLE` |
| `:164` | `hardpoints` | `dict[str, Vec3]`（必填） |
| `:165` | `bodies` | `tuple[RigidBodySpec, ...] = ()` |
| `:166` | `mass` | `MassSpec`（必填） |
| `:167` | `springs` | `tuple[LinearSpring, ...] = ()` |
| `:168` | `dampers` | `tuple[StaticDamper, ...] = ()` |
| `:169` | `bushings` | `tuple[Bushing6x6, ...] = ()` |
| `:170` | `tires` | `tuple[VerticalTire, ...] = ()` |
| `:171` | `anti_roll_bars` | `tuple[AntiRollBar, ...] = ()` |
| `:172` | `stops` | `tuple[BumpStop, ...] = ()` |
| `:173` | `side` | `Literal["left","right"] = "left"` |
| `:174` | `topology` | `Literal["symmetric_proxy","explicit"] = "symmetric_proxy"` |
| `:175` | `joints` | `tuple[IdealJointSpec, ...] = ()` |
| `:176` | `rack_axis` | `Vec3 = Vec3(x=0.0, y=1.0, z=0.0)` |
| `:177` | `rack_fixed_to_chassis` | `bool = False` |
| `:183` | `road` | `RoadSurfaceSpec | None = None` |

**共 19 个字段**。

校验器（属于形状的一部分，改形状必然触动）：
- `:185-192 _hardpoints_nonempty`（`@field_validator("hardpoints")`）
- `:194-203 _symmetric_input`（`@model_validator(mode="after")`，拒绝 `side != "left"`、拒绝 `symmetric_proxy` + `joints`）

## 5. `model_hash` 计算口径（硬门，p5-02 不得改）

调用点（`api.py`）：
```
116:    model_hash = canonical_hash(model.model_dump(mode="json"))
117:    case_hash = canonical_hash(case.model_dump(mode="json"))
290:        model_hash=canonical_hash(model.model_dump(mode="json")),
291:        case_hash=canonical_hash(case.model_dump(mode="json")),
293:    ).model_dump(mode="json")
```
（`api.py:116` 属 `run_case`；`:290` 属 `_run_axle_quasi_static`，即 `run_dynamic_case` 的 `axle_dynamic`+`quasi_static` 分支。SPEC 写的 `:116` 与 `:284` 中，`:284` 已过期，现为 `:290`。）

哈希函数（`io/results.py:16-21`）：
```
16:def canonical_hash(value: Any) -> str:
17:    """Hash canonical JSON data with stable key ordering."""
18:    payload = json.dumps(
19:        value, sort_keys=True, separators=(",", ":"), default=str
20:    ).encode("utf-8")
21:    return hashlib.sha256(payload).hexdigest()
```
口径 = `sha256( json.dumps(model.model_dump(mode="json"), sort_keys=True, separators=(",",":"), default=str).encode("utf-8") )`。

### 5.1 实测哈希值（对照基准）

输入形状 = `packages/suspension_multibody/tests/data/benchmark_axle.json` 的 `"model"` 段经
`FrontAxleModel.model_validate(...)` 得到的模型（K/C 门禁共用夹具，见 `tests/benchmark_fixture.py:27-29`）。

实跑脚本（写会话 scratch，不进仓库）：
```python
model = FrontAxleModel.model_validate(json.loads(fixture.read_text())["model"])
dump = model.model_dump(mode="json")
canonical_hash(dump)          # api.py:116 / :290 的同一次调用
```
输出（真实）：
```
fixture           : packages\suspension_multibody\tests\data\benchmark_axle.json
model_dump keys   : ['schema_version', 'name', 'units', 'coordinate_system', 'hardpoints', 'bodies', 'mass', 'springs', 'dampers', 'bushings', 'tires', 'anti_roll_bars', 'stops', 'side', 'topology', 'joints', 'rack_axis', 'rack_fixed_to_chassis', 'road']
model_dump bytes  : 1007
canonical_hash    : 72bd9f30330cce22fe672fc63c4248df09261928942443e2aedd98102b38c60b
```
（`model_dump keys` 19 个，与第 4 节字段表逐项对上。）

`subsystems/types.py:130` 里也点名了这条硬门：
```
130:    to `FrontAxleModel`: `api.py` hashes `model.model_dump(mode="json")` into
```

**p5-02 的硬门**：`model_dump(mode="json")` 的键集合（19）与 `canonical_hash` 对上述夹具的输出
`72bd9f30330cce22fe672fc63c4248df09261928942443e2aedd98102b38c60b` 必须逐位不变。
