# p5-02 证据 · `simulate` 公共入口

本文件只记**本行实跑/实读**结果。命令与退出码见同目录 `run_log.md`，门禁原文见 `gates.txt`，
哈希逐位对照见 `legacy_adapter.md`。改动文件：`src/suspension_multibody/api.py`、
`src/suspension_multibody/__init__.py`，新增测试 `tests/api/test_simulate.py`。

## 1. 签名原文（`api.py:130-133`）

```python
def simulate(
    assembly_document: str | Path | AssemblyDocument | SimulationAssembly,
    case_document: Mapping[str, Any],
) -> SimulationRun:
```

- 只有两个参数，且都**无默认值**（`tests/api/test_simulate.py::test_simulate_has_a_signature_and_a_documented_contract`
  断言 `list(inspect.signature(...).parameters) == ["assembly_document", "case_document"]` 且
  `default is inspect.Parameter.empty`）。没有隐藏的 `mode` / `family` / `rig` 参数：路由由文档给出。
- `AssemblyDocument` / `SimulationAssembly` 只在 `TYPE_CHECKING` 下导入（`api.py:90-93`），
  运行期在 `_documented_assembly` 内惰性导入；这样 `import api` 不会把 authoring 文档层拖进来。
- 返回既有中性类型 `simulation.runner.SimulationRun`（`.status` / `.raw` / `.compiled`），
  没有新增返回类型。

## 2. docstring 首段原文（`api.py:134-165`）

```
    Run one assembly document against one case document.

    This is the document-driven door onto the same run `run_case` authors: the
    two documents go to the assembler that already builds an assembly from a
    file, the contract documents they imply are compiled by the family compiler,
    and the pair is submitted through the one runner into the kernel.  Nothing
    here solves, authors a document of its own, or picks a device under test --
    a second assembly path is what this entry exists to *not* be.
```

docstring 依次说明：`assembly_document` 的三种拼法；rig 必须声明 `bench`（否则点名拒绝）；
`case_document` 用 `k` / `c` 段声明读数、两者都有或都没有都拒绝；契约 model 文档**不**由此入口接收
（它不声明属于哪个总成），成对的契约文档走 `simulation.runner.run_request`。

## 3. `__init__.py` 登记（`__init__.py:43-83`）

| 位置 | 改动 |
|---|---|
| `:21` `if TYPE_CHECKING:` 块 | `from .api import run_case, run_dynamic_case, simulate` |
| `:59` `_PUBLIC_NAMES` | 新增 `"simulate": (".api", "simulate")`（`run_vehicle_dynamics` 之后） |
| `:80` `__all__` | 新增 `"simulate"`（`run_vehicle_dynamics` 之后、`write_artifact` 之前） |

计数实测（p5-01 §1 基准：`_PUBLIC_NAMES` 16、`__all__` 17）：

```
$ uv run --no-sync python -c "import suspension_multibody as m; print(len(m.__all__), len(m._PUBLIC_NAMES))"
18 17
```

即**两处各 +1**，无删除、无改名。`from suspension_multibody import simulate` 实测可用
（`simulate.__module__ == 'suspension_multibody.api'`）。

## 4. 端到端用例与产物摘要

命令（本行实跑，退出码 0）：

```
uv run --no-sync pytest packages/suspension_multibody/tests/api/test_simulate.py -q -p no:cacheprovider
```


```
..................                                                       [100%]
18 passed in 2.12s
```

新增文件 `tests/api/test_simulate.py`，18 个用例分三类：

| 类 | 用例 |
|---|---|
| 公开面（3） | `test_simulate_is_a_public_name`（`package.simulate is api.simulate`、`"simulate" in __all__`、`_PUBLIC_NAMES["simulate"]` 恰为 `(".api", "simulate")`）、`test_simulate_has_a_signature_and_a_documented_contract`、`test_simulate_only_compiles_and_submits`（源码点名 `assembly_request_for` / `front_axle_model_for` / `preparation.kc_quasi_static` / `run_request`） |
| 端到端（3+1+3） | K 跑通、C 跑通、契约提交路由断言、总成文档 override 到达 model 文档、三种总成拼法各一条（path / AssemblyDocument / SimulationAssembly） |
| 拒绝（8） | 两个读数段同时存在、都缺失、family 不是 kc_quasi_static、rig 无 `bench`、bench 路由到别的 family、契约 model 文档当总成传、非 mapping 的 case、绞杀者回归（`FrontAxleModel` / `run_case` / `run_dynamic_case` 仍在 `__all__` 且可调用） |

### 4.1 K 端到端实跑产物

输入：`write_builtin_axle_project` 写出的总成文件（rig 的 `bench` 补成 `kc_quasi_static`）
+ 一份带 `k` 段的工况文档（`wheel_values_mm=[0.0, 20.0]`，`left_right_mode=symmetric`）。

```
[k] status        : success
[k] cases         : 2
[k] bodies        : 13 ['upper_arm_L', 'lower_arm_L', 'upright_L', 'tie_rod_L', 'wheel_hub_L',
                       'upper_arm_R', 'lower_arm_R', 'upright_R', 'tie_rod_R', 'wheel_hub_R',
                       'chassis', 'wheel_carrier_L', 'wheel_carrier_R']
[k] states shape  : (4, 13, 19)
[k] request       : assembly=axle rig=kc_quasi_static family=kc_quasi_static study=quasi_static name=k-run
[k] metadata      : {'compiler': 'KcQuasiStaticCompiler', 'family': 'kc_quasi_static',
                     'payload_schema': 'kc_quasi_static_contract_documents'}
[k] markers       : [{'name': 'wheel_center_L', 'body': 'wheel_hub_L', 'point': [0.0, -700.0, 300.0]},
                     {'name': 'wheel_center_R', 'body': 'wheel_hub_R', 'point': [0.0, 700.0, 300.0]}]
[k] model keys    : ['bodies', 'body_wrench_markers', 'contract', 'contract_version', 'elements',
                     'gravity', 'joints', 'kind', 'markers', 'name', 'tires', 'units']
[k] element wrench: True
```

### 4.2 C 端到端实跑产物

输入：`write_c_ready_axle_project`（同一 rig 修好 `bench`）+ 一份带 `c` 段的工况文档
（`paths=["fz"]`，`levels=3`，`side_mode=single`，`load_marker=wheel_center_L`）。

```
[c] status        : success
[c] cases         : 3
[c] bodies        : 11 ['chassis', 'upper_arm_L', 'lower_arm_L', 'upright_L', 'tie_rod_L',
                       'upper_arm_R', 'lower_arm_R', 'upright_R', 'tie_rod_R',
                       'wheel_carrier_L', 'wheel_carrier_R']
[c] states shape  : (6, 11, 19)
[c] request       : assembly=axle rig=kc_quasi_static family=kc_quasi_static study=quasi_static name=c-run
[c] markers       : [{'name': 'wheel_center_L', 'body': 'upright_L', 'point': [0.0, -700.0, 300.0]},
                     {'name': 'wheel_center_R', 'body': 'upright_R', 'point': [0.0, 700.0, 300.0]}]
[c] element wrench: True
```

两条都带 `element_wrench` 块：`simulate` 与 `run_case` 走**同一个** `_element_wrench_facts()`
上下文（`api.py` 中两处提交都经它），所以文档路线与对象路线的提交环境一致。

### 4.3 总成文档 override 实测

`tests/api/test_simulate.py::test_the_assembly_document_s_override_reaches_the_model`：

```
override before [0.0, -700.0, 300.0] after [0.0, -700.0, 340.0] success
```

说明改动确实来自**总成文件**的 copy-on-write override（而非子系统文件），且两次都 `success`。

## 5. 边界（本行明确不做，且不是静默降级）

1. **只支持 `assembly_kind == "suspension_axle"`**。总成文件声明 `full_vehicle` 时按名字拒绝，
   理由写进拒绝消息：整车读数有自己的 family preparation，文档侧尚无。整车文档不在本行
   写范围内（`preparation/**` 禁改），故不猜、不静默按轴跑。
2. **只支持 `family == "kc_quasi_static"`**。别的 family 的工况文档被点名拒绝。
3. **契约 model 文档不作为 `assembly_document` 接收**（`_documented_assembly` 抛 `TypeError`）：
   它不声明装配归属，接受它就是替调用方猜被测对象。成对的契约文档仍走
   `simulation.runner.run_request`（preparation 的文档旁路），这条路径本行未改动。
4. **`run_case` / `run_dynamic_case` 未字面改为调用 `simulate`**：它们手里是 `FrontAxleModel`
   而不是总成文档，硬造一份装配文档既触犯 SPEC.md:80「若发现需要复制装配逻辑，先登记并停下」，
   也会改动 `kc_baseline` / `dynamic_hash_sentinel` 冻结的 K/C 文档字节。实际共享的是
   **提交环境**（`_element_wrench_facts()`，两处提交都经它）与**同一个提交后端**：
   `run_case` 走 `_compile_plan_run` → `compile_plan` + `run_compiled`，`simulate` 走
   `run_request` → `compile_request` + `run_compiled`，两者最终都是
   `simulation/backend.py` 的同一次 `run_contract` 提交。偏差已在 `legacy_adapter.md` 登记。
5. **没有改 `composable_extension_examples.md`**：该文件的 `runnable` 块由
   `check_composable_release.py` 逐块执行，一个自包含的 `simulate` 例子需要写出一整套项目 JSON
   （约 60 行），成本与收益不成比例。本行不改示例文档，门禁输出原样为
   `[PASS] documentation examples: 3 example(s) executed: E-1, E-2, E-3`。
