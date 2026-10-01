# p2-06 证据 (d)：`cases/vehicle_kc.py` 的准备期删除改为只增不删

> 范围：`packages/suspension_multibody/src/suspension_multibody/cases/vehicle_kc.py` 的准备期删除段（父行 `SUBTASKS.csv` p2-06 (d) 与 `EPIC.md` F5 行 120 / 行 247 (d) / 行 313）。
> 与阶段一 05「试验台非侵入」的关系见第 4 节 —— **同口径、不同 Epic、不重复改动**。

## 1. 改前原文

### 1.1 `cases/vehicle_kc.py::model_document`（`git show HEAD:...cases/vehicle_kc.py`，行 114-137）

```python
def model_document(
    model_document_pair: tuple[dict[str, Any], bytes],
    *,
    wheels: tuple[VehicleKcCorner, ...],
    assembly,
) -> tuple[dict[str, Any], bytes]:
    """
    Add the sweep's driven coordinates to a vehicle model document.

    The payload is unchanged: driven coordinates are topology, not data.
    """
    document, blob = model_document_pair
    document = dict(document)
    # The vehicle model drives the rack through a prescribed steering actuator.
    # A K/C sweep drives it itself, and two rows on the same degree of freedom
    # is a rank-deficient constraint set, so the actuator is dropped here rather
    # than left in place to collide with the sweep.
    document["elements"] = [
        element
        for element in document["elements"]
        if element["type"] != "steering_actuator"
    ]
    document["joints"] = list(document["joints"]) + driven_joints(assembly, wheels)
    return document, blob
```

关键点：**先拿到含 `steering_actuator` 元素的模型文档，事后从元素面里按类型过滤删除**。这是「先构建再纠正」的形态，纠正动作必须对每个通道重复一次。

### 1.2 `cases/vehicle_kc.py::vehicle_model_document`（改前，行 188-191）

```python
def vehicle_model_document(model, prepared) -> tuple[dict[str, Any], bytes]:
    """Return the vehicle model document this family starts from."""
    return _vehicle_model_document(model, prepared)
```

### 1.3 `cases/vehicle_dynamic.py::model_document` 的 steering 元素发射段（改前，行 461-465）

```python
    elements.extend(
        _steering_element(prepared.steering, index, prepared.body_names)
        for index in range(len(prepared.steering.names))
    )
```

（无条件发射：只要准备层给出通道，元素面就有对应元素。）

## 2. 改后原文

### 2.1 `cases/vehicle_dynamic.py::model_document` 新增 keyword-only 参数（现 `:417` 起；参数在 `:422`，docstring 说明在 `:431`）

```python
def model_document(
    model: VehicleModel,
    prepared,
    *,
    name: str | None = None,
    include_steering_actuators: bool = True,
) -> tuple[dict[str, Any], bytes]:
    """
    ...
    ``include_steering_actuators=False`` leaves the steering elements out of the
    document at the *source*.  A family that drives the rack itself asks for this
    rather than filtering the elements out afterwards: a document that was built
    and then corrected is a document nobody described, and the removal would have
    to be repeated for every steering channel.
    """
```

发射段（现 `:474-478`）：

```python
    if include_steering_actuators:
        elements.extend(
            _steering_element(prepared.steering, index, prepared.body_names)
            for index in range(len(prepared.steering.names))
        )
```

默认 `True`：其它家族（`vehicle_dynamic` / `handling` / `ride_*`）的文档逐字节不变。实测证据见 `run_log.md` 的 `case_parity_check.py`（无参数）：`vehicle_dynamic  PASS  8 cases, bit-identical to the frozen snapshot` 与 `handling  PASS  4 open-loop shapes match an independent expansion (0.0e+00)`。

### 2.2 `cases/vehicle_kc.py::model_document`（现 `:126` 起；函数体在 `:148-152`）

```python
    """
    Add the sweep's driven coordinates to a vehicle model document.

    Only additions: the base document is asked for a vehicle *without* steering
    actuators and the sweep's own driven coordinates are appended to its joints.
    The earlier form built the document with the actuators and then deleted every
    ``steering_actuator`` element from it, which is why this function's contract
    is now stated as a request rather than as a correction -- a driver that is
    "the K sweep drives the rack itself" belongs in the request, and a deletion
    would have to be repeated once per steering channel now that a vehicle may
    declare several.

    The payload is unchanged: driven coordinates are topology, not data.
    """
    document, blob = model_document_pair
    document = dict(document)
    document["elements"] = list(document["elements"])
    document["joints"] = list(document["joints"]) + driven_joints(assembly, wheels)
    return document, blob
```

`document["elements"] = list(document["elements"])` 是**只读复制**（沿用既有 `document = dict(document)` 的浅拷贝约定），**没有任何 `!= "steering_actuator"` 之类的过滤表达式**。实测：

```
$ grep -rn "steering_actuator" packages/suspension_multibody/src/suspension_multibody/cases/vehicle_kc.py
（无命中，退出 1）
```

### 2.3 `cases/vehicle_kc.py::vehicle_model_document`（现 `:202-212`）

```python
def vehicle_model_document(model, prepared) -> tuple[dict[str, Any], bytes]:
    """
    Return the vehicle model document this family starts from.

    Steering actuators are left out at the source (subtask p2-06): this family
    prescribes the rack through its own driven coordinate, and two rows on one
    degree of freedom is a rank-deficient constraint set.  Asking the document
    builder for a vehicle without them is the boundary; deleting them afterwards
    was the correction.
    """
    return _vehicle_model_document(model, prepared, include_steering_actuators=False)
```

### 2.4 模块 docstring 新增段（现 `:15-24`）

```python
**This family asks for no steering actuator, rather than removing one.**  A K/C
sweep drives the rack itself, and a prescribed steering actuator on the same
degree of freedom would be a second row on one coordinate.  The answer is to ask
the model document for a vehicle without steering elements -- the same
"add, never remove" boundary subtask 05 of the assembly-layer epic applied to
the test rig, where a bench stopped stripping a declaration it did not want and
started not requesting it.  The two are the same rule read from two sides and
they belong to *different* epics: that one owns the rig files
(``subsystems/rig_link.py``, ``rigs/**``) and this one owns this family's model
document, so neither repeats the other's change.
```

## 3. 行为等价的实测

`vehicle_kc` 家族在改造前后的产物是否一致，由 `case_parity_check.py`（无参数）判定：

```
$ uv run --no-sync python packages/suspension_multibody/scripts/case_parity_check.py
  kc_quasi_static    PASS      worst error / tolerance 0.000185873 over the frozen K/C snapshot
  axle_dynamic       PASS      13 cases, bit-identical to the frozen snapshot
  vehicle_kc         PASS      grid matches an independent expansion; 10 mm reaches every wheel drive
  vehicle_dynamic    PASS      8 cases, bit-identical to the frozen snapshot
  handling           PASS      4 open-loop shapes match an independent expansion (0.0e+00); closed-loop refused
  ride_four_post     PASS      expansion matches an independently sampled excitation (0.0e+00)
  ride_random_road   PASS      expansion matches an independently expanded profile (0.0e+00)
  comparison         N/A       a per-target gate, not a solve: the kernel never reads a reference
OK: 8 families accepted
EXIT=0
$ uv run --no-sync python packages/suspension_multibody/scripts/kc_perf_gate.py --check
  k-100: 0.8245 s vs baseline 1.0230 s (x0.806)
  c-66: 1.1726 s vs baseline 1.4904 s (x0.787)

OK: benchmarks are within the recorded budget
EXIT=0
```

判据 `10 mm reaches every wheel drive` 说明 K/C 的驱动坐标（含齿条 `rack_drive`）仍然全部到位 —— 「现场不发执行器」没有让扫掠失去驱动。逐层断言见 `tests/preparation/test_steering_channels.py::test_the_kc_family_asks_for_a_vehicle_without_steering_elements`（`:289`）：

```python
    base = vehicle_model_document(model, prepared)
    assert not [entry for entry in base[0]["elements"] if entry["type"] == "steering_actuator"]
    # And the sweep's own rack drive is what drives it instead.
    document, _blob = model_document(base, wheels=(), assembly=prepared.assembly)
    assert "rack_drive" in {joint["name"] for joint in document["joints"]}
```

该用例用**两通道**模型（`_two_channel_model`）验证：源文档里两个通道的执行器元素都不发射，而扫掠自己的 `rack_drive` 仍在。旧写法下这段逻辑会漏掉第二个通道 —— 那正是把删除改成边界请求的理由。

## 4. 与阶段一 05「试验台非侵入」的关系（点名）

| | 阶段一 05（`.codex-tasks/20260929-assembly-layer-rework/tasks/20260929-05-rig-non-invasive/`） | 本行 p2-06 (d) |
|---|---|---|
| 问题形态 | 试验台（rig）**篡改被测总成内部构件**：绑定前把别人已声明的元素/角色改掉或删掉 | 家族（family）**删掉别人给的元素**：拿到含 `steering_actuator` 的模型文档后按类型过滤 |
| 口径 | 改成「**只外加约束与外加载荷**」，断言验证**被测总成不可变** | 改成「**在现场不发执行器**」（`include_steering_actuators=False`），全程**只增不删** |
| 写范围 | `subsystems/rig_link.py`（`_reown_tires` 等）、`rigs/**`、`tasks/20260929-01-freeze/raw/approved_deltas.json` | `cases/vehicle_kc.py` 的文档装配段、`cases/vehicle_dynamic.py::model_document` 的一个 keyword 参数 |
| Epic | 阶段一 Epic（`20260929-assembly-layer-rework`） | 阶段二~五 Epic（`20260929-multibody-evolution-p2-p5`）的 p2-06 |

**两处改动分属不同 Epic，写范围不相交**：本行未触碰 `subsystems/rig_link.py`、`rigs/**`、`approved_deltas.json`（见 `run_log.md` 的 `git status --short` 原文）。`EPIC.md` 行 110「不动阶段一的范围」与行 330「两处改动分属不同 Epic，必须点明关系避免重复」即为此要求。

## 5. 未触碰项自证

```
$ git status --short -- packages/suspension_multibody/tests/data/
（空，退出 0）
$ grep -rn "steering_actuator" packages/suspension_multibody/src/suspension_multibody/cases/vehicle_kc.py
（无命中，退出 1）
```

`cases/vehicle_kc.py` 的改动统计：`31 insertions(+), 11 deletions(-)`（`git diff --numstat`）。
