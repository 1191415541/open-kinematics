# 通用多体扩展示例

Template、Subsystem、Assembly 和 Case 都是声明数据。文件与 Python 对象通过同一个
DocumentLoader 生成 ResolvedModel，经 compile_resolved 提交 native 内核，返回 ResultEnvelope。
Rig 是普通 Subsystem；Wheel 声明轮体和 Tire，台架通过端口施加边界，不供应第二个车轮。

以下五个 python runnable 块由 check_composable_release.py 在独立解释器中执行。

## E-1：声明任意数量的刚体和连接

数量由作者生成的数据决定，通用解释器只遍历实体表。

```python runnable
from suspension_multibody.authoring import AssemblyDocument, SubsystemDocument, TemplateDocument, assemble_generic

template = TemplateDocument.from_payload({
    "document": "template", "schema_version": 1, "name": "three_links",
    "functional_role": "generic", "allowed_placement_roles": ["any"],
    "symmetry": "asymmetric", "units": {"length": "m"},
    "bodies": [{"name": "support", "fixed": True}] + [
        {"name": f"link_{i}", "mass": 1, "inertia": [[1,0,0],[0,1,0],[0,0,1]]}
        for i in range(3)],
    "hardpoints": [{"name": "pivot", "owner": "support"}],
    "joints": [{"name": f"pivot_{i}", "type": "revolute",
        "body_a": "support", "body_b": f"link_{i}",
        "point_a": "pivot", "point_b": "pivot", "axis": [0,1,0]} for i in range(3)],
    "elements": [], "property_slots": [], "ports": [],
})
subsystem = SubsystemDocument.from_payload({
    "document": "subsystem", "schema_version": 1, "name": "three_links",
    "template": "three_links.tpl", "functional_role": "generic",
    "placement_role": "any", "hardpoints": {"pivot": [0,0,0]},
    "property_bindings": {},
}, template=template)
assembly = AssemblyDocument.from_payload({
    "document": "assembly", "schema_version": 1, "name": "three_links",
    "assembly_kind": "generic_multibody",
    "subsystems": [{"ref": "links", "functional_role": "generic", "placement_role": "any"}],
}, subsystems={"links": subsystem})
built = assemble_generic(assembly)
assert len(built.bodies) == 4 and len(built.joints) == 3
assert len(built.resolved_model().to_document()["provenance"]) == 4
print("E-1 ok: three declared links and three pivots")
```

## E-2：普通台架连接普通机构

台架与被测机构使用相同的模板单元；连接表显式绑定输入端口。

```python runnable
from pathlib import Path
from suspension_multibody import simulate, validate

root = Path("packages/suspension_multibody/examples/generic_multibody")
compiled = validate(root / "bench.assembly.json", root / "kinematics.case.json")
assert compiled.metadata["compiler"] == "ResolvedModelCompiler"
run = simulate(root / "bench.assembly.json", root / "kinematics.case.json")
assert run.status == "success", run.result.failure_evidence
assert len(run.result.cases) == 3
assert run.result.constraint_ids
print("E-2 ok:", len(run.result.cases), "native cases, declared rig constraints")
```

## E-3：属性变化进入同一求解路径

只修改弹簧自由长度，平衡高度随之改变，编译器和台架均无需修改。

```python runnable
import json
from pathlib import Path
from suspension_multibody import simulate
from suspension_multibody.authoring import AssemblyDocument, SubsystemDocument
from suspension_multibody.authoring.properties import ElementPropertyDocument

root = Path("packages/suspension_multibody/examples/generic_multibody")
original = AssemblyDocument.load(root / "assembly.json")
entry = original.entries[0]
properties = dict(entry.effective().property_bindings)
slot = next(name for name, law in properties.items() if law.element_type == "spring")
payload = properties[slot].to_payload()
payload["parameters"]["free_length"] += 10.0
properties[slot] = ElementPropertyDocument.from_payload(payload)
changed = SubsystemDocument.from_payload(entry.subsystem.to_payload(),
    template=entry.subsystem.template, properties=properties)
modified = AssemblyDocument.from_payload(original.to_payload(),
    subsystems={entry.ref: changed})
case = json.loads((root / "dynamic.case.json").read_text(encoding="utf-8"))
base = simulate(original, case)
shifted = simulate(modified, case)
assert base.status == shifted.status == "success"
body = "slider.sub.json.carriage"
assert shifted.result.body_state(body)[-1,2] > base.result.body_state(body)[-1,2]
assert base.result.model_fingerprint != shifted.result.model_fingerprint
print("E-3 ok: spring property changes the resolved model and equilibrium")
```

## E-4：纯内存作者与文件作者使用同一入口

此例不读取建模文件。刚体、关节、弹簧属性、总成与工况均由 Python 声明。

```python runnable
import numpy as np
from suspension_multibody import simulate
from suspension_multibody.authoring import AssemblyDocument, SubsystemDocument, TemplateDocument
from suspension_multibody.authoring.properties import ElementPropertyDocument

height = .25 - 10 * 9.80665 / 10000
template = TemplateDocument.from_payload({
    "document": "template", "schema_version": 1, "name": "memory_slider",
    "functional_role": "generic", "allowed_placement_roles": ["any"],
    "symmetry": "asymmetric", "units": {"length": "m"},
    "bodies": [{"name": "support", "fixed": True},
        {"name": "carriage", "mass": 10, "position": [0,0,height]}],
    "hardpoints": [{"name": "base", "owner": "support"},
        {"name": "mount", "owner": "carriage"}],
    "joints": [{"name": "guide", "type": "prismatic", "body_a": "support",
        "body_b": "carriage", "point_a": "base", "point_b": "mount", "axis": [0,0,1]}],
    "elements": [{"name": "spring", "type": "spring", "body_a": "support",
        "body_b": "carriage", "point_a": "base", "point_b": "mount", "property_slot": "spring"}],
    "property_slots": [{"name": "spring", "element_type": "spring", "required": True}],
    "ports": [],
})
spring = ElementPropertyDocument.from_payload({
    "document": "element_properties", "schema_version": 1, "name": "spring",
    "element_type": "spring", "model": "linear", "units": {"length": "m", "force": "N"},
    "parameters": {"stiffness": 10000, "free_length": .25},
})
subsystem = SubsystemDocument.from_payload({
    "document": "subsystem", "schema_version": 1, "name": "memory_slider",
    "template": "memory_slider", "functional_role": "generic", "placement_role": "any",
    "hardpoints": {"base": [0,0,0], "mount": [0,0,height]},
    "property_bindings": {"spring": "spring"},
}, template=template, properties={"spring": spring})
assembly = AssemblyDocument.from_payload({
    "document": "assembly", "schema_version": 1, "name": "memory_slider",
    "assembly_kind": "generic_multibody",
    "subsystems": [{"ref": "slider", "functional_role": "generic", "placement_role": "any"}],
}, subsystems={"slider": subsystem})
case = {"schema_version": 1, "name": "equilibrium", "study": "dynamic",
    "samples": [0,.001,.002], "solver": {}, "boundaries": [], "inputs": [], "outputs": []}
run = simulate(assembly, case)
assert run.status == "success"
np.testing.assert_allclose(run.result.body_state("slider.carriage")[:,2], height, atol=1e-9, rtol=0)
assert run.compiled.metadata["compiler"] == "ResolvedModelCompiler"
print("E-4 ok: memory declarations reach the shared native submission")
```

## E-5：同一车轮定义的锁转与自由自转

两个工况读取完全相同的总成、模板和 TIR。悬架台架使用 locked 边界，整车动态
使用 free 边界；原有 revolute bearing 始终保留，接触 frame 始终由 carrier 提供。

```python runnable
from pathlib import Path
import numpy as np
from suspension_multibody import simulate

root = Path("packages/suspension_multibody/examples/generic_multibody")
source = root / "tire.assembly.json"
locked = simulate(source, root / "wheel_locked.case.json")
rolling = simulate(source, root / "wheel_rolling.case.json")
assert locked.status == rolling.status == "success"
a, b = locked.compiled.model_document, rolling.compiled.model_document
assert [(row["name"], row["mass"], row["inertia"]) for row in a["bodies"]] == [
    (row["name"], row["mass"], row["inertia"]) for row in b["bodies"]]
assert a["tires"] == b["tires"]
assert a["joints"][:-1] == b["joints"]
assert a["joints"][-1]["type"] == "driven_rotation"
wheel = "tire.sub.json.wheel"
np.testing.assert_allclose(locked.result.body_state(wheel)[:,3:7], [[1,0,0,0]] * 3, atol=1e-10)
assert abs(rolling.result.body_state(wheel)[-1,5]) > 1e-4
assert a["tires"][0]["parameters"]["frame_body"] == "tire.sub.json.carrier"
print("E-5 ok: one wheel definition, explicit spin boundaries")
```

实体解析位于 authoring/generic.py，资源加载位于 authoring/loader.py，运动边界位于
compilation/motion.py，唯一编译入口位于 compilation/resolved.py，结果查询位于
results/envelope.py。新物理本构仍需扩展版本化契约和 native 内核。
