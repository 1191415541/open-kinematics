# p2-07 判据 (a)：改造前的文档路由拒绝（实测原文）

> 本行来源：p2-02 / p2-03 的实测缺口，经 `code-reviewer` 裁决 `fb245729` 后新增。
> 生产动态路径唯一的内核提交点是 `packages/suspension_multibody/src/suspension_multibody/simulation/backend.py:24`
> 的 `run_contract`；它提交的是**模型文档**，不是 C ABI 的 `ElementBlock` 数组。

## 复现

脚本 `raw/document_route_probe.py`（scratch 里先跑通，后落盘）。

最小模型：两体（`ground` 固定 + `rotor` 自由）、一个绕 `y` 的转动副、
**一个 `rotational_torque` 元素**（`body_a = ground`、`body_b = rotor`）。
初始角速度 `omega = [0, 3.0, 0]`。

```python
from suspension_multibody.kernel import run_contract   # backend.py:24 的内部调用
run_contract(model_document, case_document)
```

## 实测输出

```
$ uv run --no-sync python <probe>
KernelContractError: model document: element "torque" has unsupported type "rotational_torque"
```

抛出路径：

| 层 | 位置 |
|---|---|
| 拒绝分支 | `packages/suspension_kernel/cpp/src/cases/contract_model.cpp:830-833` |
| 消息模板 | `"element " + quote(name) + " has unsupported type " + quote(type_name)` |
| Python 侧抛出 | `packages/suspension_multibody/src/suspension_multibody/kernel/__init__.py:217`（`status != "success"` 时由 `manifest.failure_message` 抛出 `KernelContractError`） |

## 根因

`contract_model.cpp` 的 `elements` 循环是**逐族白名单**：它按 `*type_name` 逐个比对
`"spring" / "damper" / "bump_stop" / "aerodynamic_drag" / "steering_actuator" / "bushing"`，
并在末尾 `:830` 对任何其它名字无条件 `fail`。`rotational_torque` 不在该白名单里。

**同一份族的名字在别处是齐的，只有文档解析这一处缺**：

| 位置 | 是否有该族 |
|---|---|
| `cpp/src/assembly/element_reader.cpp:354-390`（通用块读取器） | **有** |
| `cpp/include/mb_input/types.hpp:80`（`ELEMENT_ROTATIONAL_TORQUE = 7`） | 有 |
| `cpp/include/mb_input/types.hpp:343-346`（`kElementLayouts`） | 有 |
| `cpp/src/contract/contract_registry.cpp:36-40`（`kElements[]`） | **缺** |
| `cpp/src/cases/contract_model.cpp:830`（文档逐族白名单） | **缺** |
| `packages/suspension_contracts/.../multibody_model.schema.json`（`element.type` enum） | **缺** |

所以：C ABI 路径能读该族（p2-02 的 `test_rotational_torque.py` 就是这么验的），
**文档路径读不到**。

## 影响

- p2-05 要删除离线预采样力矩路径，改由真实力矩元进入内核——那条路走的就是文档路由，
  因此在补齐本行之前，p2-05 无法达成其 Goal。这是 p2-05 增加 `depends_on = p2-07` 的原因。
- p2-06 与更后的阶段不依赖该族。

## 未做的事

本文件只记录**复现**。修法、验收与证据见 `raw/document_route_implementation.md`
与 `raw/document_route_evidence.md`（本行后续步骤产出）。
