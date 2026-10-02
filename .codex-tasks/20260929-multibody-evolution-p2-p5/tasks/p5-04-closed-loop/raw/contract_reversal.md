# p5-04 证据 (b)：开环契约反转的理由与作用线/力路径对照（**先落盘，后改契约**）

> 实测日期 2026-10-02。父判据 `EPIC.md:281(b)` 与 F21（`EPIC.md:164`）。
> 顺序要求（`EPIC.md:325`，同阶段一 05 的做法）：**先登记反转理由与作用线/力路径对照，再改契约**。
> 本文件写作时 **`cases/handling.py` 与 `tests/cases/test_handling.py` 均未改动**。

## 1. 改前的契约原文（逐字）

`packages/suspension_multibody/src/suspension_multibody/cases/handling.py:8-11`：

```
Only open-loop manoeuvres are expressible here.  A closed-loop manoeuvre -- an
ISO lane change, say -- needs a driver following a path, which is a different
model rather than a different shape, and the kernel refuses those by name rather
than approximating them with a steering history that happens to look similar.
```

固化它的测试 `packages/suspension_multibody/tests/cases/test_handling.py:242`：

```python
def test_a_closed_loop_manoeuvre_is_refused_by_name(prepared) -> None:
    ...
    # A closed-loop manoeuvre is a driver model, not a shape; the case layer has
    # to say so rather than quietly approximating one.
    document["handling"]["steering"][0]["shape"] = "iso_lane_change"
    with pytest.raises(Exception, match="not open-loop"):
        run_request(...)
```

## 2. 反转理由：原契约说的是「输入形状」这一层，而闭环发生在别的一层

原契约的两句话在**各自的层**上都是对的，问题在于它被读成了对**整个系统**的断言：

1. **`handling` 家族的输入面**是一条**转向执行器的形状**（`SteeringShape`：常量/斜坡/阶跃/正弦）。
   在这个输入面上，「跟着一条路径走的驾驶员模型」确实**不是一种形状**——
   它是另一个模型。所以 `iso_lane_change` 作为 `shape` 被按名拒绝**仍然正确**。
2. 但**闭环不等于这个输入面**。本 Epic 的 D2 终裁已经确定：
   闭环在**内核力元求值路径**内成立——`rotational_torque` 族的求值函数在
   **每个内部残差评估**里同时接收实时 `State`（含轮胎纵向滑移 `tire_sx`）与当前
   `SampleInput`（含归一化驾驶员需求），控制律因此可以读实时状态、算需求、
   并在**同一拍**上把力矩施加回去。

### 作用线 / 力路径对照

| | 开环（`handling` 输入形状） | 闭环（p5-04 的 ABS） |
|---|---|---|
| 输入从哪来 | 方向盘 / 齿条位移的**时间函数**，由文档给定 | 由**控制器**从实时状态算出 |
| 输入在哪一层 | `cases/handling.py` 的 `steering` 形状表 | 内核 `element/anti_roll.cpp` 的力矩元求值 |
| 作用线 | 方向盘 → 齿条平动 → 转向横拉杆 → 转向节 → 接地侧向力 | 制动需求 → 力矩元力偶 → **车轮自旋轴**（纯力偶，无作用点） |
| 力路径 | 轮胎侧向力经悬架传给车身（横向动力学） | 力偶直接作用在轮体，反力由配对体承受（等大反向） |
| 谁在闭合回路 | 没有：文档给定即为全部 | 控制器：`|slip|` → 权限 → 需求 → 力偶 → 下一拍 `|slip|` |
| 时间尺度 | 整段历程（输出网格） | **求解器内部每一拍**（含被拒绝的尝试） |

**结论**：`iso_lane_change` 不该因为「现在有闭环了」而被接受为一种 `shape`；
它该被拒绝的理由从「闭环不存在」改成「**它不是一个输入形状**」。
契约的反转是把**断言的范围**收正，不是放宽它。

## 3. 反转后的契约原文（本次改动）

`cases/handling.py` 的模块 docstring 改为：

```
Only open-loop manoeuvres are expressible *as a shape here*.  This layer's input
is one steering actuator's history, so a manoeuvre that a driver model would
close around -- an ISO lane change, say -- is not a shape it can spell, and the
kernel refuses those names rather than approximating them with a steering history
that happens to look similar.

That is a statement about *this* input, not about the simulator.  A closed loop
exists, and it lives one layer down: a `rotational_torque` element is evaluated
inside every residual evaluation with the real-time state and the driver's own
demand, so a controller written there reads the state it is measured on and acts
on the same step (subtask p5-04, ABS).  The two are different layers rather than
two settings of one, which is why closing a loop there does not make a driver
model a valid shape here.
```

`tests/cases/test_handling.py:242` 的用例**保留**（`iso_lane_change` 仍被按名拒绝），
只改它声明的那件事：docstring 与注释从「闭环不存在」改成
「**拒绝的是输入形状，不是闭环本身**；闭环在内核力元路径内，见 `test_abs_closed_loop.py`」。
**不删除用例、不加 skip/xfail。**

## 4. 与阶段一 05 的关系（同口径，不同文件）

阶段一 05 反转的是 `tests/subsystems/test_rig_link.py` 的试验台契约（「试验台不得侵入」），
本行反转的是 `cases/handling.py` 的开环契约。两处都是**先把理由与作用线对照落盘、再改契约**，
但**分属不同 Epic、不同文件**，不重复改同一处。
