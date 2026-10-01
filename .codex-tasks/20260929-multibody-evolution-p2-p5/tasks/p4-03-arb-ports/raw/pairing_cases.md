# p4-03 判据 (b)：配对段决定插接位置（两个用例各一断言）

配对走**既有** `connections/matcher.py::match_requirements` 通道，没有第二条推断路径。

## 用例 1：双叉臂 → 插到**下臂**

断言：`tests/subsystems/test_arb_mount_ports.py::test_the_bars_mount_requirement_is_met_by_the_declared_port`

```python
offered = _suspension_ports()          # 实测：arb_mount_L owner=lower_arm_L，等等
report = match_requirements(
    (PortRequirement(role="arb_mount", match_labels=frozenset({"L"}), count=1),
     PortRequirement(role="droplink_mount_L", count=1)),
    offered,
)
mount = report.binding_for("arb_mount")
assert any(str(pid).endswith("arb_mount_L") for pid in mount.port_ids)
droplink = report.binding_for("droplink_mount_L")
assert any(str(pid).endswith("droplink_mount_L") for pid in droplink.port_ids)
```

实测通过。**左右不乱**：`match_labels={"L"}` 使左需求只能落到左端口。

## 用例 2：麦弗逊 → 插到**减振筒外筒**

断言：`tests/subsystems/test_arb_mount_ports.py::test_the_answer_comes_from_the_template_so_another_topology_differs`

```python
macpherson = Template(
    name="macpherson_probe", role="suspension",
    parts=(PartDefinition("strut_L"), PartDefinition("strut_R")),
    connections=(ConnectionDefinition("arb_mount_L", "arb_mount_L", owner="strut_L"),),
    ports=(PortDeclaration(name="arb_mount_L", role="arb_mount",
                           owner="strut_L", labels=frozenset({"L"})),),
)
assert [port.owner for port in macpherson.ports] == ["strut_L"]
assert [port.owner for port in DOUBLE_WISHBONE.ports] == ["lower_arm_L", "lower_arm_R"]
```

同一个 `role="arb_mount"`、同一个读者，**两个拓扑给出不同的落点**：
双叉臂落在下臂，麦弗逊落在减振筒外筒（`strut_L`）。

> **麦弗逊的声明为什么建在测试里**：本行写范围只允许动 `builtin.py` 的
> **端口/needs 段**，而新增一整个麦弗逊模板超出了那一段。测试里构造的最小声明
> 足以证明机制（换拓扑即换落点），且不越界改别人的注册文件。
> 真要做成产线模板，属后续行的工作。

## 用例 3：缺少配对时的行为**有定义**（点名拒绝）

断言：`test_a_missing_mount_is_refused_by_name`

```python
with pytest.raises(Exception) as caught:
    match_requirements(
        (PortRequirement(role="arb_mount", match_labels=frozenset({"L"}), count=1),), {}
    )
assert "arb_mount" in str(caught.value)
```

无候选时**报错并点名 `arb_mount`**，不是静默产出一个没接上的模型。

## 用例 4：owner 不在本届装配里的声明**不报出去**

断言：`test_a_mount_naming_an_absent_body_is_not_offered`

```python
offered = _declared_ports(("axle",),
    (PortDeclaration(name="ghost", role="arb_mount", owner="not_a_body"),
     PortDeclaration(name="real",  role="arb_mount", owner="a_body")),
    {"a_body": object()})
assert set(offered) == {"real"}
```

声明指向不存在的部件时跳过：报出去等于告诉邻居「你可以插到一个不存在的东西上」。
