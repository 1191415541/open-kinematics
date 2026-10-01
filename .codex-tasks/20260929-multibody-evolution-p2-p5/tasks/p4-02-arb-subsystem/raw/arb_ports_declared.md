# p4-02 判据 (c)：4 个端口的声明与实测

## 1. 声明处（本行交付）

端口是 `templates/builtin.py` 里 `ANTI_ROLL_BAR` 模板的 `ports` 字段，
由 `_ANTI_ROLL_BAR_PORTS` 生成。**不是运行期临时字符串**：名字、角色、所属部件、
左右标签都在模板声明里，import 期就存在。

```
$ uv run --no-sync python -c "
from suspension_multibody.templates.builtin import ANTI_ROLL_BAR
for p in ANTI_ROLL_BAR.ports: print(p.name, '->', p.owner, sorted(p.labels))
"
chassis_mount_L -> torsion_bar_L ['L']
chassis_mount_R -> torsion_bar_R ['R']
droplink_mount_L -> droplink_L ['L']
droplink_mount_R -> droplink_R ['R']
```

四个名字**逐字**为 `chassis_mount_L` / `chassis_mount_R` / `droplink_mount_L` /
`droplink_mount_R`，左右各自成立（每个端口带自己那侧的 `label`，左端口无法满足右需求）。

`subsystems/anti_roll_bar.py::PORTS` 是同一组名字的元组，并有测试断言它与模板声明一致
（`tests/subsystems/test_anti_roll_bar_subsystem.py::test_the_four_ports_are_declared_verbatim`），
所以两者不会各自漂移。

## 2. 缺端口即注册失败（实测）

删掉两个 `droplink_mount` 连接后的模板无法通过角色契约：

```
tests/subsystems/test_anti_roll_bar_subsystem.py::test_a_template_that_declares_no_such_port_cannot_satisfy_the_role
  with pytest.raises(TemplateError, match="droplink_mount_L"):
```

即四个端口是**契约**，不是约定：缺一个的模板根本注册不到这个角色下。

## 3. 与装配产物端口集合的分界（如实说明）

`si_assembly.py::_ports_for_bodies`（约 `:87-117`）目前为每个产出的体**运行期合成**一个
`role="body"` 端口，装配产物 `assembly.ports` 里的端口来自那个合成函数。
把模板声明接到装配产物的端口集合上（即「声明式端口取代按体名合成」）是
**p4-03 的写范围**（`SUBTASKS.csv` 的 `p4-03`：

> 悬架子系统声明语义化端口（含 arb_mount_L/R）使防倾杆可插（今天端口是装配期按 body 合成
> 见 si_assembly.py 的 _ports_for_bodies）

本行**不越界去改** `si_assembly.py`。因此本行交付的是：**防倾杆一侧的四个端口已经声明
完毕**，p4-03 可以直接拿它们与悬架的语义端口配对。
