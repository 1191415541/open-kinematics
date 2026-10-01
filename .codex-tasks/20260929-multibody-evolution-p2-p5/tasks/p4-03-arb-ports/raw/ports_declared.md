# p4-03 判据 (a)：悬架声明语义化端口（含 arb_mount_L/R）

## 1. 改动前后的机制

| | 改造前 | 改造后 |
|---|---|---|
| 悬架的端口来源 | `si_assembly.py::_ports_for_bodies` 为**每个产出的 body** 运行期合成一个 `role="body"` 端口 | 上述**仍然保留**，另外把模板**声明的**端口一并提供 |
| `arb_mount_L/R` | 不存在 | 由 `templates/builtin.py::_PORTS` **声明**，`owner` 指向承载它的部件 |
| 语义 | 「这里有个 body」（没说邻居能插什么） | 「这里有个 body」+「邻居可以往这里插，落在哪个部件上」 |

新增函数 `si_assembly.py::_declared_ports(instance, declarations, bodies)`：把模板的
`PortDeclaration` 经既有 `templates/ports.py::declaration_to_port` 变成装配的端口。
**owner 不在本届装配体表里的声明被跳过**，而不是带着悬空 owner 报出去——那是个假声明。

`subsystems/suspension.py::declared_ports(context)` 返回**该轴悬架模板自己声明的**端口，
所以换模板即换答案（见判据 (b) 的两个用例）。

## 2. 声明处（实测）

```
$ uv run --no-sync python .codex-tasks/.../p4-03-arb-ports/raw/ports_probe.py
template-declared ports (templates/builtin.py):
  arb_mount_L    role=arb_mount    owner=lower_arm_L    labels=['L']
  arb_mount_R    role=arb_mount    owner=lower_arm_R    labels=['R']
```

## 3. 装配产物里的端口（实测）

```
assembly product port set (names containing 'arb_mount'):
  suspension:arb_mount_L
      id    = axle/arb_mount_L
      role  = arb_mount
      owner = axle/lower_arm_L
      labels= ['L']
  suspension:arb_mount_R
      id    = axle/arb_mount_R
      role  = arb_mount
      owner = axle/lower_arm_R
      labels= ['R']

total ports offered by the axle contributions: 17
```

**四个名字里本行负责的两个已到产物**，且 `owner` 就是声明的那个部件
（双叉臂 = 下臂）。左右各自带 `labels`，左端口不能满足右需求。

## 4. 与 p4-02 的防倾杆 4 端口的关系

p4-02 交付的 `anti_roll_bar_simplified` 声明 `chassis_mount_L/R` 与 `droplink_mount_L/R`；
本行的 `arb_mount_L/R` 是**悬架一侧**的对接面。两者的 role 与 labels 都齐备，
可以经既有 `match_requirements` 配对（判据 (b) 的用例即此）。

**边界如实说明**：把 `anti_roll_bar` 作为一个**贡献**接进 `si_assembly.py` 的角色派发链，
使防倾杆子系统在装配里真的被构造，**不在本行**——本行的 TODO 第 1 行写的是
「悬架声明语义化端口（含 arb_mount_L/R）与端口合成段改造」，而派发链的扩充是
p4-04/p4-05 的收尾范围。本行交付的是**端口侧的对接面已经就位**。
