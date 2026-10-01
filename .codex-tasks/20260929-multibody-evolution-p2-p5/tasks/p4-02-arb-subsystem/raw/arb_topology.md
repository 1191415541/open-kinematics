# p4-02 判据 (b)：扭杆 + 左右小吊杆的选型与理由

## 1. 选型：**扭杆刚体 + 左右小吊杆刚体，力由既有的弹性连杆力元承载**

模板 `anti_roll_bar_simplified` 声明四个部件：

```
torsion_bar_L   扭杆左半（在左chassis_mount 上绕杆轴转动）
torsion_bar_R   扭杆右半
droplink_L      左小吊杆（在 droplink_mount_L 处球铰到扭杆左半）
droplink_R      右小吊杆
```

**扭杆是刚体而不是「一股等效扭簧」**，理由是可指认性：子系统装配产物必须能指出
「扭杆」与「左右小吊杆」各自的实体（本行判据 2）。把整根杆塌缩成一个力元，产物里就
没有任何东西是「扭杆」，只剩一条力。刚体化之后，两个 `chassis_mount` 端口是**扭杆在
车身上的转动副**、两个 `droplink_mount` 端口是**扭杆与吊杆之间的球铰**，四个端口各自
有明确的物理实体归属。

**杆本身的弹性由既有的弹性连杆力元承载**，即
`modeling/primitives/elements.py:601 AntiRollBarElement`：它的力偶是
`stiffness * (右端升高 − 左端升高) − reference`。这就是「等效扭簧力元」的落地形式——
把扭杆的抗扭刚度折算成两端相对垂向行程的抗力。左端与右端取的是模板声明的两个扭杆半体，
不是硬编码的体名。

## 2. 与 native 扭杆 ABI 的关系：**两套物理，本行不声称等价**

仓库里有两套防倾杆物理，`tasks/p4-01-freeze/raw/arb_two_physics.md` 已实测对照：

| | Python 侧 `AntiRollBarElement` | native 扭杆（`cpp/src/element/anti_roll.cpp`） |
|---|---|---|
| 输入 | 两端**垂向位移差** | 杆自身**扭转角与角速率** |
| 输出 | 两端等大反向的**线力对** | **纯力偶**（无作用点） |
| 阻尼 | 无 | 有（`-damping * rate`） |
| 代价 | 有势能 `0.5*k*d²` | 无势能，含耗散 |

`preparation/vehicle_dynamic.py` 明写二者 "is not equivalent"，那条拒绝**保持原样**。
本行选的是左列：它吻合「扭杆的抗扭刚度」这一弹性语义，而且它已经是模型文档能声明的
族（`schema/elements.py:224 AntiRollBar`）。

## 3. 为什么**没有**用 `RotationalTorqueElement`

本行之前刚新增的 `ELEMENT_ROTATIONAL_TORQUE` 族（`RotationalTorqueElement`）**不是**扭杆：

```
amplitude = min(stiffness * demand, max_torque)
sign      = −sign(相对角速率)      # 阻力律
```

它是**被驱动的执行器**——幅值来自驾驶员需求（`demand`），不存储能量。用承载制动/驱动
力矩元是正确的，用来表示「扭杆」则把**弹性构件**说成了**执行器**：同一份模型会因此
声称一个它没有的势能，且它的方向由速率决定而不是由位移决定，静态侧倾工况下行为完全不同。
**因此不采用**，这一点写进模块 docstring 作为记录。

## 4. 实测：四个部件与四个端口

```
$ uv run --no-sync python .codex-tasks/.../p4-02-arb-subsystem/raw/arb_rows_probe.py
declared ports (read off the template):
  chassis_mount_L    role=chassis_mount_L    owner=torsion_bar_L   labels=['L'] kind=geometry
  chassis_mount_R    role=chassis_mount_R    owner=torsion_bar_R   labels=['R'] kind=geometry
  droplink_mount_L   role=droplink_mount_L   owner=droplink_L      labels=['L'] kind=geometry
  droplink_mount_R   role=droplink_mount_R   owner=droplink_R      labels=['R'] kind=geometry
```

**四个端口各自挂在哪个部件上是有区分的**：两个 `chassis_mount` 挂在扭杆半体（它是转动副
的载体），两个 `droplink_mount` 挂在小吊杆上（它是球铰的载体）。左右各自带 `labels`，
左端口不可能满足右需求。

## 5. 实测：装配产物里的杆元件（内建双叉臂）

```
AntiRollBarElement rows: 1
  name='arb'
  left_body='upright_L'   left_point=[   0. -700.  100.]
  right_body='upright_R'  right_point=[  0. 700. 100.]
  stiffness=1000.0
```

杆元件取的两个端体仍是**该轴模板自己声明的轮端体**（双叉臂下是 `upright_L`/`upright_R`，
从 `wheel_center` 转接的 `far_owner` 读出，见 `raw/upright_grep.md`），两个作用点与刚度
**与改造前逐位相同**。即：本行把防倾杆独立成子系统，**没有改变已有模型的数值行为**。
