# p5-03 判据 (c)：总线与既有 `outputs/` 静态声明的关系（谁是真源）

## 三样东西，各是什么

| # | 东西 | 位置 | 性质 | 本行处置 |
|---|---|---|---|---|
| 1 | **静态声明集** | `outputs/declarations.py` + `outputs/builtin.py` 的 `ASSEMBLY_OUTPUTS`(`:154`) / `RIG_OUTPUTS`(`:220`) / `DERIVED_OUTPUTS`(`:760`) | 「一次运行**宣称**它产出什么」，在任何计算之前 | **真源**：运行产出什么的唯一声明处 |
| 2 | **信号总线**（本行新增） | `signal_bus.py` | 运行产出之后的**带名字读取器** + 输入侧的**带名字写入器** | **读者**：不另立声明，逐通道写出它对应的声明名 |
| 3 | **冻结的 Adams 通道表** | `results/channels.py::ChannelRegistry`（读 `adams/axle_channels.yaml`） | 与外部工具之间关于「导出哪些通道」的**契约** | **未动**：它不是运行时总线 |

## 真源归属与对接方式

**真源 = 第 1 样（静态声明集）**；总线是**它的类型化读者**。落地方式是
`MeasurementChannel.declaration` 字段：每个总线通道都写出它对应的声明名，
于是两侧可以**互相核账**：

- 「声明了但没接」——声明里有、总线上没有的，是**本行登记**的未使用声明；
- 「接了但没声明」——总线有、声明里没有的，是**没人做过的声明**。

第二个方向由 `measurement_channels_without_declaration(...)` 直接给出可判定答案，
并有测试（`test_the_declaration_set_is_the_source_of_truth_for_the_channels`）。

## 未使用声明的逐条处置

| 声明集 | 处置 |
|---|---|
| `ASSEMBLY_OUTPUTS` / `RIG_OUTPUTS` 的 `tire_force`、`wheel_center_pose` 等 | **接入**：`tire_vertical_load` 通道读的就是 `tire_force` 声明的那块数据 |
| `DERIVED_OUTPUTS`（report 层的派生量） | **登记为派生**，不适合作总线测点——总线读的是「运行产出的最小单元」，派生量由 report 从它们算出来。这是 `outputs/declarations.py` 自己的分层口径（该模块 docstring 明写「derived 是算术」） |
| `outputs/declarations.py` 注释里提到的「未被 api.py 或 rigs 生产路径消费」 | **本行不改生产路径的消费关系**：本行只**从**声明集读名字做核账，不做「让 api 去消费它」这种改动（会触及 p5-02 刚定稳的公开出口结构，且不属本行判据） |

## `ChannelRegistry` 与 `axle_channels.yaml` 未被改

```
$ git status --short -- packages/suspension_multibody/src/suspension_multibody/results/channels.py packages/suspension_multibody/src/suspension_multibody/adams/axle_channels.yaml
（空）
```

两者**一个字节未动**。
