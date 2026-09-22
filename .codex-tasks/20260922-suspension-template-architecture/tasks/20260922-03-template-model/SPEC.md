# 子任务 03：建立模板与连接点的双列数据模型

## 目标

建立三层架构的第一层——**模板（Template）**，采用 Adams Car 的一致性定义，由专家维护、使用者无需关注。

1. 新建 `suspension_multibody/templates/` 包，定义模板的完整数据结构：
   - `Template`：模板本体（name + 下面各项）；
   - `PartDefinition`：模板声明有哪些部件；
   - `ConnectionDefinition`：一个连接点，**同时可带 joint 列与 bushing 列**；
   - 弹性元件槽位：spring / damper 的定义与引用；
   - `PropertySlot`：模板要求属性文件提供什么；
   - `OutputDeclaration`：模板自带的输出声明（最小单位输出）。
2. 建立模板注册表：`templates.register(name, template)` / `templates.get(name)`，专家注册、使用者按名引用。
3. 内置一份**双叉臂模板**，把现役 `symmetric_proxy` 拓扑的 K/C 映射**如实数据化**为双列定义。**实测映射（必须照此，不得从需求 2 的字面「三个连接点都是运动副」推断）**：上/下摆臂的 **`inner_front` 在 K 模式有旋转副**（`RevoluteJoint`，轴由 front→rear 两点定义），而 **`inner_rear` 在 K 模式无任何副**（仅用于定义转轴）；C 模式两个内点各为 `BallJoint` + 占位衬套；**臂外点、拉杆两端、齿条导轨两个模式都是 joint（球副/移动副），没有衬套列**。若把 K 模式实现成两个内点都有副，会产生 14 条约束而非实测的 13 条；C 模式若丢弃仅 joint 列的点，会产生 8 条约束而非实测的 9 条。

## 非目标

- **本步不接线到装配**：只建数据结构与注册表，`build_front_axle` 不读模板。接线在 05。
- 不实现属性文件的加载与解析（06）。
- 不实现输出求值（07）；`OutputDeclaration` 本步只定义结构与序列化。
- 不拆子系统（04）。
- 不给麦弗逊/多连杆编硬点数据：只留**注册接口与结构**，用户后续自行注册模板。

## 约束

- **不得修改 `preparation/assembly/types.py`**：该文件同时是 04 的写范围，两者并行会冲突。本步只新增 `templates/` 包。
- **不改变任何现役行为**：本步纯新增，现有装配路径不引用模板，因此现有基线不应有任何变化。
- `ConnectionDefinition` 必须允许**两列都为 None**（该点两模式都不产出约束），也要允许**两列都非 None**（同一位置两模式各建一个）——后者正是用户裁决「模板里在同一个点可以建立运动副和衬套」的落点。
- 数据模型须可序列化/反序列化（模板要能被保存与共享）。
- 不引入新依赖。
- 命名注意：现有 `schema/model.py:173` 的 `topology` 已被占用（语义是「怎么描述轴」——`symmetric_proxy`/`explicit`），本步的「悬架类型/模板名」**不得复用该名字**，须用独立字段名（建议 `template`），并在 SPEC 或代码注释中写明两者的区分。

## 范围与文件归属

- 可写：
  - 新增 `packages/suspension_multibody/src/suspension_multibody/templates/**`
  - 新增测试 `packages/suspension_multibody/tests/templates/**`
  - `packages/suspension_multibody/src/suspension_multibody/__init__.py`（仅当需要导出时可加，不得改变现有导出）
- 只读：`preparation/assembly/front_axle.py`（提取现役 K/C 映射作为模板数据来源）、`schema/model.py`、`schema/elements.py`、`preparation/assembly/types.py`、02 产出的 `joints/` 表。
- 不写：`preparation/**`、`cases/**`、`subsystems/**`（04）、`outputs/**`（07）、父级计划文件（归主代理）。

## 依赖

- 前置：02（统一副表——模板的 `joint` 列要引用统一表里的副名）。
- 后续：04（子系统拆分需模板已定义结构）、05（实例化与 K/C 列激活）。

## 验收标准

1. `Template` / `PartDefinition` / `ConnectionDefinition` / 弹性元件槽位 / `PropertySlot` / `OutputDeclaration` 均可构造、可序列化、可反序列化（往返一致）。
2. `ConnectionDefinition` 能表达三种情形：仅 joint 列、仅 bushing 列、两列同时存在；且有测试覆盖三种。
3. 模板注册表可按名注册与取用；重名注册、取用未注册名、模板缺 name 均报错。
4. 内置双叉臂模板与现役装配的 K/C 映射**逐点对照一致**：有测试把模板里每个连接点的 K 列/C 列与 `build_front_axle` 实测产出对照（现役 K 产出 13 约束、C 产出 9 约束 + 8 衬套，可作对照基线）。
5. 四个负例必须失败并点名原因：连接点两列皆缺定义（无声明）、重复 role、引用了统一表里不存在的副名、属性槽重名。
6. 现役门禁保持绿：本步纯新增，`dynamic_hash_sentinel`/`kc_parity`/`case_parity` 三门与 `--strict --final` 均不应变化，且**未重录任何基线**。

## 验证协议

1. 数据结构落地后：跑 `tests/templates` 的构造与序列化往返测试。
2. 注册表落地后：跑注册/取用与重复名负例。
3. 内置模板落地后：跑「模板 vs 现役装配」逐点对照测试（这一步是后续 05 的关键前提，必须在此步就建立对照）。
4. 收尾：kernel/contracts/multibody 三套 pytest、ruff、ty、`git diff --check`、`--strict --final`、三个数值门。

本任务**不应导致任何基线重录**；若发现必须重录，说明误触了现役路径，停止并上报。
