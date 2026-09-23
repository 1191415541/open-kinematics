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
3. **子系统 role 与实现模板解耦（需求 20 / D11，本步的机制基础）**：这是「简化现在、复杂后续」不返工的前提，必须在数据模型层就立住：
   - 定义**六个子系统 role**：`suspension` / `steering` / `wheel` / `chassis` / `brake` / `drive`。role 只定义**接口**：几何挂点（如制动挂 wheel_center + spin_axis）、参数槽（`PropertySlot`）、输出声明（`OutputDeclaration`）、力矩/驱动通道；
   - `Template` 增加 `role` 字段，声明它实现哪个 role；**同一 role 可注册多个模板**，装配时按名选择；
   - role 接口**不得包含**「是否产出刚体」「产出哪些体」这类实现细节——那是模板的事。简化模板（0 刚体）与复杂模板（带卡钳/转子/动力总成体）必须能实现**同一个 role 接口**；
   - **对标 Adams 实证**（安装目录 `C:\Program Files\MSC.Software\Adams\2024_1`）：同 `MAJOR_ROLE='brake_system'` 下，`_brake_system_4Wdisk.tpl`（见 `default_brakes.sub`，`[PART_ASSEMBLY]` = 0）与 `_brake_system_4Wdisk_calipers.tpl`（见 `convertible_brake_system.sub`，4 体）并存；另有 `_detailed_brake.tpl`。powertrain role 下 `_powertrain.tpl`/`_driveline_fwd.tpl`/`_driveline_rwd.tpl` 并存。
4. **六个 role 的最小接口定义**（本步给出结构，04 落地实例）。**六个 role 必须全部定义，缺一不可**：
   - `suspension`：几何挂点 = 硬点集（`upper_front`/`upper_rear`/`upper_outer`、`lower_front`/`lower_rear`/`lower_outer`、`wheel_center`，见 `front_axle.py:114-145` 的 `_ALIASES`）；参数槽 = spring / damper / bushing 属性引用；输出 = 轮跳与定位参数（camber/toe/轮距）；**无力矩通道**；
   - `steering`：几何挂点 = `rack_center`（`_ALIASES["rack_center"]`）+ tie rod 两端（`tie_inner`/`tie_outer`）；参数槽 = `rack_axis`（`schema/model.py:175`）、`rack_fixed_to_chassis`（`:176`）；输出 = rack 位移；**无力矩通道**；**可缺席**（需求 15/D5——缺席声明属**总成层**，不是 role 层，role 本身不表达可选性）；
   - `wheel`：几何挂点 = `wheel_center` + `spin_axis`；参数槽 = 轮半径/质量/惯量/轮胎属性（D9 的试验台侧车轮用**同一套**声明）；输出 = 轮心位姿与轮胎力；**无力矩通道**（力矩由 brake/drive 侧施加到它上面，它自己不提供通道）；
   - `chassis`：几何挂点 = 车身参考点；参数槽 = 质量/惯量（现状轴侧不消费 `MassSpec`，见 04 的「未消费事实」登记）；输出 = 车身位姿；**无力矩通道**；
   - `brake`：几何挂点 = `wheel_center` + `spin_axis`；参数槽须能容纳 Adams 简单版力矩子集——`brake_mu` / `piston_area` / `effective_piston_radius` / `front_brake_bias` / `max_brake_value`（D10）；**力矩通道 = 逐轮制动力矩**；
   - `drive`：几何挂点 = `wheel_center` + `spin_axis`；参数槽容纳 `driven_wheels` / `drive_split` / `maximum_drive_torque`（沿用现有 `DrivelineSpec` 语义）；**力矩通道 = 逐轮驱动力矩**。
5. **接口的强制方式（本步须定死，否则两个实现者会各行其是）**：采用**单一 `Template` 数据结构 + 声明式 `RoleSpec` 校验**，**不用「基类 + 六个 role 子类」**。理由：用户要求「所有模板是同一个模板格式」，Adams 的 `.tpl` 亦然（同为一种文件格式，`MAJOR_ROLE` 只是一个字段）；子类化会把 role 差异写进类型系统，与「同一格式」相悖，也让「新增 role 不改结构」落空。
   - `RoleSpec`：每个 role 一份声明，字段含 `required_mounts`（必需几何挂点名）、`required_slots`（必需参数槽）、`outputs`、`has_torque_channel`；
   - `Template.role` 是普通字段（枚举值之一），指向对应 `RoleSpec`；`Template` 结构本身**不因 role 而变**；
   - **校验时机两处**：`templates.register()` 时校验模板满足其 role 的 `required_mounts`/`required_slots`；`instantiate()` 时复校（属性文件填完后必需槽位不得为空）；
   - 校验失败须**点名**缺哪个挂点/哪个槽位，不得只说「模板非法」；
   - **新增 role 只需新增一份 `RoleSpec`**，不改 `Template` 结构——与 G2b 的可扩展性同源。
6. 内置一份**双叉臂模板**，把现役 `symmetric_proxy` 拓扑的 K/C 映射**如实数据化**为双列定义。**实测映射（必须照此，不得从需求 2 的字面「三个连接点都是运动副」推断）**：上/下摆臂的 **`inner_front` 在 K 模式有旋转副**（`RevoluteJoint`，轴由 front→rear 两点定义），而 **`inner_rear` 在 K 模式无任何副**（仅用于定义转轴）；C 模式两个内点各为 `BallJoint` + 占位衬套；**臂外点、拉杆两端、齿条导轨两个模式都是 joint（球副/移动副），没有衬套列**。若把 K 模式实现成两个内点都有副，会产生 14 条约束而非实测的 13 条；C 模式若丢弃仅 joint 列的点，会产生 8 条约束而非实测的 9 条。

## 非目标

- **本步不接线到装配**：只建数据结构与注册表，`build_front_axle` 不读模板。接线在 05。
- 不实现属性文件的加载与解析（06）。
- 不实现输出求值（07）；`OutputDeclaration` 本步只定义结构与序列化。
- 不拆子系统（04）。
- 不给麦弗逊/多连杆编硬点数据：只留**注册接口与结构**，用户后续自行注册模板。

- **role 接口不得泄漏实现细节**：不得出现 `has_bodies`/`is_simple`/`body_count` 之类字段进入 role 接口；也不得把「简化版」写进 role 名（模板名可含 `_simple`/`_calipers` 之类区分，但装配层不得据此分支）。
- **为复杂模板预留而不过度设计**：本步只保证 role 接口对「0 刚体」与「带刚体」两种实现同构；**不要求**现在就实现复杂模板（那是后续工作），也不要求预先造出复杂模板的全部字段。
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
- 只读（新增）：Adams 安装目录的官方子系统定义（`C:\Program Files\MSC.Software\Adams\2024_1\acar\shared_car_database.cdb\subsystems.tbl\{TR_Brake_System,TR_Powertrain}.sub`、`acar_concept.cdb\subsystems.tbl\{default_brakes,convertible_brake_system}.sub`、`...\assemblies.tbl\acar_gs_front.asy`）——仅作 role/模板解耦的对标依据，不得依赖其存在。
- 不写：`preparation/**`、`cases/**`、`subsystems/**`（04）、`outputs/**`（07）、父级计划文件（归主代理）。

## 依赖

- 前置：02（统一副表——模板的 `joint` 列要引用统一表里的副名）。
- 后续：04（子系统拆分需模板已定义结构）、05（实例化与 K/C 列激活）。

## 验收标准

1. `Template` / `PartDefinition` / `ConnectionDefinition` / 弹性元件槽位 / `PropertySlot` / `OutputDeclaration` 均可构造、可序列化、可反序列化（往返一致）。
2. `ConnectionDefinition` 能表达三种情形：仅 joint 列、仅 bushing 列、两列同时存在；且有测试覆盖三种。
3. 模板注册表可按名注册与取用；重名注册、取用未注册名、模板缺 name 均报错。
4. 内置双叉臂模板与现役装配的 K/C 映射**逐点对照一致**：有测试把模板里每个连接点的 K 列/C 列与 `build_front_axle` 实测产出对照（现役 K 产出 13 约束、C 产出 9 约束 + 8 衬套，可作对照基线）。
5. 负例必须失败并点名原因：连接点两列皆缺定义（无声明）、重复 role、引用了统一表里不存在的副名、属性槽重名（四个）；以及缺 role 必需挂点、缺 role 必需槽位（两个，见第 10 条）。
6. 现役门禁保持绿：本步纯新增，`dynamic_hash_sentinel`/`kc_parity`/`case_parity` 三门与 `--strict --final` 均不应变化，且**未重录任何基线**。
7. **role 与模板解耦有测试证明**：同一 role 下注册两个模板（一个 0 刚体、一个声明刚体），断言：(a) 两者都能通过 role 接口校验；(b) role 接口的字段集中**不含**任何实现细节字段；(c) 按名选择模板得到各自声明。
8. **制动参数槽可容纳 D10 子集**：`brake_mu` / `piston_area` / `effective_piston_radius` / `front_brake_bias` / `max_brake_value` 均可声明、序列化、往返一致；负例（缺 `max_brake_value`、`piston_area` 非正）报错。
9. **六个 role 各有 `RoleSpec` 且都被测试覆盖**：`suspension`/`steering`/`wheel`/`chassis`/`brake`/`drive` 六份 `RoleSpec` 均存在；有测试逐 role 断言其 `required_mounts`/`required_slots`/`has_torque_channel` 与本文档第 4 条一致（`brake`/`drive` 有 `has_torque_channel=True`，其余四个为 `False`）。
10. **`RoleSpec` 校验按声明生效且报错点名**：构造缺 `required_mounts` 中某项的模板 → `register()` 失败且错误信息**点名缺失的挂点名**；构造缺 `required_slots` 的模板 → 同样点名；属性文件填完后必需槽位仍为空 → `instantiate()` 失败并点名槽位。三个负例均须失败。
11. **新增 role 不改 `Template` 结构**：有测试证明注册第七份 `RoleSpec`（测试内构造）后，`Template` 的字段集**不变**，且现有六个 role 的模板不受影响。

## 验证协议

1. 数据结构落地后：跑 `tests/templates` 的构造与序列化往返测试。
2. 注册表落地后：跑注册/取用与重复名负例。
3. 内置模板落地后：跑「模板 vs 现役装配」逐点对照测试（这一步是后续 05 的关键前提，必须在此步就建立对照）。
4. role/模板解耦落地后：跑「同 role 双模板」测试与「role 接口无实现细节字段」断言（这是需求 20 在数据模型层的门）。
5. 六个 role 接口落地后：跑第 9 条（逐 role 断言接口一致）与第 11 条（新增 role 不改结构）两条测试。
6. `RoleSpec` 校验落地后：跑第 10 条的三个负例（缺挂点、缺槽位、填完后槽位仍空），确认报错点名。
7. 收尾：kernel/contracts/multibody 三套 pytest、ruff、ty、`git diff --check`、`--strict --final`、三个数值门。

本任务**不应导致任何基线重录**；若发现必须重录，说明误触了现役路径，停止并上报。
