# Progress

## Recovery

- 任务: 文件驱动的模板、子系统、总成与试验台体系
- 形态: epic
- 状态: `SUBTASKS.csv` 行 1–8 **全部 DONE**；行 9（终局验收）按用户指令不跑两项长门，保持 PARTIAL
- 当前能力: 契约与五类文件；模板↔文件双向转换；属性文件（线性/非线性/分段）的本构值进入内核并改变内核算出的力；子系统文件（不可变拓扑、四哈希）；总成文件驱动单轴与整车的角色集合、位置与数量规则、装配级覆盖；试验台文件声明并校验支持种类/端口/执行器/测量通道并绑定注册试验台；项目加载、专家/普通用户边界、结果侧输入哈希
- 验证: 见下「本轮已验证（实跑）」，全部为本轮实跑结果
- 文件: `.codex-tasks/20260928-file-driven-authoring/SUBTASKS.csv`
- 长门: 架构 pytest 目录与 `just test-all` 按用户指令不跑（只跑与本任务相关的测试），不得记为已通过

## 本轮已验证（实跑）

| 门 | 命令 | 结果 |
|---|---|---|
| 静态 | `ruff check .` | 通过 |
| 静态 | `ty check .` | 通过 |
| 架构 | `legacy_surface_gate.py --check` | 通过，0 findings |
| 架构 | `check_module_layering.py --strict --final` | 通过，0 环、0 退役 |
| 架构 | `check_composable_release.py --skip-isolation` | 通过，3/3 |
| 快速集 | `pytest tests --ignore=adams,architecture,cases` | **1002 passed, 1 xfailed**（本轮前 989；xfail 为既有） |
| 定向 | `pytest tests/authoring tests/properties tests/instantiation tests/templates tests/subsystems` | 通过（authoring 60） |
| 契约+内核 | `pytest kernel/tests contracts/tests` | 60 passed |
| 数值 | `dynamic_hash_sentinel.py --check` | 与冻结基线逐位一致（sha256 `fdfd5a6b…` 未变） |
| 数值 | `case_parity_check.py` | 8 families PASS |
| 数值 | `kc_perf_gate.py` | 在预算内 |
| 空白 | `git diff --check` | 干净 |

**未执行**（不得算作已验证）：

| 门 | 原因 |
|---|---|
| `pytest tests/architecture`（149 用例，约 10 分钟） | 用户指令：只跑与本任务相关的测试。本轮未搬动既有模块，结构证据由三个秒级架构门脚本覆盖（已通过） |
| `just test-all`（约 33 分钟） | 用户指令：只跑与本任务相关的测试。数值门已覆盖求解路径的数值面 |
| justfile | 本机无 `just`，数值门与门禁脚本按 AGENTS.md 展开逐条执行 |

## 本轮改动

1. **阶段 3｜属性文件的本构值真正成为求解器应用的本构值。**
   - 曲线本体此前被 Python 侧丢掉（`bridge._element_rows` 只读标量），现在进入
     `LinearSpring`/`StaticDamper`/`BumpStop` 的 `force_curve`，契约产出
     `elastic_curve`/`damper_curve`/`stop_curve`。
   - 属性通道迁移为 resolved 本构值：`templates/model.py::SlotValue`
     （`element_type`/`model`/`scalar`/`curve`/`source`）；`instantiate` 接受数字或
     resolved 值并统一归一化，`resolve_properties`（旧 `PropertySet` 路线）与
     `authoring/properties.py`（元素属性文件路线）现在产出同一形状，
     `SubsystemInstance.law_for` 给出整条定律而不只是标量。数值门证明冻结数字未动。
2. **阶段 5｜总成文件决定装配的角色集合，单轴与整车两侧都成立。**
   - `assembly_request_for(总成文件)`：单轴取悬架子系统与装配级覆盖，整车取六个角色。
   - `compose_vehicle`/`compose_vehicle_runtime` 接受 `AssemblyRequest`；角色集合来自调用方，
     `check_root("vehicle", …)` 因此可以真的失败（把单轴集合当整车会被拒并点名 `brake`）；
     两根轴按「轴自己的角色词汇 ∩ 整车声明的角色」组装。
   - `vehicle_model_with_file_axles(template, 总成文件)`：整车的两根轴由文件里的悬架子系统构建，
     车轮/转向比/传动系仍是模板模型自身的数据。
3. **阶段 6｜Rig 文件的声明被校验而不只是被声明。** `supports` 与 `required_ports` 的一致性、
   `actuators` 必须是绑定试验台真正驱动的坐标（按 `RigSpec`）、可选端口参与连接集合解析、
   `measurements` 作为 `channels` 带出。
4. **阶段 8｜所有已定义的模板都能写成文件并读回。**
   - `template_document_from` 修两处：按**挂点角色**而不是按连接写挂点（零刚体角色两侧同名挂点
     会重复），以及允许**无 owner 的挂点**（模板 schema 相应放宽；零刚体角色只能声明别的刚体上的挂点，
     这正是内置模板早已在做的事）。`brake_4wdisk_simplified` 与 `powertrain_simplified`
     因此可导出→读回并保持部件、挂点角色与槽位。
   - fixture 即文件 fixture：`write_axle_project`、`write_builtin_axle_project`，新增
     `write_vehicle_project`（两悬架 + 转向/车轮/制动/驱动/车身 + 整车 Rig）。

改动文件：`authoring/{bridge,documents,solver}.py`、`templates/{model,instantiate}.py`、
`subsystems/{types,entry,vehicle_assembly}.py`、`contracts/template.schema.json`、
`tests/authoring/{fixtures,test_solver_integration,test_provenance_and_bench,test_vehicle_assembly_documents}.py`、
`tests/properties/test_properties.py`、`tests/subsystems/test_vehicle_composition.py`。

## 关键结论（有证据）

- **文件里的曲线就是内核应用的定律**。同一文件同时声明 `stiffness: 45` 与斜率 20 的曲线时，内核报告的
  弹簧力恰好是线性 45 的 20/45；换成覆盖同一区间的同斜率直线则与线性文件**逐位相同**。两条合起来排除了
  「曲线被忽略」与「曲线只改了标量」两种解释。契约的曲线是「有符号挠度 → 力」，该轴装配处于**伸长**状态：
  只覆盖压缩侧的曲线会被夹到首点而施力为 0（测试注释里写明了）。
- **文件格式与内置模板等价**：`template_document_from(DOUBLE_WISHBONE)` 往返后连接（含 per-mode 列与
  joint kind）、槽位、刚体、输出、kind/description 逐项相同，K 为 13 约束 / 0 衬套、C 为 9 约束 / 8 衬套。
- **整车侧的规则检查从「永远通过」变成「可以失败」**：角色集合此前是函数内的常量，现在是调用方的；
  单轴集合作为整车被 `check_root` 拒绝并点名 `brake`。
- **无 owner 的挂点是格式所必需**：零刚体角色（简化制动/驱动）只能声明落在别人刚体上的挂点，
  内置模板早已按这个方式声明转向/车轮/车身的挂点。
- **文件驱动的模型目前只在 K 读数下可解**：文件构造的模型不带轮胎与衬套，C 读数（`force_balance`/`pad`）
  都在静态配平阶段失败。计划验收文字是「K/C 或动态求解」，K 满足。

## 未完成 / 边界（逐条说明）

1. **终局验收的两项长门按用户指令未跑**（架构 pytest 目录、`just test-all`），见上表；行 9 因此保持 PARTIAL。
2. **steering/wheel/chassis 没有内置 `Template` 对象**：这三个角色是手写领域函数，随包只注册
   `double_wishbone`（它同时声明了三个角色的挂点）。它们的**文件形式**已由整车 fixture 证明可写，
   但装配仍由既有实现提供——把它们改成声明式模板是重新设计这三个子系统，会触及冻结的 K/C 装配，
   不属于本轮范围。
3. **整车级几何/车轮/转向比/传动系不由文件描述**：`VehicleModel` 的 `wheels`/`steering`/`driveline`
   在文件 schema 里没有对应字段，所以「整车从文件装配」停在「角色集合 + 两根轴由文件给出」。
   补全需要给四类文档加字段，属于新增契约。
4. **`brake`/`drive` 的 `register_simplified()` 仍无调用点**（既有死代码，本轮未动）：两个模板可以导出，
   但没有被注册进 registry；注册与否不改变任何求解行为，因此按「不擅自扩范围」保留原状。
5. 未新增求解器、未改 C++ ABI、未重录任何冻结基线。


## 后续四步（按「三条边界」的计划执行，属本 EPIC 之外的新范围）

计划：0 槽位承载非标量 → 1 文件驱动的模型能在 C 读数求解 → 2 整车字段与 `vehicle_model_from`
→ 3 steering/wheel/chassis 改模板驱动。**四步现已全部完成**，本节记录每一步的结论与证据。

### 第 0 步｜槽位与属性文件承载非标量

| 项 | 内容 | 证据 |
|---|---|---|
| 缺陷 1 | 任何 bushing 属性文件都会崩：`_SCALAR_FIELD` 缺 `bushing`（`KeyError`），且 `bushing` 与模型的 `bushing6x6` 拼写未对接、标量形式未按 6×6 校验 | 标量按三条平移对角校验、映射明确；矩阵形式直接按模型类校验 |
| 缺陷 2 | 文件模板里元素的两侧挂点都落在**左**臂上：`bridge` 把生成名（`lower_arm_L`）交给 `resolve_body`，后者见到已存在的名字就直接返回 | 现按 stem（`lower_arm`）交付，C 文档里右弹簧挂在 `lower_arm_R` |
| 形状 | `element_properties.schema.json` 新增 `matrix` 段（表格）、`element_type` 增加 `tire`；`_resolve` 携带表格；线性律允许只有表格 | 矩阵挂点文件可加载，`resolved["stiffness"]` 为 6×6 |
| 槽位 | `SlotValue.matrix`；`mount_stiffness` / `bushing_stiffness_matrix` 优先返回表格 | 文件提供的 6×6 到达装配：平移 10000、转动 1e7 |
| 挂点来源 | `mount_stiffness` 改为问「哪条槽位喂了 bushing 列」，不再硬编码槽位名 `bushing` | 文件可任意命名挂点槽位而不静默变成零刚度 |
| 轮胎 | `bridge` 从模板声明的 `tire` 元素构建 `FrontAxleModel.tires`（接触点=轮心降一个静半径） | 文件模型的 `tires` 数量 1 |
| 槽位连线 | bushing 槽位的 `connections` 由元素声明**推导**，不再要求作者手写生成名（`mount_upper_front_R`） | 文件只写元素与槽位即可 |
| 刚体非标量 | `template.schema.json` 的 body 增加 `center_of_mass` / `inertia`；`bridge._body_spec` 在文件声明时读取 | 文件不写则仍是模型默认，既有模板数字不动 |

### 第 1 步｜文件驱动的模型能在 C 读数（pad）下求解

**结论：能。** 之前「文件模型进 C 就停在静态配平」的原因是 fixture 缺三样东西，不是路线缺陷。

| 项 | 内容 | 证据 |
|---|---|---|
| 挂点镜像 | `front_axle_model_from` 的刚体表改用 `_mirrored_parts`，文件只写一侧也能列出整轴九个刚体 | 否则整车模型报 `front axle requires positive mass specs for upper_arm_R` |
| fixture | `write_c_ready_axle_project`：4 个柔性挂点元素（6×6 表格）、1 个轮胎元素、被加载 marker 的 label、弹簧按装配长度 | `tests/authoring/test_solver_integration.py::test_a_file_project_solves_the_c_reading` |
| 求解 | 文件项目经 `simulation.run_request` 完成 C/pad 求解：3 个载荷级全部 `status=success`，约束与动力学残差 < 1e-6 | 轮胎法向力 0 / 250.12295985 / 500.70981344 N，恰好等于文件声明的 200 N/mm × 求解出的侵入量 |
| 三样东西都是必需的 | 去掉轮胎 → 载荷全落悬架（位能 3.758 → 4.514 J）；去掉柔性挂点 → C 装成 13 约束 / 0 衬套、轮胎力 2680 N（对 500.7 N）；去掉 `wheel_center` label → 报 `c.load_marker names unknown marker wheel_center_L` | `test_every_c_ready_ingredient_changes_the_run` |
| 预载弹簧 | C/pad 的静态配平对**任何**预载都失败（5 mm 起），且内置对照**逐位相同**：零预载同样成功、5 mm 同样报 `iterations=2, force_residual=0.290426, position_residual=0.000140` | 这是该读数的性质，不是文件路线的性质；fixture 因此按装配长度声明弹簧 |

### 第 2 步｜整车字段、`vehicle_model_from` 与反向导出

| 项 | 内容 | 证据 |
|---|---|---|
| 契约 | `assembly.schema.json` 增加可选 `vehicle`（`chassis` / `wheels` / `steering` / `driveline`），结构性由 schema 判、语义由模型类判 | `vehicle_model_from` 的失败信息点名到字段：`vehicle.steering: ratio ...` |
| 读取 | `authoring/vehicle.py::vehicle_model_from(装配文件)`：四个对象来自文件，两根轴来自文件的悬架子系统（`file_axles_from`，两条路线共用） | `test_a_vehicle_document_builds_a_vehicle_model_from_its_own_numbers` |
| 导出 | `vehicle_document_from(model)` 与读取互逆且是不动点；被 `model_dump` 隐藏的字段（`tire_mass`、制动四参数）由文件显式写出，导出不丢信息 | `test_the_vehicle_numbers_survive_a_trip_through_the_file` |
| 一条转向 = 一根被转向的轴 | 文件只声明一个转向系统，故后轴齿条按 `rack_fixed_to_chassis=True` 处理 | 前轴 False、后轴 True，且 `prepare_vehicle_run` 的 `_validate_steering_topology` 接受 |
| 端到端 | 整车模型全部来自文件，跑通本包自己的 `vehicle_kc` 读数：`status=success`，装配体是文件模板的部件（`front_upper_arm_L`、`wheel_front_left`） | `test_a_file_vehicle_runs_the_vehicle_kc_reading` |
| 项目 | `Project.load` 认得整车总成与它的 `vehicle` 段 | `test_the_project_loads_the_vehicle_and_its_numbers` |

### 第 3 步｜steering / wheel / chassis 改模板驱动

**结论：完成，且装配逐位不变。** 四个单角色模板已注册，三个模块改为读模板声明。

| 项 | 内容 | 证据 |
|---|---|---|
| 模板 | `builtin.py` 新增并注册 `STEERING_GUIDED` / `STEERING_FIXED` / `CHASSIS` / `WHEEL`（`BUILTINS` 元组） | 注册表 5 个模板，四者都通过 `check_role_contract` |
| 为什么要两个转向模板 | 齿条是「沿齿条轴导向」还是「焊死在车身」由 `model.rack_fixed_to_chassis` 决定，而模板没有条件表达——两个模板各自陈述一种情形，模块按模型选 | `test_the_steering_template_follows_the_model_s_rack`（PrismaticJoint vs WeldJoint） |
| chassis | `chassis.py` 从 `CHASSIS` 的 parts 读刚体（名字与 `fixed`），姿态与惯量仍是轴侧自己的 | `test_the_chassis_template_decides_the_chassis_bodies`（模板声明第二个刚体就出现第二个刚体） |
| wheel | `wheel.py` 按角色+侧别找模板声明的轮心挂点，轮胎挂在它声明的刚体上；定律仍是模型的 | `test_the_wheel_template_decides_where_the_tire_hangs` |
| steering | `steering.py` 的刚体、两对球铰（名字、两端、先后顺序）、齿条导向的名字与种类全部来自模板；几何与齿条轴仍是模型的 | `test_the_steering_template_decides_the_rack_guide`（改名即改名） |
| 逐位不变 | 改前记录装配指纹（单轴与整车、K 与 C 的刚体/点表/连接/约束/衬套/力元），改后逐字节相同；`case_parity` 的 `vehicle_dynamic` 仍是 8 例逐位一致 | 见下「验证（实跑）」 |
| 发现的既有不一致 | `DOUBLE_WISHBONE` 对 `tie_upright_joint_*` 的 `first_body` 与它自己的装配相反（默认 vs 实际 body_a=拉杆）。它没有被消费，因此未动；新的 steering 模板按实际装配声明 | `solver._joint_bodies` 的规则是「`first_body="far"` 表示 owner 在前」 |

### 最后一项｜steering / chassis 的**文件形式**接进装配

**结论：完成。** 此前装配只读「注册的模板对象」，文件里写的 steering/chassis 子系统不参与装配——
一份声明没人读。现在文档里放的子系统就是被建出来的那个。

| 项 | 内容 | 证据 |
|---|---|---|
| 请求 | `AssemblyRequest` 增加 `steering_template` / `chassis_template`（接受注册名或已解析实例），新增 `role_instance(role)` 访问器 | `test_the_vehicle_document_decides_the_composed_vehicle_roles` |
| 转换 | `solver.role_instance_from(子系统)`：把「文件→运行时实例」这一件事抽成一份，`assembly_request_from`（悬架）与 `assembly_request_for`（steering/chassis）共用 | 三个角色走同一条转换 |
| 装配 | `steering._instance` 与 `chassis.build` 优先读请求里的模板；整车路径 `compose_vehicle_runtime` 把它转发到每根轴 | 文件 steering 模板的铰链名（`front_rack_tie`、`front_tie_upright`）出现在整车装配里，内置名（`front_rack_tie_joint_L`）不再出现；文件的 `support` 刚体被建出 |
| 谁决定「是否转向」 | 文件决定**怎么**建一根被转向的齿条；模型决定**是否**被转向。模型把齿条焊死的轴继续用内置 `STEERING_FIXED` | `test_a_bolted_rack_keeps_the_built_in_fixed_template`：整车前轴读文件模板，后轴仍是 `rear_rack_fixed_to_chassis` |
| 为什么 wheel 不在其中 | 轮心是**每侧**挂点，而转换只按 owner 的侧别标记做镜像，且模板只能拥有自己的刚体；轴上 wheel 角色不建刚体（D9：车轮由试验台提供、轮体归模型）。文件因此没有可挂轮心的刚体 | `wheel.py::_instance` 的说明；这是文件格式的边界，不是省略 |
| 逐位不变 | 内置路线的装配指纹逐字节未变（新字段默认 `None`） | 见下表 |

### 验证（实跑）

| 门 | 命令 | 结果 |
|---|---|---|
| 静态 | `ruff check .` | 通过 |
| 静态 | `ty check .` | 通过 |
| 架构 | `legacy_surface_gate.py --check` | 通过，0 findings |
| 架构 | `check_module_layering.py --strict --final` | 通过，0 环、0 退役 |
| 架构 | `check_composable_release.py --skip-isolation` | 通过，3/3 |
| 快速集 | `pytest tests --ignore=adams,architecture,cases` | 通过（本轮前 1002 → 1014 → 现更多） |
| 全量回归 | `pytest packages/suspension_multibody/tests -q` | **1468 passed, 1 skipped, 1 xfailed**（含 `architecture/`、`adams/`、`cases/`） |
| 定向 | `pytest tests/authoring,properties,instantiation,templates,subsystems,vehicle` | 335 passed, 1 xfailed |
| 契约+内核 | `pytest kernel/tests contracts/tests` | 60 passed |
| 数值 | `dynamic_hash_sentinel.py --check` | 与冻结基线逐位一致（sha256 `fdfd5a6b…` 未变） |
| 数值 | `case_parity_check.py` | 8 families PASS，含 `vehicle_dynamic: 8 cases, bit-identical to the frozen snapshot` |
| 数值 | `kc_perf_gate.py` | 在预算内 |
| 结构 | 改前/改后装配指纹 | 逐字节相同（单轴与整车、K 与 C） |
| 空白 | `git diff --check` | 干净 |

**全量回归中发现并排除的环境残留**：`src/suspension_multibody/elements/` 下留着一个退役包（commit
`8cf33f4` 删掉了它）的 `__pycache__`，`tests/architecture/test_legacy_surface_gate.py` 因此报
「legacy package still present」。该目录不含任何源码、不被 git 跟踪，删除后该文件 22 用例全通过。

**未执行**：`just`（本机无），门禁按 AGENTS.md 展开逐条执行；其余长门（架构目录、全量回归）本次已跑。

### 仍未完成 / 边界（逐条说明）

1. **`api.run_case(mode="C")` 对任何模型都只返回 1 个状态且 `z=0.0`**（benchmark 与 `_compliant_model`
   都一样，传 `controls` 也不改变状态数）。C 家族可用入口是 `simulation.run_request`。既有缺陷，未诊断。
2. **C/pad 读数的静态配平不接受任何预载弹簧**（5 mm 即失败，内置模型逐位相同）。这是该读数的性质，
   已用内置对照证明，不是文件路线的问题；未修改内核（计划 Non-Goals 禁止）。
3. **`brake`/`drive` 的 `register_simplified()` 仍无调用点**（既有死代码，未动）。
4. **`DOUBLE_WISHBONE` 的 `tie_upright_joint_*` 的 `first_body` 与自身装配相反**（见上表）；未被消费，
   未改，以免动到既有导出/往返测试。
5. **wheel 角色不能由文件驱动**：轮心是每侧挂点，转换按 owner 的侧别标记镜像，而模板只能拥有自己的
   刚体；轴上 wheel 角色不建刚体（D9）。要让文件描述 wheel 拓扑，需要先让文件格式能表达「每侧挂点落在
   别人的刚体上」，那是一次契约扩展。
6. 未新增求解器、未改 C++ ABI、未重录任何冻结基线。
