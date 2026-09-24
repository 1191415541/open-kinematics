# 实施子任务规格

本文件给出未来执行的完整边界；状态只以 [SUBTASKS.csv](SUBTASKS.csv) 为准，当前均 TODO。本文 `M` 指 `packages/suspension_multibody`，`K` 指 `packages/suspension_kernel`，`C` 指 `packages/suspension_contracts`；命令须展开为仓库相对路径执行。

## 通用执行规则

- 子任务启动时创建 CSV 指定目录中的 SPEC.md、TODO.csv、PROGRESS.md；本文件是其输入，不再从聊天猜需求。
- 每步读取子任务 TODO；记录变更文件、命令、退出码、失败/跳过原因和数值对比。新增测试与下面的脚本是未来交付物，目前未实现。
- CSV 的 validation_command 是该行主验收命令，不替代补充门禁。每行的全部补充门禁必须在该行标 DONE 前实际执行并记录，13 只是独立复验，不承接前面漏跑的放行门。
- 01 冻结既有失败与容差，02-12 每步复跑受影响回归；13 跑完整矩阵。关键新路径必须 native 实跑，不得只检查对象存在或 mock 调用。
- 核心文件、共享注册、schema、CMake、native 镜像只允许一个写者；所有代码任务结束后做只读对抗性审查。
- 新增依赖、协议/ABI/API 破坏、数值预算修改或基线重录须先确认，不能以任务已排入 CSV 当作授权。

## 01 基线、风险与可执行门禁

- 依赖：无。Goal：G9。写范围：`M/scripts/check_composable_baseline.py`、本任务证据与测试清单，不改物理实现。
- 先记录工作区初始差异、源码/夹具/原生库指纹、公共入口/CLI/产物格式、构建信息。已有未提交修改保留，后续不得归为本轮新增回归。
- 在现有固定环境实跑 TASKS 末尾命令集，记录数值数组/哈希、性能预算、失败状态和 skip/xfail 原因；判明旧任务遗留的准静态轮胎/rack 输出问题是否仍存在。
- 给 A1-A10 建立量纲明确的误差标准：几何/轴向、质量/质心/惯量、残差、输出、状态导数和性能。采用现有预算，缺项在修改前依据独立可解夹具确定并写证据；不能运行后随结果调整。
- `check_composable_baseline.py --check` 解析证据清单，验证源码及输入指纹、所需命令实际结果、容差及已知失败归类完整；缺 native、缺必需结果或未分类故障必须非零，不能仅检测文件存在。
- 对本 Goal 相关的既有失败建立归属（准静态轮胎 07、rack 输出 10）；非目标历史限制明确保留。缺失基线导致无法比较时停止该路径迁移。
- 主验收：`uv run --no-sync python packages/suspension_multibody/scripts/check_composable_baseline.py --check`。

## 02 基础模型、SI 与依赖解环

- 依赖：01。Goal：G1;G5。写范围：`M/src/.../modeling/`、`preparation/assembly/types.py`、`core/`、必要导入调用方及 `M/tests/modeling/`、`tests/architecture/test_import_boundaries.py`。
- 将刚体/关节/marker/驱动/力元声明及空间代数归入低层，定义稳定 EntityId、只读 ModelFragment、Assembly/SimulationAssembly 和基础 Port 值对象；不实现具体模板。
- 明确 SI、局部/世界变换与既有四元数约定；输入输出适配保留原外部单位。旧类型暂时转发仅用于迁移，不在此步重写所有物理逻辑。
- 冻结字段与序列化约定，供 03/04/06 消费。模型层禁止向 preparation/rigs/subsystems/simulation 反向导入。
- 验收：随机导入顺序在独立子进程成功；编译期/运行期引用无循环；单位与坐标变换、稳定 ID、重复实体拒绝和不可变输入测试通过；原默认数值回归不变。
- 主验收：`uv run --no-sync pytest packages/suspension_multibody/tests/modeling packages/suspension_multibody/tests/architecture/test_import_boundaries.py -q`。

## 03 模板图生成与实例化

- 依赖：02。Goal：G2;G5;G8。写范围：`templates/`、`properties/`、必要 `modeling` 字段完善及 `tests/templates/`、`tests/instantiation/`、`tests/properties/`。
- 沿用统一 Template + RoleSpec，扩展完整实体图与提供/需求端口；直接声明和 builder 输出同一种 ModelFragment，不把 callable 序列化。
- 模板角色约束只表达对外能力，不包含双横臂私有硬点；支持可变构件数与 0 刚体子系统，属性文件参与确定性指纹。
- K/C 保持双列/单列/纯几何点语义；激活不改变稳定身份、几何和质量。相同输入直接实例化与切换重建一致。
- 验收：两种作者方式输出图等价；不同构件数量真实出现在结果片段；模板 JSON/属性可往返；缺引用、错误单位、重复 ID、必需槽缺失拒绝；K/C/属性既有测试保持。
- 主验收：`uv run --no-sync pytest packages/suspension_multibody/tests/templates packages/suspension_multibody/tests/instantiation packages/suspension_multibody/tests/properties -q`。

## 04 端口组合、自适应与全局策略

- 依赖：03。Goal：G3;G5;G8。写范围：`connections/`、`modeling/ports.py`、`modeling/validation.py`、`tests/connections/`。
- 实现显式映射优先、能力与语义筛选、连接数量、required/optional、歧义拒绝、实例坐标求解和跨实例实体生成。
- 单轴与整车全局 policy 不可被注册覆盖；嵌套整车按根语义检查，禁止被禁止子系统藏进试验台。区分一般试验加载与车辆驱动子系统。
- 检查连接归属、质量唯一性、重复驱动、ground 引用及端口绑定证据；结构 audit 不替代内核约束秩检查。
- 验收：硬点平移和端口转动后，从独立坐标变换公式获得的预期连接位姿一致；原试验台定义未修改；全部非法组合和嵌套绕过负例点名失败；无转向可选分支整体收缩。
- 主验收：`uv run --no-sync pytest packages/suspension_multibody/tests/connections -q`。

## 05 内置物理试验台模板

- 依赖：04。Goal：G2;G3;G7;G8。写范围：`rigs/`、其内置注册入口、`tests/rigs/`；不重写 solver。
- 将现有 K/C、轴动态、整车 K/C、整车动态、操稳、四立柱、随机路面试验意图映射到物理模板及输入/测量配置；能共享的底座共享，不能只改注册名称。
- 至少一个悬架台真实贡献刚体/joint/motion/force，单轴轮体和轮胎由它拥有。整车模式复用已有车轮，不重复创建；接口随硬点与能力变化。
- 统一试验台几何与 Study 的运行语义，当前 family 名留给旧 API 适配而非模板身份。
- 验收：试验台实体图可追踪到声明；轮体归属、激励和输出分支正确；同一台的轴/整车绑定可构建；旧七种默认组合意图与单轴无转向能力保留。生产求解穿透由 07/11 完成，本步不冒称已端到端通过。
- 主验收：`uv run --no-sync pytest packages/suspension_multibody/tests/rigs -q`。

## 06 子系统迁移与唯一仿真总成

- 依赖：05。Goal：G2;G3;G4;G5;G8。写范围：`subsystems/`、`preparation/assembly/`、`preparation/{axle_dynamic,vehicle_dynamic,kc_quasi_static}.py`、`studies/assembly.py`、`studies/bridge.py`、输入 adapters 及相关测试。
- 把现有六类子系统变为模板图提供者，经端口连接生成同一个 SI SimulationAssembly；前后轴嵌套不额外复制轮体/质量。
- K/C 与动态输入 schema 仍可不同，新装配入口归一化为同一 SI 图，不再依赖 K/C 转动态桥接；去掉新链路对共享可变 SubsystemContext 的依赖。
- 本步先验证新装配入口与现役装配逐项等价；旧公共运行入口暂保留可用，不提前切换到尚未实现的编译器。07 负责运行切换和消除生产桥接依赖，12 删除无调用旧文件。简化/复杂替换、K/C 属性与轮胎质量语义保持。
- 验收：实体、连接、端口及输出与既有默认装配逐项对照；轮胎质量只累计一次，总质量/质心/等效惯量守恒；两种 schema 同物理输入生成一致模型；完整单轴和整车构建符合 D3。
- 主验收：`uv run --no-sync pytest packages/suspension_multibody/tests/subsystems packages/suspension_multibody/tests/studies packages/suspension_multibody/tests/vehicle_assembly packages/suspension_multibody/tests/tire_mass -q`。

## 07 正交请求、契约编译与真实 Study

- 依赖：06;08。Goal：G4;G5;G7。写范围：`simulation/`、`compilation/`、`cases/`、`studies/`、必要 `kernel/` 与准备调用点及 `tests/simulation/`、`tests/studies/`。
- 新请求独立提供 assembly/rig/study/case/outputs，取消 rig==family；接通06的SI总成后切换旧公共运行入口到同一编译流水线，消除K/C转动态生产桥接依赖。允许为接线修改api和service调用点，10再拆分其内部职责。
- 编译器只看 SI 图与求解计划，支持任意使用已知元件的拓扑，不读取具体模板名；生成模型、工况与输出布局的唯一实现。
- 准静态轮胎垂向响应必须 native 生效；若 01 发现已有路径缺失，在此闭合；协议表达不足先提问并取得明确变更许可，不能通过作者层隐藏物理替换。
- 验收：同一模型指纹运行两个 Study；准静态无历史依赖且垂向属性改变响应，动态有时间历史；rig 与 family 不同仍实跑；已知失败/部分结果正确传递；新旧公共请求默认行为可对照。
- 主验收：`uv run --no-sync pytest packages/suspension_multibody/tests/simulation packages/suspension_multibody/tests/studies -q`；补充 case_parity 与 kc_parity 门。

## 08 内核能力与处理器注册收敛

- 依赖：01。Goal：G6。写范围：`K/cpp/{include,src}` 的 contract/cases/joint/element/tire/assembly 能力描述与必要 CMake、`M/kernel/capabilities.py`、能力调用点、`C/tests/` 及 `K/tests/test_registry_consistency.py`。
- 工况支持查询与分派来自同一静态描述表；分别表示协议已知、本构建支持和处理器存在，保留未实现名称的明确拒绝语义。
- 关节/力元/轮胎的解析、标量和方向导数能力逐类做一致性断言，减少平行手写清单；不新增物理。
- capability 响应与 Python 消费保持兼容；若必须改变公开协议结构，先提请确认。任何新增依赖边先 review，保持 DAG。
- 验收：枚举每个宣称支持项，使用最小合法文档证明解析和处理器实际可达；未知与已知未实现负例均失败且信息不同；existing ABI/capability/contract 测试通过。
- 主验收：`uv run --no-sync pytest packages/suspension_kernel/tests/test_registry_consistency.py packages/suspension_contracts/tests -q`；构建 native 后运行严格分层门。

## 09 原生状态接口与轮胎行为解耦

- 依赖：08。Goal：G6。写范围：`K/cpp` 的 model/types、tire/tire_state/element、assembly、solve_static/solve_dynamic 及 tests；不改算法定义。
- 提取现有状态布局/初始化/残差/方向导数/投影/接触事件/精确松弛操作，装配时接入执行接口；消除求解器对具体轮胎状态数组与模型枚举的直接业务分支。
- 拆分类型头但维持共享声明低层归属；状态顺序、数值路径、收敛标准和物理律保持，ABI 不变。
- 验收：fiala/pac2002/native_brush 全部已支持状态模式覆盖，标量/方向导数一致，初始化和接受步状态/接触事件回归；动态哈希和性能预算保持。发现不可避免算法变化时暂停，不以重录掩盖。
- 主验收：`uv run --no-sync pytest packages/suspension_kernel/tests -q`；补充动态哈希、K/C 性能和严格分层门。

## 10 输出、公共 API 与兼容结果

- 依赖：07;09。Goal：G1;G4;G7;G9。写范围：`api.py`、`results/`、`outputs/`、`report/`、`io/`、`axle_dynamics/` 和 `vehicle/` 的 service/入口及测试；Adams 调用点仅作 API 适配不改对标物理。
- 解码/列布局/单位归 results，衍生声明归 outputs，指标归 report，检查点归 io；api 保持薄编排。
- 总成与试验台最小输出按实体绑定和命名空间合并；冲突拒绝、缺席分支不填零。若旧 rack 结果未收缩，在此闭合。
- 保留旧返回类型、历史文件读取、失败状态及检查点语义；生产仍需的 Python 反力/静力兼容计算集中并登记，不删除未被 native 取代的能力。
- 验收：旧指标逐值对照，完整公共 API/CLI 和历史产物 fixture 回归；无转向实际结果无 rack 通道；自定义派生量只依赖声明输出；API 不直接承担解码/本构。
- 主验收：`uv run --no-sync pytest packages/suspension_multibody/tests/results packages/suspension_multibody/tests/outputs packages/suspension_multibody/tests/metrics packages/suspension_multibody/tests/architecture -q`；补充原有全量 product 测试。

## 11 新拓扑与新试验台扩展性实证

- 依赖：10。Goal：G2;G3;G8。写范围：新增 `M/tests/composable/`、`tests/data/composable/`；仅允许必要显式注册，不改中央装配/编译/solver 来迁就夹具。
- 使用明确 synthetic 标识的左右拖曳臂单轴模型，其连接图与现有双横臂不同；几何/质量/自由度/预期运动有独立说明。不能伪称真实工程模型。
- 新增一套使用已有物理元件的物理加载台模板，实际贡献刚体/运动副/运动/力，支持同一个台连接双横臂与拖曳臂；同一可兼容台还需连接整车，按全局 policy 处理车轮归属。
- 新模板用显式测试注册经生产 API 运行，不复制编译器。注册之外的核心源文件哈希在扩展前后不变；若需改核心，本任务失败，返回负责模块修正并重审后从冻结边界重验。
- 验收矩阵：双横臂/拖曳臂 x 准静态/动态四个实跑；整车试验台至少一次实跑；单轴和整车各一个平移+姿态扰动后实跑；复杂制动/驱动替换各一次整车实跑；所有结果有限、残差与独立预期相符，不能仅检查 status=success。
- 主验收：`uv run --no-sync pytest packages/suspension_multibody/tests/composable -q`，核心场景禁止 skip/xfail；补充零核心改动哈希探针。

## 12 清理、文档与打包

- 依赖：11。Goal：G9。写范围：确认无调用的旧内部路径、README/CONTEXT-MAP/架构文档、必要 pyproject/CMake/构建脚本、`M/scripts/check_composable_release.py` 及文档示例。
- 完成旧调用迁移清单；删除仅限本次改动造成且已无生产调用者的内部重复路径，公共兼容入口保留。仍需 elements/analysis 的理由和解除条件写现状文档。
- 更新根映射、M 架构说明、K MODULES/README；发布“新增子系统模板/新试验台/硬点更新”可执行示例，不把本方案文件误当现状文档。
- 构建 contracts/kernel/multibody；使用 scratch 下隔离环境安装三包，核查原生库镜像/版本/加载路径，离开源码路径完成一次 native 运行，防 editable 安装掩盖缺包。
- `check_composable_release.py` 负责执行文档示例、产物安装/导入/运行和迁移清单检查，不以 wheel 文件存在为通过。不得自动下载新依赖；环境缺失按阻断报告。
- 主验收：`uv run --no-sync python packages/suspension_multibody/scripts/check_composable_release.py`；补充 ruff/ty 与两侧分层门。

## 13 独立终局验收

- 依赖：12。Goal：G1-G9。写范围：`M/scripts/accept_composable_architecture.py`、必要独立 acceptance 测试与本任务证据；不得修改实现或放宽期望来通过。
- 独立构造 EPIC A1-A10 场景，复用公开夹具但不只读子任务日志；A5 必须独立从两种输入 schema 构造同一物理模型，逐实体比较 SI 数据并追溯模板、参数、属性及绑定来源。运行器汇总每条 Goal/A 项的输入指纹、命令、原生版本、输出断言和结论。
- 重跑基线命令集、核心扩展矩阵、全局规则负例、包隔离运行与源码扩展边界检查；必须拒绝缺失 native、缺少必需场景或 skip/xfail。
- 发现问题返回负责子任务，父 Epic 保持未完成；修复后的集成测试重新执行。未解决的目标缺口不能登记为“不阻断”。
- 主验收：`uv run --no-sync python packages/suspension_multibody/scripts/accept_composable_architecture.py --strict`；脚本需实际执行下列命令集及 A1-A10，不只确认全部任务为 DONE。

## 基线与终局公共命令集

01 核实当前参数并登记，13 独立执行；全部从仓库根目录运行。以下命令是未来实施验证，不是本轮已执行声明。

```text
uv run --no-sync python packages/suspension_multibody/scripts/build_axle_native.py
uv run --no-sync python packages/suspension_kernel/scripts/check_module_layering.py --strict --final
uv run --no-sync pytest packages/suspension_kernel/tests -q
uv run --no-sync pytest packages/suspension_contracts/tests -q
uv run --no-sync pytest packages/suspension_multibody/tests -q
uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check
uv run --no-sync python packages/suspension_multibody/scripts/kc_parity_check.py --check
uv run --no-sync python packages/suspension_multibody/scripts/case_parity_check.py
uv run --no-sync python packages/suspension_multibody/scripts/kc_perf_gate.py
uv run --no-sync python packages/suspension_multibody/tests/architecture/legacy_surface_gate.py --check
uv run --no-sync ruff check .
uv run --no-sync ty check .
uv build --package suspension-contracts
uv build --package suspension-kernel
uv build --package suspension-multibody
git diff --check
```

已存在的命令参数如有差异，由 01 通过 --help/脚本核实后修正文档与 CSV，不更换为更弱验证。旧非目标 skip/xfail 必须记录且不得增长；A1-A10 需要的实跑不允许借此豁免。
