# suspension_mbd v1 Task Specification

> 本规格冻结用户在 2026-07-29 确认的一期范围。范围变化必须同步更新本文件并在 `PROGRESS.md` 记录原因。

## Goals

- 在同一仓库内新增可独立构建、安装和发布的 Python distribution `suspension-mbd`（import 名 `suspension_mbd`）和独立 CLI `suspension-mbd`，业务代码不得导入现有 `kinematics` 包。
- 实现面向前轴双横臂与齿条转向的准静态多体 K&C 和载荷求解，不包含时间积分。
- 支持运行级互斥的 K 模式与 C 模式，并复用同一套静平衡、驱动、扫掠和诊断框架。
- 支持弹簧预载正向静平衡，以及由目标车高、轴荷或轮荷反求预载的反向配平。
- 支持接地点驱动与轮心驱动、标准 K 扫掠、标准 C 六分量载荷路径、结构化结果和可恢复批处理。
- 使用本机 Adams/Car 2024.1 自带模板建立无人值守、可重复的精度验收。

## Non-Goals

- 不实现速度、加速度、惯性项、时间积分、路谱或瞬态动力学。
- 不把新求解器并入、包装或改造现有 `kinematics` 运动学求解器。
- 一期不支持麦弗逊、多连杆、非独立悬架、横向推力杆或非对称独立悬架。
- 一期不支持推杆、拉杆、摇臂式弹簧减振器，不把 A 臂拆成多根独立连杆。
- 一期不计算车轮自转、轮毂轴承内部载荷分配、转向拉杆轴向柔度或转向柱柔度。
- 一期不交付 GUI。
- 一期只实现线性弹簧、线性 6x6 衬套和整体等效稳定杆；非线性弹簧、空气弹簧和整体梁单元稳定杆只保留可替换接口。
- 不向仓库提交 Adams 专有模板文件。

## Functional Scope

### Topology and symmetry

- 车身为固定基体；每侧包含刚性上 A 臂、下 A 臂、转向节与车轮合并体、刚性转向拉杆。
- 齿条为独立刚体，只保留沿齿条轴线的移动自由度；支持位置锁定/位移驱动、轴向力驱动和自由状态。
- 独立悬架只输入单侧数据，另一侧关于车辆 XZ 中面对称生成；几何、衬套、弹簧、减振器和限值均对称。
- 上下 A 臂各有两个车身安装点和一个外球铰；转向拉杆两端及稳定杆吊杆两端为理想球铰。
- 弹簧与减振器可分别连接车身与上 A 臂、下 A 臂或转向节，允许不同安装点与方向。

### Coordinates and units

- 核心坐标系固定为 `+X` 向后、`+Y` 向右、`+Z` 向上；左侧为负 Y，右侧为正 Y。
- 对外支持显式坐标系转换；载荷可按全局车辆坐标或随车轮姿态变化的轮胎局部坐标输入。
- 默认 I/O 单位为 `mm`、`N`、`N*mm`、`kg`、`deg`；内部转角和求解使用 `rad`；所有文件显式记录单位。
- 符号按车辆侧归一化：正外倾为轮顶向外，正前束为 toe-in，正主销后倾为上点向后，正 KPI 为上点向车内。

### K and C modes

- 每次运行必须且只能选择 `K` 或 `C`。
- K 模式将悬架与转向连接建模为具有正确自由度的理想关节；弹簧、减振器静态力、轮胎、稳定杆和限位仍参与载荷平衡。
- C 模式将每个物理安装点建模为独立线性 6x6 衬套，包含局部坐标系、零载位姿、可选预扭角和初始六分量预载。
- C 模式自动计算并缓存同驱动、同外载的理想 K 参考；输出相对 C 基准增量、局部柔顺率、区间割线值和 C-K 差值。
- 衬套刚度矩阵必须近似对称且无负特征值；允许零刚度方向，但欠约束必须诊断。

### Static balance and mass

- 正向配平由弹簧预载、路面位置和外载求工作位置与轮荷；反向配平由目标车高、轴荷或轮荷反求预载。
- 线性弹簧支持“刚度 + 自由长度”或“刚度 + 装配参考长度下预载力”两种等价输入。
- 减振器准静态模型包含位置相关气体力、预载和可选恒定摩擦力，不计算粘性阻尼力。
- 簧上载荷支持“总簧上质量 + 整车质心/轴距”自动分轴，也支持直接轴荷或轮荷。
- 默认前、后轴簧下质量分别为对应轴簧上质量的 10% 和 12%，比例可配置；独立悬架左右均分。
- 默认每侧集中簧下质心位于轮心，并允许在转向节局部坐标中覆盖。
- 可选详细零件质量与局部质心；详细质量从目标簧下总质量扣除，剩余质量继续集中，超额直接报错。

### Force elements and contact

- 接地点驱动输入左右路面台架垂向位移；轮胎采用只受压、不受拉的单一垂向线性刚度，求解轮胎压缩、垂向力和轮心位置。
- 轮心驱动直接约束轮心位移，不启用轮胎刚度；仍允许在接地点施加外载荷和力矩。
- 纵向、横向载荷和六分量力矩不引入对应轮胎柔度。
- 稳定杆一期输入左右车身安装点、左右弯臂端点、左右吊杆悬架连接点及整体等效扭转刚度。
- 稳定杆车身安装点在 K 模式为理想转动支承，在 C 模式为两个独立 6x6 衬套；未来梁单元替换整个扭杆段和弯臂段。
- 限位块和回弹限位采用“间隙 + 接触后线性刚度”，可按减振器行程或任意两刚体安装点间距离定义。

### Cases and sweeps

- K 预设包括左右同向轮跳、反向轮跳、左右单轮轮跳、齿条扫掠，以及轮跳与齿条二维组合扫掠。
- 二维组合扫掠默认同向轮跳，允许改为反向或单轮；典型规模为 `10 x 10 = 100` 个工作点。
- C 预设包括纵向、横向、垂向力，回正、倾覆、制动/驱动力矩，以及任意六分量组合路径。
- C 载荷支持单轮、左右同向和左右反向模式；单轴默认 11 个包含零点的对称载荷级别，范围和点数可配置。
- C 默认仅在静态基准执行，也可选择部分或全部 K 工作点；组合载荷按同步路径展开，不创建六维笛卡尔积。
- 每个受控自由度只能选择位移控制或载荷控制，冲突必须在求解前拒绝。

### Outputs and diagnostics

- 输出目录包含 `manifest.json`、状态汇总表、部件载荷表、衬套结果表和告警表；Parquet 为主格式并支持 CSV。
- 状态表记录驱动、外载、轮胎压缩、刚体位姿、K&C 指标、绝对值、基准增量和 C-K 差值。
- 部件载荷按工况、部件和连接端输出全局及局部六分量载荷、限值利用率和超限状态。
- 衬套结果输出局部六分量变形、载荷、应变能、刚度标识和零载位姿。
- 每个状态保存几何约束残差、力残差和力矩残差。
- 轮胎脱离、限位切换属于有效状态；部件超限继续求解并告警；非法刚度、欠约束、过约束或平衡失败属于计算失败。
- 扫掠使用延续、自适应步长和失败恢复；支持进程数限制、逐行检查点和断点续算，并保证单/多进程数值一致。

## Constraints

- **Language/runtime**: Python 3.12+，NumPy/SciPy/Pydantic/PyArrow/Typer；不引入 GUI 依赖。
- **Packaging**: 在 `packages/suspension_mbd/` 建立独立 `pyproject.toml`、源码、测试和 wheel，并作为根 uv workspace 成员；根 `kinematics` wheel 不包含 `suspension_mbd`，两个包互不导入。
- **Public API**: Python API、版本化 YAML/JSON model/case、独立 CLI 共用同一领域模型与求解入口。
- **Platforms**: 计算核心、schema、CLI 和普通测试必须支持 Windows 与现有 Ubuntu CI；Adams 适配和验收仅在本机 Windows 运行。
- **Performance**: 本机全部物理核心下，固定 `k-100` 基准不超过 30 秒；固定 `c-6600` 基准不超过 300 秒。
- **Performance scope**: 计时包含读取、静平衡、内部 K 参考、求解、诊断和 Parquet 写入，不包含 Adams 运行；失败重试计入。
- **Accuracy**: 对 Adams 等价模型，位置误差不超过 `max(0.1 mm, 0.2%)`，角度误差不超过 `max(0.02 deg, 0.5%)`，力、力矩及柔顺率不超过 `max(绝对容差, 2%)`；绝对载荷容差在取得模板量级后冻结。
- **Provenance**: 结果记录包/格式/schema 版本、输入哈希、Adams 版本、模板标识、求解设置和坐标映射。
- **Compatibility**: 现有 `kinematics` API、CLI、测试和结果格式保持不变。

## Architecture Decisions

- **Package boundary**: `packages/suspension_mbd/` 是独立发布单元；只共享仓库、uv lock 和 CI 编排，不共享 `kinematics` 领域代码或结果 writer。
- **Pose representation**: 刚体状态保存为 SE(3) 位姿（平移 + 单位四元数），Newton 求解变量使用每刚体 6 维局部位姿增量并通过 retraction 更新，避免把四元数归一化作为额外物理约束。
- **Equations**: 统一求解几何约束、混合控制和静力/力矩平衡；理想约束反力由 Lagrange 乘子恢复，C 模式弹性连接直接贡献广义力与一致切线。
- **Nonlinear solve**: 使用稀疏 Newton/KKT 线性系统、阻尼线搜索、秩诊断、延续与自适应子步；不得退化为只最小化位置残差的旧 LM 路线。
- **Derivatives**: 空间变换、约束和力元提供解析一致切线，并逐项用中心有限差分核验；不把全系统有限差分作为生产路径。
- **Unilateral elements**: 轮胎和限位采用显式主动集与单侧切线；状态切换记录事件，切线柔顺率不得跨越主动集切换直接计算。
- **Scaling**: 内部工程单位保持 `mm/N/N*mm/kg/rad`，KKT 残差和变量使用显式尺度向量；6x6 衬套的对称性与能量正定性在单位归一化后的合同变换矩阵上判定。
- **Trim boundary**: v1 固定车身和路面基准；簧上质量用于生成目标轴荷/轮荷和配平约束，不给固定车身增加虚假运动自由度。正向配平同时报告计算轮荷与目标载荷差。
- **Determinism**: 网格并行按行分片、行内延续；结果按稳定 case id 排序，检查点包含 model/case/solver 哈希，配置变化后禁止错误续算。

## Benchmark Definitions

- `k-100`: 10 个同向轮跳位置乘 10 个齿条位置，共 100 个状态；包含读取、一次配平、求解、诊断和 Parquet 输出。
- `c-6600`: 基于同一 100 个工作点，对 6 个标准单轴载荷路径分别计算 11 个对称载荷级别，共 6600 个 C 状态；100 个 K 参考只计算一次并缓存。
- 两项基准均要求所有物理有效状态收敛、残差达标且结果完整；超时、漏点、降低精度或跳过失败点均视为失败。
- 性能报告记录 CPU 型号、物理/逻辑核心数、内存、worker 数、Python/NumPy/SciPy 版本、冷/热缓存状态和各阶段耗时。

## Adams Validation Contract

- 里程碑 2 必须定位 Adams/Car 2024.1 可执行入口、许可证探测方式、自带双横臂前悬模板标识、数据库路径和批处理结果导出字段，并固化为本机 profile；缺失安装或许可证时本机验收命令必须明确失败，普通 CI 才允许通过 marker 排除。
- 先建立理想关节 K 等价模型，再逐层加入相同的轮胎、弹簧预载、6x6 衬套、稳定杆和边界条件；禁止直接比较参数不等价的默认模板结果。
- Adams 原始输出先缓存到任务目录 `raw/`；仓库只保存参数映射、非专有数值基准、来源清单和比较报告，不保存模板、数据库或可还原专有模型的文件。
- 里程碑 2 在取得模板量级后冻结力和力矩绝对容差；里程碑 12 执行 K 几何、C 柔顺性和静态部件载荷三组完整验收。
- 数值验收通过显式外部 runner 和非专有 reference 结果衔接 Adams；缺少 runner、reference、结果组或字段时必须失败，不能退化为 profile 契约通过。

## Environment

- **Project root**: `C:/杂件/open-kinematics`
- **Language/runtime**: Python 3.12.13
- **Package manager**: uv 0.11.11
- **Build backend**: Hatchling
- **Test framework**: pytest 8.3.4
- **Existing tests**: 548 selected / 549 collected（1 deselected）
- **Build command**: `uv build`（现有包）与 `uv build --project packages/suspension_mbd`（新包）
- **Quality command**: `just check`
- **External validator**: Adams/Car 2024.1（用户确认已安装；批处理入口与模板路径在里程碑 2 核验）

## Risk Assessment

- [ ] Adams/Car 批处理命令、许可证可用性、自带模板标识和结果导出字段尚需在里程碑 2 核验；命令必须区分“CI 未安装可跳过”和“本机 require-installed 必须失败”。
- [ ] 6x6 衬套中平移/转动耦合块的单位、坐标变换和有限转角度量存在高错误风险。
- [ ] 轮胎脱离和限位切换引入非光滑主动集，可能影响延续和切线柔顺率。
- [ ] 理想关节反力与柔性安装载荷的唯一性必须通过秩检查和独立平衡算例验证。
- [ ] 6600 点/300 秒要求依赖稀疏切线、缓存、延续和并行；必须在功能正确后单独优化。
- [ ] Adams 默认模板可能包含非线性衬套、复杂轮胎或不同边界，比较前必须建立参数等价映射。
- [ ] uv workspace 与两个独立 wheel 的锁文件、脚本解析和 CI 工作目录需要在里程碑 1 验证。
- [x] Adams 专有模板不提交仓库，只保存模板标识、映射脚本、导出数值和来源记录。

## Deliverables

- `packages/suspension_mbd/` 独立 distribution、公共 API 和 `suspension-mbd` CLI；根项目保留原有独立 wheel。
- v1 model/case/result schema、YAML/JSON loader、严格验证和迁移边界。
- 刚体、理想关节、6x6 衬套、弹簧、减振器静态力、轮胎、稳定杆、限位和质量模型。
- 双横臂前轴与齿条装配器、对称生成器、正向/反向静平衡求解器。
- K/C 工况、指标、部件反力、柔顺率、C-K 参考与诊断。
- CSV/Parquet/JSON 结果、检查点/续算、多进程批处理和 CLI。
- Adams/Car 2024.1 批处理适配、等价模板映射、精度对比报告和非专有基准数据。
- 单元、性质、集成、性能、Adams 验收测试及用户文档。

## Done-When

- [ ] `suspension_mbd` 可独立构建、安装和导入；根 wheel 不包含新包，新 wheel 不包含旧包；两个包源代码互无业务导入。
- [ ] 所有 v1 schema、领域对象、求解功能、工况、结果与诊断满足本规格。
- [ ] 100 点 K 与 6600 点 C 性能测试在本机达到 30 秒/300 秒目标，全部有效状态收敛。
- [ ] Adams/Car 2024.1 自带双横臂前悬等价模型的 K、C、载荷结果满足精度阈值。
- [ ] 常规 CI 不依赖 Adams 许可证；本机 Adams 验收可一条命令无人值守执行。
- [ ] 现有 `just check`、两个 wheel 构建、新包 lint/type/test、续算一致性、性能测试和 Adams 验收全部通过。

## Final Validation Command

```powershell
just check && uv build && uv run --project packages/suspension_mbd ruff check packages/suspension_mbd && uv run --project packages/suspension_mbd ty check packages/suspension_mbd/src && uv run --project packages/suspension_mbd pytest packages/suspension_mbd/tests -m 'not adams and not performance' --strict-markers -q && uv build --project packages/suspension_mbd && uv run --project packages/suspension_mbd pytest packages/suspension_mbd/tests/performance -m performance --strict-markers -q && uv run --project packages/suspension_mbd suspension-mbd validate-adams --profile adams-car-2024.1 --full --require-installed
```

## Demo Flow

1. 使用 v1 模型文件加载对称双横臂前轴和齿条系统。
2. 运行反向配平，由目标轮荷和车高求弹簧预载并检查静平衡残差。
3. 运行 10x10 K 轮跳/齿条网格并导出结构化结果。
4. 在静态基准运行 11 级标准 C 载荷路径，检查柔顺率、C-K 差值和部件载荷。
5. 中断并恢复一次批处理，验证检查点一致性。
6. 执行 Adams/Car 批处理验证并生成误差报告。
