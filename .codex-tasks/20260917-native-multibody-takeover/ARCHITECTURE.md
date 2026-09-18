# 目标架构：契约边界 + 模块化内核

状态：2026-09-17 定稿（用户确认 5 项设计选择）。本文件是模块划分与职责划分的唯一依据；
`MODULARITY_AUDIT.md` 是现状证据，`EPIC.md` 是执行计划。

## 0. 已确认的设计选择

- **B1 文档边界**：Python 与 C++ 之间只交换版本化契约文档，不共享内存布局。
- **JSON Schema 单一来源**：契约由一份 JSON Schema 定义，Python 与 C++ 各自消费。
- **Adams 留在 Python**：C++ 不认识 Adams；Python 不认识物理求导。
- **编译期注册表**：扩展点是表驱动 + 接口头，不做运行时插件。
- **Adams 侧瘦身**：工况定义收敛到契约后，`adams/` 从「重新实现工况」变为「渲染工况」。

## 1. 边界：一份契约，三类文档，一个入口

### 1.1 三类文档（同一契约包的三个 `kind`）

**Model document**（多体模型）
`bodies[]`（质量/惯量/初始位姿速度/固定标志）、`joints[]`（类型 + 两端体/标记/轴）、
`elements[]`（表驱动：`type` + 各自参数）、`tires[]`（模型种类 + 参数）、`road`（路面类型 + 参数）、
`units`、`gravity`、`capabilities`（声明本模型需要哪些能力）。

**Case document**（工况）
`family` ∈ `kc_quasi_static | axle_dynamic | vehicle_kc | vehicle_dynamic | handling |
ride_four_post | ride_random_road | comparison`，加该工况族的**声明式**定义（网格/载荷路径/
侧模式/时程/激励）与 `solver_settings`。**不含展开后的 case 列表** —— 展开由 C++ 工况层完成。

**Result document**（结果）
`case_identity`（契约文档规范化哈希）、`model_hash`/`case_hash`/`grid_hash`、
`status`、`diagnostics`、`manifest`（契约版本/ABI 版本/编译器/构建元数据）、
`blocks[]`（大数组：名称 + dtype + shape + 通道顺序 + 偏移）。

### 1.2 容器格式（避免 base64 膨胀）

```text
payload := header(4B magic, 4B version, 8B json_len, 8B blob_len) | json | blob
```

**统一数组描述符**（复审后补充，必须覆盖全部现有 ABI 形态）：

```json
{"offset": 0, "length": 1024, "dtype": "float64", "shape": [128, 8], "order": "C", "count_kind": "flat"}
```
- `offset/count` 型曲线：`{"offsets": [...], "counts": [...], "values": {...}}`（沿用当前 `spring_damper_curve_offset/count/velocity/force` 的语义，`native.py:151-154`）。
- 二维矩阵经 `shape` 表达（如 `stiffness_6x6`，`native.py:155-166`）。
- 字符串/名称数组同样进 blob（UTF-8，长度前缀），JSON 只放描述符。
- **禁止 JSON 承载大数组**：JSON 只放计数、类型、拓扑、标量参数与数组描述符。

**解析器策略**（复审结论）：引入受控的 header-only JSON 库（优先 nlohmann/json），**只解析小型 JSON header**；
blob 由手写边界读取器按描述符直接读。强制限制 payload 长度、嵌套深度、数组描述符合法性。
不建议为「零依赖」口号自写通用 JSON 解析器——版本化契约的正确性与安全性成本高于收益。

### 1.3 规范化与哈希：两把尺子，不要混用（复审后修订）

原方案把「契约哈希」与「动态输出逐位不变」当作同一件事，这是**自相矛盾**的（复审阻断项）。
现拆成两道独立门：

**门 A —— 契约身份门（可严格做到）**
- JSON：键排序、无冗余空白、UTF-8；浮点按契约规定的 canonical 十进制（`%.17g` 或纯整数/`float` 类型分离）；
  禁止 NaN/Inf（或显式编码）。
- blob：小端、显式 dtype/shape、按声明顺序。
- 哈希 = 规范化字节 SHA-256；`case_identity` 即 Case document 的哈希。
- 必须配套**往返性质测试**：随机 `float64` → 序列化 → 解析 → **位比较**。
- 注意：现有 `canonical_hash` 用的是 Python `json.dumps(sort_keys=True, separators=(",",":"))`
  （`src/suspension_multibody/io/results.py:21-26`），**没有** `%.17g` 规则；契约定稿时必须替换为
  契约规定的规范化实现，否则门 A 不可信。

**门 B —— 数值 parity 门（严格度取决于阶段）**
- P3（纯结构重构）：固定工具链/线程（单线程）/线性后端/算法顺序时，要求结果数组**字节级一致**。
- P4（边界切换）：先由门 A 的往返测试证明「契约解析出的 double 与 Python 原值位相同」，
  再要求结果字节级一致。
- P5/P6（新求解路径与新工况族）：新算法不可能与旧实现逐位一致，改为**容差 parity**
  （继承 `adams/strict_k.py:388-403`、`adams/strict_c.py:407,422-424`）+ 事件/诊断一致性。
- 结论：**不可跨阶段滥用「逐位不变」**；它只适用于 P3/P4，且必须写明环境固定条件。

### 1.4 ABI（唯一入口）

```c
uint32_t suspension_kernel_contract_version(void);
int32_t  suspension_kernel_run(const uint8_t* payload, size_t payload_len,
                               uint8_t** result, size_t* result_len,
                               char* error, size_t error_len);
void     suspension_kernel_free(uint8_t* buffer);
```

'- 跨边界只用 POD；内存由 Python 申请、C++ 填充、Python 释放。
- **调用粒度（复审后补充）**：入口保持单一符号，但**不允许「一次调用 = 整个 Case 的全部展开结果」**。
  采用「模型句柄可缓存 + 有界批次」：模型首次传入后可复用；K100/C6600 按固定 batch size 分块；
  长时程动态工况按时间块切分并携带确定性 continuation token。否则性能门测到的是序列化与内存峰值，
  而不是内核吞吐。'
- **契约版本与内部模块解耦**：新增字段 = 契约次版本 +1；不兼容变更 = 主版本 +1。
  内部模块拆分不再触发 ABI 变更（这正是当前痛点的解药）。
- 旧的 flat ABI（`axle_run`/`vehicle_run` + ctypes 镜像）在 Phase 3 与 Phase 4 逐步淘汰。

## 2. 职责划分

| 职责 | 归属 | 说明 |
|---|---|---|
| 物理语义（关节残差/Jacobian、力元本构、轮胎模型） | C++ | 数值密集、需确定性 |
| 装配（度数编号、约束装配、质量矩阵、凝聚） | C++ | 与求解同进程 |
| 静态/准静态/动态求解 | C++ | |
| 工况展开（网格、载荷路径、侧模式、四柱激励） | C++ 工况层 | 声明式输入在契约，展开在 C++ |
| 契约定义（schema） | 契约包（JSON Schema） | 两侧唯一来源 |
| 契约校验 | Python 早校验 + C++ 防御性复校验 | |
| 作者体验（YAML/Pydantic/CLI） | Python | |
| 结果落盘/绘图/DataFrame/报告 | Python | |
| Adams 数据集/DCF/ACF 生成（渲染） | Python | 只读契约，不定义工况 |
| Adams 结果解析/对标/门禁 | Python | 保持对标独立性 |
| 线性代数后端 | C++ | 可切换实现 |

**硬规则**：C++ 不出现任何 Adams 字样；Python 不出现任何残差/本构/Jacobian 计算。

## 3. 模块划分

### 3.1 C++：七层

```text
L0  mb_numeric   向量/矩阵/四元数/旋转/曲线插值
    mb_dual      方向导数（AD）标量与几何
    mb_config    运行时与线性后端配置（从当前 mb_base 抽出）
L1  mb_contract  契约文档解析/校验/规范化/哈希 + 容器格式
    mb_model     纯数据：Body/Joint/Element/Tire/Road + 拓扑查询
'L2  mb_energy    物理账本与求解控制类型（EnergyRates/EnergyStorage/EnergyInterval/StaticContactOverride）
    mb_joint     关节语义（10 类）+ joint_registry'
    mb_element   力元语义（弹簧/阻尼/衬套/防倾杆/限位/气动/转向执行器/驱动制动力）+ element_registry
    mb_tire      轮胎本构（fiala/pac2002/brush）+ tire_registry
'L3  mb_assembly  模型→系统：自由度/坐标编号与注册、约束装配、质量矩阵、焊接体凝聚、
                 轮胎 frame 与道路挂接注册、gauge 的模型侧注册（复审后补全注册职责）'
    mb_force     力装配总线（重力/外力/元素力/轮胎力/驱动制动力）
L4  mb_linear      线性后端（dense/blocked/sparse/GMRES/Pardiso）
    mb_solve_static    静态 + 准静态（残差/Jacobian/gauge/投影/接触活动集/trim）
    mb_solve_dynamic   动态（积分器/Newton/residual/事件/内部状态）
L5  mb_cases     工况家族 + case_registry
L6  mb_output    状态/约束力/元素/轮胎/能量/性能
L7  abi           唯一入口 run_contract_document
```

**相对现状的关键变化**：`mb_base` 拆成 numeric/dual/config；`mb_constraint`→`mb_joint`；
`mb_suspension`+`mb_vehicle` 的力元部分→`mb_element`；**`mb_vehicle` 作为模块消失**
（注册→`mb_assembly`、力装配→`mb_force`、转向/驱动制动/气动→`mb_element` 类型），
因而 `mb_vehicle → mb_static` 这条反向边自然消失；求解层拆成 static/dynamic 两个独立模块。

### 3.2 Python：七个模块（大幅变薄）

```text
schema/   契约作者侧（Pydantic，与 JSON Schema 对齐）
cases/    工况的作者侧构造（薄；展开在 C++）
kernel/   唯一入口薄包装（组包/解包，目标 <200 行，替代今天 1975 行的 native.py）
io/       契约文档读写、结果落盘
report/   结果分析、绘图、DataFrame、指标展示
adams/    渲染（契约→Adams 数据集/DCF/ACF）、解析、比较、门禁
cli/      用户接口
```

**删除**：`core/`、`elements/`、`solver/`、`dynamics/`、`model/`、`analysis/` 的求解与后处理职责、
`vehicle_dynamics.py`、`pac2002_scope.py`（能力声明改由契约的 `capabilities` 承载）。

## 4. 扩展点（编译期注册表）

```cpp
struct JointTypeDescriptor   { const char* name; int rows; ResidualFn; JacobianFn; ... };
struct ElementTypeDescriptor { const char* name; ForceFn; EnergyFn; TangentFn; ... };
struct TireModelDescriptor   { const char* name; StateWidthFn; ForceFn; TangentFn; ... };
struct CaseFamilyDescriptor  { const char* name; ValidateFn; ExpandFn; RunFn; };
```

求解器**只与描述符表对话**，不 include 任何具体实现头。新增能力 =
新增一个实现文件 + 在对应注册表登记一行（+ 契约 schema 增加字段），**不改求解器、不改 ABI**。

## 5. 迁移路线（每阶段都有行为门禁）

| 阶段 | 内容 | 门禁 |
|---|---|---|
| P0 | 工具链 + 验证工具（哈希 sentinel / K-C parity / 分层 DAG 检查 / 性能门） | 工具自测通过 |
| P1 | 冻结 oracle（动态输出哈希 + K/C 快照 + 性能基线） | record 成功 |
| P2 | 契约落地：JSON Schema + 容器格式 + 规范化规则 + Python Pydantic 对齐 + C++ `mb_contract` | 契约往返测试；哈希稳定 |
| P3 | C++ 内部重构（按第 3 节分层、破聚合头、注册表化），**不加新能力** | 动态哈希逐位不变 |
| P4 | 边界切换：唯一入口 ABI + Python `kernel/` 薄壳；旧 flat ABI 双跑对照 | 新旧入口结果一致 + 哈希不变 |
| P5 | 求解接管：`mb_solve_static` 加准静态 + `mb_cases` 的 K/C 族 | K/C parity 门通过 |
| P6 | 其余工况族（整轴动态/整车 K-C/整车动态/操稳/平顺-四柱/随机路面/对标） | 各族 parity + 性能门 |
| P7 | Python 瘦身：删求解层/model/analysis；Adams 改为渲染器 | 全量测试 + Adams 门 |
| P8 | 验收：性能、wheel、CI、文档 | 全部通过 |

## 6. 风险

- **R1 浮点往返**：契约必须保证 JSON 十进制往返无损，否则哈希门失效。P2 用性质测试（随机浮点 → 序列化 → 解析 → 位比较）验证。
- **R2 解析开销**：小规模高频调用（K 扫描）需实测；必要时批量传工况。
- **R3 一次性投入**：契约 + 唯一入口 ABI + C++ 重构是前置成本，收益从 P4 起体现。
- **R4 Python 侧独立性**：Adams 留在 Python 是对标独立性的保障，P7 不得把 Adams 逻辑迁入 C++。
- **R5 LTO**：P3 的模块拆分可能改变产物，哈希门是唯一裁决者；漂移即回退该拆分。
## 7. 复审修订记录（2026-09-17）

三路复审（边界可行性 / 迁移顺序 / 分层闭合）发现的阻断项与处置：

1. **契约哈希与「输出逐位不变」自相矛盾** → §1.3 拆成门 A（契约身份）/ 门 B（数值 parity，分阶段定严格度）。
2. **JSON 承载大数组 / 自写解析器风险** → §1.2 规定统一数组描述符 + 只解析小 JSON header +
   受控 header-only 解析库；禁止 JSON 承载大数组。
3. **单一入口的调用粒度** → §1.4 规定模型句柄可缓存 + 有界批次 + continuation token。
4. **能量类型归属**（`mb_suspension→mb_vehicle`、`mb_tire→mb_vehicle` 两条反向边的根源）→
   新增 L2 `mb_energy` 中性模块；**禁止放 L6 `mb_output`**（否则造成新的反向依赖）。
5. **`add_vehicle_static_rotation_gauges` 无处安放** → 拆成两半：gauge 约束/投影归 `mb_solve_static`，
   输出通道声明归 `mb_output`。
6. **其余注册职责未明确** → `mb_assembly` 明列：DOF/坐标注册、轮胎 frame 与道路挂接注册、
   gauge 模型侧注册；元素语义注册归 `mb_element`，轮胎本构注册归 `mb_tire`。
7. **`mb_contract` 与 `mb_model` 的依赖方向** → 单向：`mb_contract → mb_model`；
   `mb_model` 不得 include `mb_contract`；JSON DTO 不直接当内部物理模型。
8. **激励定义的三处归属** → 声明式字段/枚举在 L1 契约；展开与采样在 L5 `mb_cases`；
   物理施加在 L2 `mb_element`/`mb_tire` 与 L3 `mb_force`。
   `RoadSurfaceSpec.kind`（含 `four_post`，`schema/vehicle.py:196`）字段属 L1，`four_post` 语义展开属 L5。