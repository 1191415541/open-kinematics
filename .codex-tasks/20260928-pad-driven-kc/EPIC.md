# EPIC：垫板驱动的 K&C 与力平衡求解

## 背景（用户原话）

> 对于KC悬架测试，我希望还是将试验台改为引入接地垫板（Wheel Pad）来替代直接驱动轮心，
> 车轮与垫板的交互主要通过**"法向弹性支撑 + 切向完全释放 + 实时几何求交"**来实现；
> 垫板不是实际刚体不参与动力学方程，而是解析几何边界。[给定当前步垫板高度 z_pad]
> 1. 运动学投影: 根据铰链约束 Φ(q)=0 获得合法的连杆姿态 q
> 2. 几何求交: 计算 p_patch, 得到当前压缩量 δ_z
> 3. 力元计算: 轮胎支撑反力 F_z(δ_z)、悬架主弹簧/防倾杆恢复力 F_spring(q)、衬套变形反力 F_bushing(q)
> 4. 组装残差方程: R(q, λ) = F_tire(q) + F_spring(q) - Φ_q^T · λ = 0
> ||R|| < tol ? 否 → Newton-Raphson 更新位姿 q，回第 1 步；
> **是 → 收敛，输出当前轮心坐标、轮胎载荷与实时接地点，进入下一扫描步**
> ；总体类似与axle dynamic，使用地面高度驱动轮胎。

用户随后逐条确认的决策：

| # | 决策 |
|---|---|
| 1 | K 扫描「垫板抬升 20mm」——横轴是垫板高度 |
| 2 | 垫板用模型 `road` 解析块表达 |
| 3 | 轮心驱动**并行保留**，但也改成力平衡驱动（考虑力元反力） |
| 4 | K 路线的**纯运动学**分析也要保留 |
| 5 | 垂向轮荷由**弹簧预载自动衍生**，弹簧参与计算，垫板用**强制位移**驱动 |
| 6 | 每个扫描点独立平衡求解**由内核实现** |
| 7 | **C 也改成力平衡 + 轮胎** |
| 8 | 力平衡成为 `kc_quasi_static` 的**新默认**，冻结基线重录（用户已授权） |

## 现状核查结论（本轮实测，非推断）

### 1. 你要的物理机制内核里已经全部存在

| 要素 | 位置 |
|---|---|
| 垫板高度作输入 | `tire/common/kinematics.cpp:33-36` `input.road_z[tire]` |
| 实时几何求交 | 同文件 `:57` `radius + road - center.z`，每步按当前位姿算 |
| 法向弹性支撑（只受压） | `solve_static/kernel_static_contact.cpp:112` `max(0, tire.k*compression)` |
| 切向"释放" | `contract.py:228-242` 中性系数；见 D4 的措辞更正 |
| 力元 + 残差 R(q,λ) | `solve_static/kernel_static_trim.cpp:30-114`，压缩量是未知量 |
| Newton + 活动集 | `solve_static/kernel_static_contact.cpp:283-623` |
| 垫板非刚体 | `cases/contract_model.cpp:1290-1318` 解析高度场 |

### 2. 三处真实缺口（含对上版 EPIC 两处错误的更正）

**(a) `FrontAxleModel` 无 `road` 字段。** 内核解析它（`contract_model.cpp:1290`），
作者侧拒绝：`extra inputs are not permitted`。

**(b) 契约发不出弹簧/减振器/防倾杆——K 与 C 都发不出。**
实测 `cases/kc_quasi_static/contract.py` 中 `spring`/`damper`/`anti_roll` **各 0 命中**，
只有 `bushing`（`_bushing_element`）。给模型声明 2 个弹簧后：

```
K: assembly=['LinearSpringElement','LinearSpringElement']  document=[]
C: assembly=[2×LinearSpringElement, 8×BushingElement]      document=[8 bushing]
```

即**装配体有力元，契约不发**。这是决策 3/5/7 的直接阻塞项：残差里的
`F_spring(q)` 需要契约先把弹簧发射出来。

**更正上版 EPIC 的错误**：上版写「声明力元改变 K 数值 1.307e-03」并暗示 C 已能发弹簧。
实测该 1.307e-03 来自**手工注入的元素行**（`measure_declaration_delta2.py:66-77`
拼 `{"type":"spring",...}` 再比对），**契约当前产不出这个量**。数字本身可复现，
但它描述的是「若契约发了弹簧会怎样」，不是现状。

**(c) `results/` 层无垫板模式的具名输出。** 用户框图要求「输出当前轮心坐标、
轮胎载荷与实时接地点」。实测 `results/kc_state.py` 中
`wheel_center`/`load`/`contact_point` **零命中**，只具名了 penetration 一列
（`_TIRE_PENETRATION_COLUMN = 2`）。内核 `tire_output` 有 41 列
（`mb_config/constants.hpp:21`），列含义（`tire/kernel_tire_assembly.cpp:52-59`）：

| 列 | 含义 |
|---|---|
| 0 | active |
| 1 | `-δ` |
| 2 | `penetration = max(0, δ)` |
| 3 | `-δ̇` |
| 4 | `Fn`（法向力） |
| 5 / 6 | `Fx` / `Fy` |
| 7 / 8 | `vx` / `vy` |

**41 列里没有接触点坐标**，接地点必须由几何算出。

### 3. 一个必须写进实现的校验坑

`assembly/registration.cpp:326` 要求**任何**非零 road（含 `plane`）的
`wavelength > 0`、`bump_length > 0`；`:331` 还要求 `corner_scale`。
平面路面用不到这几个量，但校验不做例外。漏写报
`vehicle road profile parameters are invalid`。

### 4. 决定可行性的物理约束

垫板驱动的收敛窗口窄，且依赖弹簧预载。手工原型（未改内核）扫描实测：

```
  preload          -40    -30    -20    -10     +0    +10    +20    +30    +40
       +0 N    fail   fail   fail   fail   OK     OK     OK     fail   fail
    -2500 N    fail   fail   fail   OK     OK     OK     fail   fail   fail
    -5000 N    fail   fail   OK     OK     OK     OK     OK     fail   fail
    -7500 N    fail   fail   OK     OK     OK     OK     OK     fail   fail
   -10000 N    fail   fail   fail   OK     OK     OK     OK     fail   fail
```

边界处力残差 0.59~0.95，而 `pinned_null_directions=0`——**不是约束欠定，是力不闭合**。

根因三件事叠加：轮胎只受压；K/C 文档 `gravity=[0,0,0]` 且 chassis 固定、
1000 kg 簧上质量落在**固定**体上；自由体质量全是 1.0。于是**没有向下的力**
把车轮压进垫板。实测：打开 gravity **反而更差**（`force_residual` 0.67~1.26）。

**关键**：`kc_native_probe.benchmark_model()` 的
`springs/tires/dampers/bushings/anti_roll_bars` **全为 0**，
即基准 fixture 上没有弹簧可衍生预载。这决定了模式 C 的夹具必须有弹簧，见 D7。

### 5. 「K + 轮胎今天必失败」的机理

实测：`travel=0` 通过、`travel=-20` 失败
（`iterations=1, force_residual=0.000000, position_residual=0.020000`）。
力已平衡、位置残差 20mm 一步未收敛。根因：**规定轮心位移与轮胎法向力互斥**。

### 6. 内核静力配平的粒度（任务 1 待确认，但证据已很强）

`run_model`（`kernel_contract_run.cpp:751`）与 `contract_apply_solver`
（同文件 `:686`）都在 `plan.cases` 循环内（`:633`）；`static_trim` 在 `run_model`
内按 `initialization_mode == 0` 调用（`kernel_abi.cpp:184-198`），
而每个 case 的初始状态从装配位姿重建（`kernel_abi.cpp:173-178`）。
所以「每个扫描点独立配平」很可能是把垫板网格展开成 `sample_count=1` 的多个 case。

### 7. 模式 A 的判据不能用 Python oracle（更正上版 EPIC）

上版写「模式 A 与重录前 K 基线逐位一致」。实测**不可满足**：
冻结的 `tests/data/kc_baseline/k_states.json` 是**退役 Python 求解器**的 oracle，
native 与它 **108 个字段中 76 个不同**，最大差 `1.655482e-06 mm`
（`k-w-10-r-5/right_wheel_center_x_mm`）。

而 native 输出**自身可复现**（同一脚本连跑两次逐字节相同）。所以正确判据是
**「与冻结的 native 模式 A 快照逐位一致」**，见 D6。仓库既有口径也是容差而非逐位
（`case_parity_check.py` 复用 strict Adams 的容差）。

### 8. 现有门禁已经是红的

`scripts/check_composable_baseline.py --check` **exit 1**：
`canonical_library` 与 `mirror_library` 的 sha256 与记录不符
（记录 `b8ea6c64…`，磁盘 `e5e51ce3…`）。且 `justfile` 无任何 recipe 调用它。

### 9. 模式选择的接线现状

`drive_mode` 要走通，须经 `compilation/plan.py:104` 的 `drive_wheels: bool`
与 `compilation/compile.py:99,115` 传给契约。三值无法用 bool 表达，
且这两个文件原不在任何任务的写入范围。

`api.py` 是 case 文档的实际作者（`_k_case_document:528`、`_case_envelope:539`），
原也不在写入范围。

`kc_quasi_static.cpp` 中 `road` **0 命中**，而 K 的轴机制（`:142`、`:277`、`:284`）
**全部**经 `model.driven_index` 解析。垫板不是被驱动坐标，所以它需要一个
**新的 case 节**，不能复用 `axis_map`。上版 D3 把这件事说轻了。

## 模式定义（三模式的发射表）

| | 模式 A `kinematics` | 模式 B `force_balance` | 模式 C `pad` |
|---|---|---|---|
| 驱动 | 轮心位移（`wheel_drive_*`） | 轮心位移 | 垫板高度（新 case 节） |
| 力元（弹簧/减振器/防倾杆/衬套/止点） | **不发** | **发** | **发** |
| 轮胎 | **不发** | **不发** | **发** |
| 残差 | 纯 `Φ(q)=0` | `Φ` + 力元反力 | `Φ` + 力元 + 轮胎 |
| 默认 | 否 | **是** | 否 |

**模式 A 为何"不发"而不是"保持现状"**：现行 `contract.py:216` 是
**无条件**发轮胎（`"tires": _tire_entries(assembly, markers)`）——只要装配体带
`VerticalTireElement`，K 文档就有轮胎，轮胎也就进残差。
所以「保留纯运动学」（用户决策 4）与「保持现状」在**带胎夹具上不是同一件事**。

本 EPIC 取字面义：**A 是纯运动学，一切力元与轮胎都不进残差**。
依据：用户决策 4 是在「力平衡成为默认」的对照下提出保留的，
对照物就是「残差里没有力」这一支；而基准 fixture 上现行 K 本就无元无胎，
所以 A 与现行 K **在基准夹具上等价**。

**由此推出一条验收约束**：模式 A 的冻结快照与验收夹具
**都不得声明轮胎**，否则 A 与快照必然不同（D6/D7）。
A 与 B 的可区分性由**力元**（弹簧）提供，不由轮胎提供。

**模式 B 不含轮胎**是用户已确认的：在被问及路线 B 的轮胎时，
用户选择「不含轮胎，只算弹簧/衬套/防倾杆反力」。

## 目标（Goal）

在 `suspension_multibody` + native kernel 上实现三种并存的 K/C 分析模式
（发射表见上），让力平衡成为 `kc_quasi_static` 的默认，并交付用户框图的三个输出物。

## Non-Goals

- 不改 `axle_dynamic` 及整车动态族。
- 不新增刚体表达垫板（垫板是解析边界，用户明确）。
- 不为垫板引入新的力元类型（复用竖直轮胎分支）。
- 不改 Adams 对标口径。
- 不重构 `static_trim` 的求解算法（Newton/活动集/线搜索保持）。
  **例外见 D8**。

## 关键决策

### D1 模式由 case 文档的 `drive_mode` 选择，默认 `force_balance`

用户选定「单一族 + case 里加 `drive_mode`」。三值见发射表。
接线须经 `compilation/plan.py` 与 `compilation/compile.py`（现状是 bool，
见现状 9），生产入口 `api.py` 一并纳入。

### D2 垫板高度是 **case 级**输入，逐点覆盖

`road` 由 `contract_model.cpp:1290` 从**模型文档**读取，且注册**每 run 一次**；
逐点变高必须走 case 级覆盖（`kernel_contract_run.cpp:644` 的 `run.has_road`）
或 case 级 `run.road_z`（`contract_model.cpp:1762` 拷入 `input.axle.road_z`）。

**收敛为唯一机制**：模式 C 在 case 文档新增一个垫板节（字段名由任务 1 定案、任务 7 实现），
K 展开读它并为**每个垫板高度生成一个 `sample_count=1` 的 case**，
每个 case 把该高度写入 `run.road_z`（按 tire 索引）。
模型级 `road` 只用于静态参数（`plane` + 校验所需的 `wavelength`/`bump_length`/`corner_scale`）。

理由：`run.road_z` 通道已存在且被 `axle_dynamic` 使用（实测 parity 逐位通过），
复用它不需要改 ABI。

### D3 每个扫描点独立配平，由内核实现

用户要求内核实现。实现方式是把垫板网格展开为 `sample_count=1` 的多个 case，
依赖「每 case 各自从装配位姿配平」这一既有行为（现状 6）。
任务 1 负责确证；结论若为「必须重构求解器」，按 D8 停机。

### D4 「切向完全释放」的准确含义

`contract.py:228-242` 的中性系数是 **1.0 而非 0**，docstring 说明其语境是
「a K/C state has no slip」。所以现状是**静力态无滑移**意义上的释放，
不是把切向刚度置零。本 EPIC 沿用该口径并在 Goal/文档中写明，
不新增切向释放机制。

### D5 模式 C 的输出物（用户框图明确要求）

每个扫描点必须产出：**轮心坐标、轮胎载荷、实时接地点**。
现状 `results/` 层无具名通道（现状 2c），须新增。接地点的几何关系写死为：

```
δ  = tire_output[..., tire, 2]            # penetration = max(0, delta)
contact = (center.x, center.y, center.z - (radius - δ))
```

于是 `contact.z == 垫板高度`（这是一条代数恒等式，可作断言）。
数据来源列见现状 2c 的列映射表。

### D6 模式 A 的判据：与冻结的 **native** 模式 A 快照逐位一致

不用 Python oracle（不可满足，现状 7）。做法：**在任何发射改动之前**，
用**同一夹具**冻结一份 native 模式 A 的 K 快照为
`artifacts/acceptance/mode_a_frozen/`，模式 A 的产出一律与该快照逐位比对。
实测 native 输出可复现（连跑两次逐字节相同），故逐位是合法判据。

**该夹具必须由任务 2 自己产出**（不能借用任务 12 的验收夹具——否则任务 2
无依赖却要用后面才存在的东西，顺序不可能）。夹具要求：
- 声明**弹簧**（使 A 与 B 可区分）、**不声明轮胎**（否则 A 与冻结快照必然不同，见上节）；
- 与任务 13 的验收夹具**同一份文件**，由任务 2 建立、后续任务引用。

**谁改发射就要依赖任务 2**：真正改变**力元发射**的是**任务 6**（任务 5 只产出映射表，
不改任何产品代码）。所以顺序约束是 **任务 6 依赖任务 2**，
而不该由任务 5 承担——任务 5 只产出映射表，与冻结快照无关，**任务 5 不依赖任务 2**。
第 4 次修订曾把这条边挂在任务 5 上，第 5 次修订按此更正。

### D7 模式 C 的夹具带预载弹簧；基准 fixture 不动

基准 fixture `springs=0`，无从衍生轮荷（现状 4）。给**专用夹具**加弹簧是安全的：
实测给模型加弹簧后 **K 模式文档逐字节一致**、K 解 `max|delta| = 0.000000e+00`
（因契约不发 K 力元）。所以不影响 D6 的逐位判据。

不把弹簧加进基准 fixture：那会改变模式 B 的力元集，放大基线重录的差异面。

**可达范围门槛**：按用户决策 1（「垫板抬升 20mm」），模式 C 的可达范围须**覆盖
`0 ~ +20 mm`**。不达标即该行 FAIL 并回报用户，不得只记录不可达区间就放行。

### D8 两个可行性闸门：任务 1（垫板轴）与任务 5（力元翻译）

**闸门一（任务 1）**：垫板轴的表达与静力配平粒度。
结论若是「需重构求解器」，本 EPIC **停机并回报用户**。

**已知约束（任务 1 必须纳入）**：内核**禁止 `sample_count == 1`**
（`cases/case_common.hpp:315,339`：`if (count < 2) return fail(error,
"case time needs at least two samples")`），而 K 的所有 case 共享文档级时间网格
（`kc_quasi_static.cpp:157,169`）。所以「每个垫板高度一个 `sample_count=1` 的 case」
不是现成能力。任务 1 必须回答如何达成「每点独立配平」，任务 7 的验收用
「**case 数 == 扫描点数，且每个 case 各自独立配平**」，
把 `sample_count == 1` 降为**候选实现**。

**实测已排除一个担心**（第 4 次修订）：每 case **本来就各自配平**，
且 **case 内样本间漂移为 0**（按 case 分组实测 3×4 样本，
三个 case 的 `max |sample - sample0|` 全为 `0.000000e+00`；
`kernel_abi.cpp:184` 只在 `initialization_mode == 0` 时配平，
每 case 的 trim 诊断行独立）。所以垫板展开**大概率不需要改 `read_time`**，
只需在 `expand_k_axes` 之外加一个垫板展开循环写 `run.road_z`。

**闸门二（任务 5）**：力元的翻译。
现状 `contract.py` 中 `spring`/`damper`/`anti_roll` **各 0 命中**，
只有 `bushing`（现状 2b）。翻译器**已存在**可复用，但不在 K/C 契约里：

| 来源 | 作用 |
|---|---|
| `preparation/vehicle_dynamic.py:743-882` | `assembly.elements` → SI 记录 |
| `cases/axle_dynamic.py:61-202` | SI 记录 → 内核 JSON 元素行 |

差别在单位：`axle_dynamic` 用米，K/C 契约用毫米（`contract.py` 的 `UNITS`）。

**防倾杆：结论已明确，不是「可选」。**
模型层只有一种表示进入装配体：`AntiRollBarElement`（`elements.py:593`），
docstring 原文 "Equivalent torsional anti-roll bar **driven by link vertical travel**"，
`evaluate` 按左右点**垂向位移差** `k*(Δz - ref)` 施加垂向力对。
装配体由 `subsystems/element_build.py:124-133` 构造，把
`AntiRollBar.torsional_stiffness` **直接当作连杆律刚度**使用。

而内核 `anti_roll_bar` 是**扭转式**（`axis_a` + `reference_quaternion` + N·m/rad，
`contract_model.cpp:712-726`）。**两者不是一个东西。**
`preparation/vehicle_dynamic.py:869-873` 明确拒绝把连杆律转成扭转式。
另实测：`vehicle_dynamic.py:270` 的 `anti_roll_bars=()`，
即**生产路径上扭转型没有消费者**（只有 3 个测试文件构造 `AxleAntiRollBar`）。

**所以任务 5 的交付是**：K/C 契约按 `AntiRollBar` 的左右挂点
生成**连杆律垂向力对**。这需要内核支持表达它，两条路：
- 复用现有元素类型（若某个类型能表达垂向力对），或
- 新增内核元素类型 —— 此时 `contract_registry.cpp` 的 `kElements` 与
  `multibody_model.schema.json` 的 element enum **必须同时改**
  （`suspension_kernel/tests/test_registry_consistency.py:274-311` 强制两侧一致）。

任务 5 输出逐元素的字段/单位/语义映射表，**并给出防倾杆的实现选择**。
「BLOCKED 回报用户」只用于「两条路都不可行」这一情形，不得作为常规完成态。

另两处换算细节须写进映射表：spring 的 `free_length = reference + preload/stiffness`
（`vehicle_dynamic.py:752`）；bump_stop 的方向在模型层是字符串 `"bump"`
（`elements.py:635`），内核要 ±1.0（`vehicle_dynamic.py:829`）。

### D8b 规范 schema 必须同时改，且此前无归属

**实测**：`multibody_case.schema.json` 的顶层与 `k` 都是
`additionalProperties: false`，校验器会真的拒绝：

```
case.drive_mode    -> ContractError: $: unexpected fields ['drive_mode']
k.pad              -> ContractError: $/k: unexpected fields ['pad']
k.pad_height_mm    -> ContractError: $/k: unexpected fields ['pad_height_mm']
```

而 `tests/cases/kc_quasi_static/test_contract_documents.py`、
`test_k_axes_grid.py` 等都会调 `validate_case`。
**故 `drive_mode` 与垫板节都必须同时改
`packages/suspension_contracts/src/suspension_contracts/contracts/multibody_case.schema.json`**，
该文件此前不在任何写入范围。模型侧 `road` 已被 `multibody_model.schema.json` 接受
（实测 `model + road` → ACCEPTED），无需改。

该文件归**任务 4 独占**（一次把 `drive_mode` 与垫板节两个字段都加进去，
字段名由任务 1 定案）。任务 7 只实现内核侧读取，**不改该 schema**。

### D8c 契约签名必须先冻结

`model_document(assembly, *, name, drive_wheels: bool = True)`（`contract.py:123`）
与 `case_document(..., drive_wheels: bool | None = None)`（`:487`）要从 bool 变三值。
`drive_wheels` 出现在 **29 个文件**（任意形式 101–102 行）（其中 `drive_wheels=` 关键字 72 处；
三个口径实测：任意形式 101 行 / 关键字行 72 / 文件 29），
分布在 `adams/`（6）、`scripts/`（13）、`tests/`（25）等**此前无归属**的位置。

所以**任务 4 就是那次冻结**：它一次性定义三值字段名、垫板节字段名与签名
（保留 `drive_wheels` 向后兼容），并写入 `multibody_case.schema.json`；
任务 6、7、9 都依赖它。
受影响的调用点按目录归任务 9（它的写入范围已扩至 `adams/`、`packages/.../scripts/`、
`tests/`）。**计数以脚本枚举为准，不冻结数字**（见 Done-When 12）。

### D9 基线重录：冻结对象、归因口径、连带失效

**冻结对象**（更正上版 EPIC）：不是拷贝 Python oracle，而是 **native 模式 A 快照**（D6）。
Python oracle 的那条比较改用仓库既有容差口径（`case_parity_check.py`）。

**归因口径**（更正上版 EPIC）：分两项，各给实测值：
- 「力元进残差」对**基准 fixture** 的贡献——实测为 **0**（该 fixture 无力元）；
- 「基线生产者由 Python oracle 换成 native」——实测最大 **1.66e-06 mm**。

上版要求「归因到力平衡成为默认」在基准 fixture 上是**假归因**，不得如此写。

**留痕**（`kc_parity_check.py:5-9` 自述这种重录是 "re-deriving the oracle from the
implementation under test"）：重录后 `kc_baseline` 由「独立参考」退化为
「回归冻结快照」，须写入 `PROGRESS.md` 与包 README。

**连带失效须全部登记**（上版漏两类）：
1. `adams/strict_k.py:214`（"frozen separately"）
2. `templates/builtin.py:41-49`（"frozen C snapshot cannot be regenerated"）
3. `.codex-tasks/multibody-composable-architecture/tasks/01-baseline/BASELINE.json`
   的 kc snapshot sha256（`check_composable_baseline.py:121-132` 校验）
4. `sources.kernel_cpp.files` 的文件指纹（`:289-305`，任务 7 会改其中 2 个文件）
5. `canonical_library` / `mirror_library` 的 sha256（`:255-264`，重建 DLL 必变）

**状态说明**：该门禁**当前已是红的**（现状 8），且 `justfile` 无 recipe 调用它。
所以它不是「重录触发」而是「本就失配」；任务须把它接进验证命令并说明处置。

**产出重录值的命令须写明**：`--record` 已随 Python 求解器退役，
`kc_parity_check.py --check` 只比对不产出。重录须由 frozen-native 快照机制承担，
具体命令由任务 11 定义并落进验收。

### D10 独立验收（不得自评）

所有 Done-When 若由任务 13 自写脚本自评，则不构成独立证据。须补：
- **模式 A**：与 D6 的冻结快照逐位比对（冻结动作发生在任何改动之前）。
- **模式 B**：在带力元的 fixture 上，A 与 B 之差可由静力学独立预测
  （单自由度约化，闭式给出符号与量级）。
- **模式 C**：单自由度约化闭式解（接地点/载荷），或 Adams 四立柱垫板对标。
- **Adams strict-K**：现有 `strict_k.py:219-223` 的模型只有 hardpoints+mass、
  无元无胎、纯运动学，**对本 EPIC 的新交付零覆盖**，不能当作 B/C 的独立证据。

## 交付边界

1. case 文档可声明 `drive_mode`，三值都可用且经生产入口 `api.py` 走通；
2. `FrontAxleModel` 可声明 `road`（含 plane 参数）；
3. **契约能发射力元**（现状 2b 的缺口），并给出逐元素的字段/单位/语义映射表；
   不能等价翻译的（至少防倾杆的连杆律）有替代实现或明确 BLOCKED；
4. 模式 C 的垫板扫描由**内核**逐点独立配平（case 数 == 扫描点数）；
5. 模式 B 为默认，力元反力进残差；
6. 模式 A 与冻结的 native 快照逐位一致（同一夹具、不含轮胎）；
7. C 改为力平衡 + 轮胎；
8. **模式 C 每点输出轮心坐标、轮胎载荷、实时接地点**（D5 的公式）；
9. 模式 C 覆盖 `0 ~ +20 mm`（D7）；
10. 基线重录留痕（D9 的五类连带 + 冻结对象 + 归因口径）；
11. 内核改动后重建并安装 DLL；
12. `drive_mode` 贯通 `plan.py`/`compile.py`/`api.py`，且契约签名变更
    （`drive_wheels: bool` → 三值）保留向后兼容，全部调用点不破；
13. 架构门、快速测试集、数值门全绿；`check_composable_baseline.py` 的处置已说明。

## Risk Assessment

| 风险 | 处置 |
|---|---|
| 契约发弹簧会改变 C 的现有 8 衬套结果与 C 基线 | 由任务 6 承接，差异量写入 C 的归因；C 基线一并重录（用户决策 7+8） |
| 模式 B 成为默认后大量测试的 K 数值变化 | 任务 9 量化受影响面（已初测 62 个文件读 K 文档），逐项判定「应更新」或「真回归」，写成 `impact.md` |
| 模式 C 可达范围可能不足 20mm | 任务 10 实测；Done-When 7 门槛不达标即 FAIL 并回报用户 |
| 垫板不是被驱动坐标，轴机制可能逼出内核返工 | 列入任务 1 问题清单（D8），结论为否则停机回报 |
| 重录基线后失去独立参考 | D9 留痕 + D10 的独立验收 |
| 内核 ABI 变更 | 禁止改 `AxleInput`/`AbiVersion`；`road_z` 通道已存在 |
| 三模式分支蔓延 | 模式差异集中在契约发射表与 case 展开，不在求解器内分叉 |

## Constraints

- **不引入新依赖**。
- 不改 `AxleInput` 结构体与 ABI 版本。
- 不改 `static_trim` / `solve_static_least_squares` 算法。
  **例外**：用户决策 6 优先；若任务 1 结论表明必须动算法，按 D8 停机回报，不擅自实施。
- 模式 A 与**冻结的 native 快照**逐位一致（D6）；不得用 Python oracle 作逐位判据。
- 契约签名变更须**保留向后兼容**：`drive_wheels` 遍布 29 个文件（任意形式 101 行、
  关键字调用 72 处），三值表达不得破坏这些调用点。计数由脚本枚举，不冻结数字。
- 数值基线重录必须证明差异来源，且**不得作与实测相反的归因**（D9）。
- 不删测试、不弱化断言；旧测试若编码了错误语义，对着 ground truth 改正并在 docstring 说明。

## Done-When

每条都可机器执行；`scripts/acceptance_pad_driven_kc.py` 由任务 13 新建并逐条实现。
**每条注明所用 fixture**（上版缺此，导致判据在所选 fixture 上恒真或恒假）。

1. **三模式可达**：`--check modes`。**分两组 fixture，因为它们的物理要求不同**：
   - **A/B 组**用任务 2 的夹具（含弹簧、**不含轮胎**）——
     模式 A 与 B 都不发轮胎，故无胎夹具正确；
   - **C 组**用任务 10 的夹具（含轮胎 + 预载弹簧）——
     垫板模式的法向力只能来自轮胎，无胎夹具上它必然退化（装配体无
     `VerticalTireElement` 时 `tires == 0`，`contract.py:271-273` 直接返回空）。
   断言：**A 与 B 在 A/B 组上求解成功，C 在 C 组上求解成功**
   （不写「三值在同一夹具上都可解」——那在任一组上都不成立），
   且各自文档的 `elements`/`tires` 名集与发射表一致。
2. **模式 A 数值不变**：`--check mode-a`。fixture = **任务 2 冻结时用的同一夹具
   （含弹簧、不含轮胎）**。断言与 `artifacts/acceptance/mode_a_frozen/`
   （改动前冻结的 native 快照）**逐位一致**，
   且**该夹具上 A ≠ B**（实测差异 `1.307305e-03`，且与弹簧刚度无关——
   差异来自"元素行存在"本身，见 D7 的实测）。
3. **模式 B 是默认**：不写 `drive_mode` 时行为等于 `force_balance`，
   `elements` 名集含力元、**不含**轮胎。
4. **模式 C 由垫板驱动**：`--check pad-drive`。断言扫描横轴是垫板高度、
   轮心驱动坐标在文档中**缺席**、**case 数 == 扫描点数**，
   且每个 case 各自独立配平——证据通道：从 `raw.diagnostics` 读每 case 的
   trim 行（`[3]=trim_iterations`、`[7]=trim_position`、`[9]=trim_force`、
   `[15]=pinned_null_directions`，写入见 `kernel_abi.cpp:318-331`），
   并钉住 `solver.initialization_mode == "static_equilibrium"`。
   不断言 `sample_count == 1`（内核禁止单样本 case，见 D8）。
5. **模式 C 交付三个输出物**：`--check pad-outputs`。断言每个垫板高度都产出
   轮心坐标、轮胎载荷、实时接地点，数量等于扫描点数；且按 D5 的公式
   断言 `contact.z == 垫板高度`（容差写在脚本内），**不是「字段存在」的弱证据**。
6. **「A 不发轮胎」这条规则本身必须被验证**（否则它是永不执行的分支：
   无胎夹具上 `_tire_entries` 本来就返回空，测的是夹具而非发射器）。
   `--check tire-emission`：在同一份**含轮胎**文档上生成 A/B/C 三种模式，
   断言 **A 的 `tires == []` 且 B/C 的 `tires != []`**。
   该断言不参与 `mode_a_frozen` 的逐位比对。
7. **垫板可达范围达标**：`artifacts/acceptance/pad-window.md`。
   夹具含预载弹簧且实测垂向轮荷**非零**（打印数值）；
   给出垫板高度扫描表（收敛与否 + 边界力残差）；
   **可达范围覆盖 `0 ~ +20 mm`**，不足即 FAIL；不可达区间如实列出并说明根因。
8. **C 路线力平衡 + 轮胎**：`--check c-force-balance`。
   断言 C 文档**确实含轮胎与力元行**（名集非空、与 C 组夹具的元素一致），
   且**在轮心为零位移时 C 的残差不再恒等于零**——即解不是纯几何。
   写成可判定形式：同一夹具下，`kinematics` 与 `force_balance` 的
   `body_state` 不逐位相同（实测差异量级已由 A/B 差给出，见 Done-When 2），
   而纯几何模式下二者必须相同。
9. **模型可声明 road 且规范 schema 允许新字段**：`--check schemas`。
   `FrontAxleModel(road=...)` 被接受，契约写进文档，
   垫板参数含 `wavelength`/`bump_length`/`corner_scale`；
   且 **`validate_case` 接受带 `drive_mode` 与垫板节的 case 文档**
   （实测当前被 `ContractError: unexpected fields` 拒绝，
   见 D8b——这证明规范 schema 真的改了，而非只改了 Python 侧）。
10. **快速门与架构门全绿**：`ruff`、`ty`、三条架构门、快速测试集全 exit 0。
11. **数值门、基线归因、连带处置与独立证据**：
    - 数值门 exit 0；
    - `artifacts/acceptance/baseline-attribution.md` 按 D9 的**两项口径**给出差异量
      （力元贡献实测值 + 生产者换人实测值），不得写成与实测相反的归因；
    - D9 的五类连带失效全部处置并留痕；
    - 模式 B/C 的**独立**判定按 D10 给出（不得只靠自写脚本）；
    - Adams strict-K 对标通过，或明确 BLOCKED 并指明缺失的安装/许可
      （必须注明它对本 EPIC 新交付的覆盖范围为零，不得当作 B/C 的证据）。
12. **契约签名向后兼容**：`--check signature`。脚本**自行枚举**全仓
    `drive_wheels` 调用点（`git grep` 为准，不冻结数字——本 EPIC 曾同时写出
    74 与 101 两个口径，冻结一个对不上的数会让该判据恒真或假失败），
    断言每一处仍可调用（三值字段与旧 bool 参数并存），
    且 `drive_wheels=True` 与 `drive_mode="force_balance"` 语义一致（旧默认不破）。
