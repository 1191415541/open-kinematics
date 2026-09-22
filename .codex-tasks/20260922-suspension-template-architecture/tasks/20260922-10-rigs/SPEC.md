# 子任务 10：试验台抽取与总成 × 试验台正交组合

## 目标

把现有 7 个写死的 `(assembly, family)` 枚举，改成「总成」与「试验台」两个独立注册轴的正交组合。

1. **试验台抽取**：把 case 侧职责（驱动哪些坐标、时间/载荷怎么施加、求解器默认、结果怎么排布）抽出为独立的「试验台」概念，与总成的模型侧职责分离。
2. **总成注册**：总成提供体、副、元件、轮心、可驱动坐标的名义空间、以及**自己的最小单位输出**（07 已定义输出声明）。
3. **试验台注册**：试验台声明它要求总成提供哪些坐标、如何驱动、以及**自己独有的输出**。
4. **正交组合**：运行 = 选一个总成 + 选一个试验台；组合矩阵可查询；无效组合在准备阶段报错并点名（哪个坐标、由谁提供）。
5. **新增组合**：至少落地一个此前不存在的组合，证明正交性有效（候选：`axle × ride_four_post` 或 `axle × ride_random_road`）。

## 非目标

- 不改各 family 的物理语义（驱动坐标的含义、时间网格的意义、求解器默认值）。
- 不删除现有 7 个组合；它们必须继续可用且行为不变。
- 不实现输出求值本身（07 已做）；本步只把总成输出与试验台输出**接进组合**。
- 不改 C++ 内核与契约格式。
- 不做"总成组合"（如整车 = 两个轴总成之和）——用户的子系统组装分层由 04 的子系统与 05 的实例化承担；本步只做**总成 × 试验台**这一维正交。

## 现状事实（制定计划时实测，实施时复核）

现有 7 个写死的组合（每个一个 compiler，`simulation/dispatch.py:20-33`）：

| assembly | family | compiler |
|---|---|---|
| `axle` | `kc_quasi_static` | `KcQuasiStaticCompiler` |
| `axle` | `axle_dynamic` | `AxleDynamicCompiler` |
| `vehicle` | `vehicle_dynamic` | `VehicleDynamicCompiler` |
| `vehicle` | `vehicle_kc` | `VehicleKcCompiler` |
| `vehicle` | `handling` | `HandlingCompiler` |
| `vehicle` | `ride_four_post` | `RideFourPostCompiler` |
| `vehicle` | `ride_random_road` | `RideRandomRoadCompiler` |

内核侧 family 由 `cpp/src/cases/case_dispatch.cpp:17-19` 的 `contract_case_supported` 枚举（7 个 + `comparison` 按设计 N/A）。

**"试验台"目前散落在各 family 内部**：
- `axle × kc_quasi_static`：驱动坐标由模型侧声明（`cases/kc_quasi_static/contract.py:177-222` 的 `_driven_coordinates`），case 给网格值。
- `vehicle × ride_four_post`／`ride_random_road`／`handling`：case 侧只产出 `case_document`（如 `cases/ride_four_post.py:39`），模型文档复用 `cases/vehicle_dynamic.py` 的 `model_document`。
- 这就是"试验台职责"尚未成型的证据：模型侧与 case 侧的边界因 family 而异。

## 约束

- **现有 7 个组合的行为必须不变**：默认路径的文档字节与结果不应改变；若某组合因重构而必须变，须单独裁决并登记基线重录。
- **总成与试验台的输出归属要分清**：总成输出是「这个总成本身有什么」，试验台输出是「这个试验台额外测量什么」；合并规则与冲突检测在 07 已定义，本步接入。
- 组合查询必须是**声明式**的（可列出所有合法组合），不是靠读 compiler 代码推断。
- 无效组合的报错必须**点名具体坐标**与提供方，不得只说"不支持"。
- 不引入新依赖。
- 内核 family 名与 Python 侧 `SimulationRequest.family` 的兼容性保持（改名或加别名，不静默改变既有调用）。

## 范围与文件归属

- 可写：
  - 新增 `packages/suspension_multibody/src/suspension_multibody/rigs/**`（试验台定义与注册表）
  - `packages/suspension_multibody/src/suspension_multibody/simulation/**`（dispatch/compiler/preparation 改为按组合解析）
  - `packages/suspension_multibody/src/suspension_multibody/cases/**`（case 侧按试验台归并）
  - `packages/suspension_multibody/src/suspension_multibody/subsystems/**`（总成注册其输出与可驱动坐标）
  - `packages/suspension_multibody/src/suspension_multibody/api.py`、`vehicle/service.py`（公开入口走组合路由）
  - 新增测试 `packages/suspension_multibody/tests/rigs/**`
  - `packages/suspension_multibody/tests/data/vehicle_dynamics_baseline/**`（若确受影响，须登记）
- 只读：`joints/`、`templates/`、`outputs/`（07）、`report/`、C++ 内核。
- 不写：`preparation/assembly/**` 的几何生成（04 的范围）；父级计划文件（归主代理）。

## 依赖

- 前置：09（准静态/动态 study 合并——试验台要建立在统一的 study 概念上）。
- 后续：11（终局验收的端到端第 (d) 项要求两个组合可跑、其中一个此前不存在）。

## 验收标准

1. 组合矩阵可查询：能列出所有合法的（总成 × 试验台）组合及其提供方。
2. 现有 7 个组合全部继续可用，且**行为不变**（默认路径文档字节与结果不改变；若有变化须逐项登记）。
3. 无效组合在**准备阶段**报错，错误信息点名具体坐标与提供方（有负例测试）。
4. 至少一个此前不存在的组合跑通（`axle × ride_four_post` 或 `axle × ride_random_road`），且不需要为新组合复制驱动坐标/时间网格/结果排布的代码。
5. 总成输出与试验台输出在组合中正确合并（复用 07 的合并与冲突检测）。
6. 门禁：`case_parity_check.py` 8 family 通过；`--strict --final` 保持绿；ruff/ty 通过；`dynamic_hash_sentinel` 视影响保持绿或登记重录。

## 验证协议

1. 试验台定义与注册表落地后：跑组合矩阵查询测试。
2. 路由改造后：逐组合跑现有 7 个 family 的既有测试，确认行为不变。
3. 无效组合检测落地后：跑负例（试验台要求总成不提供的坐标）。
4. 新增组合落地后：跑通该组合的端到端（含结果可比）。
5. 收尾：三套 pytest、ruff、ty、`--strict --final`、三个数值门、`git diff --check`。

**若现有 7 个组合中任何一个的行为发生变化**，先判断是"重构引入的回归"还是"物理改变"：前者必须修掉，后者必须单独裁决并登记——不得以"重构"为名顺手重录 vehicle 系列基线。
