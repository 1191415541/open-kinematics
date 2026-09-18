# Epic: 契约边界重构 + 内核模块化 + C++ 全面接管求解

## Goal

按 `ARCHITECTURE.md`（已定稿）交付三件事：

1. **边界重画**：Python 与 C++ 之间只交换**版本化契约文档**（JSON Schema 单一来源 + 二进制块容器），
   唯一入口 ABI，契约版本与内部模块解耦。
2. **内核模块化**：C++ 按 `ARCHITECTURE.md` §3.1 的七层重组（`mb_vehicle` 作为模块消失、
   `mb_base` 拆成 numeric/dual/config、求解层拆成 static/dynamic），模块依赖为 DAG 且由门禁强制。
3. **求解接管**：静态/准静态/动态全部在 C++；Python 只保留作者体验、IO、报告与 Adams 互操作。

「接管」的操作性定义：运行期不存在调用 Python 求解代码的分支；旧路径 fail-closed；
物理量、指标与工况展开全部由 C++ 计算。

## Non-Goals

- 不把 Adams 逻辑迁入 C++（保留对标独立性）。
- 不改变任何数值行为直到 P5：P2–P4 是纯结构/边界重构，门禁是「动态输出哈希逐位不变」。
- 不在 P3 顺带加新能力；模块化与新能力必须分成不同阶段，否则哈希门无法定位回归。
- 不重写 `schema/` 的作者侧契约（P2 是与 JSON Schema 对齐，不是重写）。

## Constraints

- **契约是唯一来源**：模型/工况/结果三份文档的字段只能来自 JSON Schema；两侧不得私自加字段。
- **浮点往返无损**：JSON 数字 `%.17g` 往返；哈希 = 规范化字节 SHA-256；这是所有门禁的前提。
- **ABI 稳定**：唯一入口 + 契约版本；内部模块拆分不得触发 ABI 变更。
- **动态路径逐位不变**（P2–P4）：`docs/axle_dynamics_results.md:211-216` 记录过迭代路径改变会改哈希。
- **CMake 不 glob**（`CMakeLists.txt:7-10`）：新增/拆分 `.cpp` 与新增静态库必须显式登记
  （源列表、模块属性循环 :185-197、聚合 group :212-230、Release IPO :282-283）。
- **环境**：MSYS2 UCRT64（g++ 16.2.0 / cmake 4.4.3 / ninja 1.13.2，与 CI 的 windows job 同源）；
  `just` 经 `uvx --from rust-just just.exe` 取得。原文「本机无 C++ 工具链、无 just」已由 child 1 推翻。

## Decisions

- D1 边界载体 = 文档边界（B1）；容器 = header + JSON + blob（见 ARCHITECTURE §1.2）。
- D2 契约来源 = JSON Schema（与既有 Geometry Contract V1 同套路）。
- D3 Adams 留 Python；C++ 不出现 Adams；Python 不出现残差/本构/Jacobian。
- D4 扩展点 = 编译期注册表（joint/element/tire/case 四张表）+ 接口头。
- D5 模块划分以 ARCHITECTURE §3 为准，`MODULARITY_AUDIT.md` 只描述现状与证据。
- D6 Adams 侧瘦身：`adams/` 从「重新实现工况」改为「渲染同一份契约」。
- D7 ty 既有 74 条诊断：清零或逐条记录理由的豁免清单，条目数不得增加。
- D8 oracle 入 `tests/data/`；`raw/` 证据默认本地。
- D9（2026-09-18，用户裁决）整车数值相关门禁退役：删除 14/15-DOF 独立整车模型、Python 轮胎力律与
  以它们为默认模拟器的 handling/ride 相关门禁，换取「Python 无本构」的字面成立；
  `Vehicle14DofParameters` 作为 Adams 输入 manifest 的数据形状迁到 `adams/vehicle_parameters.py`。
  代价（handling/ride 不再有独立数值对标）记录在 `PROGRESS.md`。

## Validation Protocol

工具由 child 2 先行产出（无需编译器）：

'- **两道独立门（复审后修订，禁止混用）**：
  - **门 A 契约身份门**：契约文档（JSON+blob）规范化字节 SHA-256 稳定；必须配浮点往返**位比较**性质测试。
    注意现有 `canonical_hash`（`io/results.py:21-26`）没有 `%.17g` 规则，契约定稿时须替换。
  - **门 B 数值 parity 门**：严格度按阶段 —— P3 纯结构重构在固定环境（单线程/固定后端/固定编译器）
    下要求结果数组字节级一致；P4 先证「契约解析出的 double 与 Python 原值位相同」再要求字节级一致；
    P5/P6 为新求解路径与新工况族，改为**容差 parity**（继承 strict K/C 容差）+ 事件/诊断一致性。
- `dynamic_hash_sentinel.py`：`--record`/`--check`；对象 = `arrays.npz` SHA-256 + 规范化 manifest SHA-256。
  P3–P4 每步必跑（门 B 的字节级档位）。
- `case_parity_check.py`：**按工况族**逐族 parity（K/C、整轴动态、整车 K-C、整车动态、操稳、
  平顺-四柱、随机路面、对标）；各族独立判定，缺族即失败。用于 child 9 的验收，避免「验收声明大于实际验证」。'
- `kc_parity_check.py`：native vs 冻结快照；身份门（model/case/工况哈希）先行；K 逐 9 状态 × 90 字段、
  C 逐 load path × level × 左右轮 6D + toe/camber；容差继承 `adams/strict_k.py:388-403`、
  `adams/strict_c.py:407,422-424`。
- `kc_perf_gate.py`：K 100 / C 6600；`--record`（基线）/`--check-native`。
- `kc_legacy_path_check.py`：断言 native 路径不实例化 `EquilibriumSolver`，旧路径 fail-closed。
- `check_module_layering.py`：模块依赖 **DAG** 门禁 + 冻结边集（先按当前边集落基线，只允许按计划收敛）；
  同时禁止 `functions.hpp` 互相包含与自包含。
- 契约往返测试：随机浮点 → 序列化 → 解析 → **位比较**（保证哈希门有效）。
- `contract_roundtrip.py`：模型/工况/结果三类文档的往返与哈希稳定性。

CI：本机不可执行部分以 `.github/workflows/ci.yml` 结论为准；无结论记为 BLOCKED。

## Risk Assessment

- R1 浮点往返（ARCHITECTURE R1）→ child 4 的性质测试必须先通过。
- R2 解析开销 → child 8/9 实测 K 100 / C 6600 性能门。
- R3 一次性前置成本（契约 + 入口 ABI + 内核重构）→ P2–P4 无用户可见收益，收益从 P5 起。
- R4 拆头文件会暴露缺失的直接 include → 逐 TU 补齐，不得重新加聚合 include。
- R5 LTO 下模块拆分可能改变产物 → 哈希门裁决，漂移即回退该拆分。
- R6 Python 侧删除范围大（core/elements/solver/dynamics/model/analysis 求解职责、
  `vehicle_dynamics.py`、`pac2002_scope.py`）→ child 10 需逐文件保留/删除矩阵。
- R7 Adams 侧现有门禁不得因瘦身而降级 → child 11 以门禁仍通过为验收。
- R8 ty 74 条既有诊断 → D7。
- R9 文档与现状矛盾（内核「不含元素语义」、K6「已删除聚合头」）→ child 12 修订。
- **R10（2026-09-17 实测修正）Phase B 的 K/C 原生化主要是工况层工作，不是新求解物理**：
  原生内核已有精确保持的驱动坐标（`tests/axle_dynamics/test_driven_coordinate.py`，12 项通过）
  与保持到 `1e-9` 的静平衡（`test_api.py::test_native_solver_preserves_static_equilibrium`，含衬套与轮胎）；
  C 模式的轮心六分量扳手可由既有 `body_wrench`（作用于刚体原点）+ 力矩参考点换算表达，很可能无需改 ABI。
  剩余不确定性集中在「前轴装配 → `AxleDynamicsModel` 转换器」的覆盖面。


## Child Deliverables（12）

**P0 基础设施**
1. `00-toolchain` — MSYS2 UCRT64；内核可编译、kernel 测试通过。
2. `01-harness` — 五个脚本 + 契约往返测试 + 分层 DAG 门禁（按当前边集落基线）。

**P1 oracle**
3. `02-freeze-oracle` — 冻结动态哈希、K/C 快照、性能基线。

**P2 契约**
'4. `03-contract-schema` — JSON Schema（model/case/result）+ 容器格式与**统一数组描述符** +
   规范化与哈希规则（门 A）+ 浮点往返性质测试；Python 侧 Pydantic 与 schema 对齐。
   **case 文档必须含统一工况族判别字段**（`kc_quasi_static | axle_dynamic | vehicle_kc | vehicle_dynamic |
   handling | ride_four_post | ride_random_road | comparison`）与各族的完整输入字段、
   以及 Adams 渲染所需的元数据（hardpoint/bushing/spring/damper/road/tire 参数引用、比较通道与容差）。
   现有 `schema/case.py` 只有 K/C（`CaseSpec.mode: Literal["K","C"]`）、`schema/dynamic.py` 只有三种动态模式，
   **不足以支撑 child 11**，必须在此补齐。'
'5. `04-mb-contract` — C++ `mb_contract`：解析/校验/规范化/哈希 + 四张注册表骨架 + 单元测试。
   解析策略：引入受控 header-only JSON 库**只解析小 JSON header**，blob 走手写边界读取；
   禁止 JSON 承载大数组；强制 payload 长度/嵌套深度/描述符合法性上限。
   依赖方向单向 `mb_contract → mb_model`。'

**P3 内核重构（行为不变）**
6. `05-kernel-restructure` — 按 ARCHITECTURE §3.1 重组七层：`mb_base`→numeric/dual/config、
   `mb_constraint`→joint、力元→element、**消解 `mb_vehicle` 模块**、求解层拆 static/dynamic、
   破聚合头与自包含、注册表化；每子步过哈希门。

**P4 边界切换**
7. `06-boundary-switch` — 唯一入口 ABI（`suspension_kernel_run`）+ Python `kernel/` 薄壳；
   旧 flat ABI 双跑对照；哈希不变。

**P5 求解接管**
8. `07-solve-static-kc` — `mb_solve_static` 增加准静态能力 + `mb_cases` 的 `kc_quasi_static` 族；
   K/C parity 门通过。

**P6 其余工况族**
'9. `08-cases-rest` — `axle_dynamic` / `vehicle_kc` / `vehicle_dynamic` / `handling` /
   `ride_four_post` / `ride_random_road` / `comparison` 各族；**每族都必须过 `case_parity_check.py`**
   与性能门，缺族即失败。'

**P7 清理**
'10. `10-adams-renderer` — `adams/` 改为同一契约的渲染器并瘦身；对标门禁保持通过。
    **必须先于 Python 瘦身**：`adams/full_vehicle_model.py:3165-3167` 依赖 `model.vehicle.build_vehicle`，
    `adams/strict_k.py:15-16`、`strict_c.py:27-29` 依赖 `analysis`/`model`；先删共享装配层会让 Adams 门禁无法运行。
11. `09-python-slim` — 删除 Python 求解层与装配层（core/elements/solver/dynamics/model/analysis 求解职责、
    `vehicle_dynamics.py`）；**逐文件保留/删除矩阵**，且 `pac2002_scope.py` 单独排期
    （`schema/dynamic.py:10` 直接导入它，scripts/tests 多处依赖，必须先迁到契约 capability 再删）。
    实际处置：求解层（`solver/`、`analysis/{k_mode,k_reference,c_mode,axle_quasi_static,sweeps,roll_center}.py`、
    `dynamics/`）与 `pac2002_scope.py` 的能力表已删/已迁；`vehicle_dynamics.py` **保留** ——
    它已不含 Python 求解，是整车侧走契约的作者层（`cases/*.py` 依赖 `prepare_vehicle_run`），
    删除会直接打掉 child 9 交付的五个整车工况族。'

**P8 验收**
12. `11-verification` — 性能/parity/门禁/wheel/CI/文档全量验收。

## Dependency Notes

- 1 与 2 可并行；3 依赖 1;2；4 依赖 3；5 依赖 4；6 依赖 5；7 依赖 6；
- 8 依赖 7；9 依赖 8；10（Adams 渲染器）依赖 9；11（Python 瘦身）依赖 10；12 依赖 11。
- 依赖关系以 `SUBTASKS.csv` 为唯一事实来源。

## Done-When

- [x] 契约：JSON Schema 是模型/工况/结果的唯一定义；浮点往返性质测试通过
- [x] 边界：唯一入口 ABI 生效，Python 侧 <200 行薄壳，旧 flat ABI 已淘汰
- [x] 模块：模块依赖为 DAG，边集只减不增；`functions.hpp` 不再互相包含/自包含
- [x] 模块：扩展点四张注册表就位；新增能力不改求解器、不改 ABI
- [x] 求解：静态/准静态/动态全部在 C++；Python 无残差/本构计算
      —— 用户裁决（2026-09-18）：删除整车数值相关门禁及其 14/15-DOF 独立模型与 Python 轮胎力律
      （`analysis/{tire_models,vehicle_correlation_model}.py`、`adams/vehicle_correlation.py` 及对应测试）；
      `Vehicle14DofParameters` 作为 Adams 输入 manifest 的数据形状迁到 `adams/vehicle_parameters.py`。
      Python 侧因此不再有任何本构力律
- [x] 门禁：门 A（契约身份）稳定；门 B 在 P3/P4 字节级、P5/P6 容差级；各族 parity 与性能门全通过
- [x] Adams：仍为 Python 侧独立实现，且从「重新实现工况」变为「渲染契约」
- [x] `ruff` 通过；`ty` 清零或按 D7 精确豁免
- [x] wheel 内容与符号可加载断言通过；CI Windows job 通过
      —— CI 各步骤的命令在本机逐条跑过并给出结论；CI 机器本身未执行（无 runner）
- [x] 文档与实现一致（含 R9 两处表述修正）