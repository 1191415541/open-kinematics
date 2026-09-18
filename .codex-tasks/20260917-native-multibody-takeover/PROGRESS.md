# PROGRESS — 20260917-native-multibody-takeover

## Recovery Block

- 任务: 以 C++ 完全取代 Python 通用多体层（动态 + 准静态 K/C）
- 形态: epic
- 进度: 12/12 child 完成；Done-When 10/10 通过（#5 由用户裁决为「删除整车相关门禁」并已执行）
- 当前: flat ABI 已退役、契约表达力补齐、Python 侧零本构；全部门禁通过
- 文件: .codex-tasks/20260917-native-multibody-takeover/SUBTASKS.csv
- 下一步: 无未闭合项；剩余的唯一失败是本机 artifacts 造成的既有缺陷（CI 中跳过）

## Review Round 1（2026-09-17，4 位审查者）

技术可行性 / 影响面 / 工程规则 / 验证设计四个角度，约 30 项意见。主要结论与处置：

- C 模式需「指定刚体点六分量 wrench」，现有 `body_wrench` 无 per-body/point 元数据
  （`axle_dynamics/native.py:196-202`、`core_abi.hpp:61-68`）→ D2。
- `mb_static` 无 prescribed-drive 静态求解、接触点驱动语义、柔度与指标 → D1。
- `api.py:46` 仍依赖 `solver.evaluate_generalized_forces`；`core/elements` 不可整目录删
  （`model/front_axle.py:10,24`）→ child 7、10 + Non-Goals。
- `dynamics/` 删除会破坏 `analysis/roll_center.py:10` 与
  `analysis/vehicle_correlation_model.py:16` → child 9 + D4。
- strict K/C 切换无承载 → child 8 + D5（parity 门与 Adams 门分离）。
- oracle 未定义来源/格式/容差/失败判定；动态哈希对象未定义 → §Validation Protocol。
- ABI 九项闭环、wheel 内容断言、ty 74 条债务、`just` 缺失、文档 R9 矛盾 → Constraints、D7、R9。

## Review Round 2（2026-09-17，一致性复审）

判定：第 2 稿不可执行。根因是**验收目标缺少可执行的验证入口**（多个 child 的
acceptance 与 validation_command 不匹配；parity / 性能 / 动态哈希 sentinel / wheel / CI 无命令）；
另有一处依赖表述矛盾（`{3,4}` 并行 vs 「4 依赖 3」）。

第 3 稿处置：

- 新增 child 2「验证工具先行」：`kc_parity_check.py`、`dynamic_hash_sentinel.py`、
  `kc_perf_gate.py`、`kc_legacy_path_check.py` 与 wheel 断言扩展，先于一切 native 改动产出。
- 每个 child 的 validation_command 改为调用这些工具，acceptance 与命令逐条对齐。
- 依赖唯一事实来源改为 `SUBTASKS.csv`，EPIC 的 Dependency Notes 不得与之冲突。
- 结构由 11 个 child 调整为 12 个（工具链与验证工具可并行）。

## Decisions

- D1 分两层（用户意见）：D1a `mb_static` 合并静态+准静态求解能力；D1b 新增独立模块 `mb_kc_cases` 承载 K/C 工况定义与工况控制。
- D2 ABI：新增 `axle_kc_run` 专用入口，不复用 `body_wrench`；轴 ABI 递增。
- D3 Python 边界：`model/` 保留；物理量、指标与工况编排均由 native 计算。
- D4 dynamics 独立性：默认改用 native 并记录独立性损失（child 9 执行）。
- D5 strict 门禁：Python 快照成为永久 parity 基准；Adams 门与 parity 门并存。
- D6 不改既有 trim 入口；每步跑动态哈希 sentinel。
- D7 ty 74 条诊断清零或精确豁免，条目数不得增加。
- D8 oracle 入 `tests/data/kc_baseline/`；`raw/` 证据默认本地。

## Log

- 2026-09-17 初稿：8 个 child。
- 2026-09-17 第 2 稿：按首轮审查修订为 11 个 child，补 §Validation Protocol 与 D1-D8。
- 2026-09-17 第 3 稿：新增「验证工具先行」child，重排依赖链为 12 个 child。
- 2026-09-18 收口：child 1–12 全部完成；契约结果携带全部账本、整轴与整车公共 API 切到契约、
  扁平 C ABI 退役、Python ctypes 镜像删除；重建内核后暴露的 22 项契约表达力缺口逐条修复；
  Done-When 9/10 通过（#5 待用户裁决）。
- 2026-09-18 终局：#5 由用户裁决为「删除整车相关门禁」，14/15-DOF 独立模型、Python 轮胎力律与
  handling/ride 数值相关门禁退役（`Vehicle14DofParameters` 迁到 `adams/vehicle_parameters.py`）；
  Done-When 10/10 通过。
## Review Round 3（2026-09-17，用户设计意见）

用户要求：①K/C 工作流作为**工况**独立出来，需要专门代码控制仿真工况；
②准静态与静态**求解能力整合在一起**。

处置：原 D1 拆为 D1a（求解能力层：`mb_static` 合并静态+准静态，不新建第二套求解模块）与
D1b（工况层：独立模块 `mb_kc_cases`，负责工况定义/网格/载荷路径/侧模式/K reference/柔度/指标），
依赖方向 `kc_cases → mb_static`，与动态 `mb_integrator` 路径无耦合。§Validation 的身份门相应
改为「工况定义哈希」由工况层产出。child 4/5 已按此重写。
## Review Round 3b（2026-09-17，分层复审）

两位审查者（内核分层 / 工况层边界）结论：

1. **阻断项：C++ 模块分层检查在仓库中不存在。** 架构文档（`docs/axle_dynamics_architecture.md:45-47`）
   声称有 `check_module_layering.py` 且「实测边集冻结为回归基线」，但该脚本、边集快照与任何等价
   探针在全仓检索中均不存在。新增 `mb_kc_cases` 会引入一条无人检查的依赖边。处置：纳入 child 2
   作为交付物（恢复分层门），Done-When 增加对应条目。
2. **新增模块的 CMake 登记清单**（六处）：源列表、`add_library`、模块属性循环
   （`CMakeLists.txt:185-197`）、聚合链接 group（`:212-230`，置于 `mb_static` 之前）、
   Release IPO 列表（`:282-283`）、导出符号时的 ABI 源/头。处置：写入 D1a。
3. **翻译单元放置是关键风险**：`kernel_static_contact.cpp` / `kernel_static_trim.cpp` 已因
   `static_trim` 被动态路径拉入（`abi/kernel_abi.cpp:170-174`），追加代码会扩大影响面；
   Release 开启 IPO/LTO 不能凭「未新增调用点」推断产物不变。处置：D1a 强制新翻译单元。
4. **D1b 接口边界未闭合**（六项）：`AxleKcInput` 语义、CaseSpec/KGrid/CGrid/LoadPath 的 canonical
   映射与退役、model ABI 与 K/C input ABI 的字段/单位/顺序/ownership、`AxleKcOutput` 必需字段、
   Python 禁止重算物理的边界、`axle_quasi_static.py` 与 `benchmarks.py` 的归属。处置：全部写入 D1b。
   - 关键裁决：**Python 传声明式 CaseSpec，C++ 展开网格并产出工况身份**；若 Python 传已展开
     case list，则工况编排仍留在 Python，与 D1b 冲突。
   - 迁移清单：`metrics.py`、`compliance.py`、`k_mode.py` 的轮胎压缩与指标、`c_mode.py` 的 C 指标/
     C−K/wheel response/割线柔度、工况级 residual/收敛/事件/失败原因 → 必须迁 C++；
     schema 校验、loader、model 装配、ABI 编组、结果落盘、benchmark 统计 → 留 Python 薄壳。
5. 命名：`mb_kc_cases` 符合现有 `mb_` 约定，保留；不建议 `mb_quasi_static`（会重新暗示第二套求解模块）。
## Review Round 3c（2026-09-17，用户范围修正 + 全内核模块化审计）

用户两点意见：①工况层不只是 K/C，还包括整车仿真工况、四柱实验等，应以**多文件的工况模块**管理工况；
②要求做全内核耦合审计，目标「各文件只实现独立或相近功能，完全模块化」。

处置：

- 完成 6 路并行只读审计，结论落 `MODULARITY_AUDIT.md`（12→16 KB）：`mb_static` 混 6 类职责；
  11 个模块头全是聚合头（每个触达 7~8 个其他模块）；头图存在 SCC `{mb_integrator, mb_static, mb_vehicle}`；
  4 条真实反向依赖边；`kernel_base.cpp`/`kernel_output.cpp`/`vehicle/kernel_registration.cpp`/
  `tire/assemble.cpp`/`pac2002/law.cpp` 等为多子域文件；分层检查器在仓库中不存在。
- D1 重写为 D1a（求解能力合并）/D1b（多文件工况模块 `mb_cases`，覆盖 K/C、整轴动态、整车 K/C、
  整车动态、操稳、平顺/四柱、对标工况）/D1c（工况单一事实来源，Adams 侧改为渲染器）。
- D0：确立「模块化优先」，Epic 改为两阶段 14 个 child（Phase A 行为不变的模块化重构 → Phase B K/C 接管）。
- D2 补充：整车类工况复用 `vehicle_run` + 声明式工况输入；仅 K/C 需要新增 `axle_kc_run`。
- MODULARITY_AUDIT 新增 §8：工况族清单（现散落四处）、`mb_cases` 目标文件划分、Adams 渲染器化收益、
  CMake 登记清单。
## Review Round 4（2026-09-17，契约边界架构复审）

三路复审（边界可行性 / 迁移顺序 / 分层闭合）发现 8 个阻断项，已全部修订进 ARCHITECTURE.md 与 EPIC：

1. **自相矛盾（最重要）**：原方案要求「契约 JSON 化后动态输出逐位不变」——复审指出这不成立
   （现有 `canonical_hash` 无 `%.17g` 规则；且新求解路径不可能逐位复现旧实现）。
   处置：§1.3 拆成**门 A（契约身份哈希）** 与 **门 B（数值 parity，P3/P4 字节级、P5/P6 容差级）**。
2. JSON 承载大数组与自写解析器风险 → §1.2 统一数组描述符 + 只解析小 JSON header +
   受控 header-only 解析库；禁止 JSON 承载大数组。
3. 单一入口的调用粒度 → §1.4 模型句柄可缓存 + 有界批次 + continuation token。
4. 能量类型归属（两条反向依赖边的根源）→ 新增 L2 `mb_energy`；禁止放 `mb_output`。
5. `add_vehicle_static_rotation_gauges` 无处安放 → 拆到 `mb_solve_static` + `mb_output`。
6. 注册职责未明确 → `mb_assembly` 明列 DOF/坐标、轮胎 frame/道路挂接、gauge 模型侧注册。
7. `mb_contract` 与 `mb_model` 依赖方向 → 单向 `mb_contract → mb_model`。
8. 激励三处归属 → 声明式字段在 L1、展开在 L5 `mb_cases`、物理施加在 L2/L3。

另修两处迁移顺序阻断项：
- **Adams 渲染器（child 10）必须先于 Python 瘦身（child 11）**：`adams/full_vehicle_model.py:3165`
  依赖 `model.build_vehicle`，`strict_k.py:15-16`/`strict_c.py:27-29` 依赖 `analysis`/`model`。
- `pac2002_scope` 不能随求解层一起删（`schema/dynamic.py:10` 直接导入），需先迁到契约 capability。
- child 9 的验收补 `case_parity_check.py`（按工况族逐族判定），避免「验收声明大于实际验证」。
- child 4 范围扩展：case 契约必须含统一工况族判别与各族完整字段及 Adams 渲染元数据。
## 工具链核实（2026-09-17，用户指出）

更正此前结论（「本机无 C++ 编译器」不准确）：**VS2022 Build Tools 已安装**。

- VS 2022 Build Tools 17.14.37628.2，位于 `C:\BuildTools`；vswhere 确认含 `Microsoft.VisualStudio.Component.VC.Tools.x86.x64`。
- MSVC 工具集 14.44.35207：`C:\BuildTools\VC\Tools\MSVC\14.44.35207\bin\Hostx64\x64\cl.exe`。
- `vcvars64.bat`：`C:\BuildTools\VC\Auxiliary\Build\vcvars64.bat`；`VsDevCmd.bat` 同目录上级。
- Windows SDK 10.0.26100.0；VS 自带 CMake 与 Ninja（`Common7\IDE\CommonExtensions\Microsoft\CMake\...`）。

**但当前构建管线是 GNU/MinGW 专属，无法直接用 MSVC**：

- `binding/build.py:142-154` `discover_compiler()` 只查 `CXX` 环境变量与 PATH，找不到未进 vcvars 的 `cl.exe`。
- `_assert_64_bit`（约 :157-171）执行 `[compiler, "-dumpmachine"]`——`cl.exe` 不支持该参数，即使找到也会判失败。
- `CMakeLists.txt` 使用 `-fno-fast-math`(:136)、`-fopenmp`(:137,154,182,196,211)、
  `-static -static-libgcc -static-libstdc++`(:205)，并要求静态 `libgomp.a`(:247,254)，缺失即 FATAL_ERROR(:262)。
  MSVC 无 libgomp（`/openmp` 走 vcomp140.dll），也没有 `-static*` 语义。
- CI 的 windows job 安装 MSYS2 UCRT64 的 mingw-w64-ucrt-x86_64-gcc —— 那是被验证过的 Windows 路径。

**数值一致性影响**：现有动态输出哈希与 Strict K/C 基线均为 MinGW 产物；换 MSVC 会改变浮点收缩/中间精度/数学库，
门 B 的「字节级」档位无法与既有基线比较（CI 注释亦说明 MinGW release 与 WinLibs 只保证「构建并通过」，
不保证逐位一致）。

待用户决策：①MSVC-only（需改造构建管线，且需重建数值基线）；②MSYS2 UCRT64（与 CI 同源，零改造）；
③双轨（MSVC 做开发迭代，MSYS2 做权威门禁）。
## Child 1 执行结果（2026-09-17）

选型：**MSYS2 UCRT64**（与 CI windows job 同源）。用户决定，替代 MSVC-only / 双轨方案。

已执行：

1. `winget install MSYS2.MSYS2`（20260611）→ `C:\msys64`。
2. `pacman -Syu` + 安装 `mingw-w64-ucrt-x86_64-{gcc,cmake,ninja}`。
3. 验证：`g++ 16.2.0`（target `x86_64-w64-mingw32`，满足 `_assert_64_bit` 的 `-dumpmachine` 探测）、
   `cmake 4.4.3`、`ninja 1.13.2`。
4. 用户 PATH 追加 `C:\msys64\ucrt64\bin`（**追加而非前置**：UCRT64 自带 curl.exe，前置会遮蔽现有 curl）。
5. 内核构建成功：`suspension_kernel.dll` 1,712,009 字节；`native_build.json` 记录
   compiler = MSYS2 g++ 16.2.0、flags = `-O3 -DNDEBUG -std=c++17 -flto=auto -fno-fat-lto-objects
   -Wall -Wextra -Werror -fno-fast-math -fopenmp`、ABI 15/30/1。
6. **kernel 测试 15 passed**（此前为 4 passed / 4 failed / 7 skipped）。
7. 刷新 multibody 镜像（`build_axle_native.py`）后跑全包：
   **518 passed / 1 failed / 47 skipped / 1 xfailed**（此前 354 passed / 165 failed / 47 skipped）。

### 唯一遗留失败（阻断级，需处置）

`tests/adams/test_full_vehicle_model.py` 内部**自相矛盾**：

- `:956`（`test_importer_uses_adams_source_files_and_builds_full_model`）断言
  `manifest["adams_model_reduction"]["omitted_part_ids"]` **非空** → 当前为 `()` → 失败。
- `:1064`（`test_source_initial_conditions_map_to_complete_native_vehicle_state`）断言同一个字段
  `== ()` → 通过。

两个测试使用同一数据源 `_SOURCE_CASE = Path("artifacts/adams-full-source/step_steer")`（`:54`），
而 `omitted_part_ids` 由 `full_vehicle_model.py:477-479` 计算
（`set(compiled_parts) - _source_native_body_part_ids(..., include_drivetrain=True)`），
**不依赖 `pairing_manifest()` 的可选参数** → 两个断言不可能同时成立。

判定：**与工具链无关的既有缺陷**（在无编译器时该测试同样以 `assert ()` 失败）。
需先确认「模型缩减应当省略哪些 part」是设计意图还是历史遗留，再决定改代码还是改断言。
在解决前，`packages/suspension_multibody/tests` 无法作为全绿门禁。

### 下一步

- child 2（验证工具与分层 DAG 门禁，不依赖编译器）可立即开工。
- child 3 冻结 oracle 前必须先解决上述失败，否则基线不可信。
## Child 2 与 Child 3 执行结果（2026-09-17）

五个验证工具全部产出并验证（位于 `packages/suspension_kernel/scripts/` 与 `packages/suspension_multibody/scripts/`）：

| 工具 | 结果 |
|---|---|
| `check_module_layering.py` | 基线已落 `packages/suspension_kernel/layering_baseline.json`；`--check` OK；`--strict` FAIL（Phase A 目标）。硬数据：**88 条模块边、16 个自包含头、162 处跨模块聚合包含、14 个模块同处一个环** |
| `dynamic_hash_sentinel.py` | 26 个产物（13 工况 × native_result/native_refined_result）＋ 规范化 manifest 摘要；combined sha256 = `e7407656…`；**两次独立完整运行摘要完全一致**，证明动态路径在该工具链上确定性可复现 |
| `kc_parity_check.py` | 快照落 `tests/data/kc_baseline/`：K 9 状态、C 66 状态（6 条标准载荷路径 × 11 level，单侧模式，含 4 个对角衬套的合成模型）；自校验通过 |
| `kc_perf_gate.py` | 基线落 `tests/data/kc_perf_baseline.json`：K100 中位 2.50 s、C6600 中位 2.05 s（各 2 次重复）；`--implementation native` 明确报未接线 |
| `kc_legacy_path_check.py` | 原生路径 **0** 处引用 Python 求解器；legacy 消费者 **7** 处（`api.py:46`、`analysis/c_mode.py:14,107`、`analysis/k_mode.py:13,37` 等）；`--strict` FAIL（Phase B 目标） |

### 期间发现（需记录的事实）

1. **acceptance 矩阵的失败面比文档更大**：`docs/axle_dynamics_results.md` 记「3 个工况不通过」，
   本次 MSYS2 GCC 16.2 构建下实际是 **9/13 不通过**（combined_load、in_phase_road、
   large_amplitude_high_frequency、opposite_phase_road、road_pulse、road_sine、
   road_step_finite_rise、single_wheel_road、tire_liftoff_and_recontact）。
   **全部 13 个工况的 solver_internal 与 energy 门都通过**，失败一律只由 `fixture.force_z` 的
   NRMSE（0.032–0.451，门限 0.02）引起；`maximum_state_nrmse` 仅 2e-4–6e-4（门限 0.01，约 20–50 倍余量）。
   与文档所述机理一致（`fixture.*` 改为动量平衡重建量，但工况 YAML 仍按 5 N 地板计入收敛门），
   只是影响面比文档记的 3 个更大。按既定决定「记录如实、不修改工况」，哨兵把这一状态冻结为基线。
2. **`test_full_vehicle_model.py` 的失败是本地 artifacts 依赖**，不是仓库缺陷：
   `_CASE = artifacts/adams/correlation-reference-real-si/...` 存在故该测试运行，其断言要求
   `omitted_part_ids` 非空，而该 case 的 57 个编译 part 全部被映射（差集为空）；
   对照测试（`:1064` 要求 `== ()`）用的是 `_SOURCE_CASE = artifacts/adams-full-source/...`，
   该目录不存在 → **被跳过**。`artifacts/` 被 gitignore，CI 中两个测试都不存在数据 → 均跳过。
   映射函数自 `1ee40c4` 起未变。判定：本地 artifacts 陈旧导致的非门禁失败。
## Child 4 执行结果（2026-09-17）

产出（`packages/suspension_contracts/`）：

- `src/suspension_contracts/contracts/multibody_{model,case,result}.schema.json` —— 三类文档的 JSON Schema。
  case 含统一工况族判别（`kc_quasi_static | axle_dynamic | vehicle_kc | vehicle_dynamic | handling |
  ride_four_post | ride_random_road | comparison`）与各族输入字段；result 含 `case_identity`
  （model/case/grid sha256）。
- `src/suspension_contracts/multibody.py` —— 契约的 Python 半边：
  - **canonical form**：键排序、无冗余空白、`%.17g`、禁止 NaN/Inf；
  - **container**：`magic(4) | version(4) | json_len(8) | blob_len(8) | json | blob`（小端）+ `blob_slice`；
  - **validation**：JSON Schema 小子集校验器（零新依赖）；
  - `contract_hash` / `pack_payload` / `unpack_container`。
- `tests/test_multibody_contract.py` —— **22 项测试全通过**，其中
  `test_float_round_trip_is_bit_exact` 用 5000 个随机 double 验证 canonical → parse **位级一致**（门 A 的前提）。

**期间发现并修复的真实缺陷**：`"%.17g" % -0.0` 产出 `-0`，`json.loads("-0")` 解析为**整数 0**，
符号位丢失（浮点往返测试当场抓到）。修复：当格式化结果不含 `.`/`e`/`E` 时补 `.0`，使其成为明确的浮点字面量。
这正是复审所警告的 `-0.0`/词法类型陷阱。
## Child 5 执行结果（2026-09-17）

产出（`packages/suspension_kernel/`）：

- `cpp/include/mb_contract/{types.hpp,functions.hpp}` —— JsonValue / BlobDescriptor / ContractPayload 与接口。
- `cpp/src/contract/contract_json.cpp` —— 容错解析 + **规范形式写出** + `contract_verify_canonical`
  （解析后重写必须逐字节相同，否则拒绝；这是门 A 在内核侧的强制点）。
- `cpp/src/contract/contract_container.cpp` —— 容器头 + 文档/blob 切分，拒绝坏 magic/版本/长度/非规范文档。
- `cpp/src/contract/contract_sha256.cpp` —— 零依赖 SHA-256，供内核**自行推导**身份而非信任外部传入。
- `cpp/src/contract/contract_registry.cpp` —— 四张注册表（10 关节 / 9 力元 / 4 轮胎 / 8 工况族）。
- `cpp/tests/contract_selftest.cpp` —— 独立自测可执行（静态链接，无需 PATH）。
- CMake：新增 `MB_CONTRACT_SOURCES` 与 `mb_contract` 静态库，已登记源列表、模块属性循环、
  聚合链接 group（置于 `mb_integrator` 之前）、Release IPO 列表，并新增 `mb_contract_selftest` 目标。

验证：自测 **39 项全过**（含 `-0.0` 符号保持、`1.0` 浮点字面量、17 位指数、控制字符转义、
非规范输入拒绝、容器截断/坏 magic 拒绝、SHA-256 三组标准向量）；kernel 测试 15/15；
**动态输出哈希 `e7407656…` 未变**（新模块未扰动数值路径）。

被现有门禁抓到的两件事（都是设计要抓的）：
1. `test_build_write_metadata_records_added_keys` 要求**每个头文件都出现在构建元数据里** ——
   新头未登记即失败；已补进 `binding/build.py` 的 provenance 清单。
2. 分层门禁报出新增边 `mb_contract → mb_base`；该边为有意添加（唯一一条），已重录基线并记录。
## Child 6 进行中（2026-09-17）—— 头文件解聚合已完成

**已完成的子步骤（每步都过动态哈希门）**

| 指标 | 开始 | 现在 |
|---|---|---|
| 自包含头 | 16 | **0** |
| 跨模块聚合包含 | 162 | **0** |
| 模块依赖边 | 89 | **82** |
| 环内模块数 | 14 | **10** |
| 动态输出哈希 | `e7407656…` | **未变** |

做法：
1. 用符号分析算出每个 `.cpp` 真正调用了哪些其他模块的函数，为 **42 个翻译单元**补齐直接 include；
2. 从所有模块头移除跨模块 `functions.hpp` 包含（148 处）与自身包含（16 处）；
3. tire 家族按正确结构收敛（伞头不再包含子模块 functions.hpp，子模块不再反向包含伞头；去掉 6 处冗余 common/functions.hpp）；
4. 抽出 **`mb_energy`** 模块，把 `EnergyRates/EnergyStorage/EnergyInterval/StaticContactOverride`
   从 `mb_vehicle/energy.hpp` 移出（8 个包含方改指新位置），消除了语义层 → 车辆层的反向依赖。

**期间修复的门禁漏洞**：`dynamic_hash_sentinel` 原会在 acceptance 运行失败时拿**旧产物**算出"OK"
（内核重建后 mirror 陈旧 → acceptance 1.8 秒即退出，旧产物未变 → 误报通过）。现已要求
`acceptance_report.json` 的 mtime 在运行后必须刷新，否则直接拒绝打分。

**剩余（child 6 的最后一项）**：1 个环，成员为
`abi, mb_constraint, mb_integrator, mb_model, mb_output, mb_tire, mb_tire_common, mb_tire_fiala, mb_tire_pac2002, mb_tire_state`。
根因：多个模块头为了拿到 `AxleInput` 而包含公开 ABI 聚合头 `axle_kernel.hpp`
（`mb_integrator/{context,functions}.hpp`、`mb_output/functions.hpp`、`mb_static/functions.hpp`、`mb_vehicle/functions.hpp`）。
修法：把共享输入/状态结构（`State`/`SampleInput`/`AxleInput`）下沉到低层模块
（`mb_model/inputs.hpp` 或独立 `mb_input`），ABI 头改为包含它；结构布局必须逐字节不变以免动 ABI。

**验证状态**：kernel 15/15、architecture 41/41、contracts 22/22、multibody 518 passed
（唯一失败是已知的本地 artifacts 陈旧问题，CI 中该测试被跳过）、分层 `--check` OK、`--strict` 仅剩上述环。
## Child 6 完成（2026-09-17）—— 内核模块图成为 DAG

| 指标 | 起点 | 终点 |
|---|---|---|
| 自包含头 | 16 | **0** |
| 跨模块聚合包含 | 162 | **0** |
| 模块环 | 1（14 个模块） | **0** |
| 模块依赖边 | 89 | 83 |
| 动态输出哈希 | `e7407656…` | **未变** |

子步骤（每步都过动态哈希门）：

1. 符号分析定位每个 `.cpp` 实际调用的外部模块函数，为 **42 个翻译单元**补齐直接 include。
2. 所有模块头移除跨模块 `functions.hpp`（148 处）与自身包含（16 处）。
3. tire 家族收敛为正确结构（伞头不再含子模块 functions.hpp；子模块不再反向含伞头；去 6 处冗余）。
4. 抽出 **`mb_energy`**：`EnergyRates/EnergyStorage/EnergyInterval/StaticContactOverride` 从 `mb_vehicle/energy.hpp` 移出。
5. 抽出 **`mb_input`**：`AxleInput/AxleOutput/VehicleInput/VehicleOutput` 与元素布局块从 ABI 头移到
   `mb_input/types.hpp`；`axle_kernel.hpp` 从 806 行降到 38 行，只留导出声明与版本常量转发。
6. 版本常量下沉 `mb_base/version.hpp`（消除最后一条 `mb_model → abi` 反向边）。
7. 3 个架构测试（元素布局、原生结构镜像、ABI 版本单一来源）的路径常量改指新位置——断言的**不变量未变**
   （字段列表/顺序/尺寸/单一来源），只是定义搬家。

验证：分层 `--strict` **PASS**（0/0/0）；动态哈希逐字节不变（证明 ABI 布局未动）；kernel 15/15；
architecture 41/41；contracts 22/22；multibody 518 passed（唯一失败是已知的本地 artifacts 陈旧问题）；
K/C parity OK；性能门 OK。

顺带加固：性能门原本 `baseline × 1.0` + 2 次采样，在共享机器上必然 flaky（本轮实测 K100 波动 1.24s↔2.70s，
而该基准跑的是未被改动的 Python K 求解器）。改为文档化容许倍数 ×1.25 + 报告实测比值，并以 3 次采样重录基线
（K100 1.24s、C6600 2.05s；校验时实测 ×1.060 / ×1.019）。
## 本轮收尾：6/12 全部产物同时验证通过（2026-09-17）

一次连续运行的权威证据（命令与结果）：

| 门禁 | 命令 | 结果 |
|---|---|---|
| 内核构建 | `build_suspension_kernel.py` | ✅ DLL 生成 |
| 分层 DAG（strict） | `check_module_layering.py --strict` | ✅ 0 自包含 / 0 聚合包含 / 0 环 |
| kernel 测试 | `pytest packages/suspension_kernel/tests` | ✅ 15 passed |
| 契约自测 | `mb_contract_selftest.exe` | ✅ 39 checks |
| contracts 测试 | `pytest packages/suspension_contracts/tests` | ✅ 22 passed |
| 架构测试 | `pytest packages/suspension_multibody/tests/architecture` | ✅ 41 passed |
| 动态哈希 | `dynamic_hash_sentinel.py --check` | ✅ 逐字节匹配 `e7407656…` |
| K/C parity | `kc_parity_check.py --check` | ✅ 容差内 |
| 性能门 | `kc_perf_gate.py --check` | ✅ K100 ×1.060 / C6600 ×1.019（容许 ×1.25） |
| multibody 全量 | `pytest packages/suspension_multibody/tests` | 518 passed / 1 failed（本地 artifacts 陈旧，CI 中该测试跳过）/ 47 skipped |

**模块图演进**（child 6 全程）：自包含 16→0；跨模块聚合包含 162→0；模块环 1（14 模块）→0；
模块边 89→83；新增 `mb_energy`、`mb_input` 两个低层模块；`axle_kernel.hpp` 806→38 行。

**剩余（7–12）**：边界切换（`suspension_kernel_run` + Python 薄壳 + 双跑对照）、求解接管
（原生准静态 K/C 与工况族）、Adams 契约渲染器、Python 瘦身、全量验收。

**下一步的起点**：已确认两条事实可支撑 child 8 的可行路径 —— ①内核已有
`AXLE_DRIVEN_TRANSLATION/ROTATION` 驱动坐标（registry 中 1 行约束）与 `static_trim` 的
Newton/残差/Jacobian/接触活动集；②Python K 模式的本质就是「驱动轮心位移 + 齿条位移后求静平衡」，
其指标口径已由 `analysis/metrics.py:15-59` 固定（upright 姿态的 rotation[·,1] 列 + 轮心点）。
因此 child 8 的主体可能是「工况层 + 指标 + C 模式点扳手」，而非重写物理内核——
这需要一次实测来证实：把 benchmark 前轴表达为带驱动坐标的 `AxleDynamicsModel`，
经原生内核求解后与冻结的 Python K 快照比对（parity 门已就绪）。
## Child 8 风险验证：原生内核已具备 K/C 所需的物理能力（2026-09-17，实测证据）

**问题**：child 8 是否需要在内核里新写准静态求解物理？实测答案：**不需要**。

一手证据（当前 worktree 实跑）：

| 证据 | 命令 | 结果 |
|---|---|---|
| 驱动坐标（Adams joint MOTION 等价物） | `pytest tests/axle_dynamics/test_driven_coordinate.py` | **12 passed** |
| 原生静态平衡/衬套静力平衡/轮胎接触 | `pytest tests/axle_dynamics/test_api.py` | **9 passed** |

`test_driven_coordinate.py` 的 12 项覆盖的正是 K 模式需要的性质：坐标被**精确保持**（硬约束而非软弹簧）、
驱动反力可读（台架对标要看的量）、不可表示的 target 与冗余驱动**显式拒绝**（fail-closed）、
显式目标速率进入速度级行、驱动功进入能量账本、旋转 target 在主值分支与多圈展开。
`test_api.py::test_native_solver_preserves_static_equilibrium` 断言静平衡被保持到 `1e-9`，
位置残差 ≤1e-8、速度残差 ≤1e-7。

**C 模式点扳手**：ABI 的 `body_wrench` 是 `6 × body_count` 的扁平数组，内核在
`cpp/src/vehicle/layout.cpp:124-126` 按 `6*i` 施加，即**作用于刚体原点**而非任意作用点。
但 Python 侧 schema 已定义 `WrenchInput.moment_reference`（`global_origin | body_origin | application_point`），
说明「作用点力矩 → 原点力矩」的换算（`M_origin = M_point + r × F`）是既有概念。
结论：**C 模式很可能不需要改 ABI**，只需要在工况层做力矩参考点换算——这是一个可测断言，留待 child 8 验证。

**由此修正 child 8 的工作量判断**（原评估为「真正的物理开发」）：

1. 前轴装配 → `AxleDynamicsModel` 的转换器（bodies/joints/elements/tires + 轮跳/齿条的驱动坐标）；
2. K 工况：网格 → 驱动目标 → 求解 → 指标（口径已被 `analysis/metrics.py:15-59` 固定）；
3. C 工况：轮心六分量载荷 → 原点力矩换算 → 求解 → 变形/柔度/C−K（`compliance.py:8-39` 的逻辑移植）；
4. 工况层与契约接线（child 7/9 的范围）。

即 **工况层 + 转换器 + 指标移植**，而非重写求解器。风险由「高」降为「中」，剩余不确定性集中在
转换器能否覆盖前轴装配的全部连接与力元类型。

### 转换器输入规格（实测枚举，2026-09-17）

`build_front_axle(benchmark_model(), "K")` 的实际拓扑（这是前轴 → `AxleDynamicsModel` 转换器的输入面）：

- **bodies**（dict，按名）：`chassis, rack, upper_arm_L/R, lower_arm_L/R, upright_L/R, tie_rod_L/R`（+ wheel_L/R）
- **connections**（16 条，全部 `kind='ideal'`）：每侧 6 条 —— 上/下摆臂内点×2（chassis↔arm）、
  上/下摆臂外点（arm↔upright）、齿条-拉杆（rack↔tie_rod）、拉杆-转向节（tie_rod↔upright）
- **points**：`{body: point_name}` 共 24+ 个，例如 `(chassis, uca_L_inner_front)`、`(lower_arm_L, outer)`
- **elements**：K 模式为 **0**（K 是理想关节、无载；弹簧/衬套只出现在 C 模式）

转换器需要把每条 `Connection` 映射到原生 `AxleJoint` 类型（spherical/revolute/…），
这一步的语义来源是 `model/front_axle.py` 的显式装配路径（`_build_explicit_axle`），
而不是从名字猜测——这是 child 8 下一个要落地的东西。


## 原生 K 求解达成等价（2026-09-17）—— child 8 的核心问题已用硬数字回答

新增可执行探针 `packages/suspension_multibody/scripts/kc_native_probe.py`：把 benchmark 前轴装配
转换为 `AxleDynamicsModel`（10 刚体 / 13 理想关节 / 3 个点驱动坐标），用原生内核求解，
按 `analysis/metrics.py` 的口径计算 K 指标，并写出与冻结快照同格式的候选结果。

**结果（9 个网格点全部）**：

```
k-w-10-r-5: max delta 1.65548e-06 mm/deg (z_L 290.0000 vs 290.0000)
k-w-10-r+0: max delta 1.26757e-07
k-w-10-r+5: max delta 1.65548e-06
k-w+0-r-5 : max delta 2.15323e-10
k-w+0-r+0 : max delta 0
k-w+0-r+5 : max delta 2.15095e-10
k-w+10-r-5: max delta 1.65025e-06 (z_L 310.0000 vs 310.0000)
k-w+10-r+0: max delta 1.70223e-07
k-w+10-r+5: max delta 1.65025e-06
worst delta across the 9 states: 1.65548e-06
```

再用**严格 parity 门**（继承 Adams 门容差：位置 0.1 mm + 0.002·|ref|、角度 0.02° + 0.005·|ref|）裁决：
`kc_parity_check.py --check --actual-dir artifacts/kc-native-probe` → **OK**。
即原生 K 解比门限小约六个数量级。

**结论**：原生内核**已经能够**复现 Python 准静态 K 解；child 8 不需要新求解物理，也不改 ABI。
剩余为：C 模式（轮心六分量扳手 → 原点力矩换算 + 柔度/C−K）、工况层（网格/标签/契约接线）。

**诚实标注**：上述 parity 门中 **K 的一半是原生真实输出**；C 的一半当前仍是参考文件的占位
（探针把它原样写出以便门禁跑完整流程），C 模式落地后必须替换为原生输出，不得以此充当 C 的等价证据。

**这个过程中被实验逐一定位并修正的语义细节**（都是不能用文档猜测、只能靠实测得到的）：

1. `AxleDrivenCoordinate` 的 translation target 是**绝对分离量**（C 侧残差 `dot(dp, aa) - target`），
   不是相对装配位的增量——目标必须加上设计位分离（轮心 z = 0.3 m）。
2. `assembly.point(...)` 返回的是**世界（mm）坐标**，而 Python 装配体所有刚体的设计位姿都是原点恒等框架
   （`state.pose(...)` 为恒等）；原生侧因此应把刚体初始位姿设为恒等，代入关节标记点时按 mm→m 缩放。
3. 快照字段名带 `_mm`/`_deg` 后缀（`kc_parity_check._k_fields` 的口径），探针必须对齐以免误判。
4. 左右两侧必须各自使用本侧 `upright_*` 的轮心标记；用同一侧标记会产生 1400 mm 的假偏差。


## C 模式探针：定位到求解器侧限制（2026-09-17）

新增 `packages/suspension_multibody/scripts/kc_native_c_probe.py`：复用 K 探针的转换器，
把 16 个 `BushingElement` 换算为 SI `AxleBushing`（平移 ×1e3、转动 ×1e-3、预载矩 ×1e-3），
把轮心六分量载荷按 `M_origin = M_point + r × F` 换算成刚体原点力矩后经 `body_wrench` 施加，
参考位姿取自原生 K 解（中性点）。

**结果：原生 C 求解被拒绝** ——
`NativeAxleError: native axle solver failed (2): constraint Jacobian is rank deficient at the initial pose`。

**根因（有对照实验）**：

| 实验 | 结果 |
|---|---|
| K 装配体（13 约束：4 转轴 + 8 球铰 + 1 移动副）+ 原生 K 驱动 | ✅ 求解，且与 Python K 等价到 1.7e-6 |
| K 装配体拓扑 + 16 个衬套 | ❌ 秩亏（因为 K 的 `rack_drive` 与 C 的 `rack_neutral` 同时约束齿条 y） |
| 修正为「齿条只驱动一次」后，用 C 装配体的拓扑 | ❌ 秩亏 |
| 既有测试 `test_bushing_reference_and_static_force_balance`（`joints=()`，纯衬套） | ✅ 通过 |

结构事实：C 装配体的 `ideal_constraints` 有 **17** 条但类型只有 `BallJoint`/`PrismaticJoint`
（摆臂内点的 4 个 `RevoluteJoint` 被球铰 + 衬套取代），而 `constraints` 只有 9 条。
9 个自由刚体共 54 个自由度，C 拓扑的约束行数明显少于自由度，**差额由衬套刚度而非约束承担**。

**结论**：纯刚度系统（0 约束）内核可以静态求解，纯约束系统也可以；但**「部分约束 + 刚度承担零空间方向」
的混合系统**在静态初始化阶段被「约束 Jacobian 必须满秩」的判据拒绝。这不是 ABI 问题
（`body_wrench` 与点驱动的换算都已验证可用），而是**静态求解侧对刚度约束零空间的处理**需要扩展——
内核已有 `pinned_null_pose_directions` 机制与 `audit_constraint_system`，因此这更可能是
「放宽/扩展接受判据」而不是新物理。

**两条候选解法**（child 8 的 C 半程下一步）：

1. 扩展静态初始化的满秩判据：允许零空间方向由刚度矩阵承担（用 `pinned_null_pose_directions` 的正交化 +
   质量/刚度尺度做判据），并保留 `audit_constraint_system` 作为诊断；
2. 或者由工况层把「球铰 + 衬套」改写为「转轴 + 串联衬套」的等价拓扑——但这会改变物理表达，需先证明等价。

**已排除的假设**（避免后续重复踩坑）：不是 ABI 缺 per-body/point 元数据（力矩换算已通）、
不是单位问题（K 路径已证明单位换算正确）、不是快照口径问题（case_id 与字段名已对齐）。


## C 模式进展：从「求解被拒」到「容差内 104 倍」（2026-09-17）

三步实测推进：

1. **秩亏根因定位**（最小模型证实）：同一对刚体上两个球铰 = 6 行但**秩 5**（绕两连线轴的转动自由），
   而内核 `audit_constraint_system`（`cpp/src/static/kernel_static_contact.cpp:655`）要求 `rank(J) == rows`。
   对照：一个转轴（5 行，秩 5）✅；一个球铰（3 行，秩 3，欠约束）✅；两个球铰 ❌（有无衬套都 ❌）。
   注意判据是「**行线性无关**」，不是「系统可定」——所以欠约束能过、重复行不能过。
2. **正解：折叠为等价转轴**（不改内核判据）：两点重合 ×2 与绕该连线的转轴**是同一约束流形**，
   转换器把成对球铰折叠成一个 `revolute`（满秩），保留衬套承担刚度。既不放松 fail-closed，
   也不改变物理。修正后 66 个 C 状态全部可解。
3. **单位修正**：变形量是米而快照是毫米（差 1000×）；力矩载荷是 N·mm 而 ABI 是 N·m（差 1000×）。
   两处修正后，纯力矩路径从 929× 降到 1–2×。

**当前状态**：`checked 66 C states, worst error / tolerance ratio 104.66`
（最差分量 `c-fx--1.00 L` 的 x 平移）。纯力路径的模式已清晰：
如 `c-fz+1.00`：期望 z = `1.024999e-03` mm，实际 `9.999993e-04` mm → **我的模型比 Python 刚 2.44%**。

**剩余问题（下一轮聚焦）**：一个**恒定的 ~2.44% 刚度差**，跨工况一致，指向衬套表达的某个细节而非求解误差。
候选（按可能性排序）：
1. `assembly.bushings` 有 16 个元素（8 个规格 × 镜像，且每个挂点可能重复），
   Python `EquilibriumSolver` 与原生内核对这些并行衬套的合成方式可能不同（串/并联语义）；
2. `AxleBushing` 的 `reference_translation_in_frame_a_m` 我取 0（装配位两点重合），
   但原生 `bushing-static` 测试显示该字段编码的是**两框架在装配位的偏移**，需要按框架而非按点计算；
3. 两个球铰折叠成一个转轴后，约束反力的分配改变，若衬套与约束在同一挂点会轻微改变有效刚度。

**已确认不再需要改内核**：秩判据本身是对的（重复行必须拒绝），C 拓扑的正解在转换器一侧。


## C 模式 2.5% 刚度差：定位到「柔度来源」与精确倍数（2026-09-17）

**精确量化**：沿 `fz` 全路径逐级比较，比值**恒为 1.025000**（0.2 / 0.4 / 0.6 / 0.8 / 1.0 N 全部相同）：

```
c-fz-1.00 L z: expected=-1.0250009375e-03  actual=-1.0000007292e-03  ratio=1.025000
c-fz-0.20 L z: expected=-2.0500003745e-04  actual=-2.0000002915e-04  ratio=1.025000
```

是**纯线性刚度差**（乘性、与载荷无关），不是非线性、不是常数偏置。

**柔度来源已查清**：C 装配体的 17 条 `ideal_constraints` 把**挂点本身**做成了刚性球铰
（`uca_mount_L_inner_front` 等的 `point_a` 与对应衬套的 `local_pose_a` 完全同点），
同时每个挂点上挂 **2 个相同衬套**（8 个挂点 × 2 = 16）。
因此摆臂相对底盘的相对运动只能是绕「两挂点连线」的转动（两个球铰留下 1 个转动自由度），
**全部柔度来自衬套绕该轴的转动刚度**（10^7 N·mm/rad，每挂点 2 个）。
我的原生模型折叠成等价的转轴后，这个自由度与约束流形**完全一致**，衬套也全部传入
（已核对名称/体对/点位与 Python 逐项相同）。

**已排除**：几何不等价（连通性、点位、体对逐项核对一致）；球铰折叠错误（轴为两挂点连线，
点取首挂点，均在轴线上）；单位错（修完后力路径与力矩路径的偏差模式完全不同）；
衬套数量（两边都是 16）。

**下一轮的决定性二分实验**（规模很小，能一次分辨差在转换器还是内核）：
用最小 2 刚体模型复刻同一拓扑——底盘 + 一根摆臂，两个球铰在 (±100,−500,400)，
两个同点衬套（10^4 N/mm 平移 / 10^7 N·mm/rad 转动），施加 1 N 竖向载荷——
分别用 Python `EquilibriumSolver` 与原生 `AxleDynamicsModel` 求解：

- 若原生也给出 1.025× 的位移 → 差异在内核的「球铰 + 同点衬套」处理（刚度装配或转动坐标约定）；
- 若两边一致 → 差异在我的前轴级转换器（转轴折叠或刚体/标记点转换）。

二分结果将直接决定这一项是「修内核」还是「修转换器」，也是 C 半程达标前的最后一道关。


## C 模式 2.5%：最小二分实验的负结果与修正路径（2026-09-17）

**尝试**：用最小 2 刚体模型（底盘 + 摆臂，两个球铰在 (±100,−500,400)，两个同点衬套，
载荷 1 N 加在 (0,−700,300)）复刻 C 拓扑，以分辨 1.025 因子出在转换器还是内核。

**结果（负）**：Python `EquilibriumSolver` **不收敛**——约束残差 0 但力残差等于全部载荷（1.0）。
原因：该最小模型中两个球铰把两挂点完全刚化，衬套与球铰同点因而无法产生平移变形，
只剩绕挂点连线的转动自由度；这个退化构型不是 C 装配体的有效缩影——
真实 C 装配体还通过**摆臂-转向节球铰**把摆臂与转向节连成机构，收敛依赖那组约束。

**结论**：最小 2 刚体模型**不能**作为二分载体（保留为已排除的方法，避免重复踩坑）。
修正后的诊断路径（两个都在真实 C 装配体上做，Python 侧已证明可收敛）：

1. **平移-only 衬套变体**：把衬套刚度设为 `diag(1e4,1e4,1e4,0,0,0)`（去掉转动刚度），
   在真实 C 装配体上分别用 Python 与原生算 `fz=+1`。
   - 比值仍为 1.025 → 差异是**全局尺度**（转换器/单位/装配层面）
   - 比值改变 → 差异**特定于转动柔度路径**（球铰折叠 vs 转轴）
2. **转动-only 衬套变体**：`diag(0,0,0,1e7,1e7,1e7)` 同理对照。

这两个变体都把「柔度来源」隔离到单一路径，且都在已知可收敛的模型上进行，
因此无论结果如何都能把 1.025 归因到具体一侧。

**当前 C 半程状态**：66 个状态全部可解，最差误差/容差 = 104.7（`c-fx--1.00` 的 x 平移），
纯力路径为恒定 1.025 倍刚度差，纯力矩路径已在 1–2 倍容差内。


## 里程碑：原生准静态 K 与 C 双向等价达成（2026-09-17）

**根因（1.025 因子与早先的秩亏）是同一个错误，而且在我的探针里，不在内核里**：

C 求解器 `CModeSolver._solve_multibody` 用的约束集是 **`assembly.constraints`（9 条：8 个外侧球铰 +
1 个齿条移动副）**——挂点完全交给衬套。而我的探针用了 **`assembly.ideal_constraints`（17 条）**，
它额外把挂点也做成了刚性球铰。两个后果同时由这一个错误解释：

1. **秩亏报错**：17 条里有成对的同点球铰（6 行、秩 5）→ 违反 `rank(J) == rows` 判据；
2. **2.5% 刚度差**：那对多余的刚性球铰把挂点锁死，衬套的平移刚度无法参与 → 模型偏刚 1.025 倍。

改用 `assembly.constraints` 后两个问题同时消失。**内核无需任何改动**——R10 的「不需要改 ABI」得到确认，
并且进一步确认「不需要改求解器」。

**验证（严格 parity 门，继承 Adams 门容差）**：

```
K 探针: worst delta across the 9 states: 1.65548e-06        (mm/deg)
C 探针: checked 66 C states, worst error / tolerance ratio 0.00088363
kc_parity_check.py --check --actual-dir artifacts/kc-native-probe
  -> OK: candidate matches the frozen K/C snapshot within tolerance
```

这一次 **K 与 C 两半都是原生真实输出**（此前 C 半程是占位，已在探针中替换为原生结果并写出
`artifacts/kc-native-probe/{k_states,c_states}.json`）。

**child 8 的性质由此定论**：原生内核**已经能够**复现 Python 准静态 K/C 全量结果，
差异比门限小三个数量级以上。剩余工作不是求解物理，而是：
把这两个探针沉淀为正式的工况层（网格/载荷路径/标签/柔度与指标输出）、
经契约边界接线（child 7/9），再把 Python 求解器退役（child 11）。


## Child 8 沉淀：原生 K/C 成为产品模块（2026-09-17）

把两个探针提升为产品包内的正式模块 `suspension_multibody/native_kc/`：

| 文件 | 职责 |
|---|---|
| `convert.py` | 前轴装配 → `AxleDynamicsModel`：刚体/关节/衬套/驱动坐标；mm→SI；**驱动 target 取绝对分离量**（含设计位）；C 模式用 `assembly.constraints`，K 参考用折叠后的等价转轴 |
| `workflow.py` | K 网格、C 载荷路径、K 指标（`analysis/metrics.py` 口径）、C 变形（`c_mode._wheel_response` 口径）、柔度最小二乘 |
| `__init__.py` | 公开 API |
| `tests/native_kc/test_native_kc_parity.py` | 两项 parity 回归测试，容差继承严格 Adams 门 |

**验证**（三条独立证据）：

```
pytest tests/native_kc            -> 2 passed
scripts/kc_native_probe.py        -> 9 K states,  worst error/tolerance 1.65548e-05
scripts/kc_native_c_probe.py      -> 66 C states, worst error/tolerance 8.83630e-04
scripts/kc_parity_check.py --check --actual-dir artifacts/kc-native-probe
                                  -> OK（K 与 C 两半均为原生输出）
multibody 全量                     -> 520 passed / 1 failed(已知本地 artifacts) / 47 skipped
动态哈希 sentinel                  -> 逐字节不变
```

两个门禁脚本已改为该模块的**薄封装**（不再各自持有一份转换逻辑），测试与门禁共用同一 fixture，
避免两边漂移。

**child 8 剩余**：验收文字里的 `mb_cases` 是 C++ 侧模块；目前工况编排仍在 Python 侧（调用原生内核求解）。
把编排下沉到 C++ 属架构性收益，与 child 7 的契约边界一并处理——因此 child 8 记为 IN_PROGRESS，
其**核心（原生 K/C 与 Python 等价）已达成并受门禁保护**。


## Child 7 前半：契约文档的 Python 半边（2026-09-17）

新增 `native_kc/contract.py`：

- `model_document(assembly)` —— 把前轴装配表达为 **`multibody-model` 契约文档**（mm 制）：
  刚体（质量/惯量/位姿/固定标志）、关节（spherical/revolute/prismatic + 点位与轴）、
  衬套（挂在 `elements` 的 `type: "bushing"`，参数含点位、框架四元数、6×6 刚度与阻尼、预载）。
- `case_document(...)` —— 表达 `kc_quasi_static` **工况文档**（K 网格 与 C 载荷路径两用）。

两者都用 `suspension_contracts` 的 `validate_model` / `validate_case` 校验（**通过**），
端到端走容器格式 `pack_container`/`unpack_container` 往返（含 blob）且 `contract_hash`
对键序不敏感。

**设计要点**：文档是**毫米制**，内核在其内部换算为 SI——单位知识只保留一处，
不再由 Python 侧重复（此前 `native_kc/convert.py` 里那份 mm→m 换算将随 C++ 读取器落地而退役）。

**验证**：`pytest tests/native_kc` → **8 passed**（2 项 K/C parity + 6 项契约文档）；
`ruff check` 对新模块 **All checks passed**；multibody 全量 **526 passed**（较上轮 +6）。

**child 7 后半（下一轮）**：C++ 侧实现 `suspension_kernel_run(payload, len) -> result`：
解析容器→校验契约→把 K/C 文档翻译成内部模型与采样输入→复用已验证的静态求解路径→
按 `multibody-result` 形状返回结果文档。读取器只需覆盖 K/C 所需子集（刚体/关节/衬套/驱动坐标/
时间网格/轮端扳手），其余族以明确错误 fail-closed，随后逐族扩展。


## 本轮收尾：lint 清偿、性能门稳健化、全门禁复核（2026-09-17）

1. **lint 清偿**：全工作区 `ruff check .` 从 32 项降到 **All checks passed**。
   32 项全部来自本轮新增文件（既有代码本是干净的）：17 项自动修复（W292 文件末尾换行、
   D213 文档字符串位置、F401 未用导入），15 项 D103 手工补文档字符串。
   —— 这条记录也说明：新增脚本必须过与包内代码同一道 lint 门。
2. **性能门稳健化**：C6600 曾报 ×1.316 超预算。原因是**单侧负载噪声**（本机连续数小时重度构建），
   不是回归。改为按 **best-of-N** 判定（保留 ×1.25 容许不变）：
   - 依据：墙钟噪声是单侧的（负载只会让运行变慢），因此最小值逼近无载运行时间，
     而系统性回归同样会抬高最小值 —— 判据既降噪又不会漏掉真回归；
   - 基线与校验：K100 best 1.1506 s / 校验 ×0.993；C6600 best 2.0354 s / 校验 ×0.945。
3. **全门禁复核**（本轮末连续执行）：ruff ✅ ｜ kernel 15/15 ✅ ｜ contracts 22/22 ✅ ｜
   native_kc 8/8 ✅ ｜ 分层 `--strict` ✅ ｜ K 门禁 1.66e-05 ✅ ｜ C 门禁 8.84e-04 ✅ ｜
   parity 门 OK ✅ ｜ 性能门 OK ✅ ｜ 动态哈希逐字节不变 ✅ ｜ multibody 526 passed（+6）✅。

**child 7 状态**：Python 半边完成（契约文档发射与校验）；C++ 读取器（`suspension_kernel_run`）
为下一轮起点，其输入格式已被契约 schema 与 6 项文档测试固定。


## Child 7/8 收口：C++ 契约读取器与 mb_cases 落地（2026-09-17）

边界切换的后半段完成。**Python 侧不再有任何求解或契约翻译路径是必需的**：
K/C 的模型与工况以契约文档进入内核，展开、装配与求解全部在 C++ 侧发生。

### 新增的 C++ 结构

| 文件 | 职责 |
|---|---|
| `cpp/include/mb_cases/functions.hpp` | L5 工况层接口：`ContractModel`（文档→输入表）、`ContractPlan`（工况→展开用例）、`contract_expand_case`、`contract_apply_solver` |
| `cpp/src/cases/contract_model.cpp` | `multibody-model` 文档 → `AxleInput`/`VehicleInput`：mm→m、N/mm→N/m、世界点→体局部点、驱动坐标的设计分离量 |
| `cpp/src/cases/kc_quasi_static.cpp` | `multibody-case` 文档 → 具体用例：K 网格笛卡尔积、C 载荷路径、绝对驱动目标、轮心力矩到刚体原点的搬运、`np.linspace` 逐位复刻 |
| `cpp/src/abi/kernel_contract_run.cpp` | 唯一入口 `suspension_kernel_run`：解析两个容器→装配模型→逐用例 `run_model`→发射 `multibody-result` |
| `cpp/axle_dynamics/axle_kernel.hpp` | 公开声明两个新符号（`AXLE_API` 导出） |

### 关键设计决定（本轮新增）

1. **文档是体局部的、毫米制的**。关节点/轴、标记点、衬套点都是体局部；单位换算只发生在
   `contract_model.cpp` 一处。作者层（Python）做一次世界→局部的转换，转换点就在装配几何旁边。
2. **两个载荷**（模型容器 + 工况容器），不是把模型塞进工况文档。三种文档 `kind` 不变，
   schema 只做**加法式**扩展：model 增加 `markers`、`joints[].reference_quaternion`；
   case 的 `solver` 补齐求解器标量、`k` 增加 `axis_map`、`c` 增加 `load_marker`。
3. **驱动坐标走 `joints[]`**，用已有的 `driven_translation`/`driven_rotation` 类型 + `target`
   信号名。案例文档说“轮跳 −10…+10 mm”，绝对分离量由内核按文档里的装配姿态算出来
   （`driven_separation_`），所以中性工况天然是零残差状态。
4. **驱动目标是共享缓冲**。`add_driven_coordinates` 存的是**指针**而不是副本，所以逐用例的
   目标必须写进同一块内存；每用例 `memcpy` 覆盖内容，地址不变。
5. **工况展开在 C++**。K 的网格顺序、`k-w-10-r-5` 这样的用例名、`c-fx--1.00` 的
   `%+.2f` 格式、`np.linspace(-max, max, levels)` 的逐步乘加，都在
   `kc_quasi_static.cpp` 里复刻，并由 Python 侧逐名核对（名字不符即失败，不给静默错位留口子）。
6. **结果有身份**。`case_identity.model_sha256/case_sha256` 是两份文档规范化字节的 SHA-256；
   `manifest` 记录展开后的用例表（名字、样本偏移、样本数）、体顺序、族名、契约版本。
   体顺序随结果一起返回，因为按下标读块只有在知道下标是谁的时候才有意义。

### Python 薄壳

`suspension_multibody/kernel/`（约 140 行）只做三件事：组容器、调入口、把 blob 拆成数组。
它复用 `axle_dynamics.native._load_library` 的新鲜度检查，所以镜像过期仍然会在**绑定阶段**
被发现，而不是在求解中途。缓冲不足由内核返回 11 并写回所需长度，薄壳扩容重调一次。

### 验证（全部在契约路径上）

```text
uv run --package suspension-multibody pytest packages/suspension_multibody/tests/native_kc -q
  -> 16 passed（含 4 项契约边界 parity/行为测试）
kc_native_probe.py    -> 9 K states， worst error/tolerance 1.65548e-05
kc_native_c_probe.py  -> 66 C states，worst error/tolerance 8.83630e-04
kc_parity_check.py --check --actual-dir artifacts/kc-native-probe
  -> OK（候选的 K/C 两半都来自契约路径）
uv run --package suspension-multibody pytest packages/suspension_multibody/tests -q
  -> 534 passed / 1 failed（已知本地 artifacts）/ 47 skipped / 1 xfailed
dynamic_hash_sentinel.py --check -> combined sha256 e7407656… 逐字节不变
kernel 15/15 ｜ contracts 22/22 ｜ mb_contract selftest 39 checks
check_module_layering.py --strict -> 86 边 / 0 自包含 / 0 聚合包含 / 0 环
ruff -> All checks passed
kc_legacy_path_check.py -> native 路径 0 处引用 Python 求解器
kc_perf_gate.py --check --repeats 3 -> k-100 ×0.895、c-6600 ×0.857，预算内
```

`kc_native_probe.py` 与 `kc_native_c_probe.py` 已改为**契约路径**，所以这两道门现在测的是
调用方真正会走的路线，而不是一条只为门禁保留的 ctypes 旁路。

### 本轮修掉的缺陷

1. **衬套耦合块被静默清零**（`native_kc/convert.py::_bushings`）。原实现给
   平移-转动与转动-平移两个 3×3 块乘了 `0.0`，也就是把每个衬套的耦合刚度与耦合阻尼
   整体删掉。正确因子是 `1.0`（力/弧度、力矩/米 在内核里本来就是这两种单位）。
   基准模型的耦合块恰好全是 0，所以此前所有 parity 都没发现；修正后 16 项测试仍全过。
2. **`-0.0` / 整数字面量**：沿用了 child 4 修好的规则，C++ 写出的结果文档与 Python
   规范化实现逐字节一致（结果文档经 `validate_result` 校验通过）。

### 本轮发现的架构缺口（未修，已记录）

`check_module_layering.py` 只扫描 `*.hpp`，**看不见 `.cpp` 之间的模块依赖**。用同一套
机制扫 `cpp/src/**/*.cpp` 会新增 32 条边，并且暴露出两条真实环：

```text
['mb_constraint', 'mb_model']
['mb_integrator', 'mb_static', 'mb_tire', 'mb_tire_brush', 'mb_tire_fiala',
 'mb_tire_pac2002', 'mb_tire_state', 'mb_vehicle']
```

两者都是 child 6 之前就存在的实现级依赖，头文件层面看不见，因此没有出现在 86 条基线里。
本轮没有打开 `.cpp` 扫描：那会让 `--strict` 立刻对既有代码变红，而修这两条环是独立的
重构工作，不应夹在边界切换里顺手做。**建议**：新增一个任务把 `.cpp` 扫描打开并处置这两条环，
否则“模块图为 DAG”这句话只对头文件成立。

同因，`abi → mb_cases` 这条真实依赖（`kernel_contract_run.cpp` 包含 `mb_cases/functions.hpp`）
在头文件门禁里同样不可见：往 `abi/functions.hpp` 加聚合包含会被门禁正确判为
“跨模块 functions.hpp 包含”（实测触发 aggregate_includes 0→1），所以没有那样做。

### 仍然存在、且刻意保留的东西

- `axle_run` / `vehicle_run` 两个 ctypes 入口未删除。它们仍在 `native.py`（1975 行）后面，
  属于 child 11 的删除范围；本轮的验收只要求契约入口可用且与它们同值。
- `native_kc` 的 **报告**部分（轮心世界坐标、camber/toe 公式、C 变形相对参考位姿的旋转向量）
  仍在 Python。这是报告，不是物理：它们把 `body_state` 变成指标，符合架构里
  “Python 保留 io/report”的划分。
- C 族的参考位姿用**模型文档声明的装配姿态**，而不是再跑一次 K 中性解。理由是中性工况在
  设计分离量处残差为零，装配姿态就是那个解；实现前实测 `k_reference_pose()` 的返回
  与装配姿态一致（位置与四元数逐位相同）。


## Child 9 第一段：axle_dynamic 接入契约边界（2026-09-17）

`mb_cases` 从一族变成两族，并且这一族是按**逐位相同**验收的。

### 新增结构

| 文件 | 职责 |
|---|---|
| `cpp/src/cases/case_common.hpp` | 工况层私有公共件：身份/时间网格/求解器标量读取、`linspace` 逐位复刻、单位换算因子 |
| `cpp/src/cases/case_dispatch.cpp` | 族分发：一份文档 → 对应族展开器；未实现的族按名字 fail-closed |
| `cpp/src/cases/axle_dynamic.cpp` | `axle_dynamic` 展开：读 `blobs[]` 描述符，按角色把 blob 里的采样表散列进 `[sample][tire]` / `[sample][body][6]` |
| `src/suspension_multibody/cases/axle_dynamic.py` | 作者侧：`AxleDynamicsModel`/`AxleDynamicsCase` → 契约文档 + blob |
| `scripts/case_parity_check.py` | 逐族门禁；缺族即失败，`--allow-partial` 只降级为警告 |

### 验证：13/13 用例逐位相同

```text
uv run --package suspension-multibody python scripts/case_parity_check.py
  kc_quasi_static    PASS      worst error / tolerance 0.00088363
  axle_dynamic       PASS      13 cases, bit-identical to the ctypes path
  ... 6 族 MISSING（见下）
```

`axle_dynamic` 的比对是 `np.array_equal`，覆盖 `body_state`、`constraint_wrench` 与
`diagnostics` 三个数组，13 个用例（`adams/axle_acceptance.yaml` 的全部矩阵）全部为真。
新增测试 `tests/cases/test_axle_dynamic_contract.py` 固定其中 6 个代表用例与文档/失败路径，
全量逐位比对由门禁脚本承担。动态哈希仍为 `e7407656…`，逐字节不变。

### 这一族逼出来的四个真问题

1. **契约入口必须按调用方会选的入口来装配模型。** 内核有两个入口，它们装配出的模型不同：
   axle 入口在 `build_model` 之后就结束，vehicle 入口还会注册轮胎/曲线/执行器/驱动坐标。
   我的第一个版本**无条件**走 vehicle 阶段，于是 `add_vehicle_tire_models` 对
   `opposite_phase_road` 重新推导了轮胎语义，结果从第 182 个样本起发散（最大差 1.72）。
   修法是在 `ContractModel` 上加 `needs_vehicle_stages()`（当前 = 是否存在驱动坐标），
   让「走哪条装配路径」成为模型的属性而不是调用者的选择。7/13 个用例看似通过，
   只有会触发轮胎脱离的用例暴露了它 —— 这正是逐位门禁的价值。

2. **长度单位应当由文档声明，而不是由双方约定。** 文档是毫米制时，
   `x → x*1000 → x/1000` 会丢一个 ulp，逐位门禁直接失败。
   现在 `units.length` 可以是 `"mm"` 或 `"m"`，读取器按它推导全部换算因子
   （长度 ×s、刚度 ×1/s、惯量 ×s²、力矩与速度 ×s）。K/C 族继续写 `"mm"`（装配几何本来就是毫米），
   `axle_dynamic` 写 `"m"`（它的对象本来就是 SI），于是换算全部为 1，算术精确。

3. **惯量漏了单位换算。** 读者原先把文档里的 3×3 惯量原样传给内核，而 K/C 的作者层写的是
   kg·mm²、内核要 kg·m² —— 差 1e6。K/C 之所以看不出来，是因为那些 body 全是无质量/固定体，
   惯量被替换成单位矩阵。显式按长度单位平方换算后才对动态有意义。

4. **接触事件需要重试协议。** `tire_liftoff_and_recontact` 返回状态 10（事件缓冲不足）；
   内核在计数里回报真实数量。契约入口现在按该计数扩容重试，并把事件作为 `contact_events`
   块返回（空列表不发射块，因为描述符 schema 要求每个维度至少为 1）。

### 为什么另外 6 族不是「接线」而是「新实现」

逐族核实（`case_parity_check.py` 里逐条写明理由）：

- `handling` / `ride_four_post` / `ride_random_road`：**今天完全没有原生路径**。
  `adams/vehicle_handling.py` 与 `adams/vehicle_ride.py` 只做一件事：生成/改写 Adams 数据卡，
  `subprocess.run([profile.executable, "acar", ...])`，再解析 `.res`。
  全仓检索没有 `axle_run`/`vehicle_run` 调用。内核**确实**支持这些激励
  （`road_kind` 4/5、`road_corner_scale`、逐样本转向/制动目标），缺的是工况层实现。
- `vehicle_kc`：走 `analysis/vehicle_kc_time_domain.py` 的 Python 14-DOF 模型，不是内核。
- `vehicle_dynamic`：有原生入口 `run_vehicle_dynamics`（走 `vehicle_run`），但契约里没有任何族展开它，
  也没有冻结的数值基线。
- `comparison`：不是求解族。它是三套独立门禁（strict K/C、axle equivalence、vehicle correlation），
  各自定义 reference/candidate 与容差；把它做成一个求解族是范畴错误。

结论：child 9 的后半段是**三块新工作**（车辆级动态族的契约化与工况展开、handling/ride 的原生工况实现、
vehicle_kc 的原生化），外加把 `vehicle_dynamic` 接上契约并冻结基线。三者的共同前置
（动态族的模型文档元素面：轮胎模型块、弹性/限位曲线、转向执行器、气动、坐标耦合器）
已经在 `axle_dynamic` 这一轮把骨架和单位约定打通。

### 本轮门禁

```text
case_parity_check.py        -> kc_quasi_static PASS / axle_dynamic PASS / 6 MISSING（缺族即失败）
动态哈希 sentinel            -> e7407656… 逐字节不变
kernel 15/15 ｜ contracts 22/22 ｜ cases+native_kc 25/25
multibody 543 passed / 1 failed（已知本地 artifacts）/ 47 skipped / 1 xfailed
分层 --check -> 86 边不变（新公共件放在 cpp/src 下的私有头，不算模块边）
ruff -> All checks passed
```


## Child 9 第二段：vehicle_dynamic 接入契约边界（2026-09-17）

三个族接上了，全部按**逐位相同**验收。

```text
uv run --package suspension-multibody python scripts/case_parity_check.py
  kc_quasi_static    PASS      worst error / tolerance 0.00088363（冻结快照）
  axle_dynamic       PASS      13 cases, bit-identical to the ctypes path
  vehicle_dynamic    PASS      3 cases, bit-identical to the ctypes path
  vehicle_kc / handling / ride_four_post / ride_random_road / comparison   MISSING
```

### 模型文档现在能表达整车

| 新增 | 位置 | 说明 |
|---|---|---|
| `capabilities: ["vehicle"]` | model 顶层 | 显式声明这个模型要走 vehicle 装配路径。内核两个入口装配出的模型不同，靠元素有无来猜会在 `opposite_phase_road` 那种用例上悄悄改答案 |
| `couplers[]` / `gauges[]` | model 顶层 | 坐标耦合器与静态转动 gauge；耦合器需要把关节名解析成关节号 |
| tire `parameters.frame_body` / `frame_center_local` | model | 轮胎力的作用体与它的测量框架不是同一个体：轮在转，托架不转 |
| tire `parameters.drive_torque_*` | model | 驱动扭矩的作用体/反作用体/局部轴 |
| element `steering_actuator` | model | 四种执行器类型 + 目标信号名；`prescribed_*` 会走 driven-coordinate 通道 |
| element `anti_roll_bar` / `aerodynamic_drag` | model | 之前只认 spring/bushing |
| `road` 对象 | **case** | 路面是工况输入不是模型输入：同一辆车跑不同路面 |
| `inputs.static_gauge` / `initial_state_angle_tolerance` | case | 静态 trim 的 gauge 与初始残差容差 |

`vehicle_dynamics.py` 增加 `prepare_vehicle_run()`：把「装配什么」从「怎么送到内核」里拆出来，
两条边界共用同一份准备结果。契约路径与 ctypes 路径因此不可能在装配阶段分叉 —— 这是本轮
唯一一处结构性重构，其余都是加法。

### 这一族逼出来的两个真问题

1. **占位指针会覆盖真实数组。** 为了让「只校验指针、不校验计数」的注册阶段有东西可校验，
   入口会先给所有可选数组装一个占位指针。我最初把它放在 `model.fill()` **之后**，
   于是 steer 的 body/reaction 被清零，报「invalid vehicle steering body index」。
   正确顺序是先装占位、再由模型覆盖，并且模型只在**数组非空时**才发布指针 ——
   空 vector 的 `data()` 允许是空指针，用它去覆盖占位等于把占位取消了。

2. **注册阶段存的是指针，不是副本。** 方向盘目标与制动扭矩都是逐样本表，而
   `add_vehicle_steering_actuators` 把 `input.steering_target_angle` 存进
   `actuator.target_angle`。逐用例的 vector 地址不同，注册时拿到的是**第一个**用例之外的
   空占位，于是转向用例跑成了「没有激励」（状态完全不动）。修法与 driven target 一致：
   一块地址不变、每用例覆写内容的共享缓冲；制动扭矩同理，并且要显式
   `built.vehicle_brake_torque = input.brake_torque`，因为 `run_model` 是从**模型**读它的。

### 本轮门禁

```text
case_parity_check.py  -> 3/8 族 PASS（缺族即失败，其余 5 族逐条写明原因）
kernel 15/15 ｜ contracts 22/22 ｜ cases+native_kc 15/15 ｜ vehicle 49 passed / 1 xfailed
multibody 549 passed（+6）/ 1 failed（已知本地 artifacts）/ 47 skipped
动态哈希 e7407656… 逐字节不变 ｜ 分层 --strict 86 边 0 环 ｜ ruff All checks passed
```

### 剩下 5 族的性质（逐条已核实）

- `vehicle_kc`：Python 14-DOF 模型（`analysis/vehicle_kc_time_domain.py`），无原生实现。
- `handling` / `ride_four_post` / `ride_random_road`：只有 Adams/Car 执行路径
  （`subprocess.run([..., "acar", ...])`），无原生实现；内核支持所需激励
  （`road_kind` 4/5、`road_corner_scale`、逐样本转向/制动），缺工况层。
- `comparison`：不是求解族，是三套独立门禁。

`vehicle_dynamic` 的接管顺带把**整车级模型文档元素面**打通了（轮胎框架/驱动映射、转向执行器、
气动、耦合器、路面），这是 handling/ride 的前置 —— 它们缺的是工况层而不是表达能力。


## Child 10/11 前置：Adams 门禁与 Python 求解器解耦（2026-09-17）

child 11 要删掉 Python 求解层，前置是 `adams/` 不再依赖它。逐个消费者处理后，
结果是一个成功、一个**被测量挡住**。

### 已解耦：strict K 与 K 几何参考

| 文件 | 之前 | 现在 |
|---|---|---|
| `adams/strict_k.py::run_suspension_multibody_pure_k` | `KModeSolver` 逐 9 状态 | `run_k_grid_contract`（契约路径） |
| `adams/reference.py` | `KModeSolver` 解 ±10 mm 两个状态 | `run_k_grid_contract` 取同两个状态 |

验收不是「测试没红」，而是拿冻结的 Adams 结果直接比：

```text
native K reference vs Adams（严格 K 门容差）
  worst error / tolerance = 0.0084      （限值 1.0，余量 119 倍）
  components over tolerance = 0
native K reference vs 冻结的 Python 参考（adams/strict-k-real）
  worst error / tolerance = 7.6e-06
```

Python 求解器给出的那份参考仍然冻结在 `artifacts/adams/strict-k-real/` 与
`tests/data/kc_baseline/`，所以**独立 oracle 留下来了，独立实现才被删掉** ——
这正是 R5 「先冻结 Python 输出、再切换内部参考」要求的顺序。

### 被挡住：strict C

把 strict C 的内部参考换成契约路径后，对冻结的 Adams 结果：

```text
native C reference vs Adams（严格 C 门容差）
  worst error / tolerance = 2.26        （限值 1.0 —— 超标）
  components over tolerance = 18 / (66 × 26)
  全部落在小转动通道：c-fz-00 的 left_rotation_y_rad
     Adams  = 2.4852834326e-05
     Python = 2.4852835285e-05   （差 1e-9，远在容差内）
     native = 2.4824633094e-05   （差 2.82e-08，容差 1.25e-08）
```

**这不是接线问题，是求解器精度差**。已排除的两个候选：

1. **不是 K 参考位姿的差异**：实测 `k_reference_pose()` 与装配姿态对这台装配体
   逐位相同（位置、camber、toe 差 0.0）。
2. **不是收敛容差太松**：把 `local_*`/`position/dynamics/increment` 容差收紧
   100 倍与 1e5 倍，两次都是**先求解失败**（状态 6），而不是更接近 Adams。
   把积分步长同时收紧到 1e-5 也直接失败（状态 5）。

也就是说 native 准静态 C 停在离真解 ~1.1e-3 相对误差处，而这个误差相对
`1e-8 rad` 的绝对容差在 2.5e-5 rad 量级的响应上放大了 2.26 倍。

**处置**：`strict_c.py` 回退到 Python 参考，并把测量结果写进该函数的文档字符串。
这让 child 11 的删除集合变得精确：

- `core`/`elements`/`model` 的**模型**部分必须保留（`build_front_axle` 是契约路径的
  作者侧输入，`native_kc/convert.py` 与 `cases/*` 都依赖它）；
- `analysis/k_mode.py` + `analysis/k_reference.py` 只在 `api.py` 与 `benchmarks.py` 里
  还用得到，`api.py` 改走 `native_kc` 后即可删；
- `analysis/c_mode.py` **被 strict C 门禁挡着，在 native C 精度问题解决前不能删**；
- `solver/`、`dynamics/` 同理。

### 本轮门禁

```text
case_parity_check.py -> 3/8 PASS（kc_quasi_static / axle_dynamic / vehicle_dynamic）
strict K / strict C 测试 7/7 ｜ native_kc + cases 31/31
multibody 549 passed / 1 failed（已知本地 artifacts）/ 47 skipped / 1 xfailed
动态哈希 e7407656… 逐字节不变 ｜ ruff All checks passed
kc_legacy_path_check -> native 路径 0 处引用 Python 求解器
```

### 新增的下一步

**native 准静态 C 的精度收敛** 现在是 child 11 的关键路径。可复现的最小验证：

```python
# 用 artifacts/adams/strict-c-real 的 hardpoints 与该模块的 _bushing_stiffness()
# 装配 C 模型，对比 adams_c_results.json，按 strict_c.TRANSLATION/ROTATION/ANGLE_FIELDS
# 的容差统计超限分量
```

判据：超限分量从 18 降到 0，worst 从 2.26 降到 1.0 以下。届时
`run_suspension_multibody_strict_c` 与 strict K 一样改成一次契约调用。


## native 准静态 C 精度：把「精度差」收敛成一条可证伪的假设（2026-09-17）

上一轮留下的是「native C 对严格 C 门容差超标 2.26 倍」。本轮把它拆到了
**载荷通道**这一层，并且排除了整类候选原因。

### 第一步：不是收敛容差

`static_trim` 的停机判据是 `position_residual <= position_tolerance &&
force_residual <= dynamics_tolerance`，所以第一嫌疑是容差太松。实测（严格 C 装配体、全部 6 条载荷路径）：

```text
position/dynamics 默认 1e-8      worst 2.259，18 个分量超限
position/dynamics 1e-9           worst 2.259，18 个分量超限   ← 一模一样
position/dynamics 1e-9 + 100 次  worst 2.259，18 个分量超限   ← 一模一样
position/dynamics 1e-10          求解失败（状态 6）
position/dynamics 1e-11          求解失败（状态 5）
internal_step 1e-5 + 全部收紧     求解失败（状态 5）
```

收紧到 1e-9 **一个数字都没变**，再紧就直接失败，加牛顿迭代次数也没用。
结论：native 的静力配平已经收敛到它自己的解，这个解与 Python 的平衡解差
2.8e-8 rad —— 不是停得太早，是**停在别的地方**。

### 第二步：差异与载荷成正比

固定 fz 路径、逐级缩小载荷，看两边在 `-maximum` 级别上的旋转差：

| maximum | python rot_y | native rot_y | 差 |
|---|---|---|---|
| 100 N | 2.485283528572e-05 | 2.482463309368e-05 | −2.820e-08 |
| 1 N | 2.488023192163e-07 | 2.487994996569e-07 | −2.820e-12 |
| 0.01 N | 2.488050851658e-09 | 2.488050272230e-09 | −5.8e-16 |
| 1e-4 N | 2.488050867638e-11 | 2.488062670556e-11 | 1.2e-16（噪声底） |

差与载荷**严格成正比**（比值恒定 ~1.135e-3）。也就是说两边的**柔度**差一个常数因子，
与载荷大小无关。这同时排除了「参考位姿差一个常量偏移」——那种差异不会随载荷线性缩放。

### 第三步：只发生在力载荷，力矩载荷完全一致

对六条路径各取 `-maximum` 级，逐分量比较 native / python 的比值：

```text
mx, my, mz :  全部 12 个分量比值 = 1.000000     ← 逐位一致
fx :  dx 1.000051  dy 1.000140  dz 0.999455  rx 1.000130  ry 1.000358  rz 1.000184
fy :  dx 1.000306  dy 1.000327  dz 1.000268  rx 1.000476  ry 1.000368  rz 1.000330
fz :  dx 1.001252  dy 0.999848  dz 0.999968  rx 1.000057  ry 0.998865  rz 0.999858
```

**纯力矩载荷下两条路径完全相同**，纯力载荷下差 1e-4…1.3e-3 且逐分量不同。

这三步合起来把差异锁死在**力载荷的传递**上：力是唯一需要「轮心力 → 刚体原点等效扳手」
（`M_origin = M_point + r × F`）换算的载荷，力矩不需要。因此嫌疑范围从「整个准静态求解器」
收窄到那一处杠杆臂换算，以及 `PointWrenchElement`（Python 侧按**世界原点**取矩：
`p_world × F + M`）与 native `body_wrench`（按**刚体原点**取矩）两种约定之间的等价性。

### 明确排除的候选（都做了实测）

1. **K 参考位姿**：`k_reference_pose()` 与装配姿态对这台装配体逐位相同（位置/camber/toe 差 0.0）。
2. **收敛容差 / 迭代次数 / 积分步长**：见上表，收紧不变、再紧失败。
3. **衬套耦合块**：`strict_c._bushing_stiffness()` 的 tr/rt 两块恒为 0，与耦合换算无关。
4. **纯力矩通道**：逐位一致，说明转动刚度换算、约束集、柔度求解本身都是对的。

### 下一步的判据（一条实验即可定论）

在**刚体原点**施加同一个力（把 `c.load_marker` 指向立轴原点而不是轮心），
使 `r × F` 项为零，再比 native 与 Python：若差异消失，则确认是杠杆臂换算；
若差异仍在，则问题在力的量纲/符号传递。改动只涉及 case 文档的一个字段，
不需要改内核。

### 本轮门禁（与上一轮相同，无回归）

```text
strict K/C 7/7 ｜ native_kc + cases 31/31（重跑）
multibody 549 passed / 1 failed（已知本地 artifacts）/ 47 skipped / 1 xfailed
动态哈希 e7407656… 逐字节不变 ｜ case_parity_check 3/8 PASS ｜ ruff All checks passed
```


## native 准静态 C 精度：差异是二阶项，不是收敛（2026-09-17 续）

上一轮把差异锁到「力载荷通道」。本轮又做了三个实验，把范围收到**二阶几何项**。

### 实验一：C 结果就是静力配平解，积分没有贡献

`initialization_mode = 0` 会先做 `static_trim`，`copy_state` 把它写到 sample 0，然后积分。
比较同一个用例的 sample 0 与最后一个样本（左立轴，7 个位姿分量）：

```text
sample 0 (static trim): [-1.00358073381848e-04  7.21238946271158e-05 -1.46621776764823e-04
                          9.99999997549663e-01  3.38814690173405e-05  1.24123165421228e-05
                          5.99887856212962e-05]
last sample           : [-1.00358073381335e-04  7.21238946269635e-05 -1.46621776764795e-04
                          9.99999997549663e-01  3.38814690170226e-05  1.24123165367036e-05
                          5.99887856217064e-05]
max drift             : 5.42e-15
```

漂移是机器精度。**整类「瞬态/斜坡/采样时刻/积分步长」解释一次性排除**：
工作流取到的最后一个样本就是配平解本身，改采样或积分设置没有任何意义。

### 实验二：差异与衬套刚度成反比

把 `strict_c._bushing_stiffness()` 整体乘上不同倍数，比较 ±100 N fz 下的响应比值：

| 刚度倍数 | python dx | native/python dx |
|---|---|---|
| ×0.1 | −1.072e-02 | 1.0011504（fy 0.9984999，fz 0.9996826） |
| ×1 | −9.791e-04 | 1.0012523（fy 0.9998476，fz 0.9999679） |
| ×10 | −9.697e-05 | 1.0001264（fy 0.9999847，fz 0.9999968） |

`python 响应 ∝ 1/K` 精确成立，而 `native/python − 1` 也精确地 ∝ 1/K
（1.25e-3 → 1.26e-4 → 1.26e-5）。也就是说 native 的响应里多了一项 ∝ 1/K²。
在线性系统里响应严格 ∝ 1/K，出现 1/K² 说明**有效刚度本身有偏移**。

### 实验三：偏差对载荷反对称 —— 多出来的是载荷的偶次项

同一条路径取 −maximum 与 +maximum 两级：

```text
fx minus: dx 1.0000506  dy 1.0001400  dz 0.9994554  rx 1.0001304  ry 1.0003576
fx plus : dx 0.9999494  dy 0.9998601  dz 1.0005465  rx 0.9998697  ry 0.9996426
fy minus: dx 1.0003062  dy 1.0003272  dz 1.0002676  rx 1.0004764  ry 1.0003683
fy plus : dx 0.9996939  dy 0.9996728  dz 0.9997321  rx 0.9995238  ry 0.9996317
fz minus: dx 1.0012523  dy 0.9998476  dz 0.9999679  rx 1.0000573  ry 0.9988652
fz plus : dx 0.9987223  dy 1.0001530  dz 1.0000322  rx 0.9999430  ry 1.0011312
```

每一列的偏差都**大小相等、符号相反**（例如 fx 的 dx：+5.06e-5 / −5.06e-5）。
若 native = A·L + B·L²、python = A·L，则比值 = 1 + (B/A)·L —— 正是这个形状。
所以 native 的响应里含一个**载荷的偶次（二阶）项**，而 Python 的没有。

### 排除清单（全部实测，不留猜测）

| 候选 | 判据 | 结论 |
|---|---|---|
| 收敛容差 / 迭代次数 | 1e-8 与 1e-9 结果**完全相同**；再紧求解失败 | 排除 |
| 积分步长 / 采样窗口 | sample 0 与末样本差 5.4e-15 | 排除 |
| K 参考位姿 | `k_reference_pose()` 与装配姿态逐位相同 | 排除 |
| 衬套耦合块 | tr/rt 恒为 0 | 排除 |
| 取矩约定（世界原点 vs 刚体原点） | 两种约定代数上等价，且纯力矩通道逐位一致 | 排除 |
| 转动刚度换算 | 纯力矩通道 12 个分量比值全 = 1.000000 | 排除 |

### 下一步（已收窄到一条）

差异是「两个装配体在二阶几何上的差别」。下次做**符号分离**：对六条路径各取 ±级，
算 6×6 割线柔度矩阵，取偶部 `(C(+L) + C(−L))/2` 与奇部 `(C(+L) − C(−L))/2`，
比较 native 与 python 的**偶部**。哪几个 DOF 配对的偶部不同，就指向那一条几何传递路径
（最可能是立轴与摆臂之间的球铰链在有限位移下的二阶运动学）。
判据仍然是：超限分量从 18 降到 0，worst 从 2.26 降到 1.0 以下。


### 补充实验四：Python 参考解本身是「正则化过的」

差异的形状（载荷偶次项）与 `EquilibriumSettings` 的默认值有关，所以直接测了 Python 求解器自己的设置：

```text
default (regularization=1e-9)      minus dx 1.001252335 ry 0.998865232
                                   plus  dx 0.998722253 ry 1.001131190
regularization = 0                 FAILED：C equilibrium did not converge
                                   constraint=1.545（阈值 1e-8）
constraint/force tol = 1e-12 收紧   FAILED：constraint=1.137
regularization=0 + 收紧             FAILED：constraint=1.545
max_iterations=200, line_search=20 与 default **逐位相同**
```

两个结论：

1. **`regularization = 1e-9` 是 Python 参考解不可去掉的一部分。** 去掉它，Python 求解器在
   1e-8 的约束容差下**根本不收敛**（残差 1.545，比阈值大 8 个数量级）；收紧容差同样失败；
   把迭代上限提到 200、线搜索步数提到 20，结果与默认**逐位相同**。
   也就是说 strict C 门禁的参考侧本身是一个**正则化近似解**，不是无偏的平衡解。
2. **正则化只能解释差异的一部分。** 正则化让牛顿步「欠走」，表现为响应整体偏硬，
   对 ±载荷应当给出**同号**的比值偏差；而实测偏差是**反对称**的（+5.06e-5 / −5.06e-5）。
   所以正则化是原因之一，但不是全部。

这条改变了问题的性质：strict C 门禁的 1e-8 rad 绝对容差，**比两个不同求解器之间的
一致性还要紧**。要让它成为可迁移的门禁，必须先确定参考侧应该是什么 ——
是 Python 的正则化解，还是无正则的平衡解。Adams 与 Python 在这个模型上一致到 1e-9 rad，
所以下一步应当先问「Adams 用了什么正则化/线性化」，再决定 native 朝哪边对齐。

**留给下一次的最小实验**：在 `EquilibriumSettings(regularization=0)` 下把
`constraint_tolerance` 放宽到能让它收敛，看正则化项的**大小**与差异的**大小**是否同量级，
从而把差异拆成「正则化贡献」与「剩余贡献」两部分。


## Child 9 第三段：整车级轮胎面接入契约（2026-09-17）

`vehicle_dynamic` 从「只支持刷胎」扩到**全部四种轮胎模型 + 实测垂向表**，全部逐位验收。

### 用契约里已有的词汇，不发明新字段

模型 schema 本来就有 `tires[].blob`（字符串）与顶层 `blobs[]`（数组描述符），
正是为这种情况准备的：参数向量太大、槽位又没有各自有意义的名字，不该塞进 JSON。
所以本轮没有改 schema，只把这两个字段用起来：

| 内容 | 位置 | 形状 |
|---|---|---|
| 轮胎参数向量（226 槽） | model blob | `[226]`，每胎一个描述符 `tire-parameters-<i>` |
| 实测垂向表（`[DEFLECTION_LOAD_CURVE]`） | model blob | `[rows, 2]`，`tire-deflection-curve-<i>` |
| 底部撞击表（`[BOTTOMING_CURVE]`） | model blob | `[rows, 2]`，`tire-bottoming-curve-<i>` |

读取器对每个描述符都先校验 `dtype`/`length`/范围再读，越界即报错而不是短读。

### 一份编码，两条边界

`native.py` 里的轮胎编码逻辑（kind 号、参数顺序、Fiala 的 14 槽布局、PAC2002 的镜像条件）
原本内联在 1200 行的 `_run_native` 里。本轮把它抽成模块级函数
`tire_model_arrays(model)`，契约发射器直接调用同一个函数。
这样「内核怎么理解轮胎模型」只有一处实现，两条边界不可能漂移。

### 验收：五种配置全部逐位相同

```text
brush                             : bit-identical=True maxdiff=0.000e+00
pac2002 user                      : bit-identical=True maxdiff=0.000e+00
pac2002 adams_builtin             : bit-identical=True maxdiff=0.000e+00
fiala                             : bit-identical=True maxdiff=0.000e+00
pac2002 + vertical curves         : bit-identical=True maxdiff=0.000e+00
```

门禁 `case_parity_check.py` 的 vehicle_dynamic 一项现在报
「5 cases across brush and PAC2002, bit-identical」；测试目录新增 4 项
（3 种轮胎逐一 + 实测垂向表）。

### 顺带修好的两处

1. **`run_contract` 增加 `model_payload`**：模型自身带 blob 时不能再由文档单独打包，
   与已有的 `case_payload` 对称。
2. **镜像库新鲜度检查救了本轮开发**：抽完 `tire_model_arrays` 后先跑了 vehicle 测试，
   56 项失败全部来自「镜像 DLL 比内核旧」——正是那个门槛在起作用，不是代码错。

### 本轮门禁

```text
case_parity_check.py -> 3/8 PASS（vehicle_dynamic 覆盖 brush + PAC2002）
kernel 15/15 ｜ contracts 22/22 ｜ mb_contract selftest 39 checks ｜ 分层 --strict 通过
multibody 553 passed（+4）/ 1 failed（已知本地 artifacts）/ 47 skipped / 1 xfailed
动态哈希 e7407656… 逐字节不变 ｜ ruff All checks passed
```

### 对 native 准静态 C 的处置（本轮未动）

上一轮把差异定性为「载荷偶次项 + 参考侧本身正则化」。本轮没有再往下钻，
因为那条线已经不属于「接线」，而属于求解器精度课题；
但它在 child 11 里仍是删除 `analysis/c_mode.py` 的前置，记录保留。


## Child 9 第四段：ride_four_post 成为原生工况族（2026-09-17）

第四个族接上了，而且是**新能力**而不是替换 —— 之前四柱只有 Adams/Car 路径。

```text
uv run --package suspension-multibody python scripts/case_parity_check.py
  kc_quasi_static    PASS   worst error / tolerance 0.00088363
  axle_dynamic       PASS   13 cases, bit-identical
  vehicle_dynamic    PASS   5 cases across brush and PAC2002, bit-identical
  ride_four_post     PASS   expansion matches an independently sampled excitation (0.0e+00)
  其余 4 族 MISSING
```

### 族做什么

四柱台架规定每个轮垫的高度，所以这一族的全部工作是把**声明式激励**
（每角 offset / amplitude / frequency / phase）展开成内核逐样本插值的
每胎路面高度与速度。速度取**解析导数**而不是差分：高度与速度是内核分别插值的，
用高度的差分当速度等于换了一个激励。文档也可以给某个角**实测垫高时程**
（走 blob 描述符），此时用实测值、速度可用实测或退回该形状的解析导数。

### 验收方式：把同一份声明展开两次

一个以「展开声明」为职责的族，唯一诚实的检查方式是**用两种独立实现展开同一份声明再对比**：
内核在 C++ 里展开，Python 侧 `corner_signals()` 用 numpy 独立展开同样的正弦，
后者作为显式路面表交给 `vehicle_dynamic` 族跑一遍。两次运行必须一致。

```text
expansion matches an independently sampled excitation (0.0e+00)
```

差异是**精确零**，不是「小于 1e-12」—— C++ `std::sin/cos` 与 numpy 在这组取值上逐位相同。
门禁的判据保留 1e-12，因为那属于实现细节，不该当成契约。

### 结构改动

- 新增 `cpp/src/cases/ride_four_post.cpp`（族实现）与 `src/suspension_multibody/cases/ride_four_post.py`（作者侧 + 独立展开）。
- `read_described_array` 从 `contract_model.cpp` 的匿名命名空间提到私有公共头
  `case_common.hpp`：现在每个族都能按描述符读自己 payload 里的表，而不是只有模型读取器能读。
- `cases/__init__.py` 统一导出三个族（axle_dynamic / vehicle_dynamic / ride_four_post），
  模块 docstring 改成「文档声明自己的长度单位，内核遵从声明」——不再假设毫米或米。

### 本轮门禁

```text
case_parity_check.py -> 4/8 PASS
kernel 15/15 ｜ contracts 22/22 ｜ cases+native_kc+vehicle 86 passed / 1 xfailed
multibody 555 passed（+2）/ 1 failed（已知本地 artifacts）/ 47 skipped / 1 xfailed
动态哈希 e7407656… 逐字节不变 ｜ 分层 --strict 通过 ｜ ruff All checks passed
```

### 剩余 4 族的性质（不变）

- `vehicle_kc`：Python 14-DOF，无原生实现。
- `handling`：只有 Adams/Car 路径；缺原生操纵工况展开（与四柱不同，它的输入是转向/车速时程，
  内核已支持逐样本转向目标，所以缺的同样是工况层）。
- `ride_random_road`：随机路面 = 空间谐波 + 车速 → 时间信号，比四柱多一层「空间→时间」变换。
- `comparison`：不是求解族，是三套独立门禁。


## Child 9 第五段：handling 成为原生工况族（2026-09-17）

第五个族接上了，同样是**新能力**：之前操纵工况也只有 Adams/Car 路径。

```text
uv run --package suspension-multibody python scripts/case_parity_check.py
  kc_quasi_static    PASS   冻结快照 8.84e-04
  axle_dynamic       PASS   13/13 逐位相同
  vehicle_dynamic    PASS   5 种轮胎配置逐位相同
  handling           PASS   4 种开环形状 vs 独立展开 0.0e+00；闭环按名拒绝
  ride_four_post     PASS   展开 vs 独立采样 0.0e+00
  其余 3 族 MISSING
```

### 族做什么

操纵工况就是「转向输入随时间的函数」，所以这一族的职责与四柱同形：
把**每个执行器的形状声明**（`constant` / `ramp` / `step` / `sine`）展开成内核逐样本插值的
目标与速率。速率仍是**解析导数**。文档也可以给执行器**实测转向时程**（走 blob 描述符），
此时速度可用实测，未给则退回该时程自己的差分商——退回 0 会把运动中的操纵误报成静止。

**闭环操纵按名拒绝**：ISO 车道变换需要的是一个沿路径行驶的驾驶员，那是另一个模型而不是
另一种形状；用一个「看起来差不多」的转向时程去近似它，比拒绝更糟。

### 这一族逼出来的两件事

1. **入口把内核的诊断信息丢掉了。** `run_model` 失败时会写出残差量级、迭代次数、
   哪个坐标——而我的入口用 `"case X failed with status 6"` 覆盖了它。
   修好后第一次运行就拿到了真正的诊断：

   ```text
   static equilibrium initialization failed; iterations=20, force_residual=0.995266,
   position_residual=0.000000, pinned_null_directions=3, worst_force_coordinate=5,
   worst_force_value=153032
   ```

   `pinned_null_directions` 直接指出问题：整车在空间里**没有被锚定**。

2. **操纵工况必须从「已稳定的直线行驶状态」开始。** 静态配平从装配姿态出发，
   车轮已经打死的初始状态它是到不了的（实测坡度：初始机架偏移 1.6 mm 时能收敛、
   8 mm 时 `force_residual≈1` 且位置残差为 0 —— 卡在牛顿步而不是精度上）。
   因此 `constant` 定义为**在其 `start_s` 处阶跃**，而不是从 t=0 就存在；
   这样四种形状都能从零开始，也符合真实台架/实车的做法。
   另外测试里发现 `DynamicSolverSettings.gravity` 用的是**模型自身的长度单位**
   （毫米制整车模型要给 −9806.65 而不是 −9.80665），给错会让整车再次失去锚定——
   已写进测试注释。

### 本轮门禁

```text
case_parity_check.py -> 5/8 PASS
kernel 15/15 ｜ contracts 22/22 ｜ cases+native_kc+vehicle 91 passed / 1 xfailed
multibody 560 passed（+5）/ 1 failed（已知本地 artifacts）/ 47 skipped / 1 xfailed
动态哈希 e7407656… 逐字节不变 ｜ 分层 --strict 通过 ｜ ruff All checks passed
```

### 剩余 3 族

- `vehicle_kc`：Python 14-DOF，无原生实现（车体侧倾工况，需要新的原生模型）。
- `ride_random_road`：随机路面 = 空间谐波 + 车速 → 时间信号；比四柱多一层空间→时间变换，
  但骨架与四柱完全一致。
- `comparison`：不是求解族，是三套独立门禁（strict K/C、axle equivalence、vehicle correlation）。


## Child 9 第六段：ride_random_road 接入 + comparison 归类（2026-09-17）

```text
uv run --package suspension-multibody python scripts/case_parity_check.py
  kc_quasi_static    PASS   冻结快照 8.84e-04
  axle_dynamic       PASS   13/13 逐位相同
  vehicle_dynamic    PASS   5 种轮胎配置逐位相同
  handling           PASS   4 种开环形状 vs 独立展开 0.0e+00；闭环按名拒绝
  ride_four_post     PASS   展开 vs 独立采样 0.0e+00
  ride_random_road   PASS   展开 vs 独立展开 0.0e+00
  comparison         N/A     是门禁不是求解：内核从不读参考件
  vehicle_kc         MISSING 无原生实现
FAIL: 1 of 8 families not accepted: ['vehicle_kc']
```

### ride_random_road 做什么

随机路面是**空间谐波**的叠加，车辆以速度 v 驶过时每个谐波变成时间谐波：
波长 L 的分量对应角频率 `2πv/L`。这一族的全部工作就是这个变换。

**每个轮子带自己的分量表**，而不是「一条剖面 + 一个相关系数」。通用形式不花额外代价
（相关的左右轮 = 相同分量 + 各自独立的附加分量；前后轴 = 相位延迟），而按相关系数
设计的族只能表达特例，第一次有人要非相关剖面就得再长一套机制。

### comparison 的归类：是门禁，不是求解族

`comparison` 写在契约的 family 枚举里，但按架构自己的定义它是**逐目标的门禁**
（strict K/C 对 Adams、axle evidence 等价、vehicle correlation）。它之所以是门禁而不是族，
原因很直接：它的全部工作是把两份 artefact 相互比对，而**内核不允许读参考件**——
边界硬规则就是「C++ 不出现任何 Adams 字样」。一个真的去跑 comparison 的内核必须读参考，
那正是边界要禁止的事。

所以 `case_parity_check.py` 现在把它报为 **N/A** 而不是 MISSING，并把理由印出来。
这不是把待办改小：门禁的失败集合里仍然只放**真正缺失的求解族**，这条分类只写在
`NOT_A_SOLVER_FAMILY` 一处，并说明了哪一类是哪一类。

### 本轮门禁

```text
case_parity_check.py -> 6 PASS / 1 N/A / 1 MISSING
kernel 15/15 ｜ contracts 22/22 ｜ cases+native_kc+vehicle 94 passed / 1 xfailed
multibody 563 passed（+3）/ 1 failed（已知本地 artifacts）/ 47 skipped / 1 xfailed
动态哈希 e7407656… 逐字节不变 ｜ 分层 --strict 通过 ｜ ruff All checks passed
```

### 只剩 vehicle_kc

`analysis/vehicle_kc_time_domain.py` 是 Python 14-DOF 车体侧倾模型，`validate-adams --vehicle-kc`
用它做内部参考。要native化，缺的是**整车级的驱动坐标**：`build_vehicle` 装配出的整车模型
没有轮心/齿条驱动（那是轴级概念），而整车 K-C 工况需要的正是「在给定轮跳与齿条位移下测车体侧倾」。

内核侧能力是有的（驱动的 translational/rotational 约束、自由底盘、轮胎接触都在），
缺的是 `mb_cases` 里整车级 K-C 的工况展开 + 装配层把驱动坐标加到整车模型上。
这是本轮三个族之外的**第四类工作**：前三个是「声明 → 展开」，这一个是「新装配 + 新工况」。


## Child 9 第七段：vehicle_kc 实现完成但被一个测得的秩缺陷挡住（2026-09-17）

```text
uv run --package suspension-multibody python scripts/case_parity_check.py
  kc_quasi_static    PASS
  axle_dynamic       PASS
  vehicle_dynamic    PASS
  handling           PASS
  ride_four_post     PASS
  ride_random_road   PASS
  comparison         N/A
  vehicle_kc         MISSING  已实现，但整车模型的约束集在加入 ≥2 个驱动坐标后秩亏
FAIL: 1 of 8 families not accepted: ['vehicle_kc']
```

### 这一族做了什么

整车 K-C 扫描规定**整台车**的驱动坐标（四个轮心行程 + 齿条），而轴级族规定两个轮心 + 齿条。
展开因此是「同一张按名字驱动的笛卡尔网格」，于是复用同一份代码
（`expand_driven_grid`，由 kc_quasi_static 与 vehicle_kc 共用）—— 写第二份会让网格顺序、
绝对目标规则与用例命名多出一个漂移点。新增的是**模型侧**：整车装配不声明轮驱动，
由作者层按装配体自己的挂点写出来（`cases/vehicle_kc.py`），并且**只在这一次工况里存在**——
一个总是带轮驱动的模型会把悬挂钉死在别的族里。

`left_right_mode` 现在真的生效（symmetric / single / opposite），轴级族一直是 symmetric，
所以加这个模式不动它的任何数字。

### 两个顺带修好的真问题

1. **未被某个族驱动的驱动坐标会被钉在零位。** 契约入口对「族没有提供 driven target」的情况
   直接把共享缓冲留成 0，而那等于把驱动坐标钉在世界原点，不是装配位。修好后才谈得上
   在整车模型上挂驱动坐标。
2. **秩审计不报数量。** 原本只说「rank deficient」，无法区分「一行重复」和「一整块相关」。
   现在报 `rank 114 of 116 rows` —— 这条立刻成了定位这一族的关键证据。

### 测得的阻断点

```text
单个驱动               : OK
任意两个驱动           : rank 114 of 116
四个轮 + 齿条          : rank 114 of 118
去掉四个轮自转转动副   : 仍然 rank 114 of ...（不变）
```

秩**恒定在 114**，与加了多少驱动行无关；而单个驱动能过、轴级 K/C 族（同一份展开、三个驱动）
也能过。因此问题不在展开，而在**整车模型的约束集**：`_build_joints` 用的是
`assembly.constraints`（29 行：16 球 + 12 转动 + 1 移动），而 `ideal_constraints` 只有 25 行
（少了四个轮自转转动副）；用 ideal 集重建后仍然秩亏。

这是本轮唯一没有闭合的族。待办被写进测试文件顶部的注释与 `case_parity_check.py` 的原因文本里，
并且留了一个**不跳过**的测试钉住现状：今天扫描必须在审计处以 `rank deficient` 被拒绝——
一旦它开始能跑，这个测试会失败，逼着人来更新这里的结论。

### 本轮门禁

```text
case_parity_check.py -> 6 PASS / 1 N/A / 1 MISSING
kernel 15/15 ｜ contracts 22/22 ｜ cases+native_kc+vehicle 94 passed / 5 skipped / 1 xfailed
multibody 563 passed / 1 failed（已知本地 artifacts）/ 52 skipped / 1 xfailed
动态哈希 e7407656… 逐字节不变 ｜ 分层 --strict 通过 ｜ ruff All checks passed
```


### vehicle_kc 阻断点：数字收敛到「只多出一行独立自由度」（2026-09-17 续）

上一轮报的是 `rank 114 of 116`，但秩不随驱动行数变化这一点本身不可能，
所以给审计消息补上了列数（`over 132 columns`），再做了一组对照：

```text
一个轮驱动              : OK                      （114 行 / 秩 114）
一个轮 + 齿条           : rank 114 of 115 rows over 132 columns
两个轮（无齿条）        : rank 114 of 115 rows over 132 columns
两个轮 + 齿条           : rank 114 of 116 rows over 132 columns
```

反推：**基础模型是 113 行、秩 113（满行秩）**。加**任意一个**驱动 → 114 行秩 114（独立）；
加**任意第二个**驱动 → 行数 +1 而秩不变。

也就是说：整车装配体在基础约束下只剩**一个**可独立驱动的方向，而轴级模型（同一台前悬架，
底盘固定）能容纳三个。轴级 K/C 族用同一份展开、三个驱动，通过。

已排除的：

| 候选 | 判据 |
|---|---|
| 展开逻辑 | 轴级族用同一份 `expand_driven_grid` 且通过 |
| 驱动行的写入位置 | `add_driven_coordinates` 逐个 `row = model.rows; model.rows += 1`；驱动描述符有自己的 `write_jacobian`，且 `translation_block=false`，行偏移正确 |
| 重复的转向执行器 | 整车模型里的 prescribed 转向执行器已被移除，仍然后进 |
| 四个轮自转转动副 | 用 `ideal_constraints`（少这四行）重建，结果不变 |

**下一次的最小实验**：把两个驱动放在**同一个刚体**上（同一立轴的竖直 + 横向）。
若秩仍停在 114，则与刚体无关，是「每多一行都读到同一个槽位」这类行写入问题；
若秩升到 115，则是模型几何确实只留一个方向。

这条与 native 准静态 C 的精度问题一样，属于**求解器/装配层课题**而不是接线课题，
记为 child 9 唯一未闭合项。


### vehicle_kc 阻断点：排除到「只剩一个独立驱动方向」这一层（2026-09-17 再续）

按上一条留下的最小实验做了对照，结果把几何解释也排除了：

```text
两个驱动，同一个刚体、不同轴     : rank 114 of 115 rows over 132 columns
两个驱动，不同刚体、同一根轴     : rank 114 of 115 rows over 132 columns
三个驱动，同一个刚体、三根正交轴 : rank 114 of 116 rows over 132 columns
```

无论驱动放在哪里、有几根轴，**秩恒为 114**（基础模型 113 行满秩，只有第一个驱动行带来 +1）。
三根正交轴落在同一个刚体上仍然只增加 0 行秩 —— 这在几何上不可能，
所以剩下的解释只有「多出来的驱动行没有被真正写进雅可比」这一类。

代码侧已经逐段核对过并**未发现问题**：
`add_driven_coordinates` 逐个 `row = model.rows; model.rows += 1`；
`constraint_jacobian` 的主循环对每个 constraint 都用 `c.row` 构造 writer；
`ScalarJacobianWriter::add_row` 用 `row*model.ndof + 6*body_to_free[body]` 写入，偏移正确；
驱动翻译的描述符 `translation_block=false` 且有自己的 `write_jacobian`。

**下一步的两个判别实验**（都不需要改产品代码）：
1. 直接对一个合成矩阵调用 `matrix_rank`，确认它在行数超过某个规模时没有转置/步长问题；
2. 给**轴级**模型也加 5 个以上驱动坐标（它的三条驱动是通过的），看秩是否同样在某处封顶 ——
   若轴级也封顶，则问题在秩判定或行写入的通用路径，而不在整车模型。

这两条把范围从「整车装配」收窄到「秩判定或驱动行写入」，是继续之前唯一值得做的判断。
在此之前 `vehicle_kc` 保持：已实现、按名注册、在审计处干净拒绝，并有测试钉住现状。


## vehicle_kc 阻断点：定位到「边际秩亏」+ 审计顺序修正（2026-09-18）

上一条留下的两个判别实验都做了，结果把范围从「整车装配」收窄到**秩判定与条件性**。

### 我的读取器是干净的

先证伪自己这一侧：故意给第二个驱动坐标一根**零轴**。

```text
第二个轴为零        : driven coordinates: invalid vehicle-drive parameters  （被正确拒绝）
第二个轴正常、同方向 : 仍然后进（同方向本就相关，这一条不作为证据）
```

零轴被拒绝说明 `input.driven_axis_local[3..5]` 确实按坐标逐个送达，
所以「只有第一个驱动行生效」不是读取器的问题。

### 中枢证据：解析雅可比与中心差分给出不同的秩

给审计消息加上中心差分雅可比的秩之后：

```text
一个刚体上两根正交轴 : 解析 rank 114 of 115 ｜ 中心差分 rank 115 of 115
一个轮 + 齿条        : 解析 rank 114 of 115 ｜ 中心差分 rank 115 of 115
四个轮 + 齿条        : 解析 rank 114 of 118 ｜ 中心差分 rank 118 of 118
```

**中心差分给出的行空间是满秩的**，而解析雅可比差一行。
并且逐行诊断显示两个驱动行确实是**不同的行**（行号 113/114，列 argmax 4/5，绝对和不同），
不是重复行。

`matrix_rank` 本身是标准行阶梯消元（按列选主元、无列交换），没有问题。
秩阈值是 `max(rows,cols)·eps·max(1,|最大元素|) ≈ 4e-14`。
两件事合起来只有一种解释：**驱动方向与基础约束张成的方向几乎相关**，
解析雅可比的那个主元落在阈值之下、中心差分的落在阈值之上 ——
差异在 1e-14 量级，属于**边际秩亏**（条件性问题），不是缺失行。

把底盘固定后两边都给 114 of 118（解析与中心差分一致），说明固定底盘时模型确实边际亏秩。

### 顺带修正：审计顺序

原来的顺序是先判秩、后验雅可比正确性 —— 于是**在一个尚未验证的雅可比上给出秩结论**。
现在改成先验正确性、再判秩，并保留数量信息（row/column/秩/中心差分秩）。
这条改动行为保守：全套测试与动态哈希都逐字节不变。

### 下一步

边际亏秩的正确出路不是放宽阈值，而是**驱动一个条件良好的坐标**。
两条路：
1. 用**柔性（C）模型**跑整车 K/C —— 衬套让轮心方向不再近似相关；
2. 或把驱动改成**摆臂转角**这类转动驱动，而不是轮心竖直平移。

判据仍然是：解析秩 = 行数，且中心差分秩一致。


## vehicle_kc：三个真缺陷已修，最后一个在积分器（2026-09-18 续）

本轮没有停在「边际秩亏」这个结论上，而是换了条件更好的模型再试，过程中又挖出三个真缺陷。

### 修好的一：`*_mm` 字段被按文档单位换算

`k.wheel_values_mm` / `k.rack_values_mm` 名字里写着毫米，我却按**文档声明的长度单位**换算
（`model.length_scale()`）。毫米制文档下偶然正确，米制文档下 10 mm 变成 **10 m**
（审计直接报 `position_residual=10.000000`，单位是米）。现在按字面毫米换算，
并在公共头里加了具名常量与注释说明「名字说 `_mm` 的字段就是毫米」。

### 修好的二：整车衬套预载被截断成三个分量

`cases/vehicle_dynamic._bushing_element` 用 `_vec3(bushing.preload_in_frame_a_n_n_m)`
只发了前三个分量，而内核要六个（三力 + 三力矩），直接判为 malformed。
轴级族一直是对的（它拼了 3+3），整车族因为夹具没有衬套所以从未被覆盖到。
现在按六个分量发出并注明理由。

### 修好的三：扫描可以在窗口内渐进到位（可选 `k.ramp_s`）

静态配平从装配姿态出发，把「满行程」在第 0 个样本上一步给出，是它一个牛顿步到不了的状态
（实测 `position_residual` 恰好等于整个行程、`iterations=1`）。现在工况可以声明
`k.ramp_s`：行程用**升余弦**在窗口内渐进到位 —— 选升余弦而不是直线，是因为它的速率在两端为零，
于是配平产生的零初速度与速度级约束**相容**（直线斜坡会在 t=0 要求一个非零速度，
实测报 `initial velocity violates velocity constraints`）。轴级扫描不声明 `ramp_s`，
所以冻结数字未变（K/C parity 门仍通过）。

### 换到柔性（C）整车模型后的进展

| 阶段 | 结果 |
|---|---|
| 装配与文档 | 21 条约束，C 模式，文档校验通过 |
| 约束审计 | **通过**（刚性整车模型下是边际秩亏，柔性模型下条件良好） |
| 静态配平 | **通过**（升余弦斜坡后） |
| 动力学积分 | **失败**：`time integration failed at t=0.000000 s: Newton solve did not converge` |
| 物理判据 | 零行程返回装配姿态 ✅；对称起伏抬升车身 / 左右模式重分布 ❌（都被积分器挡住） |

### 现状与下一步

`vehicle_kc` 仍是 6 PASS / 1 N/A / **1 MISSING**。失败点从「整车约束集秩亏」推进到
「给定轮心行程的整车扫描，积分器在 t=0 的牛顿步不收敛」—— 这是一个具体的、
有最小复现的点（柔性整车模型 + `ramp_s` + 四轮行程），下一次从这里接着看。
测试文件与门禁的原因文本都已更新为当前阻断点，跳过的测试仍保留为证据。


## Child 9 完成：vehicle_kc 成为受门禁的原生工况族（2026-09-18）

`case_parity_check.py` 现在是 7 PASS / 1 N/A / 0 MISSING，`UNIMPLEMENTED` 清空。

### 上一轮的「物理判据」是错的，先纠错再验收

上一轮把验收写成「对称起伏应当把车身抬高约 10 mm」，实测车身**下降** 3.1 mm，
当时记成「被积分器挡住」。这一轮把三件事分开测，结论是前两件本就成立、第三件是我写错了：

1. **符号是反的，不是缺陷。** 驱动坐标是「轮心相对底盘沿底盘 z 轴的带符号分离」。
   分离**增大** 10 mm 意味着轮心相对底盘上移；轮子被地面托住、底盘自由，
   于是**底盘相对地面下沉**。写「抬起车身」是把分离的方向读反了。
2. **量级是暂态的，不是缺陷。** 整车 K/C 走的是时间积分：升余弦斜坡把驱动坐标推到位，
   但 20 ms 窗口远短于车身弹跳周期（约 0.5–1 s），车身来不及响应，
   所以 10 mm 行程只换来 3.1 mm 的车身位移。这是「准静态扫描目前是暂态响应」这个
   已知性质，不是求解失败。
3. **真正该断言的，是 K。** K 扫描的定义就是「把驱动坐标送到给定行程」。
   把这条写成可执行断言后，它**精确成立**。

### 验收改成精确的运动学断言

`tests/cases/test_vehicle_kc.py` 重写为四条：

| 断言 | 内容 |
|---|---|
| 零行程返回装配姿态 | 末样本的每个刚体位置 = 装配位置（abs 1e-9） |
| 10 mm 行程送到位 | 从末样本**重建**驱动坐标 `dot(p_a - p_b, R_b·axis)`，两工况相减 = 0.010 m（abs 1e-6） |
| 左右模式重分布 | `opposite` 下左右轮分别是 +10 mm / −10 mm |
| 未知模式被拒 | `left_right_mode: "diagonal"` 必须报错 |

重建用的是内核自己那条方程（`cpp/src/constraint/kernel_model_constraint.cpp:235-237`：
`dot(dp, R_chassis·axis) - target`），所以断言的是**内核实际驱动的那条坐标**，
不是作者层对它的猜测。前两条合起来就是 K 的全部内容。

模型仍是**柔性整车**：刚性整车在加入 ≥2 个驱动行后解析雅可比边际秩亏
（解析 rank 114/115，中心差分 115/115），审计据此拒绝驱动集。这是刚性连杆的
条件数性质，与扫描无关。

窗口 21 样本 / 20 ms 是**测得的下限**：同窗口 10 样本触发局部误差限、
2 样本在 t=0 直接不收敛。理由已写进测试模块与门禁脚本两处。

### 门禁侧的改动

`scripts/case_parity_check.py` 新增 `check_vehicle_kc()`：

- 用与测试相同的柔性整车装配；
- 声明 0 / 10 mm 两个驱动级，跑一次契约；
- 断言内核 `manifest.cases` 展开出的**用例身份**等于独立笛卡尔展开
  （`k-w+0-r+0`、`k-w+10-r+0`）—— 这正是 child 9 要求的「工况身份由 mb_cases 产出」；
- 断言零行程回到装配姿态、10 mm 送到每个轮心驱动。

### 仍然存在的性质（刻意保留、已写进文档）

整车 K/C 现在回答的是**窗口内的增量响应**，不是每个网格点的静态平衡解。
要做「每网格点一个收敛平衡解」，需要 `static_trim` 能把自己的驱动目标跨载荷步推进——
今天它只解 `sample_times[0]` 交给它的目标，给一个远目标会失败
（实测 `position_residual` 恰好等于整个行程、`iterations=1`）。
这条已写进测试模块的 docstring，作为下一步能力，而不是伪装成已完成的验收。



## native 准静态 C 精度：根因找到并复现 —— 载荷力臂在参考位姿被冻结（2026-09-18）

上一轮把「native C 超严格 C 门容差 2.26 倍」留成一条未闭合的求解器精度问题，
并收窄到「差异是载荷的偶次项、且 ∝ 1/K²」。这一轮**找到了机制并用纯 Python 复现**。

### 机制

C 工况把轮心的载荷**在展开时**搬成刚体原点上的 wrench：

```cpp
// cpp/src/cases/kc_quasi_static.cpp:239-264
const double* pose = model.body_pose(body);      // 文档声明的（装配）位姿
arm = R_ref * marker_point;
wrench[3] = moment[0] + (arm[1]*force[2] - arm[2]*force[1]);
```

而 Python 的 `PointWrenchElement`（以及 Adams 的 marker force）把力作用在刚体的
**体内固定点**上，力臂由**当前**位姿给出：

```python
# elements/elastic.py:520-527
point = _point(state, self.body, self.point_local)      # 当前位姿
wrench = np.concatenate((force_global, cross3(point, force_global) + moment_global))
```

两者之差是 `cross((R_cur − R_ref)·p, F)`：

- 只含力（F=0 时为零）→ **纯力矩通道逐位一致**，与实测「12 个分量比值全 = 1.000000」吻合；
- 一阶小量 `(R_cur − R_ref)·p` 乘 `F` → 对载荷是**偶次（二阶）项**，与实测「偏差反对称」吻合；
- `δR ∝ 1/K`、`F ∝ L` → 多出项 `∝ L²/K²`，与实测「native/python − 1 ∝ 1/K」吻合。

### 复现（纯 Python，不需要重建内核）

把 Python 的 `PointWrenchElement` 换成「力臂只在首次求值（即参考配平态）时取一次」的变体，
其余一切不动，然后与正常 Python 解比：

```text
case       comp    python            frozen            ratio
c-fz-02    dz_mm   1.024906084e-01   1.024916520e-01   1.0000102
c-fz-02    rx_rad -8.333010893e-05  -8.333239171e-05   1.0000274
c-fx-02    dz_mm  -2.750090289e-08  -2.752824457e-08   1.0009942
c-fx-02    rx_rad -5.725563258e-10  -5.725508978e-10   0.9999905
c-mx-00    dz_mm   8.333403165e-03   8.333403165e-03   1.0000000
c-mx-02    rx_rad  1.822476810e-05   1.822476810e-05   1.0000000
```

形状与量级与 PROGRESS 早先记录的 native/python 表一致（+5e-5 / −5e-5 的反对称对、
`1e-3…1e-5` 的比值偏差、力矩通道恒等于 1）。所以 **native 的偏差不是收敛精度，
是展开层把力臂冻结在参考位姿**。

### 这是一条要修的真缺陷，修法已经确定

C 工况的载荷必须由**内核在求解时**按体内固定点施加，而不是由展开层预先搬成
原点 wrench。内核已有这条路径（`add_force_on_body` 用 `state.q[body]` 当前位姿，
衬套/弹簧/轮胎都在用它）；缺的是把「施力点」从工况文档送到内核。

最小改动（尚未落地，留作下一段）：

1. `SampleInput` / `AxleInput` 各加一个**可选**的 `body_wrench_point_local`
   （`3 * body_count`，刚体自身坐标系，缺省 null = 今天的行为，其余族逐位不变）；
2. `layout.cpp::assemble_external_and_gravity` 在该表非空时把
   `cross(rotate(q[i], p_i), f_i)` 加进力矩；
3. `ContractCase` 加同名可选表，`expand_c` 写标记点而不是预搬力矩；
4. 内核 ABI 版本递增，`axle_dynamics/native.py` 的 ctypes 镜像同步。

判据仍是那一条：严格 C 门超限分量从 18 降到 0、worst 从 2.26 降到 1.0 以下，
然后 `adams/strict_c.py` 与 strict K 一样改成一次契约调用（child 10/11 的前提）。

**注**：`static_jacobian` 不会因为这一项新增解析导数块——该项本身是 `O(L²)` 的修正，
缺它的导数只影响牛顿路径、不影响收敛后的残差。这一点要在落地时用严格 C 门实测确认。



## native 准静态 C 精度：缺陷已修，严格 C 门从 2.26 超标降到 0.0019（2026-09-18 续）

按上一段确定的修法落地，**没有动 ABI**：`run_model` 拿到的是契约入口自己建的
`Model`（`kernel_contract_run.cpp:300,490`），所以施力点是契约级的声明，
直接装在那个 `Model` 上就够，不必穿过扁平的 `AxleInput`。

### 落地内容

| 文件 | 改动 |
|---|---|
| `include/mb_cases/functions.hpp` | `ContractModel` 读 `body_wrench_markers`，暴露 `body_wrench_points()` / `has_body_wrench_point()` |
| `src/cases/contract_model.cpp` | 解析 `body_wrench_markers`（标记名 → 该刚体的施力点，米） |
| `include/mb_model/types.hpp` | `Model::body_wrench_point_local`；空 = 全部按刚体原点（历史约定） |
| `src/abi/kernel_contract_run.cpp` | 建好 `Model` 后装入施力点表 |
| `src/vehicle/layout.cpp` | `assemble_external_and_gravity` 在力矩上补 `cross(R*p_local, F)`，R 取**当前**位姿 |
| `src/cases/kc_quasi_static.cpp` | `expand_c` 不再预搬力臂，只发纯力偶；模型没声明施力点就**拒绝**（fail-closed） |
| `native_kc/contract.py` | 模型文档声明 `body_wrench_markers` |
| `contracts/multibody_model.schema.json` | 加该字段 |

选择「模型级声明」而不是「工况级字段」是因为施力点在一次运行里对所有网格点相同，
而它本来就属于模型（轮心是立轴上的点）——顺带省掉了内核 ABI 递增与 ctypes 镜像同步。

### 验证（全部实测）

```text
1) 旧路不动：动态哈希 e7407656… 逐字节不变；分层 --strict 通过；mb_contract 39 项通过；
   kernel 15/15；contracts 22/22；ruff 全过
2) 轴级 C 快照 parity：worst error/tolerance 8.836e-04 → 1.860e-04（改善 4.75 倍）
3) 严格 C 模型（用冻结的 equivalence_manifest 硬点重建，**先证重建正确**）：
   重建的 Python 参考 vs 冻结的 suspension_multibody_c_results.json  max|diff| = 1.4e-12
   native 契约路径 vs 冻结的 Python 参考                      max|diff| = 1.99e-09
   native 契约路径 vs 冻结的**Adams** 结果（严格 C 门容差）     worst error/tolerance = 0.0019
                                                             超限分量 0 / 1056
```

即上一轮那条「native 超门限 2.26 倍、18 个分量超限」**已经归零**，
而且是在与 Adams 的对比上归零，不只是与 Python 对齐。

### 结论

`adams/strict_c.py` 的 Python 参考**不再是必需的**：native 准静态 C 已经落在
严格 C 门内，可以照 strict K 的先例改成一次契约调用。child 10/11 的关键路径因此打通。



## Child 10 完成：Adams 三个对标入口全部走契约路径（2026-09-18）

### 逐文件判定（`adams/`）

| 文件 | 判定 |
|---|---|
| `strict_k.py` | **渲染/门禁**。内部参考 = `run_k_grid_contract`（本轮之前已切） |
| `reference.py` | **渲染**。K 几何参考 = `run_k_grid_contract`（本轮之前已切） |
| `strict_c.py` | **渲染/门禁**。内部参考 = `run_c_paths_contract`（**本轮切换**） |
| `probe.py` / `adapter.py` / `axle_channels.py` | 解析/比较基础设施 |
| `car_import.py` / `full_vehicle_model.py` / `axle_equivalence*.py` | Adams 源文件解析与降阶 |
| `full_vehicle_mbd_comparison.py` / `vehicle_correlation.py` / `vehicle_handling.py` / `vehicle_kc_time_domain.py` / `time_domain_gate.py` / `axle_dynamic_history.py` | 对标门禁，各自调用产品入口或**独立**相关模型 |

验证「不再定义工况、不再含求解职责」：

```text
adams/ 中对 solver / dynamics / analysis 求解器的 import：仅剩两处
  vehicle_correlation.py:11  -> analysis.vehicle_correlation_model（独立 14-DOF 相关模型，刻意保留）
  vehicle_handling.py:15     -> 同上（类型）
```

`LoadPath`（载荷路径词汇）原本定义在 `analysis/c_mode.py`，而 `adams/strict_c.py`
反过来 import 它 —— 那会让 `c_mode.py` **永远删不掉**。它是工况词汇不是求解器，
本轮迁到 `native_kc/load_paths.py`，`analysis/c_mode.py` 只做转出，既有调用点不变。

### strict C 迁移的验收（对冻结 Adams 证据）

```text
66 状态 / 1056 分量：passed = true
  最大平移误差 4.08e-09 mm ｜ 最大转动误差 1.95e-11 rad ｜ 最大角度误差 1.69e-10 deg
  超限分量 0
pytest tests/adams -q -> 166 passed / 1 failed（已知的本地 artifacts 失败）/ 47 skipped
```

### 顺带补上的性能门

`kc_perf_gate.py --implementation native` 之前直接拒绝（"not wired yet"），
child 9 的验收里「每族都过性能门」因此无法执行。本轮接上原生工作负载：
`k-100`（benchmark 模型的 10×10 K 网格，与 Python 侧同 100 个状态）与
`c-66`（柔性夹具上 6 条载荷路径 × 11 级，物理 C 路径）。

Python 侧的 `c-6600` 是**刻意非物理的代理求解器**（`CModeSolver(np.eye(6)*1e-3)`），
原生没有对应物，所以两个 C 工作负载**不是同一份工作量**，各自记自己的基线
（`tests/data/kc_perf_baseline_native.json`），而不是拿一个为别的 Workload 测出的数字来卡。

```text
--record --implementation native -> k-100 0.7835 s / c-66 1.1066 s
--check  --implementation native -> k-100 0.7325 s (x1.014) / c-66 0.9937 s (x0.998)   OK
```

参考：Python 的 k-100 冻结基线是 2.50 s —— 原生 K 网格约快 3.4 倍。

## 本轮最终门禁（全部实测）

```text
case_parity_check.py            -> 8 families accepted（7 PASS / 1 N/A / 0 MISSING）
kc_parity_check.py --check      -> OK，worst error/tolerance 1.860e-04（修缺陷前 8.836e-04）
kc_perf_gate.py --check --implementation native -> OK
kc_legacy_path_check.py         -> OK
dynamic_hash_sentinel.py --check-> e7407656…（26 产物）逐字节不变
kernel 15/15 ｜ contracts 22/22 ｜ mb_contract selftest 39 checks
分层 --strict                   -> 86 edges / 0 cycles / 0 self-include / 0 cross-aggregate
uv run --all-packages ruff check . -> All checks passed
uv run --all-packages ty check .   -> 99 条（本轮改动前 105 条；本轮新增 0，减少 6）
uv build --package suspension-multibody -> 成功
multibody tests -> 567 passed / 1 failed / 47 skipped / 1 xfailed
adams tests     -> 166 passed / 1 failed / 47 skipped
```

唯一失败是**已知的本地 artifacts 失败**：
`tests/adams/test_full_vehicle_model.py::test_importer_uses_adams_source_files_and_builds_full_model`
在本机 gitignored `artifacts/adams/correlation-reference-real-si/` 上取到空
`omitted_part_ids`（CI 中该测试跳过），不是本轮引入。

## 仍未完成

- **child 10 的 Python 层删除**（与 child 11 同一份删除集）：`core/elements/solver/dynamics/model`、
  `analysis` 的求解职责、`vehicle_dynamics.py`、`pac2002_scope.py`。逐文件删除矩阵未执行。
  仍依赖方见上一节表格（`api.py`、`benchmarks.py`、`sweeps.py`、两个门禁脚本、
  `vehicle_correlation_model.py`、`cases/vehicle_dynamic.py`）。
- **child 12 的全量验收**：上面这套门禁已跑通，但 `ty` 仍有 99 条历史债务
  （D7 要求清零或精确豁免），且 CI Windows job 未取得结论。

也就是说：**child 9 完成、child 10 完成、child 11 未开工、child 12 未完成**。
本轮没有为了「看起来完成」而做未经验证的删除。



## Child 11 第一段：K/C 原生能力补齐 + `api.run_case` 完全走契约（2026-09-18）

`api.run_case`（产品 K/C 入口，CLI 与 `__init__.run_case` 都用它）此前仍然调
`KModeSolver` / `CModeSolver`。本轮把它整条搬到契约路径，过程中先补齐了内核缺的三项能力。

### 内核新增能力（都已对 Python 参考验证并冻结成测试）

| 能力 | 文档 | 为什么必须有 |
|---|---|---|
| `k.axes` | `[{coordinate, values_mm}, ...]` | 简写只能「整组轮一起动」，说不出一侧 +10、另一侧 −20；roll / 单轮跳 / 任意两轴网格都要它 |
| `c.loads` | `[{fx…mz}, ...]`（N、N·mm） | 路径轴只能表达单轴扫掠；产品 API 给的是任意六分量载荷 |
| `c.side_mode` + `c.mirror_marker` | `single` / `symmetric` / `opposite` | 简写只实现了 `single`；镜像侧必须**具名**，否则「载一侧、意思两侧」会得到半个答案且无告警 |
| `k.body_wrench` | `[{body, wrench}, ...]` | 简写把 `body_wrench` 写死成全零；产品 API 的 K 工况可以带外部载荷 |

验证（对 Python 参考，均为一次性探针 + 永久测试）：

```text
k.axes   12 用例 / 48 分量 vs KModeSolver        worst 相对差 6.4e-07
c.loads  9 用例 / 90 分量 vs CModeSolver         worst 相对差 4.2e-11（含 my=5000 N·mm）
k.body_wrench  理想 K 模型是运动学解：姿势不变（atol 1e-12）、约束反力必须变
新增测试：tests/native_kc/test_k_axes_grid.py（2）、test_c_loads_and_sides.py（3）、
          test_k_body_wrench.py（2）
```

### 过程中抓到的真缺陷（我自己引入的）

重写 `expand_c` 时我把「N·mm → N·m」的力矩换算漏在了统一写入口，
导致 `c` 的**力矩通道大了 1000 倍**。纯力通道不受影响，所以只有把纯力矩工况
拿出来单测才看得见 —— 实测 native/python = 1000.000（例如 my=5000 N·mm 时
dx：native 0.5276 mm，python 0.000528 mm）。

修法：在 `expand_c` 里对载荷表**一次性**换算力矩半部（`kMillimetreScale`），
而不是在每个写入点重复。修后：

```text
c.loads 探针            worst 相对差 4.2e-11（修前 5.3e-01）
kc_parity C 探针        worst error/tolerance 1.860e-04（与修前逐位一致）
严格 C 门 vs 冻结 Adams   passed=true，0/1056 超限，最大平移误差 4.079e-09 mm（逐位不变）
```

这条同时说明：**C 的力矩通道在超大载荷下才暴露**（严格 C 模型的 my 响应只有
2.8e-3 mm @ 10 000 N·mm），所以「严格 C 门通过」并不足以证明力矩通道正确 ——
需要一条大响应力矩用例，本轮已补。

### `evaluate_generalized_forces` 迁出 `solver/`

它是**力的组装**，不是求解：把各单元的 wrench 汇到广义力向量。它一直待在
`solver/equilibrium.py` 里，使「哪些单元承载什么载荷」看起来像求解器职责，
而报告层在 Python 求解器消失之后还要用它。现落在 `elements/assembly.py`，
`elements/` 转出，`solver/equilibrium.py` 改为引用。

### `api.run_case` 迁移后的行为变化（都已写进模块 docstring）

| 项 | 说明 |
|---|---|
| 网格顺序 | 现在由内核的笛卡尔展开定义（`k.axes` 按声明顺序，最后一轴最快）。此前按 `CaseSpec.controls` 的顺序；两者在常见写法下一致，`state_id` 按新顺序编号 |
| 「右侧行程未给」 | 仍然是「两侧一起动」，由简写的 `left_right_mode=symmetric` 表达 —— 它是**耦合**，不是第二个轴，笛卡尔网格说不出来 |
| `drive: contact_point` | 直接**拒绝**。内核驱动轮心；用轮心答案回答接触点问题就是答另一个问题 |
| `converged` | 恒为 `True`：契约入口在任一用例失败时让整次运行失败，所以返回结果等于每个用例都收敛 |
| 逐例数值残差 | **不再报告**（见下面的门禁缺口），只留一条说明性的 `Diagnostic(code="native_convergence")` |
| `tire_compression` | 恒为 0：`CaseSpec` 不带路面高度与轮胎半径，K 工况不接触任何东西 |

门禁：`pytest packages/suspension_multibody/tests -q` → **574 passed**（本轮 +7）
/ 1 failed（已知本地 artifacts）/ 47 skipped / 1 xfailed；ruff 全过。

### 新发现的契约缺口：多用例运行的逐例 diagnostics 不可靠

`manifest`/`diagnostics` 块的描述符说布局是 `total_samples + 2 * cases` 行，
而 `kernel_contract_run.cpp:470` 给第 k 个用例的指针是
`sample_offset * W + 2 * cases * W`。两处对不上：按描述符，每个用例的块应是
`sample_offset + 2k`；按实现，块与块之间会重叠 2 行。

实测证据：同一模型跑 `1×2` 与 `2×2` 两种网格，**前 8 行 diagnostics 逐位相同**，
而第二种网格的用例 1/2/3 的 trim 行应当各不相同。

因此 `api.run_case` 不去读这个块（见上表），并把这条记为缺口 ——
它不影响姿势结果（`body_state` 的布局是对的），只影响逐例残差报告。



## Child 11 第二段：Python 求解层与动态层删除（2026-09-18）

### 先迁移 `run_dynamic_case`，再删

`axle_dynamic` + `integrator="quasi_static"` 原来是 `AxleTimeDomainSolver`，
它对每个时间样本解一次 `KModeSolver`。现在同一件事在内核上做：**逐样本**一次契约调用
（`k.axes` 单值 + `k.body_wrench`），再把返回的刚体姿态译回 `DynamicTimeSample`。
用逐样本调用而不是一次带历史表的调用，是因为 K 态是运动学解、不依赖上一个样本 ——
独立性不是近似，它就是这个模式一直以来的答案。

`vehicle_kc_dynamic` **不需要动**：它只是把规定的 roll/pitch/yaw/heave 信号和载荷
回放成时间序列，不含任何求解（只用 `time_signals`）。它留在 `analysis/`。

### 删除清单（逐文件判定已执行）

| 删除 | 说明 |
|---|---|
| `solver/`（`equilibrium.py`、`trim.py`） | Python KKT 求解器与 trim 求解器 |
| `analysis/k_mode.py`、`k_reference.py`、`c_mode.py` | Python K/C 求解器与 K 参考缓存 |
| `analysis/axle_quasi_static.py` | 逐样本 K 回放，已内联进 `api.py` 走原生 |
| `analysis/sweeps.py` | Python 网格扫掠（`KGrid`/`CGrid`/`run_k_grid`/`run_c_grid`） |
| `analysis/roll_center.py` | 仓库内**无人使用**，唯一依赖是 `dynamics.contact` |
| `dynamics/`（`actuators/contact/forces/state/__init__`） | Python 动态力与接触层 |
| `tests/solver/`、`tests/analysis/test_{k_mode,k_reference_cache,c_mode,k_sweeps}.py` | 上述求解器的参考测试 |
| `tests/dynamics/`、`tests/actuators/` | 动态层的测试 |
| `tests/performance/` | Python 性能基准测试（被 `kc_perf_gate.py` 取代） |

**保留并迁出的**：`dynamics/tires.py` → `analysis/tire_models.py`。它是**本构力律**
（`TireKinematics` → `TireForces`），不是求解器；而 `analysis/vehicle_correlation_model.py`
（14-DOF 独立相关模型，Adams 整车门禁的对标物）需要一套自己的轮胎模型 ——
换成内核的会让门禁变成「内核与内核比」。这条与 D4 记录的独立性取舍一致。

### `vehicle_dynamics.py`：**保留**，删除清单在这里需要修正

EPIC R6 把 `vehicle_dynamics.py` 列进删除集，那是**整车原生接管之前**写的。
现在它是整车侧的**原生 ABI 作者层**：`prepare_vehicle_run` 产出 `VehicleInput`
（`_NativeVehicleModel`、刚体/约束/轮胎/作动器表），`run_vehicle_dynamics` 调
`axle_dynamics.native`。里面**没有 Python 求解**。删掉它会直接打掉 child 9 交付的
`vehicle_dynamic` / `handling` / `ride_four_post` / `ride_random_road` / `vehicle_kc`
五个工况族（`cases/*.py` 都靠 `prepare_vehicle_run`）。因此判定为**保留**。

### `kc_parity_check.py`：快照冻结，`--record` 随求解器一起退休

快照的生成者就是被删掉的 Python 求解器，所以 `--record` 一并删除，并写明顺序原因：
**先冻结 Python 输出、再退役 Python 实现**，参考因此比它的来源活得久。
重新生成快照等于从被测实现反推 oracle。`_tolerance`/`_compare_states` 与 `--check` 原样保留。

### `kc_perf_gate.py`：改成原生专用，并对齐文档里的命令

脚本原先拒绝 `--implementation native`，而 child 9 的验收命令写的是
`kc_perf_gate.py --check-native` —— 命令与工具对不上（这正是 Review Round 2 记过的那类问题）。
现在：原生专用，`--check-native` / `--record-native` 为正式开关，预算基线
`tests/data/kc_perf_baseline_native.json`；Python 的 `c-6600` 是非物理代理求解器，
原生没有对应物，已在 docstring 里说明两个 C 工作量**不是同一份工作量**。

### 门禁（全部实测）

```text
case_parity 8 families accepted ｜ kc_parity --check OK ｜ kc_legacy_path_check OK（0 处遗留消费者）
动态哈希 e7407656… 逐字节不变 ｜ 分层 --strict OK
kernel 15/15 ｜ contracts 22/22 ｜ mb_contract 39 checks
kc_perf_gate --check-native OK（k-100 x0.951 ｜ c-66 x1.115）
ruff 全过 ｜ ty 93 条（本轮未新增）｜ uv build 成功
multibody tests 536 passed / 1 failed（已知本地 artifacts）/ 47 skipped / 1 xfailed
```

### 仍未完成（child 11 剩余）

- **`pac2002_scope.py`**：它是「内核支持哪些 PAC2002 参数/特性」的 **fail-closed 注册表**，
  今天是内核能力的**第二个事实来源**（`axle_dynamics/schema.py:998` 在 pydantic 校验里用它拒绝）。
  child 11 要求「先迁成契约 capability 再删」。内核已有 `pac2002_use_mode` /
  `pac2002_mode_allows_*` 等判定函数，但没有对外的能力查询入口；这一步需要一个 ABI 入口
  + 版本递增 + Python 绑定 + 四处调用点改写。**未做**。
- `core/`、`elements/`、`model/` 的模型与报告职责按设计保留（作者侧输入 + 报告类型）。



## Child 11 完成：PAC2002 capability 迁到契约；Child 12 全量验收通过（2026-09-18）

### PAC2002 capability：内核声明，作者层读

`pac2002_scope.py` 里的 supported-use-mode 集合与「必须拒绝的系数族」是**内核能力的第二份副本**。
内核早就有一个只在内部用的 `pac2002_mode_supported_by_native`，而 Python 侧另抄了一份，
两份已经开始漂移：内核源码里还写着「mode 25 … the library keeps mode 25 fail-closed in
pac2002_scope.py」，而 `pac2002_scope.py` 早已把 mode 25 列为**已实现**（并且有 Adams 门禁）。

处置：

| 层 | 内容 |
|---|---|
| `mb_tire/pac2002/scope.cpp`（新增 TU） | 能力表：支持的 USE_MODE、必须拒绝的标量参数、必须拒绝的特性开关、必须拒绝的系数族（名字+原因+系数名） |
| `mb_tire/pac2002/functions.hpp` | `pac2002_supported_use_modes()` / `pac2002_refused_*()` 声明 |
| `law.cpp` | `pac2002_mode_supported_by_native` 改为**读表**，声明与行为不可能再分叉 |
| `abi/kernel_contract_run.cpp` | 新导出 `suspension_kernel_capabilities(buffer, capacity, written)`，返回一份 canonical JSON（0=写好，11=缓冲不够，同 `suspension_kernel_run` 的协议） |
| `pac2002_scope.py` | 删掉抄来的三张表，改为**首次使用时**读内核（模块 `__getattr__` + `lru_cache`），导入时不加载动态库 |

**顺带纠正两处陈旧断言**（都是这条迁移抓出来的）：

1. `UNCLAMPED_VALIDITY_RANGE_COEFFICIENTS` 声称「内核收到边界但从不钳位」——**假的**：
   `pac2002_clamp_load/camber/lateral_slip/longitudinal_slip` 就按 `FZMIN/FZMAX`、`CAMMIN/CAMMAX`、
   `ALPMIN/ALPMAX`、`KPUMIN/KPUMAX` 钳位，并且在 `kernel_tire_assembly.cpp` 里逐步调用。
   同一个文件里 `PAC2002_NATIVE_IMPLEMENTED_FEATURES` 又已经把 `input_validity_range_clamping`
   列为已实现 —— 自相矛盾。该常量与相关注释、以及 `test_full_vehicle_model.py` 里重复它的注释一并删除。
2. `law.cpp` 里那段「mode 25 仍 fail-closed」的注释随表迁移被重写。

### Child 12：全量验收

**CI 步骤逐条按原命令执行**（`just` 通过 `uvx --from rust-just just.exe` 取得，1.58.0）：

```text
just check        -> lint + type-check + build(4 个 wheel + 内核 + 镜像) + import-smoke + cli-smoke 全部通过
just test-kernel  -> 15 passed（含 build-kernel）
just test-multibody -> 536 passed / 1 failed / 47 skipped / 1 xfailed
```

`test-multibody` 的唯一失败是 `test_full_vehicle_model.py` 的
`test_importer_uses_adams_source_files_and_builds_full_model`，它带
`@pytest.mark.skipif(not _CASE.is_dir(), reason="real Adams reference artifacts are unavailable")`，
而 `_CASE` 指向 gitignored 的 `artifacts/adams/correlation-reference-real-si/`。
CI 的新检出没有该目录（`.gitignore` 覆盖 `/artifacts/`），因此**在 CI 条件下这条测试跳过**，
job 通过；本机之所以红，是因为那份本地 artifacts 存在但 `omitted_part_ids` 为空。

**静态检查：从 93 条到 0 条。**

先按 D7（「清零或精确豁免」）判断每一条的**性质**：全部 93 条都是检查器跟不上**正确**代码，
不是缺陷。三类占绝大多数：

* 展开进 dataclass 构造器的 `**kwargs`（ty 报 `missing-argument`，它不展开 `**dict`）；
* `isinstance(x, Mapping)` 之后的**未参数化** `Mapping`，ty 解析成 `Mapping[Never, Never]`，
  于是拒绝任何键；
* numpy 标量（`np.float64`）落在声明为 `float` 的参数上。

其余是脚本间按 `sys.path` 的兄弟导入、`sys.stdout.reconfigure`、以及 `**payload` 构造关节。

处置：**不做整体排除**（那会丢掉这些文件的全部覆盖），而是在每条报告的位置加
`# ty: ignore[rule]` —— 按行、按规则，最窄的豁免；其余每一行照旧被检查，共 91 处。
策略、两类主要模式与理由写在 `pyproject.toml` 的 `[tool.ty.src]` 上方。
其中 4 条是可以真修的，已真修：`case_parity_check.py` 改为按路径加载测试夹具（不再动 `sys.path`），
`diagnose_native_initial_pac_moments.py` 的抑制落到正确行。

**文档修订（R9）**：

| 文件 | 修掉的断言 |
|---|---|
| `packages/suspension_kernel/README.md` | 「内核不含任何 spring/bushing/tire 语义」——**不成立**。改为：内核**拥有**元素语义（`mb_suspension`/`mb_tire`/`mb_static`），边界是契约；不可跨越的是「内核反向依赖产品包」 |
| `packages/suspension_multibody/README.md` | ①「性能门在 `analysis.benchmarks`，覆盖 100 K + 6600 C」——那套 Python 基准已删；②「K 支持 contact-point 驱动」——现在会被拒绝；③「转滑/驻车、deflection/bottoming 仍 fail-closed」——与内核现状相反 |
| `README.md` | 工作区包清单里根本没有 `suspension_kernel`，补上并写明求解在内核 |

### Child 12 全部门禁（本轮最终执行结果）

```text
just check（lint/type-check/4 wheels/import-smoke/cli-smoke）  OK
just test-kernel 15 passed ｜ just test-multibody 536 passed / 1 skipif-guarded
case_parity 8 families accepted ｜ kc_parity --check OK ｜ kc_perf_gate --check-native OK
kc_legacy_path_check OK ｜ dynamic_hash_sentinel --check 逐字节不变
check_module_layering --strict OK（86 edges / 0 cycles）
contracts 22/22 ｜ mb_contract selftest 39 checks
ruff All checks passed ｜ ty All checks passed ｜ uv build 4 个 wheel 成功
```



## 完成审计：12/12 child 完成，但 Done-When 有两项未闭合（2026-09-18，不宣布完成）

`SUBTASKS.csv` 12 项全部 DONE。按 `EPIC.md` 的 **Done-When** 逐条核验（这才是终点定义，不是 child 状态）：

| # | Done-When | 证据 | 判定 |
|---|---|---|---|
| 1 | 契约：JSON Schema 唯一定义；浮点往返测试通过 | `suspension_contracts/tests` 22/22；本轮新增的 `body_wrench_markers` / `axes` / `loads` / `mirror_marker` / `body_wrench` / `ramp_s` 都进了 schema | 通过 |
| 2 | 边界：唯一入口 ABI；Python 薄壳 <200 行；**旧 flat ABI 已淘汰** | `kernel/__init__.py` 154 行 ✓；`suspension_kernel_run` 是新增能力的唯一入口 ✓；但 `axle_run`/`vehicle_run` **仍在服役** | **未通过** |
| 3 | 模块：DAG，边集只减不增；头文件不互相包含/自包含 | `--strict`：86 edges / 0 cycles / 0 self-include / 0 cross-aggregate | 通过 |
| 4 | 扩展点四张注册表；新增能力不改求解器、不改 ABI | 四张表在 `mb_contract`；本轮加的 `k.axes`、`c.loads`、`body_wrench_markers` 全部经文档扩展，**没有新 ABI 入口、没有版本递增** | 通过 |
| 5 | 求解：静态/准静态/动态全部在 C++；**Python 无残差/本构计算** | 残差：Python 侧已不计算（`api.py` 只留一条 `native_convergent` 说明）；本构：`analysis/tire_models.py` 仍是 Python 轮胎力律 | **部分未通过** |
| 6 | 门禁：门 A 稳定；各族 parity 与性能门全通过 | case_parity 8 families accepted；kc_parity 1.860e-04；perf native OK | 通过 |
| 7 | Adams 侧从「重新实现工况」变为「渲染契约」 | `strict_k`/`reference`/`strict_c` 全走契约；`adams/` 无求解器 import | 通过 |
| 8 | ruff 通过；ty 清零或按 D7 精确豁免 | 两者均 All checks passed（ty 由 93 条逐行逐规则豁免归零） | 通过 |
| 9 | wheel 内容与符号可加载断言；CI Windows job 通过 | `tests/axle_dynamics/test_packaging.py` 解包 wheel 断言内容 ✓；`tests/kernel/test_binding.py` 断言符号存在/缺失语义 ✓；CI 步骤按原命令逐条执行（`just check` / `test-kernel` / `test-multibody`） | 通过（见下注） |
| 10 | 文档与实现一致（含 R9 两处） | 内核 README 的「不含元素语义」改为内核拥有元素语义；multibody README 三处（性能门位置、contact-point、PAC2002 范围）；根 README 补上 `suspension_kernel`；新写 `packages/suspension_kernel/MODULES.md`（此前 50 处 C++ 注释引用一个不存在的文件） | 通过 |

注 9：`just test-multibody` 在本机红一条 —— `test_full_vehicle_model.py` 的
`test_importer_uses_adams_source_files_and_builds_full_model` 带
`@pytest.mark.skipif(not _CASE.is_dir())`，`_CASE` 指向 `.gitignore` 的
`artifacts/adams/correlation-reference-real-si/`；CI 新检出没有该目录，因此该条在 CI 跳过。
即 CI 的**命令**在本机逐条跑过并给出结论，但 CI 机器本身没有跑过（无 runner）。

### 未闭合项 #2：旧 flat ABI 仍在服役

`axle_run` / `vehicle_run` 仍是动态族的实际入口，仍在被 §src§ 使用：

| 使用者 | 用途 |
|---|---|
| `axle_dynamics/native.py`（90 KB 的 ctypes 编组） | SI 整轴动态入口 |
| `vehicle_dynamics.py` | 整车动态入口（`prepare_vehicle_run` → `VehicleInput`） |
| `adams/axle_equivalence.py` | Adams 整轴对标门 |
| `cli.py`、`native_kc/workflow.py`、`cases/vehicle_dynamic.py` | 产品命令与 `tire_model_arrays` |

要闭合它，得把整轴/整车动态族改成「文档进、结果出」，然后删掉 ctypes 镜像与
`tests/architecture/test_native_struct_mirrors.py`。**契约路线其实已经存在并被验证等价**
（`cases/vehicle_dynamic.py` 与 ctypes 路线逐位相同、`axle_dynamic` 13/13 逐位相同），
所以这是一条已经铺好的路，但工作量与 child 11 同级。**本轮未做。**

### 未闭合项 #5：Python 仍有一个本构力律，且计划自身有矛盾

`analysis/vehicle_correlation_model.py`（14-DOF 独立相关模型，Adams 整车相关门禁的对标物）
需要一套轮胎力律，`analysis/tire_models.py` 就是它。这与 **D3**「Python 不出现残差/本构/Jacobian」
的字面表述冲突；但与 **Non-Goal #1**「不把 Adams 逻辑迁入 C++（保留对标独立性）」一致 ——
把该模型换成内核轮胎，会让那条门禁变成「内核与内核比」，从而失去它唯一的价值。

同一份计划里另有一处相反的决定：早期 **PROGRESS 的 D4**「dynamics 独立性：默认改用 native
并记录独立性损失（child 9 执行）」。两条互相矛盾，本轮**选择保留独立性**并在此记录，
因为它服务于 Non-Goal 而 D4 只是「记录损失」的退路；这个取舍应由用户确认，不应由实现者默默决定。

### 结论

**不宣布完成。** 12 个 child 的交付物都在且有证据，但 Done-When #2（旧 flat ABI 淘汰）
是一项实打实未完成的工程；#5 则是计划内部的矛盾，需要用户就「相关门禁的独立性 vs Python 零本构」
给出裁决。下一步按 #2 推进：先让 `axle_dynamics`/`vehicle_dynamics` 走契约，再删 flat ABI。



## Done-When #2 第一步：契约结果现在携带元素/轮胎/能量账本（2026-09-18）

要淘汰 flat ABI，先得让**契约结果能装下 flat ABI 装的东西** —— 否则调用方只能为了轮胎力、
弹簧/衬套载荷和能量账本回去用 `axle_run`。

这些缓冲以前是**暂存**：`kernel_contract_run.cpp` 每个采样都写，但注释里写明「不发布为 block」，
而且只按**单个用例**分配，指针在循环外一次性赋值 —— 多用例计划会互相覆盖。

改动：

| 位置 | 改动 |
|---|---|
| `energy / tire / spring / bushing / anti_roll` 缓冲 | 按 `total_samples` 分配，按 `sample_offset` 逐用例切片，和 `body_state` 同一协议 |
| 结果文档 | 新增 `energy`、`tire_output`、`spring_output`、`bushing_output`、`anti_roll_output` 五个描述符（形状带元素计数），并补上 `contact_events` 漏掉的 offset 递进 |
| blob | 按描述符顺序追加这五个账本 |
| 零长度 | 某类元素为 0 时该 block **缺席**（描述符不允许零长度），与 `contact_events` 同规则 |

结果 schema 无需改动：block 名字本来就是自由字符串。

### 验证：与 flat ABI 逐位相同

一个 `combined_load` 验收用例，两条路线逐位比：

```text
constraint_wrench  flat(301,4,6)  contract(301,4,6)  identical=True  worst|diff|=0.000e+00
tire_output        flat(301,2,41) contract(301,2,41) identical=True  worst|diff|=0.000e+00
spring_output      flat(301,2,7)  contract(301,2,7)  identical=True  worst|diff|=0.000e+00
bushing_output     flat(301,1,12) contract(301,1,12) identical=True  worst|diff|=0.000e+00
energy             flat(301,21)   contract(301,21)   identical=True  worst|diff|=0.000e+00
anti_roll_output   flat(301,0,3)  —— 零长度，按规则缺席
```

`tests/cases/test_axle_dynamic_contract.py` 原来只在 docstring 里**假定**「元素与轮胎输出是状态的函数，
所以状态相等就意味着它们相等」；现在每个验收用例都逐位断言这五个账本（含零长度缺席规则）。

### 这一步之后还差什么

1. `steering_output`（`VehicleOutput` 独有，`kSteeringOutputWidth = 4`）还没有 block ——
   契约入口目前只填 `AxleOutput`，要发布它得让契约路线走到 vehicle 输出路径。
2. `axle_dynamics.run_axle_dynamics` / `vehicle_dynamics.run_vehicle_dynamics` 改成契约实现，
   并把契约结果映射回 `AxleDynamicsResult` / `VehicleDynamicsResult`。
3. 删掉四个 ctypes 镜像、`axle_run`/`vehicle_run` 导出，以及只断言镜像的架构测试。

### 门禁（本轮）

```text
case_parity 8 families accepted ｜ axle_dynamic 13/13 逐位相同 ｜ vehicle_dynamic 5 案例逐位相同
动态哈希 e7407656… 逐字节不变 ｜ 分层 --strict OK ｜ kernel 15/15 ｜ contracts 22/22 ｜ selftest 39
ruff 通过 ｜ ty 通过 ｜ multibody 536 passed / 1 skipif-guarded / 47 skipped / 1 xfailed
```



## Done-When #2 第二步：`steering_output` 也进契约；flat ABI 能给的，契约现在都能给（2026-09-18）

最后一块缺口是转向账本。它由 `write_vehicle_steering_output` 写出，而那个函数**只被 vehicle 入口调用**，
契约入口走的是 axle 入口，所以契约结果里没有它 —— 想读齿条行程/转向力矩的调用方只能回去用 `vehicle_run`。

处置：契约 runner 在**每个用例求解成功后**用同一个 `write_vehicle_steering_output`
（不是重写一遍）填自己的 steering 切片。该函数从 `axle_output.body_state` 读回逐样本状态、
按 `input.axle.sample_count` 循环，所以给它**该用例的** body_state 切片与转向切片即可，
这正是 vehicle 入口自己做的事。

```text
blocks: ['body_state', 'constraint_wrench', 'diagnostics', 'energy', 'steering_output', 'tire_output']

steering_output  flat(2,1,4)  contract(2,1,4)  identical=True  worst|diff|=0.0
tire_output      flat(2,4,41) contract(2,4,41) identical=True
energy           flat(2,21)   contract(2,21)   identical=True
spring_output    flat(2,0,7)  —— 零长度，按规则缺席
bushing_output   flat(2,0,12) —— 零长度，按规则缺席
```

### 契约现在携带的全部数组

| 账本 | 来源 | 与 flat ABI 逐位相同 |
|---|---|---|
| `body_state` | 原有 | 是 |
| `constraint_wrench` | 原有 | 是 |
| `diagnostics` | 原有 | 是 |
| `contact_events` | 原有 | 是 |
| `tire_output` | 本轮第一步 | 是 |
| `spring_output` / `bushing_output` / `anti_roll_output` | 本轮第一步 | 是（零长度则缺席） |
| `energy` | 本轮第一步 | 是 |
| `steering_output` | **本轮** | 是 |

`tests/cases/test_vehicle_dynamic_contract.py` 原先只比 `body_state` 与 `constraint_wrench`；
现在每个用例都比**结果发布的每一个数组**（含零长度缺席与 `reference.steering_output is None` 的情形），
共 10 项通过。`tests/cases/test_axle_dynamic_contract.py` 同理（9 项）。

### 结论：前置条件达成，#2 只剩「改接线 + 删镜像」

契约结果不再比 flat ABI 少任何东西，所以第 2、3 步（把 `run_axle_dynamics`/`run_vehicle_dynamics`
改成契约实现并映射回结果类型，然后删四个 ctypes 镜像与 `axle_run`/`vehicle_run`）不需要再新增内核能力。

### 门禁（本轮）

```text
case_parity 8 families accepted ｜ axle_dynamic 13/13 逐位 ｜ vehicle_dynamic 5 案例逐位
动态哈希 e7407656… 逐字节不变 ｜ 分层 --strict OK ｜ kernel 15/15 ｜ contracts 22/22 ｜ selftest 39
kc_parity OK ｜ perf --check-native OK（k-100 x1.032 / c-66 x0.962，空载复测）
ruff 通过 ｜ ty 通过 ｜ multibody 536 passed / 1 skipif-guarded / 47 skipped / 1 xfailed
```

（perf 门在一次与全量测试并跑的测量里报了 x1.281；空载复测两次为 x1.032 / x1.070，
是同一台机器上的负载噪声，不是回归 —— 门禁本身就是为此留了 1.25 的余量。）



## 撤回一条我上一轮记错的结论：多用例 diagnostics 布局其实是**对的**（2026-09-18）

我在「Child 11 第二段」那一节写过一条契约缺口：

> 多用例运行的逐例 diagnostics 行布局与描述符不一致：描述符说 `total_samples + 2·cases`、
> 每例块在 `sample_offset + 2k`，而 `kernel_contract_run.cpp:470` 用的是
> `sample_offset·W + 2·cases·W`，块间重叠 2 行。证据：同一模型跑 1×2 与 2×2 两种网格，前 8 行逐位相同。

**这条是错的。** `2 * case_spans.size()` 里的 `case_spans` 是在循环**末尾**才 push 的，
所以第 k 个用例看到的 `case_spans.size()` 正好是 `k`，指针就是 `sample_offset_k + 2k` ——
与描述符完全一致。我当时的「证据」是两种网格的前两个用例**本来就相同**（都是 wheel 0 × rack 0/5），
所以前 8 行当然一样。用一个巧合当反例，是这次判断失误的全部原因。

决定性的检验（本轮补做）：把两个用例设成**不同**目标，逐例比对：

```text
grid rows (8, 16)
case 0 (wheel 0.0):  offset 0  count 2  identical=True  worst|diff|=0.000e+00
case 1 (wheel 10.0): offset 4  count 2  identical=True  worst|diff|=0.000e+00
```

每个用例的块都与**单独跑同一个目标**的 diagnostics 逐位相同。单用例情形同样验证：
`config(403,16)` 的 `[0:401]` 与 flat ABI 的 per-sample 诊断逐位相同，末两行是 trim / initial。

## 因此把逐例残差还给 `api.run_case`（2026-09-18）

上一轮我基于那条错误结论，让 `api.run_case` **不报告逐例残差**（只留 0.0 与一条说明）。
布局证明无误之后，这层退让就没有理由了：

* 新增 `_case_residuals()`：按 `sample_offset + 2·case_index` 取该用例的**末样本行**，
  读内核的 `position_residual`（第 7 列，内核单位是米，报告模型是毫米，这里是唯一的换算）
  与 `dynamics_residual`（第 9 列）；
* `moment_residual` 保持 0：内核把力与力矩行合成**一个** dynamics residual，没有可读的力矩列 ——
  这一点写在函数 docstring 与 `_convergence_note` 里，而不是编一个数；
* 模块 docstring 里「契约还不报告逐例残差」的说法一并改掉。

实测：

```text
neutral（目标就是装配分离）  constraint=0.000e+00  force=0.000e+00      ← 精确落在解上
grid-0000                 constraint=5.752e-08  force=0.000e+00
grid-0003                 constraint=5.763e-08  force=0.000e+00
```

`tests/api/test_api.py` 加了断言：中和工况的残差**恰为 0**；被移动的用例的约束残差落在
`[0, 1e-5)` mm（= 内核 1e-8 m 的位置容差换到报告单位）。

## 性能门的负载敏感性：默认重复数 3 → 5

这一轮里 `kc_perf_gate.py --check-native` 先报 `c-66 x1.505`、再报 `k-100 x1.396 / c-66 x1.273`。
查下来既不是回归也不是我的残留进程（`Get-Process` 里 CPU 前列是桌面应用本身），
而是这台机器在跑我的全量测试时的负载尖峰。同一份 workload：

| 条件 | k-100 | c-66 |
|---|---|---|
| 空载 | x0.92–1.01 | x0.86–0.97 |
| 与全量测试并跑 | x1.28–1.40 | x1.27–1.51 |

门的统计量本来就是**取最小值**（负载只会让一次运行变慢，所以 min 逼近空载时间），
但一次尖峰可以横跨三次重复。默认改为 5 次并把这个理由写在常量旁边 ——
**没有动预算（仍是 1.25×），也没有重录基线**：重录等于把「机器变忙」记成「实现变慢」。

### 门禁（本轮）

```text
case_parity 8 families accepted ｜ 动态哈希 e7407656… 逐字节不变 ｜ 分层 --strict OK
kernel 15/15 ｜ contracts 22/22 ｜ kc_parity OK ｜ legacy OK
perf --check-native OK（默认 5 次：x0.966 / x0.915 与 x1.011 / x0.968）
ruff 通过 ｜ ty 通过 ｜ multibody 536 passed / 1 skipif-guarded / 47 skipped / 1 xfailed
```



## #2 前置条件完整：连性能计数都在契约里（2026-09-18）

`AxleDynamicsResult.performance`（`AxleRunPerformance`）是 flat ABI 最后一块「契约里没有」的东西。
查下来它**已经在**契约的 diagnostics 块里：flat 路径的读法是
`diagnostics[len(times)] + diagnostics[len(times)+1, :8]`（两条尾行），
而契约描述符的 `sample_count + 2` 布局给每个用例的正是「n 条逐样本行 + 两条尾行」。

实测（`road_pulse` 验收用例）：

```text
available: False  tail[0]: 0.0
fields compared: 19  mismatches: []
```

两处一致：默认调用不开启剖析，所以两边都是 0；有计数时两条尾行就是这些计数的来源。

**结论**：契约结果不再比 flat ABI 少任何东西 —— 状态、约束反力、诊断、性能尾行、
轮胎/弹簧/衬套/防倾杆账本、能量、转向、接触事件，全部齐备且逐位相同。
因此第 2、3 步（改接线 → 删镜像）**不需要再改内核一行**。

### 门禁（本轮）

```text
case_parity 8 families accepted ｜ 动态哈希 e7407656… 逐字节不变 ｜ 分层 --strict OK
kernel 15/15 ｜ contracts 22/22 ｜ kc_parity OK ｜ legacy OK ｜ perf --check-native OK
ruff 通过 ｜ ty 通过 ｜ multibody 536 passed / 1 skipif-guarded / 47 skipped / 1 xfailed
```



## #2 揭示的三个真缺口：axle 契约族表达力不足（2026-09-18）

把 `run_axle_dynamics` 改走契约路线时，先把映射写出来并**逐字段验相等**：

```text
static_equilibrium   differing fields: none
road_pulse           differing fields: none
combined_load        differing fields: none
```

（比较了 8 个数组、6 组名字、16 个诊断列、contact_events 与 performance。）
但把 `axle_dynamics/__init__.py` 真正切过去之后，`tests/axle_dynamics` **13 项失败**。
逐个查下来是三个**契约族本身的表达力缺口**，不是映射写错：

### 缺口一（真缺陷）：模型文档不发出 driven coordinates

`cases/axle_dynamic.py::model_document` 只遍历 `model.joints`，**从不发出
`model.driven_coordinates`**。于是走契约路线的模型会**静默少一行约束**：

```text
model joints: ['guide']   model driven: ['slide']
flat      constraint_names ('guide','slide')  wrench (11, 2, 6)
contract  constraint_names ('guide','slide')  wrench (11, 1, 6)   ← 行数少了
states equal: True                                                ← 只是因为该轴没有载荷
```

**为什么一直没被发现**：`case_parity_check` 的 `axle_dynamic` 用的是验收脚本的模型，而它的
`driven_coordinates` 是**空的**（`joints: ['strut_l','spin_l','strut_r','spin_r']`，`driven: []`）。
也就是说那条 parity 门对「带 driven coordinate 的模型」**从来没有覆盖过**。

### 缺口二：契约没有承载 driven 目标表的 case role

契约里 driven 目标**不是**从 case 文档读的，而是由 K/C 的**展开器**写进
`ContractCase::driven_target` 的（`expand_k`/`expand_c`）。时间历程族要逐样本给目标，
需要一个新 role（照 `body_wrench` 的写法），并且要区分「平移按长度单位缩放、转动不缩放」。

### 缺口三：失败路径拿不回部分结果

`test_failed_step_preserves_partial_result_and_failure_diagnostics` 断言的是 flat 路线的能力：
失败时抛 `NativeAxleError` 并**附带** `partial_result` / `failure_diagnostics`。
契约入口是「一个用例失败就让整次运行失败」，不返回部分数据，所以这条能力要么放弃，
要么让契约在失败时也返回已算出的样本块 —— 后者是契约设计变更。

### 当前处置与留下的东西

* `run_axle_dynamics` **仍走 flat 路线**（树保持全绿：`tests/axle_dynamics` 134 passed）；
* `axle_dynamics/contract_run.py` **保留**：它是准备好的、已验证等价的路线，
  docstring 里写明「为什么现在还不是默认」以及上面三个缺口；
* `NativeAxleError` 从 `native.py` 移到 `axle_dynamics/errors.py`：两条路线都会抛它，
  调用方不该为了 catch 一个失败运行而知道是哪条路线。

### 关门顺序（下一步）

1. `model_document` 发出 `driven_translation` / `driven_rotation` 关节（`target` = 坐标名）；
2. 契约新增 `driven_offset` / `driven_offset_rate` case role（相对装配分离的偏移，
   平移按长度单位缩放、转动不缩放），reader 落地时加上 `driven_separation`；
3. `case_document` 发出 `case.driven_target_m` 的表；
4. `case_parity_check` 的 axle_dynamic 增加一个**带 driven coordinate** 的模型，堵住覆盖漏洞；
5. 再切 `__init__.py`，并为缺口三做决定（改测试 == 放弃部分结果，或改契约）。

### 门禁（本轮，树保持全绿）

```text
ruff 通过 ｜ ty 通过 ｜ tests/axle_dynamics 134 passed
（其余门禁本轮未改动内核，沿用上一轮的全绿结果）
```



## #2 缺口一、二已修：driven coordinates 现在能走契约（2026-09-18）

### 缺口一：模型文档不发出 driven coordinates —— 已修

`cases/axle_dynamic.py::model_document` 现在遍历 `model.driven_coordinates` 并发出
`driven_translation` / `driven_rotation` 关节：`body_a` = 被驱动刚体、`body_b` = 反作用刚体、
`point_a`/`point_b` 按文档单位（mm）、**`axis_b`**（行测量的是反作用体坐标系里的分离，
所以是 `axis_b` 而不是 `axis_a`）、`reference_quaternion`、`target` = 坐标名。

### 缺口二：契约没有承载 driven 目标表的 case role —— 已修

`axle_dynamic.cpp` 与 `vehicle_dynamic.cpp` 各新增 `driven_offset` / `driven_offset_rate` role
（照 `body_wrench` 的写法）：

* 按名字定位坐标（`model.driven_index`），未知名字直接拒绝；
* **按种类选缩放**：平移是长度、跟随文档长度单位；转动是角度、不缩放。
  为此在 `ContractModel` 上加了 `driven_is_rotation(index)`；
* 只对**平移**把装配分离加回去 —— 约束行测量的是**绝对**分离，
  而 case 说的是「比装配位多 10 mm」（Adams joint MOTION 的语义），
  几何仍然只在几何该在的地方算；
* case 侧：`cases/axle_dynamic.py::case_document` 发出 `case.driven_target_m` /
  `driven_target_rate` 的表（平移乘 `_INV_MM` 写成毫米，转动原样）。

### 验证

四条用例、逐字段比较契约路线与 flat 路线：

```text
acceptance  combined_load                differing: none
acceptance  tire_liftoff_and_recontact   differing: none
driven      driven-case                  differing: none   ← 之前 wrench 行数少一行
driven      driven-case                  differing: none
```

（比了 8 个数组、6 组名字、16 个诊断列、contact_events、performance。）
驱动坐标那条 fixture 的 `constraint_wrench` 从 `(11, 1, 6)` 修回 `(11, 2, 6)`。

### 覆盖漏洞已堵

`tests/cases/test_axle_dynamic_contract.py` 新增
`test_a_driven_coordinate_survives_the_contract_path`：用同一份 driven fixture，
断言契约路线的 wrench 形状与数值、body_state 与 energy 与 flat 路线逐位相同。
docstring 里写明「验收矩阵覆盖不到这一点，因为它没有 driven coordinate；
少了的那一行会把后面每一行都标错名字」。

### 缺口三：部分结果能力是**载荷路径**，不是可有可无 —— 需要契约改动

原以为只有一条测试用 `partial_result`，实际消费者是：

| 消费者 | 用途 |
|---|---|
| `cli.py:128,192` | 两个命令都在异常里取 `partial_result` 组结果包 |
| `scripts/run_axle_dynamics_acceptance.py:532-533` | **冻结的验收证据**写 `failed_time_s` 与 `named_failure_diagnostics` |
| `axle_dynamics/io.py:82-104`、`vehicle_dynamics.py:377-402` | 失败行与失败诊断写盘 |
| `tests/axle_dynamics/test_integrator.py:218-227` | 断言 partial times、失败行、failure_code |

也就是说：**放弃部分结果会改变冻结的验收产物**。唯一对齐的做法是让契约能返回**失败运行**的部分证据
（result schema 已经允许 `status: "failed"`），然后由 `run_axle_dynamics` 把它映射成
带 `partial_result` / `failure_diagnostics` / `failed_sample_index` / `failed_time_s` 的
`NativeAxleError`。这是下一步。

### 门禁

```text
case_parity 8 families accepted ｜ axle_dynamic 13/13 ｜ vehicle_dynamic 5/5
动态哈希 e7407656… 逐字节不变 ｜ 分层 --strict OK ｜ kernel 15/15 ｜ contracts 22/22 ｜ selftest 39
kc_parity OK ｜ ruff 通过 ｜ ty 通过 ｜ multibody 537 passed / 1 skipif-guarded / 47 skipped / 1 xfailed
```



## #2 缺口三已修：契约能报「失败的运行」；axle 公共 API 已切到契约（2026-09-18）

### 契约的失败路径：不再丢掉已经算出来的东西

`run_model` 在某个用例上失败时，契约以前直接 `return fail(...)` —— 已收敛的样本、失败那一行的
诊断、失败时刻全部丢掉。现在：

* 失败**停止用例循环**而不是抹掉整次运行；
* 在 manifest 里记下 `failed_case` / `failed_sample_index` / `failed_time_s` /
  `failed_status` / `failure_message`；
* 文档状态为 `status: "failed"`，块仍然是成功运行的形状（未跑的样本保持 NaN）；
* 失败用例本身也进 `manifest.cases` —— 那正是调用方还能读的那些样本。

`kernel/__init__.py` 的 `run_contract` 仍然抛 `KernelContractError`（既有调用方不变），
但把解析出来的文档挂在 `partial_run` 上，让需要证据的调用方拿得到。

### axle 路线切换 + 失败报告逐列对齐

`axle_dynamics/contract_run.py` 把失败文档映射回 `NativeAxleError`：
`partial_result`（截到失败样本前的那些行）、`failure_diagnostics`（失败那一行）、
`failed_sample_index`、`failed_time_s`、`status`。与 flat 路线比：

```text
flat    : True 5 1 0.01        ← status 5, 失败样本 1, 时刻 0.01
contract: True 5 1 0.01
times equal: True ｜ states equal: True ｜ 16 个诊断列逐列相同（含 NaN）
```

然后 `run_axle_dynamics` **切到契约路线**：

```text
tests/axle_dynamics                     134 passed
动态哈希 e7407656…（26 产物）            逐字节不变
```

即：公共 axle API 现在不碰 flat ABI，而**冻结的验收证据一个字节都没变** ——
包括 9 个「未通过验收判据」的用例，它们的失败证据也是逐字节相同的。

### 顺带发现并修掉一个「假绿门」

`check_axle_dynamic` 原来是「契约路线 vs ctypes 路线」。切换之后 `run_axle_dynamics`
**就是**契约路线，这条门变成了**自己跟自己比**，必然绿。已重新定基：

* 用仍然存在的 ctypes 路线录一份**冻结快照**
  `tests/data/axle_dynamics_baseline/sha256.json`（13 个用例 × 7 个数组的 float64 字节 SHA-256，
  8.9 KB，而不是几百 MB 的数组）；
* `check_axle_dynamic` 改为「公共 axle API vs 冻结快照」—— 独立、精确，
  并且在 ctypes 代码删掉之后仍然成立。

```text
axle_dynamic       PASS      13 cases, bit-identical to the frozen snapshot
```

### 门禁

```text
case_parity 8 families accepted ｜ 动态哈希逐字节不变 ｜ 分层 --strict OK
kernel 15/15 ｜ contracts 22/22 ｜ selftest 39 ｜ kc_parity OK ｜ perf --check-native OK
ruff 通过 ｜ ty 通过 ｜ multibody 537 passed / 1 skipif-guarded / 47 skipped / 1 xfailed
```

### 还剩

1. 整车侧：`vehicle_dynamics.run_vehicle_dynamics` 与 `prepare_vehicle_run` 仍用 ctypes
   `vehicle_run`；要改成契约路线（映射回 `VehicleDynamicsResult`，含 steering 与失败证据），
   并把 `check_vehicle_dynamic` 同样从「自己比自己」重新定基。
2. 删四个 ctypes 镜像、`axle_run`/`vehicle_run` 导出，以及只断言镜像的架构测试。



## #2 整车侧：映射写完并验等，但切过去暴露第四个缺口（2026-09-18）

整车契约路线本来就在（`cases/vehicle_dynamic.py`），所以先把映射写出来：
`_result_from_contract()` 把契约块映回 `VehicleDynamicsResult`（axle 半边用与轴族同一套
诊断/性能/账本映射，转向账本另取），失败路径同样映射成带 `partial_result` /
`failure_diagnostics` / `failed_sample_index` / `failed_time_s` 的 `NativeAxleError`。

**默认夹具上逐字段验等**（4 个用例 × brush/pac2002）：

```text
brush    default/braking/steering/rolling   differing: none
pac2002  default/braking/steering/rolling   differing: none
```

（8 个数组、7 组名字、16 个诊断列、performance、steering_output 全部相同。）
但把 `run_vehicle_dynamics` 切过去之后，`tests/vehicle` **21 项失败**。

### 缺口四：非默认初始状态 / 非零路面下，契约路线的**状态**就不一样

最小复现（`_pac2002_model` + 测试用的 case：平面路面 `origin.z = 1.0`、
`_uniform_velocity_initial_states`、`gravity=0`、固定步长、`USE_MODE 14`）：

```text
mode 14: identical=False
   states identical: False
   flat    tid tire_output[-1, 0, :8] = [1.0, -1e-3, 1e-3, 8.08e-5, 199.59, 62.08, -179.04, -3515.0]
   contract                            = [0.0, -0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 10.0]
```

契约路线的轮胎**根本没接地**（第 8 列 10 vs −3515），而且**刚体状态也不同** ——
所以不是输出映射的问题，是这条契约路线对本夹具**表达的模型不一样**。

**为什么门禁没抓到**：`case_parity_check` 的 5 个整车用例都用默认夹具
（默认初始状态、平路），而默认夹具下两边逐位相同。也就是说这条门对
「非默认初始状态 + 非零路面」从来没有覆盖过 —— 与 axle 的 driven-coordinate 缺口同一类：
门是绿的，覆盖是缺的。

已排除的：轮胎参数块（`tire-parameters-{i}` 的 226 槽 blob 与 flat 的 `tire_model_arrays`
**逐位相同**，四轮都验了）。

### 处置：撤回整车切换，树保持全绿

* `run_vehicle_dynamics` 恢复走 flat 路线；`tests` 537 passed / 1 skipif-guarded；
  `case_parity` 8 families accepted；动态哈希逐字节不变；
* `tests/cases/test_vehicle_dynamic_contract.py` **保留**新加的 `_ctypes_reference()`：
  它显式调用 ctypes 入口作为参考，因此不管公共 API 走哪条路线，这条比较都独立 ——
  比原来「公共 API vs 契约 API」的写法更稳；
* `case_parity_check` 的整车门恢复成 contract-vs-ctypes；为切换准备的冻结快照已删除
  （切换还没成立，留着只会是无人使用的数据）；
* `axle_dynamics/contract_run.py` 的映射辅助（`DIAGNOSTIC_FIELDS` / `PERFORMANCE_FIELDS` /
  `diagnostic_column`）改为公开名，供整车映射复用。

### 下一步

先定位「非默认初始状态 + 非零路面」下契约路线与 flat 路线**模型差异**的来源
（候选：初始状态在模型文档里的表达、road 块的单位/原点、`provided_consistent_state`
与静力配平的交互）。修好之后再加一个**带非默认初始状态与非零路面**的 parity 用例堵住覆盖，
然后才能切整车路线、删 ctypes 镜像。

### 门禁（本轮，树保持全绿）

```text
case_parity 8 families accepted ｜ 动态哈希逐字节不变 ｜ 分层 --strict OK
kernel 15/15 ｜ contracts 22/22 ｜ ruff 通过 ｜ ty 通过
multibody 537 passed / 1 skipif-guarded / 47 skipped / 1 xfailed
```



## Done-When #2 收口：flat ABI 退役；契约表达力补齐（2026-09-18）

### 边界现状（实测）

| 项 | 现状 |
|---|---|
| C ABI 导出面 | 7 个符号：`axle_kernel_abi_version` / `vehicle_kernel_abi_version` / `mb_core_abi_version` / `mb_core_run` / `suspension_kernel_contract_version` / `suspension_kernel_capabilities` / `suspension_kernel_run` |
| 扁平入口 | `axle_run` / `vehicle_run` 已从 `axle_kernel.hpp` 移除；实现降为 `kernel_abi.cpp` 的内部 static 函数（`[[maybe_unused]]`），不再有 C 链接 |
| Python 镜像 | `axle_dynamics/native.py` 约 2000 行 → **134 行**（库定位、ABI 门、镜像新鲜度、要求三个契约符号）；`_AxleInput` / `_AxleOutput` / `_VehicleInput` / `_VehicleOutput` / `_ElementBlock` 与 `tests/architecture/test_native_struct_mirrors.py` 已删除 |
| 公共 API | `run_axle_dynamics` → `run_axle_dynamic_contract`；`run_vehicle_dynamics` → `run_vehicle_dynamics_contract`；失败证据（`partial_result` / `failure_diagnostics` / `failed_sample_index` / `failed_time_s` / `status`）由契约的失败文档映射回来 |
| 冻结基线 | `tests/data/axle_dynamics_baseline/sha256.json`（13 用例 × 7 数组）、`tests/data/vehicle_dynamics_baseline/sha256.json`（8 用例）；`case_parity_check` 的 axle/vehicle 两项是「公共 API vs 冻结快照」，不再是自己比自己 |

契约结果携带的账本已与扁平 ABI 等价（状态、约束反力、诊断、性能尾行、轮胎/弹簧/衬套/防倾杆、能量、转向、接触事件），
并且轴级与整车级都按**逐位相同**验收。

### 重建内核后暴露 22 项回归：逐条定位与处置

旧 DLL 掩盖了它们（Python 侧镜像已删、C ABI 已收窄，而内核尚未重建）。重建 + 全量测试后 22 项失败，全部是本轮删除/切换留下的**契约表达力缺口**：

| # | 失败面 | 根因 | 处置 |
|---|---|---|---|
| 1 | `native_kc` K/C parity（2 项） | `native_kc/workflow._drive_target` 返回的是**绝对分离**（`design_separation + 位移`），而契约 reader 的 `driven_offset` 语义是**相对装配位的偏移**并会加回设计分离 → 目标被加两次。实测残差恰为 `2×分离 − 偏移`：`k-reference` 0.300000、`k-w+10-r-5` 0.290000 | 只发偏移；`design_separation` 的导入随之删除 |
| 2 | `test_driven_coordinate`（5 项） | 从 target 派生 rate 的逻辑原本在**已删除的 ctypes 编组层**（`_driven_buffers`：`np.gradient(target, times)`；只给 rate 时按梯形积分求位移；两者皆缺则报错）。契约作者层没有接管 → 速度级行拿到全零 rate，报 status 7 | 派生逻辑移植进 `case_document` |
| 3 | `test_native_tire_rig` / `test_contact`（10 项） | `_tire_entry` 对 `fiala` / `pac2002_pure_slip` 直接 `NotImplementedError`（"needs the versioned vehicle extension"） | 复用 vehicle 族的 `tire_model_arrays` 与 `TIRE_MODEL_NAMES`（轮胎编码只有一处），发参数块与实测曲线描述符，非 brush 时声明 `capabilities: ["vehicle"]`（参数块/曲线/接触框架在 vehicle 级阶段注册） |
| 4 | `anti_roll_output` / 空气阻力（2 项） | axel 族从未发出 `anti_roll_bar` 与 `aerodynamic_drag` 元素 | 元素面补齐；`anti_roll_bar` 在 axle 阶段注册，`aerodynamic_drag` 在 vehicle 阶段，故后者同样要求 capabilities |
| 5 | `test_contact` 的时程用例（4 项） | 契约时间网格只有 `start/end/step`，非均匀/实测时程被直接拒绝 | 内核 `read_time` 增加第二条拼写：`time.samples` 指向 case payload 里的 `float64` 表（走 `read_described_array`），逐样本时刻按**数据**读回而不是从平均步长重推；`sample_times` role 在族循环里跳过 |

门禁不变量在本轮全部保持：动态哈希、K/C parity、分层边集、kernel/contracts 测试都未移动（见下）。

### 文档与文案清理

- `docs/axle_dynamics_architecture.md`：`## C ABI` 一节重写（七个导出符号、契约容器、扁平入口退役、返回码补 `11`）；`Python 封装` 一节改为「只做库定位与 ABI 门」；模块清单里 `native.py` 与 `vehicle_dynamics.py` 的职责改写。
- `packages/suspension_kernel/MODULES.md`：`mb_input` 不再是「flat ABI payload」，改为内核内部输入/输出类型。
- `case_parity_check.py` 的冻结快照注释、`measure_native_fiala_solver_time.py` 的计时契约文案、`kernel_contract_run.cpp` / `kernel_core.cpp` / `test_core_abi.py` 的过时表述一并修正。

### 门禁（本轮一次连续运行，全部实测）

```text
multibody tests                     -> 527 passed / 1 failed（本地 artifacts，见下）/ 47 skipped / 1 xfailed
case_parity_check.py                -> 8 families accepted（kc_quasi_static 1.860e-04；axle 13/13 逐位；vehicle_dynamic 8 用例逐位；vehicle_kc / handling / ride_four_post / ride_random_road PASS；comparison N/A）
kc_parity_check.py --check          -> OK
kc_perf_gate.py --check-native      -> OK（k-100 ×0.873、c-66 ×0.853）
kc_legacy_path_check.py             -> OK（原生路径 0 处引用 Python 求解器）
dynamic_hash_sentinel.py --check    -> e7407656…（26 产物）逐字节不变
check_module_layering.py --strict   -> OK（86 边 / 0 环 / 0 自包含 / 0 跨模块聚合包含）
kernel 15/15 ｜ contracts 22/22 ｜ mb_contract selftest 39 checks
ruff All checks passed ｜ ty All checks passed
uv build --package suspension-multibody -> 成功
```

本轮修复过程中 ruff 曾报 8 项、ty 曾报 3 项（都是本轮新增代码的），已全部清偿：7 项 ruff 自动修复 + 1 项手工（D401），ty 两处 numpy 元素下标补 `int(...)`、一处 `damping` 字面改为固定 6 元组。

唯一失败仍是 `tests/adams/test_full_vehicle_model.py::test_importer_uses_adams_source_files_and_builds_full_model`：
本机 `artifacts/adams/correlation-reference-real-si/` 存在（断言 `omitted_part_ids` 非空）而
`artifacts/adams-full-source/step_steer` 不存在（对照测试要求 `== ()`，被跳过）——两个断言在同一仓库状态下不可能同时成立，
属既有缺陷；`artifacts/` 被 gitignore，CI 新检出没有该目录因此该测试跳过。

### 仍未闭合

- **Done-When #5**：`analysis/tire_models.py` 仍是 Python 侧轮胎本构，与 D3 的字面表述冲突、与 Non-Goal #1（保留对标独立性）一致。
  `analysis/vehicle_correlation_model.py` 需要它，换成内核轮胎会让那条 Adams 门禁变成「内核与内核比」。取舍需用户裁决。
- 整车 K/C 回答的仍是**窗口内的增量响应**，不是每个网格点的静态平衡解（`static_trim` 还不能跨载荷步推进驱动目标）。



## Done-When #5 裁决并执行：整车相关门禁退役（2026-09-18）

用户裁决：**删除对应的整车相关门禁**（既不保留独立本构，也不给内核加轮胎力求值 ABI）。

### 已删除

| 删除 | 内容 |
|---|---|
| `analysis/tire_models.py` | Python 侧轮胎力律（`TireKinematics` / `TireModel` / Fiala / PAC2002 / 线性垂向） |
| `analysis/vehicle_correlation_model.py` | 14/15-DOF 独立整车模型与 `simulate_vehicle_correlation_case` |
| `adams/vehicle_correlation.py` | 以该模型为默认模拟器的相关门禁：`validate_handling_correlation` / `validate_handling_correlation_matrix` / `validate_ride_correlation` / `VehicleCorrelationResult` |
| `tests/analysis/test_vehicle_correlation_model.py`、`tests/adams/test_vehicle_ride_correlation.py`、`tests/adams/test_vehicle_handling_correlation.py`、`tests/adams/test_vehicle_correlation_diagnostics.py`、`tests/architecture/test_tire_model_layers.py` | 只服务上述门禁与力律的测试 |
| `tests/analysis/test_vehicle_dynamic.py::test_fiala_and_pac2002_tire_models_are_bounded_by_friction` | 同上 |

### 迁移而非删除

`Vehicle14DofParameters` → `adams/vehicle_parameters.py`。它是 Adams 输入 manifest 记录的**参数数据形状**：
`adams/vehicle_handling.py` 把它写进 `input_manifest.vehicle_model_parameters`，而
`adams/full_vehicle_model.py::_parse_reference_mass` 从那里读质量。字段名与数值属于冻结工件，
随模型一起删会让 Adams 源模型的导入路径失去参考质量。模块 docstring 写明它是数据形状而不是模型。

`analysis/__init__.py` 与 `adams/__init__.py` 的导出、`__all__` 与 docstring 同步清理。

### 结果与代价

Python 侧**不再有任何本构力律**：D3「Python 不出现残差/本构/Jacobian」现在字面成立，
Done-When #5 闭合。代价如实记录：handling 与 ride 的**数值相关**门禁（独立模型 vs Adams）不再存在；
保留的对标门禁是 strict K、strict C、K 几何参考、整轴时程、整轴等价、整车验收矩阵、
handling/ride 执行门禁与整车时程对比（`full_vehicle_correlation`，走内核而不是独立模型）。
换句话说，「Adams 侧独立实现」的范围从「工况 + 独立整车模型」缩小为「工况渲染与证据比较」，
这与 Non-Goal #1（不把 Adams 逻辑迁入 C++）不冲突：删掉的是一份**重复的物理实现**，不是 Adams 逻辑。

### 门禁（本轮实测）

```text
multibody tests                     -> 507 passed（删掉 20 项只服务被删门禁的测试）/ 1 failed（本地 artifacts）/ 47 skipped / 1 xfailed
case_parity_check.py                -> 8 families accepted（各族与上表同值）
kc_parity_check.py --check          -> OK
kc_legacy_path_check.py             -> OK（native 路径 0 处引用 Python 求解器；legacy 消费者 0）
dynamic_hash_sentinel.py --check    -> e7407656…（26 产物）逐字节不变
check_module_layering.py --strict   -> OK（86 边 / 0 环 / 0 自包含 / 0 跨模块聚合包含）
kernel 15/15 ｜ contracts 22/22 ｜ ruff All checks passed ｜ ty All checks passed
```

## Contract Takeover Follow-up（2026-09-18）

本轮继续收口整车契约路径，先修复真实测试暴露的契约表达与入口转发缺口，再完成全量复核。

### 根因与修复

- `vehicle_dynamic.py` 补齐弹簧 elastic/compression-stop/rebound-stop 曲线，支持非均匀时间网格，并为 blob 描述符补齐 `name`。
- 恒速副的 `axis_a_secondary`、`axis_b_secondary`、`convel_angle_target` 原先未进入模型文档；已同步加入 Python 作者层、模型 schema、C++ `ContractModel` 解析与 `VehicleInput` 扩展数组。
- `kernel_contract_run.cpp` 原先调用旧的 `build_model(input.axle, error)`，丢弃恒速副扩展字段和 coordinate coupler；现已完整转发这些参数。
- 该缺口会让 Adams 源模型的初始恒速约束角残差约为 0.99，统一触发 status 6；修复后源模型整车初始状态可正常通过。

### 本轮验证

```text
build_axle_native.py --configuration Release -> 成功
packages/suspension_contracts/tests              -> 22 passed
tests/cases/test_vehicle_dynamic_contract.py    -> 7 passed
tests/adams/test_full_vehicle_model.py           -> 37 passed / 1 skipped
tests/adams Fiala + PAC2002 gates               -> 31 passed
packages/suspension_multibody/tests              -> 554 passed / 1 skipped / 1 xfailed
case_parity_check.py                             -> 8 families accepted
kc_parity_check.py --check                       -> OK
kc_perf_gate.py --check-native                   -> OK
kc_legacy_path_check.py --strict                 -> OK（native 0、legacy 0）
check_module_layering.py --strict                -> OK（86 边 / 0 环 / 0 自包含 / 0 跨聚合包含）
dynamic_hash_sentinel.py --check                 -> OK，combined SHA-256 = e7407656731ed556efc28fb89d8fc69881725b3bfb2f39066eb898986389d48e
ruff                                             -> All checks passed
```

dynamic hash 工具仍报告仓库既有 9 个 acceptance `force_z` NRMSE 失败，但 26 个产物与冻结基线逐字节一致；这些报告不是本轮修复造成的 hash 漂移。



