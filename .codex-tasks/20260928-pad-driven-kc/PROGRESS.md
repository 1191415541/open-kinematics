# PROGRESS：垫板驱动的 K&C 与力平衡求解

## 恢复块

- **任务**：实现垫板驱动（Wheel Pad）的 K&C、力平衡求解与纯运动学三模式并存；力平衡成为默认
- **形态**：epic
- **进度**：**13/13 完成**
- **当前**：终局验收 **13/13 通过**（含 Adams strict-K 独立验证）
- **文件**：`.codex-tasks/20260928-pad-driven-kc/{EPIC.md,SUBTASKS.csv,PROGRESS.md}`
- **下一步**：无（全部子任务完成，终局验收通过）

## 执行进展

### 已完成：任务 1–13（详见各 task 目录的 findings.md）

| 任务 | 交付 | 关键实测 |
|---|---|---|
| 1 内核闸门 | `artifacts/pad-kc/task01/{prove.py,findings.md}` | **NOT BLOCKED**；五问各附 `file:line` |
| 2 冻结快照 | `scripts/acceptance_pad_kc_fixture.py`、`artifacts/acceptance/mode_a_frozen/` | 两次逐字节相同；K=9 case、C=66 case |
| 3 模型 road | `schema/road.py`、`FrontAxleModel.road`、runtime 传递 | 6 测试；模型侧无需改规范 schema |
| 4 契约冻结 | `multibody_case.schema.json` 加 `drive_mode`/`pad_height_mm`；`CaseSpec.drive_mode` | 规范 schema 实测接受新字段、拒绝非法值 |
| 5 力元翻译 | `artifacts/pad-kc/task05/` | **防倾杆 = z 向单轴衬套**（无需新内核元素）；契约原发 0 力元 |
| 6 三模式发射 | `contract.py::_element_entries` + `drive_mode` 贯通 plan/compile/api | 模式 A 逐位一致；**基准基线不动** |
| 7 内核垫板扫描 | `kc_quasi_static.cpp::expand_pad`；重建 DLL | 可达 `-20..+40mm`；**修掉垫板重复计入** |
| 8 结果三输出 | `results/kc_state.py::pad_contact_from_run` | `contact.z == pad` 达 5.7e-14；**修掉夹具弹簧装错本体** |
| 9 接线验证 | `artifacts/pad-kc/task09/{check_wiring.py,impact.md}` | 旧布尔仍可用；`drive_mode` 抵达编译文档；72 处调用关键字不变 |
| 10（并入 9） | 同上 | 受影响面按目录枚举，不冻结数字 |
| 11 基线归因 | `artifacts/pad-kc/task11/check_attribution.py`、`artifacts/acceptance/baseline-attribution.md` | **基线未动**，三项门禁实测通过 |
| 12 独立验收 | `artifacts/pad-kc/task12/{independent_check.py,independent-mode-bc.md}` | 模式 B 差异固定（1.307305e-03）；模式 C 行程比恒定 0.835 且 < 1 |
| 13 终局验收 | `scripts/acceptance_pad_driven_kc.py`（13 项检查） | 见下方运行结果 |

### 执行中发现并修掉的四个真实缺陷

1. **垫板被计入两次**（任务 7）：同时设 case 级 road 与 `run.road_z`，
   而轮胎是 `radius + profile + road_z - center.z`，导致行程翻倍
   （比值 1.82 → 修正后 0.910）。隐蔽点：照样收敛、曲线单调、力位移自洽。
2. **夹具弹簧装到错误本体**（任务 8）：`body_b="lower_arm_L"` 使
   `resolve_body` 短路，右侧弹簧装到左臂。线索是左右轮荷不对称
   （4433/3321 → 修正后 3025/3025）。同样会静默通过。
3. **冻结脚本记错读法**（任务 8）：`drive_mode` 默认已是 `force_balance`，
   不显式钉住就冻成力平衡而非模式 A。
4. **模式 A 在 C 上不可解**（任务 8）：C 的臂端就是其衬套列，
   去掉元素后机构无约束（`force_residual=0.026635`）。故模式 A 限定在 K。

另修掉**上一 EPIC 验收脚本的一处过时断言**：它要求「K 模式不发任何力元」，
而新默认 `force_balance` 正确地发出弹簧。已改为显式按**读法**断言
（`kinematics` 必须为空、C 的 `force_balance` 必须非空），
即该脚本原本会把正确行为判为失败。

### 预期修正：任务 11 不再是「重录基线」

原计划假设「力平衡成为默认」会让 K/C 基线失败。实测**基准 fixture 无力元**
（`springs/dampers/bushings/arb/stops/tires` 全为 0），故 `force_balance` 与
`kinematics` 发出同一空元素表，**基线不可能移动**。已实测：
动态门逐字节一致、K/C parity within tolerance、case parity 8 族全 PASS。
故任务 11 改为**验证无漂移并归因**，不重录。三处连带陈述因此**无需更正**。

### 当前验证状态

| 检查 | 结果 |
|---|---|
| **终局验收（13 项）** | **13/13 通过** |
| 快速测试集 | **940 passed, 1 xfailed** |
| `cases/` 全目录 | 100 passed |
| `architecture/` 全目录 | **149 passed** |
| kernel + contracts | 60 passed |
| `ruff` / `ty` | 全绿 |
| 三条架构门 | 全绿 |
| 动态门 / K/C parity / case parity | 逐字节一致 / within tolerance / 8 族 PASS |
| 模式 A 逐位 | **是** |
| Adams strict-K | **通过** |

## 收尾复核：Goal 逐条取证

| Goal 要求 | 可指证据 |
|---|---|
| case 可声明 `drive_mode`，三值可用且经生产入口走通 | check 1/3/12；`plan.py`/`compile.py`/`api.py` 已贯通 |
| `FrontAxleModel` 可声明 `road` | check 9；`schema/road.py` + 6 测试 |
| 契约能发射力元 | check 8（C 含 spring+bushing）；`contract.py::_element_entries` |
| 模式 C 由内核逐点独立配平 | check 4（3 高度→3 case、无轮心驱动、每 case 独立配平诊断） |
| 模式 B 为默认，力元进残差 | check 3；A≠B 实测 `1.307305e-03` |
| 模式 A 数值不变 | check 2（**与冻结 native 快照逐位一致**） |
| C 改为力平衡 + 轮胎 | check 8（含轮胎、承载 `sum\|F\| = 1.22e+06`） |
| 模式 C 三输出物 | check 5（`\|contact_z − pad\| ≤ 5.684e-14`） |
| 模式 C 覆盖 `0~+20mm` | check 7（可达 −10..+40mm） |
| 基线重录留痕 | **实测无需重录**；`baseline-attribution.md` 按两项口径归因 |
| 内核改动后重建 DLL | 已重建，镜像 SHA256 与内核一致 |
| 签名兼容 | check 12（枚举 109 行/30 文件/72 关键字，旧布尔映射正确） |
| 门禁全绿 | check 10/11；architecture 149 passed |

## 修改记录

### 任务 1 完成：内核可行性闸门 → **NOT BLOCKED**

产物：`artifacts/pad-kc/task01/{prove.py,findings.md}`（可复跑，输出五问及 `file:line`）

| 问 | 结论 | 证据 |
|---|---|---|
| (a) `contract_apply_solver` 传什么 | **`plan`** | `case_common.hpp:165` 定义；`kernel_contract_run.cpp:686` 调用 |
| (b) 每 case 是否独立配平 | **是**（既有行为） | 循环 `:633`；每 case `run_model` `:751`；配平分支 `kernel_abi.cpp:184`→`:198`；初始状态重建 `:174` |
| (c) 如何达成每点独立配平 | 在 `expand_k_axes`（`kc_quasi_static.cpp:127`）**之外**加垫板展开循环，每个垫板高度生成 `sample_count=2` 的 case（内核禁止单样本 `case_common.hpp:315,339`），两点目标相同 → 一个平衡态保持两时刻。`read_time` **不需改** | 同上 |
| (d) 垫板轴字段定案 | **新增 `k.pad_height_mm`**（不复用 `axis_map`：`kc_quasi_static.cpp` 中 `road` 0 命中，`:142/277/284` 全走 `model.driven_index`） | 同上 |
| (e) 是否需重构求解器 | **不需要** | 残差已含轮胎接触 `kernel_static_trim.cpp:50,64`；路面表已进输入 `contract_model.cpp:1762` |

### 任务 2 完成：冻结 native 模式 A 快照

产物：`scripts/acceptance_pad_kc_fixture.py`、
`artifacts/acceptance/mode_a_frozen/{mode_a_snapshot.json,README.md}`、
`artifacts/pad-kc/task02/freeze_and_verify.py`

**实测：两次运行逐字节相同**（证明 native 可复现，逐位才是合法判据）。K：9 case；C：66 case。

**执行中发现两件事**（EPIC 未预见，已按实测调整）：

1. **K 与 C 需要不同模型，不是同一模型的两个设置**。K 走**刚性**运动学集
   （理想约束、无衬套）；C 加载**柔性**集（臂端由衬套承载）。给 C 用刚性模型会失败：
   实测 `c-fx--1.00` 停在 `force_residual=0.026635`。夹具因此提供两份模型
   （`rigid_model()` / `compliant_model()`），C 侧 4 个衬套。
2. **模式 A 的文档 `elements` 与 `tires` 均为空**（夹具含弹簧但 A 不发），
   与「纯运动学」定义一致，也印证「A 与 B 的可区分性由弹簧提供」。

## 修改记录

### 2026-09-28 第 7 次修订：第七次审核的 1 条阻断（road 的数据通路）

第七次审核**不通过**（1 条），并明确指出它是第六轮阻断 A 的**同类缺陷的另一半**：
判据补上了，喂给它的**机制**没补。我独立复核，**成立**。

#### 阻断：`road` 从 `FrontAxleModel` 走到模型文档的通道无归属

第六轮给任务 6 补了判据「模型文档含 road 节（取自装配体模型的 road 声明）」，
但任务 6 的写入范围**只有 `contract.py`**，而 `model_document(assembly, …)`
的唯一模型来源是装配体。实测：

- `SubsystemRuntime` 的字段为 bodies/bushings/capabilities/connections/constraints/
  elements/hardpoints/ideal_constraints/mode/points/state——**没有 `road`**，
  也不持有 `FrontAxleModel`；
- `model_document` 有 **6 个调用点**（`api.py:195`、`adams/reference.py:151`、
  `adams/strict_k.py:227`、`adams/strict_c.py:214`、`studies/assembly.py:163`
  及契约自身），**全都只拿到装配体**；模型在生产路径上
  （`api.run_case` → `_kc_assembly` → `compile_plan` → `emit`）被丢掉。

后果：任务 6 的执行者若照验收做，只能在 `contract.py` 里凭空造一个 road
（那就不是「取自装配体模型」，且生产路径拿不到——**假通过**），
或者越界改任务 9 的文件（并行写同一批文件）。

**处置（采纳审核的方案甲）**：把装配体侧的通道交给**任务 3 独占**——
它的写入范围增加 `subsystems/runtime.py` 与 `subsystems/si_assembly.py`，
验收增「road 声明随装配体传递到 runtime，使 `contract.model_document` 可读取」。
任务 3 无依赖、与任务 1/2/5 并行且文件不重叠；任务 6 已依赖 3，顺序天然成立。

同时按审核的可选改进：任务 6 的验收写明 **road 参数为可选关键字（默认 None）**，
使 `studies/assembly.py:163` 等既有调用者不破。

#### 残留数字

任务 9 的验收仍有「29 文件 101 处」，与 EPIC 已确立的「不冻结数字」不一致
→ 改为「脚本自行枚举，不冻结数字（三次测量给出 72/101/102 三个口径）」。
（剩余的 `101` 均在**描述测量值**的语境里，不是判据，保留。）

**修改后复核**：13 行、无环；任务 3 写入范围 5 个文件、任务 6 依赖 `2;3;4;5`。

**下一步**：第八次（终审）审核。前七轮共 33 条全部处置；其中 3 条推翻了
我自己先前的写法。第七轮审核确认「前六轮 32 条全部维持、无一条需要撤回」。

### 2026-09-28 第 6 次修订：第六次审核的 2 条承接缺口

第六次审核确认前 2 条阻断与 3 处反驳**全部清零**，但发现 2 条新阻断
（均为承接缺口）。我独立复核（`artifacts/refactor/verify_sixth_review.py`），
**两条都成立**。

#### 阻断 A：Done-When 9 的「契约写进文档」无任务承接

实测 `contract.py` 中 `road` **0 命中**，且 `model_document` 的文档键里确实没有
`road`（键为 contract/contract_version/kind/name/units/gravity/bodies/joints/
elements/tires/markers/body_wrench_markers）。而 Done-When 9 断言
「`FrontAxleModel(road=...)` 被接受，**契约写进文档**」——
**没有任何任务的验收提这件事**。

→ 任务 6（`contract.py` 的唯一写者）的验收增「模型文档含 road 节，
字段名对齐 `contract_model.cpp:1302-1316` 的 parameters」；
并补 `depends_on: 3`（需要任务 3 的字段定义），改为 `2;3;4;5`。

#### 阻断 B：任务 11 缺 `depends_on: 7`

实测 `closure(11)` 不含 7，而 `check_composable_baseline.py:279-292` 确实校验
`sources.kernel_cpp` 的逐文件指纹，任务 7 会改其中 2 个内核文件。
在 7 之前重钉指纹会记录到**变更前的值**，7 之后又失配、须重做。
→ 任务 11 改为 `7;9`。

#### 残留的冻结数字

`101` 残留在 EPIC 多处与 CSV 两处，而三次测量给出 72/101/102 三个不同口径。
→ EPIC 中改为「29 个文件（任意形式 101–102 行）」这类**测量区间**措辞；
任务 13 的 signature 检查改为「脚本自行枚举，不冻结数字」。

**修改后复核**：13 行、无环；任务 6 依赖 `2;3;4;5`、任务 11 依赖 `7;9`。

**下一步**：第七次（终审）审核。前六轮共 32 条，全部处置；其中 3 条推翻了
我自己先前的写法。第六轮审核明确表示「阻断 A、B 之外的方案已经达到可开工标准」。

### 2026-09-28 第 5 次修订：第五次审核的 2 条依赖边 + 3 处实测反驳

第五次审核**不通过**（2 条阻断，均为依赖边）。我独立复核
（`artifacts/refactor/verify_fifth_review.py`），结果与审核有一处**精细差异**，
按实测处置。

#### 阻断 1：任务 6 缺 `depends_on: 2`

审核说我「第 3 次修订加过这条边、第 4 次改回去了」。实测依赖闭包：

```
closure(6) = ['1','2','4','5']    # 2 通过任务 5 传递可达
```

即**顺序上没炸**。但审核的真问题成立：**任务 5 根本不需要任务 2**
（它的写入范围是 `artifacts/pad-kc/task05/`，只产出映射表，验收里不含冻结快照）。
所以这条边是**错放的**——一旦按正理由移除，任务 6 就真的失去与任务 2 的顺序。

处置：**任务 5 去掉 `depends_on: 2`，任务 6 显式改为 `2;4;5`**。
理由写进 EPIC 的 D6：真正改变力元发射的是任务 6，不是任务 5。

#### 阻断 2：任务 8 缺 `depends_on: 7`

**成立**。任务 8 的验收含「新增测试断言 `contact.z == 垫板高度`」，
而垫板高度驱动由任务 7 落地——实测 `kc_quasi_static.cpp` 中 `road` **0 命中**、
`:142/277/284` 全走 `model.driven_index`，任务 7 之前**构造不出**垫板驱动的 K 解。
→ 任务 8 改为 `6;7`。

#### 三处实测反驳

**反驳 1：`drive_wheels` 计数。** 我实测三个口径：
任意形式 **101 行**、`drive_wheels=` 关键字 **72 行**、文件 **29 个**
（审核报 74/103/31，口径不同但核心结论一致）。
EPIC 把 101 冻结成 Done-When 12 的硬断言，而它自己又写 74——**这个数不该冻结**。
→ Done-When 12 改为「脚本自行枚举调用点，不冻结数字」，并说明理由
（冻结一个对不上的数会让该判据恒真或假失败，正是本 EPIC 自己警示过的缺陷）。

**反驳 2：失效任务 id。** EPIC 中残留 `任务 5a`、`任务 4b` 两处（CSV 只有 1–13）。
→ 已改为「任务 5」、「任务 4 就是那次冻结」。

**反驳 3：Done-When 8「解不再是纯几何」不可判定。**
→ 改写为可断言形式：同一夹具下 `kinematics` 与 `force_balance` 的 `body_state`
不逐位相同，而纯几何模式下二者必须相同。

#### 同轮修正的其他措辞

- EPIC 的 D8b 自相矛盾（一处要任务 7 也改 `multibody_case.schema.json`，
  另一处说任务 4 独占）→ 统一为**任务 4 独占**，任务 7 只实现内核侧读取。
- Done-When 1 的「三值都能求解」在任一组夹具上都不成立
  → 改为「A 与 B 在 A/B 组求解成功，C 在 C 组求解成功」。

**修改后复核**：13 行、无环、依赖完整
（1/2/3/5 无依赖；4←1；6←2,4,5；7←1,3,4,6；8←6,7；10←3,7,8；13←11,12）。

**下一步**：第六次审核。

### 2026-09-28 第 4 次修订：第三次审核的 7 条（含 3 条新实证）

第四次审核**不通过**（7 条）。我逐条独立复核（`artifacts/refactor/verify_fourth_review.py`、
`settle_two_claims.py`），结论：**6 条成立，1 条（B7 文档同步）属自查发现**。

#### 三条新实证（此前从未检查过）

**新实证 1：规范 JSON schema 会拒绝本次全部新增字段，且此前无归属。**
`multibody_case.schema.json` 的顶层与 `k` 都是 `additionalProperties: false`，
校验器实测真的拒绝：

```
case.drive_mode    -> ContractError: $: unexpected fields ['drive_mode']
k.pad              -> ContractError: $/k: unexpected fields ['pad']
k.pad_height_mm    -> ContractError: $/k: unexpected fields ['pad_height_mm']
```

而 `test_contract_documents.py` 等会调 `validate_case`。
该 schema 文件**此前不在任何写入范围**。
→ 新增 D8b；任务 4 改为「契约冻结」，独占 `multibody_case.schema.json`
（模型侧的 `road` 实测已被 `multibody_model.schema.json` 接受，无需改）。

**新实证 2：垫板展开大概率不需要改 `read_time`——我上版说重了。**
按 case 正确分组实测（`raw.states` 是**跨 case 展平**的，上版我的漂移计算跨了边界、
得出 2.5e-02 的假漂移）：3 case × 4 样本，
**三个 case 的 `max |sample - sample0|` 全为 `0.000000e+00`**。
即每 case 本来就各自配平、case 内无漂移。
→ 上版「需要改 `read_time`/`expand_k_axes`」**降级**为
「大概率不需改 `read_time`，只需加垫板展开循环」；任务 1 仍须确认。

**新实证 3：防倾杆的结论可以直接定，不是「可选」。**
`vehicle_dynamic.py:270` 是 `anti_roll_bars=()`——**扭转型在生产路径无消费者**；
`subsystems/element_build.py:124-133` 把 `AntiRollBar.torsional_stiffness`
**直接当连杆律刚度**用，`AntiRollBarElement.evaluate` 算的是
`k*(Δz - ref)` 的左右垂向力对。
→ 上版把「替代实现或 BLOCKED」当验收通过条件是错的（TODO 行不得以 BLOCKED 结项）。
改为：任务 5 的交付是**给出连杆律垂向力对的实现选择**（复用现有元素类型，
或新增内核元素类型——后者须同时改 `contract_registry.cpp` 的 `kElements` 与
`multibody_model.schema.json` 的 element enum）。

#### 其余 4 条

| 阻断项 | 处置 |
|---|---|
| Done-When 1 的夹具与模式 C 物理不相容（pad 法向力只能来自轮胎，而无胎夹具 `tires == 0`） | Done-When 1 拆两组：A/B 组用任务 2 无胎夹具、C 组用任务 10 含胎夹具；任务 10 的夹具明确含轮胎 |
| 「A 不发轮胎」无验证路径（无胎夹具上它测的是夹具而非发射器） | 新增 Done-When 6 `tire-emission`：在**含胎**文档上断言 A 的 `tires == []`、B/C 非空，且不参与逐位比对 |
| 依赖漏边：模式 C 需要任务 3 的 `road`（`registration.cpp` 要求 plane 也带 `wavelength`/`bump_length`/`corner_scale`） | 任务 7、10 的 `depends_on` 加 3；任务 2 明确「夹具不声明 road」以免反向依赖 |
| `contract.py` 签名变更与 101 处调用点无归属 | 任务 9 的写入范围扩至 `adams/`、`scripts/`、`tests/`；新增 Done-When 12 `signature` |

#### 自查发现（审核第 7 条）

EPIC 中 6 处任务编号错位（我上版声称「已更正 4 处」但实际未改，且有 6 处）。
已逐处更正：kernel_cpp 指纹→任务 7、重录命令→任务 11、受影响面→任务 9、
窗口实测→任务 10、验收脚本→任务 13、垫板字段名→任务 1 定案/任务 7 实现。

同时更正本文件 §6 与第 3 次修订的自相矛盾（§6 原仍写 `sample_count=1` 为实现方式，
已改为实测确证的措辞并保留撤回痕迹）。

**修改后复核**：13 行、无环、依赖完整（任务 1/2/3 无依赖可并行；
6←4,5；7←1,3,4,6；10←3,7,8；13←11,12）。

**下一步**：第五次派 `code-reviewer` 审核。前四轮共提出 28 条，其中 3 条推翻了我自己的
写法（Python oracle 判据、1.307e-03 的来源、read_time 的必要性）——
这些都已按实测更正并留痕。

### 2026-09-28 第 3 次修订：第三次审核的 5 条，含两条硬事实

第三次审核**不通过**（5 条）。我逐条独立复核（`artifacts/refactor/verify_third_review.py`、
`which_anti_roll.py`、`link_law_anti_roll.py`），**全部成立**。属第 2 级修改。

#### 两条硬事实（改变方案形态，不只是措辞）

**硬事实 1：内核禁止 `sample_count == 1`。**
`cases/case_common.hpp:315` 与 `:339` 两处强制
`if (count < 2) return fail(error, "case time needs at least two samples")`；
且 K 的所有 case 共享文档级时间网格（`kc_quasi_static.cpp:157,169`）。
所以 Done-When 4 原写的「每个垫板高度是 `sample_count == 1` 的 case」**恒假**。
→ 任务 1 的问题清单加入该约束（任务 1 (c)）；
Done-When 4 与任务 7 的判据改为「**case 数 == 扫描点数且每个 case 各自独立配平**」，
`sample_count == 1` 降为**候选实现**。

**硬事实 2：防倾杆语义不等价。**
模型层有两种表示，装配体产出的是**连杆律**：

- `modeling/primitives/elements.py:593` `AntiRollBarElement`，docstring 原文
  "Equivalent torsional anti-roll bar **driven by link vertical travel**"——
  按左右点垂向位移差施加垂向力；
- 内核 `anti_roll_bar` 要**扭转式**（`axis_a` + `reference_quaternion` + N·m/rad，
  `contract_model.cpp:712-726`）。

`preparation/vehicle_dynamic.py:869-873` **明确 raise**：
"anti-roll element ... is a link anti-roll law; the native torsional anti-roll ABI is
not equivalent"。而 `cases/axle_dynamic.py:188` 的 `_anti_roll_element` 翻译的是
**扭转式** `AntiRollBar`——**两者不是同一个东西**（这一点上轮未区分）。
用户需求点名了防倾杆，故不得静默省略。
→ 新增任务 5（闸门二）：逐元素翻译映射表 + 防倾杆的替代实现或 BLOCKED。

#### 其余 3 条

| 阻断项 | 独立复核 | 处置 |
|---|---|---|
| D6 逐位判据在 Done-When 2 的夹具上不可满足 | **成立**：`contract.py:216` **无条件**发轮胎，故 A（不发轮胎）与冻结快照必然不同 | 明确模式 A 的冻结快照与验收夹具**都不得声明轮胎**；A 与 B 的可区分性由**弹簧**提供。EPIC 新增一节说明「为何 A 是'不发'而非'保持现状'」 |
| 任务 2 与验收夹具顺序不可能 | **成立**：夹具原归任务 13，任务 2 无依赖却要用它 | 夹具改由任务 2 建立（写入范围 `scripts/acceptance_pad_kc_fixture.py`），任务 13 引用 |
| 任务 2 与任务 6 无依赖边，冻结存在竞态 | **成立**：两者 depends_on 均空，「必须最先做」只写在 notes | 任务 6 的 depends_on 加入 2 |
| 契约签名变更与 101 处调用点无归属 | **成立**：实测 29 个文件、101 处 `drive_wheels=` | 任务 6 的验收增「签名变更保留向后兼容」；Constraints 记录该数量 |

#### 修正的其他事项

- EPIC 中 4 处任务编号错位已更正（D9 的 kernel_cpp 指任务 6、重录命令指任务 10、
  受影响面指任务 8、窗口实测指任务 9）。
- 任务编号整体重排为 13 行（新增任务 5 闸门二、任务 13 合并终局验收与收尾）。
- PROGRESS 的事实章节同步更正：`1.307e-03` 与「K 契约发 0 力元」两处已在第 2 次
  修订推翻，本节随之更新。

**修改后复核**：13 行、无环、依赖完整；任务 2 无依赖（必须最先）、任务 6 依赖 `1;2;4;5`。

**下一步**：第四次派 `code-reviewer` 审核。

### 2026-09-28 第 2 次修订：第二次审核的阻断项，其中两条推翻了我自己的写法

第二次独立审核结论**不通过**（8 条）。我**逐条独立复核**了它的事实声明
（`artifacts/refactor/verify_review_claims.py`、`determinism_and_columns.py`），
**全部成立**，其中两条**推翻了上版 EPIC 的写法**。属第 2 级修改，留痕如下。

#### 两条我写错、经实测更正的事实

**更正 1：模式 A 的逐位判据不可满足。**
上版写「模式 A 与重录前 K 基线逐位一致」。实测冻结的
`tests/data/kc_baseline/k_states.json` 是**退役 Python 求解器**的 oracle，
native 与它 **108 个字段中 76 个不同**，最大差 `1.655482e-06 mm`
（`k-w-10-r-5/right_wheel_center_x_mm`）。逐位比较**恒不成立**。
而 native 输出**自身可复现**（同一脚本连跑两次逐字节相同）。
→ 改为 D6：「与冻结的 **native** 模式 A 快照逐位一致」，
并新增任务 2 在**任何发射改动之前**冻结该快照。

**更正 2：1.307e-03 不是契约能产出的量。**
上版写「声明力元改变 K 数值 1.307e-03」并暗示 C 已能发弹簧。
实测 `contract.py` 中 `spring`/`damper`/`anti_roll` **各 0 命中**，
只有 `bushing`。给模型声明 2 个弹簧后：

```
K: assembly=['LinearSpringElement','LinearSpringElement']  document=[]
C: assembly=[2×LinearSpringElement, 8×BushingElement]      document=[8 bushing]
```

即**装配体有力元、契约不发**。那 1.307e-03 来自手工注入的元素行
（`measure_declaration_delta2.py:66-77` 拼 `{"type":"spring",...}`），
描述的是「若契约发了弹簧会怎样」，不是现状。
→ 上版的「K 契约发 0 力元」掩盖了真实缺口 **C 也发不出**；
新增任务 5 专门补力元发射。

#### 其余 6 条的处置

| 阻断项 | 独立复核结果 | 处置 |
|---|---|---|
| 决策 5 在基准 fixture 上无从生效 | **成立**：`benchmark_model()` 的 springs/tires/dampers/bushings/arb 全为 0 | D7：模式 C 用带预载弹簧专用夹具（实测加弹簧后 K 文档逐字节一致、K 解 Δ=0，不冲突模式 A 判据）；Done-When 6 增硬门槛「覆盖 `0 ~ +20 mm`」 |
| 缺用户要求的三个输出物 | **成立**：`results/kc_state.py` 中 `wheel_center`/`load`/`contact_point` 零命中 | D5 写死几何公式；新增任务 7（`results/` 有承载体）；Done-When 5 断言 `contact.z == 垫板高度` |
| `drive_mode` 无法经 `drive_wheels: bool` 表达，`compilation/` 无归属 | **成立**：`plan.py:104` 是 bool，`compile.py:99/115` 传它 | 任务 8 的写入范围加入 `compilation/plan.py`、`compilation/compile.py`、`api.py` |
| 垫板轴无法用现有机制表达 | **成立**：`kc_quasi_static.cpp` 中 `road` **0 命中**，`:142/277/284` 全走 `model.driven_index` | 列入任务 1 问题清单 (d)；D2 收敛为「case 级垫板节 + `run.road_z`」唯一机制 |
| `check_composable_baseline.py` 连带失效 + 无人跑 | **成立**：实跑 **exit 1**（canonical/mirror sha256 不符），justfile 无引用 | D9 登记五类连带；说明它**本就失配**而非重录触发 |
| Done-When 关键判据在所选 fixture 上恒真/恒假 | **成立**（与更正 1 同源） | Done-When 每条**注明 fixture**；Done-When 2 的 fixture 必须有力元，并断言 A≠B |
| 任务 8 的 validation 指向陈旧产物 | **成立**：`--actual-dir` 的文件只由两个 probe 生成 | 任务 10 改为 `check_attribution.py`，并写明重录值的产出方式 |
| 任务 2/3 写入范围重叠、任务 3 越界 | **成立** | 已在第 1 次修订处置；本轮保持 |

#### 未采纳（记为可选改进，不扩范围）

- 任务 4「一行四交付物」：任务 5 已承接力元发射，视为已拆。
- `kernel_contract_run.cpp:644` 的 case 级 road 不复位：登记在任务 6 的 notes。
- `check_composable_baseline.py` 摘要重钉：D9 已登记，具体动作由任务 10 定。

**修改后复核**：11 → 13 行，`depends_on` 无环、每行均有验收与 validation。

**下一步**：第三次派 `code-reviewer` 审核（改动为整篇重写，属第 2 级）。

### 2026-09-28 第 1 次修订：清零独立审核提出的 8 条阻断项

**触发**：按 AGENTS.md 强制要求派 `code-reviewer` 只读审核 EPIC/SUBTASKS（审核输入
含用户原始需求原文）。结论**不通过**，8 条阻断项。我已**独立复核**其中关键几条
（不是照抄审核结论），复核结果与处置如下。

**修改级别**：新增验收项、收紧 `acceptance_criteria`、调整依赖与写入范围 → 属第 2 级，
需留痕（本条即为留痕）。**未**删除子任务、**未**放宽任何验收、**未**改 Goal/Non-Goals。

| 阻断项 | 我的独立复核 | 处置 |
|---|---|---|
| B1 决策 5 在基准 fixture 上无从生效 | **成立**：实测 `benchmark_model()` 的 springs/tires/dampers/bushings/arb **全为 0** | 新增「模式 C 用带预载弹簧的专用夹具」；实测该修法**安全**——给模型加弹簧后 K 文档逐字节一致、K 解 `max|delta|=0`（因 K 契约不发力元）。Done-When 5 增硬判据「可达范围 ≥10mm 且覆盖 0~+20mm」，不足即 FAIL |
| B2 缺用户明确要求的三个输出物 | **成立**：用户原话「收敛，输出当前轮心坐标、轮胎载荷与实时接地点」；实测 `results/kc_state.py` 中 `wheel_center`/`load`/`contact_point` **零命中**（只具名了 penetration 一列） | EPIC 引用原话补全（不再用「……」省略）；交付边界增第 8 项；Done-When 新增第 6 条 `pad-outputs`，要求接地点与轮心/半径满足几何关系，不是「字段存在」的弱证据 |
| B3 重录前基线无独立留存归属 | **成立**：任务 8 的 write_scopes 覆盖 `kc_baseline/`，任务 9 在 8 之后才写脚本，届时旧值已不存在 | 任务 8 增加「**第一步**复制到 `kc_baseline_pre_rerecord/`」；Done-When 2 改为引用该冻结副本 |
| B4 `BASELINE.json` 的 sha256 连带失效漏登记 | **成立**：实测 `check_composable_baseline.py:121-132` 校验 kc snapshot 的两个 sha256 | D5 增列第三处连带陈述（`BASELINE.json` 的 kc snapshot sha256），任务 8 验收含该文件更新 |
| B5 任务 1「结论为否」缺结构化分支 | **成立** | 任务 1 的 `acceptance_criteria` 增停机条件；新增 D7 明确「用户决策 6 优先于 Non-Goals 的算法不重构条款」；Constraints 加例外说明 |
| B6 任务 11 的验收手段与验收项不匹配 | **成立**：原来只跑 architecture 目录，证明不了 Done-When 十条 | 任务 11 的 `validation_command` 改为 `acceptance_pad_driven_kc.py --check all`（并说明另跑门禁作补充） |
| B7 任务 8 的 validation 不验证自身交付物 | **成立**：重录后自比恒过（`kc_parity_check.py:5-19` 自述） | 任务 8 改为 `check_attribution.py`，比对对象是「冻结副本 vs 重录后基线」 |
| B8 写入范围目录级重叠 + 任务 3 验收越界 | **成立**：`schema/__init__.py` 是共享注册文件（AGENTS.md 禁并行）；`case_document` 在任务 4 的写入范围 | 任务 2 的 write_scopes 收窄到具体文件并独占 `__init__.py`；把「`case_document` 输出」从任务 3 移到任务 4 |

**未采纳的可选改进**（记入 notes，不扩范围）：任务 1 自证性质、`road` 契约形状
冻结、模式 C 基线归属等已在对应行的 notes 中说明。

**修改后复核**：`depends_on` 仍无环、11 行唯一、每行均有验收与 validation。

**下一步**：重新派 `code-reviewer` 审核修订后版本（AGENTS.md 要求）。

## 已实测的事实（本轮诊断，全部可复跑）

诊断脚本位于 `artifacts/refactor/`，均只读。

### 1. 你要的物理机制内核里已经存在

| 要素 | 位置 |
|---|---|
| 垫板高度作输入 | `tire/common/kinematics.cpp:33-36` `input.road_z[tire]` |
| 实时几何求交 | 同文件 `:57` `radius + road - center.z`，每步按当前位姿算 |
| 法向弹性支撑（只受压） | `solve_static/kernel_static_contact.cpp:112` `max(0, tire.k*compression)` |
| 切向完全释放 | `cases/kc_quasi_static/contract.py:228-242` 中性系数，docstring 明言无滑移 |
| 力元 + 残差 R(q,λ) | `solve_static/kernel_static_trim.cpp:30-114`，压缩量是未知量 |
| Newton + 活动集 | `solve_static/kernel_static_contact.cpp:283-623` |
| 垫板非刚体 | `cases/contract_model.cpp:1290-1318` 解析高度场 |

### 2. 三处真实缺口

1. **`FrontAxleModel` 无 `road` 字段**——内核解析它（`contract_model.cpp:1290`），
   作者侧拒绝：`extra inputs are not permitted`。
2. **契约发不出力元——K 与 C 都发不出**（第 2 次修订更正了原先「K 契约发 0 力元」的
   写法，那掩盖了真实缺口）。实测 `cases/kc_quasi_static/contract.py` 中
   `spring`/`damper`/`anti_roll` **各 0 命中**，只有 `bushing`（`_bushing_element`）。
   给模型声明 2 个弹簧后：

   ```
   K: assembly=['LinearSpringElement','LinearSpringElement']  document=[]
   C: assembly=[2×LinearSpringElement, 8×BushingElement]      document=[8 bushing]
   ```

   即**装配体有力元、契约不发**。`contract.py:216` 是**无条件**发轮胎的。
3. **防倾杆语义不等价**（第 3 次修订新增）。装配体产出的是连杆律
   `AntiRollBarElement`（`elements.py:593`，按左右点垂向位移差施加垂向力），
   内核 `anti_roll_bar` 要扭转式（`contract_model.cpp:712-726`），
   `vehicle_dynamic.py:869-873` 明确拒绝转换。用户需求点名防倾杆，不得静默省略。
4. **声明力元会改变数值：`max |delta| = 1.307305e-03`**。
   **注意**：该值由**手工注入的元素行**测得（`measure_declaration_delta2.py:66-77`
   拼 `{"type":"spring",...}` 后比对），**契约当前产不出这个量**；
   它描述的是「若契约发了弹簧会怎样」，不是现状。
5. **`results/` 层无垫板模式的具名输出**。实测 `results/kc_state.py` 中
   `wheel_center`/`load`/`contact_point` 零命中，只具名了 penetration 一列。
   内核 `tire_output` 41 列的列映射见 EPIC 的现状 2c。

### 3. 两个必须写进实现的坑

**(a)** `assembly/registration.cpp:326` 要求**任何**非零 road（含 `plane`）的
`wavelength > 0`、`bump_length > 0`；`:331` 还要求 `corner_scale`。
平面路面用不到这几个量，但校验不做例外。漏写报
`vehicle road profile parameters are invalid`。

**(b)** 内核**禁止 `sample_count == 1`**（`cases/case_common.hpp:315,339`），
且 K 的所有 case 共享文档级时间网格（`kc_quasi_static.cpp:157,169`）。
所以「每个垫板高度一个单样本 case」不是现成能力（第 3 次修订发现）。

### 4. 决定可行性的物理约束：收敛窗口约 ±20mm

手工构造 mode C 原型（K 文档 + 轮胎 + 力元 + `road` 平面 + 去掉轮心驱动），
**未改任何内核代码**，扫描垫板高度：

```
  preload          -40    -30    -20    -10     +0    +10    +20    +30    +40
       +0 N    fail   fail   fail   fail   OK     OK     OK     fail   fail
    -2500 N    fail   fail   fail   OK     OK     OK     fail   fail   fail
    -5000 N    fail   fail   OK     OK     OK     OK     OK     fail   fail
    -7500 N    fail   fail   OK     OK     OK     OK     OK     fail   fail
   -10000 N    fail   fail   fail   OK     OK     OK     OK     fail   fail
```

（预载 = 弹簧装配长度 250mm 与自由长度之差 × 50 N/mm）

边界处力残差跳到 0.59~0.95，而 `pinned_null_directions=0`——**不是约束欠定，是力不闭合**。

根因三件事叠加：

- 轮胎只受压（`max(0, k·δ)`）；
- K/C 文档 `gravity=[0,0,0]`、chassis 固定、1000kg 簧上质量落在**固定**体上；
- 自由体质量全是 1.0 → **没有向下的力**把车轮压进垫板。

实测对照：
- 零预载：pad 0 通过，pad −10 即失败（`force_residual=0.9199`）
- 7.5kN 预载：pad −20 通过
- 打开 gravity **反而更差**（`force_residual` 0.67~1.26）——重力作用在固定 chassis 上
- 把 chassis 改成自由体 + gravity：**全部失败**（缺垂向导向）

按用户决策 5，垂向载荷由**弹簧预载衍生**，故这是设计的一部分。任务 7 负责把
可达范围作为**实测交付**记录，不假设能覆盖完整 K&C 曲线。

### 5. 「K + 轮胎今天必失败」的机理

实测：`travel=0` 通过、`travel=-20` 失败
（`iterations=1, force_residual=0.000000, position_residual=0.020000`）。

力已平衡、位置残差 20mm 一步未收敛。根因：**规定轮心位移与轮胎法向力互斥**。
这正是本 EPIC 要解掉的东西，也印证决策 3 的方向。

### 6. 内核静力配平的粒度（第 4 次修订已实测确证）

`run_model`（`kernel_contract_run.cpp:751`）与 `contract_apply_solver`
（同文件 `:686`）都在 `plan.cases` 循环内部，所以静力配平**本来就是每个
ContractCase 一次**。第 4 次修订进一步实测确证：

- 每 case 的 trim 诊断独立（`raw.diagnostics` 每 case 一行）；
- **case 内样本间漂移为 0**：3 case × 4 样本，三个 case 的
  `max |sample - sample0|` 全为 `0.000000e+00`（按 case 正确分组；
  `raw.states` 是跨 case 展平的，直接整块比会得到假漂移）；
- `kernel_abi.cpp:184` 只在 `initialization_mode == 0` 时配平。

**结论**：垫板展开**大概率不需要改 `read_time`**，只需在 `expand_k_axes` 之外
加一个垫板展开循环写 `run.road_z`。第 3 次修订曾说「需要改 `read_time`/`expand_k_axes`」
——**那句话已按实测撤回**。

原写「实现方式是把垫板网格展开成 `sample_count=1` 的多个 case」**已撤回**：
内核禁止 `sample_count == 1`（`case_common.hpp:315,339`），
判据改用「case 数 == 扫描点数，且每 case 各自独立配平」。

`initialization_mode` 取自 `plan`（计划级，`case_common.hpp:126`），
但每 case 的初始状态从装配位姿重建（`kernel_abi.cpp:173-178`），
故「计划级取值 + case 内调用」确实产生「每 case 一次配平」。

## 与用户确认过的决策

| # | 问题 | 用户选择 |
|---|---|---|
| 1 | K 扫描横轴 | 垫板抬升 |
| 2 | 垫板实现 | 模型 `road` 解析块（路线 c） |
| 3 | 轮心驱动 | 并行保留，但也改成力平衡驱动 |
| 4 | 纯运动学 | 保留 |
| 5 | 垂向轮荷 | 由弹簧预载自动衍生；垫板用强制位移驱动 |
| 6 | 每点独立平衡 | 由内核实现 |
| 7 | C 路线 | 也改成力平衡 + 轮胎 |
| 8 | 基线 | 力平衡成为新默认，重录（已授权） |
