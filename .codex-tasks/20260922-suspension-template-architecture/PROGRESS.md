# Epic 进度：20260922-suspension-template-architecture

## 恢复信息

形态：epic。
进度：**0/12 子任务 DONE**（全部 `TODO`）。本轮为**规划交付**，未写任何生产代码。
当前：规划已闭环，可启动 01。
文件：`.codex-tasks/20260922-suspension-template-architecture/`（`EPIC.md` + `SUBTASKS.csv` + 本文件 + `tasks/20260922-01..11/`）。
验证：本轮未运行生产构建或数值测试；只做了计划文件结构与一致性自检（见下）。

## 用户原话（本轮需求原文，逐条）

1. 「我想把 K/C 模式给通用化，任何的仿真都可以由用户选择 K 或 C 模式」
2. 「在模型装配时声明是 K 模式还是 C 模式，如果用 K 模式则使用 joint，如果是 C 模式则使用 bushing；各个悬架类型的 K、C 模式的运动副、衬套的定义都是预先设定好的，比如下摆臂在 K 模式三个连接点都是运动副，在 C 模式则只有前后点是衬套。模型装配好后也可以任意切换 K、C 模式。」
3. 「我设想的对于任意仿真应该是不同总成+试验台的组合，比如 kc_quasi_static 是单轴总成+悬架 KC 试验台、vehicle_kc 是整车总成（整车总成又相当于两个单轴总成+其他子总成）+悬架 KC 试验台、axle_dynamic 是单轴总成+悬架 KC 试验台（和 kc_quasi_static 的区别应该只有求解器、轮胎的区别）；重点关注 kc_quasi_static 和 axle_dynamic，这两个应该只有求解器和轮胎区别，所以也应该尽可能通用化。」
4. 「不同类型的总成运动副类型都要统一，所以运动副类型都可以用在任意总成，也就是说所有总成公用一个底层」
5. 「对于 axle_dynamic 与 kc_quasi_static，这两个完全可以合并为一个仿真，可以选择准静态和动态模式」
6. 「可以参考 adams car 的模板概念，由专家定义多套模板（使用者无需关注），模板里在同一个点可以建立运动副和衬套，在 K 模式只激活运动副、C 模式只激活衬套，这样的话切换 K、C 模式会很方便，衬套、弹簧、减振器等弹性元件由外部属性文件控制属性，也就是说可以加载不同属性文件达到不同刚度、阻尼等等特性」
7. 「零刚度占位衬套可以改为使用模板定义好的默认属性代替」
8. 「所有仿真的输出由总成+试验台定义，总成带有它所定义的输出，试验台也带有它独有的输出，这些输出是最小单位输出，后续的衍生结果都是使用这些输出做运算得到的（比如 KC 指标、操稳指标等等），可以引入类似 adams car 的 request 功能，即可以通过这些基本输出自定义任何输出」
9. 「准静态 接受 fiala/pac2002/native_brush 只是使用其垂直刚度和轮胎尺寸、质量，相当于模型退化」
10. 「轮胎质量归属到轮胎」
11. 「彻底改三层」
12. 「可以重录基线」
13. 「子系统是左右悬架、转向、轮胎、车身，左右悬架加转向加轮胎加悬架实验台得到悬架实验总成，前后悬架加转向加车身加轮胎加整车 kc 实验台得到整车实验总成」
14. 「先制定计划，不具体实现代码」（本轮交付边界）

## 用户裁决登记（本轮四项）

| 编号 | 问题 | 用户裁决 | 落点子任务 |
|---|---|---|---|
| D1 | 准静态轮胎力律如何落地 | **不新增内核力律**：准静态接受 `fiala`/`pac2002`/`native_brush`，只使用其垂向刚度与轮胎尺寸、质量，相当于**模型退化** | 09 |
| D2 | 轮胎质量归属 | **归属轮胎**（选"乙"：tire 成为独立惯量来源，求解器显式耦合） | 08 |
| D3 | 架构层次 | **彻底改三层**：模板 → 子系统 → 仿真总成 | 03/04/05/10 |
| D4 | 基线重录 | **可以重录基线**（但须逐项登记） | 05/06/08/09/11 |

## 证据（制定计划前实测）

主代理与两个 explorer 子代理在制定本计划前完成了实测，关键结论：

- **内核副注册表已完备**（`cpp/src/contract/contract_registry.cpp:23-30`）：10 项，含 8 种真实副 + 2 种驱动坐标。**内核侧无需改动即可支持任意总成用任意副。**
- **截断点唯一**：`cases/kc_quasi_static/contract.py:44-48` 的 `_JOINT_KINDS` 只映射 3 种副；装配层 `types.py` 与 schema `IdealJointSpec` 均已支持 8 种。**02 的实质是拆截断，不是建底座。**
- **`vertical_linear` 是死名字**：在 `contract_registry.cpp:36` 名单里，但内核解析表（`cpp/src/cases/contract_model.cpp:948-950`）只有 `native_brush`/`pac2002`/`fiala`，枚举 `VehicleTireModelKind`（`cpp/include/mb_model/enums.hpp:61-70`）也只有 4 项。**D1 正确绕开了它。**
- **垂向退化已有底座**：`cpp/src/tire/fiala/forces.cpp:20-29` 的 `fiala_elastic_force` 无曲线时即 `tire.k * penetration`。
- **内核 `Tire` 无质量字段**（`cpp/include/mb_model/types.hpp:101-146`）；质量现挂轮端 body（`schema/vehicle.py:60` 的 `WheelSpec.mass` → `preparation/assembly/vehicle.py:653,722` 合并）。**D2 是真数据结构改动。**
- **契约 schema 对 tire 闭合**：`multibody_model.schema.json` 的 tire 定义 `additionalProperties: false`，08 新增字段必须先改 schema。
- **生效的 native 边界是文档契约**（`kernel/__init__.py:186-205` 传两份 JSON payload），不是 struct ABI。
- **C 模式零刚度占位衬套实测**：`benchmark_axle.json` 下 8 条衬套刚度范数**全为 0.0**（`front_axle.py:737`、`:802`）。
- **K/C 映射实测**（现役 `symmetric_proxy`）：K `constraints=13`/`ideal=13`/`bushings=0`/文档 `elements=0`；C `constraints=9`/`ideal=17`/`bushings=8`/文档 `elements=8`。
- **KC 与 axle_dynamic 实测七项差异**（不止"求解器与轮胎"）：结果对象、模型 schema、轮胎有无、K/C 有无、case 结构、时间网格语义、驱动坐标命名。
- **现有 7 个 (总成, 试验台) 组合**（`simulation/dispatch.py:20-33`）：`axle×{kc_quasi_static, axle_dynamic}`、`vehicle×{vehicle_dynamic, vehicle_kc, handling, ride_four_post, ride_random_road}`。
- **`report/metrics` 共 27 个指标函数**（axle 3 + case_specific 12 + common 8 + vehicle 4），是 07 的迁移对象。
- **全量套件实测**：`783 passed, 1 skipped, 1 xfailed`（退出 0）；`--strict --final` 0；kernel 15；contracts 22；动态哈希 26/26 逐位一致；K/C parity 0；8 family accepted；ruff/ty 0；两包 build 0；`git diff --check` 0。

## 计划结构

`SUBTASKS.csv` 12 个子任务，依赖链无环（已用脚本校验）：

```
01 基线冻结
 ├→ 02 统一副底座 ─┬→ 03 模板数据模型 → 04 子系统拆分 → 05 K/C 列激活 → 06 属性文件 ─┐
 │                 └→ 07 输出声明（与 04-06 并行，写范围不相交）                    │
 └→ 08 轮胎质量（与 03-07 并行，写范围不相交）─────────────────────────────────┴→ 09 study 合并 → 10 试验台正交
                                                                                   ↑          ↓
                                                              07 输出声明 ─────────┘   11 整车实验总成
                                                                                            ↓
                                                                                       12 终局验收
```

**并行约束**：03 与 04 都触及 `preparation/assembly/types.py`，必须串行；05 与 06 在 `templates/**` 上重叠，必须串行；08 只写 C++ 内核 + 契约 + native 镜像，与 03-07 不相交；07 只写 `outputs/**` 与 `report/metrics/**`，与 04-06 不相交。

**写入范围冲突已在各子任务 SPEC 显式声明**（"不写"清单），防止并行写者互相覆盖。

## 本轮计划自检结果

- `SUBTASKS.csv`：13 行（含表头）、**11 字段一致**、12 个子任务、id 唯一、`depends_on` 无环且依赖存在、状态全 `TODO`、类型全 `single-full`。
- 12 个子任务目录均含 `SPEC.md` + `TODO.csv` + `PROGRESS.md` + `raw/`。
- 12 个子任务的 `TODO.csv` 全部**8 列一致**、id 连续、状态全 `TODO`、`completed_at` 空、`retry_count=0`；叶子步骤数 9-11 行。
- 各 `PROGRESS.md` 均写明「0/N 步骤 TODO，尚未实施」与恢复信息。
- 本轮**未**把任何实施行置为 DONE，**未**写生产代码，**未**重录任何基线。

## 计划修订记录

### 2026-09-22 首轮（规划交付）

- **改了什么**：新建本 Epic 全部文件（`EPIC.md`、`SUBTASKS.csv`、本文件、12 个子任务三件套）。
- **为什么**：用户要求「先制定计划，不具体实现代码」，并给出 13 条需求补充与 4 项裁决（D1-D4）。
- **影响**：无既有代码改动；这是一条新 Epic，与 20260921 系列的架构偏差收敛 Epic 无冲突（后者已 9/9 DONE 并收口）。
- **修订**：按 fixer 子代理回报的实测值修正了两处事实——`WheelSpec.mass` 在 `schema/vehicle.py:60`（原写 64）；契约 schema 对 tire 是 `additionalProperties: false`，08 必须先改 schema。并据此收窄了 08 的写范围（Python 侧发射落点归 02/04-06，避免并行冲突）。

### 2026-09-22 独立审核后的修订（4 个阻断项全部处理）

独立审核 `code-reviewer cac8e120` 就本 Epic 的规划给出 4 个阻断项，逐项处理如下：

| 阻断 | 内容 | 处理 |
|---|---|---|
| B1 | **整车实验总成无子任务承接**（用户需求 13 后半「前后悬架+转向+车身+轮胎+整车 kc 试验台→整车实验总成」落空） | **新增子任务 11「整车实验总成组装」**，原终局验收顺移为 12；EPIC 的 G2、三层结构图、依赖图、Done-When 与端到端判据同步补整车侧 |
| B2 | **K/C 激活规则与实测矛盾**：G3 与 05 SPEC 原写「C 模式只激活衬套列、未激活列不产出任何对象」，照此实现会丢掉 C 模式必须保留的 9 个 joint | 实测确认（C `constraints=9`：外点 4 + 拉杆 4 + rack 导轨 1）后改写 G3 与 05 SPEC 目标 2 的激活规则：**有哪列激活哪列，仅 joint 列的点两模式都保留**；并在 03 SPEC 补实测映射细节（K 模式仅 `inner_front` 有副、`inner_rear` 无副），防止实现者从需求 2 的字面推断 |
| B3 | **10 缺对 07 的依赖**（10 要接入 07 定义的输出声明与合并） | `depends_on` 改为 `07;09`，并在 10 SPEC/SUBTASKS 注明依赖理由 |
| B4 | **D2 的 vehicle 侧发射落点无归属，且质量归属无独立终局判据** | 整车侧落点归 11（`cases/vehicle_dynamic.py`、`preparation/vehicle_dynamic.py`），轴侧归 09（`cases/axle_dynamic.py`）；EPIC Done-When 新增第 8 条「轮胎质量归属需独立于 08 自证」，端到端判据补 (e) 整车实验总成、(f) 质量归属 |

**可选改进的处理**：O1（03 SPEC 内点描述不精确）已修；O2（SUBTASKS 行级命令弱于 SPEC 门禁）部分处理——05 行的命令已改为跑自己的 `tests/instantiation`，其余行保留步骤级为完整门禁；O3（types.py 冲突描述为幻影）保留 EPIC 的串行要求但已在 03/04 SPEC 说明实际写范围不相交；O4（native dll 由多个 build 门禁重写）已登记；O5（只测 K→C 单向）已补「双向等价」要求。

**注意**：本轮修订曾一度引入依赖环（09→11→10→09），已修正为 09 不加 11、整车侧落点归 11；修正后 `depends_on` 无环（脚本校验）。

## 未闭合项

无。本轮为规划交付，未产生实施性未闭合项。D1-D4 四项裁决已全部登记并落到具体子任务。

## 下一步

启动子任务 01（冻结现状基线与可执行验证命令）。01 完成前，02-11 不得开工。
