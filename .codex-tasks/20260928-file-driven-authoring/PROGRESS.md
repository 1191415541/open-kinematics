# Progress

## Recovery

- 任务: 文件驱动的模板、子系统、总成与试验台体系
- 形态: epic
- 当前: 契约 + 模板/子系统/总成/试验台文件层 + 属性文件解析 + 文件→模型桥接 + 项目加载器 + 专家/用户边界 + K/C 求解接入 + 完整 provenance
- 验证: 见下「已验证」，全部为本轮实跑结果
- 文件: `.codex-tasks/20260928-file-driven-authoring/SUBTASKS.csv`
- 下一步（三项 PARTIAL 的顺序）: 1) `compose_axle`/`compose_vehicle` 内部改走通用总成解析（需先确认可接受 K/C 数值面影响）→ 2) 随包内置模板改为文件并保留旧入口 → 3) 旧 `PropertySet` 属性路线与文件路线的收敛

## 已验证（本轮实跑）

| 门 | 命令 | 结果 |
|---|---|---|
| 静态 | `ruff check .` | 通过 |
| 静态 | `ty check .` | 通过 |
| 架构 | `legacy_surface_gate.py --check` | 通过，0 findings |
| 架构 | `check_module_layering.py --strict --final` | 通过，0 环、0 退役 |
| 架构 | `check_composable_release.py --skip-isolation` | 通过，3/3 |
| 快速集 | `pytest tests --ignore=adams,architecture,cases` | **989 passed, 1 xfailed** |
| 契约+内核 | `pytest kernel/tests contracts/tests` | 60 passed |
| 数值 | `dynamic_hash_sentinel.py --check` | 与冻结基线逐位一致（sha256 `fdfd5a6b…` 未变） |
| 数值 | `case_parity_check.py` | 8 families PASS |
| 数值 | `kc_perf_gate.py` | 在预算内（k-100 0.64s / c-66 0.86s） |
| 空白 | `git diff --check` | 干净 |
| 定向 | `pytest tests/authoring` | **49 passed** |

**本轮未执行**（不得算作已验证）：

| 门 | 原因 |
|---|---|
| `pytest tests/architecture`（149 用例，约 10 分钟） | 未搬动既有模块；结构证据由三个秒级架构门脚本覆盖（已通过） |
| `just test-all`（约 33 分钟） | 全量含 Adams 对标与整车 K/C；本次未触及轮胎力律与 K/C 网格语义 |
| justfile | 本机无 `just`，数值门与门禁脚本按 AGENTS.md 展开逐条执行 |

## 关键结论（有证据）

- **文件格式与内置模板完全等价**。`template_document_from(DOUBLE_WISHBONE)` 导出到文件、再经 `runtime_template_from` 读回后：19 个连接的每个字段（含 `joint_modes`、`bushing_modes`、`joint_kind_by_mode`、`far_label`、`first_body`、`axis_reference_role`）、23 个槽位、10 个刚体、输出、kind 与 description **逐项相同**；据此装配出的 K 与 C 模型与内置一致（K：13 约束 / 0 衬套；C：9 约束 / 8 衬套，约束名与刚体集合相同）。
- **结果侧 provenance 不改变既有产物**。文件哈希写入独立 `inputs.json` 副作用文件并在 manifest 记一个键，只在提供 inputs 时发生；`dynamic_hash_sentinel` 的 26 个产物 sha256 与改动前完全一致。
- **装配规则只有一个归属**：`connections/policy.py` 的 `ASSEMBLY_RULES`，文件层委托它并保持「禁止角色 → 引用一致性 → 数量与位置」的报错顺序。

## 追踪文件对应

- `SUBTASKS.csv`：9 行（阶段 1–8 + 验收）。DONE：1、2、4、7；PARTIAL：3、5、6、8、9。
- SPEC：`tasks/01-schemas`、`tasks/04-subsystem-instance`、`tasks/08-compatibility`。
- 新增模块：`authoring/`（documents、properties、bridge、solver、project、security、errors）。

## 未完成（PARTIAL 项，逐条说明为何未做）

1. **阶段 3｜旧 `PropertySet` 属性路线未迁移**。`properties/load.py` + `templates/instantiate.resolve_properties` 的 `dict[str, float]` 通道仍在使用，且被 `tests/properties` 覆盖。它是内置 Python 入口的属性来源，其数值参与冻结基线；迁移会改变已录结果，因此按阶段 8「旧入口保持可用」的要求保留。文件路线已完整实现并产出 `property_file_hash`/`effective_values_hash`。
2. **阶段 5｜`compose_axle`/`compose_vehicle` 内部未改写**。规则已单点化在 `connections/policy.py`，文件层完全走它；但两个兼容入口内部仍直接使用既有装配路径。按需求改写会触及冻结的 K/C 数值面，属于需用户确认的范围变更。
3. **阶段 6｜文件未接管试验台物理**。`rig.bench` 可绑定注册试验台（`bench_spec()` 返回 `RigSpec`），加载器/驱动器/测量通道仍由该实现提供——有意为之，避免在文件里重述已被冻结验证的加载语义。
4. **阶段 8｜随包内置模板未改为文件**。导出/回读能力已实现并验证等价（见上），但发布的内置 steering/wheel/brake/drive 模板与既有 fixture 未改成文件；`AssemblyRequest`/`VehicleModel` 保持原样（符合「旧入口保持可用」）。
5. **验收｜两项长门未跑**。架构 pytest 目录与 `just test-all`（见上表）。
6. 未新增求解器、未改 C++ ABI、未重录任何冻结基线。
