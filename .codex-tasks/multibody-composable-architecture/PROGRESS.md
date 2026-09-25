# EPIC 进度与恢复入口

## Recovery
- 任务：实施可组合多体架构。
- 形态：epic。
- 进度：规划 3/3 完成；实施 **13/13 DONE**（01-13 全部完成并通过各自验收）。
- 当前：全部子任务 DONE；13 的独立终局验收 `accept_composable_architecture.py --strict` 退出码 0（29/29）。
- 文件：EPIC.md、DESIGN.md、TASKS.md、SUBTASKS.csv；规划记录见 planning/；各任务证据见 tasks/NN-*/。
- 12 已做：删除 02 遗留的两个转发壳（`preparation/geometry.py`、`preparation/assembly/types.py`）并把 5 处生产+14 处测试调用改指 `modeling/primitives`，另删除只剩 `__pycache__` 的 `core/`、`model/`、`metrics/`；更新 README/CONTEXT-MAP/K MODULES+K README/CONTEXT 术语表；发布 `docs/composable_extension_examples.md` 三个可执行示例；新增 `scripts/check_composable_release.py`（迁移清单双向核对、文档示例执行、文档根一致、三 wheel 构建+离线隔离安装+源码路径外 native 运行）。
- 12 未闭合（如实登记）：`legacy_surface_gate --final` 不可达。它要求 `elements`（A1）与 `analysis`（A2）消失，而两者都有现役生产调用且 native 尚不能承载（`element_wrench` 对固定体端早退；ABI 无静力入口且导出面冻结）。理由、现役 import 点与解除条件已写进 `packages/suspension_multibody/README.md`，由发布探针双向核对登记表与实测发现一致。
- 13 已做：新增 `scripts/accept_composable_architecture.py`；**不改实现、不重录基线**。`--strict` = native 前置 1 + selection 10 + probe 10 + 冻结命令 8 = 29/29 通过。probe 自带断言且构造不出来即失败（不 skip）；过程中修正 6 处自身判据/API 误用（含 A7 probe 直连 `run_contract` 违反公共 API 边界门，已改走 `run_compiled`）。
- 后续顺序：无。13 是整体验收 gate，已通过。待用户确认后一次性提交。

## 本轮实施记录（自主执行）

| 子任务 | 主验收 | 全量回归 | 关键实测 |
|---|---|---|---|
| 01 基线冻结 | `check_composable_baseline.py --check` 退出码 0 | 1052 passed/1 skipped/1 xfailed | 16 条命令全 0；两个目标缺口实测仍存在并归属 |
| 02 导入解环 | `pytest tests/modeling tests/architecture/test_import_boundaries.py -q` → 71 passed | 1135/1/1 | 真实环 `elements↔front_axle` 已解；`modeling/` 建立；包根改 PEP 562 惰性 |
| 03 模板产出 | `pytest tests/templates tests/instantiation tests/properties -q` → 75 passed | 1135/1/1 | 模板真实产出 ModelFragment（K 13 joints/4 forces，C 9/8）；两条作者路径片段相等 |
| 04 端口与全局规则 | `pytest tests/connections -q` → 37 passed | 1172/1/1 | 歧义拒绝、硬点驱动挂载、D3 规则单一权威；修复 02 遗留的 `results↔preparation` 真环，入口清单扩为 24 项 |
| 05 物理试验台模板 | `pytest tests/rigs -q` → 55 passed | 1214/1/1 | `rigs/bench.py`：试验台按能力分支真实产出实体（轮供给型 3 bodies/2 joints/2 tires，整车型 1 body/0 tires），不按名字分支 |
| 06 子系统与 SI 总成 | `pytest tests/subsystems tests/studies tests/vehicle_assembly tests/tire_mass -q` → 97 passed | 1234/1/1 | `subsystems/composition.py` + `si_assembly.py`；K/C 两模式 bodies 顺序、points 键集、约束数(13/9) 与既有装配逐项一致；指纹含几何且与顺序无关 |
| 08 内核能力注册 | `pytest test_registry_consistency.py contracts/tests -q` → 36 passed | — | case family 知识由三处收敛为单一描述表；`mb_cases_selftest` 41 项；未新增 ABI 与依赖边 |
| 09 原生状态接口 | `pytest packages/suspension_kernel/tests -q` → 32 passed | — | 核实 D10/D14 已落地；新增 `tire_state_selftest` 123 checks 与 2 项 Python 门禁（含禁止求解器按轮胎模型枚举分支）；未改算法与 ABI |
| 07 正交请求与契约编译 | `pytest tests/simulation tests/studies -q` → **111 passed** | 见下 | 新增 `compilation/` 三层；取消 rig==family 强制相等；**GAP-1 闭合**（k=200→Fn=8000N，k=400→Fn=16000N，属性改变响应）；生产桥接只剩输入适配器一个调用者；kc_parity 无漂移、case_parity 8 families PASS |
| 10 输出与公共 API | `pytest tests/results tests/outputs tests/metrics tests/architecture -q` → **233 passed** | 1277/1/1 | 新增 `results/kc_state.py` 承接解码；清除 api 自有列下标常量；**GAP-2 闭合**（无转向运行 3 states 收敛且 `drives` 无 rack_displacement，有转向仍含；内核 shorthand rack 轴改为可选）；`tests/api` 20 passed（新增 17）；kc_parity 无漂移、case_parity 8 families PASS |
| 11 扩展性实证 | `pytest tests/composable -q` → **16 passed** | 见下 | 编译器去掉模板零件名假设（`wheel_centre_body` 按 `wheel_center` 点声明定位）；显式拓扑角色由产出推导；合成拖曳臂（2 约束 vs 双横臂 13，无 rack/tie_rod）经 `api.run_case` 实跑，轮心 z 与独立旋转公式 0.05 mm 内一致；硬点+20 mm 传导至解；合成加载台力元进入求解 |
| 12 清理、文档与打包 | `check_composable_release.py` 退出码 0（4/4） | 1293/1/1 | 删除两个转发壳与三个空壳目录（5 处生产+14 处测试调用改指 `modeling/primitives`）；四份现状文档更新；三个可执行示例；迁移清单双向核对；三 wheel 构建+离线安装+路径外 native 收敛 |
| 13 独立终局验收 | `accept_composable_architecture.py --strict` 退出码 0（29/29） | 1293/1/1 | A1-A10 各自独立构造并实跑：A2 动态实跑、A6 轮胎法向力随刚度变化、A7 三个 C++ 自测 39/41/123 checks、A5 两 schema 同一 SI 模型 900.0 kg、A8 无转向结果无 rack 通道、A10 路径外 native 运行；冻结数值/性能门全部按原值通过 |

未提交：按用户裁决「全部完成后一次性提交」，本轮改动仍在工作区。

## 后续任务须知的现状

- `modeling/`（低层）、`templates/builders.py`（模板产出）、`connections/`（匹配/几何/policy）、`subsystems/composition.py`+`si_assembly.py`（SI 总成）四层已就位且互相不反向依赖；10/11 应在其上构建，不要再造平行抽象。
- **`compilation/` 已建立**（07）：`plan.py` 管运行是什么、`model_view.py` 管模型是什么、`compile.py` 管两者蕴含的文档。family 名只作为选 emitter 的注册键。10/11 复用这三层，不要在 compiler 里重新按 family 分支。
- `SimulationRequest` 的 rig 与 family **不再强制相等**；`RigSpec` 有 `family` 字段与 `rig_family()`。新增试验台只需声明 `family`，不必改名。
- `Assembly` 有 `physical` 回指：组合层拥有身份与端口，物理实体属于被组合的 build。读模型时两者各取所属。
- **`results/kc_state.py` 是 K/C 解码的唯一入口**（行切片、米转毫米、块列名）。10 之后 `api` 不再自己切片，也不持有任何列下标常量。原生列的九张表仍在 `axle_dynamics/result.py`，`report`/`outputs` 仍各自硬编码下标——未收口部分已在 10 的 PROGRESS 中登记。
- `CaseSpec.subsystems`（默认 `None`）是让无转向单轴运行**可经公共入口提出**的输入；`contract.has_rack(assembly)` 是判断装配有无 rack 的唯一依据。
- 导入边界门已覆盖全部 24 个顶层子包与全序对（60 项），新增子包会被自动检查。**新增 `compilation/` 后已实测通过**。
- 开发经验（重要）：本仓库文件用行号式 patch 编辑时**高频出现重复行或吞行**（api.py、si_assembly.py、test_the_rig_reaches_the_product.py 都发生过）。改大文件优先整文件重写，或改完立即用 `ast.parse` + `sed` 复核。

## 真源与优先级
用户本轮 D1-D3 > EPIC Goal/Non-Goals > DESIGN 冻结契约 > TASKS 子任务规格；状态以 SUBTASKS.csv 为唯一实施真源。发现矛盾先修正文档，不由实施者私自选较宽解释。

## 依赖主线
```text
01 -> 02 -> 03 -> 04 -> 05 -> 06 --+
01 -> 08 ------------------------+-> 07 --+
      08 -> 09 --------------------------+-> 10 -> 11 -> 12 -> 13
```
07 必须等待08，编号不是执行顺序。共享源码、schema、CMake、注册表与 native 镜像不得多写者并发。06先验证SI装配，07切换运行，10收敛API，12删除无调用旧路径。

## 本轮裁决
- 只冻结方案，不实施代码。
- 未来允许合成新拓扑和新试验台做实际求解验收，不作为工程产品交付。
- 全局限制不放宽：单轴禁制动/驱动、车轮归试验台；整车必需转向/制动/驱动；新模板和嵌套配方不可绕过。

## 交付与修改边界
新增本方案目录（EPIC/DESIGN/TASKS/SUBTASKS/REVIEW/PROGRESS及规划记录），补充 multibody 领域术语与根导航；按既有规则在 .gitignore 中仅放行本方案目录，确保文件可随仓库提交。
未修改既有10个已修改Python文件及1个未跟踪测试，未实施代码、未改旧任务完成记录、未重录基线、未调用外部服务。

## 验证记录（2026-09-24）
- code-reviewer `8d57d44c-6796-49b3-a5b7-a5e4fce7d201`：通过，无阻断；1项中等与2项低优先级意见已处理，详见 REVIEW.md。修订由主线程复核，不冒称第二轮独立审查。
- `uv run --no-sync python "$PI_SCRATCH_DIR/validate_multibody_plan.py"`：退出码0；13行全TODO，ID/目录唯一，依赖无环；TASKS与CSV的命令/依赖一致；9目标覆盖13任务；A1-A10唯一且G5独立场景齐全；11个本地链接有效，文档无编辑标记/空白问题。
- `git diff --check`：退出码0；仅Git的LF/CRLF提示。
- `git status --short -- .gitignore CONTEXT-MAP.md packages/suspension_multibody/CONTEXT.md .codex-tasks/multibody-composable-architecture`：方案目录可见，导航/术语/忽略例外均可提交。
- 已核对现有动态哈希、K/C parity、case parity与性能脚本参数；性能门默认check，不触发重录。
- 本轮没有运行新架构的实现测试；TASKS所有实现命令是未来验收规范，不是已通过声明。

## 实施时复核与提问
旧任务登记的准静态轮胎与rack输出缺口由01复测并分别归07/10，目标相关缺口不得豁免。偏心轮胎质量、制动参数来源、真实Adams等价不因本方案自动获得支持。
若协议/ABI/API需变更、必须修改物理或数值基线、引入依赖/付费资源或突破总成规则，暂停相关任务并向用户提问。当前无未决的业务选择。
