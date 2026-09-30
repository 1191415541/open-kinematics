# PROGRESS：02 显式接口配对落地

> 父 Epic：`.codex-tasks/20260929-assembly-layer-rework/EPIC.md`；父行：`SUBTASKS.csv` 的 `02`

## Session Start

- **Date**: 2026-09-29（实施完成）
- **Task name**: 20260929-02-port-binding
- **Task dir**: `.codex-tasks/20260929-assembly-layer-rework/tasks/20260929-02-port-binding/`
- **Spec**: 见 `SPEC.md`
- **Plan**: 见 `TODO.csv`（6 步）
- **Environment**: Python 3.12 / uv / pytest

## Context Recovery Block

- **Current milestone**: 全部完成（#1–#6）
- **Current status**: DONE
- **Last completed**: #6 — 产物不变判定（`snapshot.py --check` 零差异）
- **Current artifact**: `connections/links.py`、`subsystems/composition.py`、`raw/binding_evidence.md`、`raw/removal_register.md`、`raw/snapshot_diff.md`
- **Key context**: 机制落点是 `connections/links.py` 的 `LinkSpec` + `build_links`（把 bindings 变成 `Connection` + 对应 `WeldJoint`/`RevoluteJoint`/`BushingElement`），由 `subsystems/composition.py::compose_simulation_assembly` 在 `match_requirements` 之后调用；**贡献里没有配方时生成行为空**，所以既有产物零变化（01 快照 `--check` 仍然零差异）。文档侧：条目可选 `pairings`（`{"requirement_role","port"}`），`AssemblyEntry.pairings` 读入，`AssemblyRequest.pairings` 承载，最终经 `explicit_bindings_from_pairings` 渲染成端口 id 交给 matcher。**03 消费方式见本文件末尾「与 03 的契约边界」**。
- **Known issues**: `authoring/documents.py:76 _ASSEMBLY_SUPPLIED_BODIES` 的白名单仍按名字（`chassis`/`ground`/`rack`/`rack_housing`）放开引用校验——本行登记为「保留 + 交 03 改为按角色/端口推导」，理由见 `raw/removal_register.md` 的 B 节。
- **Next action**: 无（本行完成）；03 在整车路径消费本契约并复验端到端。

---

## Final Summary

6 步全部 DONE。交付：`connections/links.py`（新）、`subsystems/composition.py` 与 `subsystems/types.py` 的接线、`assembly.schema.json` 的配对段、`authoring/documents.py` 的配对读取、4 个测试文件（`tests/connections/test_links.py`、`tests/authoring/test_assembly_pairings.py`、`packages/suspension_contracts/tests/test_assembly_pairings.py`，以及既有目录内的联动断言），证据落在 `raw/`。判据实测：`just check-fast` 退出 0（快速集 1030 passed / 1 xfailed、kernel 33、contracts 32、ruff/ty/三个架构门全绿）；`connections` + `authoring` + `subsystems` 241 passed；`snapshot.py --check` 零差异（退出 0）。

**未按行验收命令原样执行的两点**（登记，不掩盖）：
1. 父表的 `validation_command` 把 `packages/suspension_contracts/tests` 与 multibody 的三个目录写在**同一次** `pytest` 调用里，实测会改变 rootdir 并使 multibody 侧的 `from tests.benchmark_fixture import ...` 解析失败（`AGENTS.md` 第 1 节已记载该坑）。本行改为**分开两次调用**并已把父表命令同步修正。
2. 全量 `pytest packages/suspension_multibody/tests` 未跑（用户明确指示不跑全量，约 33 分钟）；改用 `just check-fast` + 改动目录的针对性 pytest。

---

## 2026-09-29 计划修订（复核后）

- **改动**：`preparation/vehicle_dynamic.py:373` 的归属由 04 改为 **03**；三处字符串改写锚点（`vehicle_assembly.py:212-228`、`vehicle_parts.py:172-173` 与 `:414`、`preparation/vehicle_dynamic.py:373`）统一表述为「归 03，终局判定由 03 收口、07 复验」；`snapshot.py --check` 的判据改为 01 的「未变化部分逐项相等 + 已登记差异」（登记处 `raw/approved_deltas.json`）并写明「**02 不得产生差异，差异非空即失败**」（只有 05 被允许改变既有产物并登记）；Done-When 的 actor 表述统一为「本行自有路径（五个路径列全）」。**本节取代本文件 Known issues 中「三处落在 03/04 的写范围内」的旧表述（该行原文按约定保留）。**
- **为什么**：Epic 开工前独立审核（code-reviewer）提出 7 项阻断项，父真值文件 `EPIC.md` / `SUBTASKS.csv` 已修订——`:373` 与另两处锚点同属 03 的写范围，02 只负责自有五路径；01 的比对口径在该次修订中明确为「未变化部分逐项相等 + 已登记差异」。
- **影响**：`TODO.csv` 第 5 行与第 6 行；`SPEC.md` 的 Goal 6/7、Constraints 的跨行归属段、Risk 的字符串规则与命名语义两条、Deliverables 的 `raw/snapshot_diff.md`、Done-When、Final Validation Command、Demo Flow。

## 2026-09-29 计划修订（第二轮复核后）

- **改动**：`SPEC.md` 的 Goal 7 与 Done-When 的 `snapshot.py --check` 条、Final Validation Command 说明，统一把「待 01 的 `#1` 完成后跑／脚本尚未生成」改为「**01 已交付**（`snapshot.py` 与 `raw/assembly_snapshot.json` 已落盘，两次运行 sha256 同为 `b58d35ac…`），落地前后直接跑」，并写明**02 不得产生差异，差异非空即失败——登记只能由 05 写**。
- **为什么**：01 的第 1 行已 `DONE`，父真值文件 `SUBTASKS.csv` 与 `EPIC.md` 已同步；沿用「尚未生成／待 #1」会让本行的产物不变判据看起来仍不可执行，且「差异非空即失败」的归属边界需要与 05（唯一被允许改变产物的行）对齐。
- **影响的行 id**：02 的 #6（`TODO.csv` 的 notes 由「待 01 的 #1 完成后再跑；脚本不存在时登记为 BLOCKED」改为「01 已交付、直接跑」）；`SPEC.md` 的 Goal 7、Done-When、Final Validation 说明。行数不变、status 仍为 `TODO`。**本节取代本文件 Key context 中「01 的 `#1` 未完成，脚本尚未生成」的旧表述（该行原文按约定保留）：01 已交付，`snapshot.py` 与 `raw/assembly_snapshot.json` 已落盘。**

---

## 2026-09-29 实施记录（02 落地）

**过程**：两次派发的 fixer 分别「只设计不动手」与「动了一半并把 `compose_simulation_assembly` 的 `root_kind` 形参误删」。主代理接管收尾：修复签名回归、把配对读进 `AssemblyDocument`、补测试与证据、跑门禁并登记。**「曾经坏过、已修好并复验」这一事实写在 `raw/snapshot_diff.md` 的末尾，不作为「从未失败」陈述。**

**改动（文件:行，一句话）**：

| 文件 | 行 | 改了什么 |
|---|---|---|
| `connections/links.py` | 1–335（新） | `LinkSpec` / `LinkRow` / `LinkRows` / `build_links` / `explicit_bindings_from_pairings`：把 bindings 变成实体行，无配方即空 |
| `subsystems/composition.py` | 194–205 | 形参加 `pairings`（并与既有 `root_kind` 并存） |
| `subsystems/composition.py` | 92–105 | `SubsystemContribution.links: tuple[LinkSpec, ...]`（默认空） |
| `subsystems/composition.py` | 265–300 | 匹配前把文档配对渲染成 explicit 映射；匹配后 `build_links` 并入 fragment |
| `subsystems/composition.py` | 335–345 | 生成的 `LinkRow` 随 `SimulationAssembly.generated["links"]` 一起走，便于审计 |
| `subsystems/types.py` | 164–172 | `AssemblyRequest.pairings`（默认 `{}`，不进任何 `model_dump`） |
| `assembly.schema.json` | 37–42、64–73 | 条目层可选 `pairings` + `$defs/pairing`（两端都必填、不许多余字段） |
| `authoring/documents.py` | 642–657 | `AssemblyEntry.pairings` |
| `authoring/documents.py` | 710–735 | 读入配对；同一 `requirement_role` 出现两次即报错点名 |
| `tests/connections/test_links.py` | 新 | 11 条断言：显式=推断、`subframe` 可配对、歧义/悬空/未绑定点名、三类配方、零配方零行 |
| `tests/authoring/test_assembly_pairings.py` | 新 | 4 条：无配对读成空、有配对落到条目、重复 role 拒绝、未知字段拒绝 |
| `packages/suspension_contracts/tests/test_assembly_pairings.py` | 新 | 5 条：契约层的正例与四类负例（缺 port、未知字段、条目层仍封闭） |

**验证（真实退出码）**：

| 命令 | 结果 |
|---|---|
| `just check-fast` | **退出 0**；快速集 1030 passed / 1 xfailed、kernel 33、contracts 32、ruff/ty/三个架构门全绿 |
| `pytest tests/connections tests/authoring tests/subsystems -q`（一次调用，同一包内） | **241 passed** |
| `pytest packages/suspension_contracts/tests -q`（单独调用） | **32 passed** |
| `snapshot.py --check` | `OK`，**退出 0**（零差异） |

## 与 03 的契约边界

03 消费本行时只需要知道四件事：

1. **需求→端口的配对写在哪**：总成文件的子系统条目里 `pairings: [{"requirement_role": …, "port": …}]`（`port` 是**本装配提供的端口名**，解析成端口 id 由 `explicit_bindings_from_pairings` 负责，名字不存在时点名报错）。留空＝按角色/能力唯一匹配，既有文件不受影响。
2. **连接怎么被建出来**：贡献方声明 `SubsystemContribution.links: tuple[LinkSpec, ...]`，`LinkSpec(role, kind ∈ weld/revolute/bushing, body_a, point_a_local, point_a_label, …)`；另一端默认取匹配到的端口（其 `owner` 体 + `pose` 平移 + 局部名），可用 `body_b`/`point_b_local`/`point_b_label` 覆盖。
3. **生成物长什么样**：`compose_simulation_assembly(...).generated["links"]` 是 `LinkRow` 元组（`row_name`、`row_type`、`kind`、两端体/点标签与坐标、命中的 `port_id`），同时并入合并 fragment 的 `joints`/`ideal_constraints`/`bushings`/`connections`。
4. **零配方＝零变化**：03 若给既有路径加配方，就会改变产物，因而必须登记；若只是把「按名字改写」换成「按端口配对」，则应保证生成行的集合与今天的 `body_map` 结果逐项一致（这正是 02 在机制层已证的性质：`raw/binding_evidence.md`）。
