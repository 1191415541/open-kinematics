# SPEC：06 可选对称（镜像成为声明）

> 子任务规格。父 Epic：`.codex-tasks/20260929-assembly-layer-rework/EPIC.md`；父行：`SUBTASKS.csv` 的 `06`。

## Task Shape

- **Shape**: `single-full`

## Goals

把「镜像」从底层代码行为提升为**可声明的选项**，同时保住默认路径产物不变（`EPIC.md` G5、验证协议 06、D5）：

1. **两种写法等价**：同一悬架模板，总成文件写「一个条目 + 镜像」与写「左右两个条目 + 各自文件」两种方式，装配产物**逐项比较点坐标与约束端点**（体/点/约束的集合、名字、点坐标、约束端点、数值）后一致。
2. **不对称可装配**：左右不对称硬点（左右不同坐标）能装配并跑通一次，且断言**独立文件不被二次镜像**（比较装配后的点坐标与文件里写的坐标）。
3. **默认不变**：默认（不声明）仍走镜像，产物与 01 快照逐项一致。
4. **D5 裁决落地**：**镜像仍为默认**（文件只写一侧、装配建两侧）；**不对称与左右独立文件走显式声明**。默认路径即所有既有算例的路径，故 D5 等价于「默认零变化 + 新能力走声明」；不得把镜像改成需要声明才生效。
5. **单侧文件契约**：单侧文件的**坐标系、镜像方向与命名契约**要落文档并有测试——`side_hardpoints` / `mirror_hardpoints` 的语义、`placement_role` 取值、左右独立文件的命名规则，用户在总成文件里必须看得出「这一侧是文件写死的还是镜像出来的」。
6. **`SIDES` 收口**：`SIDES` 的消费点全部改为按总成声明取值，装配路径 `grep "_SIDES"` 无残留硬编码；过渡期的兼容路径必须在 **07 之前删除并登记**。

## Non-Goals

- 不改轮胎力律与求解器数值路径（本轮只改「谁产出、谁宣称哪一侧」）。
- 不改模板数据模型（`templates/model.py` 的 parts/connections/属性槽）。
- 不动 `subsystems/rig_link.py` 与 `rigs/`（05 独占）。
- 不实现具体车型物理（NASCAR / 特种工程车只是一个不对称最小用例，不做车型级建模）。
- 不重录任何基线。

## Constraints

- **写范围（照 `SUBTASKS.csv` 的 `06` 行 `notes`）**：`subsystems/suspension.py`、`subsystems/si_assembly.py`、`subsystems/types.py`、`subsystems/geometry.py`、`authoring/documents.py`、schema 的子系统文档新增可选声明（即 `subsystem.schema.json` 一侧）。**不得**改 `subsystems/rig_link.py` 与 `rigs/`。
- **串行约束**：06 与 04 的写范围相交（`subsystems/si_assembly.py`、`subsystems/suspension.py`），故 `depends_on` 设为 `05`，调度上必然后置，**不得与 04 并行**（`EPIC.md`「并行与写范围约束」）。
- **默认零变化的判据**：用 `tasks/20260929-01-freeze/snapshot.py --check` 作为「产物是否变化」的判据（**待 01 的 `#1` 完成后跑**；01 的 7 组合快照尚未生成，`SUBTASKS.csv` 的 `01` 行仍为 `TODO`）。默认路径落地前后各跑一次，差异即「产物被改动」的证据。
- **`model_dump(mode="json")` 产物不得被改变**（`api.py` 的 `model_hash` 依赖它）；新增声明必须是子系统文档的可选字段，缺省时行为等于今天的镜像。
- **装配层不得出现按名字猜身份的规则**（`"chassis"` / `"ground"` / 前缀拼接），对称声明也不得引入新的字符串改写（G2）。
- **不得新增 skip/xfail**；`tests/adams` 的 47 个环境 skip 是既有的，不得增长。
- 每步落地后重跑 `just check-fast`（ruff / ty / 三个架构门 / 快速集 / 另两包）；改结构后重跑 `tests/architecture`；改到求解路径则重跑数值门三项。
- 证据只记**已执行**的结果；未执行的项留 `TODO`，不得先填结论。

## Environment

- **Project root**: `c:/杂件/open-kinematics`
- **Language/runtime**: Python 3.12（`uv run --no-sync`）
- **Package manager**: `uv`
- **Test framework**: `pytest`
- **Build command**: 无（纯 Python；内核 DLL 已构建，本行不动内核）
- **Existing test count**: 快速集 1015 passed / 1 xfailed；`tests/cases` 100；`tests/architecture` 147；`tests/adams` 160/47 skip；kernel+contracts 60

## Risk Assessment

- [ ] **默认镜像路径被改动会波及所有既有几何断言**（`EPIC.md` 风险节）→ 默认值保持镜像、不对称走显式声明并用 01 快照兜底。
- [ ] **独立文件被二次镜像**：这是 D5 判据里最容易静默失败的一条 → 必须有「装配后点坐标 == 文件写的坐标」的断言，而不只比实体名字集合。
- [ ] **两种写法的等价判据太弱**：只比名字集合会漏掉镜像方向错误 → 必须逐项比点坐标与约束端点（`EPIC.md` 验证协议 06 (a)）。
- [ ] **坐标系/镜像方向契约只写在代码里**：这正是 R4 的成因（用户在文件里看不出）→ 必须落文档 + 测试。
- [ ] **`_SIDES` 消费点定位不全**：F6 已知消费点为 `subsystems/suspension.py`、`subsystems/si_assembly.py`、`subsystems/wheel.py:114-116 sides()`、`subsystems/element_build.py:205-210`；落地时以 `grep` 实测为准，`wheel.py` / `element_build.py` 不在本行写范围，若必须触及则改归 04 并在本行 PROGRESS 登记。
- [ ] **既有对称性测试只有 `tests/model/test_symmetry.py`**（F6），无任何不对称/单侧用例 → 新用例是本行必须产出的一部分。
- [ ] **01 快照尚未生成**（01 的 `#1` 在 `tasks/20260929-01-freeze/TODO.csv` 里仍为 `TODO`）→ 「默认路径与 01 快照逐项一致」在本行结束时只能以「快照落盘后补跑」的形式记录。

## Deliverables

- 子系统文档的**可选对称声明**（schema 侧新增可选字段）与装配侧消费路径；缺省行为 = 今天的镜像。
- 测试：
  - 镜像写法与「左右两条目 + 各自文件」写法的装配产物逐项比较（点坐标 + 约束端点）一致；
  - 左右不对称硬点装配并跑通一次，且断言独立文件不被二次镜像；
  - 默认路径产物与 01 快照逐项一致（快照落盘后补跑）。
- 文档：单侧文件的坐标系、镜像方向与命名契约；`side_hardpoints` / `mirror_hardpoints` 语义；`placement_role` 取值；左右独立文件的命名规则。
- 登记：`_SIDES` 硬编码消费点的清除记录与过渡兼容路径的删除登记（必须在 **07 之前**完成）。
- 验证记录：`just check-fast`、01 快照 `--check` 的前后对比（写入本子任务 `PROGRESS.md`）。

## Done-When

- [ ] 镜像与左右独立文件两种写法的装配产物逐项比较（集合、名字、点坐标、约束端点、数值）一致。
- [ ] 不对称硬点（左右不同坐标）能装配并跑通一次，且独立文件不被二次镜像有断言。
- [ ] 默认（不声明）仍走镜像；01 快照落盘后 `snapshot.py --check` 通过（默认路径产物未变）。
- [ ] `grep "_SIDES"` 在装配路径无残留硬编码；过渡兼容路径已删除并登记。
- [ ] 单侧文件的坐标系/镜像方向/命名契约在文档中有明确表述，且有测试覆盖。
- [ ] 无新增 skip/xfail；`ruff` / `ty` / 三个架构门退出码 0。

## Final Validation Command

```bash
cd /c/杂件/open-kinematics && \
uv run --no-sync pytest packages/suspension_multibody/tests/subsystems packages/suspension_multibody/tests/model packages/suspension_multibody/tests/authoring -q && \
uv run --no-sync python .codex-tasks/20260929-assembly-layer-rework/tasks/20260929-01-freeze/snapshot.py --check && \
uv run --no-sync ruff check . && \
uv run --no-sync ty check .
```

> `snapshot.py --check` 是「产物是否变化」的判据（待 01 的 `#1` 完成后跑）。
> `grep` 收口判据：装配路径 `grep "_SIDES"` 无命中。

## Demo Flow

1. **默认路径**：总成文件只写一侧悬架条目 → 装配 → 与 01 快照逐项一致（证明默认零变化）。
2. **两种写法**：同一模板分别写「一个条目 + 镜像」与「左右两条目 + 各自文件」→ 两次装配 → 逐项比较点坐标与约束端点，一致。
3. **不对称**：左右硬点写不同坐标 → 装配 + 跑通一次 study → 断言装配后的点坐标等于文件写的坐标（未被二次镜像）。
4. **判据复核**：落地前后各跑一次 `snapshot.py --check`（待 01 的 `#1` 完成后可跑），差异即产物变化证据。
