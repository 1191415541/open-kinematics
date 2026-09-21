# 子任务 06：迁移 Python 作者层声明并把 API 切到 native 元件事实

## 目标

完成 Python 作者侧职责归位，并把元件报告切换到 05 冻结的 native 事实：关节残差/Jacobian 的删除无条件执行；力元本构与力汇总的删除按 05 可选通道的实际启用状态处置（**A1 修订**，见下方约束与验收 5）。

1. `model/front_axle.py`、`model/vehicle.py` 的硬点镜像、命名与 schema→声明转换 → `preparation/assembly/{front_axle,vehicle}.py`：只作者侧转换。
2. `core/` 的关节/刚体/元素数据字段 → `preparation/assembly/types.py`：数据对象不携带 `residual`/`jacobian`/`evaluate`。
3. `core/spatial.py` 的现役输入坐标转换 → `preparation/geometry.py`；结果侧坐标转换 → `results/geometry.py`；复核调用方向，避免 `report`→`preparation`。
4. `analysis/time_signals.py` → `preparation/signals.py`（属于输入信号采样，不是报告）。
5. `pac2002_scope.py` 的能力读取 → `kernel/capabilities.py`（惰性读取），schema 的校验辅助与 Adams 证据说明分别归 `schema` 与 `adams`；旧顶层模块的删除在 08。
6. `api.py` 的元件报告取值切换到 native 事实后，删除 `elements/elastic.py`、`elements/assembly.py` 的本构与力汇总。

## 非目标

- 不建立 `report`、不迁移 metrics/analysis 的统计与柔度（07 负责）。
- 不删除 `core`/`elements`/`model`/`analysis`/`metrics` 目录与旧 `pac2002_scope.py` 文件本体（08 负责），本任务只把生产调用方全部切走。
- 不改公开 API 与 CLI 契约、不改 Adams source rendering 行为、不改七 family preparation/document bypass、不丢历史 artifact 读取。
- 不为迁就本任务删除仍有价值的测试；测试只改归属不改断言意图。

## 约束

- 删除 `elements/elastic.py` 的 `evaluate` 与 `elements/assembly.py` 的力汇总之前，必须先有 05 的通道级证据。**（2026-09-21 用户裁决 A1 修订）**：05 交付的是「默认关闭的可选 native 力旋量通道」。在可选通道未启用期间，本构的删除**不构成本任务的完成条件**；本任务须先完成 1-5 项（作者层归位、geometry/signals 拆分、pac2002_scope 迁移）与第 7 项回归，第 6 项按可选通道的实际状态执行：通道已启用则删除，未启用则登记为「待通道启用后删除」并明示理由。不得为删除本构而启用可选通道（那会改变默认 artifact 字节）。
- `preparation` 只做作者侧数据与单位/信号转换，不求解、不调用 native、不解码结果。
- 若发现 05 未覆盖的力律差异，登记并阻断切换，不得在 Python 侧保留第二套力律或"仅供报告"的近似。
- `api.py`、包 `__init__.py`、`schema` 是共享写面：本任务串行修改，不得与 07/08 并行写同一文件。

## 范围与文件归属

- 可写：`packages/suspension_multibody/src/suspension_multibody/preparation/**`、`kernel/capabilities.py`、`schema/**`、`adams/**`、`api.py`、`elements/{elastic,assembly}.py`、`core/**` 中被迁走后的残留调用面，以及 `tests/{simulation,api,adams,cases,core,model,schema,elements,physics}/**` 中随归属调整的断言。
- 只读：`packages/suspension_multibody/src/suspension_multibody/results/**`（05 已定稿）、`packages/suspension_multibody/src/suspension_multibody/analysis/**`（07 负责）、父 `EPIC.md`、`VALIDATION.md`。
- 不写：`report`（07 新建）、父 `EPIC.md`、`SUBTASKS.csv`、`PROGRESS.md`。

## 依赖

- 前置：01（`VALIDATION.md`）、02（职责边界门禁）、03、04、05（native 通道证据与 results 解码边界冻结）。05 未完成不得开工。
- 后续：07 依赖本任务后 `analysis`/`metrics` 的剩余职责清晰；08 依赖本任务把旧调用方清零后删除目录。

## 验收标准

1. `preparation` 只做作者侧数据与单位/信号转换：不求解、不调用 native、不解码结果，有架构测试断言。
2. `preparation/assembly/types.py` 的数据对象不携带 `residual`/`jacobian`/`evaluate`；`core/constraints.py` 的残差/Jacobian 不再被 production 调用。
3. `preparation/geometry.py` 与 `results/geometry.py` 分工明确，无 `report`→`preparation` 反向调用。
4. `analysis/time_signals.py` 的能力在 `preparation/signals.py`；`pac2002_scope` 的能力读取在 `kernel/capabilities.py` 且为惰性，旧顶层模块已无生产调用方。
5. **（A1 修订）** `api.py` 的元件报告在**可选通道启用时**来自 native 事实。`elements/elastic.py` 的 `evaluate` 与 `elements/assembly.py` 的力汇总：可选通道已启用则必须删除且无其它本构实现；未启用则保留但须有显式的「待删除」登记（file:line + 阻断原因 + 启用条件），并不得在 report 侧复算本构。
6. 公开 API 与 CLI、Adams source rendering、七 family preparation/document bypass、结果异常与 partial、历史 artifact 读取全部回归通过。
7. `tests/simulation`、`tests/api`、`tests/adams`、`tests/cases` 全绿；无新增失败。
8. 未提前删除任何目录或 `pac2002_scope.py` 文件本体（删除属 08）。

## 验证协议

```bash
uv run --package suspension-multibody pytest packages/suspension_multibody/tests/simulation packages/suspension_multibody/tests/api packages/suspension_multibody/tests/adams packages/suspension_multibody/tests/cases -q
uv run --package suspension-multibody pytest packages/suspension_multibody/tests/architecture -q
uv run --package suspension-multibody pytest packages/suspension_multibody/tests/schema packages/suspension_multibody/tests/io -q
uv run python packages/suspension_multibody/scripts/build_axle_native.py
uv run python packages/suspension_multibody/scripts/case_parity_check.py
uv run --all-packages ruff check . && uv run --all-packages ty check .
```

每一步结束都要跑对应测试目录；本任务收尾必须复跑 `tests/simulation`、`tests/api`、`tests/adams`、`tests/cases` 四套组合门禁。
