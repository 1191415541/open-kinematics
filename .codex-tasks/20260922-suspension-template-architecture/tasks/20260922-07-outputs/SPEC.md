# 子任务 07：输出声明与衍生输出（request 机制）

## 目标

让「输出」有明确的归属与可扩展的运算层，且不破坏 `report/` 现有边界：

1. 建立**最小单位输出**的声明机制：总成声明自己的输出、试验台声明自己的输出；运行前把两份声明合并，并检测冲突（重名但定义 / 单位 / 形状不一致必须报错）。
2. 建立**衍生输出**（类似 Adams Car Request）：衍生输出**只能**对最小单位输出求值，不得旁路直接读内核、执行 `preparation` 或复算本构。
3. 把现有 `report/metrics/{axle,case_specific,common,vehicle}.py` 的 **27 个指标函数**重述为内置衍生输出；重述后数值必须与旧实现**逐值一致**。

## 非目标

- 不改 `preparation/`、`subsystems/`、`templates/`（与 04–06 写范围冲突）。
- 不改 C++ 内核、不改 ABI、不改结果对象 schema 与 artifact 格式。
- **不删除旧指标函数**：27 个旧函数保留到 11 之后再定，本任务只做并存与逐值一致对照。
- 不改现有公开 API 的调用方式（`run_case`/`run_dynamic_case`/`run_vehicle_dynamics` 保持可用）。
- 不新增依赖、不新增图表 / UI。

## 约束

- 衍生输出求值器**不得 import kernel/native/solver**（已有 `packages/suspension_multibody/tests/architecture/legacy_surface_gate.py` 会拦截）；`report/` 现有的「不调用 native、不执行 preparation、不复算本构」边界必须保持。
- 声明与合并必须可序列化（JSON 可往返）。
- 「逐值一致」指同一输入集下新旧实现结果逐值相等，含 `None` / `NaN` 与数组形状；不得为让新机制通过而放宽现有 metrics 测试。
- 新增 `outputs/` 包不得被 `report/metrics` 之外的层反向依赖；分层判定以 02 的架构门与 `legacy_surface_gate.py` 为准。
- 本任务不重录任何基线。

## 范围与文件归属

- 可写：
  - 新增 `packages/suspension_multibody/src/suspension_multibody/outputs/**`
  - `packages/suspension_multibody/src/suspension_multibody/report/metrics/**`
  - 新增 `packages/suspension_multibody/tests/outputs/**`（至少含 `test_declarations.py`、`test_merge.py`、`test_derived.py`、`test_bypass.py`）
  - `packages/suspension_multibody/tests/metrics/**`（新增逐值一致对照）
  - `packages/suspension_multibody/src/suspension_multibody/__init__.py`（仅当需要导出新包，且不得改变现有导出）
- 只读：`packages/suspension_multibody/src/suspension_multibody/results/**`、`report/__init__.py`、`report/compliance.py`、`report/geometry.py`、`report/wheel_loads.py`、`kernel/**`、`tests/architecture/legacy_surface_gate.py` 与其 `legacy_surface_registry.json`、父 `EPIC.md`/`SUBTASKS.csv`、01 的 `raw/baseline_commands.md`。
- 不写：`preparation/**`、`subsystems/**`、`templates/**`、`packages/suspension_kernel/**`；父级 `EPIC.md`/`SUBTASKS.csv`/`PROGRESS.md` 归主代理。

## 依赖

- 前置：02（副底座统一后可开工；本任务不依赖副表本身，父级排序如此）。
- 可与 04–06 并行：写范围（`outputs/` 与 `report/metrics/`）与它们不相交。
- 后续：12 的端到端验收需要「用最小单位输出 + 自定义表达式算出一个新指标」，依赖本任务。

## 验收标准

1. 输出声明可解析：总成声明与试验台声明各自独立可构造、可序列化、可反序列化；字段含名字、单位、维度、来源域、形状、描述。
2. 合并与冲突检测：重名同定义合并；重名不同定义 / 单位 / 形状冲突报错；负例真实失败（断言报错原因，不只断言抛异常）。
3. 衍生输出只用最小单位输出：求值器签名只接受最小单位输出映射；有测试证明旁路（import kernel/native/solver、调用 preparation、复算本构）会失败，且 `legacy_surface_gate.py --check` 保持绿。
4. 27 个 `report/metrics` 函数重述后逐值一致：对照测试在同一输入集上断言新旧结果逐值相等（axle 3 + case_specific 12 + common 8 + vehicle 4 = 27）。
5. 现有 `tests/metrics` 全绿；未改 `preparation/`、`subsystems/`、`templates/` 与 C++ 内核。
6. `ruff`、`ty`、`git diff --check` 退出 0。

## 验证协议

```bash
uv run --package suspension-multibody pytest packages/suspension_multibody/tests/outputs packages/suspension_multibody/tests/metrics -q
uv run python packages/suspension_multibody/tests/architecture/legacy_surface_gate.py --check
uv run --all-packages ruff check .
uv run --all-packages ty check .
git diff --check
```

逐值一致对照必须在**每一步重述之后立即跑**，禁止积累全部重述后再对照；旁路负例（第 3 条）必须断言失败方向，不能只包含正例。
