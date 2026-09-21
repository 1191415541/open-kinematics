# 子任务 09：执行独立终局验收并逐条核对 G1–G4

## 目标

在 01–08 全部完成后，用独立于子任务完成度的终局命令逐条确认 EPIC 的 Goal，不允许以"每行 DONE"代替：

1. 执行 EPIC 终局命令全集，逐条记录实际退出码与摘要。
2. G1（C++ 职责）：目标模块职责与 cpp+header 实测 DAG 通过，`mb_base`/`mb_vehicle` 等被替代模块缺席。
3. G2（Python report）**（A1 修订）**：`report` 已建立，且其中**已无生产调用者**的旧模块、旧导入、转发壳为零；仍有现役生产调用的模块逐项有保留登记。
4. G3（Python 无求解/本构）**（A1 修订）**：生产代码无关节残差/Jacobian 与反力求解；力元本构按 05 可选通道的实际状态核对（通道已启用则须删除，未启用则须有 file:line 保留登记与解除条件）；元件载荷报告有 native 逐通道证据。数值门为独立项：字节级门保持绿且未重录任何基线。
5. G4（保留能力）：公开 API 与 CLI、七个 family、Adams 渲染、历史 artifact 读取与 success/partial/failed artifact 端到端通过。
6. 隔离 wheel 验证**（A1 修订）**：安装后 import/CLI 可用、七个导出符号一致、native 可执行、**已无生产调用者**的旧模块缺席（按 A1 保留的部分不要求缺席）。
7. 不相关的既有失败独立列明；任何新增失败阻断完成。

## 非目标

- 不在本任务修代码：发现缺口退回对应子任务，不在此处顺手改。
- 不重录基线、不放宽容差、不把未实现的整车 Adams 对标标为通过。
- 不新增验收标准之外的功能或检查项。

## 约束

- 终局命令必须逐条执行并记录，不得只跑其中一条 pytest 就宣告通过。
- 数值门、ABI 七符号门、通道级容差的命令与容差一律引用 01 的父级 `VALIDATION.md` 原文。
- 真实 Adams 执行需现有安装与许可；缺少真实执行时明确记录并保留"未做整车数值等价声明"，不得含糊通过。
- 结论必须区分「Goal 达成」与「子任务 DONE」；每条 Goal 要求都要能指向具体证据文件或命令输出。

## 范围与文件归属

- 可写：`.codex-tasks/20260921-architecture-deviation-closure/tasks/20260921-09-acceptance/` 下 `SPEC.md`、`TODO.csv`、`PROGRESS.md`、空 `raw/`。
- 只读：全部 `packages/**`、`VALIDATION.md`、`tasks/20260921-01-baseline/SYMBOL_MATRIX.csv`、`tasks/20260921-08-delete/legacy_reference_scan.py`、父 `EPIC.md`、`SUBTASKS.csv`。
- 不写：生产代码、测试、文档；父 `EPIC.md`、`SUBTASKS.csv`、`PROGRESS.md`（父状态由主代理回填）。

## 依赖

- 前置：01–08 全部完成且各自门禁通过；`VALIDATION.md`、`SYMBOL_MATRIX.csv`、删除后扫描零残留证据齐备。
- 无后续子任务；本任务是 EPIC 的终局门禁。

## 验收标准

1. EPIC 终局命令全集逐条执行并记录实际退出码与摘要，缺项即为未完成。
2. G1 证据：`check_module_layering.py --strict` 覆盖 cpp+header 且通过；目标模块集合存在、旧模块缺席；C++ 侧无 Python 本构/求解残留。
3. G2 证据**（A1 修订）**：`report` 存在且不调用 native、不执行 preparation、不复算本构；删除后扫描显示**已无生产调用者**的旧模块、旧导入、转发壳为零，仍有现役生产调用的模块逐项有保留登记。
4. G3 证据**（A1 修订）**：生产代码无关节残差/Jacobian 与反力求解；力元本构按 05 可选通道实际状态核对（未启用则须有 file:line 保留登记与解除条件）；元件载荷报告逐通道与 native 输出对照（名称/ID、两端、坐标系、作用点、符号、单位、能量、active 状态）。
5. 凝聚证据**（A1 修订）**：凝聚保留在 Python 作者层；native `kind="fixed"` 关节的等价性测试存在且通过；body ID→凝聚体 ID 映射有登记。
6. G4 证据：公开 API 与 CLI、七个 family、Adams source rendering、历史 artifact 读取与 success/partial/failed artifact 端到端通过。
7. 数值门证据（独立项）：`dynamic_hash_sentinel` 与 `case_parity_check` 字节级门保持绿，且未重录任何基线。
8. 隔离环境安装两个 wheel 后 import/CLI 可用、七个导出符号与 01 冻结清单一致、native 可执行、**已无生产调用者**的旧模块缺席（保留部分不要求缺席）。
9. 不相关既有失败独立列明；新增失败为零；未把未实现的整车 Adams 对标标为通过。

## 验证协议

EPIC 终局命令全集（逐条执行并记录）：

```bash
uv run python packages/suspension_multibody/scripts/build_axle_native.py
uv run python packages/suspension_kernel/scripts/check_module_layering.py --strict
uv run --package suspension-kernel pytest packages/suspension_kernel/tests -q
uv run --package suspension-contracts pytest packages/suspension_contracts/tests -q
uv run --package suspension-multibody pytest packages/suspension_multibody/tests -q
uv run --all-packages ruff check .
uv run --all-packages ty check .
uv build --package suspension-kernel
uv build --package suspension-multibody
git diff --check
```

数值门、ABI 七符号与通道级容差按父级 `VALIDATION.md` 原文执行；隔离 wheel 检查在会话 scratch 新建环境完成，中间日志放 scratch，可归档证据放本任务 `raw/`。
