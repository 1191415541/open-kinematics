# 子任务 01：冻结迁移清单、行为基线与可执行验证命令

## 目标

在不动任何生产代码的前提下实测现状，冻结 02–09 的全部前置事实，产出父级 `VALIDATION.md` 与本任务的 `SYMBOL_MATRIX.csv`：

1. 工具链与构建配置：mingw `x86_64-w64-mingw32-g++` 16.2.0、`Release`、`-O3 -DNDEBUG -std=c++17 -flto=auto -fno-fat-lto-objects -Wall -Wextra -Werror -fno-fast-math -fopenmp`、线程数、后端与计算顺序。冻结后 02–09 不得改动。
2. ABI 事实：实测 DLL 导出 7 个符号（`suspension_kernel_run`、`suspension_kernel_capabilities`、`suspension_kernel_contract_version`、`axle_kernel_abi_version`、`vehicle_kernel_abi_version`、`mb_core_abi_version`、`mb_core_run`），版本字面量 15 / 1 / 30，`suspension_kernel_free` 不存在且不新增。
3. 逐符号迁移清单：C++ 侧 20 个模块（86 条头文件边，且当前检查器只扫头文件、不能证明 `.cpp` 依赖正确）的旧→新归属与调用者；Python 侧 `core`、`elements`、`model`、`analysis`、`metrics`、`pac2002_scope` 现役符号的旧→新归属与调用者。
4. 数值门与容差：动态数组逐位一致、K/C parity、family parity、性能门、Python 报告通道级容差的真实命令、产物路径、容差与退出码。
5. 报告实体映射与契约输出缺口清单：元件通道字段与 native 输出的逐项缺口、凝聚后 body ID→凝聚体 ID 身份映射。

## 非目标

- 不改任何生产代码、`CMakeLists.txt`、`layering_baseline.json`、测试、schema 或文档。
- 不删除、不重命名任何模块，不开始 03/04 的结构迁移。
- 不重录数值基线使失败消失，不放宽容差，不以历史任务（`20260917`/`20260919`/`20260920`）的 DONE 记录代替本次实测。
- 不为不可执行的命令编造参数、脚本或产物路径。

## 约束

- 每条命令必须实际执行并记录：完整命令、实际退出码、结果摘要、证据文件路径（中间证据一律放会话 scratch）。
- 任何无法执行的命令必须写入 `VALIDATION.md` 的「未冻结项」章节，并明确标注它阻断哪些子任务；不得用 `echo SKIP`、占位参数或"预计通过"代替。
- 脚本参数只能通过 `--help` 与现有构建配置核实，不得虚构 flag。
- 本任务 `raw/` 规划阶段为空；实施期可归档的证据放 `raw/`，中间日志与临时脚本放会话 scratch，禁止预填未执行的证据。

## 范围与文件归属

- 可写：`.codex-tasks/20260921-architecture-deviation-closure/VALIDATION.md`（父级冻结件，由本任务产出，02–09 只读引用）；`.codex-tasks/20260921-architecture-deviation-closure/tasks/20260921-01-baseline/` 下 `SPEC.md`、`TODO.csv`、`PROGRESS.md`、`SYMBOL_MATRIX.csv`、空 `raw/`。
- 只读：父 `EPIC.md`、`SUBTASKS.csv`、`packages/**`、根 `CONTEXT-MAP.md`、`.codex-tasks/20260917-native-multibody-takeover/ARCHITECTURE.md`。
- 父 `EPIC.md`、`SUBTASKS.csv`、`PROGRESS.md` 归主代理，本任务不写。

## 依赖

- 无前置子任务，是串行主线的起点。
- 02–09 全部依赖本任务：`VALIDATION.md` 未冻结或未实测通过前不得开始结构迁移（EPIC 验证协议：动态哈希/K/C parity/family parity/性能门未冻结不得开始结构迁移）。
- 本任务不满足 03/04 的拆分前置，只提供事实基线。

## 验收标准

1. `VALIDATION.md` 的工具链条目与 `packages/suspension_kernel/src/suspension_kernel/native/native_build.json` 的 `compiler`、`compiler_version`、`configuration`、`flags`、`abi_version`、`core_abi_version`、`vehicle_abi_version` 实测一致。
2. `VALIDATION.md` 含 7 个导出符号的实测清单与调用签名，并明确记录 `suspension_kernel_free` 缺席且不新增。
3. `dynamic_hash_sentinel.py`、`kc_parity_check.py`、`kc_perf_gate.py`、`case_parity_check.py`、`check_module_layering.py` 的真实参数由 `--help` 实测冻结，命令原文写入 `VALIDATION.md`。
4. 现有全量测试与 `ruff`/`ty` 的通过、失败、skip 数量逐条记录，每条失败/跳过有原因，且区分「既有失败」与「本次引入」。
5. `SYMBOL_MATRIX.csv` 每行的旧归属、新归属、调用者、处置四列非空；C++ 模块边与 Python 现役符号两侧都有行，覆盖 `mb_base`/`mb_vehicle` 与 `core`/`elements`/`model`/`analysis`/`metrics`/`pac2002_scope`。
6. 数值门条目含真实命令、产物路径、容差与实测退出码；不可执行项单列并标注阻断范围（至少覆盖真实 Adams 执行与许可证）。
7. 报告契约缺口清单每条含通道、缺口、归属子任务、阻断结论；元件字段覆盖名称/ID、两端、坐标系、作用点、符号、单位、能量、active 状态与凝聚后身份映射。
8. `PROGRESS.md` 与实际执行一致，父级状态已回填。

## 验证协议

```bash
uv run python packages/suspension_multibody/scripts/build_axle_native.py
uv run python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --help
uv run python packages/suspension_multibody/scripts/kc_parity_check.py --help
uv run python packages/suspension_multibody/scripts/kc_perf_gate.py --help
uv run python packages/suspension_multibody/scripts/case_parity_check.py --help
uv run python packages/suspension_kernel/scripts/check_module_layering.py --help
uv run python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check
uv run python packages/suspension_multibody/scripts/kc_parity_check.py --check
uv run python packages/suspension_multibody/scripts/case_parity_check.py
uv run python packages/suspension_multibody/scripts/kc_perf_gate.py --check
uv run python packages/suspension_kernel/scripts/check_module_layering.py --strict
uv run --package suspension-kernel pytest packages/suspension_kernel/tests -q
uv run --package suspension-contracts pytest packages/suspension_contracts/tests -q
uv run --package suspension-multibody pytest packages/suspension_multibody/tests -q
uv run --all-packages ruff check . && uv run --all-packages ty check .
```

上述参数以本任务 `--help` 实测结果为准，实测后把命令原文写入 `VALIDATION.md`；02–09 只引用 `VALIDATION.md` 中的命令原文，不得自行改写参数或容差。
