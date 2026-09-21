# 子任务 02：建立源码级分层、模块集合与职责删除门禁

## 目标

在结构迁移开始前建立终局门禁，使 03–09 的每一步都能被客观判定，且负例真实可失败：

1. `packages/suspension_kernel/scripts/check_module_layering.py` 从"只扫头文件"扩展到真实依赖：`.cpp` 的 `#include` 与函数调用所在 TU 都要计入，覆盖当前 20 个模块、86 条头文件边的基线。
2. 目标模块集合与旧模块缺席断言：新集合（`mb_numeric`、`mb_dual`、`mb_config`、`mb_linear`、`mb_joint`、`mb_input`、`mb_solve_static`、`mb_solve_dynamic`、`mb_element`、`mb_assembly`、`mb_force`）必须存在，被替代模块（`mb_base`、`mb_vehicle`、`mb_suspension`、`mb_integrator`、`mb_static`、`mb_linalg`、`mb_constraint`）在终局模式必须缺席。
3. 显式允许依赖 DAG：`layering_baseline.json` 按迁移映射重写，旧边逐条解释，语义新增边人工审核，禁止用 `--record-baseline` 盲目记录。
4. Python 职责边界与删除门禁：旧 `core`/`elements`/`model`/`analysis`/`metrics` 导入、`report` 调用 native 或求解、旧路径转发壳都必须被检测；扫描器覆盖相对 import、别名与动态 import。

## 非目标

- 不做 03/04 的模块拆分或重命名，不删除任何模块（08）。
- 不把门禁写成"文件存在/退出码为 0"这类弱证据。
- 不用重录 baseline 掩盖旧边，不引入迁移期永久豁免。
- 不改生产 Python 代码、`api.py`、包 `__init__.py`。

## 约束

- 迁移阶段模式只能按迁移映射解释旧边（旧名到新名的显式映射表），终局模式必须拒绝旧边与旧路径；两种模式的判定结果都要有测试。
- 负例必须真实失败：fixture 注入反向边、旧导入、`report`→native 绕过，测试断言门禁报错而非通过。
- `CMakeLists.txt`、`MODULES.md`、`layering_baseline.json` 由 03/04 顺序修改；本任务只写门禁脚本、baseline 与架构测试。
- 命令与容差引用父级 `VALIDATION.md`（01 冻结），不自行改写。

## 范围与文件归属

- 可写：`packages/suspension_kernel/scripts/check_module_layering.py`、`packages/suspension_kernel/layering_baseline.json`、`packages/suspension_multibody/tests/architecture/**`（含负例 fixture）。
- 只读：`packages/suspension_kernel/cpp/**`、`packages/suspension_multibody/src/suspension_multibody/**`、父 `EPIC.md`、`VALIDATION.md`。
- 不写：本任务目录之外的计划文件；父 `EPIC.md`、`SUBTASKS.csv`、`PROGRESS.md` 归主代理。

## 依赖

- 前置：01（`VALIDATION.md` 冻结工具链、七符号、数值门命令与容差）。未冻结前本任务不得开工。
- 后续：03、04、05、06、07、08、09 全部依赖本任务的架构门禁与删除门禁。

## 验收标准

1. `check_module_layering.py --strict` 能报出 `.cpp` 级别的真实依赖，反向边与环计数为 0，边集与 baseline 比较；扫描范围不只头文件。
2. 目标模块集合断言与旧模块缺席断言各有一条可失败测试，分别覆盖存在与缺席两个方向。
3. `layering_baseline.json` 的每条旧边都能追到迁移映射中的解释，diff 中不存在未经说明的新增边。
4. 迁移阶段模式与终局模式行为不同且都有测试：迁移阶段按映射解释旧边，终局模式对旧边报错。
5. 负例测试覆盖：注入反向边、注入旧模块导入、`report` 调用 native/求解、转发壳；四类都必须失败。
6. 删除门禁扫描器支持相对 import、别名 import、动态 import，并有对应测试。
7. 架构测试全绿，且未修改任何生产模块源码。

## 验证协议

```bash
uv run python packages/suspension_kernel/scripts/check_module_layering.py --strict
uv run --package suspension-multibody pytest packages/suspension_multibody/tests/architecture -q
uv run --package suspension-kernel pytest packages/suspension_kernel/tests -q
uv run python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check
```

门禁自身的负例是验收对象：`pytest tests/architecture` 必须包含"注入后失败"的断言，不能只包含正例。
