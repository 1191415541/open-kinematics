- 任务：建立源码级分层、模块集合与职责删除门禁
- 形态：single-full（Epic 子任务）
- 进度：7/7 步骤 DONE
- 当前：子任务完成；终局门禁的当前缺口（10 个目标模块缺失、7 个旧模块、3 条反向边、2 个环、5 个旧 Python 包）按设计保持为 03-08 的待消除阻断。
- 文件：`.codex-tasks/20260921-architecture-deviation-closure/tasks/20260921-02-gates/`
- 验证：`--check`/`--strict` 退出 0，`--strict --final` 退出 1（预期阻断），架构测试 86 passed，kernel 15 passed，legacy_surface `--check` 退出 0、`--final` 退出 1（预期阻断），动态 sentinel `--check` 退出 0，ruff/ty/git diff --check 通过。

## 恢复信息

子任务 02 已完成。03-09 的每一步结构迁移都必须在迁移模式 `--strict` 保持退出 0 的前提下推进，终局缺口随迁移逐步消除；`--strict --final` 退出 0 是任务 08 的收尾判据之一。

## 门禁对 03-09 的调用契约（冻结）

C++ 分层门禁（迁移阶段每步必跑，终局判据在 08）：

```bash
uv run python packages/suspension_kernel/scripts/check_module_layering.py --check      # 与 baseline 比较，容忍已登记旧结构
uv run python packages/suspension_kernel/scripts/check_module_layering.py --strict     # 上者 + 空自包含/聚合断言 + 已登记周期
uv run python packages/suspension_kernel/scripts/check_module_layering.py --strict --final   # 终局：旧模块/旧路径缺席、目标齐全、无反向边、无环
```

Python 职责门禁（与 C++ 门禁并行运行）：

```bash
uv run python packages/suspension_multibody/tests/architecture/legacy_surface_gate.py --check        # 迁移：登记项容忍，registry 只能收缩
uv run python packages/suspension_multibody/tests/architecture/legacy_surface_gate.py --check --final # 终局：任何旧导入/旧包/转发壳/report native 调用均失败
```

架构回归（每次门禁变更后运行）：

```bash
uv run --package suspension-multibody pytest packages/suspension_multibody/tests/architecture -q
```

## 实现内容

- `check_module_layering.py`：扫描 51 个头文件（含 `cpp/src/**/*.hpp` 私有头）、70 个 TU；`extern "C"` 声明入符号表；header/source 边分离比较；`migration_map`（7 个旧模块 → 后继）+ `reviewed_new_edges` 人工台账 + `missing_source_evidence` 证据一致性；迁移/终局模式为显式 CLI 参数，无环境变量后门；`--record-baseline` 不写 curated 字段。
- `legacy_surface_gate.py`：AST 扫描（绝不导入被扫文件）覆盖绝对/相对/别名/动态 import（含模块级字符串常量参数）、report 域 native import/call、转发壳；registry 70 条精确匹配且 `occurrences` 参与校验，登记只能收缩。
- `test_module_layering_gate.py` + `test_legacy_surface_gate.py`：86 个测试，四类负例（反向边、旧导入、report native、转发壳）全部断言失败方向。

## 独立审查与修复记录

code-reviewer 只读审查发现 5 个缺口，全部修复并有测试：

1. `cpp/src/**` 私有头逃出门禁（`case_common.hpp` 注释自认绕过）→ 纳入 `_headers` 扫描，owner 按 `source_module` 归属。
2. 动态 import 只识别字面量 → 增加模块级字符串常量解析（`_module_constants`）。
3. self/aggregate include 用计数比较 → 改为内容集合差比较。
4. `extern "C"` 块被当函数体丢弃 → `_TYPE_SCOPE` 加入 `extern`，ABI 声明进入 `declared["abi"]`。
5. registry `occurrences` 从未校验 → `stale_registrations` 校验计数漂移。

扫描器扩展后的关键验证：新扫描（含私有头与 extern C）与既有 baseline 的 header_edges（86）、source_edges（65）、edges（100）、mutual_edges（3）、cycles（2）、evidence 键集完全一致——私有头与 ABI 声明的纳入只增加门禁可见性，没有引入新边，因此未重录 baseline。symbol_reference 证据从 128 → 130（`axle_kernel.hpp` 的 ABI 声明），来自既存边的证据扩充，不构成边集变化。

## 证据

全部原始日志在 `raw/`：`gate_strict.log`（退出 0）、`gate_check.log`（退出 0）、`gate_strict_final.log`（退出 1，预期阻断，含全部 35 条终局 finding）、`legacy_surface_check.log`（退出 0）、`legacy_surface_final.log`（退出 1，预期阻断）、`architecture_tests.log`（86 passed）、`kernel_tests.log`（15 passed）、`dynamic_sentinel.log`（退出 0，26/26 artifact 哈希匹配）、`ruff.log`、`ty.log`。

当前终局阻断清单（03-08 逐项消除的对象）：目标模块缺失 10（mb_numeric、mb_dual、mb_config、mb_linear、mb_joint、mb_solve_static、mb_solve_dynamic、mb_element、mb_assembly、mb_force）；旧模块现存 7（mb_base、mb_vehicle、mb_suspension、mb_integrator、mb_static、mb_linalg、mb_constraint）；反向边 3（mb_constraint↔mb_model、mb_integrator↔mb_tire_state、mb_static↔mb_vehicle）；环 2；旧 Python 包 5（core、elements、model、analysis、metrics）及其 70 条登记导入。
