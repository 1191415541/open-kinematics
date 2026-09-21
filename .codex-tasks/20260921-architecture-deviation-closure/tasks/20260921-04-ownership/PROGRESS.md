- 任务：拆解 mb_vehicle 并落地 assembly/force/element/tire 的职责归属
- 形态：single-full（Epic 子任务）
- 进度：7/7 步骤 DONE
- 当前：已完成收口
- 文件：`.codex-tasks/20260921-architecture-deviation-closure/tasks/20260921-04-ownership/`
- 验证：全部通过（证据在 raw/）

## 恢复信息

本子任务的源码重组（步骤 2-6 的文件迁移与拆分）在上一个会话已完成并随 HEAD 提交 7ef77a0 入库，但收口未完成：`assembly_primitives.cpp` 未登记进 CMake（构建链接失败）、`mb_element↔mb_force↔mb_tire` 与 `mb_solve_dynamic↔mb_tire_state` 环未消除、`layering_baseline.json` 与 `MODULES.md` 停留在 03 冻结态（门禁 135 条 findings）、两个架构测试仍断言"迁移期阻断存在"。本会话完成全部收口。

## 本会话完成的收口工作

1. **构建修复**：`cpp/src/element/assembly_primitives.cpp` 补进 CMakeLists.txt `MB_ELEMENT_SOURCES`（该 TU 定义 `add_force_on_body`/`add_torque_on_body`/`mat6_mul`/`bushing_deformation`，此前从未被编译）；补齐其 `mb_numeric/functions.hpp`、`mb_model/functions.hpp` include。
2. **环消除**：
   - 删除 8 处注释提及产生的无效 include：element 5 个 TU 的 `mb_force/functions.hpp`、tire 2 个 TU 的 `mb_force/functions.hpp`、`tire_state/kernel_tire_state.cpp` 的 `mb_solve_dynamic/functions.hpp`。
   - `mb_force↔mb_element` 互向边消除：`force/layout.cpp` 保留 `mb_element/functions.hpp`（真实调用 `add_force_on_body`），element 侧 include 删除。
   - mutual 边 3→0，SCC 环 1→0。
3. **职责修正（SPEC 验收 4）**：`audit_constraint_system` 从 `mb_solve_static` 迁至 `mb_joint`（实现移入 `joint/kernel_model_constraint.cpp`，声明移至 `mb_joint/functions.hpp`）；`static_rotation_gauge_for_pivot` 作为纯 Model 访问器迁至 `mb_model`。`mb_assembly→mb_solve_static` 禁止反向边随之消除；调用点（build_model 末尾、ABI 装配后校验）与诊断文本不变。
4. **baseline 同步**：`--record-baseline` 重录边集与 evidence（curated 字段人工核对未被动）；35 条新边（04 拆解产生的 assembly/element/force 依赖）逐条审核后登记 `reviewed_new_edges`（总数 189→224）。
5. **MODULES.md 重写**：23 模块表与实测依赖图一致；mb_vehicle/mb_suspension 行删除并记录拆解去向；阅读顺序更新为新目录。
6. **测试预期更新**：`test_strict_final_gate_reports_the_expected_current_blockers` 改为 `test_strict_final_gate_passes_on_the_current_tree`（终局门已绿）；`test_mode_cannot_be_switched_through_the_environment` 的 final 分支退出码预期 1→0（保留 os.environ/getenv 负例断言）。

## 验证结果（全部实际执行）

- 构建 `build_axle_native.py` 退出 0。
- 动态哈希 sentinel `--check`：26/26 逐位一致（"matches the frozen baseline byte-for-byte"）。
- K/C parity `--check`：通过（容差内）。case parity：8 families accepted。
- kernel 测试：15 passed。架构测试：86 passed。
- `check_module_layering.py --strict` 退出 0；`--strict --final` 退出 0（终局门全绿：legacy 0、cycle 0、mutual 0）。
- ABI 七符号：15/30/1/1 与冻结值一致；无新增导出。
- ruff、ty 全仓通过。
- 证据归档 raw/：step_build、step_build_after_audit_move、dynamic_hash_check、kc_parity_check、case_parity_check、kernel_tests、architecture_tests、gate_strict、gate_strict_final、abi_symbols、baseline_rerecord。

## 遗留说明

- 凝聚计算与身份映射：按 SPEC 只登记交接（见 raw/step1_symbol_ownership.md 末节），实现归 05。
- 05 可启动：mb_assembly/mb_element/mb_force/mb_tire 边界已冻结，`--strict --final` 门禁为 05 之后的工作提供零基线。
