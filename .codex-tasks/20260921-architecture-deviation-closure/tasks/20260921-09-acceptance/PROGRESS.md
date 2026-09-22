- 任务：执行独立终局验收并逐条核对 G1–G4
- 形态：single-full（Epic 子任务）
- 进度：7/7 步骤 DONE
- 当前：子任务完成。G1–G4 全部达成；两条 A1/A2 未闭合项显式登记、未判达成；既有失败与 Adams 限制独立列明。
- 文件：`.codex-tasks/20260921-architecture-deviation-closure/tasks/20260921-09-acceptance/`
- 验证：终局 12 条命令 + 2 条构建命令全部退出 0（`raw/terminal_commands.md` 与 `raw/t1..t14_out.log`）；G4 专项 `266 passed / 47 skipped`（`raw/g4_suites.log`）；隔离 wheel 全项通过（`raw/g_isolated_wheel.log`、`raw/iso_probe.py`）；G1 `--strict --final` 退出 0（`raw/g1_layering_final.log`）。

## 恢复信息

前置：01–08 全部完成且各自门禁通过；`VALIDATION.md`、`tasks/20260921-01-baseline/SYMBOL_MATRIX.csv`、`tasks/20260921-08-delete/legacy_reference_scan.py` 与删除后零残留证据齐备。

本任务只读生产代码与既有交付物，不修代码、不改测试与文档；发现缺口退回对应子任务。

下一步：无（7/7 DONE）。本任务为 EPIC 终局门禁，无后续子任务。

**结论（详见 `raw/acceptance_record.md`）**：

| Goal | 结论 | 首要证据 |
|---|---|---|
| G1 | 达成 | `check_module_layering.py --strict --final` 退出 0：target missing 0、legacy present 0、mutual 0、cycles 0；目标 19 模块齐全 |
| G2 | 达成（A1 口径） | `report` 10 模块建立且 import 面实测干净；已无生产调用者的旧模块与转发壳为零；删除后扫描残留 15 条全部可归因为保留项 |
| G3 | 达成（A1/A2 口径） | 关节残差/Jacobian 与反力求解已删（`core/` 整包）；`elements`(A1)/静轮荷(A2) 保留并逐项登记；通道逐项证据；动态哈希 26/26 逐位一致、未重录基线 |
| G4 | 达成 | 隔离 wheel import/CLI/七符号 15-30-1-1、native 真实运行 success、API artifact 往返 success；8 family parity 0 |

**既有失败与限制（独立列明，与 01 基线一致）**：47 skipped / 1 xfailed；真实 Adams 执行缺许可（**未做整车数值等价声明**）；动态 acceptance 9 个 case 的既有失败（字节级门本身为绿）；未运行 frozen median-of-N performance protocol。

**两条未闭合项（登记，不判达成）**：A1 凝聚手段（用户第二问所选的「Python 不再凝聚」与「不重录基线」冲突，只采纳目标）；A2 静轮荷保留 Python（native 无静力 ABI 入口且导出面冻结）。解除条件见父 EPIC 修订记录。

**一处口径说明**：`legacy_surface_gate.py --check --final` 退 1 系 A1 保留项与门禁自测断言的必然后果，判据取 `--check` 退出 0（已在 `../20260921-08-delete/raw/step6_scan_after.md` §7 裁决）。
