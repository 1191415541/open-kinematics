# 相关性验证矩阵

本 Epic 实施期间按改动范围选择测试。用户要求跳过无关测试时，不默认执行整仓全量；但不得用 `-k` 或 `--deselect` 长期隐藏失败。所有新增测试命令先落盘并在对应子任务完成后执行。

| 改动 | 必跑 | 按需 |
|---|---|---|
| Schema、ResolvedModel、文档加载 | contracts、authoring loader/parity、ruff、ty、三个结构门 | 全量 cases |
| Wheel/Tire、Spin、接触 frame | tire、spin、native joint-coordinate、function/physics、numeric gate | Adams tire 对标 |
| Brake/Drive 子系统和 torque | generic elements、torque wiring、native function/wrench、numeric gate | 全部 Adams |
| Port/Rig/Study | ports、rigs、simulation route、cases 相关目录、ruff/ty、结构门 | 无关 tire/Adams |
| Compiler/ResultEnvelope | simulation 下现有 compiler 测试、results、新统一路径测试；kernel/contracts 独立调用；numeric gate | 完整 architecture |
| 删除生产路径和公开入口 | retirement AST、layering、release probe、直接相关快速测试 | 仅收尾时完整 architecture |
| 物理结果或 ABI | `dynamic_hash_sentinel.py --check`、`case_parity_check.py`、`kc_perf_gate.py` | Adams/cases 仅在其输入或实现受影响时 |

固定结构门：

```text
uv run --no-sync ruff check .
uv run --no-sync ty check .
uv run --no-sync python packages/suspension_multibody/tests/architecture/legacy_surface_gate.py --check
uv run --no-sync python packages/suspension_kernel/scripts/check_module_layering.py --strict --final
uv run --no-sync python packages/suspension_multibody/scripts/check_composable_release.py --skip-isolation
```

数值门执行命令：

```text
uv run --no-sync python .codex-tasks/20261006-unified-generic-subsystems/planning/checks.py numeric
```

该命令依次执行 dynamic_hash_sentinel、case_parity_check、kc_perf_gate。`checks.py closeout` 执行上述五个结构/静态命令和 `git diff --check`；`checks.py child-N` 分开调用对应叶任务的具体验证命令，避免跨包 rootdir 变化。所有退出码和日志保存 raw；`--dry-run` 仅展示命令，不构成实施验收。

拟新增测试和脚本的精确路径及责任任务在 `planning/artifacts.csv`；目录本身不存在不能当验证命令，新增文件必须先由责任子任务实现再运行。计划 validator 同时检查路径、依赖和完成证据。

不重录既有冻结基线。新 Spin 边界改变物理时，新增证据单独命名并记录影响，不覆盖旧 baseline。
