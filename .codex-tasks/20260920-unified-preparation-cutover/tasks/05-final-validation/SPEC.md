# 子任务 05：终局回归与 Epic 收口

## 目标

独立验证统一 preparation 链路、整车迁移、旧模块删除、结果/指标/artifact 和历史读取边界，并同步 Epic 真源。

## 交付范围

- 运行 suspension_multibody 专项全量 pytest，以及 architecture、CLI、simulation、vehicle、results、Adams 关键回归。
- 运行独立 `test_dynamic_result_compat.py` 历史 `DynamicResultBundle` 读取回归和统一 artifact 回归。
- 运行 scoped ruff、compileall、全仓 ty、git diff check 和统一 `legacy_reference_scan.py --post-delete`。
- 核对 staged `prepare_request → compiler → run_request(compiled)`、便捷 facade、legacy `prepared` adapter 的 bypass/复用/stale 组合、native 唯一提交点、`results.decoder` 唯一解码入口和 metrics/artifact 证据。
- 在父级 `.codex-tasks/20260920-unified-preparation-cutover/PROGRESS.md` 逐条记录每个命令的完整文本、实际退出码、通过数量/关键断言/失败首因和必要的原始输出路径；更新子任务、Epic、父级 PROGRESS，只有全部验收通过才标记 DONE。

## 终局验收

- 专项 pytest 无新增失败。
- 统一 preparation/runner 的 registry key、document bypass、single-call 复用、staged/facade 关键路径有测试证据；不匹配 prepared context 会重新 preparation。
- `vehicle_dynamics.py` 不存在且交付目录无失效引用；旧模块完整职责清单逐项有新归属证据。
- native 唯一提交点、`RawContractResult → results.decoder → typed result`、历史读取兼容和 artifact 协议未回退；全仓恰有一个统一 `decode_result` 定义。
- 所有静态门禁通过，且父级 `PROGRESS.md` 验证记录可独立恢复终局结论。

## 验证命令

```bash
uv run --package suspension-multibody pytest packages/suspension_multibody/tests -q
uv run --package suspension-multibody pytest packages/suspension_multibody/tests/architecture packages/suspension_multibody/tests/cli packages/suspension_multibody/tests/simulation -q
uv run --package suspension-multibody pytest packages/suspension_multibody/tests/io/test_artifacts_unified.py packages/suspension_multibody/tests/adams/test_time_domain_axle.py packages/suspension_multibody/tests/adams/test_time_domain_vehicle_kc.py -q
uv run --package suspension-multibody pytest packages/suspension_multibody/tests/schema/test_dynamic_result_compat.py packages/suspension_multibody/tests/io/test_artifacts_unified.py -q
uv run --all-packages ruff check packages/suspension_multibody/src packages/suspension_multibody/tests packages/suspension_multibody/scripts
uv run --all-packages python -m compileall -q packages/suspension_multibody/src packages/suspension_multibody/tests packages/suspension_multibody/scripts
uv run --all-packages ty check .
git diff --check
uv run --package suspension-multibody python .codex-tasks/20260920-unified-preparation-cutover/tasks/04-delete-legacy/legacy_reference_scan.py --post-delete
uv run --package suspension-multibody python .codex-tasks/20260920-unified-preparation-cutover/tasks/04-delete-legacy/architecture_contract_scan.py
```
