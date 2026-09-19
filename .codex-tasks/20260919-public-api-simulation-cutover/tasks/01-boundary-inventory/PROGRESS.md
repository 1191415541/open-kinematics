# PROGRESS

- 任务：建立边界盘点、基线 allowlist 与审计式架构门禁
- 形态：single-full（Epic 子任务）
进度：5/5 步骤完成
当前：边界盘点、精确 allowlist 与审计式架构门禁已完成
文件：`.codex-tasks/20260919-public-api-simulation-cutover/tasks/01-boundary-inventory/`
下一步：进入 task 02，补齐显式 family compiler 与统一 request 路由

## 恢复信息

已完成生产代码、脚本与测试入口盘点；已生成 80 条精确到 `path/scope/rule/symbol/line` 的审计基线，并完成禁止新增旁路的 AST 门禁。

## 产物

- `BOUNDARY.md`
- `SYMBOL_MATRIX.csv`
- `LEGACY_ALLOWLIST.toml`

## 验证

`uv run --all-packages pytest packages/suspension_multibody/tests/architecture/test_public_api_boundary_gate.py -q` → 4 passed
`uv run --all-packages pytest packages/suspension_multibody/tests/architecture -q` → 41 passed；`compileall` 与 `git diff --check` 通过
