# PROGRESS

- 任务：完成兼容代码删除与最终 strict 门禁
- 形态：single-full（Epic 子任务）
- 进度：0/6 步骤完成
- 当前：等待任务 7 metrics 与 artifact IO 收口完成
- 文件：`.codex-tasks/20260919-public-api-simulation-cutover/tasks/08-final-deletion-gates/`
- 下一步：读取全部迁移和回归证据，执行全仓残留扫描、删除无调用方兼容代码、重写 facade 断言并切 strict gate

## 恢复信息

删除顺序固定为：全仓扫描 → 证明调用方已切换 → 删除 facade/重复实现 → 清理导出、测试和文档 → 专项回归 → strict architecture gate → 静态门禁 → 更新 Epic 真源。任何一项失败都不得标记 Epic 为 DONE。

## 验证

`uv run --all-packages pytest && uv run --all-packages ruff check packages/suspension_multibody/src packages/suspension_multibody/tests && uv run --all-packages ty check . && git diff --check`
