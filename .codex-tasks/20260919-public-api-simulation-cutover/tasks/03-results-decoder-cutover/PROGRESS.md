1:# PROGRESS
2:
3:- 任务：完成 RawContractResult、results.decoder 与 TimeSeriesResult 接线
4:- 形态：single-full（Epic 子任务）
5:- 进度：5/5 步骤完成
6:- 当前：任务 03 已完成，任务 04/05 为下一个可执行依赖
7:- 文件：`.codex-tasks/20260919-public-api-simulation-cutover/tasks/03-results-decoder-cutover/`
8:- 下一步：进入整轴与整车 service 入口迁移，复用统一 runner、decoder 和 TimeSeriesResult 协议
9:
10:## 恢复信息
11:
12:RawContractResult 已固定事实字段和状态语义；results.decoder 已接通 assembly→AxleResult/VehicleResult；时域/replay 聚合使用不可变 TimeSeriesResult。新生产路径不创建 DynamicResultBundle，历史读取仍由 loader/adapter 兼容。
13:
14:## 验证
15:
16:`uv run --all-packages pytest packages/suspension_multibody/tests/results packages/suspension_multibody/tests/contract packages/suspension_multibody/tests/analysis/test_axle_dynamic.py packages/suspension_multibody/tests/analysis/test_vehicle_dynamic.py -q && uv run --all-packages pytest packages/suspension_multibody/tests/io packages/suspension_multibody/tests/adams packages/suspension_multibody/tests/schema -q && uv run --all-packages pytest packages/suspension_multibody/tests/architecture -q && uv run --all-packages python -m compileall -q packages/suspension_multibody/src packages/suspension_multibody/tests scripts && git diff --check`
17:
18:- 结果/contract/analysis：20 passed
19:- IO/Adams/schema loader：8 passed
20:- architecture：41 passed
21:- compileall 与 git diff --check：通过
22:- 仅保留 CRLF normalization warnings，无功能失败
