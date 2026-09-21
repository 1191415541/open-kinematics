- 任务：建立 report 并迁移报告指标、replay 编排与声明式基准夹具
- 形态：single-full（Epic 子任务）
- 进度：0/7 步骤，TODO
- 当前：未执行（规划已建立，实施未开始）
- 文件：`.codex-tasks/20260921-architecture-deviation-closure/tasks/20260921-07-report/`
- 验证：未运行

## 恢复信息

前置：01 的 `VALIDATION.md` 已冻结；02 职责边界与负例门禁就绪；05 的 results 解码边界与 06 的作者层切换完成。

本任务新建 `report/**`，并改 `simulation/replay.py`、`results/timeseries.py`、`analysis/**` 的残留调用面、读取基准夹具的脚本与相关测试；不写 `api.py`、`schema/**`（06 已定稿），不删除 `analysis`/`metrics` 目录本体（属 08）。

下一步：步骤 1 冻结 `report` 边界与逐符号归属，步骤 2 建立 `report/metrics`，步骤 3 建立 `report/geometry.py` 与 `report/compliance.py`，步骤 4 落地 replay 编排与结果聚合，步骤 5 把基准迁成 `tests/data` 声明式夹具，步骤 6 建立边界负例门禁，步骤 7 回归并回填。

`report` 不调用 native、不执行 preparation、不复算本构；求解类符号一律归 native，不以"无内部调用"删除公开能力。raw/ 规划阶段为空；实施期可归档的证据放 raw/，中间日志放会话 scratch。
