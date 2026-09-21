- 任务：迁移 Python 作者层声明并把 API 切到 native 元件事实
- 形态：single-full（Epic 子任务）
- 进度：0/8 步骤，TODO
- 当前：未执行（规划已建立，实施未开始）
- 文件：`.codex-tasks/20260921-architecture-deviation-closure/tasks/20260921-06-authoring/`
- 验证：未运行

## 恢复信息

前置：01 的 `VALIDATION.md` 已冻结；02 职责边界门禁就绪；03/04 C++ 模块归属完成；05 的 native 通道证据与 results 解码边界完成。05 未完成时不得开工本任务。

本任务写 `preparation/**`、`kernel/capabilities.py`、`schema/**`、`adams/**`、`api.py`、`elements/{elastic,assembly}.py` 以及相关测试归属调整；不建 `report`，不删除任何目录或 `pac2002_scope.py` 文件本体。

下一步：步骤 1 冻结作者层迁移矩阵，步骤 2 落地 `preparation/assembly`，步骤 3 拆分几何转换，步骤 4 迁 `time_signals`，步骤 5 迁 `pac2002_scope` 能力读取，步骤 6 切换 `api` 并删除 Python 本构与力汇总，步骤 7 回归 API/CLI/Adams/七 family/历史读取，步骤 8 汇总回填。

`api.py`、包 `__init__.py`、`schema` 是共享写面，本任务串行修改，不得与 07/08 并行写同一文件。raw/ 规划阶段为空；实施期可归档的证据放 raw/，中间日志放会话 scratch。
