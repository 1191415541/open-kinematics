# 01 基线、风险与可执行门禁

- 父任务：`.codex-tasks/multibody-composable-architecture/EPIC.md`
- 依赖：无。Goal：G9。
- 输入规格：`../../TASKS.md` 第 01 节与「基线与终局公共命令集」。
- 真源：状态以 `../../SUBTASKS.csv` 第 2 行为准。

## 目标

1. 在**未改动物理实现**的前提下，冻结当前仓库可执行基线：源码/夹具/原生库指纹、公共入口与产物格式、构建信息、固定环境。
2. 实跑 `TASKS.md` 末尾公共命令集，逐条记录退出码、通过/失败、skip/xfail 原文原因。
3. 判明旧任务登记的**两个目标相关缺口**是否仍存在，并给出归属（准静态轮胎 → 07，rack 输出收缩 → 10）。
4. 为非目标历史限制建立清单，明确保留、不得借本轮豁免。
5. 给 A1-A10 建立量纲明确的误差标准：几何/轴向、质量/质心/惯量、残差、输出、状态导数、性能。
6. 交付 `check_composable_baseline.py --check`：解析证据清单，验证指纹、命令实跑结果、容差与已知失败归类完整；缺 native、缺必需结果、未分类故障必须非零退出，**不得只检查文件存在**。

## 非目标

- 不修改任何物理实现、测试或既有基线文件。
- 不重录任何数值/性能基线。
- 不因发现失败而放宽门限；失败按归属登记。
- 不实现 02-13 的交付物。

## 约束

- 证据必须来自实际执行，不得以"文件存在"或"脚本可导入"代替。
- 已有未提交修改保留，不得归为本轮新增回归。
- 原生库指纹须记录 kernel 与 multibody 两份镜像，并说明其一致性关系。

## 验收（对齐 SUBTASKS.csv:2）

- `uv run --no-sync python packages/suspension_multibody/scripts/check_composable_baseline.py --check` 退出码 0。
- 实际命令结果与输入及源码指纹可追溯。
- 容差、性能预算和已知失败分类完整。
- 缺必需结果 / 缺 native / 未分类故障时脚本非零退出。

## 写范围

- `packages/suspension_multibody/scripts/check_composable_baseline.py`（新增）
- 本任务目录下的证据文件（`BASELINE.json`、`PROGRESS.md`、`TODO.csv`、`SPEC.md`）

## 风险

- 原生库新鲜度门：kernel 与 multibody 两份 `suspension_kernel.dll` 必须内容一致且 multibody 侧不旧于 kernel 侧，否则大量用例以 `NativeKernelUnavailableError` 报错。本任务记录该门及其指纹要求。
