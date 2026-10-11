# 规划进度

## 恢复块

- 任务：计划及全部子任务SPEC落盘；形态：single-full（Epic规划活动）。
- 进度：4/4，规划DONE；真源：[TODO.csv](TODO.csv)。
- 产品实施0/9，与规划进度不混用。
- 已完成：taskmaster/完整设计/AGENTS/justfile已读；两探子只读契约/native/兼容测试，主线程点验导出与依赖。
- 下一步：规划已交付，产品实施仍TODO；恢复实施从父PROGRESS和P0 SPEC/TODO开始，不自动启动源码或商业服务。
- 验证：git status初始干净，artifacts写入JSON/NPZ，CSV/Parquet仅已有读取，不误算写表。
- 本目录git check-ignore命中，落盘可读但默认不进提交；不改ignore、不强制暂存。

## 2026-10-10 规划写作

按设计九阶段拆9个single-full任务/51叶子步。P4a核心、P4b/P7独立、矩阵可选；P0记录Adams未知项，P5/P6仍由核心完整证据门阻断。

## 2026-10-10 独立审查与修订

结构审查partial终态确认CSV/DAG/链接/9任务51步正确，但未逐条核对语义；新增干净定向探子完成九SPEC语义审查。唯一阻断B1为P0完成条件被通用风险句误阻断，修订后另派探子终审通过，最终无阻断。完整审查边界与证据见REVIEW.md。

## 2026-10-10 最终验证

- raw/validate_plan.py --planning通过：9子任务、51叶子步、依赖无环、来源指纹匹配，产品状态全部TODO；链接按最终文件再次核验。
- just check-fast因本机缺sh而未启动lint，退出1；没有将其记为测试失败或通过。用VALIDATION.md的PowerShell等价展开全部通过，未安装或改环境依赖。
- ruff根门、规划脚本定向ruff、ty、三条架构脚本均退出0；旧模型专项5 passed；快速multibody 1553 passed/1 xfailed；kernel 97 passed；contracts 64 passed，全部退出0。
- git diff --check退出0；本次无产品源码/测试修改，不跑数值门或全量慢测试，不声明新增产品能力验收。
- 原始日志raw/check-fast.log与raw/check-fast-expanded.log，结果摘要raw/validation.md；目录被Git忽略，仅落盘，不改ignore/暂存。
