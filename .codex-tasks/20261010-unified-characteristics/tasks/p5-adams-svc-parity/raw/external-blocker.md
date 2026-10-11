# P5 外部阻断证据（2026-10-11）

本轮按 P5 SPEC 核对 Adams Car 2024.1 与独立参考数据，结论为 pending，不能开始逐点对标。

## 软件与许可

在仓库根执行了以下只读路径核对：

```powershell
$paths = @(
  'G:\MSC.Software\Adams\2024_1',
  'G:\MSC.Software\Adams\2024.1',
  'C:\MSC.Software\Adams\2024_1',
  'C:\MSC.Software\Adams\2024.1'
)
foreach ($p in $paths) { '{0}={1}' -f $p,(Test-Path -LiteralPath $p) }
```

观测结果四项均为 `False`。本机可见的 Adams 安装是 `G:\MSC.Software\Adams\2025_1_1`，不能用它冒充 2024.1，也没有启动商业软件或许可服务。

官方 2024.1 在线文档入口可以确认 SVC 是独立的 Adams Car 分析入口，但文档本身不能替代本机 2024.1 求解器、许可和可导出的结果文件：

- [Adams 2024.1 Solver Routines](https://nexus.hexagon.com/documentationcenter/en-US/bundle/adams_2024.1/page/adams_help/Adams_Car_Package/adamscar/car_appendix/TOC.Solver.Routines.xhtml)
- [Adams Car 分析入口说明](https://nexus.hexagon.com/documentationcenter/en-US/bundle/adams_2023.1/page/adams_help/Adams_Car_Package/adamscar/car_analyses/TOC.Running.Full.Vehicle.xhtml)

## 独立参考

仓库没有与当前完整模型、profile、冻结协议、事件分支、载荷反作用位置和采样网格一一对应的独立 Adams 结果文件。项目自产冻结快照和 native 自身输出不能作为独立参考。`reference-manifest.json` 因此明确为 `pending_reference`，没有虚构 fingerprint、样本或误差分布。

## 发布判定

- G1–G6 继续保持 `pending_definition`；已有审计见 `../../p0-definition-contract/raw/adams-rule-audit.csv`。
- 独立参考继续保持 `pending_reference`。
- P5 五个步骤不能标 `DONE`，P5/P6 的 Adams 兼容发布保持阻断。
- 该阻断来自外部软件/数据缺失，与样本物理有效性无关。
