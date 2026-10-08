# 已批准数值修订验证

- `case_parity_check.py --family vehicle_dynamic`：5 STRICT原哈希全匹配；3 PHYSICAL_DIFFERENCE独立物理门通过，原哈希不匹配单列。
- 完整`planning/checks.py numeric`：三个命令退出0，原始结果见`numeric.json`和`numeric-1/2/3.log`。动态26文件combined SHA256为`fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9`，不改冻结基线；K-100/C-66相对原预算分别1.072/1.085，门通过。
- `planning/checks.py structural`：ruff、ty及三项结构门均退出0，见`structural.json`。
- 快速测试（排除adams/architecture/cases）：1624 passed/1 xfailed，105.10 s。
- kernel/contracts：158 passed，41.17 s。
- 最终相关physics两文件：21 passed，8.29 s；新增最后一项carrier转向测试后未重复全快速集。
- `git diff --check`：退出0。没有新增skip/xfail、没有运行外部Adams、没有重录冻结baseline。
- 当前8工况完整提交和数组见`artifacts/vehicle-physical-evidence/report.json`及对应model/case MBC、JSON、稳定ID数组和native全通道NPZ。

首次ruff/ty检查发现新文件缺docstring、import顺序和动态加载模块类型缺失；已修复并通过全仓静态门。首次复审的覆盖问题已修复并补反例，最终复审见`planning/vehicle_gate_implementation_review.md`。

原动态验收的9项自收敛FAILED及缺真实Adams证据BLOCKED保持可见，冻结哈希门通过不改写这些状态。公开生产链未切换，全部消费者库存映射尚未完成，任务9仍0/4叶步骤。
