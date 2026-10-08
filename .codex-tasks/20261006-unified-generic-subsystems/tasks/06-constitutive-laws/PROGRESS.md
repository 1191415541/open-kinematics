# 子系统内部力元与原生生命周期进度

## 当前恢复块

- 当前步骤：3；状态：DONE；进度：3/3。
- 真源：TODO.csv。
- 验证：25/47/18项通过；raw/validation/child-6-step-*.json记录真实日志与SHA；结构门通过，未改产品物理实现，无需重复task5数值门。
- 事实：State存native状态，solve_one_step只产生候选，driver仅accepted后写回；返回映射修改候选，拒步丢弃候选。ABS和函数力元当前为纯代数反馈无需新状态对象。
- 失败：手工probe缺previous tire_sx_dot/sy_dot槽产生访问违规；gdb定位apply_brush_return_mapping后补齐，真实投影重放通过。
- 下一步：任务7接入唯一Compiler。
