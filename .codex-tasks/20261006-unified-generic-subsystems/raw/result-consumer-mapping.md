# 结果消费者通道映射

库存：raw/inventory.json，667个导入或调用点；本表覆盖结果读取组。位置为切换前源码，任务9负责逐项迁移实际调用；READY表示新查询已有验证，不表示旧调用已经删除。

| 消费组与位置 | 新通道 | 状态 |
| --- | --- | --- |
| simulation/runner.py、axle_dynamics/contract_run.py、vehicle/service.py：decode_result | ResultEnvelope；以schema/channel版本和ResolvedModel身份解码 | READY；旧业务分派待任务9删除 |
| results/axle.py、results/vehicle.py：实体状态 | body_state、tire_state、constraint_wrench、element_state | READY；消费者待任务9迁移 |
| adams/full_vehicle_correlation.py:178：steering_output | element_state(显式actuator ID) | READY |
| api.py:1377：K/C case与残差 | cases、case_samples、case_body_state、case_residuals | READY |
| api.py:1665、results/kc_state.py:111：压缩/接点/轮荷 | tire_state与声明frame；measure(tire_load)，保留实际penetration列及作用点 | READY；报告单位转换在只读查询层 |
| api.py:1750、results/kc_state.py:240：力元报告 | element_wrench(实体ID、body ID)；世界COM矩，可显式运输到世界原点 | READY；不重新求力律 |
| adams/axle_channels.py:292：弹簧/阻尼/限位/衬套/防倾杆 | element_state(显式实体ID)，native ledger原值 | READY |
| adams/axle_equivalence.py:136：能量、接触、诊断 | energy、contact_events、diagnostics、performance | READY |
| io/artifacts.py:432：数组导出 | named_blocks，完整保留native数值块 | READY |

测量声明toe/camber要求axis、frame和rad；轮荷要求Tire Element ID和N；滚心要求两个接点frame、两个驱动frame、参考frame、明确约束ID和m。滚心读取native收敛状态的Jacobian求虚功斜率，不调用汽车装配器。

验证：raw/validation/child-8-step-*.json，车轮locked反力矩平衡、prescribed功率平衡、观察开关不改状态、文件/Python通道一致、frame姿态查询、两种机构滚心及单位/引用拒绝。
