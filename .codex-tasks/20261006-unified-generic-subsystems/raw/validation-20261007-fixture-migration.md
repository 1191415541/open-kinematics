# 车辆验收数据与显式拓扑消费者迁移

- 将车辆共享声明移至`tests/vehicle/vehicle_fixtures.py`，原native车辆测试和十个数据消费者使用同一份fixture；`case_parity_check.py`不再通过旧测试模块间接导入preparation、车辆装配或service。
- 初态由离线迁移的普通模板原点和四元数生成，保留工程单位和原稳定实体顺序，不从SI质心状态回填。八个工况的源声明SHA256均与原reference manifest完全相同，原冻结文件未重录。
- 新解释器隔离测试阻断preparation、vehicle.service、subsystems.entry和旧测试模块，再生成全部八个输入并核对原指纹；快速集包含该测试。
- 第一次相关验证：physics、offline migration、torque wiring、service contract、PAC2002 contact mass、signal bus及五个case消费者共135 passed，49.24秒。
- fixture集中引用及新解释器隔离验证完成后：快速集1626 passed、1 xfailed，81.80秒；未增加skip/xfail。Adams及完整architecture慢目录未运行。
- 只读子代理复核未发现此次提取的确证问题；该fixture仍是离线v1声明，不能据此宣布公开旧执行链退役。
- `acceptance_composable_flow.py`的explicit-topology入口改为离线声明迁移、普通Loader和ResolvedModel；新解释器阻断preparation、subsystems.entry及presets.legacy，两个分支分别验证prismatic和fixed。
- 显式拓扑首次运行因漏传Loader必需的CaseDocument而FAIL；补充普通CaseDocument后重跑PASS，两个分支均有2个约束。最终脚本输出不再称作者层被禁用，而明确禁用旧构建入口。
- 上述源码及任务记录完成后，`planning/checks.py structural`五门再次全部通过，`git diff --check`退出码0。
- 数值验收修订的完整三门、kernel/contracts158项及相关物理21项的既有记录见`validation-20261007-numeric-revision.md`；本轮未修改求解器或物理参数。新输入来源的完整整车5 STRICT/3 PHYSICAL_DIFFERENCE机器门已由相关及快速测试实际执行通过。
- 剩余：flow的template-selection/rig-entities/rig-drives、架构验收A2-A9、公共报告及库存映射；api/simulation/preparation旧公开链尚未切换或删除。Epic仍8/10、任务9步骤1 IN_PROGRESS。
