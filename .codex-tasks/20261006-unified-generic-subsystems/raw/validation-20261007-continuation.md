# 2026-10-07 迁移验证记录

本轮不切换旧公开生产入口，不删除旧生产链，不修改冻结数值文件。

| 验证 | 实际结果 |
| --- | --- |
| `planning/checks.py step 9 1` | 55 passed |
| `planning/checks.py structural` | ruff、ty、legacy surface、native layering、composable release五门PASS |
| 快速集，排除adams/architecture/cases | 最终1613 passed、1 xfailed；首次发现旧API拒绝测试将合法文件路径误判为非法Case，修正为真实装配加非文档对象后18项API及快速集通过 |
| kernel/contracts | 158 passed |
| Adams default/strict K/strict C加K/C冻结边界 | 11 passed，未运行Adams外部求解 |
| 整车通道报告 | 8 passed；实际native提交132个轮胎通道、4个操稳通道、2个采样 |
| 单胎Rig | 14 passed |
| Adams车轴/车身时域入口 | 7 passed，实际普通文档提交；包含不同左右轮驱动、载荷ID和非零初始横摆角；未启动外部Adams |
| Spin/离线迁移/普通子系统联合 | 64 passed，包含直接wheel frame及fixed hub代理继承spin的拒绝反例 |
| pad模式/默认/驱动/输出/激活/C平衡/离线签名 | 7项PASS；contact高度最大误差1.208e-13 mm |
| 完整八族 `case_parity_check.py` | exit 1，只有vehicle_dynamic FAIL；其余6个求解族PASS，comparison N/A |
| 冻结文件 | 库存9份SHA256全部不变 |
| `git diff --check` | exit 0 |

完整八族实际结果：

```text
kc_quasi_static PASS: worst error/tolerance 0.000185928
axle_dynamic PASS: 13 cases bit-identical
vehicle_kc PASS: zero grid and four 10 mm drives
vehicle_dynamic FAIL: original frozen hash assertions remain strict
handling PASS: 4 open-loop shapes, sampled reference max delta 0; closed-loop refused
ride_four_post PASS: independent sampled excitation max delta 0
ride_random_road PASS: independent expanded profile max delta 0
comparison N/A: not a solve family
```

整车默认、制动、PAC2002两种及measured table的states哈希差异涉及原实体计算顺序；显式恢复该顺序的诊断中逐位相同。steering、nondefault road and initial state、bushing force curves仍有接触frame物理差异。仅恢复历史frame后的8项严格哈希均一致，此只读诊断修改resolver后的IR，不能充当新数值门证据。

待批准的调整见`planning/vehicle_numeric_change.md`，批准前原FAIL及删除阻断均保持。当前Epic 8/10，子任务9步骤1 IN_PROGRESS，0/4叶步骤完成。
