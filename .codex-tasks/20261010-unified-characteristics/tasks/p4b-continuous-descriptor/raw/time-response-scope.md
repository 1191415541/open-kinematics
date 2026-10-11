# P4b 步骤 4：有限时域支持范围

日期：2026-10-11（Asia/Shanghai）。

## 结论

当前实现**不交付有限时域 continuous response**。本任务只登记连续描述符的目标边界；未实现初始扰动、载荷历史和时间区间的传播，也不把已有轨迹样本间差分或几何时间率包装成该能力。

## 证据

1. native 运行入口只按输出样本分配 `body_state`、约束反力、诊断和可选力元结果块（`packages/suspension_kernel/cpp/src/abi/kernel_contract_run.cpp:638-712`）；结果 manifest 只描述这些已有块（`kernel_contract_run.cpp:1063-1122`），没有 continuous descriptor 或时间响应块。
2. characteristics reader 允许 `continuous_descriptor` 作为动态定义名，但 `ContractPlan` 明确记录 kernel 尚不计算 characteristics（`packages/suspension_kernel/cpp/src/cases/case_characteristics.cpp:101-104`；`packages/suspension_kernel/cpp/include/mb_cases/functions.hpp:587-594`）。native capability response kinds 当前为空（`packages/suspension_kernel/cpp/src/abi/kernel_contract_run.cpp:229-244`）。
3. Python 结果层只有 `initial` 和 `output` 状态选择；`accepted` 因 native 未保存每个接受步状态而明确拒绝（`packages/suspension_multibody/src/suspension_multibody/results/sampling.py:149-205`）。
4. 现有 `trajectory.py` 从已保存的 body state 和速度/角速度计算几何时间率（`packages/suspension_multibody/src/suspension_multibody/results/trajectory.py:7-19,286-380`），不执行初扰动传播、载荷历史积分或区间推进。
5. Python capability guard 对 `continuous_descriptor` 明确 fail-closed：当前 native 返回 `loaded kernel does not compute characteristics response kind 'continuous_descriptor'`，没有把可解析请求误当成可回答能力。

## 发布边界

- 不返回有限时域响应数值、传播区间、初扰动响应或载荷历史响应。
- 不用输出网格代替 accepted-step 轨迹，不用相邻样本差分代替连续方程导数。
- 后续只有在 native evaluator、连续结果块、初扰动/区间协议和 Python 查询通道全部接线并有实际运行证据后，才能把本步骤从“无时间响应”范围升级为有限支持。

本步骤没有修改产品源码或冻结基线；它只锁定当前能力边界。P4b 仍保持 IN_PROGRESS，且不成为 P4a/P5/P6 前置。

## 可复核检查

- 17 类连续能力台账及证据路径检查：通过（见 `continuous-capabilities.csv`）。
- 计划真源校验：通过（9 children、51 leaf steps、DAG acyclic）。
