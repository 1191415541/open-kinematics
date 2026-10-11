# P4b 步骤 1 审计证据

日期：2026-10-11（Asia/Shanghai）。

本步骤完成了连续状态、力律、方向导数和一致扰动坐标的只读审计，台账见 [continuous-capabilities.csv](continuous-capabilities.csv)。台账登记 17 类当前动态方程/力元路径，逐项区分：

- 已存在的内部方向导数或连续状态率；
- 仍由 generalized-alpha 残量、接受步更新、活动集或事件定位控制的离散行为；
- F_x、F_xdot、F_u 在当前代码中的实际可见范围；
- 不能公开为 continuous_descriptor 的具体原因。

## 状态与扰动坐标

当前求解状态以每个自由体的位移/四元数、线速度/角速度及轮胎内部状态存储；当前方向载体只含 `dr`、`dtheta`、`dv`、`domega`、`dsx`、`dsy`（`packages/suspension_kernel/cpp/include/mb_dual/dual_geometry.hpp:43-49`）。四元数方向用 `delta q = 0.5 * [0,dtheta] ⊗ q`，因此 `dtheta` 是空间/左乘切向量，而不是四元数四分量增量（`packages/suspension_kernel/cpp/src/dual/dual_geometry.cpp:248-256`）。PAC 接触质量/转滑状态和 Maxwell 状态不在这个方向载体内；Newton 将 contact-mass 列标记为 nonlinear relaxation 并跳过解析列（`packages/suspension_kernel/cpp/src/solve_dynamic/kernel_integrator_newton.cpp:160-163,305`）。

连续 DAE 的待实现坐标应分开记录动态状态 `x=(r,theta,v,omega,z_tire)`、其时间导数 `xdot`、代数约束/乘子 `lambda` 和独立输入 `u`。保持输入时，一致状态方向需要满足位置约束切空间 `J*delta_q=0` 及其速度约束方向 `delta(J*v)=delta_J[delta_q]*v+J*delta_v=0`；若方向是驱动信号扰动，约束右端应显式带 `delta(target_rate)`。载荷方向 `delta_u` 独立进入外力侧，不能等同于状态方向。当前 `DirectionalState` 没有 `du`，native request 也没有连续的 `F_u`/测量映射结果，因此这些是一致坐标定义与能力缺口的记录，不是已实现接口（`packages/suspension_kernel/cpp/src/solve_dynamic/kernel_integrator_residual.cpp:238-252`；`packages/suspension_kernel/cpp/include/mb_dual/dual_geometry.hpp:43-49`）。

关键结论：

1. C++ reader 已能识别 continuous_descriptor（case_characteristics.cpp:101-104），但当前 capability 文档仍返回空的 response kinds，且运行链没有 continuous descriptor 结果块或 ABI 导出。因此“请求可解析”不等于“能力可回答”；本步骤将其记录为 unsupported。
2. 约束、坐标耦合器、质量/陀螺/重力、弹簧、阻尼器、限位、衬套、防倾杆、函数元、气动和转向/驱动/制动路径都有内部方向导数，但这些导数仍服务于 Newton/残量或内部力装配，不能直接当作公开连续描述符。
3. 原生刷胎有连续松弛率，但接受步可执行摩擦椭圆投影；Fiala 有连续松弛方程，同时积分器使用步长相关的精确更新；PAC2002 还可能包含接触质量、转滑和 Maxwell 状态。它们均按混合/步长依赖能力登记，不虚报为无条件连续支持。
4. 普通转向执行器、prescribed steering、驱动矩和制动矩已拆分登记；prescribed steering 属驱动约束行，普通 steering 属代数力律，制动矩在轴向速度符号分支处报告非光滑。

本步骤只完成审计和范围锁定，没有新增产品接口、ABI 字段、结果块或求解路径；步骤 2 的实现仍为 TODO。P4b 任务继续保持 IN_PROGRESS，不作为 P4a/P5/P6 前置。

## 可复核验证

- CSV 解析：17 条数据记录，10 列字段宽度一致，退出码 0。
- 台账证据路径存在性检查：全部通过，退出码 0。
- ruff check .：退出码 0。
- ty check .：退出码 0。
- 本轮未改动产品源码，未重录任何冻结基线；Windows just 入口的 shell 限制已在父进度记录，直接命令证据仍有效。
