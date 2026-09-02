# 整轴与整车动力学软件架构

本文件描述**已实现**的结构。计划中的分目录 C++ 布局未采用：内核收敛为单一翻译单元，
因为全部求解阶段共享同一 `Model`/`State` 表示与残量装配，跨文件拆分只会把内部结构
暴露为头文件接口而不减少耦合。

## 模块边界

```text
cpp/axle_dynamics/
  axle_kernel.hpp   C ABI 结构体、输出列布局注释、axle_run 声明
  axle_kernel.cpp   数学、刚体、约束、元件、接触、静态配平、积分、C ABI
  CMakeLists.txt    共享库目标，输出到 Python 包的 native/ 目录
packages/suspension_multibody/src/suspension_multibody/
  axle_dynamics/schema.py   闭集 SI 物理模型与工况（Pydantic StrictModel）
  axle_dynamics/native.py   共享库查找、ABI 校验、ctypes 编组、错误码转异常
  axle_dynamics/result.py   结果对象与全部输出列名
  axle_dynamics/io.py       模型/工况加载、NPZ+JSON 结果 artifact
  adams/axle_contract.py    冻结 manifest、通道角色绑定与哈希
  adams/axle_adams_model.py 由同一 manifest 生成原生 Adams Solver 数据集
  adams/axle_equivalence.py 独立 runner、证据包、严格比较与门禁
  adams/axle_channels.py    结果到 33 个冻结通道的导出
  model/vehicle.py           车身、前后悬架和四轮的装配及固定轮端质量凝聚
  vehicle_dynamics.py        整车输入编组、路面/转向/驱制动映射和统一 native 调用
  adams/full_vehicle_model.py Adams 源模型的部件、关节、力元和轮胎参数导入
scripts/
  build_axle_native.py/.ps1        跨平台构建入口
  run_axle_dynamics_acceptance.py  冻结工况矩阵验收
```

## C ABI

跨边界只使用显式长度的 POD 数组和标量；没有 C++ 对象或句柄跨越边界，因此不存在
跨边界所有权问题。实际接口只有两个函数：

```c
int axle_kernel_abi_version(void);

int axle_run(
    const AxleInput* input,
    AxleOutput* output,
    char* error_buffer,
    size_t error_capacity);
```

`AxleInput` 以并列数组携带刚体、关节、弹簧、衬套、稳定杆、轮胎、采样时间、路面、
驱动力矩、外载和全部求解设置；`AxleOutput` 以调用方分配的缓冲区接收 body state、
约束反力、元件输出、轮胎接触、能量账本、诊断和已定位的接触事件。当前 ABI 版本为
**14**，整车扩展 ABI 版本为 **15**；`native.py` 与 `native_build.json` 必须与之一致，
否则拒绝加载。整车 ABI 15 在整轴字段末尾追加整车轮端框架、自转轴和 PAC2002 参数；整车
ABI 15 在 PAC2002 参数数组末尾追加 Chrono PAC02 的缩放、压力和附加力矩字段；不改变
既有整轴字段的排列。

ABI 只传输 SI 数值；字符串仅用于错误缓冲区。所有数组由调用方分配，C++ 不跨边界释放内存。
返回码：`1` 参数缺失、`2` 模型构建失败（含约束秩亏）、`3` 输出缓冲区过小、
`4` 求解设置非法、`5` 时域积分失败、`6` 初始化失败、`7` 初速度违反速度约束、
`8` 初始加速度 KKT 失败、`9` 初始轮胎压缩超限、`10` 内部状态或事件缓冲区问题。
失败时已接受样本、失败时刻和诊断行仍然返回。

## 整车装配与求解

整车层把车身、前悬架、后悬架、四个轮端和轮胎装配到同一个约束残差系统；通用模型的
轮端驱动/制动以直接轮端力矩输入表达。Adams 源显式模型还保留驱动轴、三脚架和差速器
输出体，并使用非完整 `CONVEL` 自转速度约束；制动液压和动力控制律仍按项目边界作为外部
轮端输入，不伪装成已实现源子系统。
固定轮端质量通过平行轴定理凝聚到其安装体，自转轮端保留独立惯量和自转轴。

Adams 源动态结果可通过 `direct_wheel_torque_signals_from_adams_result` 回放为逐轮
`TimeSignal`：后轴驱动读取差速器左右输出转矩，制动读取四轮制动转矩。该接口只重放
已求得的轮端外部输入，不重建源模型内部控制和液压状态；显式逐轮输入存在时，仅对应通道的
全局归一化输入被覆盖，另一通道仍可独立使用全局分配。

Chrono 的整车实现给出的可迁移原则是：底盘、车桥、转向、制动、传动和轮胎作为分层
子系统创建，但所有刚体和约束最终进入同一个系统描述器。当前代码采用相同的装配边界：
`model/vehicle.py` 合并前后轴，`vehicle_dynamics.py` 将路面、转向和轮端力矩编组到同一个
native 调用。对于 Adams 源模型，前轴还保留 `rack_housing`、齿条到外壳的平移副和两个
外壳衬套；通用简化模型仍使用齿条导向副。

导入清单将两条路径分开记录：用于快速参数试装的代理模型使用理想 K 拓扑并明确不带
未解析的 Adams 衬套曲线；用于源对标的显式模型使用 C 拓扑，当前 `step_steer` 的 35 个
源 `FIELD` 已逐一映射到 native 六轴衬套。该映射只说明衬套曲线、应用坐标系和阻尼已进入
统一残量，不能替代尚未实现的其它 Adams `USER()` 力律。
同一源显式模型中的 12 个悬架 `SFORCE` 也已逐一映射：4 个 `AKISPL` 弹簧、4 个
`AKISPL` 阻尼器和 4 个 jounce stop；映射保留源标记点、曲线和限位间隙。转向齿条、动力
系统和制动的其它源力元仍单独列为未解析项。

动态 Newton 先利用位姿、速度和加速度方程的块结构消去位姿/速度增量，再对剩余的
加速度、约束反力和轮胎内部状态求解 Schur 系统；原密集 KKT 路径保留为可切换回退。
这属于代数消元，不改变物理方程或收敛判据。

PAC2002 轮胎在有源联合滑移参数时，先按当前 native 的松弛状态得到有效纵向滑移率和
侧偏角，再对纯滑移力应用选定的 `RBX/RBY/RVY` 联合滑移缩放与侧向偏置；没有这些源参数
时保留原有摩擦椭圆路径。联合滑移分支不再额外截断力，但 `friction_utilization` 仍按
纵向/侧向 PAC 峰值记录实际利用率。该实现借鉴 Chrono PAC02 的力律分解，已接入当前
源数据实际使用的部分缩放、压力、外倾角、载荷和附加力矩项，但未声称覆盖完整 TIR 参数
集或 Adams 接触力元。已有 `QBZ/QCZ/QDZ/QEZ/QHZ/SSZ` 参数会进一步计算绕接触法向的
回正力矩，`QSX/QSY` 参数会计算倾覆矩和滚动阻力矩，并通过同一方向导数路径进入 Newton
雅可比；缺少对应参数时各附加分支不增加计算开销。

静态初始化参考 Chrono 的增量 Newton/KKT 流程，将当前构型、约束残量和力平衡放在同一
个增量系统内逐步求解。连接语义仍以 Adams 源模型为准：Adams `HOOKE` 映射为当前
`UniversalJoint`，Adams `CONVEL` 映射为三个位移约束加一行非完整自转速度约束，不用
Chrono 万向连接方程替换。
参考的是系统组织、装配顺序和求解结构；没有复制 Chrono 源码，也没有加入 Chrono 头文件、
库或运行时依赖。

## Python 封装

`native.py` 负责按平台查找包内共享库、校验 ABI 版本、将 Pydantic 闭集模型转换为连续 `float64` 数组，并将错误码转换为带诊断的 Python 异常。未构建共享库时只允许导入，运行动态求解必须抛出明确的 `NativeKernelUnavailableError`。

## 构建和打包

- CMake + C++17 构建共享库（`cpp/axle_dynamics/CMakeLists.txt`），输出直接落到
  `src/suspension_multibody/native/`。
- `scripts/build_axle_native.py` 是跨平台入口：Windows 委派 `build_axle_native.ps1`
  使用 64 位 MinGW-w64 `g++`，其他平台直接调用 `CXX`/`c++`/`g++`/`clang++`。
  两条路径都固定 `-Wall -Wextra -Werror -fno-fast-math -O2`，并记录编译器与旗标。
- 共享库放在 `src/suspension_multibody/native/`，wheel 的 package-data 必须包含它。
- 构建元数据、编译器、优化旗标和 ABI 版本写入 `native_build.json`，并进入结果 artifact
  与 Adams 证据包。

## 线程与数值一致性

首版单线程确定性运行；线程并行只允许在明确的归约顺序下开启。浮点默认 IEEE-754 binary64，禁止 `fast-math`。编译器优化允许 `-O2`/`/O2`，但必须在性能报告记录。

## 结果协议

动态结果按输出时间写出 body states、constraint reactions、component loads、tire contact、
能量账本、诊断和接触事件；列名由 `result.py` 单一定义，并随 `layouts` 写入 artifact
manifest，同时记录模型/工况哈希与内核构建元数据。失败工况保留已接受样本和失败时刻，
不以空结果伪装通过。
