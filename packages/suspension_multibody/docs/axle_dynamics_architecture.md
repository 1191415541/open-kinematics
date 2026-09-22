# 整轴与整车动力学软件架构

本文件描述**已实现**的结构。

> 修订（2026-09-13）：此前本文件记载「分目录 C++ 布局未采用、内核收敛为单一翻译单元」，
> 该决策已被推翻。C++ 内核正在从单翻译单元迁出：源码移入独立包
> `packages/suspension_kernel`，由 CMake + Ninja 构建（此前 CMake 只是仓内零调用的
> 描述文件，真实构建是单文件直编），并按职责分层为可独立编译的库。本文件所述的目录
> 与构建入口随之更新；迁移过程中的过渡形态见
> `.codex-tasks/20260913-cpp-kernel-package/MODULES.md`。

## 模块边界

```text
packages/suspension_kernel/
  cpp/axle_dynamics/        公开 C ABI 面，只剩这两个头
    axle_kernel.hpp         结构体、枚举（转引 mb_model/enums.hpp）与输出列布局注释
    core_abi.hpp            mb_core_* 的输入输出结构体
  cpp/include/<module>/     每模块的头：类型头 + `functions.hpp`
    functions.hpp 只**声明**该模块定义的自由函数，并包含本层与允许依赖层的类型头；
    编译器不再有「一个头看到全内核」的过渡形态（K6 删除了 kernel_internal.hpp）
    abi/                     version.hpp、functions.hpp
    mb_base/                 vector、constants、diagnostics、env、util、dual、dual_geometry、
                             monotone_cubic、prelude.hpp（只含标准库包含）、functions.hpp
    mb_linalg/               factorization_types、functions.hpp
    mb_model/                enums（零依赖）、types、functions.hpp
    mb_tire_state/           tire_state、functions.hpp
    mb_constraint/           types（行数与残差分类，零依赖）、registry（JointTypeDescriptor 表）、
                             functions.hpp
    mb_suspension/           functions.hpp
    mb_tire/                 model（轮胎模型注册表）、assembly、force_context、functions.hpp
      common/                kinematics、functions.hpp
      brush/ fiala/ pac2002/ functions.hpp（pac2002 另有 parameters、spin、turn_slip）
    mb_vehicle/              energy、functions.hpp
    mb_integrator/           context、functions.hpp
    mb_static/ mb_output/    functions.hpp（output 另有 measurement）
  cpp/src/<module>/         每个模块目录一个静态库目标，一文件一层
    base linalg model tire_state constraint suspension tire
    tire/{common,brush,fiala,pac2002} vehicle integrator static output abi
  CMakeLists.txt            共享库 + 16 个模块静态库，显式列出全部翻译单元；
                            模块间用 --start-group 链接，分层方向由源码级探针检查
  src/suspension_kernel/native/          本包自己的构建产物
  src/suspension_kernel/binding/         与产品语义无关的 ctypes 层：
                                         库路径解析、符号探测、ABI 门、元数据、错误类型
`.codex-tasks/20260913-cpp-kernel-package/tasks/07-extract-domains/scripts/` 下有两个可执行检查：
`check_module_layering.py`（模块依赖方向，实测边集冻结为回归基线）与
`route_declarations.py`（K6 把过渡头声明分派到各模块头、并切断各翻译单元的包含；已完成）。
packages/suspension_multibody/scripts/build_axle_native.py
                      兼容包装：委派内核构建，并把产物镜像到本包 native/
                      注意：内核侧构建不会刷新这份镜像，改了 ABI 结构体后必须跑它，
                      否则 kernel/native.py 的新鲜度守卫会拒绝加载并提示这条命令
packages/suspension_multibody/src/suspension_multibody/
  native/                                 轴语义侧加载的那份共享库副本
  axle_dynamics/schema.py   闭集 SI 物理模型与工况（Pydantic StrictModel）
  kernel/native.py          共享库查找、ABI 门、镜像新鲜度检查（不做结构体编组）
  axle_dynamics/result.py   结果对象与全部输出列名
  axle_dynamics/io.py       模型/工况加载、NPZ+JSON 结果 artifact
  adams/axle_contract.py    冻结 manifest、通道角色绑定与哈希
  adams/axle_adams_model.py 由同一 manifest 生成原生 Adams Solver 数据集
  adams/axle_equivalence.py 独立 runner、证据包、严格比较与门禁
  adams/axle_channels.py    结果到 33 个冻结通道的导出
  preparation/assembly/vehicle.py 车身、前后悬架和四轮的装配及固定轮端质量凝聚（06 由 model/vehicle.py 迁入）
  preparation/
    vehicle_dynamic.py      整车输入编组、路面/转向/驱制动映射与准备上下文
  cases/vehicle_dynamic.py   整车 model/case contract document 发射
  vehicle/service.py         整车高层执行：统一 runner、metrics 与 artifact sink
  results/vehicle.py         整车 typed result 与 family 解码适配
  adams/full_vehicle_model.py Adams 源模型的部件、关节、力元和轮胎参数导入
scripts/
  build_axle_native.py/.ps1        跨平台构建入口
  run_axle_dynamics_acceptance.py  冻结工况矩阵验收
```

## C ABI

跨边界只使用显式长度的 POD 数组、标量与契约容器（`mb_contract` 线格式）；没有 C++ 对象或
句柄跨越边界，因此不存在跨边界所有权问题。共享库导出七个符号：三个内部结构版本探测
（`axle_kernel_abi_version`、`vehicle_kernel_abi_version`、`mb_core_abi_version`）、
通用内核入口 `mb_core_run`，以及契约面的三个入口

```c
int32_t suspension_kernel_contract_version(void);

int32_t suspension_kernel_capabilities(
    char* buffer, size_t capacity, size_t* written);

int32_t suspension_kernel_run(
    const uint8_t* model_payload, size_t model_length,
    const uint8_t* case_payload,  size_t case_length,
    uint8_t* result_out, size_t* result_length_in_out,
    char* error_buffer, size_t error_capacity);
```

模型与工况以版本化契约文档进入内核，结果以同格式的结果文档返回；缓冲不足时返回 11 并把
所需长度写回 `result_length_in_out`，调用方扩容后重调。内部结构版本为 **15**（轴）与
**30**（整车），`kernel/native.py` 与 `native_build.json` 必须与之一致，否则拒绝加载。

**扁平入口已退役。** `axle_run` / `vehicle_run` 不再导出：它们的内核只剩 `kernel_abi.cpp`
里的内部 static 函数，保留的唯一原因是历史诊断仍与它们共用主体。
`AxleInput` / `VehicleInput` 以并列数组携带刚体、关节、弹簧、衬套、稳定杆、轮胎、采样时间、
路面、驱动力矩、外载和全部求解设置；`AxleOutput` / `VehicleOutput` 以调用方分配的缓冲区接收
body state、约束反力、元件输出、轮胎接触、能量账本、诊断和已定位的接触事件。这两个结构体
仍是内核内部装配的数据形状，但**不再跨越边界**：调用方不填它们，契约读取器按文档填。
结构体自身的历史（轮端框架、自转轴与 PAC2002 参数，缩放/压力/附加力矩字段，
`[DEFLECTION_LOAD_CURVE]` 与 `[BOTTOMING_CURVE]` 曲线数组，**通用运动学驱动**坐标数组——
把任意两体之间的一个自由度按规定时间函数驱动，对应 Adams 的关节 `MOTION`）
只影响内部布局，版本号仍作为「DLL 与加载器是否配套」的判据。

ABI 只传输 SI 数值；字符串仅用于错误缓冲区，以及 `suspension_kernel_capabilities` 返回的能力
文档。所有数组由调用方分配，C++ 不跨边界释放内存。
返回码：`1` 参数缺失、`2` 模型构建失败（含约束秩亏）、`3` 输出缓冲区过小、
`4` 求解设置非法、`5` 时域积分失败、`6` 初始化失败、`7` 初速度违反速度约束、
`8` 初始加速度 KKT 失败、`9` 初始轮胎压缩超限、`10` 内部状态或事件缓冲区问题、
`11` 契约结果缓冲区不足（所需长度写回调用方变量）。
失败时已接受样本、失败时刻和诊断行仍然返回；契约入口在某个用例失败时停止用例循环，
把 `failed_case` / `failed_sample_index` / `failed_time_s` / `failed_status` 写进结果 manifest，
并让块保持成功运行的形状（未跑的样本为 NaN）。

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
`preparation/assembly/vehicle.py` 合并前后轴，`preparation/vehicle_dynamic.py` 将路面、转向和轮端力矩编组到同一个
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

`kernel/native.py` 只负责按平台查找包内共享库、校验 ABI 版本与镜像是否新鲜、要求三个契约符号存在，并把加载错误转换为带诊断的 Python 异常。把 Pydantic 闭集模型转换成连续 `float64` 数组这件事已不在 Python：模型与工况以契约文档交付内核，`kernel/` 薄壳只做组包与拆包。未构建共享库时只允许导入，运行求解必须抛出明确的 `NativeKernelUnavailableError`。

## 构建和打包

- CMake + Ninja + C++17 构建共享库（`packages/suspension_kernel/CMakeLists.txt`），
  `KERNEL_SOURCES` 逐个列出全部翻译单元（不扫目录），产物复制到
  `src/suspension_kernel/native/` 与 `src/suspension_multibody/native/`。
- `packages/suspension_kernel/scripts/build_suspension_kernel.py` 是唯一的构建入口：
  它发现编译器、跑 `cmake -G Ninja`、`cmake --build`、复制产物，然后**加载产物读回**
  `axle_kernel_abi_version` / `vehicle_kernel_abi_version` / `mb_core_abi_version` 写进
  `native_build.json`。`scripts/build_axle_native.py` 与 `build_axle_native.ps1` 只是包装。
- 旗标：`-O3 -DNDEBUG -std=c++17 -Wall -Wextra -Werror -fno-fast-math -fopenmp`，
  Release 下另开 `-flto`；`-static -static-libgcc -static-libstdc++` 由 `target_link_options` 给出。
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
