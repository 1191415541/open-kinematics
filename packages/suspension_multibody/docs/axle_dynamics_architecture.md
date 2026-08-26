# 整轴动力学软件架构

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
**13**，`native.py` 与 `native_build.json` 必须与之一致，否则拒绝加载。

ABI 只传输 SI 数值；字符串仅用于错误缓冲区。所有数组由调用方分配，C++ 不跨边界释放内存。
返回码：`1` 参数缺失、`2` 模型构建失败（含约束秩亏）、`3` 输出缓冲区过小、
`4` 求解设置非法、`5` 时域积分失败、`6` 初始化失败、`7` 初速度违反速度约束、
`8` 初始加速度 KKT 失败、`9` 初始轮胎压缩超限、`10` 内部状态或事件缓冲区问题。
失败时已接受样本、失败时刻和诊断行仍然返回。

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
