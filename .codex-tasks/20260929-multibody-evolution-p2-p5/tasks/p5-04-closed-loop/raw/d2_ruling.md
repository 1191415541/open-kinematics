# p5-04 证据 (c)：D2 裁决确认与内核单步接口范围实测

> 实测日期 2026-10-02。父判据 `EPIC.md:281(c)`；`TODO.csv` 第 4 步**先于**第 5 步（闭环实现）。

## 1. D2 裁决结论原文（开工前置）

`EPIC.md` D 表 D2 行（终裁，2026-10-01）原文要点：

> **裁决：不需要内核单步接口——D2 不触发本 Epic 的第二次 ABI 变更，也不新增专属子任务。**
> 实测结论：批式入口确实不支持 Python/外部控制器在一次运行中逐步读写
> （`suspension_kernel_run` 只收完整 model/case 并返回完整结果；多 case 各自从模型初值重起算，
> **多 case 不是状态接续**），但闭环可在**内核力元求值路径**内成立：求解器每个内部残差评估都用当前 `State`
> 与当前插值输入调 `external_force_vector`，而力矩元族同时接收 `State` 与 `SampleInput`
> （读实时 `state.tire_sx` 与实时驾驶员信号），故控制律可在**同一次运行的推进过程中**闭环，
> 属真实反馈而非开环回放。三段记录的落点：状态段 = `body_state` 的轮速/车身加速度 +
> `tire_output` 的纵向滑移（p5-03 总线已登记前两项，须补一个 `longitudinal_slip` 测点）；
> 执行器段 = `element_wrench` 的 `kElementWrenchRotationalTorque = 10` 两行；
> **控制段现有通道没有承载位置**——`element_wrench` 的语义固定为「实际施加的 wrench」，
> 其两个 end 行不能被重新解释为控制量，元素整数槽又属静态输入，因此须新增结果块
> `controller_output`（逐样本记 `measured_slip` / `target_slip` / `control_demand`，并带驾驶员信号）。
> 该新增是**结果契约扩展，不是 ABI 变更**。

**时点关系**：本节的实测（第 2 节）与第 5 节的闭环实现在**同一批工作内并列完成**，
且第 2 节的结论决定了第 5 节不需要任何新的 ABI 子任务——这与 `TODO.csv` 第 4 步早于第 5 步的顺序要求一致。

## 2. 实测：内核确实没有 step / state 入口

命令与输出：

```
$ grep -rn "kernel_step\|run_step\|step_once\|read_state\|get_state\|set_state\|advance_one" \
    packages/suspension_multibody/src --include=*.py
（无命中；仅命中 adams/strict_c.py 里与内核无关的局部变量名 target_state/actual_state）

$ grep -rn "suspension_kernel_run\|mb_core_run" \
    packages/suspension_kernel/cpp --include=*.cpp --include=*.hpp
packages/suspension_kernel/cpp/axle_dynamics/axle_kernel.hpp:43:AXLE_API int32_t suspension_kernel_run(
packages/suspension_kernel/cpp/axle_dynamics/core_abi.hpp:136:AXLE_API int mb_core_run(
packages/suspension_kernel/cpp/src/abi/kernel_contract_run.cpp:251:extern "C" AXLE_API int32_t suspension_kernel_run(
packages/suspension_kernel/cpp/src/abi/kernel_core.cpp:216:extern "C" int mb_core_run(

$ grep -rn "AXLE_API" packages/suspension_kernel/cpp/src/abi/*.cpp | grep -v "^.*://"
packages/suspension_kernel/cpp/src/abi/kernel_contract_run.cpp:188:extern "C" AXLE_API int32_t suspension_kernel_contract_version()
packages/suspension_kernel/cpp/src/abi/kernel_contract_run.cpp:235:extern "C" AXLE_API int32_t suspension_kernel_capabilities(
packages/suspension_kernel/cpp/src/abi/kernel_contract_run.cpp:251:extern "C" AXLE_API int32_t suspension_kernel_run(
```

`kernel/native.py:136-145` 的必填符号表同样只有这三个：

```python
            required_symbols=(
                "suspension_kernel_run",
                "suspension_kernel_contract_version",
                "suspension_kernel_capabilities",
            ),
```

**结论**：内核导出的入口是**批式**的——一次调用收完整 model/case 文档并返回完整结果，
没有「推进一拍 / 读回状态 / 写回状态」的入口。**与 D2 裁决的实测一致**，
故本行**不需要**内核单步接口，**不触发**本 Epic 的第二次 ABI 变更，
**不新增**专属子任务，**不改** `mb_config/version.hpp` 与 `kernel/native.py` 的版本常量。

## 3. 闭环走的路径：内核力元求值路径（不是批式回放）

闭环成立的位置是**求解器内部**：每个内部残差评估都拿当前 `State` 与当前样本插值输入调力装配，
而 `rotational_torque` 族的求值函数签名恰好同时接收两者（`cpp/src/element/anti_roll.cpp`）：

```cpp
void assemble_rotational_torque_forces(
    const Model& model,
    const State& state,
    const SampleInput& input,
    std::vector<Vec3>& torque,
    ...
```

它已经读 `state.tire_sx`（实时轮胎纵向滑移）：

```cpp
        const double slip = actuator.demand_tire >= 0 &&
                actuator.demand_source != TORQUE_DEMAND_UNIT &&
                static_cast<std::size_t>(actuator.demand_tire) < state.tire_sx.size()
            ? state.tire_sx[static_cast<std::size_t>(actuator.demand_tire)]
            : 0.0;
```

因此「读实时状态 → 算控制 → 施加执行器力矩 → 影响下一拍状态」这条链在**同一次运行内**是闭合的，
不需要跨次回放。**控制段**（`measured_slip` / `target_slip` / `control_demand`）需要一个新的
结果块承载（见 `raw/controller_block.md` 与 `raw/closed_loop_chain.md`），
那是结果契约扩展而非 ABI 变更。

## 4. 未做

- 未改 `mb_config/version.hpp` 与 `kernel/native.py`（本行不改版本常量）。
- 未新增任何 ABI 子任务（D2 终裁为「不需要」，第 2 节实测复核了该前提）。
