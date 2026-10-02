# p5-04 证据 (c2)：内核单步接口范围实测（逐条命令与输出）

> 实测日期 2026-10-02。与 `raw/d2_ruling.md` 同批；本节只记**已执行**的命令与输出原文。

## 实测 1：Python 侧无任何按步入口

```
$ grep -rn "kernel_step\|run_step\|step_once\|read_state\|get_state\|set_state\|advance_one" \
    packages/suspension_multibody/src --include=*.py
packages/suspension_multibody/src/suspension_multibody/adams/strict_c.py:585:  target_state = expected[case_id]
packages/suspension_multibody/src/suspension_multibody/adams/strict_c.py:588:  if field not in target_state or ...
```

两处命中均为 `strict_c.py` 内部与内核无关的局部变量名，**不是**内核入口。

## 实测 2：C++ 侧只导出批式入口

```
$ grep -rn "AXLE_API" packages/suspension_kernel/cpp/src/abi/kernel_contract_run.cpp
188:extern "C" AXLE_API int32_t suspension_kernel_contract_version()
235:extern "C" AXLE_API int32_t suspension_kernel_capabilities(
251:extern "C" AXLE_API int32_t suspension_kernel_run(
```

```
$ grep -rn "suspension_kernel_run\|mb_core_run" packages/suspension_kernel/cpp --include=*.cpp --include=*.hpp
packages/suspension_kernel/cpp/axle_dynamics/axle_kernel.hpp:43:AXLE_API int32_t suspension_kernel_run(
packages/suspension_kernel/cpp/axle_dynamics/core_abi.hpp:136:AXLE_API int mb_core_run(
packages/suspension_kernel/cpp/src/abi/kernel_contract_run.cpp:251:extern "C" AXLE_API int32_t suspension_kernel_run(
packages/suspension_kernel/cpp/src/abi/kernel_core.cpp:216:extern "C" int mb_core_run(
```

## 实测 3：Python 绑定的必填符号表只有三个（无 step/state）

`packages/suspension_multibody/src/suspension_multibody/kernel/native.py:136-145`：

```python
            required_symbols=(
                "suspension_kernel_run",
                "suspension_kernel_contract_version",
                "suspension_kernel_capabilities",
            ),
```

## 实测 4：内核自报 capability 里没有按步入口

```
$ uv run --no-sync python -c "<读 suspension_kernel_capabilities>"
capability top keys: ['contract', 'contract_version', 'pac2002_refused_families',
                      'pac2002_refused_feature_flags', 'pac2002_refused_parameters',
                      'pac2002_supported_use_modes']
```

六项全是 PAC2002 支持面与契约版本，**没有** entry_points / step / state 之类的声明。

## 结论与处置

内核是**批式 ABI**：一次 `suspension_kernel_run` 收完整 model/case 文档并返回完整结果，
无「推进一拍 / 读写状态」入口。这与 D2 终裁的实测前提一致，因此：

- **不需要**内核单步接口；
- **不触发**本 Epic 的第二次 ABI 变更；
- **不新增**专属子任务；
- **未改** `mb_config/version.hpp` 与 `kernel/native.py` 的版本常量（仍 17/32/1）。

闭环按 D2 的裁决走**内核力元求值路径**：`rotational_torque` 族的求值函数同时拿到实时 `State`
与当前样本插值输入，控制律就在这条路径内按实时轮胎纵向滑移算需求，
故「读状态 → 算控制 → 施加执行器力矩 → 影响下一拍状态」在**同一次运行内**闭合。
控制量的承载位置是新增结果块 `controller_output`（结果契约扩展，不是 ABI 变更）。
