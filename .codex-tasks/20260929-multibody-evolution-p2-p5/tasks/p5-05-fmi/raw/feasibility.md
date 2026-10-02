# p5-05 可行性实测：FMI 2.0 Co-Simulation 的打包与外部队独立校验

> 实测日期 2026-10-02。本文件只记**已执行**的实验与其原文输出。
> 它验证的是「这条技术路线在本机可行、且不需要任何新依赖」，不是内核导出本身。

## 0. 为什么先做这个实验

`EPIC.md` D6 的裁决是「**条件允许**（仅 p5-05 的 FMI 库），引入前须先说明再落地」。
在向内核/产品引入任何导出代码之前，必须先证明两件客观事实：

1. 本机能编出**带 FMI 2.0 导出符号的 x64 动态库**（否则 FMU 里没有可加载的二进制）；
2. **不需要新依赖**也能打包 `.fmu` 并**在仓库外**把它加载、步进、断言输入驱动输出
   （stdlib 的 `zipfile` + `xml` + `ctypes` 是否够）。

两条都成立时，「引入 FMI 库」就不是达成 G8/D4 的必要条件。

## 1. 实测 1：fmpy 可下载（依赖可用性，不是依赖引入）

```
$ uv run --no-sync pip download fmpy -d $PI_SCRATCH_DIR/p505/wheels --no-deps
Saved .../p505/wheels/fmpy-0.3.32-py3-none-any.whl
Successfully downloaded fmpy
```

即：引入 fmpy 在本机网络条件下**可行**；但它**不是必需的**（见实测 3）。

## 2. 实测 2：x64 FMI 2.0 动态库可编

本机 `/d/MinGW/bin/gcc` 是 32 位（`-dumpmachine` → `mingw32`），而内核是 x64
（实测 `suspension_kernel.dll` 的 PE machine = `0x8664`）。用 x64 工具链：

```
$ "$WINGET_MINGW64/bin/gcc" -dumpmachine
x86_64-w64-mingw32

$ "$WINGET_MINGW64/bin/gcc" -shared -O2 -o model.dll model.c
（成功；无错误）

$ python -c "读 PE machine"
machine 0x8664
```

即：**本机能产出 x64 的、带 `fmi2*` 导出符号的动态库**。

## 3. 实测 3：零新依赖打包 `.fmu` + 仓库外独立加载并步进（**决定性**）

打包（只用 stdlib `zipfile`）：

```
$ uv run --no-sync python $PI_SCRATCH_DIR/p505/pack_fmu.py
fmu: .../p505/probe.fmu 14317 bytes
members: ['modelDescription.xml', 'binaries/win64/probe.dll']
```

**仓库外**的独立校验脚本（`$PI_SCRATCH_DIR/p505/outside/validate_fmu.py`，
**不 import 本仓库任何模块**，只用 stdlib + ctypes）：

```
$ cd $PI_SCRATCH_DIR/p505/outside
$ uv run --no-sync python validate_fmu.py ../probe.fmu ../unpack
binary: .../p505/unpack/binaries/win64/probe.dll
fmi2GetVersion: 2.0
low : {'input': 1.0, 'output': 2.1}
high: {'input': 3.0, 'output': 6.1}
OK: the input variable drives the output variable through the FMU
EXIT=0
```

关键点：

- `fmi2GetVersion()` 返回 **`2.0`**，证明加载的确实是 FMI 2.0 Co-Simulation 二进制；
- **改变输入变量 `input`（1.0 → 3.0）使输出变量 `output` 从 2.1 变为 6.1** ——
  这是「输入影响输出」的**轨迹断言**，比「变量清单与方向正确」强；
- 校验脚本在**仓库外的目录**运行，且**零仓库依赖**；
- 全链路只用 Python 标准库（`zipfile`/`xml.etree`/`ctypes`/`tempfile`）与一个 C 编译器。

## 4. 结论

- D4 的「FMU 2.0 Co-Simulation、只导出模型 + 输入/输出变量」在本机**技术可行**；
- D6 的「FMI 库」**不是达成 D4 的必要条件**——stdlib 路线已足够，
  故实际交付**不引入 fmpy**（D6 的授权因此**不被动用**，见 `raw/dependency_decision.md`）；
- Windows 上有一个必须处理的细节：**被 ctypes 加载的 dll 无法在进程内删除**，
  所以校验脚本的临时目录不能自动清理，否则会在任务完成之后抛
  `PermissionError: [WinError 5]`，把操作系统的文件锁报成模型缺陷。
  实测第一次版本就撞到了这一点（`tempfile.TemporaryDirectory()` 的 `__exit__` 清理失败），
  改为「解包目录保留」后通过。

## 5. 未做

- 未写入仓库任何文件（本节全部在会话 scratch 完成）。
- 未把 demo dll 当成内核导出——第 3 节的 FMU 是**技术可行性探针**，
  真实的 p5-05 导出产物与其校验见 `raw/fmu_validation.md`。
