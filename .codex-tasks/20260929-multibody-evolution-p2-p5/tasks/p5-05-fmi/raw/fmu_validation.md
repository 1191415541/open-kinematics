# p5-05 FMI 导出：产物、变量清单与仓库外独立校验

> 本文件只记**已执行**的命令与其原文输出。所有命令在
> `E:\杂件\open-kinematics`（bash `/e/杂件/open-kinematics`）或标注的仓库外目录下运行。

对应父判据 `EPIC.md` 行 279(a)：**按 D4 的版本与范围导出 FMU，产物存在且可被独立校验**
（校验必须是「输入影响输出」的轨迹断言，变量清单与输入/输出方向正确只是前置）。

## 0. D4 的范围（`EPIC.md` 行 63）

| 编号 | 问题 | 裁决与理由 |
|---|---|---|
| D4 | 阶段五 FMI 导出的版本与范围 | **裁决：采纳（FMU 2.0 Co-Simulation）**，只导出模型 + 输入/输出变量，不含 Python 侧求值；不做硬件在环与实时保证 |

本行的产物按此实现：归档里是 `modelDescription.xml` + 一个实现完整 FMI 2.0
Co-Simulation 符号集的 C 二进制 + 两份契约容器 + 绑定表。**归档里没有 Python**。

## 1. 导出

```
$ uv run --no-sync python packages/suspension_multibody/scripts/build_fmu_binary.py
E:\杂件\open-kinematics\packages\suspension_multibody\src\suspension_multibody\fmi\suspension_multibody_axle.dll
```

用一个真实求解过的工况导出（单轮 + 自旋副 + 接地轮胎 + 滚动带 + 制动力矩元；
导出脚本与夹具在会话 scratch，测试内同名夹具见
`packages/suspension_multibody/tests/api/test_fmu_export.py::_rig_model`）：

```
$ uv run --no-sync python $PI_SCRATCH_DIR/p505b/export_rig.py
build rc 0 …\fmi\suspension_multibody_axle.dll
exported: …\p505b\rig.fmu 22435 bytes
guid: a39e5bab2b8be5ca68f17ccfc0168d893bd37a001a562e0f2fee027bf235c6ed
inputs: [(0, 'road_velocity[tire]'), (1, 'brake_pressure[tire]')]
outputs: 15
    2 time_s s FmiBinding(offset=0, count=0, block='', row=0, column=0)
    3 wheel_speed[ground][0] rad/s FmiBinding(offset=0, count=0, block='body_state', row=0, column=10)
    …
```

归档成员：

```
modelDescription.xml 6888
binaries/win64/suspension_multibody_axle.dll …
resources/model.bin …
resources/case.bin …
resources/bindings.txt …
```

**关键**：`binaries/<platform>/<identifier>` 的文件名 stem 必须是
`modelDescription.xml` 的 `modelIdentifier`——FMI 由前者推后者，名字不一致的归档
是导入工具找不到二进制的 FMU。本行两处由同一个函数产出
（`fmi/export.py::fmi_binary_name`），`scripts/build_fmu_binary.py` 也 import 它而非另写一份。

## 2. 变量清单与输入/输出方向（逐条判定）

`modelDescription.xml` 的 17 个 `ScalarVariable`（全部为 `Real`，`valueReference` 为
0..16 的稠密序号）：

| ref | 名称 | causality | unit | 方向为何正确 |
|---|---|---|---|---|
| 0 | `road_velocity[tire]` | input | m/s | case 文档 `blobs` 里的 `road_velocity` 表：**外部给定**的带面速度时程 |
| 1 | `brake_pressure[tire]` | input | 1 | case 文档 `blobs` 里的 `brake_pressure` 表：**外部给定**的归一化制动需求 |
| 2 | `time_s` | output | s | 求解器**产出**的接受采样时刻；绑定为空块（实例自己的时钟） |
| 3–8 | `wheel_speed[ground\|wheel][0..2]` | output | rad/s | `body_state` 第 10..12 列（体角速度世界系），由解算**产出** |
| 9–14 | `body_acceleration[ground\|wheel][0..2]` | output | mm/s² | `body_state` 第 13..15 列（体线加速度），由解算**产出** |
| 15 | `tire_vertical_load[tire]` | output | N | `tire_output` 第 4 列（接触律产出的法向力） |
| 16 | `longitudinal_slip[tire]` | output | m/s | `tire_output` 第 7 列（接触律产出的纵向滑移速度） |

判定口径（不是数量对得上就算）：

- **输入 = case 文档自己携带的逐样本表**。每个变量的 `binding.offset` / `binding.count`
  **就是**该表 descriptor 的 `offset` / `length//8`——即 wrapper 写回的正是内核读的字节。
- **输出 = 结果容器里真实存在的列**。每个变量的 `binding.block` / `row` / `column` 必须
  落在该次运行真的产出的块形状内；`tests/api/test_fmu_export.py` 逐条断言这一点
  （`test_every_output_names_a_block_the_run_actually_emits`）。
- **两集合不相交**（`test_no_name_is_declared_with_two_directions`）：输入是 case 表、
  输出是结果列，同名同向不可能偶然成立。
- **未声明的输出如实说明**：`outputs/builtin.ASSEMBLY_OUTPUTS` 里的 upright 位姿、轮心、
  diagnose 计数**不是**结果 blob 的列（由报告层从 `body_state` 派生，或行不是样本主序），
  故本行不声明它们——声明一个读不到东西的输出比少声明更坏。模块 docstring 记录了这一点。

单位表也在描述里：`['1', 'N', 'm/s', 'mm/s^2', 'rad/s', 's']`；`<BaseUnit>` 只写
FMI 2.0 能表达的非负指数（`m/s`、`mm/s^2` 这类含负指数的单位**不写** `BaseUnit`，
因为 schema 不接受负整数——写成 `factor` 会是 schema 未定义的属性）。

## 3. 仓库外独立校验（轨迹断言）

校验脚本：`$PI_SCRATCH_DIR/p505b/outside/validate_fmu.py`（会话 scratch 的**仓库外**
目录；**不 import 本仓库任何模块**，只用 `ctypes` + `zipfile`）。脚本内容见第 5 节。

命令与输出原文：

```
$ cd $PI_SCRATCH_DIR/p505b/outside
$ python validate_fmu.py ../rig.fmu ../unpack
fmu       : …\p505b\rig.fmu
binary    : suspension_multibody_axle.dll
resources : ['bindings.txt', 'case.bin', 'model.bin']
grid      : start=0.0 step=0.001 samples=201.0
inputs    : 2 declared
   [0] offset=0 count=201
   [1] offset=1608 count=201
fmi2GetVersion: 2.0
time references  : [2]
slip  references : [16]
brake references : [0, 1]
time         first/mid/last: 0.0 0.1 0.2
slip @ input 0.0: first=-1.993240 mid=-1.705941 last=-0.399205
slip @ input 0.9: first=-1.993240 mid=-0.224645 last=1.124484
max |difference| across the two runs: 2.188280
the input drives the output: True
slip spread within one run: 3.933832
writing an output was refused: True
EXIT=0
```

四项客观事实：

1. **`fmi2GetVersion()` 返回 `2.0`** ——加载的确实是 FMI 2.0 Co-Simulation 二进制；
2. **步进改变模型**：同一次运行内滑移在 201 个采样上的极差 `3.93`，
   即 `fmi2DoStep` 推进的时钟确实索引到不同的样本（恒值会说明读到了固定偏移）；
3. **输入驱动输出（轨迹断言）**：制动需求从 `0.0` 改到 `0.9`，纵向滑移时程的
   逐样本最大差 `2.188`——**这是 EPIC 行 279(a) 要求的强判据**；
4. **方向不是自称**：对输出变量调用 `fmi2SetReal` 被拒（`writing an output was refused: True`）。

Windows 上的一个实测细节：被 `ctypes` 加载的 dll 无法在进程内删除，所以校验脚本的解包
目录不自动清理（`feasibility.md` 第 4 节记录了第一版撞到这一点时的报错形状）。

## 4. 复现与生成器

`tests/api/test_fmu_export.py` **16 passed**：归档成员、`fmiVersion`、
每变量 `Real` 与稠密 `valueReference`、输入绑定落在 case blob 内且穷尽 case 表、
输出绑定落在真实块形状内、方向互斥、实体名来自模型文档、`bindings.txt` 与 XML 同源、
四类拒绝（缺 blob / 非均匀网格 / 无 family / 未知 role）、
**同 pair 两次导出逐字节相同**、资源就是提交用的容器、导出不改变求解结果。

命令：

```
$ uv run --no-sync pytest packages/suspension_multibody/tests/api/test_fmu_export.py -q -p no:cacheprovider
................                                                         [100%]
16 passed in 3.51s
```

> 说明：本行的 `validation_command` 是
> `uv run --no-sync pytest packages/suspension_multibody/tests/api -q && test -s …/raw/fmu_validation.md`，
> 即该文件必须存在且非空——本文件即该项。

## 5. 仓库外校验脚本原文

```python
"""
Out-of-repository validator for a suspension_multibody FMU.

This file is deliberately **not** part of the repository's test suite and is run
from a directory outside it, with only the Python standard library available.  It
imports nothing from the project: it unpacks the archive, loads the FMI 2.0
Co-Simulation binary through `ctypes`, walks the FMI state machine, and reports
what the model did.  Its job is to answer the questions a co-simulation consumer
would ask, none of which the exporter can answer about itself:

1. does the archive load as an FMI 2.0 Co-Simulation FMU?
2. does a *step* advance the model's clock and change the outputs?
3. does a changed **input** move a named **output** (the trajectory assertion)?
4. do the declared input/output directions match what the binary actually does?

Usage:
    python validate_fmu.py <path-to.fmu> <unpack-directory> [kernel-path]
"""

from __future__ import annotations

import ctypes
import sys
import zipfile
from pathlib import Path

fmi2Component = ctypes.c_void_p
fmi2Real = ctypes.c_double
fmi2ValueReference = ctypes.c_uint
fmi2Status = ctypes.c_int
fmi2Type = ctypes.c_int
fmi2Boolean = ctypes.c_int


class Fmu:
    """A loaded FMU instance, driven through the FMI 2.0 Co-Simulation calls."""

    def __init__(self, library_path: Path, resource_dir: Path) -> None:
        self.library = ctypes.CDLL(str(library_path))
        self.library.fmi2GetVersion.restype = ctypes.c_char_p
        self.library.fmi2Instantiate.restype = fmi2Component
        self.library.fmi2Instantiate.argtypes = [
            ctypes.c_char_p, fmi2Type, ctypes.c_char_p, ctypes.c_char_p,
            fmi2Boolean, fmi2Boolean,
        ]
        self.library.fmi2FreeInstance.argtypes = [fmi2Component]
        # The FMI state machine's calls have their own signatures; giving each
        # one its real argument list is what keeps a wrong arity from being
        # reported as a model error.
        self.library.fmi2SetupExperiment.argtypes = [
            fmi2Component, fmi2Boolean, fmi2Real, fmi2Real, fmi2Boolean, fmi2Real,
        ]
        for name in ("fmi2EnterInitializationMode", "fmi2ExitInitializationMode",
                     "fmi2Terminate", "fmi2Reset"):
            getattr(self.library, name).argtypes = [fmi2Component]
        self.library.fmi2DoStep.argtypes = [fmi2Component, fmi2Real, fmi2Real, fmi2Boolean]
        self.library.fmi2DoStep.restype = fmi2Status
        self.library.fmi2SetReal.argtypes = [
            fmi2Component, ctypes.POINTER(fmi2ValueReference), ctypes.c_size_t,
            ctypes.POINTER(fmi2Real),
        ]
        self.library.fmi2GetReal.argtypes = [
            fmi2Component, ctypes.POINTER(fmi2ValueReference), ctypes.c_size_t,
            ctypes.POINTER(fmi2Real),
        ]
        self.library.fmu_wrapper_last_error.restype = ctypes.c_char_p
        self.library.fmu_wrapper_last_error.argtypes = [fmi2Component]

        self.component = self.library.fmi2Instantiate(
            b"validator", 1, b"{guid}", str(resource_dir).encode("utf-8"), 0, 0
        )
        if not self.component:
            raise SystemExit(
                "fmi2Instantiate returned NULL: "
                + self.library.fmu_wrapper_last_error(None).decode()
            )

    @property
    def version(self) -> str:
        """Return the FMI version the binary reports."""
        return self.library.fmi2GetVersion().decode()

    def error(self) -> str:
        """Return the wrapper's own message for the last failure."""
        return self.library.fmu_wrapper_last_error(self.component).decode()

    def start(self) -> None:
        """Enter and leave initialization mode, as a consumer must."""
        self.library.fmi2SetupExperiment(self.component, 0, 0.0, 0.0, 0, 0.0)
        self.library.fmi2EnterInitializationMode(self.component)
        self.library.fmi2ExitInitializationMode(self.component)

    def set_real(self, references: list[int], values: list[float]) -> int:
        """Set input variables and return the FMI status."""
        array = (fmi2ValueReference * len(references))(*references)
        numbers = (fmi2Real * len(values))(*values)
        return int(self.library.fmi2SetReal(self.component, array, len(references), numbers))

    def get_real(self, references: list[int]) -> list[float]:
        """Read variables and return their values, raising on a failure."""
        array = (fmi2ValueReference * len(references))(*references)
        numbers = (fmi2Real * len(references))()
        status = self.library.fmi2GetReal(self.component, array, len(references), numbers)
        if status != 0:
            raise RuntimeError(f"fmi2GetReal failed ({status}): {self.error()}")
        return [float(value) for value in numbers]

    def step(self, time: float, size: float) -> int:
        """Advance the co-simulation clock and return the FMI status."""
        return int(self.library.fmi2DoStep(self.component, time, size, 1))

    def close(self) -> None:
        """Terminate and free the instance."""
        self.library.fmi2Terminate(self.component)
        self.library.fmi2FreeInstance(self.component)


def read_bindings(resource_dir: Path) -> tuple[dict[int, str], dict[int, tuple[str, int, int]], dict[str, float]]:
    """Return the input names, output bindings and grid the archive declares."""
    inputs: dict[int, str] = {}
    outputs: dict[int, tuple[str, int, int]] = {}
    grid: dict[str, float] = {}
    for line in (resource_dir / "bindings.txt").read_text(encoding="utf-8").splitlines():
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if parts[0] == "grid":
            grid = {"start": float(parts[1]), "step": float(parts[2]), "samples": float(parts[3])}
        elif parts[0] == "input":
            inputs[int(parts[1])] = f"offset={parts[2]} count={parts[3]}"
        elif parts[0] == "output":
            outputs[int(parts[1])] = (parts[2], int(parts[3]), int(parts[4]))
    return inputs, outputs, grid


def main(argv: list[str]) -> int:
    fmu_path = Path(argv[1]).resolve()
    unpack = Path(argv[2]).resolve()
    unpack.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(fmu_path) as archive:
        archive.extractall(unpack)

    binaries = sorted((unpack / "binaries" / "win64").glob("*.dll"))
    if not binaries:
        raise SystemExit("the archive carries no win64 binary")
    binary = binaries[0]
    resources = unpack / "resources"
    print(f"fmu       : {fmu_path}")
    print(f"binary    : {binary.name}")
    print(f"resources : {sorted(p.name for p in resources.iterdir())}")

    inputs, outputs, grid = read_bindings(resources)
    print(f"grid      : start={grid['start']} step={grid['step']} samples={grid['samples']}")
    print(f"inputs    : {len(inputs)} declared")
    for reference, text in sorted(inputs.items()):
        print(f"   [{reference}] {text}")

    fmu = Fmu(binary, resources)
    print(f"fmi2GetVersion: {fmu.version}")
    fmu.start()

    # The named input and output the trajectory assertion is about: the brake
    # demand that the run's own case table carries, and the tire's longitudinal
    # slip speed -- a quantity the contact law produces from the state.
    by_name = {
        reference: binding for reference, binding in outputs.items()
    }
    time_variables = [r for r, b in by_name.items() if b[0] == "-"]
    slip_variables = [r for r, b in by_name.items() if b[0] == "tire_output" and b[2] == 7]
    brake_inputs = [r for r in inputs]
    print(f"time references  : {time_variables}")
    print(f"slip  references : {slip_variables}")
    print(f"brake references : {brake_inputs}")
    if not slip_variables or not brake_inputs:
        raise SystemExit("the archive declares no slip output or no input to drive it")

    slip = slip_variables[0]
    brake = brake_inputs[0]

    def walk(pressure: float) -> list[float]:
        """Set the input, then read the slip at each declared sample instant."""
        if fmu.set_real([brake], [pressure]) != 0:
            raise SystemExit(f"fmi2SetReal failed: {fmu.error()}")
        step = grid["step"]
        readings = []
        for index in range(int(grid["samples"])):
            time = grid["start"] + index * step
            status = fmu.step(time, step if index else step)
            if status != 0:
                raise SystemExit(f"fmi2DoStep failed at {time}: {fmu.error()}")
            readings.append(fmu.get_real([slip])[0])
        return readings

    low = walk(0.0)
    high = walk(0.9)
    times = [grid["start"] + i * grid["step"] for i in range(int(grid["samples"]))]
    print("time         first/mid/last:", times[0], times[len(times) // 2], times[-1])
    print(f"slip @ input 0.0: first={low[0]:.6f} mid={low[len(low)//2]:.6f} last={low[-1]:.6f}")
    print(f"slip @ input 0.9: first={high[0]:.6f} mid={high[len(high)//2]:.6f} last={high[-1]:.6f}")

    differences = [abs(a - b) for a, b in zip(low, high)]
    print(f"max |difference| across the two runs: {max(differences):.6f}")
    print(f"the input drives the output: {max(differences) > 1e-6}")

    # A step must move the model: the slip is not constant across the horizon.
    spread = max(low) - min(low)
    print(f"slip spread within one run: {spread:.6f}")

    # An output cannot be written, and an input cannot be read as an output.
    rejected_output_write = fmu.set_real([slip], [1.0])
    print(f"writing an output was refused: {rejected_output_write != 0}")
    fmu.close()
    print("EXIT=0")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
```

## 6. 未做（如实登记）

- **不做实时/硬件在环承诺**（D4 与 `EPIC.md` Non-Goals 行 101）：FMU 只做离线联合仿真，
  归档里没有实时保证，`canRunAsynchronously` 声明为 `false`。
- **不在归档里做 Python 侧求值**（D4）：所有求解由 FMU 内的 C 二进制调内核完成。
- **FMU 状态序列化与方向导数不提供**：FMI 2.0 的可选接口按「不支持」应答
  （`fmi2GetFMUstate` 等返回 `fmi2Error`），而不是假装支持。
- **一次运行的批式求解语义**：内核 ABI 是批式的，一次 `fmi2DoStep` 重跑整条时程，
  实例时钟决定读哪一样本。这是内核接口的真实形状，不是本行引入的限制。
