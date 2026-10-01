# p5-01 冻结现状 · 六类「不存在」事实的锚点复核

全部命令本行实跑，命令原文 + 命中数 + 退出码逐条给出（命中 0 也写 0）。
排除目录统一定义为：
```
EX=--exclude-dir=.codex-tasks --exclude-dir=.git --exclude-dir=.venv \
   --exclude-dir=.venv-build-gui --exclude-dir=.mindfs --exclude-dir=build \
   --exclude-dir=dist --exclude-dir=.pi
```

---

## 1. SignalBus / Sensor / Measurement / signal_bus（EPIC F19，`:164`）

命令与命中数：

| 命令 | 命中 |
|---|---|
| `grep -rnE "class SignalBus\|class Measurement\|class Sensor\|signal_bus\|SignalBus" --include=*.py --include=*.hpp --include=*.h --include=*.cpp --include=*.yaml --include=*.json --include=*.md . $EX` | **0**（exit 1） |
| `grep -rn "class SignalBus" …` | **0** |
| `grep -rn "class Measurement" …` | **0** |
| `grep -rn "class Sensor" …` | **0** |
| `grep -rn "signal_bus" …` | **0** |
| `grep -rn "SignalBus" …` | **0** |

放宽到词根（-i）：
| 命令 | 命中 | 说明 |
|---|---|---|
| `grep -rniE "sensor" $EX`（py/hpp/cpp/md） | **5** | 全部是 Adams 侧的**通道名**，不是运行时传感器抽象：`adams/vehicle_handling.py:59/61/62` 的 `"condition_sensors"`（Adams result 组名）+ `tests/adams/test_full_vehicle_model.py:766/905` 的 `"SENSOR": 18`（Adams 元素类型计数） |
| `grep -rniE "measurement" $EX` | 55 | 与信号总线无关（多为 `measurement` 英文散文与 `maximum_measurement` 之类的字段名） |
| `grep -rniE "VariableDamp" $EX` | **0** | 无可变阻尼 |
| `grep -rniE "wheel_speed" $EX` | 42 | 有轮速数据字段，但无总线/测点抽象 |
| `grep -rniE "body_acceleration" $EX` | 5 | 同上 |

**结论：`SignalBus` / `Measurement` / `class Sensor` / `signal_bus` 全仓零命中，信号总线抽象不存在。** F19 成立。
现存的是两套互不相通的东西，交叉证据见 `outputs_vs_channels.md`。

## 2. 可变阻尼 `damper_ratio|variable_damp`（EPIC F20，`:166`）

```bash
$ grep -rniE "damper_ratio|variable_damp" --include=*.py --include=*.hpp --include=*.h \
    --include=*.cpp --include=*.md --include=*.json . $EX
（无输出）
exitcode=1
$ … | wc -l
0
```
**零命中**：无可变阻尼/CDC。F20 前半成立。

配套（F20 后半的「开环/预设」事实）：
- `authoring/documents.py:93 OVERRIDE_KEYS = frozenset({"hardpoints", "property_bindings"})` —— `override` 是总成文档的 copy-on-write 覆盖（消费点 `:675-679`、`:777-781`），不是执行器输入。
- 工况文档 `actuators` 字段：`authoring/documents.py:1018 _check_actuators`（原文见下），**只校验名字是否为该 rig 所驱坐标**，不表达任何执行器物理：
```
1018:    def _check_actuators(path: Path, payload: Mapping[str, Any], spec: Any) -> None:
1019:        """
1020:        Refuse an actuator the bench this rig names does not drive.
...
1027:        actuators = [str(name) for name in payload.get("actuators", ())]
1028:        if len(actuators) != len(set(actuators)):
...
1031:        unknown = sorted(set(actuators) - set(spec.coordinate_names()))
1032:        if unknown:
1033:            raise AuthoringError(
1034:                f"{path}: actuator(s) {unknown} are not driven by bench "
```
（SPEC/F20 写的锚点 `:948` 已过期，现为 `:1018`。）
- 主动力矩是**按时间预置的 TimeSignal**，正是路线图要废除的范式：
```
packages/suspension_multibody/src/suspension_multibody/adams/full_vehicle_model.py:1360:    wheel_drive_torque: Mapping[str, TimeSignal] | None = None,
packages/suspension_multibody/src/suspension_multibody/adams/full_vehicle_model.py:1361:    wheel_brake_torque: Mapping[str, TimeSignal] | None = None,
packages/suspension_multibody/src/suspension_multibody/axle_dynamics/schema.py:982:    drive_torque_body: str | None = None
packages/suspension_multibody/src/suspension_multibody/axle_dynamics/schema.py:983:    drive_torque_reaction_body: str | None = None
packages/suspension_multibody/src/suspension_multibody/axle_dynamics/schema.py:984:    drive_torque_axis_local: Vec3Tuple | None = None
```
（F20 写的 `adams/full_vehicle_model.py:1360` 与 `axle_dynamics/schema.py:982-984` **锚点全部成立**。`TimeSignal` 定义在 `schema/dynamic.py:24`。）

**结论：无可变阻尼/CDC 执行器输入；既有 actuator 全部是开环/预设。** F20 成立。

## 3. ABS / ESC / PID（EPIC F21，`:168`）

### 3.1 `\babs\b`

```bash
$ grep -rnE "\babs\b" --include=*.py packages src tests scripts | wc -l
457           # 全仓 py
$ … packages/suspension_multibody | wc -l
310           # 仅 multibody
```
逐行检查这些命中：**没有一条是 ABS 制动防抱死系统**。它们全是内建 `abs()` / `np.abs` / `.abs`，或 Python 参数名 `abs=`、散文里的 "abs"。真正**不是** `abs(` / `np.abs` / `.abs` 形式的行（全仓 py）共 **60** 行，抽查前 4 条：
```
packages/suspension_kinematics/src/suspension_kinematics/gui/reporting.py:97:    "max_abs_tie_rod_residual": "Max abs link residual",
packages/suspension_kinematics/src/suspension_kinematics/steering/comparison.py:135:    axes[1].plot(pitman, delta, "d-", color="#d62728", label="max abs angle delta")
packages/suspension_multibody/scripts/plot_adams_native_comparison.py:378:  ... `max abs ${formatValue(series.metric.maximum_absolute_error)}` ...
packages/suspension_multibody/scripts/plot_tire_handling_comparison.py:474:    return value.toFixed(abs < 1 ? 5 : 2);
```
（最后一条是内嵌 JS 的局部变量 `abs`；第 2、3 条是图例/标签文本；第 1 条是 ASCII 键名。再加 `tests/**/*.py` 里的 `pytest.approx(..., abs=1e-9)` 形式。）
**结论：`\babs\b` 的命中没有一条指 ABS。** F21 前半成立。

全仓代码里 `ABS`/`ESC` 作为大写标识符只有 1 处，且是 Adams 侧模型文本里的 Adams 函数名（不是本仓库实现的 ABS）：
```
packages/suspension_multibody/src/suspension_multibody/adams/axle_adams_model.py:686:
    f"ABS(VX({centre}, 1, 1, 1)*VARVAL({forward_x})"
```

### 3.2 `\besc\b` / `PID`

```bash
$ grep -rniE "\besc\b" --include=*.py --include=*.hpp --include=*.cpp --include=*.md . $EX | wc -l
0
$ grep -rnE "\bPID\b" --include=*.py --include=*.hpp --include=*.cpp --include=*.md . $EX | wc -l
0
$ grep -rnE "\bpid\b" … | wc -l
50
```
（小写 `\bpid\b` 的 50 处全部是**变量名** `pid` = point id，例如 `suspension_kinematics/cli.py:71-73`、`core/dual.py:403-409`、`solver.py:207`，与 PID 控制器无关。）

### 3.3 `controller`

```bash
$ grep -rniE "controller" . $EX --include=*.py --include=*.hpp --include=*.cpp --include=*.md | wc -l
4
```
四条全部指**内核积分器的 local-error 步长控制器**，不是控制论控制器：
```
packages/suspension_kernel/cpp/src/solve_dynamic/kernel_integrator_step.cpp:6:   // measures the controller compares against the tolerances, and the contact
packages/suspension_kernel/MODULES.md:44:  | `mb_solve_dynamic` | the dynamic solve: the residual, the Newton step, the step controller, events. | ...
packages/suspension_multibody/tests/cases/test_vehicle_kc.py:26:  local-error controller needs enough samples across that window to follow it.
packages/suspension_multibody/tests/cases/test_vehicle_kc.py:72:  #: the contract's uniform grid) is the coarsest grid the error controller
```

### 3.4 开环契约声明原文与拒绝用例

`cases/handling.py:8-11`（原文）：
```
8:Only open-loop manoeuvres are expressible here.  A closed-loop manoeuvre -- an
9:ISO lane change, say -- needs a driver following a path, which is a different
10:model rather than a different shape, and the kernel refuses those by name rather
11:than approximating them with a steering history that happens to look similar.
```
（SPEC.md:13/F21 写的锚点 `cases/handling.py:8` **成立**。）

拒绝用例 `tests/cases/test_handling.py:242`（原文）：
```
242:def test_a_closed_loop_manoeuvre_is_refused_by_name(prepared) -> None:
...
254:    document["handling"]["steering"][0]["shape"] = "iso_lane_change"
255:    model_doc, model_blob = vehicle_dynamic_model_document(model, base)
256:    with pytest.raises(Exception, match="not open-loop"):
257:        run_request(
258:            SimulationRequest(
259:                assembly="vehicle",
260:                family="handling",
261:                model=model_doc,
262:                case=document,
263:                context={"model_payload": pack_container(model_doc, model_blob)},
264:            )
265:        )
```
（SPEC.md:13/F21 写的锚点 `tests/cases/test_handling.py:242` **成立**。）

拒绝来自内核侧，原文锚点（本行 grep 得到，补充）：
```
packages/suspension_kernel/cpp/src/cases/handling.cpp:138:
    " not open-loop and is not implemented");
```

**结论：无 ABS/ESC/PID 实现；`cases/handling.py:8` 明文只表达开环，测试 `:242` 固化该拒绝。** F21 成立。

## 4. FMI / FMU / co-simulation（EPIC F22，`:170`）

```bash
$ grep -rniE "fmi|fmu|cosim|co-simulation|co_simulation" . $EX | wc -l
518            # 含 artifacts/（Adams 原始 .adm 日志、ftire.log）

$ grep -rniE "fmi|fmu|cosim|co-simulation|co_simulation" . $EX --exclude-dir=artifacts | wc -l
5
$ grep -rniE "fmi|fmu|cosim|co-simulation|co_simulation" . $EX --exclude-dir=artifacts
Binary file ./packages/suspension_kinematics/images/plot.png matches
./.gitignore:113:#   https://pdm.fming.dev/latest/usage/project/#working-with-version-control
./packages/suspension_multibody/docs/multibody_architecture_evolution.md:160:    公共 API 现代化 & FMI 闭环总线                   :2027-01, 2027-02
./packages/suspension_multibody/docs/multibody_architecture_evolution.md:198:### 阶段五：公共 API 现代化与信号闭环总线（FMI）
./packages/suspension_multibody/docs/multibody_architecture_evolution.md:203:  3. 支持简单的闭环控制（ABS/ESC）与标准 FMI 联合仿真导出。

$ grep -rniE "fmi|fmu|cosim|co-simulation|co_simulation" packages src tests scripts docs justfile pyproject.toml | wc -l
43             # 绝大多数是 packages/suspension_kernel/build/**.obj 的二进制命中
$ grep -rniE "fmi|fmu|cosim|co-simulation|co_simulation" packages src tests scripts docs justfile pyproject.toml | grep -v "^Binary file"
./packages/suspension_multibody/docs/multibody_architecture_evolution.md:160: ...
./packages/suspension_multibody/docs/multibody_architecture_evolution.md:198: ### 阶段五：...（FMI）
./packages/suspension_multibody/docs/multibody_architecture_evolution.md:203:   3. 支持简单的闭环控制（ABS/ESC）与标准 FMI 联合仿真导出。
```
剔除 `artifacts/`（Adams 参考产物，非本仓库实现）、`build/`（编译产物二进制命中）、`.gitignore`（指向第三方站点 `fming.dev`）之后，**本仓库代码与文档里的 FMI/FMU/联合仿真命中只有 3 处，全部在路线图文档本身**：

```
packages/suspension_multibody/docs/multibody_architecture_evolution.md:160:  公共 API 现代化 & FMI 闭环总线       :2027-01, 2027-02
packages/suspension_multibody/docs/multibody_architecture_evolution.md:198:### 阶段五：公共 API 现代化与信号闭环总线（FMI）
packages/suspension_multibody/docs/multibody_architecture_evolution.md:203:  3. 支持简单的闭环控制（ABS/ESC）与标准 FMI 联合仿真导出。
```

**结论：本仓库零 FMI/FMU/co-simulation 实现**（只命中路线图文档本身，与 F22 原文一致）。F22 成立。

---

## 5. 内核批式 ABI（EPIC F23，`:172`，D2 的硬约束）

### 5.1 两个入口的签名原文

`packages/suspension_kernel/cpp/axle_dynamics/axle_kernel.hpp:43-51`（声明）：
```
43:AXLE_API int32_t suspension_kernel_run(
44:    const std::uint8_t* model_payload,
45:    std::size_t model_length,
46:    const std::uint8_t* case_payload,
47:    std::size_t case_length,
48:    std::uint8_t* result_out,
49:    std::size_t* result_length_in_out,
50:    char* error_buffer,
51:    std::size_t error_capacity);
```
定义：`packages/suspension_kernel/cpp/src/abi/kernel_contract_run.cpp:251-256`（同签名）。

`packages/suspension_kernel/cpp/axle_dynamics/core_abi.hpp:135-141`：
```
135:AXLE_API int mb_core_abi_version();
136:AXLE_API int mb_core_run(
137:    const MbCoreInput* input,
138:    MbCoreOutput* output,
139:    char* error_buffer,
140:    std::size_t error_capacity
141:);
```
（SPEC.md:13/F23 写的 `cpp/axle_dynamics/core_abi.hpp:135-136` **成立**。）

### 5.2 无 step/state 入口的 grep 证据

```bash
$ grep -rniE "kernel_step|run_step|step_once|read_state|get_state|set_state|advance_one"     packages/suspension_kernel/cpp/axle_dynamics     packages/suspension_kernel/cpp/src/abi     packages/suspension_multibody/src/suspension_multibody/kernel
（无输出）
exitcode=1
```
```bash
$ grep -rniE "_step|_state|kernel_step|set_state|get_state|advance"     packages/suspension_kernel/cpp/axle_dynamics/*.hpp
core_abi.hpp:75:    int adaptive_step;
core_abi.hpp:76:    double internal_step;
core_abi.hpp:77:    double min_step;
core_abi.hpp:78:    double max_step;
core_abi.hpp:108:    double* body_state;
```
这 5 条**全是输入/输出结构体字段**（步长参数与结果缓冲指针），**不是入口函数**。

Python 侧绑定的符号也印证这一点（`kernel/native.py:139-143`）：
```
139:            required_symbols=(
140:                "suspension_kernel_run",
141:                "suspension_kernel_contract_version",
142:                "suspension_kernel_capabilities",
143:            ),
```
**只有 run / 版本 / 能力声明三个符号；没有任何 step 或按调用读写状态的符号。**

**结论：内核是严格的批式 ABI——一次调用跑完整个时间历程，无单步步进、无按调用读状态。** F23 成立（D2 的硬约束原文固化完毕）。

### 5.3 Python 侧唯一同步入口

`kernel/__init__.py:186-192`：
```
186:def run_contract(
187:    model_document: dict[str, Any],
188:    case_document: dict[str, Any],
189:    *,
190:    model_payload: bytes | None = None,
191:    case_payload: bytes | None = None,
192:) -> ContractRun:
```
`kernel/__init__.py:174-183`（状态反序列化）：
```
174:def _read_blocks(document: dict[str, Any], blob: bytes) -> dict[str, np.ndarray]:
175:    blocks: dict[str, np.ndarray] = {}
176:    for descriptor in document.get("blocks", []):
177:        dtype = _DTYPE[str(descriptor["dtype"])]
178:        offset = int(descriptor["offset"])
179:        length = int(descriptor["length"])
180:        shape = tuple(int(extent) for extent in descriptor["shape"])
181:        values = np.frombuffer(blob, dtype=dtype, count=length // dtype().itemsize, offset=offset)
182:        blocks[str(descriptor["name"])] = values.reshape(shape).copy()
183:    return blocks
```
状态块布局（`kernel/__init__.py:14-16`）：
```
14:The concrete block layout a caller gets back: ``body_state`` is
15:``[sample, body, 19]`` with ``[x, y, z, qw, qx, qy, qz, v, omega, ...]``, and
16:``blocks`` is keyed by block name so a caller never indexes the blob itself.
```
调用链：`run_contract` `:200-202` 打包 → `_invoke(_library(), model_bytes, case_bytes)` → `suspension_kernel_run`；返回码 `11` 表示缓冲区不足，按 `length.value` 增长重试（`:163-167`）。`_DTYPE` 在 `:32`。

**唯一归属复核**：Python 侧生产代码里 `run_contract` 只有一处调用点 —— `simulation/backend.py:24`（`NativeContractBackend.run`），见 `callers.md` §4。

**结论：Python 侧唯一同步入口就是 `run_contract`；状态从返回文档的 `blocks["body_state"]`（`[sample, body, 19]`）一次性反序列化，无法在推进中途读状态或写执行器。这就是 D2 的硬约束。**
