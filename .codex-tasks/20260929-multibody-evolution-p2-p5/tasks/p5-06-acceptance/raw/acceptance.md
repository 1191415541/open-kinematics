# p5-06 终局独立验收记录

本行是 Epic `20260929-multibody-evolution-p2-p5` 的收口验收。所有命令由主代理本人重跑，
未采信任何子任务自报结论（`EPIC.md:289`）。每条都记命令、退出码与依据行号。

验收期间暴露并修复了两个真实缺陷（下面单独成节），**修复前后的实跑结果都原文保留**，
不把修复后的绿灯写成「验收一直全绿」（p5-06 SPEC 第 6 条要求）。

**验收起点提交**：`1fe7544`（p5-05 FMI 导出）。
**收口提交**：`7e9f2dd`（FMU 构建落盘修复 + 转向换算按声明口径修正）。

---

## 0. 验收期间暴露的两个真实缺陷

### 0.1 缺陷 A：FMU wrapper 在非 ASCII 仓库路径下无法链接（全量 16 errors）

**首次暴露**：全量 `pytest packages/suspension_multibody/tests -q` 实跑
`1730 passed, 1 skipped, 1 xfailed, 16 errors`。16 个 error 全部是
`tests/api/test_fmu_export.py` 的模块级 fixture `wrapper_binary` 构建失败：

```
ld.exe: cannot open output file E:\杂件\open-kinematics\packages\suspension_multibody\
  src\suspension_multibody\fmi\suspension_multibody_axle.dll: No such file or directory
collect2.exe: error: ld returned 1 exit status
```

**一度被误判为并发假象**（隔离跑 `tests/api` 时 75 passed）。后续受控实验推翻了该判断：
单独跑 `pytest packages/suspension_multibody/tests/adams` 之后紧接着跑
`build_fmu_binary.py` **必失败**；期间无任何其它 pytest 进程；失败后立刻重试又成功。

**根因**（受控实验，逐轮可复现）：MinGW binutils 的 `ld.exe` 用窄字符 API 打开输出文件、
按**当前控制台代码页**解码该路径。仓库路径含非 ASCII（`杂件`），代码页 1252/65001 无法表示，
路径解码即错、报「No such file or directory」——而目录确实存在且可写（`touch` 成功）。
实测同一 gcc 命令同一源文件：

| 输出路径 | 控制台代码页 | 结果 |
|---|---|---|
| 仓库内（非 ASCII） | 936 (GBK) | 成功 |
| 仓库内（非 ASCII） | 1252 | 失败 |
| 仓库内（非 ASCII） | 65001 | 失败 |
| 仓库内（非 ASCII） | 936（再切回） | 成功 |
| 系统 temp（ASCII） | 任意 | 成功 |

反复 3 轮，稳定。目录无占用、无权限问题（`ls` 确认文件不存在、`touch` 成功）。

**触发链**：`tests/adams/test_probe.py::test_local_adams_profile_discovers_expected_template`
会真的 `subprocess.run` 一个 Adams `.bat` 启动器；控制台代码页是 console 级共享状态，
子进程 `chcp` 改变父进程后续读到的值。实测该用例单独跑一次即把代码页从 936 变为 1252
（该文件 7 条用例中只有这一条会改）。全量里 `tests/adams` 排在 `tests/api` 之前，
故 16 errors 在干净单进程下必然复现。

**裁定**（code-reviewer `2240db96`）：这是 p5-05 的产品缺陷，应在 Epic 内修；退回 p5-05
修脚本，p5-06 只记录失败并在修复后重新独立验收（p5-06 SPEC 第 36 行「只读验收轮」）。
修法「ASCII 暂存目录编译 + `shutil.copy2` 回原位」，且必须处理「temp 目录本身也可能非 ASCII」。
裁定同时指出：**不构成判据弱化**——只改产物落盘方式，编译命令、源码、断言与基线均未动；
仓库已有同类先例 `scripts/build_axle_native.py`（异地构建 + 复制回包内）。

**修复**（`packages/suspension_multibody/scripts/build_fmu_binary.py`，提交 `7e9f2dd`）：
输出路径非 ASCII 时先编到 ASCII 暂存目录再复制回原位；暂存目录由 `TEMP`/`TMP`/系统 temp
探测并**实测确认** ASCII，探测不到即就地编译（等同 ASCII 检出机的既有行为）；
`--output` 的最终路径语义不变；暂存目录在 `finally` 清理。
`build_axle_native.py` 不受影响（走 CMake 到 build 目录再 `copy2`，实测同条件下正常）。

**修复后验证**（全部在故障态下实跑）：

| 检查 | 命令 | 结果 |
|---|---|---|
| 失败态构建 | `chcp 1252` 后 `python scripts/build_fmu_binary.py` | EXIT=0，产出 60747 B |
| 故障顺序复现 | 先 `pytest tests/adams/test_probe.py`（代码页 936→1252），再跑 FMU | `16 passed` |
| `--output` 语义 | `--output <ascii>/custom_out.dll` | EXIT=0，落点即指定路径 |
| 暂存目录清理 | 构建后查 `%TEMP%\fmu_build_*` | 无残留 |
| 全量 | `pytest packages/suspension_multibody/tests -q` | **0 errors**（见第 2 节） |

**残留风险（如实登记）**：`tempfile` 不保证其位置 ASCII（用户名可能是路径的一部分），
所以代码按**实测**拒绝非 ASCII 候选而非信任它；若该机器上不存在任何 ASCII 临时目录，
脚本退回就地编译，在非 ASCII 检出 + 非 GBK 代码页下仍会失败。本机实测探测成功。
CI 用 ASCII 路径不会暴露此缺陷，故本节的复现链必须保留。

### 0.2 缺陷 B：转向输出按名字猜物理类型，prescribed rotation 被误缩放

**来源**：code-reviewer `442ffad6` 在裁决 steering_input 回归时指出的第二处静默数值缺陷。

原实现（`adams/full_vehicle_correlation.py`）按**名字**猜物理类型：先试 `front_rack` 并除以
`steering_ratio_m_per_rad`，查不到才用 `steering_input` 原值。p2-06 之后 prescribed rotation
的通道名也叫 `front_rack`，于是命中前支、把**角度**按 m/rad 换算。

**主代理独立复现**（真实 Adams 源算例 `artifacts/adams-full-source/step_steer`；
`probe_scaling.py`）：`actuator_mode=prescribed_rotation`、`input=steering_wheel_angle`、
`ratio=27.6`、`rack_disp_per_swa=5.729577951289618`、`channel_name=front_rack`。
输出第 2 列**等于方向盘转角信号本身**（不是齿条位移）。修复前值 `column[2]/ratio` 末样本
`2.6991247722060177e-06`；修复后 `1.546484578261121e-05`；两者之比 **5.729577951289618**
（恰为齿条比）——即修复前偏小 5.73 倍。

**修复**（提交 `7e9f2dd`）：由调用方声明的 `case.vehicle.steering.actuator_mode` 决定换算、
用声明的 `channel_name` 取输出；未声明通道名、或声明的名字在运行结果里不存在，都直接报错
而不猜测。两个调用点同步：
`scripts/run_full_native_three_model_comparison.py::_native_handling_history` 与
`adams/full_vehicle_mbd_comparison.py::compare_full_vehicle_mbd_case`。

**修复后验证**（同一真实算例，`verify_point3.py`）：

```
reported steering_angle[-3:] : [-8.76486655e-15  2.82739771e-16  1.54648458e-05]
case steering signal[-3:]    : [-8.76486655e-15  2.81709606e-16  1.54648458e-05]
max |reported - signal|      : 1.0301656715891476e-18
reported == signal           : True
reported is NOT the pre-fix  : True
```

**测试补充**：`tests/adams/test_full_vehicle_correlation.py` 从 2 条扩到 **8 passed**（新增
齿条换算、旋转读角度、改名寻址、未声明通道拒绝、缺名字拒绝、非正 ratio 拒绝）；其中
`test_native_handling_reads_a_prescribed_rotation_as_an_angle` 直接锁死「mode 决定换算」。

### 0.3 缺陷 C（本行造成的）：p5-05 修复首版删除了仓库工作树

**如实登记**：在实施 0.1 的修复时，首版写了一个真值判断错误——用 `staged = Path()` 当
「无暂存目录」的哨兵，而 `Path()` 即 `.` 且在 Python 中**恒为真值**（`bool(Path()) == True`）。
当 `--output` 指向 ASCII 路径时 `_staged_build_directory()` 未被调用、`staged` 保持 `Path()`，
于是 `finally: shutil.rmtree(staged)` 执行了 `shutil.rmtree('.')`，配合 `ignore_errors=True`
删除了仓库根目录。**这是本行引入的缺陷，不是环境问题。**

**损失与恢复**：

- **已恢复**：`.git/objects` 幸存（215 MB，271 commit / 3115 tree / 4288 blob），已备份并据此
  重建仓库，`1fe7544` 的 1535 个跟踪文件全部还原，工作树干净。内核 DLL 与 `.venv` 已重建，
  导入正常。在途改动（0.1、0.2 的修复）全部重新应用。
- **不可恢复**：`artifacts/` 目录。它被 `.gitignore:211` 排除、从未被 git 跟踪，
  `shutil.rmtree` 不经回收站，本机无 Adams 安装（`MSC.Software\Adams` 不存在）故
  `regenerate_adams_reference.py` 无法重跑。内含 Adams 参考数据
  （`adams-full-source*/step_steer`、`adams-mode-ref/**`）。

**后果**：`tests/adams` 的 skip 从基线 **1** 增至 **47**（见第 2 节逐条登记）。
数值门三项与 sentinel 不受影响：sentinel 自己跑求解并重建
`artifacts/axle-dynamics-acceptance`（26 artifact 逐字节一致，combined sha256 与冻结值相同），
`case_parity_check.py` 是自足的。

**处置**（用户裁定）：按「已知环境缺失」如实登记 47 个 skip 的来源与影响，继续 Epic 收口。

**修复**：`staged` 改为 `None` 哨兵，全部判断用 `staged is not None`；`Path()` 不再出现在
哨兵位置（`grep -c "staged = Path()"` 为 0）。

---

## 1. Done-When (a)–(j) 逐条实跑

每条给命令、退出码、依据的 `EPIC.md` 行号。括号内为证据文件。

### (a) 力矩内建 — 依据 `EPIC.md:292-295`

| 项 | 命令 | 退出码 | 结果 |
|---|---|---|---|
| 声明分支隔离 | `grep -n "_build_wheel_torque_signals" src/.../subsystems/torque_elements.py` | 0 | 只在 `none` 兜底分支被调用 |
| `front_brake_bias` opt-in 读取次数 | `grep -n "front_brake_bias" src/.../subsystems/` | 0 | opt-in 路径 0 次 |
| 静止/倒车/抱死三态 | `pytest tests/subsystems/test_torque_element_wiring.py -q` | 0 | 全通过 |
| 力矩时程一致 | `pytest tests/cases/test_rotational_torque_document.py -q` | 0 | 全通过 |
| 非零响应独立夹具 | `pytest tests/subsystems/test_torque_element_wiring.py -q` | 0 | code10 rows 42，首样本 1.0 N·m（rel=1e-12），逐样本 atol=0 |

判据修订见裁决 `ddc3f952`（原「两符号 grep 全仓无命中」与「8 个冻结用例逐位一致」不可同时
成立，已改为声明分支隔离），已同步 `SUBTASKS.csv`、p2-05 `SPEC.md` 与 `EPIC.md` Done-When (a)。

### (b) 转向通道 — 依据 `EPIC.md:296-299`

| 项 | 命令 | 退出码 | 结果 |
|---|---|---|---|
| 两通道总成跑通 | `pytest tests/preparation/test_steering_channels.py -q` | 0 | 8 passed |
| 分配律断言 | `pytest tests/preparation/test_steering_allocator.py -q` | 0 | 24 passed |
| 后轮转向限制解除 | `pytest tests/schema/test_vehicle_steering_channels.py -q` | 0 | 14 passed |
| schema 三项逐项不变 | `pytest tests/schema/test_vehicle_steering_channels.py tests/authoring -q` | 0 | 12 键 dump / 11835 B / canonical_hash 逐字节未变 |

判据修订见裁决 `1331b13d`（方案 B：保留 `steering` 必填单例）。
**同时登记一处契约反转**（裁决 `442ffad6` 点 1、点 2）：p2-06 把 actuator 名从既有字面量
`"steering_input"` 改为 `steering_spec.channel_name`（默认 `"front_rack"`）。
旧名是既有的结果寻址契约，新契约是**所有 actuator 以声明的 `channel_name` 寻址**。
`tests/adams/test_full_vehicle_model.py::test_source_prescribed_steering_reports_rate_in_actuator_coordinates`
因此改调 `model.steering.channel_name` 并补 `result.steering_names` 显式断言；
**数值断言未放宽**（1 passed）。

### (c) 通用运动学 — 依据 `EPIC.md:300-302`

| 项 | 命令 | 退出码 | 结果 |
|---|---|---|---|
| 四种构型滚转中心 | `pytest tests/physics/test_roll_centres_by_topology.py -q` | 0 | 全通过 |
| 无硬点名称嗅探 | `pytest tests/vehicle/test_screw_kinematics.py -q` | 0 | 全通过 |
| 瞬轴解析解 | 同上 | 0 | 全通过 |
| 滚转中心虚功导数矩阵 | `pytest tests/physics -q` | 0 | 全通过 |

### (d) 广义静平衡 — 依据 `EPIC.md:303-305`

| 项 | 命令 | 退出码 | 结果 |
|---|---|---|---|
| 3 轴 6 点 | `pytest tests/physics/test_static_loads.py -q` | 0 | 全通过 |
| 可解/不可解例 | 同上 | 0 | 载荷相容、rank(A)=1=N 唯一性、残差超容差报错各有例 |
| 4 轮逐位一致 | 同行 | 0 | 零回归硬门通过 |

### (e) ARB 独立 — 依据 `EPIC.md:306-307`

| 项 | 命令 | 退出码 | 结果 |
|---|---|---|---|
| ARB 插到两种构型 | `pytest tests/subsystems/test_anti_roll_bar_subsystem.py tests/subsystems/test_arb_mount_ports.py -q` | 0 | 全通过 |
| `upright_L`/`upright_R` 在 ARB 路径无命中 | `grep -rn "upright_L\|upright_R" src/.../subsystems/anti_roll_bar.py` | 1 | 0 命中 |

### (f) 轮端统一（独立复验阶段一 04 的实际交付）— 依据 `EPIC.md:75`、`:93`、`:271`

**不看阶段一自报**，按文件路径与内容指纹独立复验：

| 项 | 命令 | 退出码 | 结果 |
|---|---|---|---|
| 单轴与整车同一份 wheel 文件 | `sha256sum src/.../subsystems/wheel.py` | 0 | `6313b9c8898f8d4b2ec0b35539d45f42c2c3c00bfdb6ddd3b9d01ebda3091729`；两侧消费同一路径 |
| `VerticalTireElement` 在装配路径无命中 | `grep -c VerticalTireElement src/.../subsystems/assembler.py` | 1 | **0** |
| `assembler.py` 的 isinstance 过滤不存在 | `grep -c isinstance src/.../subsystems/assembler.py` | 1 | **0** |

（`element_build.py` 的构建性引用与 `rig_link.py` 的宿主属阶段一 05 范围，不在 (f) 判据内。）

### (g) API — 依据 `EPIC.md:95`、`:310`

| 项 | 命令 | 退出码 | 结果 |
|---|---|---|---|
| `simulate` 端到端 | `pytest tests/api/test_simulate.py -q` | 0 | 18 passed |
| `FrontAxleModel` 仍可用 | `pytest tests/api tests/authoring -q` | 0 | 全通过 |
| 三个公共 API 门禁全绿 | `legacy_surface_gate.py --check`；`check_composable_release.py --skip-isolation`；`pytest tests/architecture -q` | 0/0/0 | findings 0；3 PASS；147 passed |

### (h) 总线与闭环 — 依据 `EPIC.md:97`、`:308-309`

| 项 | 命令 | 退出码 | 结果 |
|---|---|---|---|
| 总线双向读写 | `pytest tests/api/test_signal_bus.py -q` | 0 | 13 passed |
| 测点与结果文档逐项对照 | 同上 | 0 | 全通过 |
| 执行器输入改变同次仿真轨迹 | 同上 | 0 | 前后两次运行轨迹对照 |
| **ABS 实际反馈闭环 + 同次运行三段数值记录** | `pytest tests/cases/test_abs_closed_loop.py -q` | 0 | **6 passed** |
| ABI 第二次变更已按 D2 裁决 | `pytest tests/architecture/test_kernel_abi_version_single_source.py -q` | 0 | 绿；D2 终裁「不需内核单步接口」，未发生第二次 ABI 变更 |
| FMI 产物可被仓库外脚本加载并步进 | 见第 3 节 | 0 | 轨迹断言通过 |

**闭环三段**（`run_log.md` 记录）：`controller_output` 结果块宽 4
（`[0]measured_slip [1]target_slip [2]control_demand [3]driver_demand`），
开关 `SUSPENSION_KERNEL_CONTROLLER_OUTPUT` 默认关闭。控制律
`authority = clamp(1 + gain*(target - |slip|), 0, 1)`、`demand = driver_demand * authority`，
**符号与形式均经实测修正**（原稿正反馈在 t=0.05 使 Newton 不收敛）。
载体为自建滚动轮试验台（既有四个夹具均不可用）。开环回放不算，故载体是实际闭环。

### (i) 零回归 — 依据 `EPIC.md:310`

| 项 | 命令 | 退出码 | 结果 |
|---|---|---|---|
| `kc_baseline` 逐字节未变 | `git status --short -- packages/suspension_multibody/tests/data/` | 0 | 空 |
| `dynamic_hash_baseline` 逐字节未变 | `dynamic_hash_sentinel.py --check` | 0 | 26 artifact；combined sha256 `fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9`（与冻结值相同） |
| 快速集 | 见第 2 节 | 0 | 全绿 |
| `tests/architecture` | 见第 2 节 | 0 | 147 passed |
| 数值门三项 | 见第 2 节 | 0/0/0 | 全绿 |
| 全量回归 | 见第 2 节 | 0 | 1700 passed |
| **无新增 skip/xfail** | 见第 2 节 | — | **未满足**：skip 1→47（缺陷 C，见 0.3） |

### (j) ABI — 依据 `EPIC.md:311-312`

| 项 | 命令 | 退出码 | 结果 |
|---|---|---|---|
| 本 Epic 的 ABI 变更全部登记 | `grep -n kAxleKernelAbiVersion src/.../mb_config/version.hpp` | 0 | `= 17`（p2-02 登记：16/31/1 → 17/32/1） |
| 单一真源门绿 | `pytest tests/architecture/test_kernel_abi_version_single_source.py -q` | 0 | 绿 |
| 车辆/核心常量同步 | `grep -n kVehicleKernelAbiVersion\|kCoreKernelAbiVersion` | 0 | `= 32`、`= 1` |

D2 终裁「不需要内核单步接口」，故 p5-04 未触发第二次 ABI 变更——不是登记缺口，而是裁决结论。

---

## 2. 全量回归与各门（命令、退出码、原文）

### 2.1 修复前的全量（保留原文，不掩盖失败）

```
$ uv run --no-sync pytest packages/suspension_multibody/tests -q -p no:cacheprovider
1730 passed, 1 skipped, 1 xfailed, 16 errors in 1789.59s (0:29:49)
```

16 errors 全为 `tests/api/test_fmu_export.py` 的 `wrapper_binary` fixture（缺陷 A）。
另有 `tests/adams/test_full_vehicle_model.py::test_source_prescribed_steering_reports_rate_in_actuator_coordinates`
的 `KeyError: "unknown steering actuator 'steering_input'"`（缺陷 B 的命名面，已修）。

### 2.2 修复后的全量

```
$ uv run --no-sync pytest packages/suspension_multibody/tests -q -p no:cacheprovider
1700 passed, 47 skipped, 1 xfailed in 859.37s (0:14:19)
```

**0 failed / 0 errors**。

### 2.3 skip 与 xfail 对照（`AGENTS.md` 第 3 节基线 1297 passed / 1 skipped / 1 xfailed）

xfail：**1**，与基线一致，未增长。

skipped：**47**（基线 1，**+46**），全部源于缺陷 C 的 `artifacts/` 丢失。逐条登记：

| 文件 | skip 数 | 原因（原文） |
|---|---|---|
| `tests/adams/test_native_fiala_correlation_gate.py` | 16 | `artifacts/adams-full-source-fiala/step_steer` 不可用 |
| `tests/adams/test_full_vehicle_model.py` | 15 | `real Adams reference artifacts are unavailable` / `strict Adams source artifacts are unavailable` |
| `tests/adams/test_pac2002_adams_correlation_gate.py` | 14 | `artifacts/adams-mode-ref/**`、`artifacts/adams-full-source-2025_1_1/step_steer` 不可用 |
| `tests/adams/test_pac2002_parking_torque_structure.py` | 2 | `mode-25 parking reference unavailable` |

其中 `test_full_vehicle_model.py:841` 的 1 条**是基线 skip**（`artifacts/adams-full-source`
在验收起点就只在装了 Adams 的机器上存在）。**新增的 46 条**全部是同一目录缺失。

这些 skip 是既有 `skipif(not <dir>.is_dir(), ...)` 的**守卫行为**，不是新加的 skip：
测试代码未改动，新 skip 未引入。但按 `EPIC.md:233` 与 (i) 的字面要求，**计数确已增长**，
故 (i) 判定为**未完全满足**，并在此如实登记而非以「环境缺失」一笔带过。

**可复现性**：这些用例原依赖本机 Adams/Car 安装生成的参考数据（`regenerate_adams_reference.py`
明确记录「该 `.res` 文件不纳入版本控制，装了 Adams/Car 的人可重新生成」）。本机无 Adams，
无法重新生成；恢复需要外部 Adams 参考数据。

### 2.4 其余各门（全部实跑，退出码 0）

| 门 | 命令 | 退出码 | 结果 |
|---|---|---|---|
| 三架构门 | `legacy_surface_gate.py --check` | 0 | mode migration，findings **0** |
| | `check_module_layering.py --strict --final` | 0 | 0 环，cross-aggregate 0 |
| | `check_composable_release.py --skip-isolation` | 0 | 3 release checks passed |
| 数值门 | `dynamic_hash_sentinel.py --check` | 0 | 26 artifact，sha256 `fdfd5a6b…eebc9`，逐字节一致 |
| | `case_parity_check.py`（**无参数**） | 0 | 8 families accepted |
| | `kc_perf_gate.py --check` | 0 | 预算内（k-100 ×0.874、c-66 ×1.023） |
| 两包测试 | `pytest packages/suspension_contracts/tests packages/suspension_kernel/tests -q` | 0 | **79 passed**（单独一次调用，rootdir 未漂移） |
| 架构测试 | `pytest packages/suspension_multibody/tests/architecture -q` | 0 | **147 passed** |
| 静态 | `ruff check .` | 0 | All checks passed |
| | `ty check .` | 0 | All checks passed |

`case_parity_check.py` 一律无参数调用：实测其 `:1142-1161` 只接受 `--family`/`--allow-partial`/`--record`，
写 `--check` 必然失败（`EPIC.md:234` 审核阻断项 5）。未使用不带 `--actual-dir` 的
`kc_parity_check.py` 自比较（`EPIC.md:235`）。

---

## 3. 四类交付证据（本人重跑，未采信子任务自报）

| 类 | 命令 | 退出码 | 产物 |
|---|---|---|---|
| `simulate` 端到端 | `pytest tests/api/test_simulate.py -q` | 0 | 18 passed |
| 总线双向读写 | `pytest tests/api/test_signal_bus.py -q` | 0 | 13 passed |
| ABS 实际反馈闭环 | `pytest tests/cases/test_abs_closed_loop.py -q` | 0 | 6 passed，含同次运行三段数值 |
| FMU 独立校验 | 仓库外 `ctypes`+`zipfile` 脚本 | 0 | 见下 |

**FMU 仓库外校验**（`raw/fmu_validation.md` 记录路径、命令与原文输出）：不 import 本仓库任何
模块，只用 stdlib。实跑：`fmi2GetVersion()` 返回 `2.0`；**轨迹断言**——把输入 `0.0→0.9`
使纵向滑移时程逐样本最大差 **2.188280**，同运行内滑移极差 **3.933832**；写输出被拒
`True`；`EXIT=0`。变量 17 个（2 input / 15 output），valueReference 0..16 稠密、两集合不相交。

---

## 4. 集成验证（合并后由本人跑）

- 修复 A 与修复 B 合并后：`pytest tests/api tests/adams/test_full_vehicle_correlation.py tests/subsystems tests/architecture -q`
  → **427 passed**（EXIT=0）。
- 全量合并后（**当时快照**，`artifacts/` 尚未重建）：**1700 passed / 47 skipped / 1 xfailed**（EXIT=0）。
  终态见第 6 节。
- 内核重建后镜像新鲜：`build_axle_native.py` → `suspension_kernel.dll` 2251625 B。

---

## 5. 结论（2026-10-02 独立复核后更正）

**此前本节的「9/10 满足、Epic 可结项」结论已作废。** 独立复核（code-reviewer `8ab15196`）
判定该结论偏宽，处置裁决（`40d78977`）裁定**不能结项**。主代理逐条独立实证后确认成立，
逐条处置见 `review_findings.md`。

更正后的实测结论（**当日读数，已被第 6 节取代**）：

| Done-When | 结论 | 依据 |
|---|---|---|
| (a) 力矩内建 | 修复后待并入复验 | 复核指出旧 helper 是无条件调用（AST 唯一调用点 `:342`，无 enclosing 分支）；主代理已改为 `none` 才调的条件调用，opt-in 路径不再读 `front_brake_bias`（新增 3 条测试，还原旧实现即失败），默认路径 sentinel 逐字节一致、8 cases bit-identical |
| (b) 转向通道 | 满足 | 46 passed；契约反转已登记 |
| (c) 通用运动学 | 满足 | 37 passed |
| (d) 广义静平衡 | 满足（G4 动态通道子项见下） | 静平衡 4 项实证；`normal_load_axle_{placement}` 只在源码中确认存在，**未在验收中给出该子项的实跑断言** |
| (e) ARB 独立 | 满足 | 30 passed；ARB 路径 0 命中 |
| (f) 轮端统一 | **已补齐证据** | 两侧经正常文档加载打开同一份 wheel 文档（同解析路径、同 sha256 `7510feade8…4d7f`），probe 体名到达两侧产物；此前「唯一实现文件」的论证**不足以支撑判据** |
| (g) API | 满足 | 18 passed；三门禁绿 |
| (h) 总线与闭环 | **未达成** | 总线写入实测不改轨迹：`variable_damping_L` 直接 `BusError`（`elements` 被当映射，实际是列表）；`motor_torque_FL` 写入成功但状态轨迹差 `0.000000e+00`。闭环侧误差随目标单调收缩（1.128e-02→6.417e-03→1.482e-03），但**未断言固定目标下随时间收敛** |
| (i) 零回归 | **未达成** | skip 1→47（`artifacts/` 被误删，本机无 Adams 不可恢复） |
| (j) ABI | 满足 | `= 17/32/1`；单一真源门绿 |

**结论：Done-When 实测满足 (b)(c)(e)(g)(j)；(a) 已修待复验；(d) 有一条子项缺证据；
(f) 证据已补齐；(h)(i) 未达成。Epic 不能关闭。**

**已并入的真实缺陷修复**（提交 `7e9f2dd`）：FMU 构建在非 ASCII 仓库路径下的产物落盘；
转向输出按声明口径换算。**待修复**（裁决 `40d78977`）：p5-03 总线写路径、
p5-04 固定目标收敛断言、p5-05 FMU 输入时间因果性。

---

## 6. 终态结论（2026-10-03 收口 · **Epic 关闭**）

第 5 节列出的全部未达成项与待修复项**已逐项就地修复并复验**，判据未弱化、基线未重录。

| Done-When | 终态 | 依据 |
|---|---|---|
| (a) 力矩内建 | **满足** | `_build_wheel_torque_signals` 仅在 `torque_demand == "none"` 分支被调用（提交 `4c9b283`）；3 条新测试验证鉴别力（还原旧实现即失败）；默认路径 sentinel 逐字节一致、8 cases bit-identical |
| (b)(c)(e)(g)(j) | **满足** | 同第 5 节；终局复跑未变 |
| (d) 广义静平衡 | **满足** | 静平衡 4 项实证；动态通道子项由 `tests/metrics/test_placement_channels.py` 实跑覆盖——**7 passed**（2026-10-03 复跑），其中 `test_the_third_axle_of_a_six_wheel_run_is_a_real_placement_not_a_rounding` 断言纵向加速度下 `normal_load_axle_front` 下降、`normal_load_axle_rear` 上升、`load_transfer_front_minus_rear < 0` |
| (f) 轮端统一 | **满足** | 两侧经正常文档加载打开同一份 wheel 文档（同解析路径、同 sha256），probe 体名到达两侧装配产物（提交 `4c9b283`） |
| (h) 总线 | **满足** | `variable_damping_L` / `motor_torque_FL` 写入均成功**且改变同次仿真的状态轨迹**（提交 `bd8d84c`；`tests/api/test_signal_bus.py` 16 passed） |
| (h) 闭环 | **满足** | ABS 试验台固定 `target=0.30`，尾段 `[120:201]` 实测 ON `mean(e)=0.0180` / `max(e)=0.0180` / `ptp=2e-6`，OFF `mean(e)=0.2132`，ratio **0.0843**；四条预置线全部满足。独立复核实跑吻合。取证 `raw/p504_convergence.md` |
| (h) FMU | **满足** | 输入写入保留时钟前历史（自 `floor(elapsed/step)+1` 起），`tests/api/test_fmu_export.py` 17 passed；仓库外独立脚本轨迹断言（输入 `0.0→0.9` 使滑移时程最大差 **2.188280**） |
| (i) 零回归 | **满足** | 本机 Adams 2025.1.1 重建 `artifacts/`，skip 47 → **0**（提交 `a19c2a5`）；全量 **1755 passed, 1 xfailed, 0 skipped, 0 failed, 0 errors**（2026-10-03 收口后重跑，1443.82 s）；`tests/data/` 未写 |

**结论：Done-When (a)–(j) 十条全部满足。`p5-06 → DONE`（`completed_at = 2026-10-03`），
`Counter({'DONE': 28})`，Epic 关闭。**

**skip/xfail**：无新增。skipped **0**（基线 1，源于本机缺 Adams 的采集用例），xfailed **1** 与基线一致。
测试计数 +5 全部为有意新增（`test_abs_closed_loop.py` 6→7、`test_signal_bus.py` 13→16、
`test_fmu_export.py` 16→17），无不明的计数增长。

**ABI**：本 Epic 仅发生 p2-02 一次变更（`16/31/1 → 17/32/1`）。D2 终裁（`a8312da6`）判定闭环
**不需要**内核单步接口，`controller_output` 属结果契约扩展，故第二次 ABI 变更**未发生**。
