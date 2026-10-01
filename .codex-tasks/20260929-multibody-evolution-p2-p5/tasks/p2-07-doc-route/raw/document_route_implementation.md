# p2-07 判据 (b)：文档解析落地（改造内容与未改 ABI 的自证）

## 1. 逐文件改动（file:line 为改造后的实际行号）

| 文件 | 行 | 改动 |
|---|---|---|
| `packages/suspension_kernel/cpp/src/contract/contract_registry.cpp` | `:36-40` | `kElements[]` 加入 `"rotational_torque"`（10 → 11 条），排列重排为 4 列 |
| `packages/suspension_kernel/cpp/tests/contract_selftest.cpp` | `:132` | 新增 `check(contract_element_known("anti_roll_bar"), ...)`（相邻族本来漏测，本行补上） |
| | `:133-135` | 新增 `check(contract_element_known("rotational_torque"), ...)` |
| | `:144` | `contract_registry_size(1)` 断言 `10 → 11`，标签改为 `"eleven element types registered"` |
| `packages/suspension_contracts/src/suspension_contracts/contracts/multibody_model.schema.json` | `:139-145` | `$defs.element.properties.type.enum` 加入 `"rotational_torque"`（元素族的 enum，**不是** joint 的；joint 的 enum 在 `:80-85`，未动） |
| `packages/suspension_kernel/cpp/include/mb_cases/functions.hpp` | `:272-282` | 新增只读访问器 `ContractModel::rotational_torques()` |
| | `:422-431` | 新增存储 `std::vector<ElementBlock> rotational_torques_;` |
| `packages/suspension_kernel/cpp/src/cases/contract_model.cpp` | `:830-915` | `ContractModel::read` 的 `elements` 循环新增 `rotational_torque` 分支，位置在 `:919` 的 fall-through 之前 |
| `packages/suspension_kernel/cpp/src/abi/kernel_contract_run.cpp` | `:432-457` | `build_model` + `install_tire_mass` + `compute_effective_body_inertia` 之后，对本族块调用既有 `read_element_blocks` 追加 |
| 新增 `packages/suspension_multibody/tests/cases/test_rotational_torque_document.py` | 全文件 | 验收测试（7 项） |

### contract_model.cpp 分支的要点

- `body_a` / `body_b` 按名字查 `body_lookup`，未知体按名报错（与 bushing 分支同风格）。
- `parameters` 必须是对象；键名与 `element_reader.cpp:354-390` 的 `ELEMENT_ROTATIONAL_TORQUE` 分支逐一对齐：
  `stiffness`、`damping`、`axis_a`（3 个）、`reference_quaternion`（4 个，可选）、`max_torque`。
- **参考姿态的处理与通用读取器一致**：文档不写 `reference_quaternion` 时，块的 4 个槽保持 **0**，而不是补成单位四元数。
  `element_reader.cpp:375-380` 明确把「四个零」当作本族的「无参考姿态」（该族的律不读它）。
  若这里补成单位四元数，同一份文档经两条路径会得到不同的块——所以按通用读取器的语义写。
- 槽位：`ELEMENT_ROTATIONAL_TORQUE_STIFFNESS = 128`、`..._DAMPING = 129`、`..._AXIS_A = 130`（3 个连续）、
  `..._REFERENCE_QUATERNION = 133`（4 个连续）、`..._MAX_TORQUE = 137`；`ints` 与 `cached_parameters` 留零。
- 长度尺度：`stiffness` / `damping` / `max_torque` 都是力矩，乘 `length_scale_`（与 anti-roll 分支同）；
  `axis_a` 与 `reference_quaternion` 无量纲，不缩放。
- 解析期即按通用读取器的规则校验（轴有方向、增益与上限非负、有限），错误按名字报出，
  而不是等到读取器回一句笼统的 `invalid rotational torque parameters`。

### kernel_contract_run.cpp 追加点的要点

- 传 `element_curves` 为 `count * kElementCurveSlots`（`= 8`）个零项的真实数组：本族 `curve_slots = 0`，
  读取器不会读其中任何一项，但它在 `element_reader.cpp:34-37` 会拒绝「有块却没有曲线数组」。
- 错误经既有 `fail(error_buffer, error_capacity, 2, "model element: " + error)` 路径报出。
- **没有**把块放进 `AxleInput::elements` / `element_count`：这两个入口形式互斥，
  `build_model.cpp:41-58` 的 `only_when_no_blocks` 守卫会拒绝同时给出两者的文档。
  本行只把块直接交给 `read_element_blocks`，守卫看到的仍然只有逐族数组。

## 2. 未改 ABI 的自证

### 2.1 实测输出（**不是空**，如实说明）

```
$ git diff --stat -- packages/suspension_kernel/cpp/include/mb_config/version.hpp \
                  packages/suspension_multibody/src/suspension_multibody/kernel/native.py
 .../suspension_kernel/cpp/include/mb_config/version.hpp     | 13 +++++++++++--
 .../src/suspension_multibody/kernel/native.py               |  4 ++--
 2 files changed, 13 insertions(+), 4 deletions(-)
exit=0
```

这两行差异是 **p2-02 的未提交工作**（ABI 16/31 → 17/32 及其 Python 镜像），不是本行带来的。
该 diff 的正文自己写着来源：

```
$ git diff -- packages/suspension_kernel/cpp/include/mb_config/version.hpp
-inline constexpr int kAxleKernelAbiVersion = 16;
+inline constexpr int kAxleKernelAbiVersion = 17;   // 注释：「Bumped 16 -> 17 with 本行 rotational actuator (p2-02, 2026-10-01)」
-inline constexpr int kVehicleKernelAbiVersion = 31;
+inline constexpr int kVehicleKernelAbiVersion = 32;
```

所以「未改 ABI 版本常量」的自证不能靠「`git diff` 是否为空」——这两个文件在本工作区里本来就脏。
自证改用三条互相独立的证据：

1. **本行的写操作从未指向这两个路径。** 本行全部 `Edit` / `Write` 的目标只有
   `contract_registry.cpp`、`contract_model.cpp`、`kernel_contract_run.cpp`、
   `mb_cases/functions.hpp`、`contract_selftest.cpp`、`multibody_model.schema.json`、
   新测试与本次任务目录的 `raw/` / `PROGRESS.md`；`version.hpp` 与 `native.py` 一次都没有出现。
2. **mtime**：

   ```
   $ stat -c '%y %n' packages/suspension_kernel/cpp/include/mb_config/version.hpp \
                     packages/suspension_multibody/src/suspension_multibody/kernel/native.py
   2026-10-01 08:19:29.932422300 +0800  .../mb_config/version.hpp
   2026-10-01 08:19:29.968999500 +0800  .../kernel/native.py
   ```

   两个时刻只差 36 毫秒、都精确落在 `08:19:29`，是本行会话开始之前；
   本行自己的 C++ 改动落在 `09:02 - 09:04`。若本行碰过它们，mtime 会是 `09:0x`。
3. **内容**（用于后续复核，sha256）：

   ```
   5ec0e968fc78570bd3d02e7db7d518e3e573e6f14849d4b73d3d059edbaf2094  .../mb_config/version.hpp
   c447527f8e43e0f47b08f6800628121a01a90b106720792737744b4345f690db  .../kernel/native.py
   ```

   两个文件里仍是 p2-02 落的 `17 / 32`（`version.hpp`）与 `17 / 32 / 1`（`native.py`），
   没有被本行再改一次。

### 2.2 `AxleInput` 的字段面

写范围里没有 `mb_input/types.hpp`，该文件本行未触碰（它出现在 `git status` 里同样是 p2-02 的改动）。
本行只在 `mb_cases/functions.hpp` 的 `ContractModel` **私有成员**里加了一个
`std::vector<ElementBlock>`：那是文档读取器的自有存储，不是 ABI 结构。
`mb_input/types.hpp` 里的 `ElementBlock` 定义、`kElementBlockSize = 216`、
`kElementCurveSlots = 8` 与 `kElementLayouts` 全部逐字未变。

## 3. `git diff --stat`（本行全部改动）

```
 packages/suspension_contracts/.../multibody_model.schema.json          |  4 +-
 packages/suspension_kernel/cpp/include/mb_cases/functions.hpp          | 23 ++++++
 packages/suspension_kernel/cpp/src/abi/kernel_contract_run.cpp         | 27 +++++++
 packages/suspension_kernel/cpp/src/cases/contract_model.cpp            | 86 ++++++++++++++++++++++
 packages/suspension_kernel/cpp/src/contract/contract_registry.cpp      |  6 +-
 packages/suspension_kernel/cpp/tests/contract_selftest.cpp             |  7 +-
 6 files changed, 147 insertions(+), 6 deletions(-)
```

新增文件：`packages/suspension_multibody/tests/cases/test_rotational_torque_document.py`（未跟踪）。

**删除行只有两处**，都是把同一批名字重排/重写后的必然结果，没有语义删除：

1. `contract_registry.cpp` 的 3 行名字表（同一批名字 + 1 个新名字，重排列宽）；
2. `contract_selftest.cpp` 的 1 行计数断言（`== 10` 改成 `== 11`，同一断言的更新）。

`contract_model.cpp`、`kernel_contract_run.cpp`、`mb_cases/functions.hpp` 三个文件的 diff 中
**删除行数为 0**（`git diff -U0` 过滤 `^-` 为空），即既有族的读取路径逐字未动。

## 4. 重编与自测

```
$ uv run --no-sync python packages/suspension_kernel/scripts/build_suspension_kernel.py
E:\杂件\open-kinematics\packages\suspension_kernel\src\suspension_kernel\native\suspension_kernel.dll
exit=0

$ uv run --no-sync python packages/suspension_multibody/scripts/build_axle_native.py
E:\杂件\open-kinematics\packages\suspension_multibody\src\suspension_multibody\native\suspension_kernel.dll
exit=0

$ packages/suspension_kernel/build/Release/mb_contract_selftest.exe
mb_contract selftest: OK (45 checks)
exit=0
```

（计数从 44 涨到 45：`contract_element_known` 新增两条，计数断言一条。
经 `packages/suspension_kernel/tests/test_registry_consistency.py:87-89` 的既有 gate 复核，12 passed。）
