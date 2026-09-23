- 任务：轮胎质量归属轮胎与内核惯量耦合
- 形态：single-full（Epic 子任务）
- 进度：9/9 步骤 DONE
- 当前：契约字段与内核 `Tire` 字段已落（08a）；求解器已**显式耦合**轮胎惯量（08b）——`Model` 缓存每体有效质量/惯量，14 个消费点全部改读有效值。**零质量路径逐字节不变**，只有声明了轮胎质量的模型才受影响。
- 文件：`.codex-tasks/20260922-suspension-template-architecture/tasks/20260922-08-tire-mass/`
- 验证：契约 27、内核 21、架构 91、`tests/tire_mass` 4 passed；全量 multibody 974 passed／1 skipped／1 xfailed；`dynamic_hash_sentinel --check` 26 artifact 逐字节一致（未重录）；`kc_parity_check --check` 0；`case_parity_check` 8 families accepted；`--strict --final` 0；两包 `uv build` 0；`tests/data` 与 `layering_baseline.json` 无 diff；ruff/ty 全树通过。

## 08b：求解器耦合（2026-09-23，主代理实施）

上一轮三个子代理在此步分别耗尽预算或连接中断（零落盘），主代理接手实施。

**设计（已冻结并落地）**：新增「有效惯性」缓存，把「轮胎也拥有惯量」这一事实在**模型层**求和一次，热循环只读缓存：

- `Model` 新增 `body_effective_mass` / `body_effective_inertia_body`（与 `bodies` 同序）；空表表示「未计算」，访问器回退到该体自身的值。
- `compute_effective_body_inertia`：`m_eff = m_body + Σ tire.mass`，`I_eff = I_body + Σ (tire.inertia + tire.mass·((c·c)E − c⊗c))`。**某个体没有带质量的轮胎时，两个值与原始值逐位相等**（加零），这是零质量路径不变的根据。
- 三个 `build_model` 调用点（`kernel_contract_run.cpp`、`kernel_abi.cpp` 两处）在构建后立即调用它，保证任何入口下缓存都已填好。
- **14 个消费点**全部改读访问器：残余惯性力/力矩、newton 的质量块与惯量块与 `-j/m`、积分器质量矩阵、`mass_inverse_of_jt_mu` 与其方向导数、**重力**（`force/layout.cpp`）、陀螺项、动能、静态 `total_mass`。

**已知限制（有意，非漏做）**：当 `tire.mass != 0` 且轮胎中心**不在**其体原点时，`compute_effective_body_inertia` 返回失败并点名，调用点以状态码 2 与 `"tire inertia: "` 前缀报出。原因：残余把体原点当作质心（无臂项），偏心质量需要额外的平动/转动耦合项，那是**新物理**而非「换归属」，必须有独立验收，不能悄悄加。作者层实际发射的文档里 `center_local` 为 `[0,0,0]`，故当前全部路径都在支持范围内。

**质量守恒断言（先行，逐位精确）**：`tests/tire_mass/test_mass_conservation.py` 用二进制精确拆分（`2.0+20.0=22.0`、`0.5+0.25=0.75`、`0.5+0.5=1.0`）构造「同一份惯量、只换归属」的两跑，断言 `body_state` 与 `energy` **逐位相等**（`max|diff| = 0.0`）。**不使用容差**：容差恰好会掩盖本测试要抓的丢项/重复计项/结合顺序错误。

**耦合确实生效**：同模型「tire 声明 20 kg」vs「不声明」，`body_state` 差 21.7（非零），证明惯量进入了残余与雅可比，而不是只停在 `Tire` 字段上。

**三个数值门照旧通过、未重录任何基线**：既有文档都不声明轮胎质量，按设计零质量路径逐字节不变，故 `dynamic_hash_sentinel --check`（26 artifact 逐字节一致）、`kc_parity_check`、`case_parity_check` 全部无需重录。这是本步最重要的安全性证据。

## 实施分两半的记录

本子任务按「内核侧」与「求解器侧」分两半推进：08a 让字段到达 `Tire`，08b 让求解器真正读它。两半都已完成（见上）。下面保留 08a 的落点表作为出处，并补充 08b 的落点。

## TODO 1-3 的实现（2026-09-23，08a）


| 步骤 | 落点 | 关键事实 |
|---|---|---|
| 1 ABI 与质量现状实测 | `raw/abi_evidence_08a.md`、`raw/abi_symbols_08a.txt` | 七符号逐符号 ctypes 探针；版本常量 15/30/1/1；`Tire` 既有 32 字段偏移实测 |
| 2 契约字段 | `multibody_model.schema.json` 的 `$defs.tire` | 新增**可选** `mass`（`number`, `minimum: 0`, `finite: true`）与 `inertia`（3×3 `array of number`）；`required` 仍为 `[name, model, body]`，`additionalProperties: false` 保留。`multibody.py` 的 `_check` 新增 `finite` 关键字支持 |
| 3 内核字段与解析 | `cpp/include/mb_model/types.hpp` 的 `struct Tire` 末尾、`cpp/src/cases/contract_model.cpp`、`cpp/src/abi/kernel_contract_run.cpp` | `double mass{0.0}` 与 `Mat3 inertia{}` **append 在末尾**（既有偏移不变，`sizeof` 2096→2176）；解析在 `contract_model.cpp:1036-1059`，安装点 `kernel_contract_run.cpp:425`（紧随 `body_wrench_point_local` 的既有先例），定义在 `contract_model.cpp:1451` |

**关键设计取舍（违背它的后果已写进代码注释）**：**未给 `AxleInput`/`VehicleInput` 追加任何数组字段**——那属 ABI 冻结解除，SPEC 要求停止并单独裁决。改走既有先例：`ContractModel` 读文档 → 存自己的 `tire_mass_`/`tire_inertia_` → `kernel_contract_run.cpp` 在 `build_model` 之后装到 `built.tires`。实测 `AxleInput` 字段数仍 106、`VehicleInput` 仍 97。长度不匹配时 `install_tire_mass` 返回 `false` 并报错点名（`N masses and M inertias for K tires`），**不静默截断**。

## 独立复核（主代理实跑，非采信子任务自证）

| 命令 | 结果 |
|---|---|
| `pytest packages/suspension_contracts/tests -q` | 27 passed |
| `pytest packages/suspension_kernel/tests -q` | 21 passed |
| `pytest packages/suspension_multibody/tests/architecture -q` | 91 passed |
| `dynamic_hash_sentinel.py --check` | 26 artifact 逐字节一致（`e7407656…`，未重录） |
| `kc_parity_check.py --check` / `case_parity_check.py` | 退出 0 / 8 families accepted |
| `check_module_layering.py --strict --final` | 退出 0，cycles 0、新增头文件边 0 |

另核对：schema 侧 `required`/`additionalProperties` 未变、新增两字段为可选；`types.hpp` 既有字段声明未被改动，`mass`/`inertia` 仅在 `Tire` 末尾。

## 行为不变的独立实测

作者层真实发射的 `axle_dynamic` 模型文档（`road_pulse`），原样 vs 每个 tire 追加 `mass=12.5` + 对角惯量，各跑一次 `suspension_kernel_run`：`body_state`/`energy` 逐位相等，`diagnostics` 含 NaN 位整体相等。证据在 `raw/no_behavior_change_08a.{py,out}`。

## 偏离、未核实项与风险（如实登记）

1. **内核侧测试是编译期探针 + pytest 形式**（`tests/fixtures/tire_mass_probe.cpp` 由 pytest 现场编译并链接 `build/Release/libmb_*.a`）。原因：`Tire::mass` 不经产品 C ABI 暴露，纯 Python 无法观测。探针调用真实 `build_model` + `install_tire_mass`，不是复刻实现。副作用：首次运行多约 9 秒编译；`build` 目录不存在时该文件 skip。探针源码**未**登记进 `build.py` 的 `SOURCE_RELATIVES`（它是测试夹具、不属内核构建产物），`test_binding.py` 的 provenance 断言实测仍通过。
2. **附带发现（未改，不属本步范围）**：`suspension_contracts.multibody.validate_model()` 会拒绝作者层发射的模型文档——`blobs` 描述符带 `name` 字段而 `$defs.blob` 为 `additionalProperties: false`，即 `validate_model` 目前不在发射路径上。已记录，未处置。
3. **未做且不声称做过**：求解器残余/雅可比耦合轮胎惯量、质量守恒断言、动态门按 D4 登记、两包构建与隔离 wheel 复验——全部属 08b。

## 下一步

08b 从 `TODO.csv` 第 4 行开始：质量守恒断言**先行**，再做求解器显式耦合，最后按 D4 登记数值门变化。注意 SPEC 要求「质量守恒断言必须在任何数值差异评估与动态字节门重录之前跑通」。
