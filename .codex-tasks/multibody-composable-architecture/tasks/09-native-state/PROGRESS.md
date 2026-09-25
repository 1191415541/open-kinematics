# 09 原生状态接口与轮胎行为解耦

## Recovery

- 任务：`09 提取轮胎状态行为接口`。形态：single-full。依赖 08（DONE）。
- 状态见 `../../SUBTASKS.csv` 第 10 行；输入规格 `../../TASKS.md` 第 09 节。
- 主验收：`uv run --no-sync pytest packages/suspension_kernel/tests -q`；补充动态哈希、K/C 性能、严格分层门。

## 实测现状（进入本任务时）

TASKS 描述的缺陷**已由前序任务（D10 / D14 / K5 层）落地修复**。实测确认：

1. **求解器不再按模型枚举分支**。`solve_dynamic/kernel_integrator.cpp` 中已无 `VehicleTireModelKind` 比较，也没有 `pac2002`/`fiala`/`brush` 的具体分支；它读的是 `Tire` 的语义标志（`uses_exact_relaxation`、`has_state_return_mapping`、`state_slot_width`）：
   ```cpp
   bool has_fiala_tire = false;
   for (const Tire& tire : model.tires) {
       has_fiala_tire = has_fiala_tire || tire.uses_exact_relaxation;
   }
   ```
   —— 变量名保留是因为那段几何意义确实是「精确松弛模型」，但判断依据已是**能力**而非**身份**。

2. **描述表单一**：`mb_tire/model.hpp` 的 `kTireModelDescriptors` 一行一个模型，携带 `kind`/`name` 与 7 个语义标志（`uses_exact_relaxation`、`has_state_return_mapping`、`uses_pac2002_law`、`evaluates_at_wheel_center`、`projects_compression_on_spin`、`uses_pac2002_mode_table`）。头文件自己写明「新增模型是加一行，不是改积分器」。

3. **状态布局单点持有**：`mb_tire_state/tire_state.hpp` 的 `kTireSlotDescriptors` 12 个槽一行一个，含值/导数成员指针、最小宽度与打包信息；宽度由 `tire_block_width` 从 `Tire.state_slot_width` 缓存取，不回查模型。

所以本任务**不做无谓重写**——那会把已验证的机制推倒重来，违反「不改算法定义」。本任务补的是 TASKS 要求的**验收覆盖**：状态行为的结构性回归此前没有独立证据。

## 做了什么

### C++ 状态接口自检（`cpp/tests/tire_state_selftest.cpp`，新建，123 checks）

断言分两组，都是结构性事实——即断言**已编译的真实表**：

**描述表完备且自洽**
- 每个已注册模型有非空名字、kind 互不相同、且能被自己的 kind 反查命中（查询与表必须同源）；
- 未注册 kind 无描述符；
- 基宽恒为 2（线性瞬态对），比它窄的模型会丢掉瞬态状态；
- `uses_pac2002_mode_table` ⟺ `uses_pac2002_law`（双向蕴含，而非硬列名单）；
- `uses_exact_relaxation` 与 `has_state_return_mapping` 互斥——一个模型不能既替换 BDF2 行又被通用路径投影；
- 支持的 PAC2002 USE_MODE 集合非空、无重复，且每个「声明支持」的模式都被求解器实际使用的谓词 `pac2002_mode_supported_by_native` 接受。这条断言的是**声明与谓词一致**：列在能力文档里却被运行时拒绝，正是 08 要求消除的那类不一致。

**槽布局内部一致**
- 槽号唯一、有名字、最小宽度 ≥2、打包数 ≥1；
- 打包组（turn-slip 块）的首槽不晚于自身且在组内；未打包槽的该字段保持文档说明的 0；
- 每个槽的 value 与 rate 指针不同——存到同一个向量会每步自我覆盖。

### Python 侧接入（`kernel/tests/test_registry_consistency.py`，新增 2 项）

- 实跑 `mb_tire_state_selftest` 并要求 OK（断言编译产物，非源码）；
- **求解器不按模型枚举分支**：扫描 `solve_dynamic/`、`solve_static/` 的 `.cpp`，出现 `VehicleTireModelKind` 或 `model_kind ==` 即失败。这是 09 要建立的性质本身，写成门禁后新增模型不会悄悄把分支加回来。

## 过程中修正的一处自身错误

自检首轮 14 项失败，原因是我把 `packed_first_slot` 的语义写错：该字段在 `packing == 1` 时**被文档明确说明忽略**（值为 0），而我断言它等于 `slot`。按头文件的原文修正为「打包组内才做包含判断；未打包时断言该字段为其文档值 0」。修正后 123 项全过——修正的是我的断言，不是实现；没有为了让测试变绿而放宽任何真实要求。

## 验证记录（实测退出码）

| 命令 | 退出码 | 结果 |
|---|---|---|
| `pytest packages/suspension_kernel/tests -q` | 0 | 主验收 32 passed |
| `mb_tire_state_selftest`（新建） | 0 | 123 checks OK |
| `mb_cases_selftest` / `mb_contract_selftest` | 0 | 41 / 39 checks OK |
| `build_axle_native.py` | 0 | 编译通过，两份 dll 哈希一致（`5178d25a0d98…`） |
| `kc_parity_check.py --check` | 0 | 冻结快照容差内 |
| `case_parity_check.py` | 0 | 8 families 全 PASS |
| `dynamic_hash_sentinel.py --check` | 0 | combined sha256 仍为 `e7407656731e…`，逐位一致 |
| `kc_perf_gate.py --check` | 0 | k-100 ×0.672、c-66 ×0.653，预算内 |
| `check_module_layering.py --strict --final` | 0 | 0 环、0 反向边，未新增边 |
| `ruff check .` / `ty check .` | 0 | All checks passed |

## 未覆盖与保留

- **未改任何算法定义**：状态顺序、数值路径、收敛标准、物理律与 ABI 全部保持。`dynamic_hash` 逐位一致是最强证据。
- 本任务未新增 ABI 导出符号；自检走独立可执行文件，与 08 同一约定。
- TASKS 提到的「初始化 / 接受步状态 / 接触事件回归」由既有产品测试覆盖（`tests/axle_dynamics/test_contact.py`、`test_integrator.py`、`test_physics_consistency.py`、`test_native_tire_rig.py` 等），它们与动态哈希门共同构成数值侧证据；本次新增的是结构侧证据，两者互补而非替代。
- 未做头文件拆分：TASKS 要求的「拆分类型头但维持共享声明低层归属」在前序工作中已完成（`mb_tire_state/`、`mb_tire/`、`mb_model/` 已分层，`MODULES.md` 有记录），本任务核实而非重做。
