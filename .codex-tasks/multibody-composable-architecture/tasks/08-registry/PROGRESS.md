# 08 内核能力与处理器注册收敛

## Recovery

- 任务：`08 收敛内核能力注册与实际分派`。形态：single-full。依赖 01（DONE）。
- 状态见 `../../SUBTASKS.csv` 第 9 行；输入规格 `../../TASKS.md` 第 08 节。
- 主验收：`uv run --no-sync pytest packages/suspension_kernel/tests/test_registry_consistency.py packages/suspension_contracts/tests -q`

## 实测确认的缺陷

`case family` 的知识原本分散在**三处**，且没有任何东西检查三者一致：

1. `mb_contract` 的 `kCaseFamilies`（8 个名字，含 `comparison`）；
2. `case_dispatch.cpp` 的 `contract_case_supported`——手写 `||` 链（7 个名字）；
3. `contract_expand_case` 的 `if` 阶梯（7 个分支，无兜底拒绝）。

后果：一个「协议已声明但本构建未实现」的家族只能靠两个列表**恰好不同**才被拒；而一个实现了却漏在列表外的家族会被报成不存在。`comparison` 正是第一种情况的活例子（它是 per-target gate，不是一次 solve）。

## 做了什么

### 单一描述表（`cpp/src/cases/case_dispatch.cpp`）

一行 `FamilyRow` 携带三件事：名字、是否在协议内、展开函数指针。于是：

- `contract_case_supported` 与 `contract_expand_case` 读**同一行**，查询与分派不可能不一致；
- `expander == nullptr` **恰好**对应「本构建未实现」，因为「能不能跑」与「谁跑」是同一份数据；
- 区分两种失败：名字不在协议内 → `unknown case family "x"`；协议定义但无处理器 → `defined by the contract but not implemented by this build`。把两者合并会告诉用户「拼写错误」和「功能缺失」是同一件事。

两个网格家族（`kc_quasi_static`、`vehicle_kc`）的展开函数不接 blob，用两个薄适配器统一签名。表的行类型刻意一致——类型不同的行无法被搜索，差异集中在适配器里可见，而不是散在分派逻辑中。

### 名字枚举（两侧）

- `contract_case_family_count(supported_only)` / `contract_case_family_name(index)`：让自检**枚举**表，而不是再抄一份列表。测试自己列一份就是表要消灭的第四份拷贝。
- `contract_registry_name(table, index)`：契约侧只允许枚举「纯名字表」（elements / tires / case families）；`kJoints` 带行数，仍走 `contract_joint_rows` 查询，避免调用方从错误的地方读计数。

### 一致性自检（`cpp/tests/case_registry_selftest.cpp`，新建，41 项）

断言的是**已编译的真实表**，不是源码文本：

- 每行都是协议名；supported ⊆ declared；
- supported 的家族必有处理器，known-unimplemented 的必被**报为已知**而非未知；
- 未知名字（`rally`、空串）既不在协议也不 supported；
- 协议列的每个名字都被 cases 表分类，且**两表成员数相等**——协议名有表未提及即失败；
- `comparison` 必须是「协议内 + 无展开器」。

CMake 新增 `mb_cases_selftest` 目标，仅链接既有模块（`mb_cases`/`mb_contract`/`mb_model`/`mb_input`/`mb_config`/`mb_numeric`），**未新增任何依赖边**。

### Python 侧（`kernel/tests/test_registry_consistency.py`，新建，9 项）

- 实跑两个 C++ 自检并要求 OK（断言编译后的表，而非源码）；
- 协议家族在契约表与分派表中都被提及，且只有 `comparison` 的 expander 为 null；
- 产品从内核**读取**能力（`suspension_kernel_capabilities`）而非自带一份副本——旧的硬抄范围表曾声称内核从不施加有效范围钳位，而内核每步都在调用它；
- joint 表：作者名 → 内核名 → 行数三者一致，且总和与内核注册表宣称的一致；
- ctypes 能真正加载库并调用 ABI 版本探测（文件存在 ≠ 可加载）。

## 未新增 ABI

ABI 只有 3 个导出符号（`..._contract_version`、`..._capabilities`、`..._run`）。本次**不新增导出符号**：新查询是模块内部 API，由独立自检可执行文件覆盖——这类验证按仓库既有约定「通过独立可执行文件而非产品 C ABI 进行，以免提升任何 ABI 版本」。改 ABI 属 `DESIGN` 要求先确认的事项，未触发。

## 验证记录（实测退出码）

| 命令 | 退出码 | 结果 |
|---|---|---|
| `pytest kernel/tests/test_registry_consistency.py contracts/tests -q` | 0 | 主验收 36 passed |
| `mb_cases_selftest`（新建可执行） | 0 | 41 checks OK |
| `mb_contract_selftest` | 0 | 39 checks OK |
| `pytest packages/suspension_kernel/tests -q` | 0 | 30 passed |
| `build_axle_native.py` | 0 | 编译通过；两份 dll 哈希一致（`758028e93acd…`） |
| `kc_parity_check.py --check` | 0 | 冻结快照容差内 |
| `case_parity_check.py` | 0 | 8 families 全 PASS |
| `dynamic_hash_sentinel.py --check` | 0 | combined sha256 仍为 `e7407656731e…`，逐位一致 |
| `check_module_layering.py --strict --final` | 0 | 0 环、0 反向边、0 legacy 模块，未新增边 |
| `ruff check .` / `ty check .` | 0 | All checks passed |

## 未覆盖与保留

- 只收敛了 **case family** 的注册与分派。joint / element / tire 三类的 C++ 表（`kJoints`、`kElements`、`kTires`）本已是单表，其 Python 映射由 `joints/table.py` 持有并已被一致性测试覆盖，无需再动。
- `comparison` 保持「已知但未实现」：它不是一次 solve，而是 per-target gate，`case_parity_check.py` 已在 family 级实现它。不为其造空展开器。
- 未改动任何数值算法、收敛标准或 ABI；`dynamic_hash` 逐位不变即证据。
