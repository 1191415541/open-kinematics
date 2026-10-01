# p2-05 证据 (c)：契约表按新机制生成

> 实测日期 2026-10-01。

## 1. case 文档的 role 与行数（四次实跑）

| `torque_demand` | 模型文档 `rotational_torque` 元素数 | case 文档 role 集合 | 旧表行数 W/B | 新需求表行数 W/B |
|---|---|---|---|---|
| `none` | **0** | `brake_torque, steering_rate, steering_target, wheel_torque` | 4 / 4 | 0 / 0 |
| `brake` | 4 | `brake_pressure, steering_rate, steering_target` | 0 / 0 | 0 / 4 |
| `drive` | 2 | `steering_rate, steering_target, throttle_demand` | 0 / 0 | 2 / 0 |
| `both` | 6 | `brake_pressure, steering_rate, steering_target, throttle_demand` | 0 / 0 | 2 / 4 |

口径：**同一轮端不得同时出现两种单位制**（`brake_torque` 与 `brake_pressure` 互斥、
`wheel_torque` 与 `throttle_demand` 互斥）。这是内核 `vehicle_dynamic.cpp` 的
「同通道混用按名拒绝」在 Python 侧的同一条规则：preparation 层根本不为 opt-in 的轮端生成旧表，
内核再对文档里同时出现两制的写法按名拒绝（负例实测见
`tasks/p2-10-demand-plumbing/raw/demand_plumbing.md` §5）。

## 2. 既有契约测试

`tests/cases/test_vehicle_dynamic_contract.py` **未改**：默认路径的 role 集合与表头逐项不变，
该文件现有断言继续成立。实测（`packages/suspension_multibody/tests/cases` 全绿，
见 §3 的 383 passed）。

**未删除或放宽任何既有断言。**

## 3. 本行验证的是「隔离」，不是新的表格式

opt-in 的三档元素数与 role 已由 `packages/suspension_multibody/tests/subsystems/
test_torque_element_wiring.py`（p2-09 交付）覆盖；本行验证的是**调用隔离**与
**字段数据流隔离**，两者以 AST 加运行时计数取证（见 `retirement_grep.md`），
故未新增测试文件、未改既有断言。

实测命令与结果：

```
$ uv run --no-sync pytest packages/suspension_multibody/tests/cases \
    packages/suspension_multibody/tests/vehicle \
    packages/suspension_multibody/tests/subsystems -q -p no:cacheprovider
383 passed, 1 xfailed in 81.31s
```
