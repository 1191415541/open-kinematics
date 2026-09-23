# 简化制动幅值公式的证据与偏差登记（子任务 04 / TODO 7）

## 1. 基准

Adams 安装目录本机不可访问（`C:\MSC.Software` 仅剩 Licensing），唯一可用基准是仓库内冻结产物：

`artifacts/adams-fiala-handling/step_steer/adams_raw/handling_step_steer_dynamic.adm`

原文（行号为该文件内行号）：

| 行 | 声明 | 公式 |
| --- | --- | --- |
| 8846-8860 | `SFORCE/31` (rear right) | `2.0*2500.0*(1.0-0.6)*VARVAL(96)*1.0*0.1*0.4*130.0*STEP(VARVAL(284),-10.0D,1,10.0D,-1)` |
| 8861-8875 | `SFORCE/32` (front right) | `2.0*2500.0*IF(0.0:0,2.0-1.0,0.0)*0.6*VARVAL(96)*1.0*0.1*0.4*145.0*STEP(VARVAL(282),...)` |
| 8876-8890 | `SFORCE/33` (front left) | `2.0*2500.0*IF(0.0:0,1.0,0.0)*0.6*VARVAL(96)*1.0*0.1*0.4*145.0*STEP(VARVAL(281),...)` |
| 8891-8905 | `SFORCE/34` (rear left) | `2.0*2500.0*(1.0-0.6)*VARVAL(96)*1.0*0.1*0.4*130.0*STEP(VARVAL(283),-10.0D,1,10.0D,-1)` |

- `VARIABLE/96`（:11944-11945）= `0.0`，即 `testrig.vas_brake_demand` 的制动需求；
- `VARIABLE/281-284`（:12656-12694）是 `abgVDM::var1004` 的轮速符号变量，`SFORCE` 末项
  `STEP(<wheel speed>,-10,1,10,-1)` 用它决定力矩方向；
- `IF(0.0:0,1.0,0.0)` / `IF(0.0:0,2.0-1.0,0.0)` 在 `IF` 条件恒为 0 时取第二分支，即前左/前右系数各为 1.0。

## 2. Python 侧公式（`subsystems/brake.py::wheel_torque_amplitudes`）

```
T = 2 * piston_area * bias_share * demand * max_brake_value * brake_mu * effective_piston_radius
bias_share = front_brake_bias (front_*) | 1 - front_brake_bias (rear_*)
```

本模块按四个轮名逐一发出同一幅值（左右同值），与 `.adm` 的展开一致。

常数与 D10 参数子集的对应：

| `.adm` 常数 | 本模块参数 |
| --- | --- |
| `2500.0` | `piston_area` |
| `0.6` | `front_brake_bias` |
| `0.4` | `brake_mu` |
| `0.1` | `max_brake_value` |
| `145.0`（前）/ `130.0`（后） | `effective_piston_radius`（单一值，见偏差 ②） |
| `VARVAL(96)` | `demand`（`TimeSignal`） |
| `STEP(轮速,...)` | 不上 Python 侧，方向由内核按轴向转速符号决定（`cpp/src/element/drive_brake.cpp`） |

## 3. 算术核对（`demand = 1.0`，其余取 `.adm` 常数与单一 145.0）

前轴：

```
2 * 2500.0 * 0.6 * 1.0 * 0.1 * 0.4 * 145.0 = 17400.0
```

后轴：

```
2 * 2500.0 * (1 - 0.6) * 1.0 * 0.1 * 0.4 * 145.0 = 11600.0
```

前后分摊比例 `17400 / 11600 = 1.5 = 0.6 / 0.4`，即两个 bias 份额之和为 1。

实跑断言在 `tests/subsystems/test_brake_subsystem.py`：

- `test_the_amplitude_matches_the_adams_sforce_shape`：
  `front_left == front_right == 17400.0`、`rear_left == rear_right == 11600.0`，比例断言 1.5；
- `test_an_out_of_range_demand_names_the_signal`：`demand < 0` 或 `> 1` 抛 `ValueError` 且消息含 `brake_input`；
- `test_the_amplitude_is_a_nonnegative_magnitude_at_every_demand`：逐时刻非负，且半需求为半幅值。

## 4. 未核实项与偏差（须由主代理登记进 PROGRESS）

① **常数 `0.1` 的出处未核实**。它在 `.adm` 里是 `SFORCE/31-34` 的乘子，位置上是制动需求的缩放；
本机无法访问 Adams 安装目录，无法回到 `default_brakes.sub` / `sedan_brake_system.sub` 核对它的名字与数量级。
本模块按原文登记为 `max_brake_value` 并保留默认 `0.1`，在 docstring 明确写为**未核实常数**。

② **145.0 / 130.0 的单参数偏差**。D10 的参数子集只有一个 `effective_piston_radius`，而 `.adm` 前后轴各有一个
（前 145.0、后 130.0）。本实现**不新增第二个半径参数**（D10 参数子集固定），`effective_piston_radius` 默认取前轴的
145.0，测试钉住前轴值；后轴沿用 145.0 而非 130.0，故后轴幅值比 `.adm` 同参数下高 `145/130 - 1 ≈ 11.5%`。
这是**已登记的偏差**，不是被静默修正的项：若后续要逐参数对齐后轴，需要 D10 层面的裁决（新增后轴半径或恢复
`sedan_brake_system.sub` 的双半径口径），不在本步范围。
