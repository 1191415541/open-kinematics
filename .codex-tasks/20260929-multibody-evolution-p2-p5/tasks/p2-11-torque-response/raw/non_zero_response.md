# p2-11 证据：rotational_torque 非零响应专用夹具验收

> 全部为 2026-10-01 本机实跑/实读结果。来源：`code-reviewer` 裁决 `8e86187b`
> （p2-09 收口口径：非零力矩响应这条硬判据移交本行独立验收）。

## 为什么需要独立一行

`p2-09` 已把力矩元接进整车装配面，并**修正了反力体拓扑**（见 §4），但整车夹具上
code-10 的力矩幅值恒为 `0.0`。原因是该夹具的拓扑与求解器约束互相锁死：

| 条件 | 实测结果 |
|---|---|
| 给初速（`wheel_speeds`）以获得非零相对角速度 | `status 5: time integration failed at t=0.000000 s: Newton solve did not converge` |
| 加内部子步（1e-5 / 1e-4，`min_internal_step_size` 1e-7 / 1e-8，自适应） | 仍 `status 5` |
| 加 `_with_ride_springs`（夹具自身的补救手段）+ 子步 | 仍 `status 5` |
| `gravity=(0,0,-9810)` + `static_equilibrium=True`（收敛，轮胎确实承载 7654 N / 7514 N） | 收敛但**要求零初速**，故相对角速度为 0，命中内核「静止对不施力偶」分支，力矩仍为 `0.0` |

即「有载」与「非零初速」在该夹具上互斥。既有 `xfail`
（`packages/suspension_multibody/tests/vehicle/test_native_vehicle.py:1698-1721`）记录的是同一个
结构性问题（理想关节、无弹簧/衬套、轮胎未加载、单步不可细分），并明确说需要夹具重设计。

**因此本行用既有两体装置独立验收这条判据**——它是文档路由上的真实提交，不是元素级单测，
也**不是**用两体结果冒充整车夹具通过。

## 装置

复用 `tests/cases/test_rotational_torque_document.py` 的既有最小模型：

- 两个自由刚体、一个沿力偶轴的自旋 `RevoluteJoint`、无轮胎、无道路、无其它力；
- driven 端初始角速度 `SPIN = 3.0 rad/s`（远超 `kEps`），reaction 端为 0；
- 元素 `stiffness = max_torque = 1.0`，需求为单位需求（`demand_source = 0`）；
- 经 `simulation.run_request(compile_document_pair(...))` 提交，即生产文档路径。

## 判据与实测

新增用例 `test_the_couple_the_kernel_applied_is_non_zero`，读内核自身的 `element_wrench`
通道（`SUSPENSION_KERNEL_ELEMENT_WRENCH_OUTPUT=1`），type code `10`：

```
code10 rows: 42            # SAMPLES(21) × 2 端
sample0 pair moments:
[-0. -1. -0.]              # 范数 1.0 N·m
[ 0.  1.  0.]              # 范数 1.0 N·m
last pair norms: 0.0 0.0
```

| 断言 | 实测 |
|---|---|
| 对照组（无元素）code-10 行数 | **0**（"这些行属于该元素"是测量而非假设） |
| opt-in 行数 = `SAMPLES * 2` | **42** |
| 每个样本两端力矩**严格等大反向**（`atol=0`） | 通过（`[-0,-1,-0]` 与 `[0,1,0]`） |
| **首个有效样本非零**且范数 = `max_torque`（`rel=1e-12`） | **1.0 N·m**，逐位符合 |
| 末样本归零（相对速率会合后第三分支不施力偶） | **0.0**（`abs=1e-12`）——「力偶跟随状态」的另一半 |

末样本归零这条是必需的：只断言「非零」的检查对「一直施满力偶的错律」同样通过。

## 门禁（实跑）

| 命令 | 结果 |
|---|---|
| 本行 `validation_command` 的三份测试文件 | **30 passed** |
| `dynamic_hash_sentinel.py --check` | combined sha256 `fdfd5a6b…eebc9`，**逐字节一致** |

## 诚实登记

- 本行**不**声称整车 `vehicle_dynamic` 夹具的非零响应已达成：那个缺口已如实记在
  `tasks/p2-09-torque-wiring/raw/torque_wiring.md`。
- 本行**未**改任何生产代码，只新增一条测试断言（`packages/suspension_multibody/tests/cases/
  test_rotational_torque_document.py` 新增 `_raw` 辅助与 1 条用例）。
- 本行**未**改 ABI、**未**重录基线、**未**新增 skip/xfail。
