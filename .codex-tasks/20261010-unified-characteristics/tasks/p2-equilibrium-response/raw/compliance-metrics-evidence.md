# P2 步骤 4：Wheel/Ride/Roll 与载荷角度纯算子

对应 SPEC §50（步骤 4）、§41（实现规则）、设计 §6.7。
本文件只记实测。

## 0. 结论摘要

| 项 | 结果 |
|---|---|
| 新增实现 | `src/suspension_multibody/results/compliance.py` |
| 新增测试 | `tests/characteristics/test_compliance_metrics.py`（**42 passed**） |
| Wheel Rate | `1/h_left`、`1/h_right`（N/m）；**负值保留**，不取绝对值 |
| Ride Rate | 完整接地点响应取倒数；串联式**只在**已分离时单独调用 |
| Roll Rate | `ΔM/Δφ`（N*m/rad）→ N*m/deg **乘 `pi/180`**（数变小） |
| 载荷角度 | 走 `G_angle`，**不碰** rx/ry/rz 姿态行；含直接项 |
| 拒绝路径 | 一律不带数字（`values=()`），带具名理由 |
| 反向验证 | 自测 6 处破坏 + 复审 6 处破坏，**全部被逮住** |
| 门 | 快速集 **1773 passed / 1 xfailed**；characteristics 216；kernel+contracts 176；ruff/ty/三架构门 0；gate-numeric 三条 0 |

## 1. 纯算子分层：本层不向机理再提问

`compliance.py` 的每个函数只消费**已算出的响应**，自身不触发新求解。这不是洁癖：
率值是**倒数**，所以一个极小的位移响应会变成一个极大的率，这里是"看似合理的错数"
最容易产生的地方。把"求解"与"从解算指标"分开，才能让同一份响应同时服务多个指标
（设计 §427 的"相同模式跨指标复用"），也才能单独审计本层的算术。

## 2. Wheel Rate：必须是同名同相模式的倒数

```
h_left, h_right = 同相 heave 模式的左右垂向位移响应 (m/N)
wheel_rate = (1/h_left, 1/h_right)          N/m
```

设计 §41 明文禁止用端口 `Kzz`、单轮模式或 `1/C33` 替代。测试
`test_the_wheel_rate_is_not_the_mathematical_product_of_the_response` 用
`h=0.5` 断言结果必须是 2（而非 0.5 或 4），把"倒数 vs 载荷响应 vs 其平方"三种混淆
钉死。

**左右分别判定**：一侧有效而另一侧无效是可能的，合并成一个判决会要么掩盖真问题、
要么丢掉有效数。`test_each_side_of_a_wheel_rate_is_judged_separately` 断言
`wheel_rate(0.5, 0.0)` 被拒且理由里出现 `right`。

**负值保留**：`test_a_negatively_loading_wheel_keeps_its_sign` 断言
`wheel_rate(-0.5, 0.25) == (-2.0, 4.0)`。`abs()` 只用于阈值比较，从不作用于输出——
设计 §454 要求负切线连同稳定性诊断一起保留。

## 3. Ride Rate：轮胎只计一次

两条路径分开命名，不共用一个函数：

| 情形 | 函数 | 算术 |
|---|---|---|
| 已独立分离出悬架与轮胎 | `ride_rate_series(Ks, Kt)` | `Ks*Kt/(Ks+Kt)` |
| 完整模型接地点响应 | `ride_rate(h)` | `1/h`，**不再套串联式** |

`test_the_full_model_ride_rate_does_not_apply_the_series_formula_again` 是关键断言：
接地点响应 `1/50` 已含轮胎，答案是 50；**额外传入** `Ks=30, Kt=20` 后答案必须**不变**。
若有人"顺手"再套一次串联式，会得到更软的数且不可察觉。

**半套串联被拒**：只给悬架率不给轮胎率不是分解，`(None) != (None)` 的异或判定
拒绝它，理由是"半套串联不是串联律，会重复计入或漏掉轮胎"。

**不允许"关掉轮胎重算一个伪 Ks"**：设计 §456 明确禁止。本层通过"必须成对传入"
与"串联式是独立函数"两条结构性约束实现，而不是靠注释。

## 4. Roll Rate 与单位方向

```
roll_rate(ΔM, Δφ) = ΔM/Δφ     N*m/rad
roll_rate_per_degree(...) → 乘 pi/180     N*m/deg
```

方向容易搞反：每弧度的刚度换算成每**更小的**度时，**数值变小**。
`RAD_TO_DEG = pi/180` 按设计原文写法保留，便于逐字对照文档。

`roll_rate_per_degree` 是**独立函数而非开关**，且检查输入单位必须是 `N*m/rad`，
否则抛错——转换两次是静默的 57 倍误差，测试
`test_converting_an_already_converted_roll_rate_is_refused` 钉死这一点。

## 5. 载荷角度：用 G_angle，不用姿态行

`load_angle(G_angle, X, direct_term=D_angle P)` 只做 `gradient @ motion + direct`。
签名**不接受**任何位姿，所以"把 rx/ry/rz 行直接当 Toe/Camber"这个设计点名的错误
在本层**不可能发生**（不是靠注释禁止，是接口上没有那条路径）。

梯度与运动长度不符时抛 `CharacteristicsError`——长度不符是缺陷，不是小角度。

**拒绝时返回 `(0.0, status, reason)` 是有意为之**，与 `RateMetric` 的"拒绝不带数字"
看似冲突，实为不同情形，复审专门论证过：
- `RateMetric` 的规则针对**标量率**，因为 `1/inf == 0.0` 会静默产生一个**看似合法的零**
  （读作"无限软"，与真意相反）。所以率必须用空元组。
- `load_angle` 的 `0.0` 不可能被误读：调用方拿到三元组，第二个分量就是 status。
  要误用必须主动丢弃 status，那在任何约定下都无救。返回 `0.0` 保持返回类型稳定。

## 6. 交付后独立只读复审（D2）：发现并修复 6 个真实缺陷

| # | 缺陷 | 实测确认 | 修复 |
|---|---|---|---|
| **R1** | **`roll_mode` 的通道映射错**（步骤 3 引入）：`LoadMode.kind` 不是标签，它经 `LOAD_MODE_COMPONENTS` **决定扰动放在哪一行**。旧表 `roll_moment:5` 在 `[Fx,Fy,Fz,Mx,My,Mz]` 顺序下是 **`Mz`（偏航）**，不是侧倾 | 实测 `load_mode_matrix([heave, roll])` 的非零行落在索引 5，且 `amplitude_units="N*m"` 与力幅值自相矛盾 | 三处力矩重排为**绕各自轴**：`roll_moment:3`(Mx)、`camber_torque:4`(My, 自转轴)、`aligning_torque:5`(Mz, 转向轴)；`roll_mode` 的 kind 改回 `vertical_force`/`N`（反对称力对**产生**侧倾力矩，不**携带**它） |
| **R2** | 拒绝理由用字符串替换**伪造**响应单位：`units.replace('N/m','m/N')` 只在 `units=="N/m"` 时对 | `kN/m` → `km/N`、`N/mm` → `m/Nm` | 不再从率单位反推响应单位，理由改为陈述物理量 |
| **R3** | `roll_rate` 的零角分支报 `unidentifiable`，而 `wheel_rate`/`ride_rate` 同一物理情形报 `singular` | 确认 | 统一为 `singular`（分母趋零）；非有限情形保留 `unidentifiable` |
| **R4** | **`RATE_FLOOR` 是绝对阈值**，误拒合法但很软的机构 | `wheel_rate(1e-13, 1.0)` 被拒（真值 1e13 N/m） | 改为**相对**判据：`RATE_FLOOR * scale`，`scale` 取同一请求内最大的同类响应。这正是步骤 3 在响应层已确立的同一原则（`equilibrium.py` 注释点名 `1e-11` 为合法场景） |
| **R5** | 调用方 `extra` 里的 `metric_audit` 被**静默覆盖** | 确认 | 键冲突改为具名拒绝 |
| **R6** | `responses` 与 `modes` 之间无语义校验，键名与模式不符时静默接受 | 确认 | 记录为**设计取舍**（列标签由调用方负责，长度由下游校验），不加固——见 §8 缺口 |

**R1 最严重**：它会让侧倾指标求解**偏航**系统，而返回值格式完好。
它之所以逃过自测，是因为旧测试只断言 `kind == "roll_moment"`、从不检查 `component`，
也从不把 `roll_mode()` 送进 `load_mode_matrix`——**断言了部署，没断言行为**。
新测试改为经真实矩阵构建器检查非零行落在 `Fz`、且三个力矩行全为零。

## 7. 反向验证汇总（12 处破坏，全部被逮住）

自测阶段（步骤 4 首版）：

| # | 破坏 | 结果 |
|---|---|---|
| A | 率返回响应本身而非其倒数 | **FAIL**（5 条） |
| B | 完整模型响应上再套一次串联式 | **FAIL** |
| C | `pi/180` 方向反了 | **FAIL** |
| D | 零响应被求倒数而非拒绝 | **FAIL**（4 条） |
| E | 车轮两侧只判左侧 | **FAIL**（首轮**存活**，补测试后才逮住 → 见下） |
| F | 被拒指标仍列为已形成 | **FAIL** |

**E 首轮存活**，暴露真实覆盖缺口：非有限响应的检查没有任何断言覆盖。
`1/inf == 0.0` 会**静默产生一个有限零**，读作"无限软"，与真意相反。
补 `test_a_non_finite_wheel_response_is_refused_not_inverted` 与
`test_a_non_finite_roll_response_is_refused_not_inverted` 后逮住。

复审修复阶段（重放复审发现的原缺陷）：

| # | 破坏 | 结果 |
|---|---|---|
| G | `roll_mode` 声明成力矩 | **FAIL**（2 条） |
| H | 三个力矩 kind 按力轴顺序排 | **FAIL** |
| I | 恢复绝对阈值 | **FAIL**（2 条） |
| J | 理由伪造单位 | **FAIL** |
| K | 零角状态与零位移状态不一致 | **FAIL** |
| L | 覆盖调用方 `extra` 键 | **FAIL** |

全部破坏后均已从备份恢复并复核无残留。

## 8. 本步命令与退出码

| 命令 | 退出码 | 说明 |
|---|---|---|
| `pytest .../test_compliance_metrics.py` | 0 | **42 passed** |
| `pytest .../characteristics` | 0 | **216 passed** |
| 快速集 | 0 | **1773 passed, 1 xfailed**（+42 = 本步新增；xfail 未增长） |
| kernel+contracts | 0 | **176 passed** |
| `ruff check .` / `ty check .` | 0 / 0 | All checks passed |
| 三架构门 | 0 / 0 / 0 | 见 §0 |
| `dynamic_hash_sentinel --check` | 0 | 逐位一致 |
| `case_parity_check` | 0 | 8 族接受 |
| `kc_perf_gate` | 0 | ×1.059（c-66），在预算内 |
| `git diff --check` | 0 | 干净 |

## 9. 缺口与未做

- **`responses` 与 `modes` 无语义交叉校验**：本层校验长度，不校验"响应名是否对应所给模式"。
  列标签由调用方负责。这是一个明确的取舍，不是遗漏；若要加固，应在请求构造层做，
  而不是在纯算子层猜语义。
- **仍无生产调用方**：本步与步骤 3 相同，用构造输入验证算术与拒绝语义。
  真实 `constraint_jacobian` / 力律导数驱动的端到端指标属步骤 6。
- **Ride 的"完整接地点响应"数值来源未接**：本步定义了 `ride_rate(h)` 的语义与
  轮胎只计一次的约束，但 `h` 由调用方提供；从模型装配出接地点垂向响应属步骤 6。
- **`heave_mode`/`roll_mode` 的幅值基是 2 端口**：轮端数不做两轮限制是设计 §429 的要求，
  但本步的便利构造器是两轮的。多轮端/整车通道的具名模式属步骤 6 与 P4a。
- **未做**：防倾杆的完整整车耦合（本步用 2 自由度解析模型验证交叉项）、
  预载非线性、固定/自由车身差异、车身/轴间耦合——这些属步骤 6 的独立 oracle。
