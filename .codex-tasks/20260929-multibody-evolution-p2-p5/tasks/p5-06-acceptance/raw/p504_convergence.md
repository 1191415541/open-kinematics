# p5-04：收敛判据的最终口径与实测（固定目标）

裁决 003e00a0 判「现有数字不通过」，并要求**不接受仅以 ON 误差小于 OFF 代替收敛**。
本文件记录把「被测量收敛到目标」做成可判定断言的过程与读数。

## 1. 载体的两个真缺陷（都必须先修，否则任何收敛断言都不可达）

### 1.1 轮胎接触帧挂在自转的车轮上

`tire_frame_body()`（`cpp/src/model/kernel_model_accessors.cpp:33-35`）在 `tire.frame_body < 0`
时**回退到 `tire.body`**，而 `forward` 轴（滑移的测量方向）与 `rolling_radius` 都在这个帧里：

```
cpp/src/tire/assemble.cpp:162-164   forward = rotate(state.q[frame_body], t.forward_axis); forward.z=0
cpp/src/tire/assemble.cpp:201-232   loaded_radius / delta 用 frame_body 的姿态
```

车轮一旦自转，`forward` 随之翻滚，滑移率每半圈变号。实测（旧夹具，轮心 z 恒 0.29、法向力恒
3600 N）：`vx` 在 2.0/1.1/0.6/0.85 之间跳变、`sx` 在 ±1.9e-02 反复变号 —— 与 2a3ef3ff 的
复核结论一致。轴族此前**未**声明 `frame_body`，而整车族在
`preparation/vehicle_dynamic.py:1162-1209` 早就设了它。

修法：试验台的轮胎声明 `frame_body="carrier"`（不旋转的载体）。

### 1.2 开环滑移必须**越过**设定值，否则 ON 与 OFF 逐位相同

控制律（`cpp/src/element/anti_roll.cpp:243-251`）：

```
authority = clamp(1 + gain*(target - |slip|), 0, 1)
demand    = driver_demand * authority
```

`authority` 在 `|slip| <= target` 时**恒等于 1**，所以控制器只能**削减**制动力矩，不能放大。
（原文：`cpp/src/element/anti_roll.cpp:243-250` 的 `authority`、`:251` 的 `demand = demand * authority`；
`bound_slip` 定义在 `:160`，驱动列在 `:207` 被替换前记入 `driver_demand`。）
开环平衡点若落在 target 以下，`authority ≡ 1`，`demand ≡ driver`，两次运行**逐字节相同**。

实测（`cslip=1000`、`relax=0.25`、τ=200 N·m）：OFF `mean|sx|=0.3180` 与 ON `mean|sx|=0.3180`
完全一致（`np.array_equal == True`）；`|sx|` 在 tau=200 时恰好 0.3180，tau=150 时 0.3132 ——
即 `|sx| = τ/(4·r_l·cslip)`，`4` 来自 `USE_MODE=2` 下**力读瞬时滑移**而**控制器读松弛态**
（见 §3）。

## 2. 预置验收线（003e00a0 原文）

| 项 | 要求 |
|---|---|
| target | 固定 `0.30` |
| 尾段 | 固定索引 `[120:201]` |
| 模型/输入/时钟 | ON 与 OFF 完全相同，仅切换控制器 |
| `e` | `abs(abs(sx) - 0.30)`，`sx` 读 **`tire_output[:,0,10]`（无量纲滑移率）**，不是第 7 列（m/s 滑移速度） |
| ON 尾段 | `max(e) <= 0.05` 且 `mean(e) <= 0.03` |
| OFF 尾段 | `mean(e) >= 0.10` |
| 比例 | ON `mean(e) <= 0.8 * OFF mean(e)` |
| 保留 | 需求调制、非零执行器力矩、ON/OFF 状态不同的三段链断言 |

## 3. 为什么 τ 有窗口、以及 final 参数的由来

- 轮系回落时间常数 ≈ `I·V/(r_l²·cslip)`；`relax` 决定松弛态追赶运动学值的时间常数
  `≈ relax/V`（`cpp/src/tire/fiala/relaxation.cpp:135-137`：`rate = rolling_speed/relax_x`）。
- 试验台要在一个 0.2 s 工况内**安定**，两者都必须远小于 0.2 s。
- 实测扫描（`cslip=1000`、`I`、`relax` 网格）确定：`I=0.02`、`relax=1.0` 时两个时间常数分别为
  ~13 ms 与 ~100 ms，尾段 `[120:201]` 已在稳态。

最终参数（均为既有声明字段，未新增旋钮/未改内核）：

| 参数 | 值 | 依据 |
|---|---|---|
| `frame_body` | `"carrier"` | §1.1 |
| `wheel` 自转 | `V/(R - compression)` | 滚动半径口径 |
| `SPIN_INERTIA` | 0.02 kg·m² | 沉降时间常数 |
| `CARRIER_MASS` | 1500 kg | 现实载体质量；实测 1500–1e6 结果不变 |
| `relax` (纵向) | 1.0 m | §3 |
| 制动元 `stiffness` | 200 N·m | 置于饱和上限之内 |
| 制动元 `max_torque` | 400 N·m | 实测 400 与 1e9 结果不变 |
| `controller_gain` | 30 | §4 |
| `target_slip` | 0.30 | 预置线 |
| Fiala `cslip` | 1000 | 默认值，未覆盖 |

## 4. 实测（固定目标收敛）

命令：`pytest packages/suspension_multibody/tests/cases/test_abs_closed_loop.py -q`
结果：**7 passed**。

尾段 `[120:201]` 读数：

| 运行 | `mean|sx|` | `ptp` | `min` | `max` | `mean(e)` | `max(e)` |
|---|---|---|---|---|---|---|
| OFF | 0.5132 | 0.14022 | 0.4339 | 0.5741 | **0.2132** | 0.2741 |
| ON gain=30 | 0.3180 | **0.000002** | 0.3180 | 0.3180 | **0.0180** | **0.0180** |

对预置线逐项：

- ON `max(e) = 0.0180 <= 0.05` ✔
- ON `mean(e) = 0.0180 <= 0.03` ✔
- OFF `mean(e) = 0.2132 >= 0.10` ✔
- `ratio = 0.0180 / 0.2132 = 0.0843 <= 0.8` ✔
- ON 尾段 `ptp = 2e-6`（§10 独立复核实测值；四位小数显示为 0.00000）：不是「平均落在带内」，
  而是**已安定**（决策要点）。断言用的门槛是 `ptp < 1e-3`。

增益扫描（同一 τ=200、同一 OFF）：

| gain | ON `mean(e)` | `max(e)` | ratio | 预置线 |
|---|---|---|---|---|
| 5 | 0.0848 | 0.0874 | 0.396 | 未过（max > 0.05） |
| 10 | 0.0492 | 0.0493 | 0.230 | 未过（mean > 0.03） |
| 20 | 0.0263 | 0.0263 | 0.123 | 过 |
| **30** | **0.0180** | **0.0180** | **0.084** | **过（采用）** |
| 60 | 0.0092 | 0.0092 | 0.043 | 过 |
| 120 | 0.0047 | 0.0047 | 0.022 | 过 |

误差随增益单调下降且无稳定性上限，与解析式 `|sx|_eq = (1+g·T)/(g + K)`（`K = r_l·cslip/τ`）
一致 —— 取 gain=30 而非更大值，是为了让断言对参数扰动留有余量而不是贴着极限跑。

固定目标之外，设定值扫描（同一次运行口径，`SETTLED` 窗）：

| target | ON 尾段 `mean|sx|` | `mean(e)` |
|---|---|---|
| 0.15 | 0.1749 | 0.0249 |
| 0.20 | 0.2226 | 0.0226 |
| 0.25 | 0.2703 | 0.0203 |
| 0.30 | 0.3180 | 0.0180 |
| 0.35 | 0.3657 | 0.0157 |

同一 driver 信号下两目标（0.15 / 0.30）的安定 `demand` 分别为 0.2536 / 0.4611（差 0.2075），
证明调制项在**做**事而不是把 driver 原样透传。

## 5. 断言清单（`tests/cases/test_abs_closed_loop.py`，7 条）

1. `test_the_run_carries_all_three_segments` —— 一次运行三段可读（状态/控制/执行器）。
2. `test_the_demand_follows_the_state_rather_than_the_clock` —— 安定段 driver `ptp==0`、
   `demand` 落在 `0 < demand < driver`（控制器在削减）。
3. `test_the_actuator_moves_the_state_in_the_same_run` —— ON/OFF 状态差 `> 0.05`。
4. `test_the_measured_slip_converges_to_the_target` —— **预置线本体**：ON `mean(e)<=0.03`、
   `max(e)<=0.05`、OFF `mean(e)>=0.10`、`ratio<=0.8`，且 ON 尾段 `ptp < 1e-3`（已安定）。
5. `test_moving_the_setpoint_moves_the_measurement` —— 5 点目标单调跟随且两端差 `> 0.1`，
   各点尾段 `ptp < 1e-3`，`demand` 随 target 变化而 driver 不变。
6. `test_a_controller_that_reaches_the_blob_recorded_its_own_end` —— code-10 两端逐样本
   等大反向 `atol=0`、幅值 `0 < max <= cap`。
7. `test_the_control_ledger_is_absent_unless_it_is_asked_for` —— 开关关闭时无
   `controller_output` 块。

## 6. 鉴别力（判据不得弱化）

把新文件临时改写为四种弱化变体，放在 `tests/cases/` 内实跑（跑完删除，未留残留）：

| 变体 | 结果 |
|---|---|
| 删除 `frame_body`（滑移随轮翻滚） | **3 failed**, 4 passed |
| `controller_enabled` 恒 False（ON≡OFF） | **4 failed**, 3 passed |
| `stiffness = cap = 5000`（远超轮胎抓地） | **7 failed** |
| `relax` 回 0.25（松弛态追不上） | **4 failed**, 3 passed（§10 复核更正：原记 7 failed 系误抄） |

四种都失败，说明断言真的在测物理；若把新判据换成「ON 误差小于 OFF」这种弱形式，上述
第 2、3、4 条都不会失败 —— 这正是 003e00a0 拒收的形态。

## 7. 撤销了此前的错判（如实登记）

- 旧文件第 2 条断言 `demand` 在安定段 `ptp > 1e-9`（「仍在变」）。本 rig 收敛后
  `demand` 的安定段 `ptp` 是 **5.1e-05**（≈ 静止），旧断言的形态与「收敛」互斥。
  新文件把它改为「`0 < demand < driver`」并显式断言 `ptp < 1e-3`。
- 旧文件第 4 条只断言「目标移动、滑移同向跟随」，不构成收敛（003e00a0 明确拒收）。
  新文件第 4 条改为对 `target=0.30` 的**绝对误差**断言。
- 旧文件的「belt」叙述（`road_velocity` 当传送带速度）是错的：内核把 `road_velocity`
  组成 `{0,0,road_v}`（`cpp/src/tire/assemble.cpp:273`），它是**竖直方向**的路面速度。
  试验台的前进速度必须写在**车体状态**上。新文件据此重写，并把
  `road_velocity` 表置零。

## 8. 未声称

- 这不是整车 ABS 路试。试验台是**机构**夹具，证明的是「反馈通路闭合且被测量收敛到
  设定值」，即 EPIC.md:285 要求的那一条，不是车型级物理。
- 未改内核、未改 ABI（仍 17/32/1）、未新增依赖；`controller_gain`/`target_slip`/
  `frame_body`/`relax` 都是既有声明字段。
- 未重录任何基线；`tests/data/` 未写。

## 9. 门禁（本次改动只动测试文件）

- `pytest tests/cases/test_abs_closed_loop.py -q` → **7 passed**
- `pytest tests/cases -q` → **116 passed**
- `ruff check .` → All checks passed（先修掉一处 D401）
- `ty check .` → All checks passed
- `dynamic_hash_sentinel.py --check` → combined sha256
  `fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9`，逐字节一致

## 10. 独立复核（explorer `8d135895`，实跑，未采信本文件自报）

复核方自建脚本（`$PI_SCRATCH_DIR/p506review/recompute.py`，不复用测试 helper）独立复算：

```
[ON ] tail n=81  mean(e)=0.017964  max(e)=0.017965
[ON ] min|sx|=0.317963  max|sx|=0.317965  ptp(|sx|)=0.000002
[OFF] tail n=81  mean(e)=0.213156  max(e)=0.274147
[OFF] min|sx|=0.433927  max|sx|=0.574147  ptp(|sx|)=0.140221
ratio ON/OFF mean(e) = 0.084274
```

与本文件 §4 的 0.0180 / 0.2132 / 0.0843 逐项吻合（差异仅四舍五入）。**一处细化**：
ON 尾段 `ptp` 实测是 `2e-6`，不是严格的 `0`；本文件 §4 与测试注释里写的 `0.00000` 是
四位小数的显示值。断言用的是 `ptp < 1e-3`，两者都满足，但记录应写实测值。

复核方独立复现的四个弱化变体（均产生 `failed`，非收集错误）：

| 变体 | 复核结果 | 本文件 §6 声称 | 一致 |
|---|---|---|---|
| A 删 `frame_body` | 3 failed, 4 passed | 3 failed, 4 passed | 是 |
| B `controller_enabled=False` | 4 failed, 3 passed | 4 failed, 3 passed | 是 |
| C `stiffness=cap=5000` | 7 failed | 7 failed | 是 |
| D `relax` 回 0.25 | 4 failed, 3 passed | 7 failed | **否，已更正** |

变体 D 的失败条数以复核实测为准：**4 failed**（第 2、3、4、5 条），不是本文件 §6 先前
写的 7 failed —— 那是我把变体 C 的结果误抄到了 D 行。§6 表格据复核更正为 4 failed。
条数多少不影响结论（该变体确实失败），但记录必须准确。

三处机理引用经复核逐条核对源码原文，全部属实：

- `kernel_model_accessors.cpp:33-35` —— `return tire.frame_body >= 0 ? tire.frame_body : tire.body;`
- `tire/assemble.cpp:273` —— `const Vec3 road_velocity{0.0, 0.0, road_v};`；`forward` 取自
  `:47` 的 `frame_body`（`:162`）
- `element/anti_roll.cpp:243-251` —— `authority` 与 `demand = demand * authority` 原文一致

**复核方补充的准确口径**：`anti_roll.cpp` 的控制器开启条件是
`:209` 的 `actuator.controller_gain > 0.0 && actuator.target_slip >= 0.0`，**不是**直接判断
`controller_enabled`；后者决定的是 ledger 的记录路径。这解释了变体 B 为何是 4 条失败而非
全灭 —— 元素块的 `controller_enabled` 经 `element_reader.cpp:422-428` 在关闭时把
`controller_gain` 归零，故律确实关闭，但断言命中数与其余变体不同。
