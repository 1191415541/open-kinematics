# p2-07 判据 (c)：两次运行对照（该族经文档路由被实际求值）

## 1. 两次运行的构造

同一个模型文档，唯一差别是 `elements` 里有没有那条 `rotational_torque`：

- 两体 `reaction` / `driven`，各 1 kg、惯量 `diag(0.01, 0.01, 0.01) kg*m^2`，**都是自由体**；
- 一个绕 `y` 的**转动副**把它们连起来（模型不是无约束的漂浮体，但仍只有这一个约束）；
- 初始状态：`reaction` 静止，`driven` 绕 `y` 转 `3.0 rad/s`；
- 元素（仅含族的那次）：`body_a = reaction`（反力端）、`body_b = driven`（受力端）、
  `axis_a = [0, 1, 0]`、`stiffness = 1.0`、`damping = 0.0`、`max_torque = 1.0`；
- case：`axle_dynamic`，`0 → 0.2 s`，21 个采样点，`rho_inf = 1.0`，
  `initialization_mode = provided_consistent_state`（初态是文档给的，不做静态配平），
  模型不带轮胎、不带路面，case 也不带任何 body_wrench / 道路 / 轮端力矩表——
  **除这个元素之外，没有任何东西能推动这对体**。

两次都走 `suspension_multibody.kernel.run_contract`（生产动态路径
`simulation/backend.py:24` 提交的就是它），提交前都经 `suspension_contracts.validate_model`。

测试文件：`packages/suspension_multibody/tests/cases/test_rotational_torque_document.py`；
下面的数字由 `raw/document_route_two_run.py` 打印——它按路径加载那份测试并直接调用它的
`_run` / `_omega_y`，所以打印的就是测试断言的那个读数，不存在两份会各自漂移的 fixture。

```
$ uv run --no-sync python .codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p2-07-doc-route/raw/document_route_two_run.py
with    driven   omega_y: [3.0, 1.9999999999999993, 1.499999999999999, ... 1.499999999999999]
without driven   omega_y: [3.0, 3.0, ... 3.0]
with    reaction omega_y: [0.0, 0.9999999999999998, 1.5000000000000002, ... 1.5000000000000002]
without reaction omega_y: [0.0, 0.0, ... 0.0]
driven   final: with=1.499999999999999 without=3.0 difference=1.500000000000001
reaction final: with=1.5000000000000002 without=0.0 difference=1.5000000000000002
tolerance: 0.5
omitted reference pose is byte-identical to the explicit identity: True
exit=0
```

测试本身：

```
$ uv run --no-sync pytest packages/suspension_multibody/tests/cases/test_rotational_torque_document.py -q -p no:cacheprovider
7 passed in 0.28s
exit=0
```

## 2. 实测数字

两次运行的 `status` 都是 `success`，`body_state` 形状 `(21, 2, 19)`，`np.isfinite(...).all()` 为 `True`。
下表是 `body_state[:, body, 10:13]` 的 `y` 分量（`kStatePerBody = 19`，omega 起始偏移 10）。

| 量 | 含元素 | 不含元素（对照） | 差 |
|---|---|---|---|
| 受力体 `driven` 末态 `omega_y` (rad/s) | **1.499999999999999** | **3.0** | **1.500000000000001** |
| 反力体 `reaction` 末态 `omega_y` (rad/s) | **1.5000000000000002** | **0.0** | **1.5000000000000002** |
| 受力体首态 `omega_y` | 3.0 | 3.0 | 0 |
| 反力体首态 `omega_y` | 0.0 | 0.0 | 0 |

逐样本的 `omega_y`（含元素）：

```
driven  : [3.0, 1.9999999999999993, 1.499999999999999, 1.499999999999999, ... 1.499999999999999]
reaction: [0.0, 0.9999999999999998, 1.5000000000000002, 1.5000000000000002, ... 1.5000000000000002]
```

逐样本的 `omega_y`（不含元素）：

```
driven  : [3.0, 3.0, 3.0, ... 3.0]     （21 个样本全为 3.0）
reaction: [0.0, 0.0, 0.0, ... 0.0]     （21 个样本全为 0.0）
```

## 3. 容差取值与理由

测试里的 `TOLERANCE = 0.5 rad/s`（`test_rotational_torque_document.py:79`），断言形式是
`abs(含族末态 − 移除族末态) > TOLERANCE`。

取值理由（不是凑出来的圆整数，是由物理定出来的）：

- 元素是纯力偶：`+tau` 加在受力体、`−tau` 加在反力体，两个惯量相等（各 `0.01 kg*m^2`），
  所以两端各以 `tau / I = 1 / 0.01 = 100 rad/s^2` 反向加速；
- 初始相对角速度 3 rad/s，两端在 `3 / 200 = 0.015 s` 相遇于 `1.5 rad/s`；
  此后相对角速度为 0，落到该族符号律的第三条分支（`|rate| <= kEps` 不施加力偶），
  于是**停在那儿**——这正是实测末态 `1.499999999999999` / `1.5000000000000002`。

因此：

- 真实效应 ≈ `1.5 rad/s`，容差 `0.5` 是它的三分之一，**效应不足三分之一就会被判失败**；
- 而「解析了却没求值」的差恰好是 `0`，「写错槽位导致力偶偏小」的差也会落进这个区间；
- 对照地，两次**完全相同**的运行之间只会有求解器步长舍入的 ~1e-15 量级差异，
  容差比它高约 15 个数量级，不会把「同一族被调用两次」误报成漂移；
- 再收紧就是在断言求解器选的步长而不是断言这个元素，所以取 0.5 而不是 `pytest.approx` 的默认相对容差。

方向也一并断言（`test_the_couple_changes_the_driven_bodys_final_rate`）：
`abs(含族) < abs(不含族)`——力偶是**抵抗**相对转速的，受力体必须被减速。
只断言幅度会让一个符号写反的实现蒙混过关。

## 4. 反力体响应

`test_the_couple_meets_in_the_middle`：

- 对照里反力体全程 `0.0`，含族运行里变成 `1.5000000000000002`；
- 且 `reaction[-1] == pytest.approx(driven[-1], abs=1e-9)`——两端**相等**。

等惯量 + 等值反向力偶 ⇒ 两端在中间相遇。这条等式的判别力在于：
只对一端施力的实现会把反力体留在 0，而「两端都动了一点」的实现在这里也过不去。

## 5. 参考姿态的两种写法必须等价

`test_the_omitted_reference_pose_means_the_same_thing_on_both_routes`：
把 `reference_quaternion` 从文档里省掉、与显式写 `[1,0,0,0]`，两次运行
`np.array_equal` 为 `True`。

理由是该族块读取器的语义：`element_reader.cpp:375-380` 把「四个槽为 0」当作本族的
「无参考姿态」（该族的律不读它），所以文档读取器也必须写 0 而不是补单位四元数；
否则同一份文档经 C ABI 路径与文档路径会得到不同的块。这条断言正是钉这一点。

## 6. 这条证据能说明什么、不能说明什么

**能说明**：文档路由不再按族名拒绝；解析出的块经既有读取器进了 `Model::rotational_torques`；
该族的力偶在同一次求解里改变了受力体与反力体的末态角速度，且改变了可归因于这个元素的量。

**不能说明**（已知边界，不在本行范围）：`anti_roll.cpp:128` 把驾驶员需求硬编码为 `1.0` 且不读轮胎滑移；
本行只证明「该族经文档可达且影响轨迹」，实时打滑/驾驶员信号那一层归 p2-05 结项前处置。
