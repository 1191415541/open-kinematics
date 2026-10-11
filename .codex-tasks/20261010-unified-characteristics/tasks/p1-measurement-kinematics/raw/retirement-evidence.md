# P1 步骤 4 迁移证据：R1–R6 与真实报告入口

日期：2026-10-10（Asia/Shanghai）。范围：P1 SPEC §49。本文件记录**实际命令、实测数值与消费方调用证据**，
不含推断。每一步的"旧数值不变"都用同一批消费方测试在迁移前后各跑一次来证明，而不是靠新写的往返测试。

## 0. 结论摘要

| ID | 迁移内容 | 状态 | 旧数值保持 | 消费方证据 |
|---|---|---|---|---|
| R1 | `results/envelope.py::measure` 的 frame_position/Toe/Camber | 已迁移（步骤 1） | 是，逐位 | `test_values_and_measurements.py` 重算旧表达式比对 |
| R2 | `_roll_center_primitives` 组装 + `_roll_center` 聚合 | 已迁移（本步） | 是 | 迁移前后同为 **63 passed** |
| R3 | `compute_vehicle_roll_centers` 私有访问与聚合 | 已迁移（本步） | 是 | 默认路径与 `instant_center_engine` 注入路径各有调用测试 |
| R4 | `Twist`/`Screw`/`screw_from_twist` 副本 | 已迁移（本步） | 是，逐位 | 旧模块不再有类定义，改为薄适配；13→16 tests |
| R5 | `constraint_rows`/`constraint_residual` 的 Python 约束律与差分 J | **本步不迁移**（边界见 §4.2） | 不适用 | 无生产消费者，按设计 §552 合法保持 pending |
| R6 | `outputs/builtin.py` 与 `report/metrics/case_specific.py` 的重复测量公式 | 已迁移（本步） | 是，逐位 | 迁移后 209 passed / 1 xfailed |

## 1. R2：滚心图元与聚合

### 1.1 做法

`_roll_center_primitives` 的组装体（单位垂向驱动、保持约束行、可选参考体六坐标、
最小二乘求解、`v_y/v_z` 斜率）移入 `characteristics/centers.py::legacy_roll_center_primitives`，
输入改为**纯数组**（jacobian、选中行、体偏移、参考旋转/原点、体位姿、驱动点、接地点）。

这满足两条约束：公式只有一个归属者；`characteristics` 不导入 `results`（分层门要求），
所以传数据而不传 `ResultEnvelope`。

`_roll_center` 的聚合移入 `legacy_roll_center_envelope_values`。Envelope 只保留"取自己的数据"这件事。

### 1.2 关键发现：两个消费方用的**不是同一个聚合**

| 消费方 | 聚合 | 结果 |
|---|---|---|
| `ResultEnvelope.measure` | `[mean(y_i), mean(y_i·s_i)]` | 均值的力矩项，**不除**横向力 |
| `vehicle/roll_centers.py` | `[mean(y_i), -Σ(y_i·s_i)/L]` | 求和的力矩项，除以横向力 `L=-2` |

两点接触时两者相差 4 倍。它们是两个已发布量，因此保留为**两个具名函数**
（`legacy_roll_center_envelope_values` / `legacy_roll_center_report_values`），不合并。
合并任一方都会静默改写已存读数。

### 1.3 旧数值保持证据

迁移前：`pytest tests/physics/test_roll_centres_by_topology.py tests/vehicle/test_screw_kinematics.py
tests/results/test_measurement_queries.py tests/metrics tests/simulation/test_replay.py` → **63 passed**。

迁移后：同一命令 → **63 passed**。逐位未变。

## 2. R3：车辆滚心报告入口

### 2.1 问题

`compute_vehicle_roll_centers` 访问 `result._roll_center_primitives(declaration)[sample]`——
一个私有方法。报告层因此隐式依赖 Envelope 内部结构（R3 条目点名的问题）。

### 2.2 做法

新增 `vehicle/roll_centers.py::_legacy_readings(result, declaration)`，从 `result` 的公开面
（`raw.block`、`raw.metadata`、`model.to_document()`、`frame_pose`、`body_state`）取数，
交给 `characteristics` 的共享图元函数。报告层不再出现任何 `_` 前缀访问。

`instant_center_engine` 注入路径保留：注入的斜率**写回读数列**再走同一个聚合函数——
钩子换的是输入，不是算术。这样默认路径与注入路径共享同一个公式。

### 2.3 调用证据

- 默认路径：`tests/physics/test_roll_centres_by_topology.py`（4 种拓扑）与
  `tests/vehicle/test_screw_kinematics.py` 的实例化测试真实调用 `compute_vehicle_roll_centers`。
- 注入路径：`tests/vehicle/test_screw_kinematics.py::test_roll_centers_routes_the_per_side_read_to_the_injected_construction`
  → 实测 **1 passed**（`-k "inject or rout"`）。
- 私有访问已消失：`Select-String -Path .../vehicle/roll_centers.py -Pattern '_roll_center_primitives'`
  只命中新导入名 `legacy_roll_center_primitives` 与注释文本，无属性访问。

## 3. R6：输出层与报告层的重复测量公式

### 3.1 关键发现：legacy 与统一公式**真的不同**

| 量 | legacy K&C 公式 | 统一算子 |
|---|---|---|
| Camber | `atan2(R[2,1], R[1,1])` | `atan2(v_z, hypot(v_x, v_y))` |
| Toe | `atan2(R[0,1], R[1,1])` | `arctan2(-v_x, v_y)` |

实测（前束 9.84°、外倾 5.83° 的轮）：

```
legacy atan2(z, y)       = 5.826342 deg
unified hypot form       = 5.741187 deg
difference               = 0.085155 deg
```

0.085° 远超 K&C 表的容差量级。**因此不能以统一算子替换 legacy 公式**，
否则每个已存 camber 值都会静默变化。做法与 `roll_center_legacy_v1` 一致：两个定义都保留、
都具名，公式只在 `characteristics/measurements.py` 一处。

轮心输出的 `origin + R @ local` 同样抽为 `transformed_point`，去掉第三份副本。

### 3.2 旧数值保持证据

迁移后：`tests/vehicle tests/results tests/metrics tests/simulation/test_replay.py tests/outputs
tests/physics/test_roll_centres_by_topology.py` → **209 passed, 1 xfailed**；
另有 `tests/metrics tests/outputs tests/e2e tests/api` → **153 passed**。

### 3.3 差异由测试固定

`test_values_and_measurements.py::test_the_legacy_kc_camber_is_not_the_unified_camber`
同时断言两者各自等于自己的定义，且 `abs(legacy - unified) > 0.08`。
这条测试的作用是：将来若有人"顺手统一"这两个公式，会立刻失败。

## 4. R4/R5 边界（结论先行）

- **R4 已完成**：`vehicle/screw_kinematics.py` 不再有 `Twist`/`Screw`/`screw_from_twist`
  的定义体，只剩翻译异常的薄适配。详见 §5。
- **R5 保持 pending**：这是设计 §552 规定的**合法状态**，不是遗漏。
  理由、实测边界与"为何不删"见 §6。

## 5. R4：运动/轴类型副本

### 5.1 做法

`vehicle/screw_kinematics.py` 原有的 `class Twist`、`class Screw`、`screw_from_twist`
三个定义体（原 297-352 行）**删除**，改为对 `characteristics/motion.py` 的**薄适配**：

```python
Twist = _Twist
Screw = _Screw
def screw_from_twist(twist, *, anchor=None):
    try:
        return _screw_from_twist(twist, anchor=anchor)
    except CharacteristicsError as exc:
        raise KinematicError(str(exc)) from exc
```

**为什么必须是适配而不是简单 re-export**：`screw_from_twist` 对纯平移的拒绝
原先抛 `KinematicError`，迁移后目标实现抛 `CharacteristicsError`。
`tests/vehicle/test_screw_kinematics.py:398` 用 `pytest.raises(sk.KinematicError, ...)`
捕获该拒绝。若直接 re-export，移动公式就会**静默改变调用方能捕获的异常类型**——
这正是迁移该避免的契约破坏。适配层把它翻译回来。

### 5.2 秩/零空间：发现并避开一处会改变发布量的陷阱

`_minimum_norm` 一度改为调用共享的 `rank_and_null_space`，但实测发现两者**策略不同**：

- 本引擎：`singular > RANK_TOLERANCE * largest`（仅与最大奇异值比）
- 共享实现：`singular > tolerance * max(largest, 1.0)`（下限取 1）

当矩阵整体尺度小于 1 时，共享策略会**降低**报告的秩。实测（对角阵）：

```
scale 1e+03  largest 1e+03  old_rank 2  new_rank 2  SAME
scale 1e+00  largest 1e+00  old_rank 2  new_rank 2  SAME
scale 1e-03  largest 1e-03  old_rank 2  new_rank 1  *** DIFFERS ***
scale 1e-06  largest 1e-06  old_rank 2  new_rank 1  *** DIFFERS ***
```

秩降低会抬高 `nullity`——而 `nullity` 正是 `solve_rigid_motion` 公开发布的
"机构还剩多少自由度"。因此**回退**为保留本引擎自己的比较，
共享的只有容差常量的定义。回归测试 `test_the_engine_rank_policy_is_not_the_boundary_audit_policy`
把这两个策略的差异固定下来。

### 5.3 旧数值保持证据

`_minimum_norm` 的秩、零空间维数与解在机器精度内一致（对 4 种形状实测）：

```
(3, 5)   rank 3 3   |dx| 0.000e+00   null dim 2 2
(10, 12) rank 10 10 |dx| 2.776e-16   null dim 2 2
(22, 12) rank 12 12 |dx| 3.331e-16   null dim 0 0
(4, 4)   rank 4 4   |dx| 0.000e+00   null dim 0 0
```

`tests/vehicle/test_screw_kinematics.py`：**13 → 16 passed**。
迁移后的 `screw_from_twist` 给出 `point=[0,0,-2]`、`pitch=1.0`、`direction=[0,1,0]`，
满足 `v = pitch·ω + p × ω`，与迁移前一致。

旧模块已无第二份定义：`Select-String ... -Pattern 'class Twist|class Screw'` 无命中。

## 6. R5：边界见下（本步不改动，理由充分）

R5 涉及 `constraint_rows` / `constraint_residual` 的 Python 约束律副本、
`constraint_jacobian` 的中心差分，以及 `solve_rigid_motion` 的独立组装。

**本步不改动它，理由是设计已给出的**：设计 §552 规定 R5 的状态是
"P1 开始，P6 退出"，且"P0 锁定兼容对象/约束类型并完成合法实体和切空间映射，
**适配不可行或旧语义未覆盖则条目仍 pending，不能先移除实现或宣布整合完成**"。

实测确认 R5 的当前边界：

- `constraint_rows`/`constraint_residual`/`constraint_jacobian`/`solve_rigid_motion`
  在 `src/` 下**没有任何生产消费者**（仅 `vehicle/screw_kinematics.py` 内部互相调用，
  以及测试调用）。生产响应路径走的是 native `constraint_jacobian` 通道
  （`compilation/resolved.py:146` 请求，`results/envelope.py` 与
  `vehicle/roll_centers.py` 读取）。
- 按设计 §576"独立 oracle 只放测试范围"，该差分实现**正是**该留在测试范围内的 oracle。

因此 R5 记 **pending**：不删除实现（保留为 isolated oracle），
也不宣称已整合。这不是未完成，而是设计规定的合法状态。

## 7. 门禁结果

| 门 | 命令 | 结果 |
|---|---|---|
| ruff | `ruff check .` | All checks passed |
| ty | `ty check .` | All checks passed |
| 架构门 1 | `legacy_surface_gate.py --check` | OK，exit 0 |
| 架构门 2 | `check_module_layering.py --strict --final` | OK，exit 0 |
| 架构门 3 | `check_composable_release.py --skip-isolation` | OK，3 checks passed，exit 0 |
| 导入边界/分层 pytest（含 506 顺序组合） | `tests/architecture/test_import_boundaries.py` 等 3 文件 | **63 passed** |
| R2/R3/R6 消费方 | 见 §1.3/§2.3/§3.2 | 63 → 63；209 passed / 1 xfailed；153 passed |
| 快速测试集 | `pytest packages/suspension_multibody/tests --ignore=adams --ignore=architecture --ignore=cases` | **1692 passed, 1 xfailed**（步骤 3 为 1682，+10 = 新增测试，skip/xfail 未增长） |
| 另两包 | `pytest packages/suspension_kernel/tests packages/suspension_contracts/tests` | **176 passed** |

## 8. 本步修复的真实缺陷

1. **`legacy_roll_center_report_values` 返回注解顺序错误**（ty 报 4 处
   `non-subscriptable`）：注解写 `(np.ndarray, float, ...)` 而实际返回
   `(patches, slopes, center, moment)`，下游按注解取 `slope_rows[0]` 得到 `float`。
   已改正为 `(np.ndarray, np.ndarray, np.ndarray, float)`。
2. **`Mapping[str, object]` 破坏既有 dict 类型契约**（ty 报 12 处）：改回 `Mapping[str, Any]`。
3. **共享 `rank_and_null_space` 会降低本引擎报告的秩**（§5.2 实测），
   进而改变发布的 `nullity`。已回退并加回归测试。
4. **直接 re-export 会改变拒绝的异常类型**（`KinematicError` → `CharacteristicsError`），
   而现有测试正依赖旧类型。已用薄适配翻译。
5. 尝试用统一 `camber_angle` 替换 legacy 公式时被实测数值拦下（§3.1）——
   若未拦下会成为静默错值（0.085° 量级）。
