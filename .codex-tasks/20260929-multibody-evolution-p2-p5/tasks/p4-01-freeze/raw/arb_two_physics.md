# ARB 两套物理对照（p4-01 冻结证据 · F11）

仓库根：`/e/杂件/open-kinematics`；HEAD `8d8c5c0`（2026-10-01）。所有原文由本行开工时实测（`sed`/`grep -n`）。
两套物理**不得等价化**（`EPIC.md:137`、`EPIC.md:317`）。

## A 侧：Python `AntiRollBarElement`（力元）

- 力律原文：`packages/suspension_multibody/src/suspension_multibody/modeling/primitives/elements.py:592-621`
  - `:596-602` 字段：`name, left_body, left_point, right_body, right_point, stiffness, reference_difference=0.0`
  - `:604-607`：
    ```python
    left = _point(state, self.left_body, self.left_point)
    right = _point(state, self.right_body, self.right_point)
    difference = (right[2] - left[2]) - self.reference_difference
    ```
  - `:608-610`：`scalar = self.stiffness * difference`；`left_force = [0,0,scalar]`；`right_force = -left_force`
  - `:611-621` 返回 `ForceEvaluation`，`:615-616` 把 `left_force` 施在 `left_point`（`_point_wrench`）、`right_force` 施在 `right_point`；`tangent` 2×2 对称刚度阵。
  - **力律**：两端**垂向 (z) 位移差**驱动的一对**等大反向线力**（各施于各自硬点上），等效为力偶；能量 `0.5*k*difference^2`（`:613`）。
- 自由度：两端各自是完全自由的刚体上的**附着点**；元素本身不引入任何扭杆刚体、也无独立旋转自由度——它只是把两个既存刚体按 z 差耦合。
- 装配入口：`subsystems/element_build.py:66-67` 构造分派
  ```python
  if row.kind == "anti_roll_bar":
      return _anti_roll_bar(row, cast(AntiRollBar, row.spec))
  ```
  `element_build.py:124-133 def _anti_roll_bar(...)`：把 `row.body_a/point_a → left_body/left_point`、`row.body_b/point_b → right_body/right_point`、`spec.torsional_stiffness → stiffness`。注意 **不读** `left_body_mount/right_body_mount/left_arm_end/right_arm_end`。
- 声明入口（跨两侧、只发一次）：`subsystems/suspension.py:696-709 global_elements`
  ```python
  body_a="upright_L",
  point_a=context.local("upright_L", spec.left_link_point.as_array()),
  body_b="upright_R",
  point_b=context.local("upright_R", spec.right_link_point.as_array()),
  ```
  （F11 称 `suspension.py:682-695`；实测当前行为 `:696-709`——**锚点轻微漂移**，语义一致。）
- 被 `element_build.element_rows` 装配一次：`subsystems/element_build.py:216` `rows.extend(suspension_subsystem.global_elements(context))`（F11 称 `element_build.py:211` 唯一入口；实测为 `:216`，`:211` 现为注释）。
- schema：`schema/elements.py:224-234 class AntiRollBar` 有 **6 个硬点字段** `left_body_mount/right_body_mount/left_arm_end/right_arm_end/left_link_point/right_link_point` + `torsional_stiffness`；装配期**只读** `left_link_point`/`right_link_point`（构造见上），其余 4 个被忽略。
- 数量可配置且可为空：`schema/model.py:171 anti_roll_bars: tuple[AntiRollBar, ...] = ()`。

## B 侧：内核 native 扭杆（纯力偶）

- ABI 编组声明（输入）：`packages/suspension_kernel/cpp/include/mb_input/types.hpp:578-584`
  ```cpp
  std::size_t anti_roll_bar_count;
  const int* anti_roll_body_a;
  const int* anti_roll_body_b;
  const double* anti_roll_axis_a;
  const double* anti_roll_reference_quaternion;
  const double* anti_roll_stiffness;
  const double* anti_roll_damping;
  ```
  输出通道：`types.hpp:683-685` `anti_roll_output`（每杆 3 列：relative angle, relative rate, torque on body_b）。
- 模型结构：`packages/suspension_kernel/cpp/include/mb_model/types.hpp:142-147`
  ```cpp
  struct AntiRollBar {
      int a{-1}, b{-1};
      Vec3 axis_a{0, 0, 1};
      Quat reference{};
      double stiffness{0.0}, damping{0.0};
  };
  ```
- 力律原文：`packages/suspension_kernel/cpp/src/element/anti_roll.cpp:26-85`（`assemble_anti_roll_forces`）
  - `:43-51`：
    ```cpp
    const Quat qrel = qmul(qconj(state.q[bar.a]), state.q[bar.b]);
    const Vec3 phi  = qlog(qmul(qconj(bar.reference), qrel));
    const Vec3 axis_world = normalized(rotate(state.q[bar.a], bar.axis_a));
    const double angle = dot(phi, bar.axis_a);
    const double rate  = dot(axis_a_world, state.omega[bar.b] - state.omega[bar.a]);
    const double tau   = internal_force_scale * (-bar.stiffness * angle - bar.damping * rate);
    ```
  - `:52-63`：注释明说「The bar applies a pure couple, which has no application point」；`add_torque_on_body(torque, model, bar.b, axis_world * tau, sink)` 与 `add_torque_on_body(torque, model, bar.a, axis_world * (-tau), sink)` —— 在 `body_b`/`body_a` 上各加等大反向**纯力矩**（无施力点坐标）。
  - `:64-76` 能量/耗散：`potential += 0.5*stiffness*angle^2`、`dissipation += damping*rate^2`。
  - `:77-83` 输出：`[angle, rate, tau]`。
- ABI 编组（构造）：`packages/suspension_kernel/cpp/src/assembly/build_model.cpp:333-352`（从 `in.anti_roll_*` 数组读入，校验 `bar.axis_a` 非零、`stiffness/damping>=0`）。
- 契约读取：`packages/suspension_kernel/cpp/src/assembly/element_reader.cpp:333-350`（`ELEMENT_ANTI_ROLL` 分支，读 `ELEMENT_ANTI_ROLL_STIFFNESS/DAMPING/AXIS_A/REFERENCE_QUATERNION`）。
- Python 侧 ABI 载体：`axle_dynamics/schema.py:921-946 class AxleAntiRollBar`（`body_a/body_b/axis_a/reference_quaternion_a_to_b/stiffness_n_m_per_rad/damping_n_m_s_per_rad`，非 6 硬点）。
- Python → 契约元素：`cases/axle_dynamic.py:188-202 _anti_roll_element`（`type="anti_roll_bar"`，参数 `axis_a/reference_quaternion/stiffness/damping`）。
- 消费：`axle_dynamics/contract_run.py:242 anti_roll_bar_names`、`:250 anti_roll_output=ledger("anti_roll_output", len(model.anti_roll_bars), 3)`。

## 两套物理逐条对照表

| 维度 | A：Python `AntiRollBarElement` | B：内核 native 扭杆 | 差异 |
|---|---|---|---|
| 力律表达式 | `scalar = stiffness * ((z_r - z_l) - ref)`，两端等大反向**线力** `[0,0,±scalar]`（`elements.py:607-610`） | `tau = -stiffness*angle - damping*rate`，`angle = dot(phi, axis_a)`（纯力矩，`anti_roll.cpp:47-51`） | A 是**平移位移差→力**；B 是**相对旋转角→力矩**，且 B 含**阻尼**项，A 无 |
| 自由度 | 引用的两个刚体照常各 6 自由度；元素不新增自由度，只在两点施加约束力偶（`elements.py:604-621`） | 同上：不新增自由度；在 body_a/body_b 上施加等大反向**力矩**（`anti_roll.cpp:58/63`），另由 `state.q/omega` 读取相对角 | 结构相同，但读数来源不同：A 读两**点** z 坐标；B 读两**体** 相对四元数/角速度 |
| 施力与反力体 | `left_body`/`right_body`（装配期硬编码 `upright_L`/`upright_R`，`suspension.py:703-706`），力作用在 `left_point`/`right_point` | `bar.a`/`bar.b`，**纯力偶无作用点**（注释 `anti_roll.cpp:52-53`） | A 有明确作用点坐标；B 无作用点 |
| 消费路径 | 多体层 `ForceEvaluation` → 装配/求解（Python 侧力元求值） | native 内核 ABI（`mb_input` 数组 → `build_model` → `assemble_anti_roll_forces` → `anti_roll_output` 3 列） | 两条独立通道 |
| 刚度参数来源 | `schema/elements.py:234 torsional_stiffness`（单标量） | `stiffness` + `damping`（两个标量，`types.hpp:583-584`） | B 多一个阻尼 |
| 硬点 | 6 字段但装配只读 `left_link_point`/`right_link_point`（`schema/elements.py:228-233`；构造 `element_build.py:124-133`） | 无硬点概念，`body_a/body_b + axis_a + reference_quaternion` | A 有 6 硬点的声明面；B 无 |
| 交叉拒绝 | native 侧**显式拒绝** A：`preparation/vehicle_dynamic.py:936-939` `raise ValueError("... anti-roll element ... is a link anti-roll law; the native torsional anti-roll ABI is not equivalent")` | — | 代码本身把两者判为**不等价** |

对照用例：`packages/suspension_multibody/tests/axle_dynamics/test_api.py:315 test_anti_roll_bar_reports_physical_angle_rate_and_torque`，断言在 `:377-382`：
```python
output = result.anti_roll_bar_state("bar")[0]
np.testing.assert_allclose(output[0], angle, ...)        # 相对角
np.testing.assert_allclose(output[1], angular_rate, ...) # 相对角速度
np.testing.assert_allclose(output[2], -stiffness * angle - damping * angular_rate, ...)  # 力矩
```
即该用例只覆盖 **B 侧**（native 扭杆），不涉及 A 侧 `AntiRollBarElement`。

## 结论

- 两套物理在**力律（力 vs 力矩）、读数（点位移 vs 相对角）、阻尼（无 vs 有）、施力点（有 vs 无）**四处不同，且代码在 `vehicle_dynamic.py:936-939` 明确互不承认。
- p4-02 选型时必须引用本表，不得把任一侧输出当作另一侧等价物。
- **锚点漂移记录**（只记录、不改父文件）：`suspension.py:682-695` → 实测 `:696-709`；`element_build.py:211` → 实测入口 `:216`。
