# 05 步骤 4：静轮荷归属收口与保留登记（A2 修订：不迁 native）

按 2026-09-22 用户裁决 A2，本步由「迁入 native」改为「按 A1 同类偏差登记」。本文件是该偏差的正式登记，含 file:line、阻断原因与解除条件。

## 1. 保留对象（file:line）

| 项目 | 位置 | 说明 |
|---|---|---|
| 求解实现 | `analysis/vehicle_physics.py:88-142` | `compute_static_wheel_loads`；3 行平衡方程 × 4 未知支反力，`np.linalg.lstsq(matrix, rhs, rcond=1e-12)`（:131），返回最小范数解 |
| 输入装配 | `analysis/vehicle_physics.py:110` | `build_vehicle(vehicle, mode="K")` —— **K 模式**装配，与动态整车算例（`preparation/vehicle_dynamic.py`）不是同一套装配 |
| 结果面字段 | `results/vehicle.py:32`（`VehicleDynamicsResult.static_wheel_loads`）、`results/vehicle.py:71` 与 `vehicle/service.py:71,91`（失败/成功两路回填） | 归属收口位置：结果类型持有该字段 |
| 唯一生产调用者 | `vehicle/service.py:30`（import）、`:40`（调用） | `static_wheel_loads = compute_static_wheel_loads(model).wheel_loads`；`except (ValueError, np.linalg.LinAlgError)` 降级为 `None`（:41-42） |
| 包级再导出 | `analysis/__init__.py:27,43` | 公开能力，未删除 |

**归属收口结论**：算法本体与 service 调用语义**保持原样**（A2 明确要求），归属在 `results` 结果面收口——`VehicleDynamicsResult.static_wheel_loads` 是该事实的唯一对外载体，调用者只取 `.wheel_loads` 后写入结果对象。本步**不移动文件**：A2 的口径是「保留 + 登记」，不是「迁移」。迁往 `results/` 会引入 `results → model` 反向依赖边（该函数需要 `VehicleModel` 与 `build_vehicle`），与 G4 的「report 不调 preparation、结果层只解码事实」边界冲突，属为迁而迁。

## 2. 阻断原因（为何不能迁 native）

1. **无 ABI 入口**：`packages/suspension_kernel/cpp/src/abi/kernel_abi.cpp:854-858` 原文——「The flat entry points are retained only as internal C++ functions while their body is shared with historical diagnostics. They intentionally have no C linkage, so the shared library exposes exactly one run entry point: `suspension_kernel_run` in `kernel_contract_run.cpp`.」
2. **无等价 C++ 原语**：`cpp/include/mb_solve_static/functions.hpp:61` 的 `solve_static_least_squares` 是**方形**方程求解——实现 `cpp/src/solve_static/kernel_static_projection.cpp:212-213` 强制 `matrix.size() == dimension*dimension`，不是 3×4 最小范数；且声明在 `namespace axle_kernel` 内，无 `extern "C"`。
3. **导出面冻结**：EPIC「ABI 签名及现有导出不变」+ 01 冻结的七符号门（`VALIDATION.md` §4）。迁入 native 必然要求新增导出或新增契约输入路径。
4. **装配不互通**：输入来自 `build_vehicle(mode="K")`，而唯一 run 入口处理的是 case family 文档（`kc_quasi_static` / `vehicle_dynamic`）；把它接进 native 需要新增一条「K 模式静力反力」原生调用路径，属数小时级 C++ 工作，且不消除架构偏差——这是 3×4 线性代数，不是多体求解。

## 3. 数值门影响（可判定性证据）

静轮荷**不参与**两个字节级门：

- `scripts/case_parity_check.py:413-421` 的 `_VEHICLE_LEDGERS = ("states", "constraint_wrench", "spring_output", "bushing_output", "anti_roll_output", "tire_output", "energy")` —— 不含 `static_wheel_loads`。
- `scripts/dynamic_hash_sentinel.py` 全文 0 处 `static_wheel_loads`；比较对象是 axle-dynamics acceptance 的 26 个 artifact（`arrays.npz` 字节）。

因此本步的保留决定**不会**使任一字节级门失败，也不需要重录任何基线。

## 4. 保留期测试（证明算法被保留，非迁移证据）

新增 `packages/suspension_multibody/tests/physics/test_vehicle_physics.py:66-86` 的 `test_static_wheel_loads_are_the_minimum_norm_solution`：断言四个支反力等于 `total` 的四等分（`rtol=1e-9`）。

该断言是**真判别器**，实测证据（本步实测，非推断）：

```text
A = [[1, 1, 1, 1],
     [1400, 1400, -1400, -1400],
     [-750, 750, -750, 750]]
rank(A) = 3   （4 未知量，零空间维数 1）
loads  = [3580650, 3580650, 3580650, 3580650]，2-范数 = 7161300
零空间方向 [1,-1,-1,1]：A·[1,-1,-1,1] = [0,0,0]（精确平衡，残差 0.0）
加上 1000·[1,-1,-1,1] 后 2-范数 = 7161300.279 > 7161300
```

即：平衡方程只约束到 3 维，族内**均匀解**才具有最小 2-范数；任何返回族内其它平衡解（或单纯最小二乘拟合）的实现都满足原余额断言却在此失败。这固定了 A2 决定保留的算法选择。

原有覆盖保持不变且继续通过：`tests/physics/test_vehicle_physics.py:11,27,40`（余额与纵横载荷转移）、`tests/vehicle/test_service_contract.py:406`（service 集成到 artifact）。

## 5. 解除条件

先单独裁决下列任一事项，在此之前不得以「已登记」代替交付，也不得判本步为「已迁 native」：

1. 是否允许**扩展 ABI 导出面**（新增静力求解专用导出，需同时解除 01 的七符号冻结）；或
2. 是否允许**在既有 `suspension_kernel_run` 契约下新增默认关闭的静力反力输出块**（须保持默认路径 artifact 字节不变、ABI 七符号与版本常量不变）。

解除后本偏差按新裁决实施并重新验收。

## 6. 未做（明确记录，不伪造达成）

- 未新增任何 C++ 导出符号或契约字段。
- 未修改 `analysis/vehicle_physics.py` 的算法与 `vehicle/service.py` 的调用语义。
- 未删除、未移动、未转发封装该函数。
