# 旧整轴动力学实现审查与删除清单

审查日期：2026-08-19
结论：旧整轴动态路径不能作为新求解器基础，必须从公共调用链移除并删除不可复现实现。准静态 K/C 路径不在本清单删除范围。

## 公共调用链

- [api.py:130-148](../src/suspension_multibody/api.py) 将 `mode="axle_dynamic"` 在非 `quasi_static` 情况固定路由到 `AxleDynamicIntegratorSolver`。
- [analysis/axle_dynamic.py:278-297](../src/suspension_multibody/analysis/axle_dynamic.py) 再调用旧 `ConstrainedDynamicIntegrator`。
- `CAxleIntegrator` 未接入公共 API；仅由性能测试直接实例化，不能代表产品默认路径。

## 逐文件结论

### `analysis/axle_dynamic.py`

- `AxleCoordinateDriveElement` 将轮端/齿条位移信号转换为高刚度弹簧和阻尼力，而不是时变几何约束；驱动目标按全局绝对坐标计算。已有测试以约 `150 mm` 的世界坐标点配合 `0 -> 10` 输入验证这一语义（旧测试 `tests/analysis/test_axle_dynamic.py:89-97`）。
- 高刚度柔性驱动会改变真实系统的固有频率、能量和约束反力，不能用于与 Adams 的等约束模型对标。
- `AxleDynamicIntegratorSolver` 强制 `max_corrector_iterations` 至多为 1 并打开 `reuse_constraint_linearization`（旧实现约 `:211-217`），不具备强非线性系统的 Newton 收敛保证。
- 处理：删除整轴动态类和旧 drive element；新 API 只接受 manifest 中显式的几何驱动约束或轮端外载。

### `dynamics/constrained.py`

- 旧实现的约束导数把 `future_jacobian` 直接设为当前 `jacobian`，随后计算差分 `Jdot*v`，该项恒为零（旧实现约 `:659-683`），无法正确处理转动机构的速度二次项。
- 快路径力汇总只覆盖 drive、重力和 ramp，跳过完整 force elements、轮胎、衬套和外载（旧实现约 `:589-631`、`:903-990`）；另一条路径的行为取决于是否传入外力函数。
- 坐标转换快路径使用 `R @ force`，与项目中标准的 `R.T @ force` 约定不一致（旧实现约 `:1040-1045` 对比 `core/spatial.py:355-377`）。
- `_constraint_nullspace()` 在 SVD 后没有返回值，后续 rank 代码位于 return 之后（旧实现约 `:876-880`、`:1047-1051`）。
- 处理：删除旧积分器，而不是修补其中的导数、快路径或 KKT 分支。

### `dynamics/integrator.py`

- 旧积分器将位置投影、速度恢复和内部步长重试组合在一起，无法提供新规格要求的 GGL 增广方程、事件定位、能量账本和物理反力唯一性。
- 处理：文件及依赖它的整轴动态入口整体替换为新 C++ 内核；旧全车动态调用若未迁移必须显式失败。

### `model/_axle_mass.py`

- 以 `upright=0.60`、每根 arm `0.16`、tie rod `0.04` 等经验比例分摊总非簧载质量，且以硬编码 `300/250/150/100` 长度构造惯量（旧实现约 `:22-47`、`:75-121`）。
- 该模型不能由硬点、真实质量、质心和惯量验证，且在 mm/SI 混用时会造成量纲错误。
- 处理：删除 `ensure_axle_mass` 及所有零质量自动补齐；新 schema 缺少刚体质量属性时直接拒绝。

### `c_axle_integrator.py` 与 `csrc/axle_fast.*`

- `csrc/axle_fast.c` 源码在未完成表达式处截断，无法通过语法检查；现有 DLL 不能从仓库源码重建。
- Python 包装器硬编码 Windows DLL 路径并立即加载，未提供跨平台错误处理；C ABI 传递大量裸指针，索引类型与声明不具备跨平台合同。
- wheel 只打包 `src/suspension_multibody`，不包含包外 `csrc/axle_fast.dll`。
- C 路径与 Python 路径物理和数值语义不同：注释为半隐式 Euler，参数传入的 generalized-alpha 系数为零，速度残差固定写成 `0.0`，且未覆盖完整 drive/prismatic/output 写回。
- 处理：删除包装器、C 源、DLL、性能测试和所有引用；新 C++ 内核采用 CMake 生成、明确 C ABI、wheel 内置平台共享库。

### `core/fast_ball_system.py`

- 将 Ball/PointCoincidence 约束行重排到前面，导致乘子和诊断行号与输入约束顺序不一致。
- 该模块只优化旧积分器局部热点，不能提供完整约束、接触、能量和反力语义。
- 处理：删除，或仅在新 C++ 内核内部使用经过独立验证的结构化约束装配；不得保留旧 Python 快路径。

### `adams/strict_dynamic_axle.py`

- 旧路径生成 `.adm` 和本求解器结果，但没有真实 Adams 运行证据时只能做离线自一致性检查，不能支持 5% 结论。
- 其 manifest/响应通道基于旧动力学输出，不能直接继承到新物理模型。
- 处理：删除旧动态等效模块；按新 `ACCEPTANCE.yaml`、`CHANNELS.yaml` 和 Dynamic Axle Manifest 重建独立 runner、原始日志与比较器。

### 相关测试与文档

- 删除或改写依赖旧动态实现的 `tests/analysis/test_axle_dynamic.py` 动力学部分、`tests/performance/test_axle_dynamic_runtime.py`、`tests/adams/test_strict_dynamic_axle.py`。
- 保留准静态 K/C、Geometry Contract、静态 Adams Strict K/C 测试；新增测试必须验证新 C++ 内核。

## 删除顺序

1. 先加入新 API 的显式“旧动态路径不可用”错误，避免删除中出现静默回退。
2. 删除旧 Python 轴动态、旧 C 包装器/DLL/源码、经验质量派生、快速球铰和旧动态 Adams 适配。
3. 清理 `__init__`、CLI、测试和打包入口中的旧导入。
4. 执行依赖边界、准静态回归和静态检查，确认没有旧动态符号残留。
5. 新 C++ 内核通过解析/物理门后，再恢复 `axle_dynamic` 公共命令并指向新 runner。

## 不得采用的“修复”

- 调大弹簧或阻尼增益以追踪输入。
- 用结果缩放、经验轮荷修正、输出插值、回归拟合或 Adams 参考表修正。
- 放宽约束/速度残差、跳过拒步、把失败样本标记为收敛。
- 用预编译且无法从源码重建的 DLL 作为发布内核。
