# 步骤 1：mb_base 逐函数归属表（冻结）

来源：`cpp/include/mb_base/`、`cpp/src/base/` 全量阅读。归属判定依据 Epic 迁移矩阵与 02 冻结的 MIGRATION_MAP。

## 头文件归属（11 个 → 三模块）

| 现路径 mb_base/ | 目标模块 | 依据 |
|---|---|---|
| prelude.hpp | mb_config | 纯标准库聚合、非层ing装置；与 env 同级（CONFIG 定义运行环境） |
| version.hpp | mb_config | 版本字面量单一来源，值 15/30/1 保持不变 |
| env.hpp | mb_config | 运行时开关声明（MB_BASE 注释自述 solver environment switches） |
| diagnostics.hpp | mb_config | 计时与 profiling 计数器（运行时诊断） |
| constants.hpp | mb_config | 容差/宽度/限制常量——注意：宽度假结构化，按 Epic "env、版本、运行时诊断配置" 归 config |
| vector.hpp | mb_numeric | Vec3/Mat3/Quat 依赖无关数值类型 |
| util.hpp | mb_numeric | finite_vec/max_abs 全向量工具（注释自述依赖无关） |
| monotone_cubic.hpp | mb_numeric | CurveValue 返回类型；普通曲线插值 |
| dual.hpp | mb_dual | DirectionalScalar 对偶标量 |
| dual_geometry.hpp | mb_dual | DVec3/DMat3/DQuat 对偶几何 |
| functions.hpp | 拆分 | 聚合声明头：按每条声明所属 TU 拆入各模块 functions.hpp（见下） |

## kernel_base.cpp 函数归属（724 行 → numeric/config 拆分，dual 部分实际在 kernel_dual_algebra.cpp / dual_geometry.cpp）

### → mb_config（env/诊断开关）
- profiling_enabled, runtime_jacobian_validation_enabled, static_debug_enabled（L27-40）
- linearization_reuse_limit, newton_jacobian_refresh_period, linear_solver_threads（L42-79）
- blocked_lu_enabled, blocked_lu_parallel_enabled, lu_equilibration_enabled（L81-94）
- sparse_gmres_enabled, sparse_gmres_restart, sparse_gmres_max_iterations, sparse_lu_enabled（L96-122）
- mkl_dense_enabled, mkl_pardiso_enabled, mkl_pardiso_full_enabled, mkl_pardiso_debug_enabled, mkl_pardiso_matching_enabled, mkl_pardiso_pivot_perturbation, mkl_pardiso_ordering, mkl_pardiso_threads, blocked_lu_block_size（L124-196）
- exact_fiala_relaxation_enabled, light_fiala_relaxation_enabled, acceleration_predictor_enabled, acceleration_schur_probe_enabled（L198-221）
- finite_vec, max_abs（L12-25）→ 按 util.hpp 注释属依赖无关数值工具：**mb_numeric**（更正：util.hpp 目标随函数归 numeric）

### → mb_numeric（向量/矩阵/四元数/旋转/普通曲线）
- dot, cross, norm, normalized（L223-234）
- determinant, inverse3, finite_symmetric, symmetric_positive_definite（L236-320）
- transpose, Mat3 operator*, operator+, operator-, identity3, Mat3*Vec3, skew, outer, row_times（L322-386）
- qmul, qconj, qnorm, qdot, qnegated, qnormalize, normalized_continuous, unit_quaternion, qmat, qexp, qlog, rotate（L388-468）
- interpolate_curve, akima_curve_slopes, interpolate_akima_curve, integrate_akima_segment, integrate_akima_curve_from_zero, integrate_curve_interval, integrate_curve_from_zero（L470-634）
- cardan_xyz_from_rotation, cardan_xyz_rate（L636-667）
- monotone_cubic（L676-722）→ Fritsch-Carlson 单调三次；普通曲线插值：mb_numeric
- log_left_jacobian_inverse（angles.cpp L13）→ SO(3) 左雅可比逆：mb_numeric

### → mb_dual
- DirectionalScalar operator+,-,unary-,*,/ 及全部 d_* 标量助手（kernel_dual_algebra.cpp，160 行）
- DVec3/DMat3/DQuat 全部运算符与 d_* 几何/四元数函数、d_cardan_*、d_body_quaternion、d_qlog、d_log_left_jacobian_inverse、d_rotate、so3_left_jacobian、d_identity3（dual_geometry.cpp，323 行）
- interpolated_curve_directional（curves.cpp L14）→ 普通曲线的对偶变体：**mb_dual**（对偶曲线归 dual）
- finite_vec/max_abs 的注释虽提 linalg/integrator 调用者，但其为依赖无关工具 → mb_numeric（见上）

## 常量归属修正（constants.hpp）

`kPac2002CamberLimit` 等轮胎宽度常量混在 constants.hpp。SPEC 范围限 mb_base 三拆（numeric/dual/config）不动 mb_tire；本任务将 constants.hpp 整体归 mb_config（容差/限制/宽度常量=运行配置常量），不改名任何符号。04 处理元件语义时再评估拆出。

## 版本单一来源门

- `mb_config/version.hpp` 保持 ABI 版本 15/30/1 唯一定义；`abi/version.hpp` 与 `mb_model` 的引用改为 include mb_config/version.hpp（01 冻结值不变）。
- 测试 `test_kernel_abi_version_single_source.py:25` 的 VERSION_HEADER 路径同步改为 `mb_config/version.hpp`。

## 调用者闭合

mb_base 被 75 处 include（40 functions、35 prelude、23 vector、21 constants、20 dual、17 util/env/dual_geometry/diagnostics、16 monotone_cubic、3 version）。全部按目标模块机械改写 include 路径；无符号改名、无函数移动逻辑变化。

## 依赖方向约束（目标）

- mb_numeric：不依赖任何模块（只依赖标准库）。
- mb_dual → mb_numeric（DirectionalScalar 组合普通几何语义时用 Vec3/Mat3/Quat）。
- mb_config：不依赖任何模块。
- 无 numeric→dual 边（DAG）。

## 后续模块重命名映射（步骤 5-6）

- mb_linalg → mb_linear：src/linalg/kernel_linalg.cpp → src/linear/，include/mb_linalg/{functions,factorization_types}.hpp → include/mb_linear/。
- mb_constraint → mb_joint：src/constraint/{kernel_directional_constraint,kernel_model_constraint,registry}.cpp → src/joint/，include/mb_constraint/{functions,registry,types}.hpp → include/mb_joint/。
- mb_integrator → mb_solve_dynamic：src/integrator/*.cpp(6) → src/solve_dynamic/，include/mb_integrator/{context,functions}.hpp → include/mb_solve_dynamic/。
- mb_static → mb_solve_static：src/static/*.cpp(3) → src/solve_static/，include/mb_static/functions.hpp → include/mb_solve_static/。
- 共享输入采样 → mb_input：kernel_integrator_input.cpp 的 interpolate_input/next_prescribed_input_breakpoint 移到 src/input/（SampleInput 结构已在 mb_model/types.hpp，不移）；kernel_integrator_input.cpp 剩余 NewtonLinearizationCache/pose_candidate/state_from_unknown 归 mb_solve_dynamic。static/contact.cpp、output、abi 对 interpolate_input 的调用改 include mb_input。
- mb_solve_static 不再 include mb_solve_dynamic 的 functions.hpp（去静态→动态实现依赖；contact.cpp 只需 mb_input 的 interpolate_input）。

## 四份清单同步点

CMakeLists.txt（MB_BASE_SOURCES 等目标名）、MODULES.md 模块表、layering_baseline.json（legacy_modules 收缩、边集重扫）、测试硬编码（test_module_layering_gate.py、test_kernel_abi_version_single_source.py）。
