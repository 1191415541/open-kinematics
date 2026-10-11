# 规划源码证据索引

日期：2026-10-10。仅说明当前落点，不代表新增能力实现；实施前重查符号/行号。主线程完整读取原设计。

| 事实 | 当前证据 |
|---|---|
| 封闭schema/旧outputs | packages/suspension_contracts/src/suspension_contracts/contracts/solve_plan.schema.json:4、multibody_case.schema.json:158、multibody_result.schema.json:6 |
| plan/编译扩展 | packages/suspension_multibody/src/suspension_multibody/compilation/resolved.py:25 plan_from_case、:114 compile_resolved；modeling/resolved.py:115 |
| native capability与输出 | packages/suspension_kernel/cpp/src/abi/kernel_contract_run.cpp:199 capability_document、:621输出开关、:1125结果 |
| 静态切线/动态Newton/接受态 | packages/suspension_kernel/cpp/src/solve_static/kernel_static_trim.cpp:357；src/solve_dynamic/kernel_integrator_newton.cpp:785；src/abi/kernel_abi.cpp:431/:582/:676 state提交、:794输出（后两路径同cpp根） |
| 单RHS分解已存在，多RHS/转置需实施 | packages/suspension_kernel/cpp/include/mb_linear/factorization_types.hpp各分解solve(vector)；未见统一多RHS/transpose，P0逐后端再审 |
| 当前JSON/NPZ写入，表格读取存在 | packages/suspension_multibody/src/suspension_multibody/io/artifacts.py:81分派、:121CSV/Parquet读取、:315 _write_multibody_result仅arrays.npz/native_result.json；主线程点验 |
| pyarrow已依赖/csv标准库 | packages/suspension_multibody/pyproject.toml dependencies pyarrow>=21.0.0 |
| 滚心报告真实default/injection | vehicle/roll_centers.py:25/:40；tests/physics/test_roll_centres_by_topology.py；tests/vehicle/test_screw_kinematics.py:427（均multibody包） |
| 旧screw对象与约束映射 | vehicle/screw_kinematics.py:180 constraint_rows；Twist/Screw/TangentDrive/PointDrive/RigidMotion/solve_rigid_motion；对应tests/vehicle/test_screw_kinematics.py含DistanceConstraint拒绝与纯平移 |
| 旧测量/metrics/replay | multibody tests/results/test_measurement_queries.py、tests/metrics/test_outputs_match_legacy.py、tests/metrics/test_metrics.py、tests/simulation/test_replay.py |
| 当前门/环境 | AGENTS.md、justfile、pyproject.toml；check-fast实际另含test_no_legacy_vehicle_models.py；Python>=3.12/uv/pytest/CMake+Ninja |
| 目录被忽略 | git check-ignore .codex-tasks/20261010-unified-characteristics/EPIC.md命中，不改ignore/不强制暂存 |

探子仅只读检索，没有执行构建/测试。关于“已有写表”的探子初述经主线程点验修正为已有读取和JSON/NPZ写入。未来文件/测试均在SPEC标拟新增。
