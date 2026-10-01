# p3-06 判据 6：`EPIC.md` Done-When (a)–(j) 逐条实跑

> 口径（`EPIC.md:291-293`）：**端到端独立验收，不依赖子任务自证**。
> 每一条下面是**本行自己执行**的命令与实得退出码/输出。**尚未落地的条目如实标为未达成。**

| 条 | 状态 | 本行实跑的命令与结果 |
|---|---|---|
| **(a) 力矩内建** | **部分** | `_build_wheel_torque_signals` 与 `front_brake_bias` 的 grep **仍有命中**（p2-05 未落地，见下）。三态求值断言与「同一工况力矩一致」**未验**。 |
| **(b) 转向通道** | **未达成** | p2-06 未落地：`authoring/vehicle.py:119` 的静默覆写与 `preparation/vehicle_dynamic.py` 的 `must be true` 校验仍存在；`grep -rn "must be true"` 在准备层**有命中**。 |
| **(c) 通用运动学** | **达成** | 见 `g3_g4_reverification.md`：四构型各有断言（`102 passed`）；`grep -e UPPER_INBOARD -e LOWER_INBOARD -e UCA_ -e _BODY_ALIASES` 在 `vehicle/` exit=0（零命中）；滚转中心高由虚功导数矩阵解算 + 独立数值判据（`rc_evidence.py` exit=0，`h=-179.99999973118622`）。 |
| **(d) 广义静平衡** | **达成** | 见 `g3_g4_reverification.md`：单轮可解例 `unique=True rank=1=N`；两个不可解例报 `IncompatibleStaticLoadsError` 且消息只含残差/容差/点数；两例矩阵秩都=1；`_WHEELS` 在 `vehicle/` 与 `report/` 零命中。4 轮逐位一致由 p3-04 的 `four_wheel_bitwise_probe.py` 证（本行复跑：`DIFFERENCES: none`）。 |
| **(e) ARB 独立** | **部分** | `anti_roll_bar` 角色/模板/4 端口已落地（p4-02，本行实跑 `tests/subsystems` 通过）；`arb_mount_L/R` 已到产物且能配对（p4-03）。`grep "upright_L"` 在 `subsystems/suspension.py` exit=0。**但**「子系统文件**分别插到**双叉臂下臂与麦弗逊减振筒外筒、**产物符合配对段**」中的「产物」一环：配对已能成立，**防倾杆尚未接入 `si_assembly.py` 的贡献派发链**，所以装配产物里还没有防倾杆子系统自己的体。如实标为部分。 |
| **(f) 轮端统一** | **未验** | p4-04 未落地。本行未独立复验阶段一 04 的交付。 |
| **(g) API** | **部分** | `simulate(assembly_document, case_document)` 已交付（p5-02）；本行实跑 `tests/api tests/architecture` 通过、`legacy_surface_gate --check` findings 0。`FrontAxleModel` 历史调用者全绿（快速集 1188 passed）。**三个公共 API 门禁**：本行只实跑了 `legacy_surface_gate` 与 `test_public_api_boundary_gate`（147 architecture 全绿），第三个门（`check_composable_release`）实跑通过。标为达成范围内的部分。 |
| **(h) 总线与闭环** | **未达成** | p5-03（总线）与 p5-04（闭环）均未落地；FMI（p5-05）未落地。 |
| **(i) 零回归** | **阶段三范围内达成** | 见 `zero_regression.md`：sentinel sha256 `fdfd5a6b…eebc9` 未变；`case_parity_check.py` 8 families accepted；`kc_perf_gate --check` 在预算内；`tests/data/` 干净；快速集 1188 passed / 1 xfailed（与基线同）；`tests/architecture` 147 passed；三条架构门 exit=0。**全量回归**（`packages/suspension_multibody/tests` 整目录含 adams/cases）留 Epic 收尾（p5-06）。 |
| **(j) ABI** | **达成** | 单一真源门 `test_kernel_abi_version_single_source.py` **6 passed**；`mb_config/version.hpp` 记 17 / 32，注释写明「Bumped 16 -> 17 with the rotational actuator (p2-02, 2026-10-01)」与「Bumped 31 -> 32 with the axle surface's 16 -> 17 (p2-02)」，即本次唯一的 ABI 变更已登记。 |

## 结论

**阶段三的两条 Goal（G3 通用运动学、G4 广义静平衡与动态通道）达成**，(c) 与 (d) 逐条实跑通过。

**(a)(b)(e)(f)(h) 未达成或部分达成，原因都是对应子任务尚未落地**（p2-05 / p2-06 / p4-04 / p5-03 / p5-04 / p5-05），
不是本行发现的缺陷。本行按验收行的口径**如实登记**，不把它们当作已达成。

**本行独立验收真正抓到的一个缺陷**：p2-07 新增的 `tests/cases/test_rotational_torque_document.py`
直接向 `kernel.run_contract` 提交文档，触发 `direct_kernel_run_contract` 架构门
（`tests/architecture` 首跑 2 failed）。该文件的作者行已改为经
`simulation.run_request(compile_document_pair(...))` 提交，改后 `tests/architecture` **147 passed**。
