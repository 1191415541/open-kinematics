# P1 实施进度

## 恢复入口

- 日期：2026-10-10（Asia/Shanghai）。
- 任务：建立唯一测量/Twist/中心算子与 native 运动学响应，逐准静态样本提供几何值及受控输入偏导；贯通最小只读查询和原生持久化。
- 形态：single-full；状态：DONE。
- 进度：6/6（**全部完成 2026-10-10**）；下一步：无（P1 已关闭，父 SUBTASKS.csv 已回填 DONE）。
- 真源：[TODO.csv](TODO.csv)；范围：[SPEC.md](SPEC.md)；父真源：[SUBTASKS.csv](../../SUBTASKS.csv)。
- 验收：SPEC §53–59 五条全部 `[x]`；总证据 [raw/validation.md](raw/validation.md)。
- 已完成：
  - 步骤 1 [DONE 2026-10-10] `characteristics/{values,measurements}.py`；R1 已落地
    （`results/envelope.py::measure` 的 frame_position/Toe/Camber 公式体删除，改为委托）。
  - 步骤 2 [DONE 2026-10-10] `characteristics/motion.py` 秩/零空间/可辨识性审计；
    native `constraint_jacobian` 通道已贯通（实测 12 坐标 / 10 行 / 零空间 2 维）。
  - 步骤 3 [DONE 2026-10-10] `characteristics/centers.py` + `registry.py`；
    Twist/Screw/主销/Ackermann 算子入 `motion.py`；版本化中心定义按 kind+version 精确匹配。
  - 步骤 4 [DONE 2026-10-10] R2/R3/R4/R6 已迁移，R5 按设计保持 pending。
    证据：[raw/retirement-evidence.md](raw/retirement-evidence.md)。
    - R2 `_roll_center_primitives`/聚合 → `characteristics/centers.py`；
      **关键发现**：Envelope 与车辆报告用的是两个不同聚合（均值 vs 求和后除 L=-2，
      两点接触差 4 倍），故保留两个具名函数，不合并。
    - R3 `compute_vehicle_roll_centers` 去掉对 `result._roll_center_primitives` 的私有访问，
      改从公开面取数；默认与 `instant_center_engine` 注入路径**均有真实调用测试**。
    - R4 旧模块 `Twist`/`Screw`/`screw_from_twist` 定义体删除，改为薄适配
      （翻译 `CharacteristicsError`→`KinematicError`，否则静默改变调用方可见异常）。
    - R6 legacy K&C 与统一算子实测差 0.085°（前束 9.84° 时），
      故保留两个具名定义而非替换。
    - R5 pending 是设计 §552 的合法状态：无生产消费者，按 §576 保留为测试范围 oracle。
  - 步骤 5 [DONE 2026-10-10] `results/characteristics.py` 查询层 +
    `ResultEnvelope.characteristic()` 委托；`tests/characteristics/test_query_artifact.py` 20 用例。
    证据：[raw/query-artifact-evidence.md](raw/query-artifact-evidence.md)。
    - 样本身份按 `(case_index, output_sample_id, time_s)`，不用数组下标。
    - 未请求的通道报**查询错误并列出可用通道**，绝不零填充（读请求≠能应答，P0 已拆成两个能力）。
    - 冷读不重解以两道锁证明：`run_compiled` 换为抛错函数 + 计数器断言为 0；
      另把 `result.model` 换成访问即抛错的对象后应答仍可读。
    - **关键发现**：`roll_center` 是 `(samples,2)` 向量量，首轮 `ravel` 会把它压成一维、
      谎称两分量共用一个单位且让每样本原因无处存放 → 改用 `VectorValue`，
      并给 `VectorValue` 补 `valid_mask`/`all_valid`（此前与 `MeasurementValue` 不对称）。
    - **关键发现**：工件 `arrays.npz` 存的是 **native 通道**（`body_state`/`constraint_jacobian`/…），
      `manifest["tables"]` 为空；可查询性来自随行的 `model` 文档而非冻结的派生数组。
      据此把往返测试改为「重建 envelope 后重答同一测量并逐值比对」，而非从 npz 取 `roll` 数组。
    - 删除了一个同义反复守卫（key 与 `measurement_id` 不一致永不触发，id 由 key 派生），
      改为测真实的「响应长度与请求样本数不一致」拒绝。
  - 步骤 6 [DONE 2026-10-10] 运行专项及仓库门并归档。
    证据：[raw/validation.md](raw/validation.md)（门总表 + SPEC §53–59 逐条对账 + §61–66 场景对账 + 已知缺口）。
    - SPEC §66 五个专项文件 52 passed；characteristics 158 passed；快速集 1715 passed/1 xfailed。
    - gate-numeric 三条全通过（sentinel 逐位一致、parity 8 族、perf 在预算内），未重录任何冻结基线。
    - 本步只跑门与归档，除状态回填外未改动 `src`/`tests` 代码。
- 启动依赖：P0 已 DONE。
- 下一步：无。P1 已关闭；后续 P2/P3/P4a 为新的子任务。
- 最新实施验证：快速集 **1715 passed / 1 xfailed**（skip 0，未增长；较步骤 4 的 1692 增 23，
  即本步新增测试）；characteristics+results+vehicle 273 passed/1 xfailed；
  另两包 176 passed；ruff/ty/三架构门全 0。
- 交付后独立只读复审（D2）：发现并修复 2 个真实缺陷（见下 ⑪⑫）。
- 已修复的真实缺陷（均为"测试全绿但数值错"型）：
  ①力等效中心驱动轴取反（roll 用 v_x 而非 v_y），零驱动列静默返回零；
  ②Ackermann 理想关系符号写反，往返测试因两侧复用同一错公式而掩盖；
  ③kingpin 竖轴被误判为"无投影"而拒绝（应报 0 度）；
  ④审计把冗余行误判 singular，导致 22×12 良态边界被拒；
  ⑤请求在 case 重建处丢失、循环导入（P0 阶段）；
  ⑥共享 `rank_and_null_space` 的 `max(largest,1.0)` 下限会在小尺度矩阵上**降低**报告秩，
  进而抬高 `solve_rigid_motion` 发布的 `nullity`，已回退并加回归测试；
  ⑦`legacy_roll_center_report_values` 返回注解顺序写错（ty 4 处）；
  ⑧`Mapping[str, object]` 破坏既有 dict 类型契约（ty 12 处）；
  ⑨直连 re-export 会改变拒绝的异常类型（R4）；
  ⑩`roll_center` 向量量被 `ravel` 压平，谎称两分量共用一个单位（步骤 5）；
  ⑪`VectorValue` 缺 `valid_mask`，与 `MeasurementValue` 不对称（步骤 5）；
  ⑫多 case 结果的时间身份按 flat offset 索引会配错时间、越界时静默留 `None`
  （`None` 是合法值，故把结构错误伪装成「合法无时刻」）——复审发现（步骤 5）；
  ⑬`envelope_from_artifact` 无条件 `dtype=float` 强转，会破坏将来的非 float 块——复审发现（步骤 5）。

## 证据与同步规则

实施在raw/保存实际命令/退出码、oracle/误差、预算测量/调用迁移证据。未通过不DONE；同类3次失败暂停重评，保留FAILED/retry_count。每步同步本恢复块；子任务完成后回填父SUBTASKS，completed_at记实际时间。
